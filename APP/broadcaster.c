/********************************** (C) COPYRIGHT *******************************
 * File Name          : broadcaster.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2020/08/06
 * Description        : 广播应用程序，初始化广播连接参数，然后处于广播态一直广播

 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CONFIG.h"
#include "devinfoservice.h"
#include "broadcaster.h"
#include "app_i2c.h"

// 广播间隔 (units of 625us, min is 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL 1600 * 2 //2s

// 采集间隔
#define SBP_PERIODIC_EVT_PERIOD 1600 * 20  // 20s更新

// opt3001 转换间隔
#define OPT3001_READ_EVENT_PERIOD 1600 * 1 //2s

// opt3001 采集间隔
#define OPT3001_CJ_EVENT_PERIOD 1600 * 30   //40s


#define COUNTER_OFFSET 16
#define BAT_OFFSET 13
#define WINDOWS_OFFSET 19
// ADC 采样粗调偏差值
static signed short RoughCalib_Value = 0;
// 电池电压
static uint16_t bat = 0;

static uint32_t opt3001_lux = 0; // 光照值

uint16_t mag_count = 0; // Task ID for internal task/event processing
uint8_t mag_int_flag = 0; // 磁强计中断标志位
uint8_t lux_int_flag = 0; // 光照计标志位

// Task ID for internal task/event processing
static uint8_t Broadcaster_TaskID;

// 广播数据 (包含UUID和测量数据)
static uint8_t advertData[] = {
    0x02, GAP_ADTYPE_FLAGS, GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
    
    0x10, 0x16, 0xD2, 0xFC, // 长度、AD类型、UUID (BTHome UUID FCD2) 
    0x40,                   // BTHome v2 无加密，定期广播
    0x05, 0x00, 0x00, 0x00, // 照度 3字节
    0x0C, 0x00, 0xFF,       // 电压 2字节
    0x3D, 0x00, 0x00,       // 计数 2字节
    0x2D, 0x00,             // 门窗状态
    0x09, GAP_ADTYPE_LOCAL_NAME_COMPLETE, // 完整本地名称
    'L', 'M', 'S', 'e', 'n', 's', 'o', 'r', // 名称 "LMSensor"

};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void Broadcaster_ProcessTMOSMsg(tmos_event_hdr_t* pMsg);
static void Broadcaster_StateNotificationCB(gapRole_States_t newState);
extern bStatus_t GAP_UpdateAdvertisingData(uint8_t taskID, uint8_t adType, uint16_t dataLen, uint8_t* pAdvertData);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesBroadcasterCBs_t Broadcaster_BroadcasterCBs = {
    Broadcaster_StateNotificationCB, // Profile State Change Callbacks
    NULL
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

// 电池电压采样
__HIGH_CODE
uint16_t sample_battery_voltage()
{
    // VINA 实际电压值 1050±15mV
    const int vref = 1050;

    ADC_InterBATSampInit();

    // 每200次进行一次粗调校准
    static uint8_t calib_count = 0;
    calib_count++;
    if (calib_count == 1) {
        RoughCalib_Value = ADC_DataCalib_Rough();
    }
    calib_count %= 200;

    ADC_ChannelCfg(CH_INTE_VBAT);
    return (ADC_ExcutSingleConver() + RoughCalib_Value) * vref / 512 - 3 * vref;
}

/*
# Step 1: 唤醒 + 触发单次测量（CT=0，100ms，RN=1100=自动范围，M=01=单次模式）
I2C_WriteRegister(0x44, 0x01, 0xC010)  # 配置寄存器：1100 0000 0001 0000b

# Step 2: 等待测量完成（轮询CRF = bit7 = 1）
do {
    cfg = I2C_ReadRegister(0x44, 0x01)
} while ((cfg & 0x0080) == 0)

# Step 3: 读取光照值
raw = I2C_ReadRegister(0x44, 0x00)
E = (raw >> 12) & 0x0F
R = raw & 0x0FFF
lux = 0.01 * (2^E) * R

# Step 4: 写回配置寄存器，使其进入关断状态
I2C_WriteRegister(0x44, 0x01, 0xC000)
*/

uint8_t ipt3001_state = 0; // 0: 开始转换，1: 获取转换数据

#define OPT3001_I2C_ADDR  0x44  // 如果 ADDR 引脚接 GND

/**
 * @brief   启动 OPT3001 的光照值转换
 * @param   NULL
 * @return  0 成功，<0 错误码
 */
__HIGH_CODE
int opt3001_start()
{
     i2c_app_init(OPT3001_I2C_ADDR);
    uint8_t buf[3] = {0x01, 0xCA, 0x10}; // 配置寄存器：1100 0000 0001 0000b

    int ret = i2c_write_to(OPT3001_I2C_ADDR, buf, 3, 1, 1); // 不发送 STOP（Repeated Start）
    return ret;  // 返回写入结果
}


// /**
//  * @brief  检查 OPT3001 单次测量是否完成（CRF == 1）
//  * @param  timeout_ms 最大轮询时间（毫秒）
//  * @return 1 表示测量完成，0 表示未完成或超时
//  */
// __HIGH_CODE
// int opt3001_is_ready(int timeout_ms)
// {
//     uint8_t reg_addr = 0x01;  // 配置寄存器地址
//     uint8_t buf[2];
//     uint16_t cfg;
//     int ret;

//     while (timeout_ms-- > 0) {
//         // Step 1: 写入配置寄存器地址（不发送 STOP）
//         ret = i2c_write_to(OPT3001_I2C_ADDR, &reg_addr, 1, 1, 0);
//         if (ret < 0) return 0;

//         // Step 2: 读取配置寄存器（发送 STOP）
//         ret = i2c_read_from(OPT3001_I2C_ADDR, buf, 2, 1, 1000);
//         if (ret < 0) return 0;

//         cfg = ((uint16_t)buf[0] << 8) | buf[1];

//         // Step 3: 检查 CRF 位（bit 7）
//         if (cfg & 0x0080) {
//             return 1;  // 测量完成
//         }

//         DelayMs(1);  // 延迟 1ms，再试
//     }

//     return 0;  // 超时未完成
// }


static uint8_t gpio_flag = 1; 
__HIGH_CODE
void Read_GPIO_PB12(void)
{
    // 读取PA0电平
    if(GPIOB_ReadPortPin(GPIO_Pin_12))
        gpio_flag = 1;  // 高电平
    else
        gpio_flag = 0;  // 低电平
}


/**
 * @brief   读取 OPT3001 的光照值（单位：0.01lux）
 * @param   lux_out 指针，输出 lux 值（uint32_t数）
 * @return  0 成功，<0 错误码
 */
__HIGH_CODE
int opt3001_read_lux(uint32_t *lux_out)
{

    uint8_t reg_addr = 0x00;       // Result register
    uint8_t buf[2];                // 接收 2 字节寄存器数据
    
    i2c_app_init(OPT3001_I2C_ADDR);

    // Step 1: 发送要读取的寄存器地址
    int ret = i2c_write_to(OPT3001_I2C_ADDR, &reg_addr, 1, 1, 0); // 不发送 STOP（Repeated Start）
    if (ret < 0) return ret;

    // Step 2: 从设备读取2字节数据（MSB + LSB）
    ret = i2c_read_from(OPT3001_I2C_ADDR, buf, 2, 1, 100); // 发送 STOP，timeout=1000
    if (ret < 0) return ret;

    // Step 3: 解析原始数据
    uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
    uint8_t exponent = (raw >> 12) & 0x0F;
    uint16_t mantissa = raw & 0x0FFF;

    // Step 4: 计算 lux
    uint16_t lsb = (1 << exponent);
   // PRINT("OPT3001: raw=0x%04X, exponent=%d, mantissa=0x%03X, lsb=%d\n", raw, exponent, mantissa, lsb);
    *lux_out = lsb * mantissa;
    PRINT("OPT3001: lux=%lu\n", *lux_out);


    return 0;
}

__HIGH_CODE
void update_advert_data() {

    if (mag_int_flag) {
        mag_int_flag = 0; // 清除中断标志
        // 读取磁强计数据
        advertData[COUNTER_OFFSET] = (uint8_t)(mag_count & 0xFF);       // 低字节
        advertData[COUNTER_OFFSET + 1] = (uint8_t)((mag_count >> 8) & 0xFF); // 高字节
    }


    uint16_t bl_bat;
    bl_bat = sample_battery_voltage();
    advertData[BAT_OFFSET] = (uint8_t)(bl_bat & 0xFF);       // 低字节
    advertData[BAT_OFFSET + 1] = (uint8_t)((bl_bat >> 8) & 0xFF); // 高字节
    
    advertData[9] = (uint8_t)(opt3001_lux & 0xFF);       // 低字节
    advertData[10] = (uint8_t)((opt3001_lux >> 8) & 0xFF); // 中字节
    advertData[11] = (uint8_t)((opt3001_lux >> 16) & 0xFF); // 高字节

    Read_GPIO_PB12();
    // 更新门窗状态
    if (gpio_flag) {
        advertData[WINDOWS_OFFSET] = 0x01; // 门窗状态：开
    } else {
        advertData[WINDOWS_OFFSET] = 0x00; // 门窗状态：关
    }
    PRINT("GPIO PB12: %d\n", gpio_flag);

}



/*********************************************************************
 * @fn      Broadcaster_Init
 *
 * @brief   Initialization function for the Broadcaster App
 *          Task. This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by TMOS.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void Broadcaster_Init()
{

    Broadcaster_TaskID = TMOS_ProcessEventRegister(Broadcaster_ProcessEvent);

    // Setup the GAP Broadcaster Role Profile
    {
        // Device starts advertising upon initialization
        uint8_t initial_advertising_enable = TRUE;
        uint8_t initial_adv_event_type = GAP_ADTYPE_ADV_NONCONN_IND;
        // uint8_t initial_adv_event_type = GAP_ADTYPE_ADV_IND;
        // uint8_t initial_adv_event_type = GAP_ADTYPE_EXT_NONCONN_NONSCAN_UNDIRECT;

        // Set the GAP Role Parameters
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t), &initial_advertising_enable);
        GAPRole_SetParameter(GAPROLE_ADV_EVENT_TYPE, sizeof(uint8_t), &initial_adv_event_type);
        GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);
    }

    // Set advertising interval
    {
        uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

        GAP_SetParamValue(TGAP_DISC_ADV_INT_MIN, advInt);
        GAP_SetParamValue(TGAP_DISC_ADV_INT_MAX, advInt);

        // GAP_SetParamValue(TGAP_ADV_SECONDARY_PHY, GAP_PHY_VAL_LE_CODED); // 125K
        // GAP_SetParamValue(TGAP_ADV_PRIMARY_PHY, GAP_PHY_VAL_LE_CODED); // 125K
    }

    // Setup a delayed profile startup
    // tmos_set_event(Broadcaster_TaskID, SBP_START_DEVICE_EVT);
    tmos_start_task(Broadcaster_TaskID, SBP_START_DEVICE_EVT, DEFAULT_ADVERTISING_INTERVAL);

    // 设置定时读取传感器并更新广播
    tmos_start_task(Broadcaster_TaskID, SBP_PERIODIC_EVT, 2 * DEFAULT_ADVERTISING_INTERVAL - 320);

    tmos_start_task(Broadcaster_TaskID, OPT3001_START_EVENT, 1600);
}

/*********************************************************************
 * @fn      Broadcaster_ProcessEvent
 *
 * @brief   Broadcaster Application Task event processor. This
 *          function is called to process all events for the task. Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The TMOS assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16_t Broadcaster_ProcessEvent(uint8_t task_id, uint16_t events)
{
    if (events & SYS_EVENT_MSG) {
        uint8_t* pMsg;

        if ((pMsg = tmos_msg_receive(Broadcaster_TaskID)) != NULL) {
            Broadcaster_ProcessTMOSMsg((tmos_event_hdr_t*)pMsg);

            // Release the TMOS message
            tmos_msg_deallocate(pMsg);
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if (events & SBP_START_DEVICE_EVT) {
        // Start the Device
        GAPRole_BroadcasterStartDevice(&Broadcaster_BroadcasterCBs);

        return (events ^ SBP_START_DEVICE_EVT);
    }

    if (events & SBP_PERIODIC_EVT) {
        tmos_start_task(Broadcaster_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD);
        PRINT("SBP_PERIODIC_EVT\n");
        // 数据采集并更新广播
        update_advert_data();
        GAP_UpdateAdvertisingData(0, TRUE, sizeof(advertData), advertData);

        return (events ^ SBP_PERIODIC_EVT);
    }


    if (events & OPT3001_START_EVENT) {
        // 启动 OPT3001 进行光照测量
        opt3001_start();
        // ipt3001_state = 0; // 设置状态为开始转换
        tmos_start_task(Broadcaster_TaskID, OPT3001_READ_EVENT, OPT3001_READ_EVENT_PERIOD);
        return (events ^ OPT3001_START_EVENT);
    }

    if (events & OPT3001_READ_EVENT) {
        // 读取 OPT3001 光照值
        // ipt3001_state = 1; // 设置状态为转换结束
        opt3001_read_lux(&opt3001_lux);
        tmos_start_task(Broadcaster_TaskID, OPT3001_START_EVENT, OPT3001_CJ_EVENT_PERIOD);
        return (events ^ OPT3001_READ_EVENT);
    }

    // Discard unknown events
    return 0;
}

/*********************************************************************
 * @fn      Broadcaster_ProcessTMOSMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void Broadcaster_ProcessTMOSMsg(tmos_event_hdr_t* pMsg)
{
    switch (pMsg->event) {
    default:
        break;
    }
}

/*********************************************************************
 * @fn      Broadcaster_StateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void Broadcaster_StateNotificationCB(gapRole_States_t newState)
{
    switch (newState) {
    case GAPROLE_STARTED:
        PRINT("Initialized..\n");
        break;

    case GAPROLE_ADVERTISING:
        PRINT("Advertising..\n");
        break;

    case GAPROLE_WAITING:
        PRINT("Waiting for advertising..\n");
        break;

    case GAPROLE_ERROR:
        PRINT("Error..\n");
        break;

    default:
        break;
    }
}
