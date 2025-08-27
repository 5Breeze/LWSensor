/********************************** (C) COPYRIGHT *******************************
 * File Name          : broadcaster.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2020/08/06
 * Description        : �㲥Ӧ�ó��򣬳�ʼ���㲥���Ӳ�����Ȼ���ڹ㲥̬һֱ�㲥

 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CONFIG.h"
#include "devinfoservice.h"
#include "broadcaster.h"
#include "app_i2c.h"

// �㲥��� (units of 625us, min is 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL 1600 * 2 //2s

// �ɼ����
#define SBP_PERIODIC_EVT_PERIOD 1600 * 20  // 20s����

// opt3001 ת�����
#define OPT3001_READ_EVENT_PERIOD 1600 * 1 //2s

// opt3001 �ɼ����
#define OPT3001_CJ_EVENT_PERIOD 1600 * 30   //40s


#define COUNTER_OFFSET 16
#define BAT_OFFSET 13
#define WINDOWS_OFFSET 19
// ADC �����ֵ�ƫ��ֵ
static signed short RoughCalib_Value = 0;
// ��ص�ѹ
static uint16_t bat = 0;

static uint32_t opt3001_lux = 0; // ����ֵ

uint16_t mag_count = 0; // Task ID for internal task/event processing
uint8_t mag_int_flag = 0; // ��ǿ���жϱ�־λ
uint8_t lux_int_flag = 0; // ���ռƱ�־λ

// Task ID for internal task/event processing
static uint8_t Broadcaster_TaskID;

// �㲥���� (����UUID�Ͳ�������)
static uint8_t advertData[] = {
    0x02, GAP_ADTYPE_FLAGS, GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
    
    0x10, 0x16, 0xD2, 0xFC, // ���ȡ�AD���͡�UUID (BTHome UUID FCD2) 
    0x40,                   // BTHome v2 �޼��ܣ����ڹ㲥
    0x05, 0x00, 0x00, 0x00, // �ն� 3�ֽ�
    0x0C, 0x00, 0xFF,       // ��ѹ 2�ֽ�
    0x3D, 0x00, 0x00,       // ���� 2�ֽ�
    0x2D, 0x00,             // �Ŵ�״̬
    0x09, GAP_ADTYPE_LOCAL_NAME_COMPLETE, // ������������
    'L', 'M', 'S', 'e', 'n', 's', 'o', 'r', // ���� "LMSensor"

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

// ��ص�ѹ����
__HIGH_CODE
uint16_t sample_battery_voltage()
{
    // VINA ʵ�ʵ�ѹֵ 1050��15mV
    const int vref = 1050;

    ADC_InterBATSampInit();

    // ÿ200�ν���һ�δֵ�У׼
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
# Step 1: ���� + �������β�����CT=0��100ms��RN=1100=�Զ���Χ��M=01=����ģʽ��
I2C_WriteRegister(0x44, 0x01, 0xC010)  # ���üĴ�����1100 0000 0001 0000b

# Step 2: �ȴ�������ɣ���ѯCRF = bit7 = 1��
do {
    cfg = I2C_ReadRegister(0x44, 0x01)
} while ((cfg & 0x0080) == 0)

# Step 3: ��ȡ����ֵ
raw = I2C_ReadRegister(0x44, 0x00)
E = (raw >> 12) & 0x0F
R = raw & 0x0FFF
lux = 0.01 * (2^E) * R

# Step 4: д�����üĴ�����ʹ�����ض�״̬
I2C_WriteRegister(0x44, 0x01, 0xC000)
*/

uint8_t ipt3001_state = 0; // 0: ��ʼת����1: ��ȡת������

#define OPT3001_I2C_ADDR  0x44  // ��� ADDR ���Ž� GND

/**
 * @brief   ���� OPT3001 �Ĺ���ֵת��
 * @param   NULL
 * @return  0 �ɹ���<0 ������
 */
__HIGH_CODE
int opt3001_start()
{
     i2c_app_init(OPT3001_I2C_ADDR);
    uint8_t buf[3] = {0x01, 0xCA, 0x10}; // ���üĴ�����1100 0000 0001 0000b

    int ret = i2c_write_to(OPT3001_I2C_ADDR, buf, 3, 1, 1); // ������ STOP��Repeated Start��
    return ret;  // ����д����
}


// /**
//  * @brief  ��� OPT3001 ���β����Ƿ���ɣ�CRF == 1��
//  * @param  timeout_ms �����ѯʱ�䣨���룩
//  * @return 1 ��ʾ������ɣ�0 ��ʾδ��ɻ�ʱ
//  */
// __HIGH_CODE
// int opt3001_is_ready(int timeout_ms)
// {
//     uint8_t reg_addr = 0x01;  // ���üĴ�����ַ
//     uint8_t buf[2];
//     uint16_t cfg;
//     int ret;

//     while (timeout_ms-- > 0) {
//         // Step 1: д�����üĴ�����ַ�������� STOP��
//         ret = i2c_write_to(OPT3001_I2C_ADDR, &reg_addr, 1, 1, 0);
//         if (ret < 0) return 0;

//         // Step 2: ��ȡ���üĴ��������� STOP��
//         ret = i2c_read_from(OPT3001_I2C_ADDR, buf, 2, 1, 1000);
//         if (ret < 0) return 0;

//         cfg = ((uint16_t)buf[0] << 8) | buf[1];

//         // Step 3: ��� CRF λ��bit 7��
//         if (cfg & 0x0080) {
//             return 1;  // �������
//         }

//         DelayMs(1);  // �ӳ� 1ms������
//     }

//     return 0;  // ��ʱδ���
// }


static uint8_t gpio_flag = 1; 
__HIGH_CODE
void Read_GPIO_PB12(void)
{
    // ��ȡPA0��ƽ
    if(GPIOB_ReadPortPin(GPIO_Pin_12))
        gpio_flag = 1;  // �ߵ�ƽ
    else
        gpio_flag = 0;  // �͵�ƽ
}


/**
 * @brief   ��ȡ OPT3001 �Ĺ���ֵ����λ��0.01lux��
 * @param   lux_out ָ�룬��� lux ֵ��uint32_t����
 * @return  0 �ɹ���<0 ������
 */
__HIGH_CODE
int opt3001_read_lux(uint32_t *lux_out)
{

    uint8_t reg_addr = 0x00;       // Result register
    uint8_t buf[2];                // ���� 2 �ֽڼĴ�������
    
    i2c_app_init(OPT3001_I2C_ADDR);

    // Step 1: ����Ҫ��ȡ�ļĴ�����ַ
    int ret = i2c_write_to(OPT3001_I2C_ADDR, &reg_addr, 1, 1, 0); // ������ STOP��Repeated Start��
    if (ret < 0) return ret;

    // Step 2: ���豸��ȡ2�ֽ����ݣ�MSB + LSB��
    ret = i2c_read_from(OPT3001_I2C_ADDR, buf, 2, 1, 100); // ���� STOP��timeout=1000
    if (ret < 0) return ret;

    // Step 3: ����ԭʼ����
    uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
    uint8_t exponent = (raw >> 12) & 0x0F;
    uint16_t mantissa = raw & 0x0FFF;

    // Step 4: ���� lux
    uint16_t lsb = (1 << exponent);
   // PRINT("OPT3001: raw=0x%04X, exponent=%d, mantissa=0x%03X, lsb=%d\n", raw, exponent, mantissa, lsb);
    *lux_out = lsb * mantissa;
    PRINT("OPT3001: lux=%lu\n", *lux_out);


    return 0;
}

__HIGH_CODE
void update_advert_data() {

    if (mag_int_flag) {
        mag_int_flag = 0; // ����жϱ�־
        // ��ȡ��ǿ������
        advertData[COUNTER_OFFSET] = (uint8_t)(mag_count & 0xFF);       // ���ֽ�
        advertData[COUNTER_OFFSET + 1] = (uint8_t)((mag_count >> 8) & 0xFF); // ���ֽ�
    }


    uint16_t bl_bat;
    bl_bat = sample_battery_voltage();
    advertData[BAT_OFFSET] = (uint8_t)(bl_bat & 0xFF);       // ���ֽ�
    advertData[BAT_OFFSET + 1] = (uint8_t)((bl_bat >> 8) & 0xFF); // ���ֽ�
    
    advertData[9] = (uint8_t)(opt3001_lux & 0xFF);       // ���ֽ�
    advertData[10] = (uint8_t)((opt3001_lux >> 8) & 0xFF); // ���ֽ�
    advertData[11] = (uint8_t)((opt3001_lux >> 16) & 0xFF); // ���ֽ�

    Read_GPIO_PB12();
    // �����Ŵ�״̬
    if (gpio_flag) {
        advertData[WINDOWS_OFFSET] = 0x01; // �Ŵ�״̬����
    } else {
        advertData[WINDOWS_OFFSET] = 0x00; // �Ŵ�״̬����
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

    // ���ö�ʱ��ȡ�����������¹㲥
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
        // ���ݲɼ������¹㲥
        update_advert_data();
        GAP_UpdateAdvertisingData(0, TRUE, sizeof(advertData), advertData);

        return (events ^ SBP_PERIODIC_EVT);
    }


    if (events & OPT3001_START_EVENT) {
        // ���� OPT3001 ���й��ղ���
        opt3001_start();
        // ipt3001_state = 0; // ����״̬Ϊ��ʼת��
        tmos_start_task(Broadcaster_TaskID, OPT3001_READ_EVENT, OPT3001_READ_EVENT_PERIOD);
        return (events ^ OPT3001_START_EVENT);
    }

    if (events & OPT3001_READ_EVENT) {
        // ��ȡ OPT3001 ����ֵ
        // ipt3001_state = 1; // ����״̬Ϊת������
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
