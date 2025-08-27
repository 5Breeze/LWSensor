/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2020/08/06
 * Description        : �㲥Ӧ��������������ϵͳ��ʼ��
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/******************************************************************************/
/* ͷ�ļ����� */
#include "CONFIG.h"
#include "HAL.h"
#include "broadcaster.h"

/*********************************************************************
 * GLOBAL TYPEDEFS
 */
__attribute__((aligned(4))) uint32_t MEM_BUF[BLE_MEMHEAP_SIZE / 4];

#if (defined(BLE_MAC)) && (BLE_MAC == TRUE)
const uint8_t MacAddr[6] = { 0x84, 0xC2, 0xE4, 0x03, 0x02, 0x02 };
#endif

/*********************************************************************
 * @fn      Main_Circulation
 *
 * @brief   ��ѭ��
 *
 * @return  none
 */
__HIGH_CODE
__attribute__((noinline)) void Main_Circulation()
{
    while (1) {
        TMOS_SystemProcess();
    }
}

/*********************************************************************
 * @fn      main
 *
 * @brief   ������
 *
 * @return  none
 */
int main(void)
{
#if (defined(DCDC_ENABLE)) && (DCDC_ENABLE == TRUE)
    PWR_DCDCCfg(ENABLE);
#endif
    SetSysClock(CLK_SOURCE_PLL_60MHz);
#if (defined(HAL_SLEEP)) && (HAL_SLEEP == TRUE)
    GPIOA_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PU);
    GPIOB_ModeCfg(GPIO_Pin_All, GPIO_ModeIN_PU);
#endif
#ifdef DEBUG
    GPIOA_SetBits(bTXD1);
    GPIOA_ModeCfg(bTXD1, GPIO_ModeOut_PP_5mA);
    UART1_DefInit();
#endif
    /* ���û���ԴΪ GPIO - PB12 */
    GPIOB_ModeCfg(GPIO_Pin_12, GPIO_ModeIN_Floating);

    if(GPIOB_ReadPortPin(GPIO_Pin_12))
        GPIOB_ITModeCfg(GPIO_Pin_12, GPIO_ITMode_FallEdge); // �½��ػ���
    else
        GPIOB_ITModeCfg(GPIO_Pin_12, GPIO_ITMode_RiseEdge); // �½��ػ���

    

    PFIC_EnableIRQ(GPIO_B_IRQn);
    PWR_PeriphWakeUpCfg(ENABLE, RB_SLP_GPIO_WAKE, RB_SLP_GPIO_WAKE);

    PRINT("%s\n", VER_LIB);
    CH59x_BLEInit();
    HAL_Init();
    GAPRole_BroadcasterInit();
    Broadcaster_Init();
    Main_Circulation();
}

//�ж������ж�
__INTERRUPT
__HIGH_CODE
void GPIOB_IRQHandler( void )
{
    // ��⵽PB12�ж�
    if(GPIOB_ReadITFlagBit(GPIO_Pin_12))
    {
        PRINT("ENTER GPIOB_IRQHandler\n");
        mag_int_flag = 1; // ���ô�ǿ���жϱ�־λ
        mag_count++;
            if(GPIOB_ReadPortPin(GPIO_Pin_12))
        GPIOB_ITModeCfg(GPIO_Pin_12, GPIO_ITMode_FallEdge); // �½��ػ���
    else
        GPIOB_ITModeCfg(GPIO_Pin_12, GPIO_ITMode_RiseEdge); // �½��ػ���
        GPIOB_ClearITFlagBit(GPIO_Pin_12);
    }
}
/******************************** endfile @ main ******************************/
