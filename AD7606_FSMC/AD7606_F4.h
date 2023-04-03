//
// Created by 86187 on 2023/3/25.
//

#ifndef AD7606_F4_H
#define AD7606_F4_H

#define BUSY_ENABLE 0

#if BUSY_ENABLE
#define BUSY_Pin GPIO_PIN_8
#define BUSY_GPIO_Port GPIOC
#define BUSY_EXTI_IRQn EXTI9_5_IRQn
#endif

#include "main.h"


#define RST_Pin GPIO_PIN_3
#define RST_GPIO_Port GPIOB
#define STBY_Pin GPIO_PIN_6
#define STBY_GPIO_Port GPIOB
#define OS12_Pin GPIO_PIN_7
#define OS12_GPIO_Port GPIOB
#define OS11_Pin GPIO_PIN_8
#define OS11_GPIO_Port GPIOB
#define OS10_Pin GPIO_PIN_9
#define OS10_GPIO_Port GPIOB



typedef enum
{
    AD_OS_NO = 0,
    AD_OS_X2 = 1,
    AD_OS_X4 = 2,
    AD_OS_X8 = 3,
    AD_OS_X16 = 4,
    AD_OS_X32 = 5,
    AD_OS_X64 = 6
}AD7606_OS_E;

typedef struct
{
    uint8_t ucOS;			/* 过采样倍率，0 - 6. 0表示无过采样 */
    uint8_t ucRange;		/* 输入量程，0表示正负5V, 1表示正负10V */
    int16_t sNowAdc[8];		/* 当前ADC值, 有符号数 */
}AD7606_VAR_T;

void Init_AD7606(void);
void AD7606_SetOS(uint8_t _ucOS);
void AD7606_Reset(void);
void AD7606_StartConvst(void);
void AD7606_ReadNowAdc(void);

void AD7606_EnterAutoMode(uint32_t _ulFreq);
void AD7606_StartRecord(uint32_t _ulFreq);
void AD7606_StopRecord(void);
uint8_t AD7606_FifoNewData(void);
uint8_t AD7606_ReadFifo(uint16_t *_usReadAdc);
uint8_t AD7606_FifoFull(void);

extern AD7606_VAR_T g_tAD7606;

#endif //AD7606_FSMC_AD7606_F4_H
