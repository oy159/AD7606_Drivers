//
// Created by 0Y on 2023/3/25.
//

/*********************AD7606+FSMC+TIM+DMA/EXTI********************/

#include "AD7606_F4.h"

#define OS0_1()		HAL_GPIO_WritePin(OS10_GPIO_Port, OS10_Pin, GPIO_PIN_SET)
#define OS0_0()		HAL_GPIO_WritePin(OS10_GPIO_Port, OS10_Pin, GPIO_PIN_RESET)
#define OS1_1()		HAL_GPIO_WritePin(OS11_GPIO_Port, OS11_Pin, GPIO_PIN_SET)
#define OS1_0()		HAL_GPIO_WritePin(OS11_GPIO_Port, OS11_Pin, GPIO_PIN_RESET)
#define OS2_1()		HAL_GPIO_WritePin(OS12_GPIO_Port, OS12_Pin, GPIO_PIN_SET)
#define OS2_0()		HAL_GPIO_WritePin(OS12_GPIO_Port, OS12_Pin, GPIO_PIN_RESET)
#define RESET_0()    HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET)
#define RESET_1()    HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET)
#define STby_1()	HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET)


#define CONVST_RCC_GPIO_CLK_ENABLE	__HAL_RCC_GPIOC_CLK_ENABLE
#define CONVST_TIM8_CLK_ENABLE      __HAL_RCC_TIM8_CLK_ENABLE
#define CONVST_RCC_GPIO_CLK_DISBALE	__HAL_RCC_GPIOC_CLK_DISABLE
#define CONVST_TIM8_CLK_DISABLE     __HAL_RCC_TIM8_CLK_DISABLE
#define CONVST_GPIO		GPIOC
#define CONVST_PIN		GPIO_PIN_6
#define CONVST_AF		GPIO_AF3_TIM8
#define CONVST_TIMX		TIM8
#define CONVST_TIMCH	TIM_CHANNEL_1


#define TIMx_UP_DMA_STREAM_CLK_ENABLE  	__HAL_RCC_DMA2_CLK_ENABLE
#define TIMx_UP_DMA_STREAM_CLK_DISABLE  __HAL_RCC_DMA2_CLK_DISABLE
#define TIMx_UP_DMA_STREAM             DMA2_Stream1
#define TIMx_UP_DMA_CHANNEL            DMA_CHANNEL_7
#define TIMx_UP_DMA_IRQn               DMA2_Stream1_IRQn
#define TIMx_UP_DMA_IRQHandler         DMA2_Stream1_IRQHandler


#define CONVST_1()		CONVST_GPIO->BSRR = CONVST_PIN
#define CONVST_0()		CONVST_GPIO->BSRR = ((uint32_t)CONVST_PIN << 16U)

#define AD7606_BASE    	0x6C000000
#define AD7606_RESULT()	*(__IO uint16_t *)0x6C000000

static DMA_HandleTypeDef TIMDMA = {0};
static TIM_HandleTypeDef TimHandle = {0};

#define AD7606_BUFSIZE        1024*8
__align(16) int16_t g_sAd7606Buf[AD7606_BUFSIZE];
AD7606_VAR_T g_tAD7606;

static void AD7606_FSMC_GPIO_Init(void);
static void AD7606_FSMCConfig(void);
static void AD7606_SetTIMOutPWM(TIM_TypeDef* TIMx, uint32_t _ulFreq);

void Init_AD7606(void)
{
    AD7606_FSMC_GPIO_Init();
    AD7606_FSMCConfig();

    AD7606_SetOS(AD_OS_NO);		/* �޹����� */
		STby_1();							
    AD7606_Reset();				/* ��λ */
    CONVST_1();					/* ����ת����GPIO��ƽʱ����Ϊ�� */
}

static void AD7606_FSMC_GPIO_Init(void){
    /*
     * */
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Peripheral clock enable */
		__HAL_RCC_GPIOD_CLK_ENABLE();
		__HAL_RCC_GPIOE_CLK_ENABLE();
		__HAL_RCC_GPIOF_CLK_ENABLE();
		__HAL_RCC_GPIOG_CLK_ENABLE();
		__HAL_RCC_GPIOH_CLK_ENABLE();
		__HAL_RCC_GPIOI_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();

		
    __HAL_RCC_FSMC_CLK_ENABLE();

    /** FSMC GPIO Configuration
    PF0   ------> FSMC_A0
    PE7   ------> FSMC_D4
    PE8   ------> FSMC_D5
    PE9   ------> FSMC_D6
    PE10   ------> FSMC_D7
    PE11   ------> FSMC_D8
    PE12   ------> FSMC_D9
    PE13   ------> FSMC_D10
    PE14   ------> FSMC_D11                                                                                             
    PE15   ------> FSMC_D12
    PD8   ------> FSMC_D13
    PD9   ------> FSMC_D14
    PD10   ------> FSMC_D15
    PD14   ------> FSMC_D0
    PD15   ------> FSMC_D1
    PD0   ------> FSMC_D2
    PD1   ------> FSMC_D3
    PD4   ------> FSMC_NOE
    PG12   ------> FSMC_NE4
    */
    /* GPIO_InitStruct */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FSMC;

    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* GPIO_InitStruct */
    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FSMC;

    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* GPIO_InitStruct */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FSMC;

    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* GPIO_InitStruct */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FSMC;

    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /* CONVST ����ADCת����GPIO = PC6 */
    {
        GPIO_InitTypeDef   GPIO_InitStructure;
        CONVST_RCC_GPIO_CLK_ENABLE();

        /* ����PC6 */
        GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		/* ����������� */
        GPIO_InitStructure.Pull = GPIO_NOPULL;				/* ���������費ʹ�� */
        GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;    /* GPIO�ٶȵȼ� */

        GPIO_InitStructure.Pin = CONVST_PIN;
        HAL_GPIO_Init(CONVST_GPIO, &GPIO_InitStructure);
    }

    /* ����OS0 = PB9,OS1 = PB8,OS2 = PBB7
     * ��ʼ���ⲿ�жϣ�DMAʱʡȥBusy�� */
    {
        /*Configure GPIO pin Output Level */
        HAL_GPIO_WritePin(GPIOB, RST_Pin|STBY_Pin|OS12_Pin|OS11_Pin
                                 |OS10_Pin, GPIO_PIN_RESET);

        #if BUSY_ENABLE
        /*Configure GPIO pin : PtPin */
        GPIO_InitStruct.Pin = BUSY_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        /* EXTI interrupt init*/
        HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
        #endif
        /*Configure GPIO pins : PBPin PBPin */
        GPIO_InitStruct.Pin = RST_Pin|STBY_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /*Configure GPIO pins : PBPin PBPin PBPin */
        GPIO_InitStruct.Pin = OS12_Pin|OS11_Pin|OS10_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    }
}


static void AD7606_FSMCConfig(void)
{
    /*
       DM9000����չIO��OLED��AD7606����һ��FMC���ã����������������FMC�ٶ�������Ϊ׼��
       �Ӷ���֤�������趼��������������
    */
    SRAM_HandleTypeDef hsram = {0};
    FSMC_NORSRAM_TimingTypeDef SRAM_Timing = {0};

    /*
        AD7606�����Ҫ��(3.3Vʱ��ͨ�ŵ�ƽVdriver)��RD���źŵ͵�ƽ���������21ns����ӦFMC��DataSetupTime
        CSƬѡ��RD���źŶ�����ʽ�ĸߵ�ƽ������̿��15ns��
        CSƬѡ��RD���źŲ�����ʽ�ĸߵ�ƽ������̿��22ns��
        ���ｫ22ns��Ϊ��Сֵ������Щ����ӦFMC��AddressSetupTime

        4-x-6-x-x-x  : RD�߳���35.7ns���͵�ƽ����23.8ns. ��ȡ8·�������ݵ��ڴ������476ns��
    */
    hsram.Instance  = FSMC_NORSRAM_DEVICE;
    hsram.Extended  = FSMC_NORSRAM_EXTENDED_DEVICE;

    /* FMCʹ�õ�HCLK����Ƶ168MHz��1��FMCʱ�����ھ���5.95ns */
    SRAM_Timing.AddressSetupTime       = 4;  /* 4*5.95ns=23.8ns����ַ����ʱ�䣬��Χ0 -15��FMCʱ�����ڸ��� */
    SRAM_Timing.AddressHoldTime        = 0;  /* ��ַ����ʱ�䣬����ΪģʽAʱ���ò����˲��� ��Χ1 -15��ʱ�����ڸ��� */
    SRAM_Timing.DataSetupTime          = 6;  /* 6*5.95ns=35.7ns�����ݱ���ʱ�䣬��Χ1 -255��ʱ�����ڸ��� */
    SRAM_Timing.BusTurnAroundDuration  = 0;  /* �������ò���������� */
    SRAM_Timing.CLKDivision            = 0;  /* �������ò���������� */
    SRAM_Timing.DataLatency            = 0;  /* �������ò���������� */
    SRAM_Timing.AccessMode             = FSMC_ACCESS_MODE_A;

    hsram.Init.NSBank             = FSMC_NORSRAM_BANK4;              /* ʹ�õ�BANK2����ʹ�õ�ƬѡFMC_NE2 */
    hsram.Init.DataAddressMux     = FSMC_DATA_ADDRESS_MUX_DISABLE;   /* ��ֹ��ַ���ݸ��� */
    hsram.Init.MemoryType         = FSMC_MEMORY_TYPE_SRAM;           /* �洢������SRAM */
    hsram.Init.MemoryDataWidth    = FSMC_NORSRAM_MEM_BUS_WIDTH_16;	/* 16λ���߿�� */
    hsram.Init.BurstAccessMode    = FSMC_BURST_ACCESS_MODE_DISABLE;  /* �ر�ͻ��ģʽ */
    hsram.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;   /* �������õȴ��źŵļ��ԣ��ر�ͻ��ģʽ���˲�����Ч */
    hsram.Init.WaitSignalActive   = FSMC_WAIT_TIMING_BEFORE_WS;      /* �ر�ͻ��ģʽ���˲�����Ч */
    hsram.Init.WriteOperation     = FSMC_WRITE_OPERATION_DISABLE;     /* ����ʹ�ܻ��߽�ֹд���� */
    hsram.Init.WaitSignal         = FSMC_WAIT_SIGNAL_DISABLE;        /* �ر�ͻ��ģʽ���˲�����Ч */
    hsram.Init.ExtendedMode       = FSMC_EXTENDED_MODE_DISABLE;      /* ��ֹ��չģʽ */
    hsram.Init.AsynchronousWait   = FSMC_ASYNCHRONOUS_WAIT_DISABLE;  /* �����첽�����ڼ䣬ʹ�ܻ��߽�ֹ�ȴ��źţ�����ѡ��ر� */
    hsram.Init.WriteBurst         = FSMC_WRITE_BURST_DISABLE;        /* ��ֹдͻ�� */
    hsram.Init.ContinuousClock    = FSMC_CONTINUOUS_CLOCK_SYNC_ONLY; /* ��ͬ��ģʽ����ʱ����� */
    hsram.Init.WriteFifo          = FSMC_WRITE_FIFO_ENABLE;          /* ʹ��дFIFO */
		hsram.Init.PageSize 					= FSMC_PAGE_SIZE_NONE;

    /* ��ʼ��SRAM������ */
    if (HAL_SRAM_Init(&hsram, &SRAM_Timing, &SRAM_Timing) != HAL_OK)
    {
        /* ��ʼ������ */
        Error_Handler();
    }
}

void AD7606_SetOS(uint8_t _ucOS)
{
    g_tAD7606.ucOS = _ucOS;
    switch (_ucOS)
    {
        case AD_OS_X2:
            OS2_0();
            OS1_0();
            OS0_1();
            break;

        case AD_OS_X4:
            OS2_0();
            OS1_1();
            OS0_0();
            break;

        case AD_OS_X8:
            OS2_0();
            OS1_1();
            OS0_1();
            break;

        case AD_OS_X16:
            OS2_1();
            OS1_0();
            OS0_0();
            break;

        case AD_OS_X32:
            OS2_1();
            OS1_0();
            OS0_1();
            break;

        case AD_OS_X64:
            OS2_1();
            OS1_1();
            OS0_0();
            break;

        case AD_OS_NO:
        default:
            g_tAD7606.ucOS = AD_OS_NO;
            OS2_0();
            OS1_0();
            OS0_0();
            break;
    }
}

void AD7606_Reset(void)
{
    RESET_0();	/* �˳���λ״̬ */

    RESET_1();	/* ���븴λ״̬ */
    RESET_1();	/* �������ӳ١� RESET��λ�ߵ�ƽ��������С50ns�� */
    RESET_1();
    RESET_1();

    RESET_0();	/* �˳���λ״̬ */
}

void AD7606_StartConvst(void)
{
    /* page 7��  CONVST �ߵ�ƽ�����Ⱥ͵͵�ƽ��������� 25ns */
    /* CONVSTƽʱΪ�� */
    CONVST_0();
    CONVST_0();
    CONVST_0();

    CONVST_1();
}

void AD7606_ReadNowAdc(void)
{
    g_tAD7606.sNowAdc[0] = AD7606_RESULT();	/* ����1·���� */
    g_tAD7606.sNowAdc[1] = AD7606_RESULT();	/* ����2·���� */
    g_tAD7606.sNowAdc[2] = AD7606_RESULT();	/* ����3·���� */
    g_tAD7606.sNowAdc[3] = AD7606_RESULT();	/* ����4·���� */
    g_tAD7606.sNowAdc[4] = AD7606_RESULT();	/* ����5·���� */
    g_tAD7606.sNowAdc[5] = AD7606_RESULT();	/* ����6·���� */
    g_tAD7606.sNowAdc[6] = AD7606_RESULT();	/* ����7·���� */
    g_tAD7606.sNowAdc[7] = AD7606_RESULT();	/* ����8·���� */

}

void AD7606_StartRecord(uint32_t _ulFreq)
{
    AD7606_StopRecord();

    AD7606_Reset();					/* ��λӲ�� */
    AD7606_StartConvst();			/* ���������������1������ȫ0������ */

    /* ����PC6ΪTIM8_CH1���� */
    AD7606_SetTIMOutPWM(CONVST_TIMX, _ulFreq);
}

void AD7606_StopRecord(void)
{
    /* ����PC6 ����͵�ƽ���ر�TIM */
    HAL_GPIO_DeInit(CONVST_GPIO, CONVST_PIN);
    CONVST_TIM8_CLK_DISABLE();
    TIMx_UP_DMA_STREAM_CLK_DISABLE();

    /* CONVST ����ADCת����GPIO = PC6 */
    {
        GPIO_InitTypeDef   GPIO_InitStructure;
                CONVST_RCC_GPIO_CLK_ENABLE();

        /* ����PC6 */
        GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		/* ����������� */
        GPIO_InitStructure.Pull = GPIO_NOPULL;				/* ���������費ʹ�� */
        GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;  /* GPIO�ٶȵȼ� */

        GPIO_InitStructure.Pin = CONVST_PIN;
        HAL_GPIO_Init(CONVST_GPIO, &GPIO_InitStructure);
    }

    CONVST_1();			/* ����ת����GPIOƽʱ����Ϊ�� */
}


__weak void AD7606_DmaCplCb(DMA_HandleTypeDef *hdma)
{

}

/* DMA�봫����ɻص������������� */
__weak void AD7606_DmaHalfCplCb(DMA_HandleTypeDef *hdma)
{

}

static void AD7606_SetTIMOutPWM(TIM_TypeDef* TIMx, uint32_t _ulFreq)
{
    TIM_OC_InitTypeDef sConfig = {0};
    GPIO_InitTypeDef   GPIO_InitStruct;
    uint16_t usPeriod;
    uint16_t usPrescaler;
    uint32_t uiTIMxCLK;
    uint32_t pulse;


    /* ����ʱ�� */
            CONVST_RCC_GPIO_CLK_ENABLE();
            CONVST_TIM8_CLK_ENABLE();
            TIMx_UP_DMA_STREAM_CLK_ENABLE();


    /* �������� */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = CONVST_AF;
    GPIO_InitStruct.Pin = CONVST_PIN;
    HAL_GPIO_Init(CONVST_GPIO, &GPIO_InitStruct);

    /*-----------------------------------------------------------------------
        system_stm32f4xx.c �ļ��� void SetSysClock(void) ������ʱ�ӵ��������£�

        HCLK = SYSCLK / 1     (AHB1Periph)
        PCLK2 = HCLK / 2      (APB2Periph)
        PCLK1 = HCLK / 4      (APB1Periph)

        ��ΪAPB1 prescaler != 1, ���� APB1�ϵ�TIMxCLK = PCLK1 x 2 = SystemCoreClock / 2;
        ��ΪAPB2 prescaler != 1, ���� APB2�ϵ�TIMxCLK = PCLK2 x 2 = SystemCoreClock;

        APB1 ��ʱ���� TIM2, TIM3 ,TIM4, TIM5, TIM6, TIM7, TIM12, TIM13,TIM14
        APB2 ��ʱ���� TIM1, TIM8 ,TIM9, TIM10, TIM11

    ----------------------------------------------------------------------- */
    if ((TIMx == TIM1) || (TIMx == TIM8) || (TIMx == TIM9) || (TIMx == TIM10) || (TIMx == TIM11))
    {
        /* APB2 ��ʱ��ʱ�� = 168M */
        uiTIMxCLK = SystemCoreClock;

        if (_ulFreq < 100)
        {
            usPrescaler = 10000 - 1;						/* ��Ƶ�� = 10000 */
            usPeriod =  (uiTIMxCLK / 10000) / _ulFreq  - 1;	/* �Զ���װ��ֵ�� usPeriod��Сֵ168, ��λ59us */
            pulse = usPeriod;                               /* ���õ͵�ƽʱ��59us��ע��usPeriod�Ѿ������˼�1���� */
        }
        else if (_ulFreq < 3000)
        {
            usPrescaler = 100 - 1;							/* ��Ƶ�� = 100 */
            usPeriod =  (uiTIMxCLK / 100) / _ulFreq  - 1;	/* �Զ���װ��ֵ�� usPeriod��Сֵ560����λ595ns */
            pulse = usPeriod-1;                           	/* ���õ͵�ƽʱ��1.19us��ע��usPeriod�Ѿ������˼�1���� */
        }
        else	/* ����4K��Ƶ�ʣ������Ƶ */
        {
            usPrescaler = 0;								/* ��Ƶ�� = 1 */
            usPeriod = uiTIMxCLK / _ulFreq - 1;				/* �Զ���װ��ֵ�� usPeriod��Сֵ840����λ5.95ns */
            pulse = usPeriod - 199;              			/* ���õ͵�ƽʱ��1.19us��ע��usPeriod�Ѿ������˼�1���� */
        }
    }
    else
    {
        /* APB1 ��ʱ�� = 84M */
        uiTIMxCLK = SystemCoreClock / 2;

        if (_ulFreq < 100)
        {
            usPrescaler = 10000 - 1;							/* ��Ƶ�� = 10000 */
            usPeriod =  (uiTIMxCLK / 10000) / _ulFreq  - 1;		/* �Զ���װ��ֵ�� usPeriod��Сֵ84, ��λ119us */
            pulse = usPeriod;                               	/* ���õ͵�ƽʱ��119us��ע��usPeriod�Ѿ������˼�1���� */
        }
        else if (_ulFreq < 3000)
        {
            usPrescaler = 100 - 1;							/* ��Ƶ�� = 100 */
            usPeriod =  (uiTIMxCLK / 100) / _ulFreq  - 1;	/* �Զ���װ��ֵ�� usPeriod��Сֵ280����λ1.19us */
            pulse = usPeriod;                           	/* ���õ͵�ƽʱ��1.19us��ע��usPeriod�Ѿ������˼�1���� */
        }
        else	/* ����4K��Ƶ�ʣ������Ƶ */
        {
            usPrescaler = 0;								/* ��Ƶ�� = 1 */
            usPeriod = uiTIMxCLK / _ulFreq - 1;				/* �Զ���װ��ֵ�� usPeriod��Сֵ420����λ11.9ns */
            pulse = usPeriod - 99;              			/* ���õ͵�ƽʱ��1.19us��ע��usPeriod�Ѿ������˼�1���� */
        }
    }

    /*  PWMƵ�� = TIMxCLK / usPrescaler + 1��/usPeriod + 1��*/
    TimHandle.Instance = TIMx;
    TimHandle.Init.Prescaler         = usPrescaler;         /* �������ö�ʱ����Ƶ */
    TimHandle.Init.Period            = usPeriod;            /* �������ö�ʱ������ */
    TimHandle.Init.ClockDivision     = 0;                   /* ����ָʾ��ʱ��ʱ�� (CK_INT) Ƶ���������������Լ������˲�����ETR�� TIx��
	                                                           ��ʹ�õ�����������ʱ�� (tDTS) ֮��ķ�Ƶ��*/
    TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;  /* �������ü���ģʽ�����ϼ���ģʽ */
    TimHandle.Init.RepetitionCounter = 0;                   /* ���������ظ����������� TIM1 �� TIM8 �У�������ʱ��û�� */
    TimHandle.Init.AutoReloadPreload = 0;                   /* �������ö�ʱ���� ARR �Զ���װ�Ĵ����Ǹ����¼�����ʱд����Ч */

    if (HAL_TIM_PWM_DeInit(&TimHandle) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK)
    {
        Error_Handler();
    }

    /* ���ö�ʱ��PWM���ͨ�� */
    sConfig.OCMode       = TIM_OCMODE_PWM1;         /* ��������Ƚ�ģʽ */
    sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;     /* ��������ߵ�ƽ��Ч */
    sConfig.OCFastMode   = TIM_OCFAST_DISABLE;      /* �رտ������ģʽ */
    sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;    /* ���û�������ߵ�ƽ��Ч */
    sConfig.OCIdleState  = TIM_OCIDLESTATE_SET;     /* ����״̬ʱ����������Ƚ�����Ϊ�ߵ�ƽ */
    sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;  /* ����״̬ʱ�����û�������Ƚ�����Ϊ�͵�ƽ */

    /* ռ�ձ� */
    sConfig.Pulse = pulse;
    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, CONVST_TIMCH) != HAL_OK)
    {
        Error_Handler();
    }

    /* ʹ�ܶ�ʱ���ж�  */
    __HAL_TIM_ENABLE_DMA(&TimHandle, TIM_DMA_UPDATE);

    /* ����PWM��� */
    if (HAL_TIM_PWM_Start(&TimHandle, CONVST_TIMCH) != HAL_OK)
    {
        Error_Handler();
    }

    /* ��ʱ��UP���´���DMA���� */
    TIMDMA.Instance                 = TIMx_UP_DMA_STREAM;      /* ����ʹ�õ�DMA������ */
    TIMDMA.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;     /* ʹ��FIFO*/
    TIMDMA.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL; /* �������÷�ֵ */
    TIMDMA.Init.MemBurst            = DMA_MBURST_INC8;	       /* ���ڴ洢��ͻ�� */
    TIMDMA.Init.PeriphBurst         = DMA_PBURST_INC8;	       /* ��������ͻ�� */
    TIMDMA.Init.Channel             = TIMx_UP_DMA_CHANNEL;     /* DMAͨ�� */
    TIMDMA.Init.Direction           = DMA_PERIPH_TO_MEMORY;    /* ���䷽���Ǵ����赽�洢�� */
    TIMDMA.Init.PeriphInc           = DMA_PINC_DISABLE;        /* �����ַ������ֹ */
    TIMDMA.Init.MemInc              = DMA_MINC_ENABLE;         /* �洢����ַ����ʹ�� */
    TIMDMA.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; /* �������ݴ���λ��ѡ����֣���16bit */
    TIMDMA.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD; /* �洢�����ݴ���λ��ѡ����֣���16bit */
    TIMDMA.Init.Mode                = DMA_CIRCULAR; 		   /* ѭ��ģʽ */
    TIMDMA.Init.Priority            = DMA_PRIORITY_LOW;        /* ���ȼ��� */

    /* ��λDMA */
    if(HAL_DMA_DeInit(&TIMDMA) != HAL_OK)
    {
        /* DMA����æ״̬��ֱ����ֹDMA */
        if(HAL_DMA_Abort_IT(&TIMDMA) != HAL_OK)
        {
            Error_Handler();
        }
    }

    /* ��ʼ��DMA */
    if(HAL_DMA_Init(&TIMDMA) != HAL_OK)
    {
        Error_Handler();
    }

    /* ����DMA�����TIM */
    //__HAL_LINKDMA(&TimHandle, hdma[TIM_DMA_ID_UPDATE], TIMDMA);

    /* ����DMA�ж� */
    HAL_NVIC_SetPriority(TIMx_UP_DMA_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIMx_UP_DMA_IRQn);

    /* ע��봫������жϺʹ�������ж� */
    HAL_DMA_RegisterCallback(&TIMDMA, HAL_DMA_XFER_CPLT_CB_ID, AD7606_DmaCplCb);
    HAL_DMA_RegisterCallback(&TIMDMA, HAL_DMA_XFER_HALFCPLT_CB_ID, AD7606_DmaHalfCplCb);

    /* ����DMA���� */
    HAL_DMA_Start_IT(&TIMDMA, (uint32_t)AD7606_BASE, (uint32_t)g_sAd7606Buf, AD7606_BUFSIZE);
}
