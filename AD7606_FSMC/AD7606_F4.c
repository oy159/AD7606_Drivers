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

    AD7606_SetOS(AD_OS_NO);		/* 无过采样 */
		STby_1();							
    AD7606_Reset();				/* 复位 */
    CONVST_1();					/* 启动转换的GPIO，平时设置为高 */
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

    /* CONVST 启动ADC转换的GPIO = PC6 */
    {
        GPIO_InitTypeDef   GPIO_InitStructure;
        CONVST_RCC_GPIO_CLK_ENABLE();

        /* 配置PC6 */
        GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		/* 设置推挽输出 */
        GPIO_InitStructure.Pull = GPIO_NOPULL;				/* 上下拉电阻不使能 */
        GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;    /* GPIO速度等级 */

        GPIO_InitStructure.Pin = CONVST_PIN;
        HAL_GPIO_Init(CONVST_GPIO, &GPIO_InitStructure);
    }

    /* 配置OS0 = PB9,OS1 = PB8,OS2 = PBB7
     * 初始化外部中断，DMA时省去Busy线 */
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
       DM9000，扩展IO，OLED和AD7606公用一个FMC配置，如果都开启，请以FMC速度最慢的为准。
       从而保证所有外设都可以正常工作。
    */
    SRAM_HandleTypeDef hsram = {0};
    FSMC_NORSRAM_TimingTypeDef SRAM_Timing = {0};

    /*
        AD7606规格书要求(3.3V时，通信电平Vdriver)：RD读信号低电平脉冲宽度最短21ns，对应FMC的DataSetupTime
        CS片选和RD读信号独立方式的高电平脉冲最短宽度15ns。
        CS片选和RD读信号并联方式的高电平脉冲最短宽度22ns。
        这里将22ns作为最小值更合理些，对应FMC的AddressSetupTime

        4-x-6-x-x-x  : RD高持续35.7ns，低电平持续23.8ns. 读取8路样本数据到内存差不多就是476ns。
    */
    hsram.Instance  = FSMC_NORSRAM_DEVICE;
    hsram.Extended  = FSMC_NORSRAM_EXTENDED_DEVICE;

    /* FMC使用的HCLK，主频168MHz，1个FMC时钟周期就是5.95ns */
    SRAM_Timing.AddressSetupTime       = 4;  /* 4*5.95ns=23.8ns，地址建立时间，范围0 -15个FMC时钟周期个数 */
    SRAM_Timing.AddressHoldTime        = 0;  /* 地址保持时间，配置为模式A时，用不到此参数 范围1 -15个时钟周期个数 */
    SRAM_Timing.DataSetupTime          = 6;  /* 6*5.95ns=35.7ns，数据保持时间，范围1 -255个时钟周期个数 */
    SRAM_Timing.BusTurnAroundDuration  = 0;  /* 此配置用不到这个参数 */
    SRAM_Timing.CLKDivision            = 0;  /* 此配置用不到这个参数 */
    SRAM_Timing.DataLatency            = 0;  /* 此配置用不到这个参数 */
    SRAM_Timing.AccessMode             = FSMC_ACCESS_MODE_A;

    hsram.Init.NSBank             = FSMC_NORSRAM_BANK4;              /* 使用的BANK2，即使用的片选FMC_NE2 */
    hsram.Init.DataAddressMux     = FSMC_DATA_ADDRESS_MUX_DISABLE;   /* 禁止地址数据复用 */
    hsram.Init.MemoryType         = FSMC_MEMORY_TYPE_SRAM;           /* 存储器类型SRAM */
    hsram.Init.MemoryDataWidth    = FSMC_NORSRAM_MEM_BUS_WIDTH_16;	/* 16位总线宽度 */
    hsram.Init.BurstAccessMode    = FSMC_BURST_ACCESS_MODE_DISABLE;  /* 关闭突发模式 */
    hsram.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;   /* 用于设置等待信号的极性，关闭突发模式，此参数无效 */
    hsram.Init.WaitSignalActive   = FSMC_WAIT_TIMING_BEFORE_WS;      /* 关闭突发模式，此参数无效 */
    hsram.Init.WriteOperation     = FSMC_WRITE_OPERATION_DISABLE;     /* 用于使能或者禁止写保护 */
    hsram.Init.WaitSignal         = FSMC_WAIT_SIGNAL_DISABLE;        /* 关闭突发模式，此参数无效 */
    hsram.Init.ExtendedMode       = FSMC_EXTENDED_MODE_DISABLE;      /* 禁止扩展模式 */
    hsram.Init.AsynchronousWait   = FSMC_ASYNCHRONOUS_WAIT_DISABLE;  /* 用于异步传输期间，使能或者禁止等待信号，这里选择关闭 */
    hsram.Init.WriteBurst         = FSMC_WRITE_BURST_DISABLE;        /* 禁止写突发 */
    hsram.Init.ContinuousClock    = FSMC_CONTINUOUS_CLOCK_SYNC_ONLY; /* 仅同步模式才做时钟输出 */
    hsram.Init.WriteFifo          = FSMC_WRITE_FIFO_ENABLE;          /* 使能写FIFO */
		hsram.Init.PageSize 					= FSMC_PAGE_SIZE_NONE;

    /* 初始化SRAM控制器 */
    if (HAL_SRAM_Init(&hsram, &SRAM_Timing, &SRAM_Timing) != HAL_OK)
    {
        /* 初始化错误 */
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
    RESET_0();	/* 退出复位状态 */

    RESET_1();	/* 进入复位状态 */
    RESET_1();	/* 仅用于延迟。 RESET复位高电平脉冲宽度最小50ns。 */
    RESET_1();
    RESET_1();

    RESET_0();	/* 退出复位状态 */
}

void AD7606_StartConvst(void)
{
    /* page 7：  CONVST 高电平脉冲宽度和低电平脉冲宽度最短 25ns */
    /* CONVST平时为高 */
    CONVST_0();
    CONVST_0();
    CONVST_0();

    CONVST_1();
}

void AD7606_ReadNowAdc(void)
{
    g_tAD7606.sNowAdc[0] = AD7606_RESULT();	/* 读第1路样本 */
    g_tAD7606.sNowAdc[1] = AD7606_RESULT();	/* 读第2路样本 */
    g_tAD7606.sNowAdc[2] = AD7606_RESULT();	/* 读第3路样本 */
    g_tAD7606.sNowAdc[3] = AD7606_RESULT();	/* 读第4路样本 */
    g_tAD7606.sNowAdc[4] = AD7606_RESULT();	/* 读第5路样本 */
    g_tAD7606.sNowAdc[5] = AD7606_RESULT();	/* 读第6路样本 */
    g_tAD7606.sNowAdc[6] = AD7606_RESULT();	/* 读第7路样本 */
    g_tAD7606.sNowAdc[7] = AD7606_RESULT();	/* 读第8路样本 */

}

void AD7606_StartRecord(uint32_t _ulFreq)
{
    AD7606_StopRecord();

    AD7606_Reset();					/* 复位硬件 */
    AD7606_StartConvst();			/* 启动采样，避免第1组数据全0的问题 */

    /* 配置PC6为TIM8_CH1功能 */
    AD7606_SetTIMOutPWM(CONVST_TIMX, _ulFreq);
}

void AD7606_StopRecord(void)
{
    /* 配置PC6 输出低电平，关闭TIM */
    HAL_GPIO_DeInit(CONVST_GPIO, CONVST_PIN);
    CONVST_TIM8_CLK_DISABLE();
    TIMx_UP_DMA_STREAM_CLK_DISABLE();

    /* CONVST 启动ADC转换的GPIO = PC6 */
    {
        GPIO_InitTypeDef   GPIO_InitStructure;
                CONVST_RCC_GPIO_CLK_ENABLE();

        /* 配置PC6 */
        GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		/* 设置推挽输出 */
        GPIO_InitStructure.Pull = GPIO_NOPULL;				/* 上下拉电阻不使能 */
        GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM;  /* GPIO速度等级 */

        GPIO_InitStructure.Pin = CONVST_PIN;
        HAL_GPIO_Init(CONVST_GPIO, &GPIO_InitStructure);
    }

    CONVST_1();			/* 启动转换的GPIO平时设置为高 */
}


__weak void AD7606_DmaCplCb(DMA_HandleTypeDef *hdma)
{

}

/* DMA半传输完成回调函数，弱定义 */
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


    /* 配置时钟 */
            CONVST_RCC_GPIO_CLK_ENABLE();
            CONVST_TIM8_CLK_ENABLE();
            TIMx_UP_DMA_STREAM_CLK_ENABLE();


    /* 配置引脚 */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = CONVST_AF;
    GPIO_InitStruct.Pin = CONVST_PIN;
    HAL_GPIO_Init(CONVST_GPIO, &GPIO_InitStruct);

    /*-----------------------------------------------------------------------
        system_stm32f4xx.c 文件中 void SetSysClock(void) 函数对时钟的配置如下：

        HCLK = SYSCLK / 1     (AHB1Periph)
        PCLK2 = HCLK / 2      (APB2Periph)
        PCLK1 = HCLK / 4      (APB1Periph)

        因为APB1 prescaler != 1, 所以 APB1上的TIMxCLK = PCLK1 x 2 = SystemCoreClock / 2;
        因为APB2 prescaler != 1, 所以 APB2上的TIMxCLK = PCLK2 x 2 = SystemCoreClock;

        APB1 定时器有 TIM2, TIM3 ,TIM4, TIM5, TIM6, TIM7, TIM12, TIM13,TIM14
        APB2 定时器有 TIM1, TIM8 ,TIM9, TIM10, TIM11

    ----------------------------------------------------------------------- */
    if ((TIMx == TIM1) || (TIMx == TIM8) || (TIMx == TIM9) || (TIMx == TIM10) || (TIMx == TIM11))
    {
        /* APB2 定时器时钟 = 168M */
        uiTIMxCLK = SystemCoreClock;

        if (_ulFreq < 100)
        {
            usPrescaler = 10000 - 1;						/* 分频比 = 10000 */
            usPeriod =  (uiTIMxCLK / 10000) / _ulFreq  - 1;	/* 自动重装的值， usPeriod最小值168, 单位59us */
            pulse = usPeriod;                               /* 设置低电平时间59us，注意usPeriod已经进行了减1操作 */
        }
        else if (_ulFreq < 3000)
        {
            usPrescaler = 100 - 1;							/* 分频比 = 100 */
            usPeriod =  (uiTIMxCLK / 100) / _ulFreq  - 1;	/* 自动重装的值， usPeriod最小值560，单位595ns */
            pulse = usPeriod-1;                           	/* 设置低电平时间1.19us，注意usPeriod已经进行了减1操作 */
        }
        else	/* 大于4K的频率，无需分频 */
        {
            usPrescaler = 0;								/* 分频比 = 1 */
            usPeriod = uiTIMxCLK / _ulFreq - 1;				/* 自动重装的值， usPeriod最小值840，单位5.95ns */
            pulse = usPeriod - 199;              			/* 设置低电平时间1.19us，注意usPeriod已经进行了减1操作 */
        }
    }
    else
    {
        /* APB1 定时器 = 84M */
        uiTIMxCLK = SystemCoreClock / 2;

        if (_ulFreq < 100)
        {
            usPrescaler = 10000 - 1;							/* 分频比 = 10000 */
            usPeriod =  (uiTIMxCLK / 10000) / _ulFreq  - 1;		/* 自动重装的值， usPeriod最小值84, 单位119us */
            pulse = usPeriod;                               	/* 设置低电平时间119us，注意usPeriod已经进行了减1操作 */
        }
        else if (_ulFreq < 3000)
        {
            usPrescaler = 100 - 1;							/* 分频比 = 100 */
            usPeriod =  (uiTIMxCLK / 100) / _ulFreq  - 1;	/* 自动重装的值， usPeriod最小值280，单位1.19us */
            pulse = usPeriod;                           	/* 设置低电平时间1.19us，注意usPeriod已经进行了减1操作 */
        }
        else	/* 大于4K的频率，无需分频 */
        {
            usPrescaler = 0;								/* 分频比 = 1 */
            usPeriod = uiTIMxCLK / _ulFreq - 1;				/* 自动重装的值， usPeriod最小值420，单位11.9ns */
            pulse = usPeriod - 99;              			/* 设置低电平时间1.19us，注意usPeriod已经进行了减1操作 */
        }
    }

    /*  PWM频率 = TIMxCLK / usPrescaler + 1）/usPeriod + 1）*/
    TimHandle.Instance = TIMx;
    TimHandle.Init.Prescaler         = usPrescaler;         /* 用于设置定时器分频 */
    TimHandle.Init.Period            = usPeriod;            /* 用于设置定时器周期 */
    TimHandle.Init.ClockDivision     = 0;                   /* 用于指示定时器时钟 (CK_INT) 频率与死区发生器以及数字滤波器（ETR、 TIx）
	                                                           所使用的死区及采样时钟 (tDTS) 之间的分频比*/
    TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;  /* 用于设置计数模式，向上计数模式 */
    TimHandle.Init.RepetitionCounter = 0;                   /* 用于设置重复计数器，仅 TIM1 和 TIM8 有，其它定时器没有 */
    TimHandle.Init.AutoReloadPreload = 0;                   /* 用于设置定时器的 ARR 自动重装寄存器是更新事件产生时写入有效 */

    if (HAL_TIM_PWM_DeInit(&TimHandle) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK)
    {
        Error_Handler();
    }

    /* 配置定时器PWM输出通道 */
    sConfig.OCMode       = TIM_OCMODE_PWM1;         /* 配置输出比较模式 */
    sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;     /* 设置输出高电平有效 */
    sConfig.OCFastMode   = TIM_OCFAST_DISABLE;      /* 关闭快速输出模式 */
    sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;    /* 配置互补输出高电平有效 */
    sConfig.OCIdleState  = TIM_OCIDLESTATE_SET;     /* 空闲状态时，设置输出比较引脚为高电平 */
    sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;  /* 空闲状态时，设置互补输出比较引脚为低电平 */

    /* 占空比 */
    sConfig.Pulse = pulse;
    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, CONVST_TIMCH) != HAL_OK)
    {
        Error_Handler();
    }

    /* 使能定时器中断  */
    __HAL_TIM_ENABLE_DMA(&TimHandle, TIM_DMA_UPDATE);

    /* 启动PWM输出 */
    if (HAL_TIM_PWM_Start(&TimHandle, CONVST_TIMCH) != HAL_OK)
    {
        Error_Handler();
    }

    /* 定时器UP更新触发DMA传输 */
    TIMDMA.Instance                 = TIMx_UP_DMA_STREAM;      /* 例化使用的DMA数据流 */
    TIMDMA.Init.FIFOMode            = DMA_FIFOMODE_ENABLE;     /* 使能FIFO*/
    TIMDMA.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL; /* 用于设置阀值 */
    TIMDMA.Init.MemBurst            = DMA_MBURST_INC8;	       /* 用于存储器突发 */
    TIMDMA.Init.PeriphBurst         = DMA_PBURST_INC8;	       /* 用于外设突发 */
    TIMDMA.Init.Channel             = TIMx_UP_DMA_CHANNEL;     /* DMA通道 */
    TIMDMA.Init.Direction           = DMA_PERIPH_TO_MEMORY;    /* 传输方向是从外设到存储器 */
    TIMDMA.Init.PeriphInc           = DMA_PINC_DISABLE;        /* 外设地址自增禁止 */
    TIMDMA.Init.MemInc              = DMA_MINC_ENABLE;         /* 存储器地址自增使能 */
    TIMDMA.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; /* 外设数据传输位宽选择半字，即16bit */
    TIMDMA.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD; /* 存储器数据传输位宽选择半字，即16bit */
    TIMDMA.Init.Mode                = DMA_CIRCULAR; 		   /* 循环模式 */
    TIMDMA.Init.Priority            = DMA_PRIORITY_LOW;        /* 优先级低 */

    /* 复位DMA */
    if(HAL_DMA_DeInit(&TIMDMA) != HAL_OK)
    {
        /* DMA处于忙状态，直接终止DMA */
        if(HAL_DMA_Abort_IT(&TIMDMA) != HAL_OK)
        {
            Error_Handler();
        }
    }

    /* 初始化DMA */
    if(HAL_DMA_Init(&TIMDMA) != HAL_OK)
    {
        Error_Handler();
    }

    /* 关联DMA句柄到TIM */
    //__HAL_LINKDMA(&TimHandle, hdma[TIM_DMA_ID_UPDATE], TIMDMA);

    /* 配置DMA中断 */
    HAL_NVIC_SetPriority(TIMx_UP_DMA_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIMx_UP_DMA_IRQn);

    /* 注册半传输完成中断和传输完成中断 */
    HAL_DMA_RegisterCallback(&TIMDMA, HAL_DMA_XFER_CPLT_CB_ID, AD7606_DmaCplCb);
    HAL_DMA_RegisterCallback(&TIMDMA, HAL_DMA_XFER_HALFCPLT_CB_ID, AD7606_DmaHalfCplCb);

    /* 启动DMA传输 */
    HAL_DMA_Start_IT(&TIMDMA, (uint32_t)AD7606_BASE, (uint32_t)g_sAd7606Buf, AD7606_BUFSIZE);
}
