 /**********************************************************************************************
      AD转换C文件
 **********************************************************************************************/
#include "BSP_Config.H"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

/*******************************************************************************
                            初始化通用IO口
********************************************************************************/
void GPIO_Configuration(void)
{

    EXTI_InitTypeDef EXTI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB| RCC_APB2Periph_GPIOC| RCC_APB2Periph_GPIOD, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;//初始化LED0
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    //PA0初始化 作为外部中断EXTI0
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_Init(&EXTI_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource2);

    EXTI_InitStructure.EXTI_Line = EXTI_Line2;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_Init(&EXTI_InitStructure);
}

/*******************************************************************************
    使能或者失能AHB外设时钟
********************************************************************************/
void RCC_Configuration(void)
{

/* Enable peripheral clocks --------------------------------------------------*/
    /* Enable DMA1 clock */
    // RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /* Enable  clock */
    // RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);      //使能串口1的时钟

    /* TIM2 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);        //使能定时器2的时钟

    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
    //                      RCC_APB2Periph_GPIOC , DISABLE);
}


/**
  * @brief  Configures the nested vectored interrupt controller.
  * @param  None
  * @retval : None
  */

void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

#ifdef  VECT_TAB_RAM
    /* Set the Vector Table base location at 0x20000000 */
    NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  /* VECT_TAB_FLASH  */
    /* Set the Vector Table base location at 0x08000000 */
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif

    /* Configure the NVIC Preemption Priority Bits */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);//设置优先级分组：先占优先级和从优先级

    /* Enable the USART1 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);


    /* Enable the TIM2 gloabal Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);
}

unsigned short int CCR1_Val = 49152;    //0xC000
unsigned short int CCR2_Val = 32768;    //0x8000
//unsigned short int CCR3_Val = 16384;    //0x4000
//unsigned short int CCR4_Val = 8192;    //0x2000


/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/

/**
  * @brief  This function handles TIM2 global interrupt request.
  * @param  None
  * @retval : None
  */
void TIM2_Config(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

 /* ---------------------------------------------------------------
    TIM2 Configuration: Output Compare Timing Mode:
    TIM2CLK = 36 MHz, Prescaler = 4, TIM2 counter clock = 7.2 MHz
    CC1 update rate = TIM2 counter clock / CCR1_Val = 146.48 Hz
    CC2 update rate = TIM2 counter clock / CCR2_Val = 219.7 Hz
    CC3 update rate = TIM2 counter clock / CCR3_Val = 439.4 Hz
    CC4 update rate = TIM2 counter clock / CCR4_Val =  878.9 Hz
  --------------------------------------------------------------- */

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 65535;//TIM_Period设置了在下一个更新事件装入活动的自动重装载寄存器周期的值
    TIM_TimeBaseStructure.TIM_Prescaler = 0;//TIM_Prescaler设置了用来作为TIMx时钟频率除数的预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; //TIM_ClockDivision设置了时钟分割
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//TIM向上计数模式

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    /* Prescaler configuration */
    TIM_PrescalerConfig(TIM2, 4, TIM_PSCReloadMode_Immediate);//设置TIMx预分频:TIM预分频值即时装入

    /* Output Compare Timing Mode configuration: Channel1 */

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing; //TIM输出比较时间模式
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;    //
    TIM_OCInitStructure.TIM_Pulse = CCR1_Val;//TIM_Pulse设置了待装入捕获比较寄存器的脉冲值
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//TIM输出比较极性高

    TIM_OC1Init(TIM2, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);

    /* Output Compare Timing Mode configuration: Channel2 */
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = CCR2_Val;

    TIM_OC2Init(TIM2, &TIM_OCInitStructure);

    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Disable);

  /* Output Compare Timing Mode configuration: Channel3 */
 // TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
 // TIM_OCInitStructure.TIM_Pulse = CCR3_Val;

 // TIM_OC3Init(TIM2, &TIM_OCInitStructure);

 // TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Disable);

  /* Output Compare Timing Mode configuration: Channel4 */
 // TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_Pulse = CCR4_Val;

 // TIM_OC4Init(TIM2, &TIM_OCInitStructure);

 // TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Disable);//使能或者失能TIMx在CCR4上的预装载寄存器

    /* TIM IT enable */
    TIM_ITConfig(TIM2, TIM_IT_CC1 | TIM_IT_CC2 , ENABLE);
    //TIM_ITConfig(TIM2,TIM_IT_Update, ENABLE);
    /* TIM2 enable counter */
    TIM_Cmd(TIM2, ENABLE);//使能或者失能TIMx外设
}

/*******************************************************************************
* Function Name  : HW_SPI_Init
* Description    : Initializes the peripherals used by the SPI FLASH driver.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI1_Init(void)
{
    SPI_InitTypeDef  SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable SPI1 and GPIOA clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | RCC_APB2Periph_GPIOA, ENABLE);

    /* Configure SPI1 pins: NSS, SCK, MISO and MOSI */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure PA.4 as Output push-pull, used as Flash Chip select */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Deselect the FLASH: Chip Select high */
//  SPI_FLASH_CS_HIGH();

    /* SPI1 configuration */
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //SPI设置为双线双向全双工
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;     //设置为主SPI
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;    //SPI发送接收8位帧结构
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; //时钟悬空高
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;//数据捕获于第二个时钟沿
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;    //内部NSS信号有SSI位控制
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;//波特率预分频值为4
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;//数据传输从MSB位开始
    SPI_InitStructure.SPI_CRCPolynomial = 7; //SPI_CRCPolynomial定义了用于CRC值计算的多项式
    SPI_Init(SPI1, &SPI_InitStructure);

    /* Enable SPI1  */
    SPI_Cmd(SPI1, ENABLE);
    //  SPI_FLASH_CS_LOW();
}


void SPI2_Init(void)           //硬件SPI2口初始化
{
    SPI_InitTypeDef  SPI_InitStructure;

    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable SPI2 and GPIOB clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE) ;//使能SPI2的时钟

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//使能IO口时钟

    /* Configure SPI2 pins: SCK, MISO and MOSI */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//复用推挽输出
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Configure PB.12 as Output push-pull, used as Flash Chip select */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//推挽输出
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_ResetBits(GPIOB, GPIO_Pin_12);//片选信号为低

    /* SPI1 configuration */
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //SPI设置为双线双向全双工
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;     //设置为主SPI
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;    //SPI发送接收8位帧结构
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; //时钟悬空高
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;//数据捕获于第二个时钟沿
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;    //内部NSS信号有SSI位控制
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;//波特率预分频值为2
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;//数据传输从MSB位开始
    SPI_InitStructure.SPI_CRCPolynomial = 7; //SPI_CRCPolynomial定义了用于CRC值计算的多项式
    SPI_Init(SPI2, &SPI_InitStructure);

    /* Enable SPI2  */
    SPI_Cmd(SPI2, ENABLE);
}

/*************************************************
函数功能：硬件SPI口发送或者接收一个字节数据

*************************************************/

//SPIx 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
unsigned char SPI1_ReadWriteByte(unsigned char TxData)
{
    unsigned int retry=0;
    while((SPI1->SR&1<<1)==0)//等待发送区空
    {
        retry++;
        if(retry>2000)return 0;
    }
    SPI1->DR=TxData;           //发送一个byte
    retry=0;
    while((SPI1->SR&1<<0)==0) //等待接收完一个byte
    {
        retry++;
        if(retry>2000)return 0;
    }
    return SPI1->DR;          //返回收到的数据
}

/*************************************************
函数功能：硬件SPI口发送或者接收一个字节数据

*************************************************/

//SPIx 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
unsigned char SPI2_ReadWriteByte(unsigned char TxData)
{
    unsigned int retry=0;
    while((SPI2->SR&1<<1)==0)//等待发送区空
    {
        retry++;
        if(retry>2000)return 0;
    }
    SPI2->DR=TxData;           //发送一个byte
    retry=0;
    while((SPI2->SR&1<<0)==0) //等待接收完一个byte
    {
        retry++;
        if(retry>2000)return 0;
    }
    return SPI2->DR;          //返回收到的数据
}






