
/*************************************************************************
SPI驱动程序

//4Kbytes为一个Sector
//16个扇区为1个Block
//W25X16
//容量为2M字节,共有32个Block,512个Sector
*************************************************************************/
#include "SPI_FLASH.H"

/*************************************************
函数功能：硬件SPI口发送或者接收一个字节数据

*************************************************/

//SPIx 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
unsigned char SPIx_ReadWriteByte(unsigned char TxData)
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


//读取SPI_FLASH的状态寄存器
//BIT7  6   5   4   3   2   1   0
//SPR   RV  TB BP2 BP1 BP0 WEL BUSY
//SPR:默认0,状态寄存器保护位,配合WP使用
//TB,BP2,BP1,BP0:FLASH区域写保护设置
//WEL:写使能锁定
//BUSY:忙标记位(1,忙;0,空闲)
//默认:0x00
unsigned char SPI_Flash_ReadSR(void)
{
    unsigned char byte=0;
    SPI_FLASH_CS_LOW();                            //使能器件
    SPIx_ReadWriteByte(W25X_ReadStatusReg);    //发送读取状态寄存器命令
    byte=SPIx_ReadWriteByte(0Xff);             //读取一个字节
    SPI_FLASH_CS_HIGH();                            //取消片选
    return byte;
}
//写SPI_FLASH状态寄存器
//只有SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)可以写!!!
void SPI_FLASH_Write_SR(unsigned char sr)
{
    SPI_FLASH_CS_LOW();                            //使能器件
    SPIx_ReadWriteByte(W25X_WriteStatusReg);   //发送写取状态寄存器命令
    SPIx_ReadWriteByte(sr);               //写入一个字节
    SPI_FLASH_CS_HIGH();                            //取消片选
}

//SPI_FLASH写使能
//将WEL置位
void SPI_FLASH_Write_Enable(void)
{
    SPI_FLASH_CS_LOW();                            //使能器件
    SPIx_ReadWriteByte(W25X_WriteEnable);      //发送写使能
    SPI_FLASH_CS_HIGH();                            //取消片选
}

//SPI_FLASH写禁止
//将WEL清零
void SPI_FLASH_Write_Disable(void)
{
    SPI_FLASH_CS_LOW();                            //使能器件
    SPIx_ReadWriteByte(W25X_WriteDisable);     //发送写禁止指令
    SPI_FLASH_CS_HIGH();                            //取消片选
}

//读取芯片ID W25X16的ID:0XEF14
unsigned short int SPI_Flash_ReadID(void)
{
    unsigned short int Temp = 0;
    SPI_FLASH_CS_LOW();
    SPIx_ReadWriteByte(0x90);//发送读取ID命令
    SPIx_ReadWriteByte(0x00);
    SPIx_ReadWriteByte(0x00);
    SPIx_ReadWriteByte(0x00);
    Temp|=SPIx_ReadWriteByte(0xFF)<<8;
    Temp|=SPIx_ReadWriteByte(0xFF);
    SPI_FLASH_CS_HIGH();
    return Temp;
}

unsigned int  SPI_Flash_ReadDeviceID(void)          //读取FLASH ID
{
    unsigned int Temp = 0;
    SPI_FLASH_CS_LOW();
    SPIx_ReadWriteByte(0x9F);//发送读取ID命令
    //SPIx_ReadWriteByte(0x00);
    //SPIx_ReadWriteByte(0x00);
    //SPIx_ReadWriteByte(0x00);
    Temp|=SPIx_ReadWriteByte(0xFF)<<16;
    Temp|=SPIx_ReadWriteByte(0xFF)<<8;
    Temp|=SPIx_ReadWriteByte(0xFF);
    SPI_FLASH_CS_HIGH();

    return Temp;
}


//读取SPI FLASH
//在指定地址开始读取指定长度的数据
//pBuffer:数据存储区
//ReadAddr:开始读取的地址(24bit)
//NumByteToRead:要读取的字节数(最大65535)
void SPI_Flash_Read(unsigned char* pBuffer,unsigned int ReadAddr,unsigned short int NumByteToRead)//读取SPI FLASH 在指定地址开始读取指定长度的数据
{
     unsigned short int i;
    SPI_FLASH_CS_LOW();                                        //使能器件
    SPIx_ReadWriteByte(W25X_ReadData);                         //发送读取命令
    SPIx_ReadWriteByte((unsigned char)((ReadAddr)>>16));    //发送24bit地址
    SPIx_ReadWriteByte((unsigned char)((ReadAddr)>>8));
    SPIx_ReadWriteByte((unsigned char)ReadAddr);
    for(i=0;i<NumByteToRead;i++)
    {
        pBuffer[i]=SPIx_ReadWriteByte(0XFF);                   //循环读数
    }
    SPI_FLASH_CS_HIGH();                                    //取消片选
}

//SPI在一页(0~65535)内写入少于256个字节的数据
//在指定地址开始写入最大256字节的数据
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)
//NumByteToWrite:要写入的字节数(最大256),该数不应该超过该页的剩余字节数!!!
void SPI_Flash_Write_Page(unsigned char* pBuffer,unsigned int WriteAddr,unsigned short int NumByteToWrite)//SPI在一页(0~65535)内写入少于256个字节的数据
{
     unsigned short int i;
    SPI_FLASH_Write_Enable();                                      //SET WEL
    SPI_FLASH_CS_LOW();                                            //使能器件
    SPIx_ReadWriteByte(W25X_PageProgram);                          //发送写页命令
    SPIx_ReadWriteByte((unsigned char)((WriteAddr)>>16));         //发送24bit地址
    SPIx_ReadWriteByte((unsigned char)((WriteAddr)>>8));
    SPIx_ReadWriteByte((unsigned char)WriteAddr);
    for(i=0;i<NumByteToWrite;i++)SPIx_ReadWriteByte(pBuffer[i]);//循环写数
    SPI_FLASH_CS_HIGH();                                        //取消片选
    SPI_Flash_Wait_Busy();                                           //等待写入结束
}

//无检验写SPI FLASH
//必须确保所写的地址范围内的数据全部为0XFF,否则在非0XFF处写入的数据将失败!
//具有自动换页功能
//在指定地址开始写入指定长度的数据,但是要确保地址不越界!
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)
//NumByteToWrite:要写入的字节数(最大65535)
//CHECK OK
void SPI_Flash_Write_NoCheck(unsigned char* pBuffer,unsigned int WriteAddr,unsigned short int NumByteToWrite)
{
    unsigned short int pageremain;
    pageremain=256-WriteAddr%256; //单页剩余的字节数
    if(NumByteToWrite<=pageremain)pageremain=NumByteToWrite;//不大于256个字节
    while(1)
    {
        SPI_Flash_Write_Page(pBuffer,WriteAddr,pageremain);
        if(NumByteToWrite==pageremain)break;//写入结束了
         else //NumByteToWrite>pageremain
        {
            pBuffer+=pageremain;
            WriteAddr+=pageremain;

            NumByteToWrite-=pageremain;              //减去已经写入了的字节数
            if(NumByteToWrite>256)pageremain=256; //一次可以写入256个字节
            else pageremain=NumByteToWrite;       //不够256个字节了
        }
    };
}

//写SPI FLASH
//在指定地址开始写入指定长度的数据
//该函数带擦除操作!
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)
//NumByteToWrite:要写入的字节数(最大65535)

unsigned char SPI_FLASH_BUF[4096];

void SPI_Flash_Write(unsigned char* pBuffer,unsigned int WriteAddr,unsigned short int NumByteToWrite)
{
    unsigned int secpos;
    unsigned short int secoff;
    unsigned short int secremain;
     unsigned short int i;



    secpos=WriteAddr/4096;//扇区地址 0~511 for w25x16
    secoff=WriteAddr%4096;//在扇区内的偏移
    secremain=4096-secoff;//扇区剩余空间大小

    if(NumByteToWrite<=secremain)secremain=NumByteToWrite;//不大于4096个字节
    while(1)
    {
        SPI_Flash_Read(SPI_FLASH_BUF,secpos*4096,4096);//读出整个扇区的内容
        for(i=0;i<secremain;i++)//校验数据
        {
            if(SPI_FLASH_BUF[secoff+i]!=0XFF)break;//需要擦除
        }
        if(i<secremain)//需要擦除
        {
            SPI_Flash_Erase_Sector(secpos);//擦除这个扇区
            for(i=0;i<secremain;i++)       //复制
            {
                SPI_FLASH_BUF[i+secoff]=pBuffer[i];
            }
            SPI_Flash_Write_NoCheck(SPI_FLASH_BUF,secpos*4096,4096);//写入整个扇区

        }else SPI_Flash_Write_NoCheck(pBuffer,WriteAddr,secremain);//写已经擦除了的,直接写入扇区剩余区间.
        if(NumByteToWrite==secremain)break;//写入结束了
        else//写入未结束
        {
            secpos++;//扇区地址增1
            secoff=0;//偏移位置为0

               pBuffer+=secremain;  //指针偏移
            WriteAddr+=secremain;//写地址偏移
               NumByteToWrite-=secremain;                //字节数递减
            if(NumByteToWrite>4096)secremain=4096;    //下一个扇区还是写不完
            else secremain=NumByteToWrite;            //下一个扇区可以写完了
        }
    };
}
//擦除整个芯片
//整片擦除时间:
//W25X16:25s
//W25X32:40s
//W25X64:40s
//等待时间超长...
void SPI_Flash_Erase_Chip(void)
{
    SPI_FLASH_Write_Enable();                  //SET WEL
    SPI_Flash_Wait_Busy();
      SPI_FLASH_CS_LOW();                        //使能器件
    SPIx_ReadWriteByte(W25X_ChipErase);        //发送片擦除命令
    SPI_FLASH_CS_HIGH();                       //取消片选
    SPI_Flash_Wait_Busy();                      //等待芯片擦除结束
}

//擦除一个扇区
//Dst_Addr:扇区地址 0~511 for w25x16
//擦除一个扇区的最少时间:150ms
void SPI_Flash_Erase_Sector(unsigned int Dst_Addr)
{
    Dst_Addr*=4096;
    SPI_FLASH_Write_Enable();                  //SET WEL
    SPI_Flash_Wait_Busy();
      SPI_FLASH_CS_LOW();                            //使能器件
    SPIx_ReadWriteByte(W25X_SectorErase);      //发送扇区擦除指令
    SPIx_ReadWriteByte((unsigned char)((Dst_Addr)>>16));  //发送24bit地址
    SPIx_ReadWriteByte((unsigned char)((Dst_Addr)>>8));
    SPIx_ReadWriteByte((unsigned char)Dst_Addr);
    SPI_FLASH_CS_HIGH();                            //取消片选
    SPI_Flash_Wait_Busy();                      //等待擦除完成
}

//等待空闲
void SPI_Flash_Wait_Busy(void)
{
    while ((SPI_Flash_ReadSR()&0x01)==0x01);   // 等待BUSY位清空
}


//进入掉电模式
void SPI_Flash_PowerDown(void)
{
      SPI_FLASH_CS_LOW();                            //使能器件
    SPIx_ReadWriteByte(W25X_PowerDown);        //发送掉电命令
    SPI_FLASH_CS_HIGH();                            //取消片选
    delay_ms(1);                               //等待TPD
}
//唤醒
void SPI_Flash_WAKEUP(void)
{
    SPI_FLASH_CS_LOW();                            //使能器件
    SPIx_ReadWriteByte(W25X_ReleasePowerDown);   //  send W25X_PowerDown command 0xAB
    SPI_FLASH_CS_HIGH();                            //取消片选
    delay_ms(1);                               //等待TRES1
}


//使用SysTick的普通计数模式对延迟进行管理
//包括delay_us,delay_ms
//2010/5/27
static unsigned int TimingDelay;

void SysTick_init(void)
{

    SysTick_Config(SystemFrequency / 1000);

}

 /**
  * @brief  Inserts a delay time.
  * @param nTime: specifies the delay time length, in milliseconds.
  * @retval : None
  */
void delay_ms(unsigned int nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval : None
  */
void TimingDelay_Decrement(void)
{
    if (TimingDelay != 0x00)
    {
        TimingDelay--;
    }
}



extern unsigned  int       ID;

extern unsigned char TxBuffer[10];//串口发送缓冲区
extern unsigned char RxBuffer[1000]; //串口接收缓冲区

//请不要调该函数，该函数容易使打印机的字库损坏导致无法打印出字符























