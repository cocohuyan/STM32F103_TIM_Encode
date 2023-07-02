
/*****************************************************************************************************


     汉字字库查询字符的处理方式



*****************************************************************************************************/

#include "Search_HZ.H"

//extern unsigned char DF_buffer[528];

unsigned char SP_DATA[72];//取一个字符的缓冲区

/*************按照区位码查询方式来查询字符所在缓冲区的指针地址 *************************************/

/*************************************************************************************************
 函数名：Char_Addr
 输入参数：
           unsigned char qh,             字符的位置偏移的区
           unsigned char wh,             字符的位置偏移的位
           unsigned char *Data_char      点阵字符的返回缓冲区指针

		   // ADDRESS=((qh-0xa1-0x0f)*94+(wh-0xa1))*72;  //24点阵在数据中的地址

***************************************************************************************************/



void Char_Addr(unsigned char qh,unsigned char wh)//输入参数为汉字的区位内码
 {

   unsigned int ADDRESS;
   unsigned int ADDRESS1;
   unsigned int ADDRESS2;
   unsigned char ADDRESS3;
   unsigned char ADDRESS4;

   ADDRESS3=qh-0xa1; //-0x0f当使用DOS字库的时候需要减去0xf;//一下运算是将上面运算分解计算，防止编译器出错

   ADDRESS4=wh-0xa1;

   ADDRESS2=(unsigned int)ADDRESS3*94;

   ADDRESS1=ADDRESS2+(unsigned int)ADDRESS4;

   ADDRESS=(ADDRESS1*72)+0x3000;//0x3000之前的地址存放的为ASC码位置

   SPI_Flash_Read(SP_DATA,ADDRESS,72);//读取SPI FLASH 在指定地址开始读取指定长度的数据

 }










/*************************************ASC顺序查找点阵字符*************************************************/

/*********************************************************************************************************
 函数名：ASC_Addr
 函数功能：读取ASC码点阵字符

 输入参数：
           unsigned char qh,             字符的位置偏移的区
           unsigned char wh,             字符的位置偏移的位
           unsigned char *Data_char      点阵字符的返回缓冲区指针
***************************************************************************************************/



void ASC_Addr(unsigned char QW)
 {

   unsigned int ADDRESS;

   ADDRESS=(unsigned int)QW*48;  //16*24点阵ASCII在数据中的地址

   SPI_Flash_Read(SP_DATA,ADDRESS,48);//读取SPI FLASH 在指定地址开始读取指定长度的数据


 }

