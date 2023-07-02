 /******************************************************************************************
        ;;文件名称: send_DATA.c
        ;;文件概述: 数据发送函数
        ;;作    者:
        ;;编写日期: 2010年9月23日
        ;;版 本 号: V1.0
        ;;变更记录:
******************************************************************************************/
#include "send_DATA.H"

void DATA_send (unsigned char *DATA_P,unsigned int nCount)
{
    unsigned int i;

    for(i=0;i<nCount;i++)//启动数据转发程序
    {
        /* Check if the transmit data register is full or not */
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); //检查发送寄存器是否为空
        USART_SendData(USART1, *DATA_P);
        *DATA_P++;
    }
}

/**********************************初始化缓冲区为0x0或者0xFF**********************************/
void Init_Buffer (unsigned char *P,unsigned int Count,unsigned char Type)
{
  unsigned int i;

   if(Type==0)
   {
     for(i=0;i<Count;i++)*P++=0x0;
   }
   else
   {
     for(i=0;i<Count;i++)*P++=0xFF;
   }
}



/**************************************************************************************
  函数功能：内存拷贝函数
**************************************************************************************/
void Copy_Mem (unsigned char *P1, const unsigned char *P2,unsigned int Count)
{
   unsigned int i ;
   for(i=0;i<Count;i++)*P1++=*P2++;
}


/**************************************************************************************
  函数功能：内存拷贝函数

**************************************************************************************/
void Copy_Char (unsigned char *P1, unsigned char *P2,unsigned int Count)
{
   unsigned int i ;
   for(i=0;i<Count;i++)*P1++=*P2++;
}




/**************************************************************************************
  函数功能：内存比较函数函数
  比较指针指向的两个缓冲区的数据是否一样
  一样返回值为：  0
  不一样返回值为：1

***************************************************************************************/
unsigned char Compare_Mem (unsigned char *P1,unsigned char *P2,unsigned int Count)
{

   unsigned int i ;

   for(i=0;i<Count;i++)
   {

     if(*P1==*P2)
     {

       P1++;

       P2++;

     }
     else
     {
       return 1;

     }

   };
    return 0;

}



/*************************************************************************************

  接收字库文件的时候，回复上位机软件的回复命令

**************************************************************************************/
void Send_Start(unsigned char baohao1,unsigned char baohao2)//回复上位机
{
    unsigned char state_buffer[10];      //应答主机状态寄存器

    state_buffer[0]=0x01;
    state_buffer[1]=0x05;
    state_buffer[2]=baohao1;
    state_buffer[3]=baohao2;
    AH_CRC_Verify (state_buffer,state_buffer[1]-1, 1 );
    DATA_send (state_buffer,state_buffer[1]+1);
}










