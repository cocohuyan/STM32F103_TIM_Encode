#ifndef   __send_DATA_H
#define   __send_DATA_H     1

/******************************************************************
包含文件
*****************************************************************/
#include "stm32f10x.h"//加入头文件

/********************************************************************


    DATA_P:要进行发送的数据的首地址
    nCount:要进行发送的数据的长度

 *******************************************************************/

void DATA_send (unsigned char *DATA_P,unsigned int nCount);

void Init_Buffer (unsigned char *P,unsigned int Count,unsigned char Type);//初始化缓冲区为0x0或者0xFF

void Copy_Mem (unsigned char *P1,const unsigned char *P2,unsigned int Count);//函数功能：内存拷贝函数


void Copy_Char (unsigned char *P1, unsigned char *P2,unsigned int Count);//函数功能：按字节内存拷贝函数

unsigned char Compare_Mem (unsigned char *P1,unsigned char *P2,unsigned int Count);//函数功能：内存比较函数函数

void Send_Start(unsigned char baohao1,unsigned char baohao2);//回复上位机


#endif

