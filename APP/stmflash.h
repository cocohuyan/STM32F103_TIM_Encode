#ifndef __STMFLASH_H__
#define __STMFLASH_H__
#include <stdint.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////
//用户根据自己的需要设置
#define STM32_FLASH_SIZE 32               //所选STM32的FLASH容量大小(单位为K)  0x0801 0000   0x0800 FC00
#define STM32_FLASH_WREN 1                //使能FLASH写入(0，不是能;1，使能)
//////////////////////////////////////////////////////////////////////////////////////////////////////

//FLASH起始地址
#define STM32_FLASH_BASE 0x08000000          //STM32 FLASH的起始地址
#define DATAFLASH_SAVE_ADDR 0x08007C00      //片内Flash 存放数据起始地址
//FLASH解锁键值

uint16_t STMFLASH_ReadHalfWord(uint32_t faddr);          //读出半字
void STMFLASH_WriteLenByte(uint32_t WriteAddr, uint32_t DataToWrite, uint16_t Len);    //指定地址开始写入指定长度的数据
uint32_t STMFLASH_ReadLenByte(uint32_t ReadAddr, uint16_t Len);                        //指定地址开始读取指定长度数据
void STMFLASH_Write(uint32_t WriteAddr, uint16_t *pBuffer, uint16_t NumToWrite);        //从指定地址开始写入指定长度的数据
void STMFLASH_Read(uint32_t ReadAddr, uint16_t *pBuffer, uint16_t NumToRead);           //从指定地址开始读出指定长度的数据
void BoardConfigParaSave(void);
void BoardConfigParaRead(void);

#endif /* __STMFLASH_H__ */

















