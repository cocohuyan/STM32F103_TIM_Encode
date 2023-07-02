#ifndef   __Eeprom_H
#define   __Eeprom_H    1


#include "stm32f10x.h"//加入头文件

                                                    //通讯端口配置
#define            CLR_SCL2()             GPIO_ResetBits(GPIOB, GPIO_Pin_6)//PTM_PTM3=0

#define            SET_SCL2()             GPIO_SetBits(GPIOB, GPIO_Pin_6)//PTM_PTM3=1

#define            CLR_SDA2()             GPIO_ResetBits(GPIOB, GPIO_Pin_7)//DDRM_DDRM4=1;PTM_PTM4=0

#define            SET_SDA2()             GPIO_SetBits(GPIOB, GPIO_Pin_7)//DDRM_DDRM4=1;PTM_PTM4=1

#define            READ_SDA2()            GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7)//PTIM_PTIM4==1



void I2cStart2(void);

void I2cStop2(void);

void DelayIntr2(unsigned char Time_CLK);

unsigned char WaitAck2(void);

void SendAck2(void);

void SendNotAck2(void);

void I2cSend2(unsigned char ucData2);

//unsigned char I2cReceive2(void);

unsigned char I2cReceive2(unsigned char ACK);

unsigned char I2CReadByte(unsigned int mem_addr,unsigned char *DATAp,unsigned int count);

unsigned char I2CWriteByte(unsigned int mem_addr,unsigned char*DDATAp,unsigned int count);

#endif

