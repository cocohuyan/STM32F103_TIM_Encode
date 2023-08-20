#include <stdint.h>
#include "stmflash.h"
#include "stm32f10x.h"
#include "stm32f10x_flash.h"
#include "motor_control.h"

#if (STM32_FLASH_SIZE < 256)
#define STM_SECTOR_SIZE    1024 //字节
#else
#define STM_SECTOR_SIZE    2048
#endif

#define DATAFLASH_SAVE_SIZE     32          //保存在片内Flash中的数据大小

uint16_t STMFLASH_BUF[STM_SECTOR_SIZE / 2];//最多是2K字节

//读取指定地址的半字(16位数据)
//faddr:读地址(此地址必须为2的倍数!!)
//返回值:对应数据.
uint16_t STMFLASH_ReadHalfWord(uint32_t faddr)
{
    return *(volatile uint16_t*)faddr;
}

//不检查的写入
//WriteAddr:起始地址
//pBuffer:数据指针
//NumToWrite:半字(16位)数
void STMFLASH_Write_NoCheck(uint32_t WriteAddr,uint16_t *pBuffer,uint16_t NumToWrite)
{
    uint16_t i;
    for (i = 0; i < NumToWrite; i++) {
        FLASH_ProgramHalfWord(WriteAddr, pBuffer[i]);
        WriteAddr += 2;//地址增加2.
    }
}

//从指定地址开始写入指定长度的数据
//WriteAddr:起始地址(此地址必须为2的倍数!!)
//pBuffer:数据指针
//NumToWrite:半字(16位)数(就是要写入的16位数据的个数.)
void STMFLASH_Write(uint32_t WriteAddr, uint16_t *pBuffer, uint16_t NumToWrite)
{
    uint32_t secpos;       //扇区地址
    uint16_t secoff;       //扇区内偏移地址(16位字计算)
    uint16_t secremain;    //扇区内剩余地址(16位字计算)
    uint16_t i;
    uint32_t offaddr;      //去掉0x08000000后的地址
    if (WriteAddr < STM32_FLASH_BASE || (WriteAddr >= (STM32_FLASH_BASE + 1024 * STM32_FLASH_SIZE))) {
        return;            //非法地址
    }

    FLASH_Unlock();                            //解锁
    offaddr = WriteAddr - STM32_FLASH_BASE;       //实际偏移地址.
    secpos  = offaddr / STM_SECTOR_SIZE;          //扇区地址  0~127 for STM32F103RBT6
    secoff  = (offaddr % STM_SECTOR_SIZE) / 2;    //在扇区内的偏移(2个字节为基本单位.)
    secremain = STM_SECTOR_SIZE / 2 - secoff;     //扇区剩余空间大小
    if (NumToWrite <= secremain) {
        secremain = NumToWrite;                   //不大于该扇区范围
    }

    while(1) {
        STMFLASH_Read(secpos*STM_SECTOR_SIZE + STM32_FLASH_BASE, STMFLASH_BUF, STM_SECTOR_SIZE / 2);//读出整个扇区的内容
        for(i = 0; i < secremain; i++)//校验数据
        {
            if (STMFLASH_BUF[secoff + i] != 0xFFFF) {
                break;//需要擦除
            }
        }

        if (i < secremain) {//需要擦除
            FLASH_ErasePage(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE);//擦除这个扇区
            for (i = 0; i < secremain; i++) {//复制
                STMFLASH_BUF[i+secoff] = pBuffer[i];
            }
            STMFLASH_Write_NoCheck(secpos * STM_SECTOR_SIZE + STM32_FLASH_BASE, STMFLASH_BUF, STM_SECTOR_SIZE / 2);//写入整个扇区
        } else {
            STMFLASH_Write_NoCheck(WriteAddr, pBuffer, secremain);   //写已经擦除了的,直接写入扇区剩余区间.
        }

        if (NumToWrite == secremain) {  //写入结束了
            break;
        } else {                        //写入未结束
            secpos++;                   //扇区地址增1
            secoff = 0;                 //偏移位置为0
            pBuffer   += secremain;     //指针偏移
            WriteAddr += secremain;     //写地址偏移
            NumToWrite -= secremain;    //字节(16位)数递减
            if (NumToWrite > (STM_SECTOR_SIZE / 2)) {
                secremain = STM_SECTOR_SIZE / 2; //下一个扇区还是写不完
            } else {
                secremain = NumToWrite;   //下一个扇区可以写完了
            }
        }
    }
    FLASH_Lock(); //上锁
}

//从指定地址开始读出指定长度的数据
//ReadAddr:起始地址
//pBuffer:数据指针
//NumToWrite:半字(16位)数
void STMFLASH_Read(uint32_t ReadAddr, uint16_t *pBuffer, uint16_t NumToRead)
{
    uint16_t i;
    for (i = 0; i < NumToRead; i++) {
        pBuffer[i] = STMFLASH_ReadHalfWord(ReadAddr);//读取2个字节.
        ReadAddr += 2;//偏移2个字节.
    }
}

/**
 * @brief read board parameters from flash
 *        数据格式: 第一字节为 $ 第二三字节 offset
 * @retval void
*/
void BoardConfigParaRead(void)
{
    uint8_t tempData[DATAFLASH_SAVE_SIZE] = {0};
    uint16_t i;
    int16_t offset;

    STMFLASH_Read(DATAFLASH_SAVE_ADDR, (uint16_t *)tempData, DATAFLASH_SAVE_SIZE / 2);

    for (i = 0; i < DATAFLASH_SAVE_SIZE; i++) {
        if (tempData[i] != 0xFF) {
            break;
        }
    }

    // Flash中未设置有效的参数; 参数按照默认设置
    if (i == DATAFLASH_SAVE_SIZE) {
        MotorControlSetAngleOffset(0, SET_UPDAE_FLASH);
        return;
    }

    // Flash 中存有数据, 但数据格式不正确 参数按照默认设置
    if (tempData[0] != '$') {
        MotorControlSetAngleOffset(0, SET_UPDAE_FLASH);
        return;
    }

    offset = tempData[1]<<8 | tempData[2];
    if (offset > 18000 || offset < -18000) {
        printf("BoardConfigParaRead offset err! %d", offset);
        return;
    }
    MotorControlSetAngleOffset(offset, SET_FROM_FLASH);
}

void BoardConfigParaSave(void)
{
    uint8_t tempData[DATAFLASH_SAVE_SIZE] = {0};
    uint16_t j = 0;
    uint8_t checksum;
    Controller_t *saveData;
    saveData = MotorGetControlPara();

    tempData[j++] = '$';
    tempData[j++] = (uint8_t)(saveData->deltaAngle >> 8);
    tempData[j++] = (uint8_t)(saveData->deltaAngle & 0x00FF);
    memset(tempData + j, 0, 3 * sizeof(double)); // sizeof(double) == 8
    j += 3 * sizeof(double); // 3个 kp ki kd 参数;
    tempData[j++] = '*';
    tempData[j++] = checksum;
    STMFLASH_Write(DATAFLASH_SAVE_ADDR, (uint16_t *)tempData, DATAFLASH_SAVE_SIZE / 2);
}
