#include "AD9834.h"

/****************************************************************
函数名称: AD9834_Write_16Bits
功    能: 向AD9834写入16为数据
参    数: data --  要写入的16位数据
返回值  : 无
*****************************************************************/
void AD9834_Write_16Bits(unsigned int data)
{
    AD9834_FSYNC_CLR;
    uint8_t LByte = data & 0xff;
    uint8_t HByte = (data >> 8) & 0xff;
    HAL_SPI_Transmit(&hspi2, &HByte, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi2, &LByte, 1, HAL_MAX_DELAY);    
    AD9834_FSYNC_SET;
}
/***********************************************************************************
函数名称：AD9834_Select_Wave
功    能：软件为控制，
    --------------------------------------------------
    IOUT正弦波 ，SIGNBITOUT方波 ，写FREQREG0 ，写PHASE0
    ad9834_write_16bit(0x2028)   一次性写FREQREG0
    ad9834_write_16bit(0x0038)   单独改写FREQREG0的LSB
    ad9834_write_16bit(0x1038)   单独改写FREQREG0的MSB
    --------------------------------------------------
    IOUT三角波 ，写PHASE0
    ad9834_write_16bit(0x2002)   一次性写FREQREG0
    ad9834_write_16bit(0x0002)   单独改写FREQREG0的LSB
    ad9834_write_16bit(0x1008)   单独改写FREQREG0的MSB
参    数：initdata -- 要输入的命令
返回值  ：无
************************************************************************************/
void AD9834_Select_Wave(unsigned int initdata)
{
    AD9834_FSYNC_SET;

    AD9834_RESET_SET;

    AD9834_Write_16Bits(initdata);
}
/****************************************************************
函数名称: AD9834_Set_Freq
功    能: 设置频率值
参    数: freq_number -- 要写入的平率寄存器(FREQ_0或FREQ_1)
          freq -- 频率值 (Freq_value(value)=Freq_data(data)*FCLK/2^28)
返回值  : 无
*****************************************************************/
void AD9834_Set_Freq(unsigned char freq_number, unsigned long freq)
{
    unsigned long FREQREG = (unsigned long)(268435456.0 / AD9834_SYSTEM_COLCK * freq);

    unsigned int FREQREG_LSB_14BIT = (unsigned int)FREQREG;
    unsigned int FREQREG_MSB_14BIT = (unsigned int)(FREQREG >> 14);

    if (freq_number == FREQ_0)
    {
        FREQREG_LSB_14BIT &= ~(1U << 15);
        FREQREG_LSB_14BIT |= 1 << 14;
        FREQREG_MSB_14BIT &= ~(1U << 15);
        FREQREG_MSB_14BIT |= 1 << 14;
    }
    else
    {
        FREQREG_LSB_14BIT &= ~(1 << 14);
        FREQREG_LSB_14BIT |= 1U << 15;
        FREQREG_MSB_14BIT &= ~(1 << 14);
        FREQREG_MSB_14BIT |= 1U << 15;
    }

    AD9834_Write_16Bits(FREQREG_LSB_14BIT);
    AD9834_Write_16Bits(FREQREG_MSB_14BIT);
}
//============================================================//
/*   注：使用时先调用初始化函数初始化IO配置，然后调用AD9834_Select_Wave()选择波形，最后选择频率值即可输出。 */
