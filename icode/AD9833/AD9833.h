/*************************************************************************************
 Title	:   Analog Devices AD9833 DDS Wave Generator Library for STM32 Using HAL Libraries
 Author:    Bardia Alikhan Afshar <bardia.a.afshar@gmail.com>  
 Software:  IAR Embedded Workbench for ARM
 Hardware:  Any STM32 device
*************************************************************************************/
#ifndef _AD_9833_H
#define _AD_9833_H
#include <math.h>
#include "spi.h"

// ------------------------- Defines -------------------------
#define FMCLK 25000000        // Master Clock On AD9833
#define AD9833_SPI_PORT hspi2 // SPI PORT OF AD9833
#define AD9833_FSYNC_GPIO_Port GPIOB      // PORT OF AD9833
#define AD9833_FSYNC_Pin GPIO_PIN_14   // SPI Chip Select
enum WaveType{SIN, SQR, TRI}; // Wave Selection Enum

// ------------------ Functions  ---------------------
void AD9833_SetWave(uint16_t Wave);                      // Sets Output Wave Type
void AD9833_SetWaveData(float Frequency,float Phase);    // Sets Wave Frequency & Phase
void AD9833_Init(uint16_t Wave,float FRQ,float Phase);   // Initializing AD9833
#endif