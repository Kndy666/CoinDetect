#ifndef INC_HMI_H_
#define INC_HMI_H_

#include "main.h"
#include <stdbool.h>

#define REV_BUF_SIZE 512

typedef struct HMI_Rev
{
    uint8_t bufferRev[REV_BUF_SIZE];
    uint16_t dataLength;
    bool isRevOK;
}HMI_Rev;

extern DMA_HandleTypeDef hdma_usart1_rx;

void HMI_Init(UART_HandleTypeDef *huart, HMI_Rev *rev);
void USAR_UART_IDLECallback(UART_HandleTypeDef *huart);
void HMI_Printf(uint8_t *cmd);
#endif