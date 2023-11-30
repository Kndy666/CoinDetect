#ifndef INC_HMI_H_
#define INC_HMI_H_

#include "main.h"
#include <stdbool.h>

#define REV_BUF_SIZE 512

typedef struct HMI_Handle
{
    uint8_t bufferRev[REV_BUF_SIZE];
    uint16_t dataLength;
    bool isRevOK;
}HMI_Handle;

void HMI_Init(UART_HandleTypeDef *huart, HMI_Handle *rev);
void HMI_Printf(uint8_t *cmd);
void HMI_BufReset();
#endif