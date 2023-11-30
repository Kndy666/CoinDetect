#include "HMI.h"
#include "Retarget.h"
#include <string.h>

HMI_Handle *_rev;
void HMI_Init(UART_HandleTypeDef *huart, HMI_Handle *rev)
{
    _rev = rev;
    HAL_UARTEx_ReceiveToIdle_DMA(huart, _rev->bufferRev, REV_BUF_SIZE);
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart -> Instance == USART1)
    {
        _rev->dataLength = Size;
        _rev->isRevOK = true;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, _rev->bufferRev, REV_BUF_SIZE);
    }
}
void HMI_Printf(uint8_t *cmd)
{
    printf("%s%s", cmd, "\xff\xff\xff");
}
void HMI_BufReset()
{
    _rev->isRevOK = false;
    _rev->dataLength = 0;
    memset(_rev->bufferRev, 0, REV_BUF_SIZE);
}