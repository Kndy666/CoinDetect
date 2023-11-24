#include "HMI.h"

HMI_Rev *_rev;
void HMI_Init(UART_HandleTypeDef *huart, HMI_Rev *rev)
{
    _rev = rev;
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    HAL_UART_Receive_DMA(huart, _rev->bufferRev, REV_BUF_SIZE);
}
void USAR_UART_IDLECallback(UART_HandleTypeDef *huart)
{
    HAL_UART_DMAStop(huart);
    _rev->dataLength = REV_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
    _rev->isRevOK = true;
    HAL_UART_Receive_DMA(huart, _rev->bufferRev, REV_BUF_SIZE);
}
void HMI_Printf(uint8_t *cmd)
{
    printf("%s%s", cmd, "\xff\xff\xff");
}