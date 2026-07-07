#include "bsp_usart.h"

#define BSP_USART_UART &huart1

static volatile uint8_t bsp_usart_tx_busy;

void bsp_usart_send(const uint8_t *data, uint16_t size)
{
    HAL_UART_Transmit(BSP_USART_UART, (uint8_t *)data, size, 100);
}

HAL_StatusTypeDef bsp_usart_send_async(const uint8_t *data, uint16_t size)
{
    if (size == 0U)
        return HAL_OK;

    if (bsp_usart_tx_busy != 0U)
        return HAL_BUSY;

    bsp_usart_tx_busy = 1U;
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(BSP_USART_UART, (uint8_t *)data, size);
    if (status != HAL_OK)
        bsp_usart_tx_busy = 0U;
    return status;
}

uint8_t bsp_usart_tx_is_idle(void)
{
    return (uint8_t)(bsp_usart_tx_busy == 0U);
}

void bsp_usart_tx_complete(UART_HandleTypeDef *huart)
{
    if (huart == BSP_USART_UART)
        bsp_usart_tx_busy = 0U;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    bsp_usart_tx_complete(huart);
}

void bsp_usart_recv(uint8_t *data, uint16_t size)
{
    HAL_UART_Receive(BSP_USART_UART, data, size, 1000);
}


