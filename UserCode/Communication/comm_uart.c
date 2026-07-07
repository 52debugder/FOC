#include "comm_uart.h"
#include "bsp_usart.h"

#define COMM_UART_DMA_BUFFER_SIZE 64U

static char comm_uart_dma_buffer[2][COMM_UART_DMA_BUFFER_SIZE];
static uint8_t comm_uart_dma_index;

void comm_uart_send(const uint8_t *data, uint16_t len)
{
    bsp_usart_send(data, len);
}

HAL_StatusTypeDef comm_uart_send_async(const uint8_t *data, uint16_t len)
{
    return bsp_usart_send_async(data, len);
}

uint8_t comm_uart_tx_is_idle(void)
{
    return bsp_usart_tx_is_idle();
}

void comm_uart_recv(uint8_t *data, uint16_t len)
{
    bsp_usart_recv(data, len);
}

void comm_vofa_send1(const float data)
{
    char data_buf[256];
    uint8_t len = sprintf(data_buf, "%.3f\n", data);
    bsp_usart_send((uint8_t *)data_buf, len);
}

void comm_vofa_send2(const float data, const float data2)
{
    char data_buf[256];
    uint8_t len = sprintf(data_buf, "%.3f,%.3f\n", data, data2);
    bsp_usart_send((uint8_t *)data_buf, len);
}

void comm_vofa_send3(const float data, const float data2, const float data3)
{
    char data_buf[256];
    uint8_t len = sprintf(data_buf, "%.3f,%.3f,%.3f\n", data, data2, data3);
    bsp_usart_send((uint8_t *)data_buf, len);
}

HAL_StatusTypeDef comm_vofa_send3_async(const float data, const float data2, const float data3)
{
    char *data_buf = comm_uart_dma_buffer[comm_uart_dma_index];
    int len = snprintf(data_buf, COMM_UART_DMA_BUFFER_SIZE, "%.3f,%.3f,%.3f\n", data, data2, data3);
    if ((len <= 0) || (len >= (int)COMM_UART_DMA_BUFFER_SIZE))
        return HAL_ERROR;

    HAL_StatusTypeDef status = bsp_usart_send_async((uint8_t *)data_buf, (uint16_t)len);
    if (status == HAL_OK)
        comm_uart_dma_index ^= 1U;
    return status;
}

void comm_vofa_send4(const float data, const float data2, const float data3, const float data4)
{
    char data_buf[256];
    uint8_t len = sprintf(data_buf, "%.3f,%.3f,%.3f,%.3f\n", data, data2, data3, data4);
    bsp_usart_send((uint8_t *)data_buf, len);
}

