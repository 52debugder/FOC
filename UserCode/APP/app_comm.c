#include "app_comm.h"
#include "app_motor.h"
#include "comm_uart.h"
#include "comm_iic.h"

#define APP_DEBUG_SAMPLE_DIVIDER 10U

typedef struct
{
    float u;
    float v;
    float w;
} app_debug_sample_t;

static volatile uint8_t app_debug_sample_pending;
static uint8_t app_debug_sample_counter;

void app_uart_send(const uint8_t *p_data, uint16_t length)
{
    comm_uart_send(p_data, length);
}

void app_uart_recv(uint8_t *p_data, uint16_t length)
{
    comm_uart_recv(p_data, length);
}

void app_iic_transmit(const uint8_t *p_data, uint16_t length)
{
    comm_iic_transmit(p_data, length);
}

void app_iic_receive(uint8_t *p_data, uint16_t length)
{
    comm_iic_receive(p_data, length);
}

void app_debug_sample_from_isr(void)
{
    app_debug_sample_counter++;
    if (app_debug_sample_counter < APP_DEBUG_SAMPLE_DIVIDER)
        return;

    app_debug_sample_counter = 0U;
    app_debug_sample_pending = 1U;
}

void app_debug_print(void)
{
    if ((app_debug_sample_pending == 0U) || (comm_uart_tx_is_idle() == 0U))
        return;

    app_motor_telemetry_t telemetry;
    app_debug_sample_t sample;

    app_motor_get_telemetry(1, &telemetry);

    __disable_irq();
    sample.u = telemetry.target_speed_rpm;
    sample.v = telemetry.measured_speed_rpm;
    sample.w = (float)telemetry.sample_time_us;
    app_debug_sample_pending = 0U;
    __enable_irq();

    HAL_StatusTypeDef status = comm_vofa_send3_async(sample.u, sample.v, sample.w);
    if (status != HAL_OK)
    {
        __disable_irq();
        if (app_debug_sample_pending == 0U)
            app_debug_sample_pending = 1U;
        __enable_irq();
    }
}

