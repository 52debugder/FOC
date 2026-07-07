#include "comm_iic.h"
#include "bsp_iic.h"

void comm_iic_transmit(const uint8_t *data, uint16_t len)
{
    bsp_iic_transmit(data, len);
}

void comm_iic_receive(uint8_t *data, uint16_t len)
{
    bsp_iic_receive(data, len);
}
