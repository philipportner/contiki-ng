#include "contiki.h"
#include "dw1000.h"

void board_init(void)
{
  dw1000_driver.set_value(RADIO_PARAM_PAN_ID, 0xabcd);
  dw1000_driver.set_value(RADIO_PARAM_16BIT_ADDR,(((uint8_t*)&linkaddr_node_addr)[0]) << 8 | (((uint8_t*)&linkaddr_node_addr)[1])); // converting from big-endian format
}