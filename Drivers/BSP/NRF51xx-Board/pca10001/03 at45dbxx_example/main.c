#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "nrf.h"
#include "simple_uart.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_delay.h"
#include "AT45DBXX.h"


int main(void)
{
	uint8_t ID[4];
	uint8_t i;
	uint32_t * p_spi_base_address;
	
  simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, HWFC);
	p_spi_base_address = spi_master_init(SPI0, SPI_MODE3, false);

  while(true)
  {
		AT45DBXX_Read_ID(p_spi_base_address, ID);
		nrf_delay_ms(100);
		for (i = 0;i < 4;++i)
		{
			simple_uart_put(ID[i]);
		}
  }
}


