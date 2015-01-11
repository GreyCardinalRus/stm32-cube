#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "simple_uart.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_delay.h"
#include "at24cxx.h"

#define TESTADDRESS 254

int main(void)
{
	static uint8_t writeDate = 0x00;
	uint8_t temp;
  simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, HWFC);
	twi_master_init();
	nrf_gpio_cfg_output(LED_0);
	nrf_gpio_cfg_output(LED_1);
	nrf_gpio_cfg_input(BUTTON_0,BUTTON_PULL);
	nrf_gpio_cfg_input(BUTTON_1,BUTTON_PULL);
	if (0 == AT24CXX_Check())
	{
		nrf_gpio_pin_set(LED_0);
	}
	else
	{
		nrf_gpio_pin_set(LED_1);
	}
	while(1)
	{	
		nrf_delay_ms(100);
		if (0 == nrf_gpio_pin_read(BUTTON_0))
		{
			AT24CXX_ReadOneByte(TESTADDRESS, &temp, 1);
			simple_uart_put(temp);
			nrf_delay_ms(100);
		}
		if (0 == nrf_gpio_pin_read(BUTTON_1))
		{
			AT24CXX_ReadOneByte(TESTADDRESS,&writeDate, 1);
			nrf_delay_us(5);
			AT24CXX_WriteOneByte(TESTADDRESS,++writeDate);
			nrf_delay_ms(100);
		}
	}

}
