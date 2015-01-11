#ifndef __AT24CXX_H
#define	__AT24CXX_H

#include <stdbool.h>
#include <stdint.h>
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_delay.h"
#include "twi_master.h"

#define AT24XXADDRESS 0xa0

bool AT24CXX_WriteOneByte(uint8_t register_address, uint8_t value);
bool AT24CXX_ReadOneByte(uint8_t register_address, uint8_t *destination, uint8_t number_of_bytes);
uint8_t AT24CXX_Check(void);

#endif
