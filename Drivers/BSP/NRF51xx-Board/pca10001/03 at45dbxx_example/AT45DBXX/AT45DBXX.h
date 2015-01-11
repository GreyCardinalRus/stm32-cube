#ifndef __AT24CXX_H
#define	__AT24CXX_H

#include "spi_master.h"
#include "spi_master_config.h"

#define READ_STATE_REGISTER 0xD7
#define Read_ID 0x9F

void AT45DBXX_Busy(uint32_t *spi_base_address);
void AT45DBXX_Read_ID(uint32_t *spi_base_address, uint8_t *IData);

#endif
