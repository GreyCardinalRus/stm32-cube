#include "AT45DBXX.h"

void AT45DBXX_Busy(uint32_t *spi_base_address)
{
	uint8_t tx_data[2]={0x00,0x00}; 
	uint8_t rx_data[2]={0x00,0x00};  
	uint16_t count = 0;
	tx_data[0] = READ_STATE_REGISTER;
		
	while((!(rx_data[1] & 0x80)))
	{
		spi_master_tx_rx(spi_base_address, 2, tx_data, rx_data); 
		count++;
		if (count > 10)
			break;
	}
}

void AT45DBXX_Read_ID(uint32_t *spi_base_address, uint8_t *IData)
{
	uint8_t i;
	uint8_t tx_data[5]={0x00,0x00,0x00,0x00,0x00}; 
	uint8_t rx_data[5]={0x00,0x00,0x00,0x00,0x00};  
	AT45DBXX_Busy(spi_base_address);
  tx_data[0] = Read_ID;
	spi_master_tx_rx(spi_base_address, 5, tx_data, rx_data); 
	for(i=0;i<4;i++)
	{
		IData[i] = rx_data[i+1];
	}
}
