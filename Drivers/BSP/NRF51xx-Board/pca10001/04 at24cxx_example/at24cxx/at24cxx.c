#include "at24cxx.h"

bool AT24CXX_WriteOneByte(uint8_t register_address, uint8_t value)
{	
	uint8_t w2_data[2];
	
	w2_data[0] = register_address;
	w2_data[1] = value;
  return twi_master_transfer(AT24XXADDRESS, w2_data, 2, TWI_ISSUE_STOP);	  
}

bool AT24CXX_ReadOneByte(uint8_t register_address, uint8_t *destination, uint8_t number_of_bytes)
{
  bool transfer_succeeded;
  transfer_succeeded = twi_master_transfer(AT24XXADDRESS, &register_address, 1, TWI_DONT_ISSUE_STOP);
  transfer_succeeded &= twi_master_transfer(AT24XXADDRESS|TWI_READ_BIT, destination, number_of_bytes, TWI_ISSUE_STOP);
  return transfer_succeeded;
}

uint8_t AT24CXX_Check(void)
{
	uint8_t temp;
	AT24CXX_ReadOneByte(255, &temp, 1);
	if(temp==0X55)return 0;		   
	else
	{
		AT24CXX_WriteOneByte(255,0X55);
	  AT24CXX_ReadOneByte(255, &temp, 1);	  
		if(temp==0X55)return 0;
	}
	return 1;											  
}

