#include "BMG160.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
bool BMG_write_register(TWI_struct *twi_dev, uint8_t m_device_address, uint8_t register_address, const uint8_t value);
bool BMG_read_register(TWI_struct *twi_dev, uint8_t m_device_address, uint8_t register_address, uint8_t length, uint8_t *value);


bool BMG160_measurement(BMG160_struct *dev)
{	
	uint8_t rev_data[6];
	BMG_read_register(dev->twi_dev, dev->dev_addr, 0x02, 0x06, rev_data);
	dev->ang_vel[0] = (rev_data[1] << 8) + rev_data[0];
	dev->ang_vel[1] = (rev_data[3] << 8) + rev_data[2];
	dev->ang_vel[2] = (rev_data[5] << 8) + rev_data[4];
	
	return 0;
}

bool BMG160_configuration(BMG160_struct *dev)
{
	if(
	BMG_write_register(dev->twi_dev, dev->dev_addr, 0x10, dev->filter)&&
	BMG_write_register(dev->twi_dev, dev->dev_addr, 0x0f, dev->range)&&
	BMG_write_register(dev->twi_dev, dev->dev_addr, 0x1A, 0x20)&&
	BMG_write_register(dev->twi_dev, dev->dev_addr, 0x31, 0x07)){
		return true;
	}else{
		return false;
	}
}





bool BMG_write_register(TWI_struct* twi_interface,uint8_t m_device_address, uint8_t register_address, const uint8_t value){

	uint8_t w2_data[2];
	w2_data[0] = register_address;
	w2_data[1] = value;
	
	//sprintf(dbg, "%x\n\r", twi_interface);
	//simple_uart_putstring(dbg);

	return twi_master_transfer(twi_interface, m_device_address, w2_data, 2, TWI_ISSUE_STOP);
}


bool BMG_read_register(TWI_struct* twi_interface, uint8_t m_device_address, uint8_t register_address, uint8_t length, uint8_t *value){

	bool transfer_succeeded = false;

	transfer_succeeded = twi_master_transfer(twi_interface, m_device_address, &register_address, 1, TWI_DONT_ISSUE_STOP);
	if (transfer_succeeded){		
		transfer_succeeded &= twi_master_transfer(twi_interface, m_device_address | TWI_READ_BIT, value, 6, TWI_ISSUE_STOP);
	}
	return transfer_succeeded;
}
