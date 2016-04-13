
#include "twi_master_dev.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
//#include "simple_uart.h"
#include "BMA250E.h"
#include "nordic_common.h"

typedef BMA250E_struct BMA250E_struct;
bool BMA_write_register(TWI_struct *twi_dev, uint8_t m_device_address, uint8_t register_address, const uint8_t value);
bool BMA_read_register(TWI_struct *twi_dev, uint8_t m_device_address, uint8_t register_address, uint8_t length, uint8_t *value);


bool BMA250E_measurement(BMA250E_struct *dev)
{	
	uint8_t rev_data[6];
	uint8_t ret_state;
	ret_state = BMA_read_register(dev->twi_dev, dev->dev_addr, 0x02, 0x06, rev_data);
	dev->acc_value[0] = ((int16_t)(rev_data[1] << 8) + rev_data[0]) >> 6;
	dev->acc_value[1] = ((int16_t)(rev_data[3] << 8) + rev_data[2]) >> 6;
	dev->acc_value[2] = ((int16_t)(rev_data[5] << 8) + rev_data[4]) >> 6;
	return ret_state;
}

bool BMA250E_softreset(BMA250E_struct *dev)
{	
	return BMA_write_register(dev->twi_dev, dev->dev_addr, 0x14, 0xB6);
}

bool BMA250E_configuration(BMA250E_struct *dev)
{
	BMA_write_register(dev->twi_dev, dev->dev_addr, 0x14, 0xB6);
	if(BMA_write_register(dev->twi_dev, dev->dev_addr, RANGE_REG, dev->range) &&
			BMA_write_register(dev->twi_dev, dev->dev_addr, BANDWIDTH_REG, dev->bandwidth)){
		return true;
	}else{
		return false;
	}
}





bool BMA_write_register(TWI_struct* twi_interface,uint8_t m_device_address, uint8_t register_address, const uint8_t value){

	uint8_t w2_data[2];
	w2_data[0] = register_address;
	w2_data[1] = value;
	return twi_master_transfer(twi_interface,m_device_address, w2_data, 2, TWI_ISSUE_STOP);
}


bool BMA_read_register(TWI_struct* twi_interface, uint8_t m_device_address, uint8_t register_address, uint8_t length, uint8_t *value){

	bool transfer_succeeded = false;

	transfer_succeeded = twi_master_transfer(twi_interface, m_device_address, &register_address, 1, TWI_DONT_ISSUE_STOP);
	if (transfer_succeeded){		
		transfer_succeeded &= twi_master_transfer(twi_interface, m_device_address | TWI_READ_BIT, value, 6, TWI_ISSUE_STOP);
	}
	return transfer_succeeded;
}
