#ifndef _BMA250E_H_
#define _BMA250E_H_

#include "twi_master_dev.h"
#define RANGE_REG	(0x0F)
#define RANGE_PN2G	(0x03)
#define RANGE_PN4G	(0x05)
#define RANGE_PN8G	(0x08)
#define RANGE_PN16G	(0x0C)


#define BANDWIDTH_REG	(0x10)
#define BAND_7D81HZ	(0x08)
#define BAND_15D63HZ	(0x09)
#define BAND_31D25HZ	(0x0A)
#define BAND_62D5HZ	(0x0B)
#define BAND_125HZ	(0x0C)
#define BAND_250HZ	(0x0D)
#define BAND_500HZ	(0x0E)
#define BAND_1000HZ	(0x0F)

#define POWER_MODE	(0x11)
#define NORMAL_MODE	(0x10)
#define DEEP_SUSPEND_MODE	(0x30)
#define LOW_POWER_MODE	(0x50)
#define SUSPEND_MODE	(0x90)


typedef struct{
	TWI_struct *twi_dev;
	uint8_t dev_addr;
	uint8_t range;
	uint8_t bandwidth;
	int16_t acc_value[3];
}BMA250E_struct;

bool BMA_write_register(TWI_struct *twi_dev, uint8_t m_device_address, uint8_t register_address, const uint8_t value);
bool BMA_read_register(TWI_struct *twi_dev, uint8_t m_device_address, uint8_t register_address, uint8_t length, uint8_t *value);
bool BMA250E_measurement(BMA250E_struct *dev);
bool BMA250E_softreset(BMA250E_struct *dev);
bool BMA250E_configuration(BMA250E_struct *dev);
bool BMA_write_register(TWI_struct* twi_interface,uint8_t m_device_address, uint8_t register_address, const uint8_t value);
bool BMA_read_register(TWI_struct* twi_interface, uint8_t m_device_address, uint8_t register_address, uint8_t length, uint8_t *value);

#endif
