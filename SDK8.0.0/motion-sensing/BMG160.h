#include "twi_master_dev.h"

#define BANDWIDTH_32HZ (0x07)
#define BANDWIDTH_64HZ (0x06)
#define BANDWIDTH_12HZ (0x05)
#define BANDWIDTH_23HZ (0x04)
#define BANDWIDTH_47HZ  (0x3)
#define BANDWIDTH_116HZ (0x02)
#define BANDWIDTH_230HZ (0x01)
#define BANDWIDTH_NONE (0x00)

#define FULLSCALE_2000 (0x00)
#define FULLSCALE_1000 (0x01)
#define FULLSCALE_500 (0x02)
#define FULLSCALE_250 (0x03)
#define FULLSCALE_125 (0x04)

typedef struct{
	TWI_struct *twi_dev;
	uint8_t dev_addr;
	int16_t ang_vel[3];
	uint8_t temp;
	uint8_t range;
	uint8_t filter;
}BMG160_struct;

bool BMG_write_register(TWI_struct *twi_dev, uint8_t m_device_address, uint8_t register_address, const uint8_t value);
bool BMG_read_register(TWI_struct *twi_dev, uint8_t m_device_address, uint8_t register_address, uint8_t length, uint8_t *value);
bool BMG160_measurement(BMG160_struct *dev);
bool BMG160_configuration(BMG160_struct *dev);
