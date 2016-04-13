/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "twi_master_dev.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_assert.h"
#include "nrf_soc.h"
#include "nrf_error.h"

/* Max cycles approximately to wait on RXDREADY and TXDREADY event, this is optimum way instead of using timers, this is not power aware, negetive side is this is not power aware */
//#define MAX_TIMEOUT_LOOPS             (20000UL)        /*!< MAX while loops to wait for RXD/TXD event, original design*/
#define MAX_TIMEOUT_LOOPS             (5000000UL)        /*!< MAX while loops to wait for RXD/TXD event, extend the maximal for slow device*/

/*TWI pin assignment for the following manipulation*/
#define TWI_MASTER_CONFIG_CLOCK_PIN_NUMBER (twi_interface->scl_pin)
#define TWI_MASTER_CONFIG_DATA_PIN_NUMBER (twi_interface->sda_pin)

//#define USE_SHORTS


/*
 *	the following TWI pin manipulation were modify from the macro to the function,
 *	such as set the input/output mode or toggling the SDA and SCL pins
 *
#define TWI_SCL_HIGH()   do { NRF_GPIO->OUTSET = (1UL << TWI_MASTER_CONFIG_CLOCK_PIN_NUMBER); } while(0)   // Pulls SCL line high 
#define TWI_SCL_LOW()    do { NRF_GPIO->OUTCLR = (1UL << TWI_MASTER_CONFIG_CLOCK_PIN_NUMBER); } while(0)   // Pulls SCL line low  
#define TWI_SDA_HIGH()   do { NRF_GPIO->OUTSET = (1UL << TWI_MASTER_CONFIG_DATA_PIN_NUMBER);  } while(0)   // Pulls SDA line high 
#define TWI_SDA_LOW()    do { NRF_GPIO->OUTCLR = (1UL << TWI_MASTER_CONFIG_DATA_PIN_NUMBER);  } while(0)   // Pulls SDA line low  
#define TWI_SDA_INPUT()  do { NRF_GPIO->DIRCLR = (1UL << TWI_MASTER_CONFIG_DATA_PIN_NUMBER);  } while(0)   // Configures SDA pin as input  
#define TWI_SDA_OUTPUT() do { NRF_GPIO->DIRSET = (1UL << TWI_MASTER_CONFIG_DATA_PIN_NUMBER);  } while(0)   // Configures SDA pin as output 
#define TWI_SCL_OUTPUT() do { NRF_GPIO->DIRSET = (1UL << TWI_MASTER_CONFIG_CLOCK_PIN_NUMBER); } while(0)   // Configures SCL pin as output 
 */

uint8_t TWI_SDA_READ(TWI_struct* twi_interface)
{
	return ((NRF_GPIO->IN >> twi_interface->sda_pin) & 0x1UL);
}

uint8_t TWI_SCL_READ(TWI_struct* twi_interface)
{
	return ((NRF_GPIO->IN >> twi_interface->scl_pin) & 0x1UL);
}
void TWI_SCL_HIGH(TWI_struct* twi_interface)
{
	do{
		NRF_GPIO->OUTSET = (1UL << twi_interface->scl_pin);
	}while(0);
}

void TWI_SCL_LOW(TWI_struct* twi_interface)
{
	do{
		NRF_GPIO->OUTCLR = (1UL << twi_interface->scl_pin);
	}while(0);
}

void TWI_SDA_HIGH(TWI_struct* twi_interface)
{	
	do{
		NRF_GPIO->OUTSET = (1UL << twi_interface->sda_pin);
	}while(0);
}

void TWI_SDA_LOW(TWI_struct* twi_interface)
{
	do{
		NRF_GPIO->OUTCLR = (1UL << twi_interface->sda_pin);
	}while(0);
}

void TWI_SDA_INPUT(TWI_struct* twi_interface)
{
	do{
		NRF_GPIO->DIRSET = (1UL << twi_interface->sda_pin);  
	}while(0);
}

void TWI_SDA_OUTPUT(TWI_struct* twi_interface)
{	
	do{
		NRF_GPIO->DIRSET = (1UL << twi_interface->sda_pin);
	}while(0);
}

void TWI_SCL_OUTPUT(TWI_struct* twi_interface)
{
	do{
		NRF_GPIO->DIRSET = (1UL << twi_interface->scl_pin); 
	}while(0);
}


bool twi_master_write(TWI_struct* twi_interface, uint8_t *data, uint8_t data_length, bool issue_stop_condition)
{
	uint32_t timeout = MAX_TIMEOUT_LOOPS;   /* max loops to wait for EVENTS_TXDSENT event*/
	NRF_TWI_Type* twi_dev = twi_interface->twi_dev;
	if (data_length == 0)
	{
		/* Return false for requesting data of size 0 */
		return false;
	}

	twi_dev->TXD = *data++;
	twi_dev->TASKS_STARTTX = 1;


	while (true)
	{
		while(twi_dev->EVENTS_TXDSENT == 0 && (--timeout))
		{
		}

		if (timeout == 0)
		{
			twi_dev->EVENTS_BB=0;//@kate
			twi_dev->EVENTS_STOPPED = 0; 
			twi_dev->TASKS_STOP = 1; 
			/* Wait until stop sequence is sent */
			while(twi_dev->EVENTS_STOPPED == 0) 
			{ 
			}

			/* Timeout before receiving event*/
			return false;
		}

		twi_dev->EVENTS_BB=0;//@kate
		twi_dev->EVENTS_TXDSENT = 0;
		if (--data_length == 0)
		{
			break;
		}

		twi_dev->TXD = *data++;
	}

	if (issue_stop_condition) 
	{ 
		twi_dev->EVENTS_STOPPED = 0; 
		twi_dev->TASKS_STOP = 1; 
		/* Wait until stop sequence is sent */ 
		while(twi_dev->EVENTS_STOPPED == 0) 
		{ 
		} 
	}

	return true;
}

bool twi_master_read(TWI_struct* twi_interface, uint8_t *data, uint8_t data_length, bool issue_stop_condition)
{
	NRF_TWI_Type* twi_dev = twi_interface->twi_dev;
	uint32_t timeout = MAX_TIMEOUT_LOOPS;   /* max loops to wait for RXDREADY event*/

	if (data_length == 0)
	{
		/* Return false for requesting data of size 0 */
		return false;
	}
	else if (data_length == 1)
	{
		twi_dev->EVENTS_BB=0;//@kate
#ifdef USE_SHORTS
		twi_dev->SHORTS=TWI_SHORTS_BB_STOP_Msk;
#else
		//NRF_PPI->CH[0].TEP = (uint32_t)&twi_dev->TASKS_STOP;
		sd_ppi_channel_assign(0,
				&(twi_dev->EVENTS_BB),
				&(twi_dev->TASKS_STOP));
#endif
	}
	else
	{
		twi_dev->EVENTS_BB=0;//@kate
#ifdef USE_SHORTS
		twi_dev->SHORTS=TWI_SHORTS_BB_SUSPEND_Msk;
#else
		//NRF_PPI->CH[0].TEP = (uint32_t)&twi_dev->TASKS_SUSPEND;
		sd_ppi_channel_assign(0,
				&(twi_dev->EVENTS_BB),
				&(twi_dev->TASKS_SUSPEND));
#endif
	}
#ifndef USE_SHORTS
	//NRF_PPI->CHENSET = PPI_CHENSET_CH0_Msk;
	sd_ppi_channel_enable_set(PPI_CHEN_CH0_Msk);
#endif


	twi_dev->EVENTS_RXDREADY = 0;
	twi_dev->TASKS_STARTRX = 1;
	while (true)
	{
		nrf_delay_us(10);	//add for CEAA-D0
		while((twi_dev->EVENTS_RXDREADY == 0) && (--timeout))
		{    
		}
		twi_dev->EVENTS_BB=0;//@kate
		twi_dev->EVENTS_RXDREADY = 0;

		if (timeout == 0)
		{
			twi_dev->EVENTS_BB=0;//@kate
			/* Wait until stop sequence is sent */ 
			twi_dev->EVENTS_STOPPED = 0; 
			twi_dev->TASKS_STOP = 1; 
			while(twi_dev->EVENTS_STOPPED == 0) 
			{ 
			}
#ifdef USE_SHORTS
			twi_dev->SHORTS=0;
#else
			//NRF_PPI->CHENCLR = PPI_CHENCLR_CH0_Msk;
			sd_ppi_channel_enable_clr(PPI_CHEN_CH0_Msk);
#endif
			return false;
		}

		twi_dev->EVENTS_BB=0;//@kate

		*data++ = twi_dev->RXD;

		/* Configure PPI to stop TWI master before we get last BB event */
		if (--data_length == 1)
		{
			//twi_dev->EVENTS_STOPPED = 0;// kate test
			//comment the previous because of the BMP180 failed
#ifdef USE_SHORTS
			twi_dev->SHORTS=TWI_SHORTS_BB_STOP_Msk;
#else
			//  NRF_PPI->CH[0].TEP = (uint32_t)&twi_dev->TASKS_STOP;
			sd_ppi_channel_assign(0,
					&(twi_dev->EVENTS_BB),
					&(twi_dev->TASKS_STOP));
#endif
		}

		if (data_length == 0)
		{
			break;
		}

		twi_dev->TASKS_RESUME = 1;
	}

	/* Wait until stop sequence is sent */
	//twi_dev->EVENTS_STOPPED = 0;// kate test move inside judgement:"if(--data_length == 1) "
	while(twi_dev->EVENTS_STOPPED == 0)
	{
	}

#ifdef USE_SHORTS
	twi_dev->SHORTS=0;
#else
	//  NRF_PPI->CHENCLR = PPI_CHENCLR_CH0_Msk;
	sd_ppi_channel_enable_clr(PPI_CHEN_CH0_Msk);
#endif
	return true;
}

bool twi_master_rep_start_read(TWI_struct* twi_interface, uint8_t address, uint8_t *data, uint8_t data_length, bool issue_stop_condition)
{
	uint32_t timeout = MAX_TIMEOUT_LOOPS;   /* max loops to wait for RXDREADY event*/
	NRF_TWI_Type* twi_dev = twi_interface->twi_dev;
	if (data_length == 0)
	{
		/* Return false for requesting data of size 0 */
		return false;
	}
	//twi_dev->ADDRESS = (address >> 1);

	twi_dev->TXD = address;
	twi_dev->TASKS_STARTTX = 1;


	while(twi_dev->EVENTS_TXDSENT == 0 && (--timeout))
	{
	}

	if (timeout == 0)
	{
		twi_dev->EVENTS_BB=0;//@kate
		twi_dev->EVENTS_STOPPED = 0; 
		twi_dev->TASKS_STOP = 1; 
		/* Wait until stop sequence is sent */
		while(twi_dev->EVENTS_STOPPED == 0) 
		{ 
		}

		/* Timeout before receiving event*/
		return false;
	}

	twi_dev->EVENTS_BB=0;//@kate
	twi_dev->EVENTS_TXDSENT = 0;
	// --data_length;

	timeout = MAX_TIMEOUT_LOOPS;


	if (data_length == 1)
	{
		twi_dev->EVENTS_BB=0;//@kate
#ifdef USE_SHORTS
		twi_dev->SHORTS=TWI_SHORTS_BB_STOP_Msk;
#else
		//NRF_PPI->CH[0].TEP = (uint32_t)&twi_dev->TASKS_STOP;
		sd_ppi_channel_assign(0,
				&(twi_dev->EVENTS_BB),
				&(twi_dev->TASKS_STOP));
#endif
	}
	else
	{
		twi_dev->EVENTS_BB=0;//@kate
#ifdef USE_SHORTS
		twi_dev->SHORTS=TWI_SHORTS_BB_SUSPEND_Msk;
#else
		//NRF_PPI->CH[0].TEP = (uint32_t)&twi_dev->TASKS_SUSPEND;
		sd_ppi_channel_assign(0,
				&(twi_dev->EVENTS_BB),
				&(twi_dev->TASKS_SUSPEND));
#endif
	}
#ifndef USE_SHORTS
	//NRF_PPI->CHENSET = PPI_CHENSET_CH0_Msk;
	sd_ppi_channel_enable_set(PPI_CHEN_CH0_Msk);
#endif


	twi_dev->EVENTS_RXDREADY = 0;
	twi_dev->TASKS_STARTRX = 1;

	while (true)
	{
		while((twi_dev->EVENTS_RXDREADY == 0) && (--timeout))
		{    
		}
		twi_dev->EVENTS_BB=0;//@kate
		twi_dev->EVENTS_RXDREADY = 0;

		if (timeout == 0)
		{
			twi_dev->EVENTS_BB=0;//@kate
			/* Wait until stop sequence is sent */ 
			twi_dev->EVENTS_STOPPED = 0; 
			twi_dev->TASKS_STOP = 1; 
			while(twi_dev->EVENTS_STOPPED == 0) 
			{ 
			}
#ifdef USE_SHORTS
			twi_dev->SHORTS=0;
#else
			//NRF_PPI->CHENCLR = PPI_CHENCLR_CH0_Msk;
			sd_ppi_channel_enable_clr(PPI_CHEN_CH0_Msk);
#endif
			return false;
		}

		twi_dev->EVENTS_BB=0;//@kate

		*data++ = twi_dev->RXD;

		/* Configure PPI to stop TWI master before we get last BB event */
		if (--data_length == 1)
		{
//			twi_dev->EVENTS_STOPPED = 0;// kate test
#ifdef USE_SHORTS
			twi_dev->SHORTS=TWI_SHORTS_BB_STOP_Msk;
#else
			//  NRF_PPI->CH[0].TEP = (uint32_t)&twi_dev->TASKS_STOP;
			sd_ppi_channel_assign(0,
					&(twi_dev->EVENTS_BB),
					&(twi_dev->TASKS_STOP));
#endif
		}

		if (data_length == 0)
		{
			break;
		}

		twi_dev->TASKS_RESUME = 1;
	}

	/* Wait until stop sequence is sent */
	//twi_dev->EVENTS_STOPPED = 0;// kate test move inside judgement:"if(--data_length == 1) "
	while(twi_dev->EVENTS_STOPPED == 0)
	{
	}

#ifdef USE_SHORTS
	twi_dev->SHORTS=0;
#else
	//  NRF_PPI->CHENCLR = PPI_CHENCLR_CH0_Msk;
	sd_ppi_channel_enable_clr(PPI_CHEN_CH0_Msk);
#endif
	return true;
}

/**
 * Detects stuck slaves (SDA = 0 and SCL = 1) and tries to clear the bus.
 *
 * @return
 * @retval false Bus is stuck.
 * @retval true Bus is clear.
 */
bool twi_master_clear_bus(TWI_struct* twi_interface)
{
	uint32_t twi_state;
	bool bus_clear;
	uint32_t clk_pin_config;
	uint32_t data_pin_config;
	NRF_TWI_Type *twi_dev = twi_interface->twi_dev;

	// Save and disable TWI hardware so software can take control over the pins
	twi_state = twi_dev->ENABLE;
	twi_dev->ENABLE = TWI_ENABLE_ENABLE_Disabled << TWI_ENABLE_ENABLE_Pos;

	clk_pin_config = NRF_GPIO->PIN_CNF[TWI_MASTER_CONFIG_CLOCK_PIN_NUMBER];
	NRF_GPIO->PIN_CNF[TWI_MASTER_CONFIG_CLOCK_PIN_NUMBER] = 
		(GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
		| (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos)
		| (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)
		| (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)
		| (GPIO_PIN_CNF_DIR_Output     << GPIO_PIN_CNF_DIR_Pos);    

	data_pin_config = NRF_GPIO->PIN_CNF[TWI_MASTER_CONFIG_DATA_PIN_NUMBER];
	NRF_GPIO->PIN_CNF[TWI_MASTER_CONFIG_DATA_PIN_NUMBER] = 
		(GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
		| (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos)
		| (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)
		| (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)
		| (GPIO_PIN_CNF_DIR_Output     << GPIO_PIN_CNF_DIR_Pos);    

	TWI_SDA_HIGH(twi_interface);
	TWI_SCL_HIGH(twi_interface);
	TWI_DELAY();

	if (TWI_SDA_READ(twi_interface) == 1 && TWI_SCL_READ(twi_interface) == 1)
	{
		bus_clear = true;
	}
	else
	{
		uint_fast8_t i;
		bus_clear = false;

		// Clock max 18 pulses worst case scenario(9 for master to send the rest of command and 9 for slave to respond) to SCL line and wait for SDA come high
		for (i=18; i--;)
		{
			TWI_SCL_LOW(twi_interface);
			TWI_DELAY();
			TWI_SCL_HIGH(twi_interface);
			TWI_DELAY();

			if (TWI_SDA_READ(twi_interface) == 1)
			{
				bus_clear = true;
				break;
			}
		}
	}

	NRF_GPIO->PIN_CNF[TWI_MASTER_CONFIG_CLOCK_PIN_NUMBER] = clk_pin_config;
	NRF_GPIO->PIN_CNF[TWI_MASTER_CONFIG_DATA_PIN_NUMBER] = data_pin_config;

	twi_dev->ENABLE = twi_state;

	return bus_clear;
}

bool twi_master_init(TWI_struct* twi_interface)
{
	/* To secure correct signal levels on the pins used by the TWI
	   master when the system is in OFF mode, and when the TWI master is 
	   disabled, these pins must be configured in the GPIO peripheral.
	   */

	NRF_TWI_Type* twi_dev = twi_interface->twi_dev;

	uint32_t err_code;
	NRF_GPIO->PIN_CNF[TWI_MASTER_CONFIG_CLOCK_PIN_NUMBER] = 
		(GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
		| (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos)
		| (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)
		| (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)
		| (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos);    

	NRF_GPIO->PIN_CNF[TWI_MASTER_CONFIG_DATA_PIN_NUMBER] = 
		(GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
		| (GPIO_PIN_CNF_DRIVE_S0D1     << GPIO_PIN_CNF_DRIVE_Pos)
		| (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)
		| (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)
		| (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos);    

	twi_dev->EVENTS_RXDREADY = 0;
	twi_dev->EVENTS_TXDSENT = 0;
	twi_dev->PSELSCL = TWI_MASTER_CONFIG_CLOCK_PIN_NUMBER;
	twi_dev->PSELSDA = TWI_MASTER_CONFIG_DATA_PIN_NUMBER;
	twi_dev->FREQUENCY = TWI_FREQUENCY_FREQUENCY_K100 << TWI_FREQUENCY_FREQUENCY_Pos;
	//twi_dev->FREQUENCY = TWI_FREQUENCY_FREQUENCY_K400 << TWI_FREQUENCY_FREQUENCY_Pos;

#ifdef USE_SHORTS
	twi_dev->EVENTS_BB=0;
#else
	/*
	   NRF_PPI->CH[0].EEP = (uint32_t)&twi_dev->EVENTS_BB;
	   NRF_PPI->CH[0].TEP = (uint32_t)&twi_dev->TASKS_SUSPEND;
	   NRF_PPI->CHENCLR = PPI_CHENCLR_CH0_Msk;*/
	err_code = sd_ppi_channel_assign(0,
			&(twi_dev->EVENTS_BB),
			&(twi_dev->TASKS_SUSPEND));
	ASSERT(err_code == NRF_SUCCESS);

	err_code = sd_ppi_channel_enable_clr(PPI_CHEN_CH0_Msk);
	ASSERT(err_code == NRF_SUCCESS);
#endif

	twi_dev->ENABLE = TWI_ENABLE_ENABLE_Enabled << TWI_ENABLE_ENABLE_Pos;

	return twi_master_clear_bus(twi_interface);
}

bool twi_master_transfer(TWI_struct* twi_interface, uint8_t address, uint8_t *data, uint8_t data_length, bool issue_stop_condition)
{
	bool transfer_succeeded = true;
	NRF_TWI_Type *twi_dev = twi_interface->twi_dev;
	if (data_length > 0 && twi_master_clear_bus(twi_interface))
	{

		twi_dev->ADDRESS = (address >> 1);

		if ((address & TWI_READ_BIT))
		{
			transfer_succeeded = twi_master_read(twi_interface, data, data_length, issue_stop_condition);
			//transfer_succeeded = twi_master_rep_start_read(reg, data, data_length, issue_stop_condition);
		}
		else
		{
			transfer_succeeded = twi_master_write(twi_interface, data, data_length, issue_stop_condition);
		}
	}
	return transfer_succeeded;
}

/*lint --flb "Leave library region" */
