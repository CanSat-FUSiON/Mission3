#include "lps25hb.h"
#include <math.h>

uint8_t data = 0;
uint8_t address = LPS25HB_DEVICE_ADDRESS_0;

uint8_t lps25hb_init(void)
{
	uint8_t status = 1;

	LL_mDelay(100);

	uint8_t val = lps25hb_read_byte(LPS25HB_WHO_AM_I_ADDRESS);

	if(val == LPS25HB_WHO_AM_I_VALUE)
	{
		status = 1;
	}
	else			//if the device is not found on one address, try another one
	{
		address = LPS25HB_DEVICE_ADDRESS_1;
		val = lps25hb_read_byte(LPS25HB_WHO_AM_I_ADDRESS);
		if(val == LPS25HB_WHO_AM_I_VALUE)
		{
			status = 1;
		}
		else
		{
			status = 0;
			//return status;
		}
	}

	uint8_t ctrl1 = 0b11000000;
	lps25hb_write_byte(LPS25HB_ADDRESS_CTRL1, ctrl1); //active mode

	return status;
}

void lps25hb_get_pressure(float *p)
{
	uint8_t data[3];

	lps25hb_readArray(data, LPS25HB_PRESSURE_ADDRESS, 3);
	*p = ((int32_t)(data[2] << 16) | (data[1] << 8) | data[0])/(float)(4096);
}


void lps25hb_get_altitude(float *a)
{
	uint8_t data[3];
	uint32_t pressure;
	float p0 = 1013.25;
	float scale = 4096;

	lps25hb_readArray(data, LPS25HB_PRESSURE_ADDRESS, 3);
	pressure = ((uint32_t)data[2]) << 16 | ((uint16_t)data[1]) << 8 | data[0];
	*a = 44330*(1-pow((float)(pressure)/(float)(scale*p0),(1/5.255f)));
}

uint8_t lps25hb_read_byte(uint8_t reg_addr)
{
	data = 0;
	return *(i2c_master_read(&data, 1, reg_addr, LPS25HB_DEVICE_ADDRESS, 0));
}

void lps25hb_readArray(uint8_t * data, uint8_t reg, uint8_t length)
{
	i2c_master_read(data, length, reg, LPS25HB_DEVICE_ADDRESS, 1);
}


void lps25hb_write_byte(uint8_t reg_addr, uint8_t value)
{
	i2c_master_write(value, reg_addr, LPS25HB_DEVICE_ADDRESS, 0);
}
