/*
 * include/linux/mfd/acx00/core.h -- Core interface for ACX00
 *
 * Copyright 2009 Wolfson Microelectronics PLC.
 *
 * Author: Mark Brown <broonie@opensource.wolfsonmicro.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __MFD_AC108_CORE_H__
#define __MFD_AC108_CORE_H__

#include <linux/mutex.h>
#include <linux/interrupt.h>


/*SYS_VERSION:0x0000*/
#define CHIP_PACKAGE		14
#define CHIP_VERSION		0

/*SYS_CONTROL:0x0002*/
#define CIHP_RESET			0

#define AC108_MAX_NUM			2	/*range[1, 4] */

struct voltage_supply {
	char *regulator_name;
	struct regulator *vcc3v3;
};

struct gpio_power {
	unsigned int gpio;
	bool level;
};

struct ac108_config {
	unsigned int used;
	unsigned int twi_bus_num;
	unsigned int twi_dev_num;
	unsigned int twi_addr;
	unsigned int pga_gain;
	unsigned int digital_vol;
	unsigned int slot_width;
	unsigned int power_vol;
	struct voltage_supply regulator_cfg;
	unsigned int gpio_pwr_used;
	struct gpio_power gpio_power;
};

struct mfd_ac108 {
	struct mutex lock;
	struct device *dev;
	struct i2c_client **i2c;
	struct regmap *regmap;
	struct ac108_config *dts_cfg;
	struct attribute_group attr_group;
	int dev_num;
	int irq;
};

/* Device I/O API */
int ac108_reg_read(u8 reg, u8 *rt_value, struct i2c_client *client);
int ac108_reg_write(u8 reg, unsigned char value, struct i2c_client *client);
int ac108_update_bits(u8 reg, u8 mask, u8 value, struct i2c_client *client);

#endif
