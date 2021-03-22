/*
 * ac108-core.c  --  Device access for allwinnertech ac108
 *
 * Author: yumingfeng@allwinnertech.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/mfd/core.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/mfd/ac108-mfd.h>
#include <linux/pwm.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/sys_config.h>

#undef AC108_MATCH_DTS_EN

struct ac108_config ac108_cfg[AC108_MAX_NUM];
static int mfd_add_flag;
static long dump_val;

static struct mfd_cell ac108_devs[] = {
	{
		.name = "ac108-codec",
	},
#if 0
	{
		.name = "ac108-codec1",
	},
	{
		.name = "ac108-codec2",
	},
	{
		.name = "ac108-codec3",
	},
#endif
};

/**
 * ac108_reg_read: read a single AC108 register.
 *
 * @client: Device to read.
 * @reg: Register to read.
 * @rt_value: Value to read.
 */
int ac108_reg_read(u8 reg, u8 *rt_value, struct i2c_client *client)
{
	int ret;
	u8 read_cmd[3] = { 0 };
	u8 cmd_len = 0;

	read_cmd[0] = reg;
	cmd_len = 1;

	if (client == NULL && client->adapter == NULL) {
		pr_err("ac108_read client->adapter==NULL\n");
		return -1;
	}

	ret = i2c_master_send(client, read_cmd, cmd_len);
	if (ret != cmd_len) {
		pr_err("ac108_read error1\n");
		return -1;
	}

	ret = i2c_master_recv(client, rt_value, 1);
	if (ret != 1) {
		pr_err("ac108_read error2, ret = %d.\n", ret);
		return -1;
	}

	return 0;
}

/**
 * ac108_reg_write: Write a single AC108 register.
 *
 * @client: Device to write to.
 * @reg: Register to write to.
 * @value: Value to write.
 */
int ac108_reg_write(u8 reg, unsigned char value, struct i2c_client *client)
{
	int ret = 0;
	u8 write_cmd[2] = { 0 };

	write_cmd[0] = reg;
	write_cmd[1] = value;

	if (client == NULL && client->adapter == NULL) {
		pr_err("ac108_read client->adapter==NULL\n");
		return -1;
	}

	ret = i2c_master_send(client, write_cmd, 2);
	if (ret != 2) {
		pr_err("ac108_write error->[REG-0x%02x, val-0x%02x], ret: = %d\n",
			reg, value, ret);
		return -1;
	}

	return 0;
}

int ac108_update_bits(u8 reg, u8 mask, u8 value, struct i2c_client *client)
{
	u8 val_old, val_new;

	ac108_reg_read(reg, &val_old, client);
	val_new = (val_old & ~mask) | (value & mask);
	if (val_new != val_old) {
		ac108_reg_write(reg, val_new, client);
	}

	return 0;
}

/**************************read reg interface**************************/
static ssize_t ac108_dump_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	u8 reg;
	u8 num;

	dump_val = simple_strtol(buf, NULL, 16);
	reg = (dump_val >> 8) & 0xFF;
	num = dump_val & 0xff;
	if (reg <= 0xc7) {
		if ((reg+num) <= 0xc7)
			pr_err("\nRead: start REG:0x%02x,count:0x%02x\n", reg, num);
		else
			pr_err("\nReg[0x%02x] + num[0x%02x] is out of range! (Reg Range: 0x00 --> 0xC7)\n", reg, num);
	} else
		pr_err("The REG:0x%02x not exist!! (Range: 0x00 --> 0xC7)\n", reg);

	return count;
}

static ssize_t ac108_dump_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	size_t count = 0;
	u8 reg;
	u8 num;
	u8 value_r;
	u8 i = 0;
	struct mfd_ac108 *ac108 = dev_get_drvdata(dev);

	count += sprintf(buf, "echo reg|count > dump\n");
	count += sprintf(buf + count, "Dump reg: AC108 twi_dev_num: %d\n",
			ac108->dts_cfg->twi_dev_num);
	count += sprintf(buf + count, "eg read star addres=0x06,count 0x10:echo 0x0610 > dump\n");

	reg = (dump_val >> 8) & 0xFF;
	num = dump_val & 0xff;
	count += sprintf(buf + count, "\nRead: start REG:0x%02x,count:0x%02x\n", reg, num);

	if (reg >= 0 && num >= 0) {
		do {
			value_r = 0;
			ac108_reg_read(reg, &value_r, ac108->i2c[ac108->dts_cfg->twi_dev_num]);
			count += sprintf(buf + count, "REG[0x%02x]: 0x%02x;  ", reg, value_r);
			reg++;
			i++;
			if ((i == num) || (i % 4 == 0))
				count += sprintf(buf + count, "\n");
		} while ((i < num) && (reg <= 0xc7));
	}

	return count;
}

static ssize_t ac108_write_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int val = 0;
	u8 reg;
	u8 value_w;
	u8 value_r;
	struct mfd_ac108 *ac108 = dev_get_drvdata(dev);

	val = simple_strtol(buf, NULL, 16);

	pr_err("[%s] ac108->dts_cfg->twi_dev_num:%d, count:%ld\n", __func__,
			ac108->dts_cfg->twi_dev_num, count);
	reg = (val >> 8) & 0xFF;
	value_w = val & 0xFF;
	if (reg <= 0xc7) {
		ac108_reg_write(reg, value_w, ac108->i2c[ac108->dts_cfg->twi_dev_num]);
		pr_err("Write 0x%02x to REG:0x%02x\n", value_w, reg);
		ac108_reg_read(reg, &value_r, ac108->i2c[ac108->dts_cfg->twi_dev_num]);
		pr_err("Current reg [0x%02x] -->  0x%02x\n", reg, value_r);
	} else {
		pr_err("The REG:0x%02x not exist!! (Range: 0x00 --> 0xC7)\n", reg);
	}
	return count;
}

static ssize_t ac108_write_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	size_t count = 0;
	struct mfd_ac108 *ac108 = dev_get_drvdata(dev);

	count += sprintf(buf, "echo reg|value > write\n");
	count += sprintf(buf + count, "Dump reg: AC108 twi_dev_num: %d\n",
			ac108->dts_cfg->twi_dev_num);
	count += sprintf(buf + count, "eg write value:0xfe addres=0x06, echo 0x06fe > write\n");

	return count;
}

static DEVICE_ATTR(dump, 0644, ac108_dump_show, ac108_dump_store);
static DEVICE_ATTR(write, 0644, ac108_write_show, ac108_write_store);

static struct attribute *audio_debug_attrs[] = {
	&dev_attr_dump.attr,
	&dev_attr_write.attr,
	NULL,
};

static struct attribute_group audio_debug_attr_group = {
	.name   = "ac108_reg_debug",
	.attrs  = audio_debug_attrs,
};

static struct ac108_config *get_mfd_ac108_config(struct i2c_client *client)
{
	struct i2c_adapter *adapter = client->adapter;
	struct ac108_config *ac108 = NULL;
	int index = 0;

	for (index = 0; index < AC108_MAX_NUM; index++) {
		if (!ac108_cfg[index].used)
			continue;
		if ((ac108_cfg[index].twi_bus_num == adapter->nr)
				&& (ac108_cfg[index].twi_addr == client->addr)) {
			ac108 = &ac108_cfg[index];
			break;
		} else {
			continue;
		}
	}

	return ac108;
}

static const struct regmap_config ac108_regmap_config = {
	.reg_bits = 8,		/*Number of bits in a register address */
	.val_bits = 8,		/*Number of bits in a register value */
};

static struct i2c_client *i2c_ctl[AC108_MAX_NUM];

/*
 * Instantiate the generic non-control parts of the device.
 */
static int ac108_device_init(struct mfd_ac108 *ac108)
{
	int ret;

	if (!mfd_add_flag) {
		mfd_add_flag = 1;
		ret = mfd_add_devices(ac108->dev, PLATFORM_DEVID_NONE,
			      ac108_devs, ARRAY_SIZE(ac108_devs),
			      NULL, 0, NULL);
		if (ret != 0) {
			dev_err(ac108->dev, "Failed to add children: %d\n", ret);
			mfd_add_flag = 0;
			goto err;
		}
	}

	return 0;
err:
	mfd_remove_devices(ac108->dev);
	return ret;
}

static void ac108_device_exit(struct mfd_ac108 *ac108)
{
	mfd_remove_devices(ac108->dev);
}

static int ac108_i2c_probe(struct i2c_client *i2c,
			const struct i2c_device_id *id)
{
	struct mfd_ac108 *ac108;
	int ret = 0;
	static int index;

	pr_err("[%s] line:%d\n", __func__, __LINE__);
	ac108 = devm_kzalloc(&i2c->dev, sizeof(struct mfd_ac108), GFP_KERNEL);
	if (ac108 == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, ac108);
	ac108->dts_cfg = get_mfd_ac108_config(i2c);
	i2c_ctl[index] = i2c;
	ac108->i2c = i2c_ctl;
	ac108->dev = &(i2c->dev);
	ac108->irq = i2c->irq;
	index++;

	memcpy(&(ac108->attr_group), &audio_debug_attr_group, sizeof(struct attribute_group));
	ret = sysfs_create_group(&i2c->dev.kobj, &(ac108->attr_group));
	if (ret) {
		pr_err("failed to create attr group\n");
	}

	return ac108_device_init(ac108);
}

static int ac108_i2c_remove(struct i2c_client *i2c)
{
	struct mfd_ac108 *ac108 = i2c_get_clientdata(i2c);

	sysfs_remove_group(&i2c->dev.kobj, &(ac108->attr_group));
	ac108_device_exit(ac108);
	devm_kfree(&i2c->dev, ac108);

	return 0;
}

static int ac108_gpio_iodisable(u32 gpio)
{
	char pin_name[8];
	u32 config, ret;
	sunxi_gpio_to_name(gpio, pin_name);
	config = 7 << 16;
	ret = pin_config_set(SUNXI_PINCTRL, pin_name, config);
	return ret;
}

static int ac108_i2c_suspend(struct device *dev)
{
#if 0
	/*
	 * if the regulator or gpio was disabled at ac108 module.
	 * it should be noted this part at here.
	 */
	int index = 0;
	int ret = 0;

	for (index = 0; index < AC108_MAX_NUM; index++) {
		if (ac108_cfg[index].gpio_power.gpio) {
			gpio_set_value(ac108_cfg[index].gpio_power.gpio,
					!ac108_cfg[index].gpio_power.level);
		}
		if (ac108_cfg[index].regulator_cfg.vcc3v3) {
			ret = regulator_disable(ac108_cfg[index].regulator_cfg.vcc3v3);
			if (ret != 0) {
				pr_err("[%s] disable regulator failed!\n", __func__);
			} else
				continue;
		}
	}

	return ret;
#endif
	return 0;
}

static int ac108_i2c_resume(struct device *dev)
{
#if 0
	/*
	 * if the regulator or gpio was disabled at ac108 module.
	 * it should be noted this part at here.
	 */
	int index = 0;
	int ret = 0;

	for (index = 0; index < AC108_MAX_NUM; index++) {
		if (ac108_cfg[index].gpio_power.gpio) {
			gpio_set_value(ac108_cfg[index].gpio_power.gpio,
					ac108_cfg[index].gpio_power.level);
		}
		if (ac108_cfg[index].regulator_cfg.vcc3v3) {
			ret = regulator_enable(ac108_cfg[index].regulator_cfg.vcc3v3);
			if (ret != 0)
				pr_err("[%s] enable regulator failed!\n", __func__);
		} else
			continue;
	}
	return ret;
#endif
	return 0;
}

static const struct dev_pm_ops ac108_core_pm_ops = {
	.suspend_late = ac108_i2c_suspend,
	.resume_early = ac108_i2c_resume,
};

static int ac108_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
	int ret = -ENODEV;
	char i2c_type[I2C_NAME_SIZE] = {0};
	int index = 0;

	pr_err("[%s] line:%d, adapter->nr:%d, client->addr:0x%x\n",
			__func__, __LINE__, adapter->nr, client->addr);

	for (index = 0; index < AC108_MAX_NUM; index++) {
		if (!ac108_cfg[index].used)
			continue;

		if ((ac108_cfg[index].twi_bus_num == adapter->nr)
				&& (ac108_cfg[index].twi_addr == client->addr)) {
			memset(info->type, 0, I2C_NAME_SIZE);
			strncpy(i2c_type, "MicArray_", strlen("MicArray_"));
			sprintf(i2c_type + strlen("MicArray_"), "%u", ac108_cfg[index].twi_dev_num);
			strncpy(info->type, i2c_type, I2C_NAME_SIZE);
			pr_err("[%s] line:%d, adapter->nr:%d, info->type:%s\n",
				__func__, __LINE__, adapter->nr, info->type);
			ret = 0;
		}
	}

	return ret;
}

/*
 * device tree source or i2c_board_info both use to
 * transfer hardware information to linux kernel,
 * use one of them wil be OK
 */
/*
static struct i2c_board_info ac108_i2c_board_info[] = {
	{I2C_BOARD_INFO ("MicArray_0", 0x3b),},
	{I2C_BOARD_INFO ("MicArray_1", 0x35),},
	{I2C_BOARD_INFO ("MicArray_2", 0x3c),},
	{I2C_BOARD_INFO ("MicArray_3", 0x36),},
};
*/

static const unsigned short ac108_i2c_addr[] = {
	0x3b, 0x35, 0x3c, 0x36,
	I2C_CLIENT_END,
};

static const struct i2c_device_id ac108_i2c_id[] = {
	{"MicArray_0", 0},
	/*ac108_0 */
	{"MicArray_1", 1},
	/*ac108_1 */
	{"MicArray_2", 2},
	/*ac108_2 */
	{"MicArray_3", 3},
	/*ac108_3 */
	{}
};
MODULE_DEVICE_TABLE(i2c, ac108_i2c_id);

static const struct of_device_id ac108_dt_ids[] = {
	{.compatible = "MicArray_0",},
	/*ac108_0 */
	{.compatible = "MicArray_1",},
	/*ac108_1 */
	{.compatible = "MicArray_2",},
	/*ac108_2 */
	{.compatible = "MicArray_3",},
	/*ac108_3 */
	{},
};

static struct i2c_driver ac108_i2c_driver = {
	.class 		= I2C_CLASS_HWMON,
	.id_table 	= ac108_i2c_id,
	.probe 		= ac108_i2c_probe,
	.remove 	= ac108_i2c_remove,
	.driver 	= {
		.owner = THIS_MODULE,
		.name = "AC108-CHIP",
#ifdef AC108_MATCH_DTS_EN
		.of_match_table = ac108_dt_ids,
#endif
		.pm = &ac108_core_pm_ops,
	},
#ifndef AC108_MATCH_DTS_EN
	.address_list = ac108_i2c_addr,
	.detect = ac108_i2c_detect,
#endif
};

/* type: 0:invalid, 1: int; 2:str; 3:gpio*/
static int sys_script_get_item(char *main_name, char *sub_name, unsigned int value[],
			       int type)
{
	char compat[32];
	u32 len = 0;
	struct device_node *node;
	int ret = 0;

	len = sprintf(compat, "allwinner,sunxi-%s", main_name);
	if (len > 32) {
		pr_warn("size of mian_name is out of range\n");
		goto error_exit;
	}

	node = of_find_compatible_node(NULL, NULL, compat);
	if (!node) {
		pr_warn("of_find_compatible_node %s fail\n", compat);
		goto error_exit;
	}

	if (1 == type) {
		if (of_property_read_u32_array(node, sub_name, value, 1)) {
			pr_info("of_property_read_u32_array %s.%s fail\n",
				main_name, sub_name);
			goto error_exit;
		} else
			ret = type;
	} else if (2 == type) {
		const char *str;

		if (of_property_read_string(node, sub_name, &str)) {
			pr_info("of_property_read_string %s.%s fail\n",
				main_name, sub_name);
			goto error_exit;
		} else {
			ret = type;
			memcpy((void *)value, str, strlen(str) + 1);
		}
	} else if (3 == type) {
		struct gpio_config gpio_cfg;
		*value = of_get_named_gpio_flags(node,
					sub_name, 0,
					(enum of_gpio_flags *)&gpio_cfg);
		ret = type;
	}

	return ret;
error_exit:
	return -1;
}

static int __init ac108_i2c_init(void)
{
	int ret = 0;
	unsigned int temp_val;
	int index;
	char ac108_key_name[20] = {0};
	char regulator_name[20] = {0};
	int i2c_used = 0;

	memset(&ac108_cfg, 0, AC108_MAX_NUM * sizeof(struct ac108_config));
	strcpy(ac108_key_name, "MicArray");
	for (index = 0; index < 4; index++) {
		sprintf(ac108_key_name + 8, "%u", index);
		pr_err("[%s] ac108_key_name:%s\n", __func__, ac108_key_name);
		ret = sys_script_get_item(ac108_key_name, "ac108_used", &temp_val, 1);
		if (ret == 1) {
			ac108_cfg[index].used = temp_val;
		} else {
			ac108_cfg[index].used = 0;
			continue;
		}

		if (ac108_cfg[index].used) {
			i2c_used = 1;
			ret = sys_script_get_item(ac108_key_name, "twi_bus", &temp_val, 1);
			if (ret == 1) {
				ac108_cfg[index].twi_bus_num = temp_val;
			} else {
				pr_err ("get ac108 twi_bus failed \n");
				ac108_cfg[index].twi_bus_num = 0;
			}

			ret = sys_script_get_item(ac108_key_name, "twi_num", &temp_val, 1);
			if (ret == 1) {
				ac108_cfg[index].twi_dev_num = temp_val;
			} else {
				pr_err ("get ac108 twi_bus failed \n");
				ac108_cfg[index].twi_dev_num = 0;
			}

			ret = sys_script_get_item(ac108_key_name, "twi_addr", &temp_val, 1);
			if (ret == 1) {
				ac108_cfg[index].twi_addr = temp_val;
			} else {
				pr_err ("get ac108 twi_addr failed \n");
				ac108_cfg[index].twi_addr = 0;
			}

			ret = sys_script_get_item(ac108_key_name, "pga_gain", &temp_val, 1);
			if (ret == 1) {
				ac108_cfg[index].pga_gain = temp_val;
			} else {
				pr_err ("get ac108 pga gain failed \n");
				ac108_cfg[index].pga_gain = 0;
			}

			ret = sys_script_get_item(ac108_key_name, "digital_vol", &temp_val, 1);
			if (ret == 1) {
				ac108_cfg[index].digital_vol = temp_val;
			} else {
				pr_err ("get ac108 gigital volume failed \n");
				ac108_cfg[index].digital_vol = 0;
			}

			ret = sys_script_get_item(ac108_key_name, "slot_width", &temp_val, 1);
			if (ret == 1) {
				ac108_cfg[index].slot_width = temp_val;
			} else {
				pr_err ("get ac108 slot_width failed \n");
				ac108_cfg[index].slot_width = 0;
			}

			ret = sys_script_get_item(ac108_key_name, "gpio_pwr_used", &temp_val, 1);
			if (ret == 1) {
				ac108_cfg[index].gpio_pwr_used = temp_val;
			} else {
				pr_err ("get ac108 gpio for power_used failed \n");
				ac108_cfg[index].gpio_pwr_used = 0;
			}

			ret = sys_script_get_item(ac108_key_name, "gpio_pwr_level", &temp_val, 1);
			if (ret == 1) {
				ac108_cfg[index].gpio_power.level = temp_val;
			} else {
				pr_err ("get ac108 gpio for power_used failed \n");
				ac108_cfg[index].gpio_power.level = 0;
			}

			if (ac108_cfg[index].gpio_pwr_used) {
				ret = sys_script_get_item(ac108_key_name, "gpio-power", &temp_val, 3);
				if (ret == 3) {
					ac108_cfg[index].gpio_power.gpio = temp_val;
					/* may for i2c */
					if (!gpio_is_valid(ac108_cfg[index].gpio_power.gpio)) {
						pr_err("[MicArray-%d] get ac108 power_gpio is invalid\n", index);
					} else {
						pr_err("[MicArray-%d] get ac108 power_gpio:%u\n",
							index, ac108_cfg[index].gpio_power.gpio);
						ret = gpio_request(ac108_cfg[index].gpio_power.gpio, "gpio-power");
						if (ret) {
							pr_err("get ac108 power_gpio failed\n");
						} else {
							gpio_direction_output(ac108_cfg[index].gpio_power.gpio, 1);
							gpio_set_value(ac108_cfg[index].gpio_power.gpio,
									ac108_cfg[index].gpio_power.level);
						}
					}
				} else {
					pr_err ("[MicArray-%d] get ac108 gpio-power failed \n", index);
					ac108_cfg[index].gpio_power.gpio = 0;
				}
			}

			ret = sys_script_get_item(ac108_key_name, "regulator_name", (int *)&regulator_name, 2);
			if (ret == 2) {
				if (strncmp(regulator_name, "nocare", 6)) {
					ac108_cfg[index].regulator_cfg.vcc3v3 = regulator_get(NULL,
						      regulator_name);
					if (IS_ERR(ac108_cfg[index].regulator_cfg.vcc3v3)) {
						pr_err ("get ac108 audio-3v3 failed\n");
					}
					regulator_set_voltage(ac108_cfg[index].regulator_cfg.vcc3v3,
						3300000, 3300000);
					ret = regulator_enable(ac108_cfg[index].regulator_cfg.vcc3v3);
					if (ret != 0)
						pr_err("[AC108] %s: fail to enable regulator!\n", __func__);
				} else {
					pr_err ("get ac108 regulator nocare!\n");
				}
			} else {
				pr_err ("get ac108 regulator_name failed \n");
			}
		}
	}

	if (i2c_used) {
		ret = i2c_add_driver(&ac108_i2c_driver);
		if (ret != 0) {
			pr_err("Failed to register ac108 I2C driver: %d\n", ret);
		}
	}

	return ret;
}
subsys_initcall_sync(ac108_i2c_init);

static void __exit ac108_i2c_exit(void)
{
	int index = 0;

	for (index = 0; index < AC108_MAX_NUM; index++) {
		if (ac108_cfg[index].gpio_power.gpio) {
			gpio_set_value(ac108_cfg[index].gpio_power.gpio,
					!ac108_cfg[index].gpio_power.level);
			ac108_gpio_iodisable(ac108_cfg[index].gpio_power.gpio);
			gpio_free(ac108_cfg[index].gpio_power.gpio);
			ac108_cfg[index].gpio_power.gpio = 0;
		}
		if (ac108_cfg[index].regulator_cfg.vcc3v3) {
			regulator_put(ac108_cfg[index].regulator_cfg.vcc3v3);
			ac108_cfg[index].regulator_cfg.vcc3v3 = NULL;
		}
	}

	i2c_del_driver(&ac108_i2c_driver);
}
module_exit(ac108_i2c_exit);

MODULE_DESCRIPTION("Core support for the AC108 audio CODEC");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("yumingfeng<yumingfeng@allwinnertech.com>");
