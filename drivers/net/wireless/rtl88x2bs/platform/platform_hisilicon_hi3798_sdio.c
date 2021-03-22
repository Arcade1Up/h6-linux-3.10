/******************************************************************************
 *
 * Copyright(c) 2017 - 2018 Realtek Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 *****************************************************************************/
#include <linux/delay.h>		/* mdelay() */
#include <linux/gpio.h>			/* gpio_to_irq() and etc */
#include <mach/hardware.h>		/* __io_address(), readl(), writel() */
#include "platform_hisilicon_hi3798_sdio.h"	/* HI_S32() and etc. */

typedef enum hi_GPIO_DIR_E {
	HI_DIR_OUT = 0,
	HI_DIR_IN  = 1,
} HI_GPIO_DIR_E;

#define RTL_REG_ON_GPIO		(4*8 + 3)
#if 0
#define RTL_OOB_INT_GPIO	(5*8 + 1)	/* WL_HOST_WAKE gpio */
#else
#ifndef RTL_OOB_INT_GPIO
#ifdef CONFIG_GPIO_WAKEUP
#error "RTL_OOB_INT_GPIO not defined"
#else /* !CONFIG_GPIO_WAKEUP */
#define RTL_OOB_INT_GPIO	0
#endif /* !CONFIG_GPIO_WAKEUP */
#endif /* !RTL_OOB_INT_GPIO */
#endif

#define REG_BASE_CTRL		__io_address(0xf8a20008)

int gpio_wlan_reg_on = RTL_REG_ON_GPIO;
#if 0
module_param(gpio_wlan_reg_on, uint, 0644);
MODULE_PARM_DESC(gpio_wlan_reg_on, "wlan reg_on gpio num (default:gpio4_3)");
#endif

int gpio_wlan_wake_host = RTL_OOB_INT_GPIO;
module_param(gpio_wlan_wake_host, uint, 0644);
MODULE_PARM_DESC(gpio_wlan_wake_host, "wlan wake host gpio num (default:gpio5_1)");

#ifdef CONFIG_GPIO_WAKEUP
int hisi_oob_irq = 0;
#endif /* CONFIG_GPIO_WAKEUP */

static int hi_gpio_set_value(u32 gpio, u32 value)
{
	HI_S32 s32Status;

	s32Status = HI_DRV_GPIO_SetDirBit(gpio, HI_DIR_OUT);
	if (s32Status != HI_SUCCESS) {
		pr_err("gpio(%d) HI_DRV_GPIO_SetDirBit HI_DIR_OUT failed\n",
			gpio);
		return -1;
	}

	s32Status = HI_DRV_GPIO_WriteBit(gpio, value);
	if (s32Status != HI_SUCCESS) {
		pr_err("gpio(%d) HI_DRV_GPIO_WriteBit value(%d) failed\n",
			gpio, value);
		return -1;
	}

	return 0;
}

#ifdef CONFIG_GPIO_WAKEUP
static int hisi_wlan_get_oob_irq(unsigned gpio)
{
	int host_oob_irq = 0;
	int err = 0;


	err = gpio_request(gpio, "oob irq");
	if (err) {
		pr_err("%s: gpio-%d gpio_request failed(%d)\n",
		       __FUNCTION__, gpio, err);
		return 0;
	}

	err = gpio_direction_input(gpio);
	if (err) {
		pr_err("%s: gpio-%d gpio_direction_input failed(%d)\n",
		       __FUNCTION__, gpio, err);
		gpio_free(gpio);
		return 0;
	}

	host_oob_irq = gpio_to_irq(gpio);
	pr_info("%s: GPIO(%u) IRQ(%d)\n",
		__FUNCTION__, gpio, host_oob_irq);

	return host_oob_irq;
}

static void hisi_wlan_free_oob_gpio(unsigned gpio)
{
	pr_info("%s: gpio_free(%u)\n", __FUNCTION__, gpio);
	gpio_free(gpio);
}
#endif /* CONFIG_GPIO_WAKEUP */

static int hisi_wlan_set_carddetect(bool present)
{
	u32 regval;
	u32 mask;


#ifndef CONFIG_HISI_SDIO_ID
	return;
#endif
	pr_info("SDIO ID=%d\n", CONFIG_HISI_SDIO_ID);
#if (CONFIG_HISI_SDIO_ID == 1)
	mask = 1;
#elif (CONFIG_HISI_SDIO_ID == 0)
	mask = 2;
#endif

	regval = readl(REG_BASE_CTRL);
	if (present) {
		pr_info("====== Card detection to detect SDIO card! ======\n");
		/* set card_detect low to detect card */
		regval |= mask;
	} else {
		pr_info("====== Card detection to remove SDIO card! ======\n");
		/* set card_detect high to remove card */
		regval &= ~(mask);
	}
	writel(regval, REG_BASE_CTRL);

	return 0;
}

/*
 * Return:
 *	0:	power on successfully
 *	others: power on failed
 */
int platform_wifi_power_on(void)
{
	int ret = 0;


#ifdef CONFIG_GPIO_WAKEUP
	hisi_oob_irq = hisi_wlan_get_oob_irq(gpio_wlan_wake_host);
#endif

	hi_gpio_set_value(gpio_wlan_reg_on, 1);
	mdelay(100);
	hisi_wlan_set_carddetect(1);
	mdelay(2000);
	pr_info("======== set_carddetect delay 2s! ========\n");

	return ret;
}

void platform_wifi_power_off(void)
{
#ifdef CONFIG_GPIO_WAKEUP
	hisi_oob_irq = 0;
	hisi_wlan_free_oob_gpio(gpio_wlan_wake_host);
#endif

	hisi_wlan_set_carddetect(0);
	mdelay(100);
	hi_gpio_set_value(gpio_wlan_reg_on, 0);
}

#ifdef CONFIG_GPIO_WAKEUP
int platform_wifi_get_oob_irq(void)
{
	int irq = 0;


	irq = hisi_oob_irq;

	return irq;
}
#endif /* CONFIG_GPIO_WAKEUP */

