/*
 * SBC-i.MX51 GSM/GPRS driver
 * Copyright (C) 2010 Harald Krapfenbauer, Bluetechnix
 * 
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free 
 * Software Foundation; either version 2 of the License, or (at your option) 
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT 
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for 
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with 
 * this program; if not, write to the Free Software Foundation, Inc., 59 Temple 
 * Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <mach/hardware.h>
#include <mach/board-sbc51.h>
#include <linux/delay.h>

static int blt_gsm_probe(struct platform_device *pdev)
{
	struct blt_gsm_platform_data *platform_data;
	platform_data = (struct blt_gsm_platform_data *)pdev->dev.platform_data;
	if (platform_data->power) {
		platform_data->power(1);
	}
	return 0;
}

static int blt_gsm_remove(struct platform_device *pdev)
{
	struct blt_gsm_platform_data *platform_data;
	platform_data = (struct blt_gsm_platform_data *)pdev->dev.platform_data;
	if (platform_data->power) {
		platform_data->power(0);
	}
	return 0;
}

static struct platform_driver gsm_driver = {
	.driver = {
		.name = "blt_gsm",
	},
	.probe = blt_gsm_probe,
	.remove = blt_gsm_remove,
};

static __init int gsm_init(void)
{
	printk(KERN_INFO "Enabling GSM/GPRS module\n");
	return platform_driver_register(&gsm_driver);
}

static void __exit gsm_exit(void)
{
	printk(KERN_INFO "Disabling GSM/GPRS module\n");
	platform_driver_unregister(&gsm_driver);
}

module_init(gsm_init);
module_exit(gsm_exit);
MODULE_AUTHOR("Harald Krapfenbauer");
MODULE_LICENSE("GPL");
