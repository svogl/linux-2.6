/*
 * SBC-i.MX51 GPS driver
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
#include <linux/regulator/consumer.h>
#include <mach/hardware.h>
#include <mach/board-sbc51.h>

static struct regulator *gps_vdd;

static int blt_gps_probe(struct platform_device *pdev)
{
	struct blt_gps_platform_data *platform_data;
	platform_data = (struct blt_gps_platform_data *)pdev->dev.platform_data;
	if (platform_data->reg && platform_data->uV) {
		gps_vdd = regulator_get(&pdev->dev, platform_data->reg);
		regulator_set_voltage(gps_vdd, platform_data->uV,
				      platform_data->uV);
		regulator_enable(gps_vdd);
	}
	return 0;
}

static int blt_gps_remove(struct platform_device *pdev)
{
	if (gps_vdd) {
		regulator_disable(gps_vdd);
		regulator_put(gps_vdd);
	}
	return 0;
}

static struct platform_driver gps_driver = {
	.driver = {
		.name = "blt_gps",
	},
	.probe = blt_gps_probe,
	.remove = blt_gps_remove,
};

static __init int gps_init(void)
{
	printk(KERN_INFO "Enabling GPS module\n");
	return platform_driver_register(&gps_driver);
}

static void __exit gps_exit(void)
{
	printk(KERN_INFO "Disabling GPS module\n");
	platform_driver_unregister(&gps_driver);
}

module_init(gps_init);
module_exit(gps_exit);
MODULE_AUTHOR("Harald Krapfenbauer");
MODULE_LICENSE("GPL");
