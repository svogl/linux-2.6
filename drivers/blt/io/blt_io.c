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
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>

#include "../../../arch/arm/mach-mx5/iomux.h"
#include "../../../arch/arm/mach-mx5/mx51_pins.h"

/* tl says these are our pins:
	DIG_IO.IN1 -> Stecker Pin: 32
	...
	DIG_IO.IN4 -> Stecker Pin: 35

	DIG_IO.OUT1 -> Stecker Pin: 24
	...
	DIG_IO.OUT4 -> Stecker Pin: 27
*/


static struct delayed_work cd_work;

static void print_inputs(void)
{
	int d20 = gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_EIM_D20));
	int d21 = gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_EIM_D21));
	int d22 = gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_EIM_D22));
	int d23 = gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_EIM_D23));

	printk(KERN_INFO "BLT IO IN  %d %d %d %d\n", d20, d21, d22, d23);
}

static void format_inputs(char* buf)
{
	int d20 = gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_EIM_D20));
	int d21 = gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_EIM_D21));
	int d22 = gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_EIM_D22));
	int d23 = gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_EIM_D23));

	sprintf(buf, "[%d,%d,%d,%d]\n", d20, d21, d22, d23);
}

static void cd_work_func(struct work_struct *work)
{
	print_inputs();

	schedule_delayed_work(&cd_work, msecs_to_jiffies(1000));
}

/*********************************************************************/
/* char device functions */

static struct cdev blt_io_cdev;
static struct completion read_complete;
static int wait4irq=0;

struct  blt_io_device {
	int minor;

	wait_queue_head_t read_queue;
};

typedef struct blt_io_device blt_io_device_t;


static ssize_t blt_io_read(struct file *fd, char __user *buf, size_t len,
                              loff_t *ptr)
{
	int ret;
	char line[80];
        blt_io_device_t *dev = (blt_io_device_t *)fd->private_data;

	init_completion(&read_complete);
	wait4irq=1;
	ret = wait_for_completion_interruptible_timeout(
		&read_complete, 10 * HZ);
	if (ret) {
		format_inputs(line);
		if ( len < strlen(line) ) {
			return -ENOMEM; // increase user buffer!
		}
		copy_to_user(buf,line, strlen(line));
		*ptr = strlen(line);
		return strlen(line);
	}
	return 0;
}

static ssize_t blt_io_write(struct file *fd, const char __user *buf,
                               size_t len, loff_t *fpos)
{
	char line[80];
	int ret, state;
        blt_io_device_t *dev = (blt_io_device_t *)fd->private_data;

	if (len>=80) {
                ret = -ENOMEM;
                goto error0;
	}
	if (len<2) {
                ret = -EINVAL;
                goto error0;
	}
        printk(KERN_INFO " BLT IO WRITE: got %d bytes \n", len);

	if (copy_from_user(line, buf, len)) {
	        printk(KERN_INFO " BLT IO WRITE: copy fail \n");
                ret = -EFAULT;
                goto error0;
        }
	// first byte: channel select
        if ( (line[0]<'0' || line[0]>'3') ) {
	        printk(KERN_INFO " BLT IO WRITE: 0 illegal %d\n", line[0]); // skip it..
		*fpos++;
                ret = 1;
                goto error0;
        }
	// second byte: state
        if ( (line[1]<'0' || line[1]>'1') ) {
	        printk(KERN_INFO " BLT IO WRITE: 1 illegal \n");
                ret = -EINVAL;
                goto error0;
        }
	state = ( line[1] == '1' );
	
        printk(KERN_INFO " BLT IO WRITE: setting %d to %d\n", line[0]-'0', state);

	switch (line[0]-'0') {
	case 0:
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_CSPI1_MISO), state);
		break;
	case 1:
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_CSPI1_SS0), state);
		break;
	case 2:
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_CSPI1_SS1), state);
		break;
	case 3:
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_CSPI1_RDY), state);
		break;
	}
	if (line[2]=='\n') {
		*fpos+=3;
		return 3;
	} else {
		*fpos+=2;
		return 2;
	}
error0:
	return ret;
}

static int blt_io_open(struct inode *inode, struct file *fd)
{
        int ret;
        unsigned int id;
        blt_io_device_t *dev;
        id = iminor(inode);

       	printk(KERN_INFO "OPENING... %d \n", id);
	
        if (fd->private_data != NULL) {
        	printk(KERN_INFO "device already open!");
        	ret =-EBUSY;
                goto error0;
        }

	dev = kmalloc( sizeof(blt_io_device_t), GFP_KERNEL );	   
	if (!dev) {
                ret = -ENOMEM;
                goto error0;
        }
	init_waitqueue_head(&dev->read_queue);

        fd->private_data = (void *)dev;
        ret = 0;

//	schedule_delayed_work(&cd_work, msecs_to_jiffies(1000));

error0:
       	printk(KERN_INFO "OPENED? %d \n", ret);

        return ret;
}

static int blt_io_release(struct inode *inode, struct file *fd)
{
        int ret;
        unsigned int id;
        blt_io_device_t *dev;
        id = iminor(inode);
        
        kfree(fd->private_data);
        return 0;
}

static long blt_io_ioctl(struct file *file, unsigned int cmd,
                          unsigned long arg)
{
	return 0;
}
static long blt_io_compat_ioctl(struct file *file,
                                 unsigned int cmd, unsigned long arg)
{
	return 0;
}

static struct file_operations blt_io_fops = {
        .owner = THIS_MODULE,
        .llseek = no_llseek,
        .read = blt_io_read,
        .write = blt_io_write,

//        .unlocked_ioctl = blt_io_ioctl,
#ifdef CONFIG_COMPAT
//        .compat_ioctl = blt_io_compat_ioctl,
#endif
        .open = blt_io_open,
        .release = blt_io_release,
};

static int blt_io_probe(struct platform_device *pdev)
{
	int ret=0;
	printk(KERN_INFO "BLT IO PROBE");

	INIT_DELAYED_WORK(&cd_work, cd_work_func);

	print_inputs();
	/*
	cdev_init(&blt_io_cdev, &blt_io_fops);
	blt_io_cdev.owner = THIS_MODULE;
	blt_io_cdev.ops = &blt_io_fops;
	ret = cdev_add(&blt_io_cdev, 0, 1);
	if (ret<0) {
		printk(KERN_WARNING " failed to add devices %d\n", ret);
	}

	*/
	printk(KERN_INFO " add returned %d\n", ret);
	return 0;
}

static int blt_io_remove(struct platform_device *pdev)
{
	printk(KERN_INFO "BLT IO REMOVE");

	//cdev_del(&blt_io_cdev);

	printk(KERN_INFO "BLT IO REMOVED");
	return 0;
}

static struct platform_device blt_io_device = {
	.name = "blt_io",
	.id = 0,
};

static struct platform_driver blt_io_driver = {
	.driver = {
		.name = "blt_io",
	},
	.probe = blt_io_probe,
	.remove = blt_io_remove,
};

static irqreturn_t blt_io_irqhandler(int irq, void *dev_id)
{
	printk(KERN_INFO "BLT IO IRQ %d\n", irq);
	print_inputs();

	if (wait4irq) {
		complete(&read_complete);
		wait4irq = 0;
	}
        return IRQ_HANDLED;
}

static __init int blt_io_init(void)
{
	int ret;
	/*
	int devnum = MKDEV(242, 0);

	ret = register_chrdev_region(devnum, 1, blt_io_driver.driver.name);
	if (ret<0) {
		printk(KERN_WARNING " failed to register major %d\n", ret);
	}
	*/
	ret = register_chrdev(242, "blt_io", &blt_io_fops);
	if (ret)
		return ret;

        ret |= gpio_request(IOMUX_TO_GPIO(MX51_PIN_CSPI1_MISO), blt_io_driver.driver.name );
	if (ret<0) {
		printk(KERN_WARNING " failed to request gpio(s 1) %d\n", ret);
	}
        ret |= gpio_request(IOMUX_TO_GPIO(MX51_PIN_CSPI1_SS0), blt_io_driver.driver.name );
	if (ret<0) {
		printk(KERN_WARNING " failed to request gpio(s 2) %d\n", ret);
	}
        ret |= gpio_request(IOMUX_TO_GPIO(MX51_PIN_CSPI1_SS1), blt_io_driver.driver.name );
	if (ret<0) {
		printk(KERN_WARNING " failed to request gpio(s 3) %d\n", ret);
	}
        ret |= gpio_request(IOMUX_TO_GPIO(MX51_PIN_CSPI1_RDY), blt_io_driver.driver.name );
	if (ret<0) {
		printk(KERN_WARNING " failed to request gpio(s 4) %d\n", ret);
	}

	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_CSPI1_MISO), 0);
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_CSPI1_SS0), 0);
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_CSPI1_SS1), 0);
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_CSPI1_RDY), 1);


	ret = 0;
        ret |= gpio_request(IOMUX_TO_GPIO(MX51_PIN_EIM_D20), blt_io_driver.driver.name );
	if (ret<0) {
		printk(KERN_WARNING " failed to request gpio(s 5) %d\n", ret);
	}
        ret |= gpio_request(IOMUX_TO_GPIO(MX51_PIN_EIM_D21), blt_io_driver.driver.name );
	if (ret<0) {
		printk(KERN_WARNING " failed to request gpio(s 6) %d\n", ret);
	}
        ret |= gpio_request(IOMUX_TO_GPIO(MX51_PIN_EIM_D22), blt_io_driver.driver.name );
	if (ret<0) {
		printk(KERN_WARNING " failed to request gpio(s 7) %d\n", ret);
	}
        ret |= gpio_request(IOMUX_TO_GPIO(MX51_PIN_EIM_D23), blt_io_driver.driver.name );
	if (ret<0) {
		printk(KERN_WARNING " failed to request gpio(s 8) %d\n", ret);
	}

	gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_EIM_D20));
	gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_EIM_D21));
	gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_EIM_D22));
	gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_EIM_D23));
	ret =0;
	ret |= request_irq(IOMUX_TO_IRQ(MX51_PIN_EIM_D20) , blt_io_irqhandler, 
		IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, blt_io_driver.driver.name, NULL);
	ret |= request_irq(IOMUX_TO_IRQ(MX51_PIN_EIM_D21) , blt_io_irqhandler, 
		IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, blt_io_driver.driver.name, NULL);
	ret |= request_irq(IOMUX_TO_IRQ(MX51_PIN_EIM_D22) , blt_io_irqhandler, 
		IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, blt_io_driver.driver.name, NULL);
	ret |= request_irq(IOMUX_TO_IRQ(MX51_PIN_EIM_D23) , blt_io_irqhandler, 
		IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, blt_io_driver.driver.name, NULL);
        if (ret<0) {
		printk(KERN_WARNING " failed to request irq(s) %d\n", ret);
	}
	print_inputs();

	printk(KERN_INFO "Enabling IO module\n");

	platform_device_register(&blt_io_device);
	return platform_driver_register(&blt_io_driver);
}

static void __exit blt_io_exit(void)
{
	/*
	int devnum = MKDEV(242, 0);
	unregister_chrdev_region(devnum, 1);
	*/
	//i2c_del_driver(&i2cdev_driver);
	//class_destroy(i2c_dev_class);

	unregister_chrdev(242, "blt_io");

	free_irq(IOMUX_TO_IRQ(MX51_PIN_EIM_D20) , NULL);
	free_irq(IOMUX_TO_IRQ(MX51_PIN_EIM_D21) , NULL);
	free_irq(IOMUX_TO_IRQ(MX51_PIN_EIM_D22) , NULL);
	free_irq(IOMUX_TO_IRQ(MX51_PIN_EIM_D23) , NULL);

	gpio_free(IOMUX_TO_GPIO(MX51_PIN_EIM_D20));
	gpio_free(IOMUX_TO_GPIO(MX51_PIN_EIM_D21));
	gpio_free(IOMUX_TO_GPIO(MX51_PIN_EIM_D22));
	gpio_free(IOMUX_TO_GPIO(MX51_PIN_EIM_D23));

	gpio_free(IOMUX_TO_GPIO(MX51_PIN_CSPI1_MISO));
	gpio_free(IOMUX_TO_GPIO(MX51_PIN_CSPI1_SS0));
	gpio_free(IOMUX_TO_GPIO(MX51_PIN_CSPI1_SS1));
	gpio_free(IOMUX_TO_GPIO(MX51_PIN_CSPI1_RDY));

	printk(KERN_INFO "Disabling IO module\n");

	platform_device_unregister(&blt_io_device);
	printk(KERN_INFO "Disabling IO module 2\n");
	udelay(100);
	platform_driver_unregister(&blt_io_driver);
	printk(KERN_INFO "Disabling IO module 3\n");
}

module_init(blt_io_init);
module_exit(blt_io_exit);
MODULE_AUTHOR("Simon Vogl");
MODULE_LICENSE("GPL");
