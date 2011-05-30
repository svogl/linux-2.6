/*
 *  HID driver for some eaton "special" devices
 *
 *  Copyright (c) 1999 Andreas Gal
 *  Copyright (c) 2000-2005 Vojtech Pavlik <vojtech@suse.cz>
 *  Copyright (c) 2005 Michael Haboustak <mike-@cinci.rr.com> for Concept2, Inc
 *  Copyright (c) 2007 Paul Walmsley
 *  Copyright (c) 2008 Jiri Slaby
 *  Copyright (c) 2006-2008 Jiri Kosina
 */
#define DEBUG 1
/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */

#include <linux/device.h>
#include <linux/hid.h>
#include <linux/module.h>
#include <linux/usb.h>
#include "usbhid/usbhid.h"

#include "hid-ids.h"

struct eaton_sc {
	unsigned long quirks;
};



static int eaton_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	int ret;
	unsigned long quirks = id->driver_data;
	struct eaton_sc *sc;

printk(KERN_INFO "HID-EATON probe %08x\n", quirks);

	sc = kzalloc(sizeof(*sc), GFP_KERNEL);
	if (sc == NULL) {
		dev_err(&hdev->dev, "can't alloc descriptor\n");
		return -ENOMEM;
	}

#ifdef CONFIG_HID_DEBUG
extern int hid_debug;
	hid_debug=0xff;
#endif

	sc->quirks = quirks;
	hid_set_drvdata(hdev, sc);

	ret = hid_parse(hdev);
	if (ret) {
		dev_err(&hdev->dev, "parse failed\n");
		goto err_free;
	}

printk(KERN_INFO "HID-EATON hid %08x\n", hdev->status);
printk(KERN_INFO "HID-EATON hid %08x\n", hdev->claimed);
printk(KERN_INFO "HID-EATON hid %08x\n", hdev->quirks);
printk(KERN_INFO "HID-EATON hid %s\n", hdev->name);
printk(KERN_INFO "HID-EATON hid %s\n", hdev->phys);
printk(KERN_INFO "HID-EATON hid %s\n", hdev->uniq);

printk(KERN_INFO "HID-EATON hid app %p\n", hdev->collection_size);
printk(KERN_INFO "HID-EATON hid app %p\n", hdev->maxcollection);
printk(KERN_INFO "HID-EATON hid app %p\n", hdev->maxapplication);
	
	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT & ~HID_CONNECT_FF);
	if (ret) {
		dev_err(&hdev->dev, "hw start failed\n");
		goto err_free;
	}

//	hdev->quirks &= ~HID_QUIRK_NOGET;

/*	int usb_control_msg(usb_dev_handle *dev, int requesttype, 
			int request, int value, int index, 
			char *bytes, int size, int timeout);
00452   int len = usb_control_msg(hidif->dev_handle,
00453       USB_TYPE_CLASS + USB_RECIP_INTERFACE,
00454       HID_SET_IDLE,
00455       report_id + ((duration & 0xff) << 8),
00456       hidif->interface,
00457       NULL, 0, USB_TIMEOUT);
00458 
*/
/*	// set idle
	usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
		0x0a, USB_TYPE_CLASS | USB_RECIP_INTERFACE,
		(0x20<<8) | 0, interface->desc.bInterfaceNumber, 
		NULL, 0, USB_CTRL_SET_TIMEOUT);
*/

	//usbhid_submit_report(hid, sjoyff->report, USB_DIR_OUT);

printk(KERN_INFO "HID-EATON ret %d\n", ret);
	if (ret )
		goto err_free;
	ret = hid_connect(hdev, HID_CONNECT_HIDRAW | HID_CONNECT_HIDDEV | HID_CONNECT_HIDDEV_FORCE);
printk(KERN_INFO "HID-EATON ret %d\n", ret);
	if (ret) {
		hdev->ll_driver->stop(hdev);
		goto err_free;
//	return ret;
	}

	/*
	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT | HID_CONNECT_HIDDEV_FORCE);
	if (ret) {
		dev_err(&hdev->dev, "hw start failed\n");
		goto err_free;
	}
	*/
	

printk(KERN_INFO "HID-EATON probe OK\n");
	return 0;
err_stop:
	hid_hw_stop(hdev);
err_free:
	kfree(sc);
	return ret;
}

static void eaton_remove(struct hid_device *hdev)
{
printk(KERN_INFO "HID-EATON remove\n");
	hid_hw_stop(hdev);
	kfree(hid_get_drvdata(hdev));
}



static int eaton_raw_event(struct hid_device *hdev, struct hid_report *report,
		u8 *data, int size)
{
	printk(KERN_INFO "raw event %d len\n",size);
	return 0;
}

//	int (*event)(struct hid_device *hdev, struct hid_field *field,
//			struct hid_usage *usage, __s32 value);


#define EATON_VENDOR	0x188A	
#define EATON_PRODUCT	0x1101

static const struct hid_device_id eaton_devices[] = {
	{ HID_USB_DEVICE(EATON_VENDOR, EATON_PRODUCT) },
	{ }
};
MODULE_DEVICE_TABLE(hid, eaton_devices);

static struct hid_driver eaton_driver = {
	.name = "eaton",
	.id_table = eaton_devices,
	.probe = eaton_probe,
	.remove = eaton_remove,
//	.report_fixup = eaton_report_fixup,

	.raw_event = eaton_raw_event,
//	.event = eaton_event,
};

static int eaton_init(void)
{
printk(KERN_INFO "HID-EATON init\n");
	return hid_register_driver(&eaton_driver);
}

static void eaton_exit(void)
{
printk(KERN_INFO "HID-EATON exit\n");
	hid_unregister_driver(&eaton_driver);
}

module_init(eaton_init);
module_exit(eaton_exit);
MODULE_LICENSE("GPL");
