/**
 * \file openFC-CM4-USB-Driver.c
 * \author Vatsal Joshi
 * \date 01 May 2023
 * \version 0.1
 * \brief A driver to read sensor data and send commands to
 * an rp2040 based PI CM4 board which is used as flight controller
 * and the PI CM4 is used as the main processing unit.
 */
#include <linux/module.h>	 /* Needed by all modules */
#include <linux/kernel.h>	 /* Needed for KERN_INFO */
#include <linux/init.h>	 /* Needed for the macros */
#include <linux/usb.h>	/* Needed for usb macros and functions */
#include <linux/slab.h> /* Needed for memory allocation and such. */

// Module Information
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vatsal Joshi");
MODULE_DESCRIPTION("A simple Hello world LKM!");
MODULE_VERSION("0.1");

// USB device info from the firmware
#define VENDOR_ID 0x0000  // Vendor ID set in the Firmware
#define PRODUCT_ID 0x0001 // Product ID set in the Firmware

/**
 * \brief Table of supported USB devices
 */
static struct usb_device_id device_id_table[] = {
	{USB_DEVICE(VENDOR_ID, PRODUCT_ID)},
	{},
};

// Make the device available to the user-space
MODULE_DEVICE_TABLE(usb, device_id_table);

// Temperory buffer to hold data
u8 buf[21];

void urb_complete_function(struct urb *urb)
{
	int i;
	i = 0;
	printk(KERN_INFO "picoSensAcq: URB transfer successful %u.", urb->actual_length);
	printk(KERN_INFO "picoSensAcq: xfr1: time: %20llu accX: %9d accY: %9d accZ: %9d gyrX: %9d gyrY: %9d gyrZ: %9d\n", *(uint64_t *)(buf + i * 21 + 1), *(int16_t *)(buf + i * 21 + 9), *(int16_t *)(buf + i * 21 + 11), *(int16_t *)(buf + i * 21 + 13), *(int16_t *)(buf + i * 21 + 15), *(int16_t *)(buf + i * 21 + 17), *(int16_t *)(buf + i * 21 + 19));
}

/**
 * \brief This function is called when USB device is attached.
 * \param[in,out] intrfc Pointer to the device interface in use. Helpful for implementing class drivers.
 * \param[in,out] id Pointer to an entry in device_id_table
 */
static int probe_function(struct usb_interface *intrfc, const struct usb_device_id *id)
{
	struct usb_endpoint_descriptor ep;
	struct urb *urb;
	struct usb_device *udev;

	printk(KERN_INFO "picoSensAcq: Raspberry Pi Pico Sensor Acquisition Device was attached.\n");

	ep = intrfc->cur_altsetting->endpoint[0].desc;

	printk(KERN_INFO "picoSensAcq: Endpoint Address %u.\n", ep.bEndpointAddress);

	// For now, try to get a single data-set from the device
	urb = usb_alloc_urb(1,GFP_KERNEL);

	udev = usb_get_dev(interface_to_usbdev(intrfc));
	urb->dev = udev;
	urb->pipe = usb_rcvisocpipe(udev, ep.bEndpointAddress);
	urb->interval = 1;
	urb->transfer_flags = URB_ISO_ASAP;
	urb->transfer_buffer = buf;
	urb->transfer_buffer_length = 21;
	urb->complete = urb_complete_function;
	urb->number_of_packets = 1;
	urb->iso_frame_desc[0].offset = 0;
	urb->iso_frame_desc[0].length = 21;

	usb_submit_urb(urb, GFP_KERNEL);

	return 0;
}

/**
 * \brief This function is called when USB device is detached.
 * \param[in,out] intrfc Pointer to the device interface in use. Helpful for implementing class drivers.
 */
static void disconnect_function(struct usb_interface *intrfc)
{
	printk(KERN_INFO "picoSensAcq: Raspberry Pi Pico Sensor Acquisition Device was detached.\n");
}

/**
 * \brief Instance of this USB device driver that will be provided to USB subsystem.
*/
static struct usb_driver picoSensAcqDriver = {
	.name = "Pico Sensor Acquisition Device Driver",
	.id_table = device_id_table,
	.probe = probe_function,
	.disconnect = disconnect_function,
};

static int __init picoSensAcq_init(void)
{
	int ret;
	printk(KERN_INFO "picoSensAcq: Module Initialization.\n");
	ret = usb_register(&picoSensAcqDriver);
	if (ret)
	{
		printk(KERN_ERR "picoSensAcq: Error registering the driver.\n");
		return -ret;
	}
	return 0;
}

static void __exit picoSensAcq_exit(void)
{
	printk(KERN_INFO "picoSensAcq: Module Deinitialization.\n");
	usb_deregister(&picoSensAcqDriver);
}

module_init(picoSensAcq_init);
module_exit(picoSensAcq_exit);
