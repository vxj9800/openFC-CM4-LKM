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
#include <linux/fs.h>	/* Needed for file operations. */
#include <linux/cdev.h>	/* Needed for creating a character device. */

// Module Information
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vatsal Joshi");
MODULE_DESCRIPTION("A simple Hello world LKM!");
MODULE_VERSION("0.1");

/**
 * \brief Variables to hold IMU character device data
*/
#define IMU_DRIVER_NAME "picoIMU"
#define IMU_DRIVER_CLASS "picoSensor"
static dev_t imuDevNum;
static struct class *imuDevClass;
static struct cdev imuDev;
u8 buf[21];

/**
 * \brief This function is called when the char device file for IMU is opened.
 * \param[in] devFile Pointer to the inode for the device file.
 * \param[in] devInstance Pointer to the file instance of the inode described by devFile.
*/
static int imu_open(struct inode *devFile, struct file *devInstance)
{
	printk("imuDev: File open was called.\n");
	return 0;
}

/**
 * \brief Called when user tries to read from the device file.
*/
static ssize_t imu_read(struct file *imuFile, char *userBuffer, size_t bufSize, loff_t *offset)
{
	// toCopy: Amount of data to be copied.
	// notCopied: Amount of data that was not coped.
	int toCopy, notCopied;

	// Decide how much data should be copied
	toCopy = min((size_t)21, bufSize); // For now, we know that there will be 21 bytes of data coming from the sensor acquisition system

	// Copy data to the user buffer
	notCopied = copy_to_user(userBuffer, buf, toCopy);

	// Calculate the number of bytes that were copied
	return (toCopy - notCopied);
}

/**
 * \brief This function is called when the char device file for IMU is closed.
 * \param[in] devFile Pointer to the inode for the device file.
 * \param[in] devInstance Pointer to the file instance of the inode described by devFile.
 */
static int imu_close(struct inode *devFile, struct file *devInstance)
{
	printk("imuDev: File close was called.\n");
	return 0;
}

/**
 * \brief Structure to hold file operations information for IMU data.
*/
const struct file_operations imu_fops = {
	.owner = THIS_MODULE,
	.open = imu_open,
	.read = imu_read,
	.release = imu_close,
};

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

/**
 * \brief Global variables to store usb device information
 */
struct urb *urb[2];
struct usb_endpoint_descriptor ep;
struct usb_device *udev;

// Make the device available to the user-space
MODULE_DEVICE_TABLE(usb, device_id_table);

void urb_complete_function(struct urb *urb)
{
	int err;

	// Check the status of URB
	if (urb->status || urb->iso_frame_desc[0].status)
	{
		printk(KERN_INFO "picoSensAcq: URB status not successful.\n");
		return;
	}

	// Resubmit a urb
	urb->status = 0;
	urb->iso_frame_desc[0].status = 0;
	err = usb_submit_urb(urb, GFP_KERNEL);
	if (err)
		printk(KERN_INFO "picoSensAcq: Error submitting urb from completion function.\n");
}

/**
 * \brief This function is called when USB device is attached.
 * \param[in,out] intrfc Pointer to the device interface in use. Helpful for implementing class drivers.
 * \param[in,out] id Pointer to an entry in device_id_table
 */
static int probe_function(struct usb_interface *intrfc, const struct usb_device_id *id)
{
	int err1, err2;

	printk(KERN_INFO "picoSensAcq: Raspberry Pi Pico Sensor Acquisition Device was attached.\n");

	ep = intrfc->cur_altsetting->endpoint[0].desc;

	printk(KERN_INFO "picoSensAcq: Endpoint Address %u.\n", ep.bEndpointAddress);

	// Sumbit two urbs so that they are double buffered.
	urb[0] = usb_alloc_urb(1,GFP_KERNEL);
	udev = usb_get_dev(interface_to_usbdev(intrfc));
	urb[0]->dev = udev;
	urb[0]->pipe = usb_rcvisocpipe(udev, ep.bEndpointAddress);
	urb[0]->interval = 1;
	urb[0]->transfer_flags = URB_ISO_ASAP;
	urb[0]->transfer_buffer = buf;
	urb[0]->transfer_buffer_length = 21;
	urb[0]->complete = urb_complete_function;
	urb[0]->number_of_packets = 1;
	urb[0]->iso_frame_desc[0].offset = 0;
	urb[0]->iso_frame_desc[0].length = 21;

	err1 = usb_submit_urb(urb[0], GFP_KERNEL);

	urb[1] = usb_alloc_urb(1, GFP_KERNEL);
	udev = usb_get_dev(interface_to_usbdev(intrfc));
	urb[1]->dev = udev;
	urb[1]->pipe = usb_rcvisocpipe(udev, ep.bEndpointAddress);
	urb[1]->interval = 1;
	urb[1]->transfer_flags = URB_ISO_ASAP;
	urb[1]->transfer_buffer = buf;
	urb[1]->transfer_buffer_length = 21;
	urb[1]->complete = urb_complete_function;
	urb[1]->number_of_packets = 1;
	urb[1]->iso_frame_desc[0].offset = 0;
	urb[1]->iso_frame_desc[0].length = 21;

	err2 = usb_submit_urb(urb[1], GFP_KERNEL);

	// If urb is submitted successfully, then initialize the IMU char device
	if (!err1 && !err2)
	{
		// Allocate device number
		if (alloc_chrdev_region(&imuDevNum, 0, 1, IMU_DRIVER_NAME))
		{
			printk(KERN_INFO "picoSensAcq: Could not allocate char device region.\n");
			return -1;
		}
		printk(KERN_INFO "picoSensAcq: CHAR device Major: %d, Minor: %d.\n", MAJOR(imuDevNum), MINOR(imuDevNum));

		// Create device class
		if ((imuDevClass = class_create(THIS_MODULE, IMU_DRIVER_CLASS)) == NULL)
		{
			printk(KERN_INFO "picoSensAcq: Could not create device class.\n");
			unregister_chrdev(MAJOR(imuDevNum), IMU_DRIVER_NAME);
			return -1;
		}

		// Create device file
		if (device_create(imuDevClass, NULL, imuDevNum, NULL, IMU_DRIVER_NAME) == NULL)
		{
			printk(KERN_INFO "picoSensAcq: Could not create device file.\n");
			class_destroy(imuDevClass);
			unregister_chrdev(MAJOR(imuDevNum), IMU_DRIVER_NAME);
			return -1;
		}

		// Initialize device file
		cdev_init(&imuDev, &imu_fops);

		// Register the device with the kernel
		if (cdev_add(&imuDev, imuDevNum, 1) < 0)
		{
			printk(KERN_INFO "picoSensAcq: Could not register device with the kernel.\n");
			device_destroy(imuDevClass, imuDevNum);
			class_destroy(imuDevClass);
			unregister_chrdev(MAJOR(imuDevNum), IMU_DRIVER_NAME);
			return -1;
		}
	}
	else
	{
		printk(KERN_INFO "picoSensAcq: Error submitting initial urb, %d.\n", err1);
		return err1;
	}

	return 0;
}

/**
 * \brief This function is called when USB device is detached.
 * \param[in,out] intrfc Pointer to the device interface in use. Helpful for implementing class drivers.
 */
static void disconnect_function(struct usb_interface *intrfc)
{
	printk(KERN_INFO "picoSensAcq: Raspberry Pi Pico Sensor Acquisition Device was detached.\n");
	cdev_del(&imuDev);
	device_destroy(imuDevClass, imuDevNum);
	class_destroy(imuDevClass);
	unregister_chrdev(MAJOR(imuDevNum), IMU_DRIVER_NAME);
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
