/**
 * \file openFC-CM4-USB-Driver.c
 * \author Vatsal Joshi
 * \date 01 May 2023
 * \version 0.1
 * \brief A driver to read sensor data and send commands to
 * an rp2040 based PI CM4 board which is used as flight controller
 * and the PI CM4 is used as the main processing unit.
 */
#include "picoSensActuHAT.h"

// Module Information
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vatsal Joshi");
MODULE_DESCRIPTION("A simple Hello world LKM!");
MODULE_VERSION("0.1");

/**
 * \brief Variables to hold device configuration info
*/
size_t numSens = 0;			// Number of sensors available onboard
size_t numActu = 0;			// Number of actuators available onboard
data_cfg_t sensorList[128]; // Configuration for each sensor available
data_cfg_t actuatList[128]; // Configuration for each actuator available

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
	size_t i;
	// toCopy: Amount of data to be copied.
	// notCopied: Amount of data that was not coped.
	int toCopy, notCopied;

	// Copy all the data to a temp buffer
	uint8_t outBuf[64];
	uint8_t *bufAdd = outBuf;
	strcpy(bufAdd, "picoSensActuHAT"); // WHOAMI
	bufAdd += 16;
	*bufAdd = numSens;	// NSENS
	++bufAdd;
	*bufAdd = 0;	// NACTU
	++bufAdd;
	for (i = 0; i < min(numSens,(size_t)5); ++i)	// sensorList
	{
		memcpy(bufAdd, &sensorList[i], sizeof(data_cfg_t));
		bufAdd += sizeof(data_cfg_t);
	}
	memcpy(bufAdd, buf, 21);

	// Decide how much data should be copied
	toCopy = min(sizeof(outBuf), bufSize); // For now, we know that there will be 21 bytes of data coming from the sensor acquisition system

	// Copy data to the user buffer
	notCopied = copy_to_user(userBuffer, outBuf, toCopy);

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
		printk(KERN_INFO "picoSensActuHAT: URB status not successful.\n");
		return;
	}

	// Resubmit a urb
	urb->status = 0;
	urb->iso_frame_desc[0].status = 0;
	err = usb_submit_urb(urb, GFP_KERNEL);
	if (err)
		printk(KERN_INFO "picoSensActuHAT: Error submitting urb from completion function.\n");
}

/**
 * \brief This function is called when USB device is attached.
 * \param[in,out] intrfc Pointer to the device interface in use. Helpful for implementing class drivers.
 * \param[in,out] id Pointer to an entry in device_id_table
 */
static int probe_function(struct usb_interface *intrfc, const struct usb_device_id *id)
{
	int ctrlTrnsfStatus = 0;
	char whoAmI[32];
	size_t i;
	int err1, err2;
	// Get the usb device to perform control transfers
	udev = usb_get_dev(interface_to_usbdev(intrfc));
	if (udev == NULL)
	{
		printk(KERN_ERR "picoSensActuHAT: Error getting device from the interface in probe() function.\n");
		return -1;
	}

	// Perform Control Transfers
	// Get WHOAMI from the device. It should be a 16 byte long string "picoSensActuHAT".
	ctrlTrnsfStatus = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0), USB_VEND_REQ_WHOAMI, USB_TYPE_VENDOR | USB_DIR_IN, 0, 0, (void *) whoAmI, 32, 1000);
	if (ctrlTrnsfStatus == 0)
	{
		printk(KERN_ERR "picoSensActuHAT: Device doesn't support WHOAMI request.\n");
		return -1;
	}
	else if (ctrlTrnsfStatus < 0)
	{
		printk(KERN_ERR "picoSensActuHAT: Error in a control transfer from probe() function 1, %d.\n", ctrlTrnsfStatus);
		return -1;
	}
	else if (strcmp(whoAmI, "picoSensActuHAT"))
	{
		printk(KERN_ERR "picoSensActuHAT: Device is identifying itself as %s.\n", whoAmI);
		return -1;
	}
	// Get NSENS. This value indicates the number of sensors available onboard. Max 128 is allowed.
	ctrlTrnsfStatus = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0), USB_VEND_REQ_NSENS, USB_TYPE_VENDOR | USB_DIR_IN, 0, 0, (void *)&numSens, 1, 1000);
	if (ctrlTrnsfStatus == 0)
	{
		printk(KERN_ERR "picoSensActuHAT: Device doesn't support NSENS request.\n");
		return -1;
	}
	else if (ctrlTrnsfStatus < 0)
	{
		printk(KERN_ERR "picoSensActuHAT: Error in a control transfer from probe() function 2, %d.\n", ctrlTrnsfStatus);
		return -1;
	}
	// Get info for all the sensors.
	for (i = 0; i < numSens; ++i)
	{
		ctrlTrnsfStatus = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0), USB_VEND_REQ_SENS_INFO, USB_TYPE_VENDOR | USB_DIR_IN, 0, i, (void *)&sensorList[i], sizeof(data_cfg_t), 1000);
		if (ctrlTrnsfStatus == 0)
		{
			printk(KERN_ERR "picoSensActuHAT: Error retrieving info for sensor number %lu. Possible error in the firmware.\n", i+1);
			return -1;
		}
		else if (ctrlTrnsfStatus < 0)
		{
			printk(KERN_ERR "picoSensActuHAT: Error in a control transfer from probe() function 3, %d.\n", ctrlTrnsfStatus);
			return -1;
		}
	}

	// Device file can be created, now that we have all the necessary info.

	printk(KERN_INFO "picoSensActuHAT: Raspberry Pi Pico Sensor Acquisition Device was attached.\n");

	ep = intrfc->cur_altsetting->endpoint[0].desc;

	// Sumbit two urbs so that they are double buffered.
	urb[0] = usb_alloc_urb(1,GFP_KERNEL);
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
			printk(KERN_INFO "picoSensActuHAT: Could not allocate char device region.\n");
			return -1;
		}
		printk(KERN_INFO "picoSensActuHAT: CHAR device Major: %d, Minor: %d.\n", MAJOR(imuDevNum), MINOR(imuDevNum));

		// Create device class
		if ((imuDevClass = class_create(THIS_MODULE, IMU_DRIVER_CLASS)) == NULL)
		{
			printk(KERN_INFO "picoSensActuHAT: Could not create device class.\n");
			unregister_chrdev(MAJOR(imuDevNum), IMU_DRIVER_NAME);
			return -1;
		}

		// Create device file
		if (device_create(imuDevClass, NULL, imuDevNum, NULL, IMU_DRIVER_NAME) == NULL)
		{
			printk(KERN_INFO "picoSensActuHAT: Could not create device file.\n");
			class_destroy(imuDevClass);
			unregister_chrdev(MAJOR(imuDevNum), IMU_DRIVER_NAME);
			return -1;
		}

		// Initialize device file
		cdev_init(&imuDev, &imu_fops);

		// Register the device with the kernel
		if (cdev_add(&imuDev, imuDevNum, 1) < 0)
		{
			printk(KERN_INFO "picoSensActuHAT: Could not register device with the kernel.\n");
			device_destroy(imuDevClass, imuDevNum);
			class_destroy(imuDevClass);
			unregister_chrdev(MAJOR(imuDevNum), IMU_DRIVER_NAME);
			return -1;
		}
	}
	else
	{
		printk(KERN_INFO "picoSensActuHAT: Error submitting initial urb, %d.\n", err1);
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
	printk(KERN_INFO "picoSensActuHAT: Raspberry Pi Pico Sensor Acquisition Device was detached.\n");
	cdev_del(&imuDev);
	device_destroy(imuDevClass, imuDevNum);
	class_destroy(imuDevClass);
	unregister_chrdev(MAJOR(imuDevNum), IMU_DRIVER_NAME);
}

/**
 * \brief Instance of this USB device driver that will be provided to USB subsystem.
*/
static struct usb_driver picoSensActuHATDriver = {
	.name = "Pico Sensor Acquisition Device Driver",
	.id_table = device_id_table,
	.probe = probe_function,
	.disconnect = disconnect_function,
};

static int __init picoSensActuHAT_init(void)
{
	int ret;
	printk(KERN_INFO "picoSensActuHAT: Module Initialization.\n");
	ret = usb_register(&picoSensActuHATDriver);
	if (ret)
	{
		printk(KERN_ERR "picoSensActuHAT: Error registering the driver.\n");
		return -ret;
	}
	return 0;
}

static void __exit picoSensActuHAT_exit(void)
{
	printk(KERN_INFO "picoSensActuHAT: Module Deinitialization.\n");
	usb_deregister(&picoSensActuHATDriver);
}

module_init(picoSensActuHAT_init);
module_exit(picoSensActuHAT_exit);
