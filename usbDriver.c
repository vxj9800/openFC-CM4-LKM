/**
 * \file picoSensActuHAT_Driver.c
 * \author Vatsal Joshi
 * \date 28 May 2023
 * \version 0.1
 * \brief A driver to receive sensor data and send actuator commands to
 * an rp2040 based PI CM4 carrier board which is used as flight controller
 * and the PI CM4 is used as the main processing unit.
 */
#include "picoSensActuHAT.h"

// Module Information
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vatsal Joshi");
MODULE_DESCRIPTION("Driver for picoSensActuHAT.");
MODULE_VERSION("0.1");

///////////////////////////////////
// Globals for char devices fops //
///////////////////////////////////
/**
 * \brief Variables to hold character devices info. There will be four char
 * devices, 1. sensCnfg, 2. ActuCnfg, 3. sensData and 4. actuData. The 'sensCnfg'
 * device will contain the info received through control transfers for NSENS and
 * SENS_INFO vendor requests. Similarly, 'actuCnfg' device will contain the info
 * received through control transfers for NACTU and ACTU_INFO vendor requests.
 */
#define DEV_CLASS_NAME "sensActuHAT"
#define DEV_REGION_NAME "picoSensActu"
static dev_t hatDevNum, sensCnfgDevNum, sensDataDevNum, actuCnfgDevNum, actuDataDevNum;
static struct class *sensActuClass;
static struct cdev sensCnfg;
static struct cdev sensData;
static struct cdev actuCnfg;
static struct cdev actuData;
u8 buf[21];

/**
 * \brief These are buffers for the four device files that will be created.
*/
static struct {
	uint8_t numSens;		  // Number of sensors available onboard
	data_cfg_t sensList[128]; // Configuration for each sensor available
} __attribute__((packed)) sensCnfgBuf;
static struct
{
	uint8_t numActu;		  // Number of actuators available onboard
	data_cfg_t actuList[128]; // Configuration for each actuator available
} __attribute__((packed)) actuCnfgBuf;
static uint8_t sensDataBuf[1024];	 // Buffer to hold sensor data
static uint8_t actuDataBuf[1024];	 // Buffer to hold actuator data
static uint16_t sensDataOffset[128]; // Offset for the sensor data in sensDataBuf
static uint16_t actuDataOffset[128]; // Offset for the actuator data in actuDataBuf




//////////////////////////////
// Globals for USB host API //
//////////////////////////////
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

/**
 * \brief Global variables to store usb device information
 */
struct urb *urb[2];
struct usb_endpoint_descriptor ep;
struct usb_device *udev;

/**
 * \brief These arrays hold the raw data received/sent through USB Iso transfers.
*/
uint8_t usbRecBuf[1023];
uint8_t usbSndBuf[1023];




////////////////////////////////////
// sensCnfg Char device functions //
////////////////////////////////////
/**
 * \brief This function is called when the char device file for IMU is opened.
 * \param[in] devFile Pointer to the inode for the device file.
 * \param[in] devInstance Pointer to the file instance of the inode described by devFile.
*/
static int sensCnfg_open(struct inode *devFile, struct file *devInstance)
{
	printk("sensCnfg: File open was called.\n");
	return 0;
}

/**
 * \brief Called when user tries to read from the device file.
*/
static ssize_t sensCnfg_read(struct file *devFile, char __user *userBuffer, size_t bufSize, loff_t *offset)
{
	// toCopy: Amount of data to be copied.
	// notCopied: Amount of data that was not coped.
	int toCopy, notCopied;

	// Decide how much data should be copied
	toCopy = min(sizeof(sensCnfgBuf) - *offset, (unsigned long long)bufSize);
	if (toCopy <= 0)
		return 0;

	// Copy data to the user buffer
	notCopied = copy_to_user(userBuffer, &sensCnfgBuf + *offset, toCopy);
	if (notCopied <= 0)
		return -EFAULT;

	// Calculate the number of bytes that were copied
	*offset += toCopy - notCopied;
	return (toCopy - notCopied);
}

/**
 * \brief This function is called when the char device file for IMU is closed.
 * \param[in] devFile Pointer to the inode for the device file.
 * \param[in] devInstance Pointer to the file instance of the inode described by devFile.
 */
static int sensCnfg_close(struct inode *devFile, struct file *devInstance)
{
	printk("sensCnfg: File close was called.\n");
	return 0;
}

/**
 * \brief Structure to hold file operations information for sensCnfg.
 */
const struct file_operations sensCnfg_fops = {
	.owner = THIS_MODULE,
	.open = sensCnfg_open,
	.read = sensCnfg_read,
	.release = sensCnfg_close,
};




////////////////////////////////////
// sensData Char device functions //
////////////////////////////////////
/**
 * \brief This function is called when the char device file for IMU is opened.
 * \param[in] devFile Pointer to the inode for the device file.
 * \param[in] devInstance Pointer to the file instance of the inode described by devFile.
 */
static int sensData_open(struct inode *devFile, struct file *devInstance)
{
	printk("sensData: File open was called.\n");
	return 0;
}

/**
 * \brief Called when user tries to read from the device file.
 */
static ssize_t sensData_read(struct file *devFile, char __user *userBuffer, size_t bufSize, loff_t *offset)
{
	// toCopy: Amount of data to be copied.
	// notCopied: Amount of data that was not coped.
	int toCopy, notCopied;

	// Decide how much data should be copied
	toCopy = min(sizeof(sensDataBuf) - *offset, (unsigned long long)bufSize);
	if (toCopy <= 0)
		return 0;

	// Copy data to the user buffer
	notCopied = copy_to_user(userBuffer, &sensDataBuf + *offset, toCopy);
	if (notCopied <= 0)
		return -EFAULT;

	// Calculate the number of bytes that were copied
	*offset += toCopy - notCopied;
	return (toCopy - notCopied);
}

/**
 * \brief This function is called when the char device file for IMU is closed.
 * \param[in] devFile Pointer to the inode for the device file.
 * \param[in] devInstance Pointer to the file instance of the inode described by devFile.
 */
static int sensData_close(struct inode *devFile, struct file *devInstance)
{
	printk("sensData: File close was called.\n");
	return 0;
}

/**
 * \brief Structure to hold file operations information for sensData.
 */
const struct file_operations sensData_fops = {
	.owner = THIS_MODULE,
	.open = sensData_open,
	.read = sensData_read,
	.release = sensData_close,
};





////////////////////////////////////
// actuCnfg Char device functions //
////////////////////////////////////
/**
 * \brief This function is called when the char device file for IMU is opened.
 * \param[in] devFile Pointer to the inode for the device file.
 * \param[in] devInstance Pointer to the file instance of the inode described by devFile.
 */
static int actuCnfg_open(struct inode *devFile, struct file *devInstance)
{
	printk("actuCnfg: File open was called.\n");
	return 0;
}

/**
 * \brief Called when user tries to read from the device file.
 */
static ssize_t actuCnfg_read(struct file *devFile, char __user *userBuffer, size_t bufSize, loff_t *offset)
{
	// toCopy: Amount of data to be copied.
	// notCopied: Amount of data that was not coped.
	int toCopy, notCopied;

	// Decide how much data should be copied
	toCopy = min(sizeof(actuCnfgBuf) - *offset, (unsigned long long)bufSize);
	if (toCopy <= 0)
		return 0;

	// Copy data to the user buffer
	notCopied = copy_to_user(userBuffer, &actuCnfgBuf + *offset, toCopy);
	if (notCopied <= 0)
		return -EFAULT;

	// Calculate the number of bytes that were copied
	*offset += toCopy - notCopied;
	return (toCopy - notCopied);
}

/**
 * \brief This function is called when the char device file for IMU is closed.
 * \param[in] devFile Pointer to the inode for the device file.
 * \param[in] devInstance Pointer to the file instance of the inode described by devFile.
 */
static int actuCnfg_close(struct inode *devFile, struct file *devInstance)
{
	printk("actuCnfg: File close was called.\n");
	return 0;
}

/**
 * \brief Structure to hold file operations information for actuCnfg.
 */
const struct file_operations actuCnfg_fops = {
	.owner = THIS_MODULE,
	.open = actuCnfg_open,
	.read = actuCnfg_read,
	.release = actuCnfg_close,
};






////////////////////////////////////
// actuData Char device functions //
////////////////////////////////////
/**
 * \brief This function is called when the char device file for IMU is opened.
 * \param[in] devFile Pointer to the inode for the device file.
 * \param[in] devInstance Pointer to the file instance of the inode described by devFile.
 */
static int actuData_open(struct inode *devFile, struct file *devInstance)
{
	printk("actuData: File open was called.\n");
	return 0;
}

/**
 * \brief Called when user tries to read from the device file.
 */
static ssize_t actuData_write(struct file *devFile, const char __user *userBuffer, size_t bufSize, loff_t *offset)
{
	// toCopy: Amount of data to be copied.
	// notCopied: Amount of data that was not coped.
	int toCopy, notCopied;

	// Decide how much data should be copied
	toCopy = min(sizeof(actuDataBuf) - *offset, (unsigned long long)bufSize);
	if (toCopy <= 0)
		return 0;

	// Copy data to the user buffer
	notCopied = copy_from_user(&actuDataBuf + *offset, userBuffer, toCopy);
	if (notCopied <= 0)
		return -EFAULT;

	// Calculate the number of bytes that were copied
	*offset += toCopy - notCopied;
	return (toCopy - notCopied);
}

/**
 * \brief This function is called when the char device file for IMU is closed.
 * \param[in] devFile Pointer to the inode for the device file.
 * \param[in] devInstance Pointer to the file instance of the inode described by devFile.
 */
static int actuData_close(struct inode *devFile, struct file *devInstance)
{
	printk("actuData: File close was called.\n");
	return 0;
}

/**
 * \brief Structure to hold file operations information for actuData.
 */
const struct file_operations actuData_fops = {
	.owner = THIS_MODULE,
	.open = actuData_open,
	.write = actuData_write,
	.release = actuData_close,
};




////////////////////////////
// USB Host API functions //
////////////////////////////
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
		printk(KERN_ERR "picoSensActuHAT: Error in a control transfer from probe() function, %d.\n", ctrlTrnsfStatus);
		return -1;
	}
	else if (strcmp(whoAmI, "picoSensActuHAT"))
	{
		printk(KERN_ERR "picoSensActuHAT: Device is identifying itself as %s.\n", whoAmI);
		return -1;
	}
	// Get NSENS. This value indicates the number of sensors available onboard. Max 128 is allowed.
	ctrlTrnsfStatus = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0), USB_VEND_REQ_NSENS, USB_TYPE_VENDOR | USB_DIR_IN, 0, 0, (void *)&sensCnfgBuf.numSens, 1, 1000);
	if (ctrlTrnsfStatus == 0)
	{
		printk(KERN_ERR "picoSensActuHAT: Device doesn't support NSENS request.\n");
		return -1;
	}
	else if (ctrlTrnsfStatus < 0)
	{
		printk(KERN_ERR "picoSensActuHAT: Error in a control transfer from probe() function, %d.\n", ctrlTrnsfStatus);
		return -1;
	}
	// Get info for all the sensors.
	for (i = 0; i < sensCnfgBuf.numSens; ++i)
	{
		ctrlTrnsfStatus = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0), USB_VEND_REQ_SENS_INFO, USB_TYPE_VENDOR | USB_DIR_IN, 0, i, (void *)&sensCnfgBuf.sensList[i], sizeof(data_cfg_t), 1000);
		if (ctrlTrnsfStatus == 0)
		{
			printk(KERN_ERR "picoSensActuHAT: Error retrieving info for sensor number %lu. Possible error in the firmware.\n", i+1);
			return -1;
		}
		else if (ctrlTrnsfStatus < 0)
		{
			printk(KERN_ERR "picoSensActuHAT: Error in a control transfer from probe() function, %d.\n", ctrlTrnsfStatus);
			return -1;
		}
		if (i)
			sensDataOffset[i] = sensDataOffset[i - 1] + sensCnfgBuf.sensList[i - 1].nDataVal * (sensCnfgBuf.sensList[i - 1].dataType & DTYPE_WIDTH_MASK);
	}
	// // Get NACTU. This value indicates the number of sensors available onboard. Max 128 is allowed.
	// ctrlTrnsfStatus = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0), USB_VEND_REQ_NACTU, USB_TYPE_VENDOR | USB_DIR_IN, 0, 0, (void *)&numActu, 1, 1000);
	// if (ctrlTrnsfStatus == 0)
	// {
	// 	printk(KERN_ERR "picoSensActuHAT: Device doesn't support NACTU request.\n");
	// 	return -1;
	// }
	// else if (ctrlTrnsfStatus < 0)
	// {
	// 	printk(KERN_ERR "picoSensActuHAT: Error in a control transfer from probe() function, %d.\n", ctrlTrnsfStatus);
	// 	return -1;
	// }
	// // Get info for all the actuators.
	// for (i = 0; i < numActu; ++i)
	// {
	// 	ctrlTrnsfStatus = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0), USB_VEND_REQ_ACTU_INFO, USB_TYPE_VENDOR | USB_DIR_IN, 0, i, (void *)&sensorList[i], sizeof(data_cfg_t), 1000);
	// 	if (ctrlTrnsfStatus == 0)
	// 	{
	// 		printk(KERN_ERR "picoSensActuHAT: Error retrieving info for sensor number %lu. Possible error in the firmware.\n", i + 1);
	// 		return -1;
	// 	}
	// 	else if (ctrlTrnsfStatus < 0)
	// 	{
	// 		printk(KERN_ERR "picoSensActuHAT: Error in a control transfer from probe() function, %d.\n", ctrlTrnsfStatus);
	// 		return -1;
	// 	}
	// if (i)
	// 	actuDataOffset[i] = actuDataOffset[i - 1] + actuorList[i - 1].nDataVal * (actuorList[i - 1].dataType & DTYPE_WIDTH_MASK);
	// }

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

	// If urb is submitted successfully, then initialize the char devices
	if (!err1 && !err2)
	{
		// Create sensCnfg device
		if (IS_ERR_VALUE(device_create(sensActuClass, NULL, sensCnfgDevNum, NULL, "sensActuHAT!sensCnfg")))
		{
			printk(KERN_INFO "picoSensActuHAT: Could not create sensCnfg device file.\n");
			goto sensCnfgDevCreateErr;
		}
		cdev_init(&sensCnfg, &sensCnfg_fops);
		if (cdev_add(&sensCnfg, sensCnfgDevNum, 1) < 0)
		{
			printk(KERN_INFO "picoSensActuHAT: Could not register sensCnfg device with the kernel.\n");
			goto sensCnfgDevAddErr;
		}

		// Create sensData device
		if (IS_ERR_VALUE(device_create(sensActuClass, NULL, sensDataDevNum, NULL, "sensActuHAT!sensData")))
		{
			printk(KERN_INFO "picoSensActuHAT: Could not create sensData device file.\n");
			goto sensDataDevCreateErr;
		}
		cdev_init(&sensData, &sensData_fops);
		if (cdev_add(&sensData, sensDataDevNum, 1) < 0)
		{
			printk(KERN_INFO "picoSensActuHAT: Could not register sensData device with the kernel.\n");
			goto sensDataDevAddErr;
		}

		// Create actuCnfg device
		if (IS_ERR_VALUE(device_create(sensActuClass, NULL, actuCnfgDevNum, NULL, "sensActuHAT!actuCnfg")))
		{
			printk(KERN_INFO "picoSensActuHAT: Could not create actuCnfg device file.\n");
			goto actuCnfgDevCreateErr;
		}
		cdev_init(&actuCnfg, &actuCnfg_fops);
		if (cdev_add(&actuCnfg, actuCnfgDevNum, 1) < 0)
		{
			printk(KERN_INFO "picoSensActuHAT: Could not register actuCnfg device with the kernel.\n");
			goto actuCnfgDevAddErr;
		}

		// Create actuData device
		if (IS_ERR_VALUE(device_create(sensActuClass, NULL, actuDataDevNum, NULL, "sensActuHAT!actuData")))
		{
			printk(KERN_INFO "picoSensActuHAT: Could not create actuData device file.\n");
			goto actuDataDevCreateErr;
		}
		cdev_init(&actuData, &actuData_fops);
		if (cdev_add(&actuData, actuDataDevNum, 1) < 0)
		{
			printk(KERN_INFO "picoSensActuHAT: Could not register actuData device with the kernel.\n");
			goto actuDataDevAddErr;
		}

		goto devFilesSuccess;

	actuDataDevAddErr:
		device_destroy(sensActuClass, actuDataDevNum);
	actuDataDevCreateErr:
		cdev_del(&actuCnfg);
	actuCnfgDevAddErr:
		device_destroy(sensActuClass, actuCnfgDevNum);
	actuCnfgDevCreateErr:
		cdev_del(&sensData);
	sensDataDevAddErr:
		device_destroy(sensActuClass, sensDataDevNum);
	sensDataDevCreateErr:
		cdev_del(&sensCnfg);
	sensCnfgDevAddErr:
		device_destroy(sensActuClass, sensCnfgDevNum);
	sensCnfgDevCreateErr:
		return -1;
	devFilesSuccess:
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
	cdev_del(&actuData);
	device_destroy(sensActuClass, actuDataDevNum);
	cdev_del(&actuCnfg);
	device_destroy(sensActuClass, actuCnfgDevNum);
	cdev_del(&sensData);
	device_destroy(sensActuClass, sensDataDevNum);
	cdev_del(&sensCnfg);
	device_destroy(sensActuClass, sensCnfgDevNum);
}

/**
 * \brief Instance of this USB device driver that will be provided to USB subsystem.
 */
static struct usb_driver picoSensActuHATDriver = {
	.name = "Pico Sensor Actuator HAT Driver",
	.id_table = device_id_table,
	.probe = probe_function,
	.disconnect = disconnect_function,
};





///////////////////////////////////
// Linux Kernel Module Functions //
///////////////////////////////////
static int __init picoSensActuHAT_init(void)
{
	int ret;
	printk(KERN_INFO "picoSensActuHAT: Module Initialization.\n");

	ret = usb_register(&picoSensActuHATDriver);
	if (ret)
	{
		printk(KERN_ERR "picoSensActuHAT: Error registering the usb driver.\n");
		return -ret;
	}

	// Allocate char device region, provides device major number
	if (alloc_chrdev_region(&hatDevNum, 0, 1, DEV_REGION_NAME))
	{
		printk(KERN_INFO "picoSensActuHAT: Could not allocate char device region.\n");
		return -1;
	}

	// Create device class
	if ((sensActuClass = class_create(THIS_MODULE, DEV_CLASS_NAME)) == NULL)
	{
		printk(KERN_INFO "picoSensActuHAT: Could not create device class.\n");
		unregister_chrdev_region(hatDevNum, 1);
		return -1;
	}

	// Define device numbers for char devices
	sensCnfgDevNum = MKDEV(MAJOR(hatDevNum), 0);
	sensDataDevNum = MKDEV(MAJOR(hatDevNum), 1);
	actuCnfgDevNum = MKDEV(MAJOR(hatDevNum), 2);
	actuDataDevNum = MKDEV(MAJOR(hatDevNum), 3);

	return 0;
}

static void __exit picoSensActuHAT_exit(void)
{
	printk(KERN_INFO "picoSensActuHAT: Module Deinitialization.\n");
	class_destroy(sensActuClass);
	unregister_chrdev_region(hatDevNum, 1);
	usb_deregister(&picoSensActuHATDriver);
}

module_init(picoSensActuHAT_init);
module_exit(picoSensActuHAT_exit);
