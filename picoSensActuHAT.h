#ifndef __PICO_SENS_ACTU_HAT_H__
#define __PICO_SENS_ACTU_HAT_H__

#include <linux/module.h> /* Needed by all modules */
#include <linux/kernel.h> /* Needed for KERN_INFO */
#include <linux/init.h>   /* Needed for the macros */
#include <linux/usb.h>    /* Needed for usb macros and functions */
#include <linux/slab.h>   /* Needed for memory allocation and such. */
#include <linux/fs.h>     /* Needed for file operations. */
#include <linux/cdev.h>   /* Needed for creating a character device. */

// Definitions for vendor specific requests
#define USB_VEND_REQ_WHOAMI 0
#define USB_VEND_REQ_NSENS 1
#define USB_VEND_REQ_NACTU 2
#define USB_VEND_REQ_SENS_INFO 3
#define USB_VEND_REQ_ACTU_INFO 4

// Structure to hold sensor configuration data
typedef struct
{
    uint8_t sensType;
    uint8_t dataType;
    uint8_t nDataVal;
    uint16_t samplingRate;
} __attribute__((packed)) data_cfg_t;

#endif // __PICO_SENS_ACTU_HAT_H__