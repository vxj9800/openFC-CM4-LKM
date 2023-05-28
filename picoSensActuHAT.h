#ifndef __PICO_SENS_ACTU_HAT_H__
#define __PICO_SENS_ACTU_HAT_H__

#include <linux/module.h> /* Needed by all modules */
#include <linux/kernel.h> /* Needed for KERN_INFO */
#include <linux/init.h>   /* Needed for the macros */
#include <linux/usb.h>    /* Needed for usb macros and functions */
#include <linux/slab.h>   /* Needed for memory allocation and such. */
#include <linux/fs.h>     /* Needed for file operations. */
#include <linux/cdev.h>   /* Needed for creating a character device. */

// Following definitions are used for the dataType field  of the
// data_cfg_t  which indicates what is the data type of the values
// for the sensor/actuator
// bits 0:3 define the width of the data type in bytes
// bit 4 defines the number type, i.e. integer or floating point
// bit 5 defines whether the number is signed or unsigned, ignored for float
#define DTYPE_WIDTH_MASK 0xf
#define DTYPE_1BYTES (1 << 0)
#define DTYPE_2BYTES (1 << 1)
#define DTYPE_4BYTES (1 << 2)
#define DTYPE_8BYTES (1 << 3)
#define DTYPE_NUM_TYPE_MASK 0x10
#define DTYPE_FP (1 << 4)
#define DTYPE_SIGN_MASK 0x20
#define DTYPE_SGND (1 << 5)
#define DTYPE_UINT8 (DTYPE_1BYTES)
#define DTYPE_UINT16 (DTYPE_2BYTES)
#define DTYPE_UINT32 (DTYPE_4BYTES)
#define DTYPE_UINT64 (DTYPE_8BYTES)
#define DTYPE_INT8 (DTYPE_1BYTES | DTYPE_SGND)
#define DTYPE_INT16 (DTYPE_2BYTES | DTYPE_SGND)
#define DTYPE_INT32 (DTYPE_4BYTES | DTYPE_SGND)
#define DTYPE_INT64 (DTYPE_8BYTES | DTYPE_SGND)

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