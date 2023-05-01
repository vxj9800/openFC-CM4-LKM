/**
 * @file openFC-CM4-USB-Driver.c
 * @author Vatsal Joshi
 * @date 01 May 2023
 * @version 0.1
 * @brief A driver to read sensor data and send commands to
 * an rp2040 based PI CM4 board which is used as flight controller
 * and the PI CM4 is used as the main processing unit.
 */
#include <linux/module.h>	 /* Needed by all modules */
#include <linux/kernel.h>	 /* Needed for KERN_INFO */
#include <linux/init.h>	 /* Needed for the macros */

///< The license type -- this affects runtime behavior
MODULE_LICENSE("GPL");

///< The author -- visible when you use modinfo
MODULE_AUTHOR("Vatsal Joshi");

///< The description -- see modinfo
MODULE_DESCRIPTION("A simple Hello world LKM!");

///< The version of the module
MODULE_VERSION("0.1");

static int __init hello_init(void)
{
	printk(KERN_INFO "Loading hello module...\n");
	printk(KERN_INFO "Hello world\n");
	return 0;
}

static void __exit hello_exit(void)
{
	printk(KERN_INFO "Goodbye!\n");
}

module_init(hello_init);
module_exit(hello_exit);
