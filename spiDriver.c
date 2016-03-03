/**
 * @file   spiDriver.c
 * @author Dean Miller
 * @date   Feb 23 2016
 * @brief  Driver for SPI motor driver
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of_device.h>
//#include <linux/gpio.h>       // Required for the GPIO functions
//#include <linux/interrupt.h>  // Required for the IRQ code
#include <linux/spi/spi.h>    // Required for SPI stuff

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dean Miller");
MODULE_DESCRIPTION("SPI driver for servo drive.");
MODULE_VERSION("0.1");

static int servodrv_spi_probe(struct spi_device *spi){
    int err = 0;
    printk(KERN_INFO "DRV: probing device...\n");
    return err;
}


static int servodrv_spi_remove(struct spi_device *spi){
    int err = 0;
    printk(KERN_INFO "DRV: removing device...\n");
    return err;
}

static const struct of_device_id servodrv_of_match[] = {
	{ .compatible = "servodrv", },
	{ }
};
MODULE_DEVICE_TABLE(of, servodrv_of_match);

static struct spi_driver servodrv_spi_driver = {
	.driver = {
                .owner  = THIS_MODULE,
		.name	= "servodrv01",
		.of_match_table	= servodrv_of_match,
	},
	.probe		= servodrv_spi_probe,
	.remove		= servodrv_spi_remove,
};

static int __init servodrv_init(void){
    int err = 0;
    
    printk(KERN_INFO "DRV: initializing module...\n");
    err = spi_register_driver(&servodrv_spi_driver);
    return err;
}

static void __exit servodrv_exit(void){
    printk(KERN_INFO "DRV: exiting module...\n");
    spi_unregister_driver(&servodrv_spi_driver);
}

module_init(servodrv_init);
module_exit(servodrv_exit);
