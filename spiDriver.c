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
#include <linux/kobject.h>
#include <linux/gpio.h>       // Required for the GPIO functions
#include <linux/interrupt.h>  // Required for the IRQ code
#include <linux/spi/spi.h>    // Required for SPI stuff
#include <linux/miscdevice.h>
#include <linux/proc_fs.h>
#include <linux/mutex.h>
#include <linux/kfifo.h>

#ifndef VM_RESERVED
#define VM_RESERVED (VM_DONTEXPAND | VM_DONTDUMP)
#endif

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dean Miller");
MODULE_DESCRIPTION("SPI driver for servo drive.");
MODULE_VERSION("0.1");

#define SERVODRV_INT_PIN    115

/* fifo size in elements (bytes) */
#define FIFO_SIZE	4096

/* name of the proc entry */
#define	PROC_FIFO	"servodrv-fifo"

/* lock for procfs read access */
static DEFINE_MUTEX(read_lock);

/* lock for procfs write access */
static DEFINE_MUTEX(write_lock);

struct servodrv {
    struct  miscdevice      miscdev;
    struct  spi_device      *myspi;       //spi device
    struct  kfifo           fifo;
    unsigned int            intpin;
    unsigned int            irqnumber;
};

/// Function prototype for the custom IRQ handler function -- see below for the implementation
static irq_handler_t  servodrv_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs);

#define to_servodrv(dev)	container_of((dev), \
		struct servodrv, miscdev)

static const struct file_operations servodrv_fops;

static int servodrv_f_open(struct inode *inode, struct file *filp)
{
    struct servodrv *drv = to_servodrv(filp->private_data);
    //struct device *dev = drv->miscdev.this_device;
    
    printk(KERN_INFO "DRV: open command received.\n");

    filp->private_data = drv;
    
    return 0;
}

static long servodrv_f_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct servodrv *drv = filp->private_data;
    //struct device *dev = drv->miscdev.this_device;
    
    printk(KERN_INFO "DRV: IOCTL command received.\n");
    
    return 0;
}

static ssize_t servodrv_f_write (struct file *filp, const char *buf, size_t len, loff_t *off){
    int ret;
    unsigned int copied;
    struct servodrv *drv = filp->private_data;
    
    printk(KERN_INFO "DRV: write command received: %s\n", buf);
    
    if (mutex_lock_interruptible(&write_lock))
		return -ERESTARTSYS;
    
    ret = kfifo_from_user(&drv->fifo, buf, len, &copied);
    
    mutex_unlock(&write_lock);
    
    return ret ? ret : copied;
}

static ssize_t servodrv_f_read (struct file *filp, char __user *buf, size_t len, loff_t *off)
{
	int ret;
	unsigned int copied;
        struct servodrv *drv = filp->private_data;
        
        printk(KERN_INFO "DRV: read command received.\n");
        
	if (mutex_lock_interruptible(&read_lock))
		return -ERESTARTSYS;

	ret = kfifo_to_user(&drv->fifo, buf, len, &copied);

	mutex_unlock(&read_lock);

	return ret ? ret : copied;
}

static const struct file_operations servodrv_fops = {
    .owner          =   THIS_MODULE,
    .open           =   servodrv_f_open,
    .unlocked_ioctl =   servodrv_f_ioctl,
    .write          =   servodrv_f_write,
    .read           =   servodrv_f_read,
};

//TODO: may want to put some useful attributes in here
//declare attributes
static struct kobj_attribute test_attr = __ATTR(test, 0660, NULL, NULL);

static struct attribute *attrs[] = {
    &test_attr.attr,
    NULL,
};

static struct attribute_group attr_group = { .attrs = attrs };

static const struct file_operations fifo_fops = {
    .owner      = THIS_MODULE,
};

//declare probe and remove functions
static int servodrv_spi_probe(struct spi_device *spi){
    int err = 0;
    struct servodrv *drv;
    struct device *dev;
    
    printk(KERN_INFO "DRV: probing device...\n");
    
    drv = kzalloc(sizeof(struct servodrv), GFP_KERNEL);
    if(!drv){
        printk(KERN_ERR "DRV: could not allocate servodrv struct.\n");
        return -1;
    }
    
    drv->miscdev.fops = &servodrv_fops;
    drv->miscdev.minor = MISC_DYNAMIC_MINOR;
    drv->miscdev.mode = S_IRUGO;
    drv->miscdev.name = "servodrv_misc";
    
    //register the misc device
    err = misc_register(&drv->miscdev);
    if(err)
        goto fail;
    dev = drv->miscdev.this_device;
    dev_set_drvdata(dev, drv);
    
    drv->intpin = SERVODRV_INT_PIN;
    
    gpio_request(drv->intpin, "sysfs");       // Set up the gpio interrupt
    gpio_direction_input(drv->intpin);        // Set the button GPIO to be an input
    gpio_export(drv->intpin, false);          // Causes gpio115 to appear in /sys/class/gpio
                  // the bool argument prevents the direction from being changed

    // GPIO numbers and IRQ numbers are not the same! This function performs the mapping for us
    drv->irqnumber = gpio_to_irq(drv->intpin);
    printk(KERN_INFO "DRV: The interrupt is mapped to IRQ: %d\n", drv->irqnumber);

    // This next call requests an interrupt line
    err = request_irq(drv->irqnumber,             // The interrupt number requested
                         (irq_handler_t) servodrv_irq_handler, // The pointer to the handler function below
                         IRQF_TRIGGER_RISING,   // Interrupt on rising edge (button press, not release)
                         "servodrv_interrupt_handler",    // Used in /proc/interrupts to identify the owner
                         drv); 
    
    //initialize SPI interface
    drv->myspi = spi;
    drv->myspi->max_speed_hz = 4000000;
    spi_setup(drv->myspi);
    spi_set_drvdata(spi, drv);
    
    //initialize fifo
    err = kfifo_alloc(&drv->fifo, FIFO_SIZE, GFP_KERNEL);
    if(err){
        printk(KERN_ERR "DRV: error allocating fifo.\n");
        goto failfifoalloc;
    }
    
    if(proc_create(PROC_FIFO, 0, NULL, &fifo_fops) == NULL){
        printk(KERN_ERR "DRV: failed to create fifo proc.\n");
        goto failproccreate;
    }
    
    //create device files /dev/servodrv_misc and /sys/devices/virtual/servodrv_misc/
    err = sysfs_create_group(&dev->kobj, &attr_group);
    if(err){
        printk(KERN_ERR "DRV: registration failed.\n");
        goto faildreg;
    }
    
    printk(KERN_INFO "DRV: servodrv sucessfully loaded!\n");
    
    return err;
faildreg:
        kfifo_free(&drv->fifo);
failproccreate:
        kfree(drv);
failfifoalloc:
	misc_deregister(&drv->miscdev);
fail:
    return -1;
}

static int servodrv_spi_remove(struct spi_device *spi){
    //TODO: all this
    int err = 0;
    struct servodrv *drv = spi_get_drvdata(spi);
    struct device *dev = drv->miscdev.this_device;
    
    printk(KERN_INFO "DRV: removing device...\n");
    
    free_irq(drv->irqnumber, NULL); //free irq
    gpio_unexport(drv->intpin);
    gpio_free(drv->intpin);
    
    //free fifo
    remove_proc_entry(PROC_FIFO, NULL);
    kfifo_free(&drv->fifo);
    
    /* Remove the sysfs attributes */
    sysfs_remove_group(&dev->kobj, &attr_group);

    /* Deregister the misc device */
    misc_deregister(&drv->miscdev);

    kfree(drv);
    return err;
}

static irq_handler_t servodrv_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs){
    int status;
    unsigned char txbuf[8];
    struct servodrv *drv = dev_id;
    
   //printk(KERN_INFO "DRV: interrupt received.\n");
    
    status = mutex_lock_interruptible(&read_lock)

    status = kfifo_out(&drv->fifo, &txbuf, 8);

    mutex_unlock(&read_lock);
    
    status = spi_write(drv->myspi, &txbuf[0], 8);
        if (status < 0)
                printk(KERN_ERR "DRV: FAILURE: spi_write() failed with status %d\n",
                        status);
    
   return (irq_handler_t) IRQ_HANDLED;      // Announce that the IRQ has been handled correctly
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
