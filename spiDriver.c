/**
 * @file   spiDriver.c
 * @author Dean Miller
 * @date   Feb 23 2016
 * @brief  Driver for SPI motor driver
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>       // Required for the GPIO functions
#include <linux/interrupt.h>  // Required for the IRQ code
#include <linux/spi/spi.h>    // Required for SPI stuff
#include <linux/kobject.h>    // Using kobjects for the sysfs bindings

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dean Miller");
MODULE_DESCRIPTION("SPI driver for servo drive.");
MODULE_VERSION("0.1");

static unsigned int gpioRDY = 115;       ///< Default GPIO is 115
module_param(gpioRDY, uint, S_IRUGO);    ///< Param desc. S_IRUGO can be read/not changed
MODULE_PARM_DESC(gpioRDY, " GPIO RDY number (default=115)");  ///< parameter description

static char gpioName[7] = "gpioXXX";          ///< Null terminated default string -- just in case
static int    irqNumber;                    ///< Used to share the IRQ number within this file
enum modes { SPI, STEP_DIR };              ///< The available communication modes. SPI or step/direction
static enum modes mode = SPI;             ///< Default mode is SPI
static int interpPeriod	= 1;			///< Default interpolation period is 1ms

//******** SPI STUFF *******//

static struct spi_device *myspi;

// this FIFO store is used for storing incoming data from the sysfs file
// this stored data gets sent to the SPI device
#define FIFOSTORESIZE 20
#define FIFOSTOREDATASIZE 16
typedef struct {
	uint8_t data[FIFOSTOREDATASIZE];
	int size;
} fifostoreentry;
static fifostoreentry fifostore[FIFOSTORESIZE];
static int fifostorepos = 0;

static struct workqueue_struct* spikernmodex_workqueue;
static struct work_struct spikernmodex_work_processfifostore;
static struct work_struct spikernmodex_work_read;

// this spinlock is used for disabling interrupts while spi_sync() is running
static DEFINE_SPINLOCK(spilock);
static unsigned long spilockflags;

// this function writes "count" bytes from "buf" to the given "reg" register
static void spi_write_reg_burst(uint8_t reg, const uint8_t *buf, size_t count)
{
	struct spi_transfer t[2];
	struct spi_message m;
    
	spi_message_init(&m);
    
	memset(t, 0, sizeof(t));
    
	t[0].tx_buf = &reg;
	t[0].len = 1;
	spi_message_add_tail(&t[0], &m);
    
	t[1].tx_buf = buf;
	t[1].len = count;
	spi_message_add_tail(&t[1], &m);
    
	spin_lock_irqsave(&spilock, spilockflags);
	spi_sync(myspi, &m);
	spin_unlock_irqrestore(&spilock, spilockflags);
}

// this function is called when writing to the "store" sysfs file
static ssize_t spikernmodex_store_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	if (count > FIFOSTOREDATASIZE) {
		printk(KERN_ERR "can't store data because it's too big.");
		return 0;
	}
    
	if (fifostorepos >= FIFOSTORESIZE) {
		printk(KERN_ERR "can't store data because fifo is full.");
		return 0;
	}
    
	printk(KERN_DEBUG "store_store(): storing %d bytes to store pos 0x%.2x\n", count, fifostorepos);
    
	memcpy(fifostore[fifostorepos].data, buf, count);
	fifostore[fifostorepos].size = count;
	fifostorepos++;
    
	printk(KERN_DEBUG "queueing work PROCESSFIFOSTORE\n");
	queue_work(spikernmodex_workqueue, &spikernmodex_work_processfifostore);
    
	return count;
}

void spikernmodex_workqueue_handler(struct work_struct *work) {
	int i, j;
    
	// this work outputs all data stored in the FIFO to the SPI device
	if (work == &spikernmodex_work_processfifostore) {
		printk(KERN_DEBUG "work PROCESSFIFOSTORE called\n");
        
		while (fifostorepos > 0) { // processing all items in the fifo store
			printk(KERN_DEBUG "%d entries in fifo store\n", fifostorepos);
            
			printk(KERN_DEBUG "sending %d bytes\n", fifostore[0].size);
			BEAGLEBONE_LED3ON;
			spi_write_reg_burst(SPIDEVDATAREG, fifostore[0].data, fifostore[0].size);
			BEAGLEBONE_LED3OFF;
            
			// left shifting the FIFO store
			for (i = 1; i < FIFOSTORESIZE; i++) {
				for (j = 0; j < fifostore[i].size; j++)
					fifostore[i-1].data[j] = fifostore[i].data[j];
				fifostore[i-1].size = fifostore[i].size;
			}
			fifostorepos--;
		}
	}
    
	// this work reads some data from the SPI device to the ringbuffer
	if (work == &spikernmodex_work_read) {
		printk(KERN_DEBUG "work READ called, ringbuf %.2x\n", ringbufferpos);
        
		memset(&ringbuffer[ringbufferpos].data, 0, RINGBUFFERDATASIZE);
		ringbuffer[ringbufferpos].used = 1;
        
		BEAGLEBONE_LED4ON;
		spi_read_reg_burst(SPIDEVDATAREG, ringbuffer[ringbufferpos].data, RINGBUFFERDATASIZE);
		ringbuffer[ringbufferpos].size = RINGBUFFERDATASIZE;
		BEAGLEBONE_LED4ON;
        
		printk(KERN_DEBUG "read stopped, ringbuf %.2x\n", ringbufferpos);
		ringbuffer[ringbufferpos].completed = 1;
        
		ringbufferpos++;
		if (ringbufferpos == RINGBUFFERSIZE)
			ringbufferpos = 0;
	}
    
	printk(KERN_DEBUG "work exit\n");
}


// this functions gets called when the user reads the sysfs file "somereg" USED FOR TESTING
static ssize_t spikernmodex_somereg_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	// outputting the value of register "SOMEREG"
	return sprintf(buf, "%x\n", spi_read_reg(SOMEREG));
}

// this function gets called when the user writes the sysfs file "somereg" USED FOR TESTING
static ssize_t spikernmodex_somereg_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int val;
	sscanf(buf, "%x", &val);
	spi_write_reg(SOMEREG, (uint8_t)val);
	printk(KERN_DEBUG "stored %.2x to register SOMEREG\n", (uint8_t)val);
	return count;
}

//TODO: THIS SHOULD PRINT WHAT'S TO BE WRITTEN TO DEVICE
// this function is called when reading from the "store" sysfs file
static ssize_t spikernmodex_store_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int currentbufsize;
	int i = ringbufferpos+1;
    
	if (i == RINGBUFFERSIZE)
		i = 0;
    
	while (i != ringbufferpos) {
		if (ringbuffer[i].completed) {
			currentbufsize = ringbuffer[i].size;
			// we found a used & completed slot, outputting
			printk(KERN_DEBUG "store_show(): outputting ringbuf %.2x, %d bytes\n", i, currentbufsize);
			memcpy(buf, ringbuffer[i].data, currentbufsize);
			ringbuffer[i].completed = ringbuffer[i].used = 0;
			return currentbufsize;
		}
        
		i++;
		if (i == RINGBUFFERSIZE)
			i = 0;
	}
    
    return 0;
}

//*************************//

/// Function prototype for the custom IRQ handler function -- see below for the implementation
static irq_handler_t  rdy_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs);


/** @brief A callback function to display the communication mode
 *  @param kobj represents a kernel object device that appears in the sysfs filesystem
 *  @param attr the pointer to the kobj_attribute struct
 *  @param buf the buffer to which to write the number of presses
 *  @return return the number of characters of the mode string successfully displayed
 */
static ssize_t mode_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf){
   switch(mode){
      case SPI:   return sprintf(buf, "SPI\n");       // Display the state -- simplistic approach
      case STEP_DIR:    return sprintf(buf, "step/direction\n");
      default:    return sprintf(buf, "LKM Error\n"); // Cannot get here
   }
}

/** @brief A callback function to store the communication mode using the enum above */
static ssize_t mode_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count){
   // the count-1 is important as otherwise the \n is used in the comparison
   if (strncmp(buf,"spi",count-1)==0) { mode = SPI; }   // strncmp() compare with fixed number chars
   else if (strncmp(buf,"step_dir",count-1)==0) { mode = STEP_DIR; }
   return count;
}

/** @brief A callback function to display the LED period */
static ssize_t period_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf){
   return sprintf(buf, "%d\n", interpPeriod);
}

/** @brief A callback function to store the LED period value */
static ssize_t period_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count){
   unsigned int period;                     // Using a variable to validate the data sent
   sscanf(buf, "%du", &period);             // Read in the period as an unsigned int
   if ((period>0)&&(period<=20)){        // Must be 2ms or greater, 20ms or less
      interpPeriod = period;                 // Within range, assign to blinkPeriod variable
   }
   return period;
}

/** Use these helper macros to define the name and access levels of the kobj_attributes
 *  The kobj_attribute has an attribute attr (name and mode), show and store function pointers
 *  The period variable is associated with the interpPeriod variable and it is to be exposed
 *  with mode 0666 using the period_show and period_store functions above
 */
static struct kobj_attribute period_attr = __ATTR(interpPeriod, 0660, period_show, period_store);
static struct kobj_attribute mode_attr = __ATTR(mode, 0660, mode_show, mode_store);

//************ SPI STUFF *********//
static struct kobj_attribute store_attribute = __ATTR(data, 0660, spikernmodex_store_show, spikernmodex_store_store);
static struct kobj_attribute somereg_attribute = __ATTR(addr, 0660, spikernmodex_somereg_show, spikernmodex_somereg_store);
//********************************//

/** The drv_attrs[] is an array of attributes that is used to create the attribute group below.
 *  The attr property of the kobj_attribute is used to extract the attribute struct
 */
static struct attribute *drv_attrs[] = {
   &store_attribute.attr,                   // data to be written
   &somereg_attribute.attr,                 // read or write register
   &period_attr.attr,                       // The servo cycle or interpolation period
   &mode_attr.attr,                         // what is the communication mode for the driver
   NULL,
};

/** The attribute group uses the attribute array and a name, which is exposed on sysfs -- in this
 *  case it is drv01, which is automatically defined in the drv_init() function below
 *  using the custom kernel parameter that can be passed when the module is loaded.
 */
static struct attribute_group attr_group = {
   .name  = gpioName,                        // The name is generated in drv_init()
   .attrs = drv_attrs,                      // The attributes array defined just above
};

//*********** SPI STUFF **************//

static int __devinit spikernmodex_probe(struct spi_device *spi); // prototype
static int spikernmodex_remove(struct spi_device *spi); // prototype

// this is the spi driver structure which has the driver name and the two
// functions to call when probing and removing
static struct spi_driver spikernmodex_driver = {
	.driver = {
		.name = "spikernmodex", // be sure to match this with the spi_board_info modalias in arch/am/mach-omap2/board-am335xevm.c
		.owner = THIS_MODULE
	},
	.probe = spikernmodex_probe,
    .remove = __devexit_p(spikernmodex_remove)
};

// this function gets called when a matching modalias and driver name found
static int __devinit spikernmodex_probe(struct spi_device *spi)
{
	int err;
    
	printk(KERN_DEBUG "spikernmodex_probe() called.\n");
    
	// initalizing SPI interface
	myspi = spi;
	myspi->max_speed_hz = 5000000; // get this from your SPI device's datasheet
	spi_setup(myspi);
    
	// initializing device
	spi_cmd(SOMEKINDOFRESETCOMMAND);
	mdelay(100);
    
	// initializing workqueue
	spikernmodex_workqueue = create_singlethread_workqueue("spikernmodex_workqueue");
	INIT_WORK(&spikernmodex_work_processfifostore, spikernmodex_workqueue_handler);
	INIT_WORK(&spikernmodex_work_read, spikernmodex_workqueue_handler);
    
	spikernmodex_clearringbuffer();
    
	printk(KERN_INFO "Example SPI driver by Nonoo (www.nonoo.hu) loaded.\n");
    
	return err;
}

// this function gets called when our example SPI driver gets removed with spi_unregister_driver()
static int spikernmodex_remove(struct spi_device *spi)
{
	printk(KERN_DEBUG "spikernmodex_remove() called.\n");
    
	// destroying the workqueue
	flush_workqueue(spikernmodex_workqueue);
	destroy_workqueue(spikernmodex_workqueue);
    
	return 0;
}

//************************************//

static struct kobject *drv_kobj;            /// The pointer to the kobject


/** @brief The LKM initialization function
 *  The static keyword restricts the visibility of the function to within this C file. The __init
 *  macro means that for a built-in driver (not a LKM) the function is only used at initialization
 *  time and that it can be discarded and its memory freed up after that point. In this example this
 *  function sets up the GPIOs and the IRQ
 *  @return returns 0 if successful
 */
static int __init drv_init(void){
   int result = 0;
   unsigned long IRQflags = IRQF_TRIGGER_RISING;      // The default is a rising-edge interrupt

   printk(KERN_INFO "DRV: Initializing the servo driver LKM\n");
   sprintf(gpioName, "gpio%d", gpioRDY);      // Create the gpio115 name for /sys/ebb/led49

   drv_kobj = kobject_create_and_add("drv", kernel_kobj->parent); // kernel_kobj points to /sys/kernel
   if(!drv_kobj){
      printk(KERN_ALERT "DRV: failed to create kobject\n");
      return -ENOMEM;
   }
   // add the attributes to /sys/drv/ -- for example, /sys/drv/gpio115/mode
   result = sysfs_create_group(drv_kobj, &attr_group);
   if(result) {
      printk(KERN_ALERT "DRV: failed to create sysfs group\n");
      kobject_put(drv_kobj);                // clean up -- remove the kobject sysfs entry
      return result;
   }

   gpio_request(gpioRDY, "sysfs");          // gpioLED is 49 by default, request it
   gpio_direction_input(gpioRDY);   // Set the gpio to be in input mode
   gpio_export(gpioRDY, false);  // causes gpio49 to appear in /sys/class/gpio
                                 // the second argument prevents the direction from being changed

   /// GPIO numbers and IRQ numbers are not the same! This function performs the mapping for us
   irqNumber = gpio_to_irq(gpioRDY);
   printk(KERN_INFO "DRV: The RDY pin is mapped to IRQ: %d\n", irqNumber);
   // This next call requests an interrupt line
   result = request_irq(irqNumber,             // The interrupt number requested
                        (irq_handler_t) rdy_irq_handler, // The pointer to the handler function below
                        IRQflags,              // Use the custom kernel param to set interrupt type
                        "drv_rdy_handler",  // Used in /proc/interrupts to identify the owner
                        NULL);
    
    //********* SPI STUFF *******//
    // registering SPI driver, this will call spikernmodex_probe()
	result = spi_register_driver(&spikernmodex_driver);
	if (result < 0) {
		printk(KERN_ERR "spi_register_driver() failed %d\n", result);
		return result;
	}
    //*************************//

   return result;
}

/** @brief The LKM cleanup function
 *  Similar to the initialization function, it is static. The __exit macro notifies that if this
 *  code is used for a built-in driver (not a LKM) that this function is not required.
 */
static void __exit drv_exit(void){
   kobject_put(drv_kobj);                   // clean up -- remove the kobject sysfs entry
   free_irq(irqNumber, NULL);               // Free the IRQ number, no *dev_id required in this case
   gpio_unexport(gpioRDY);                  // Unexport the Button GPIO
   gpio_free(gpioRDY);                      // Free the RDY GPIO
    
   //******* SPI STUFF ***************//
    spi_unregister_driver(&spikernmodex_driver);
    //*******************************//
    
   printk(KERN_INFO "DRV: DRV LKM has been unloaded!\n");
}

/** @brief The GPIO IRQ Handler function
 *  This function is a custom interrupt handler that is attached to the GPIO above. The same interrupt
 *  handler cannot be invoked concurrently as the interrupt line is masked out until the function is complete.
 *  This function is static as it should not be invoked directly from outside of this file.
 *  @param irq    the IRQ number that is associated with the GPIO -- useful for logging.
 *  @param dev_id the *dev_id that is provided -- can be used to identify which device caused the interrupt
 *  Not used in this example as NULL is passed.
 *  @param regs   h/w specific register values -- only really ever used for debugging.
 *  return returns IRQ_HANDLED if successful -- should return IRQ_NONE otherwise.
 */
static irq_handler_t rdy_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs){
   printk(KERN_INFO "DRV: RDY interrupt triggered\n");
   return (irq_handler_t) IRQ_HANDLED;  // Announce that the IRQ has been handled correctly
}

/// This next calls are  mandatory -- they identify the initialization function
/// and the cleanup function (as above).
module_init(drv_init);
module_exit(drv_exit);
