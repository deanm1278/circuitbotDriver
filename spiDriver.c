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

/** The drv_attrs[] is an array of attributes that is used to create the attribute group below.
 *  The attr property of the kobj_attribute is used to extract the attribute struct
 */
static struct attribute *drv_attrs[] = {
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
