#include <linux/init.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

#include <linux/irq.h>
#include <linux/gpio.h>

unsigned int mpu6050_gpio = 6;    // INT pin of GY-521(MPU6050 board) connected to PA6 pin 7 EXTIN orangePi-One
unsigned int mpu6050_irq_n;       // system IRQ
unsigned int mpu6050_irq_cnt = 0; // how many time IRQ happened


static irq_handler_t  mpu6050_gpio_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs){
	mpu6050_irq_cnt++;

	return  (irq_handler_t) IRQ_HANDLED;  
}


int mpu6050_irq_init(void) {
	int result = 0;
	gpio_request(mpu6050_gpio, "sysfs");   // Set up the mpu6050_gpio
	gpio_direction_input(mpu6050_gpio);    // Set the button GPIO to be an input
	gpio_export(mpu6050_gpio, false);      // Causes gpio6 to appear in /sys/class/gpio
											// the bool argument prevents the direction from being changed
	printk(KERN_INFO "mpu6050: gpio.%d connected to pin INT state is currently: %d\n",
		mpu6050_gpio, gpio_get_value(mpu6050_gpio));
 
	mpu6050_irq_n = gpio_to_irq(mpu6050_gpio);
	printk(KERN_INFO "mpu6050: gpio.%d irq is mapped to irq: %d\n", mpu6050_gpio, mpu6050_irq_n);
 
	// This next call requests an interrupt line
	result = request_irq(mpu6050_irq_n,             // The interrupt number requested
                        (irq_handler_t) mpu6050_gpio_irq_handler, // The pointer to the handler function below
                        IRQF_TRIGGER_RISING,     // Interrupt on rising edge (button press, not release)
                        "mpu6050_gpio_irq_handler",   // Used in /proc/interrupts to identify the owner
                        NULL);                   // The *dev_id for shared interrupt lines, NULL is okay
 
	printk(KERN_INFO "mpu6050: mpu_irq_init: The interrupt request result is: %d\n", result);
	return result;
}


void mpu6050_irq_free(void) {
	printk(KERN_INFO "mpu6050: The IRQ was called %d times\n", mpu6050_irq_cnt);
	free_irq(mpu6050_irq_n, NULL);     // Free the IRQ number, no *dev_id required in this case
	gpio_unexport(mpu6050_gpio);
	gpio_free(mpu6050_gpio);
}
