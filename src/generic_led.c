/*
 * Simple functions to control board LEDs
 */

#include <sys/printk.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

#define LED0_NODE DT_ALIAS(led0)
#define LED0    DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN0	    DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS0   DT_GPIO_FLAGS(LED0_NODE, gpios)

#define LED1_NODE DT_ALIAS(led1)
#define LED1    DT_GPIO_LABEL(LED1_NODE, gpios)
#define PIN1	    DT_GPIO_PIN(LED1_NODE, gpios)
#define FLAGS1   DT_GPIO_FLAGS(LED1_NODE, gpios)

const struct device *led0_dev;
const struct device *led1_dev;
bool led0_is_on = true;
bool led1_is_on = false;

int generic_led_init(void){
    // Config LED
    int err;
    led0_dev = device_get_binding(LED0);
    if (led0_dev == NULL) {
        printk("No device for LED0.");
        return -1;
    }
    err = gpio_pin_configure(led0_dev, PIN0, GPIO_OUTPUT_ACTIVE | FLAGS0);
    if (err < 0) {
        printk("GPIO config error in LED0.");
        return -1;
    }
    led1_dev = device_get_binding(LED1);
    if (led1_dev == NULL) {
        printk("No device.");
        return -1;
    }
    err = gpio_pin_configure(led1_dev, PIN1, GPIO_OUTPUT_ACTIVE | FLAGS1);
    if (err < 0) {
        printk("GPIO config error.");
        return -1;
    }
    gpio_pin_set(led0_dev, PIN0, (int)(0));
    gpio_pin_set(led1_dev, PIN1, (int)(0));
    return 0;
}

void red_led_on(void){
    gpio_pin_set(led0_dev, PIN0, (int)(1));
    led0_is_on = 1;
    // printk("led\n");
}

void red_led_blink(void){
    gpio_pin_set(led0_dev, PIN0, (int)(led0_is_on));
    led0_is_on = !led0_is_on;
    // printk("led\n");
}

void blue_led_on(void){
    gpio_pin_set(led1_dev, PIN1, (int)(1));
    led1_is_on = 1;
    // printk("led\n");
}

void blue_led_blink(void){
    gpio_pin_set(led1_dev, PIN1, (int)(led1_is_on));
    led1_is_on = !led1_is_on;
    // printk("led\n");
}
