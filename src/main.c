/*
 * Copyright (c) 2017 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <stdio.h>
#include <sys/printk.h>
#include <logging/log.h>

// Device tree (DT)
#include <device.h>
#include <devicetree.h>
#include <drivers/sensor.h>
#include <drivers/gpio.h>

#include "sub/multiply.h"

//#include <vl53lx_def.h>
//#include "vl53lx_platform_user_defines.h"

LOG_MODULE_REGISTER(main, CONFIG_SENSOR_LOG_LEVEL);

#if DT_NODE_HAS_STATUS(DT_INST(0, st_vl53lx), okay)
#else
  #error "st_vl53lx devicetree alias is not defined"
#endif

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
//#define LED0_NODE DT_NODELABEL(led0)
//#define LED0_NODE DT_PATH(leds, led_0)
#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
  #define LABEL_LED0    DT_GPIO_LABEL(LED0_NODE, gpios)
  #define PIN_LED0      DT_GPIO_PIN(LED0_NODE, gpios)
  #define FLAGS_LED0    DT_GPIO_FLAGS(LED0_NODE, gpios)
#else
  /* A build error here means your board isn't set up to blink an LED. */
  #error "led0 devicetree alias is not defined"
#endif

#define LED1_NODE DT_ALIAS(led1)
#if DT_NODE_HAS_STATUS(LED1_NODE, okay)
  #define LABEL_LED1    DT_GPIO_LABEL(LED1_NODE, gpios)
  #define PIN_LED1      DT_GPIO_PIN(LED1_NODE, gpios)
  #define FLAGS_LED1    DT_GPIO_FLAGS(LED1_NODE, gpios)
#else
  /* A build error here means your board isn't set up to blink an LED. */
  #error "led1 devicetree alias is not defined"
#endif

#define SW0_NODE	DT_ALIAS(sw0)
#if DT_NODE_HAS_STATUS(SW0_NODE, okay)
  #define SW0_LABEL DT_GPIO_LABEL(SW0_NODE, gpios)
  #define SW0_PIN	 DT_GPIO_PIN(SW0_NODE, gpios)
  #define SW0_FLAGS (GPIO_INPUT | DT_GPIO_FLAGS(SW0_NODE, gpios))
#else
  #error "sw0 devicetree alias is not defined"
#endif

/*
 * Get a device structure from a devicetree node with compatible
 * "bosch,bme280". (If there are multiple, just pick one.)
 */
static const struct device *get_TOF_device(void)
{
  const struct device *dev = device_get_binding(DT_LABEL(DT_INST(0, st_vl53lx)));

  __ASSERT(dev, "Failed to get device binding for VL53L3CX"); // Zephye is halted 
  
  //if (dev == NULL){
  if (!device_is_ready(dev)) {
    printk("\nError: Device \"%s\" is not ready; "
	   "check the driver initialization logs for errors.\n",
          dev->name);
    return NULL;
  }

  printk("Found TOF device \"%s\", getting sensor data\n", dev->name);
  return dev;
}

static void test_miltiply(void){
  int x = 5;
  int y = 2;
  int result = multiply(x, y);
  printk("%d times %d equals %d\n", x, y, result);
}

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
  printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
}

void main(void)
{
  int ret;
  struct sensor_value value;

  bool led_is_on = true;
  
  struct gpio_callback button_cb_data;


  printk("TEST FILO BT\n");
  LOG_INF("TEST FILO BT");
  LOG_DBG("TEST FILO BT");
  test_miltiply();

  const struct device *devTOF = get_TOF_device();
  const struct device *devLED0 = device_get_binding(LABEL_LED0);
  const struct device *devLED1 = device_get_binding(LABEL_LED1);
  const struct device *devSW0 = device_get_binding(SW0_LABEL);
	
  if (devTOF == NULL) {
    printk("Could not get VL53L3CX device\n");
  }
  if (devLED0 == NULL) {
      printk("Could not get LED0 device\n");
  }
  else{
    ret = gpio_pin_configure(devLED0, PIN_LED0, GPIO_OUTPUT_ACTIVE | FLAGS_LED0);
  }

  if (devLED1 == NULL) {
      printk("Could not get LED1 device\n");
  }
  else{
    ret = gpio_pin_configure(devLED1, PIN_LED1, GPIO_OUTPUT_ACTIVE | FLAGS_LED1);
  }

  if (devSW0 == NULL) {
      printk("Could not get SW0 device\n");
  }
  else{
    ret = gpio_pin_configure(devSW0, SW0_PIN, SW0_FLAGS);
    ret = gpio_pin_interrupt_configure(devSW0, SW0_PIN, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&button_cb_data, button_pressed, BIT(SW0_PIN));
    gpio_add_callback(devSW0, &button_cb_data);
  }

  while (1) {
    if (devTOF != NULL) {
      ret = sensor_sample_fetch(devTOF);
      if (ret) {
        printk("sensor_sample_fetch failed ret %d\n", ret);
      }

      ret = sensor_channel_get(devTOF, SENSOR_CHAN_DISTANCE, &value);
      if (ret) {
        printk("sensor_channel_get failed ret %d\n", ret);
      }
      else{
        printk("distance is %dmm\n", value.val1);
        LOG_INF("distance is %dmm\n", value.val1);
      }
    }
    if (devLED0 != NULL) {

      gpio_pin_set(devLED0, PIN_LED0, (int)led_is_on);
      led_is_on = !led_is_on;
    }
    if (devLED1 != NULL && devSW0 != NULL) {
      bool val;
      val = gpio_pin_get(devSW0, SW0_PIN);
      gpio_pin_set(devLED1, PIN_LED1, val);
    }
    printk("._");
    //k_busy_wait(K_MSEC(1000)); 
    k_sleep(K_MSEC(100));
  }
}
