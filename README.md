
Please go to the reworked example https://github.com/dev889/NordicDTSgpioExample
This repo is not sustained

# NRFconnect_out-of-tree_example

** This is a working repo. It is still bugy ... ** 

```
../src/modules/vl53lx/vl53lx.c: In function 'vl53lx_standby':
zephyr/include/generated/devicetree_unfixed.h:3877:29: error: 'DT_N_S_tof_S_tof_xshut_P_gpios_IDX_0_VAL_pin' undeclared (first use in this function); did you mean 'DT_N_S_leds_S_led_3_P_gpios_IDX_0_VAL_pin'?
#define DT_N_ALIAS_xshut    DT_N_S_tof_S_tof_xshut
```

This makro failes
``` c
#define TOF_XSHUT_PIN DT_GPIO_PIN(TOF_XSHUT_NODE, gpios)
```

In the «board.dts» is a snipped like:

```
#include <nordic/nrf52832_qfaa.dtsi>
/dts-v1/;

{
    model = "TEST_BT NRF52832";
	compatible = "nordic";
    leds {
        compatible = "gpio-leds";
	    led0: led_0 {
	        gpios = <&gpio0 17 GPIO_ACTIVE_LOW>;
	        label = "Green LED 0";
        };
    tof {
        compatible = "st,vl53lx";
        //compatible = "gpio-leds"; // HACK it works with that because the binding is in Zephyr tree
        tof2: tof_xshut {
		    gpios = <&gpio0 7 GPIO_ACTIVE_LOW>; // BUG this definition does not exist
		    label = "Xshutdown active low";
	    };
    };
}
```

In the "devicetree_unfixed.h" there is a diffence between the tof GPIO and the LED ones:
The bindig is missing for the TOF

```
/*
 * Devicetree node: /leds/led_0
 *
 * Node identifier: DT_N_S_leds_S_led_0
 *
 * Binding (compatible = gpio-leds):
 *   $ZEPHYR_BASE\dts\bindings\gpio\gpio-leds.yaml
 *
 * (Descriptions have moved to the Devicetree Bindings Index
 * in the documentation.)
 */
```

```
/*
 * Devicetree node: /tof
 *
 * Node identifier: DT_N_S_tof
 */
 ```
