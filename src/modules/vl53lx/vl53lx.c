/* vl53lx.c - Driver for ST VL53LX time of flight sensor */

#define DT_DRV_COMPAT st_vl53lx

/*
 * Copyright (c) 2017 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <kernel.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <init.h>
#include <drivers/gpio.h>
#include <sys/__assert.h>
#include <zephyr/types.h>
#include <device.h>
#include <logging/log.h>

#include "vl53lx_api.h"
#include "vl53lx_platform.h"
#include "vl53lx_platform_user_defines.h"
#include "vl53lx_platform_user_config.h"

LOG_MODULE_REGISTER(VL53LX, CONFIG_SENSOR_LOG_LEVEL);

/* All the values used in this driver are coming from ST datasheet and examples.
 * It can be found here:
 *   http://www.st.com/en/embedded-software/stsw-img005.html
 * There are also examples of use in the L4 cube FW:
 *   http://www.st.com/en/embedded-software/stm32cubel4.html
 */
#define VL53LX_REG_MODEL_ID   			0x010F
#define VL53LX_MODEL_ID        			0xEA
#define VL53LX_SETUP_SIGNAL_LIMIT         	(0.1*65536)
#define VL53LX_SETUP_MAX_TIME_FOR_RANGING     	33000
#define VL53LX_SETUP_SIGMA_LIMIT          	(60*65536)
#define VL53LX_SETUP_PRE_RANGE_VCSEL_PERIOD   	18
#define VL53LX_SETUP_FINAL_RANGE_VCSEL_PERIOD 	14

#ifndef DT_HAS_COMPAT_STATUS_OKAY(st_vl53lx)
    #if DT_NODE_HAS_COMPAT(st_vl53lx)
        #error "Devicetree have no node with a compatible st_vl53lx"
    #else
        #error "Devicetree have a node with a compatible st_vl53lx but status is not okay"
    #endif
#endif

#define TOF_INT_NODE DT_ALIAS(inter)
#if DT_NODE_HAS_STATUS(TOF_INT_NODE, okay)
  #define TOF_INT_LABEL DT_GPIO_LABEL(TOF_INT_NODE, gpios)
  #define TOF_INT_PIN	 DT_GPIO_PIN(TOF_INT_NODE, gpios)
  #define TOF_INT_FLAGS (GPIO_INPUT | DT_GPIO_FLAGS(TOF_INT_NODE, gpios))
#else
  #error "TOF_INT_NODE devicetree alias is not defined"
#endif
#define TOF_XSHUT_NODE DT_ALIAS(xshut) // Gets the node identifier (node_id)
//#define TOF_XSHUT_NODE DT_NODELABEL(tof2) // Gets the node identifier (node_id)
//#define TOF_XSHUT_NODE DT_PATH(tof, tof_xshut) // Gets the node identifier (node_id)
#if DT_NODE_HAS_STATUS(TOF_XSHUT_NODE, okay)
  #define TOF_XSHUT_LABEL DT_GPIO_LABEL(TOF_XSHUT_NODE, gpios)
  #define TOF_XSHUT_PIN DT_GPIO_PIN(TOF_XSHUT_NODE, gpios)
  //#define TOF_XSHUT_PIN	 DT_GPIO_PIN_BY_IDX(TOF_XSHUT_NODE, gpios, 0)
  #define TOF_XSHUT_FLAGS DT_GPIO_FLAGS(TOF_XSHUT_NODE, gpios)
  //#define TOF_XSHUT_FLAGS DT_GPIO_FLAGS_BY_IDX(TOF_XSHUT_NODE, gpios, 0)
#else
  #error "TOF_XSHUT_NODE devicetree alias is not defined"
#endif

// Device tree "class". Instansiation see end of that file
struct vl53lx_data {
    const struct device *i2c; 	// Interface
    VL53LX_Dev_t vl53lx; 		// Vandor data. "TOF object". DT magic: Must have the same name as the device?
    VL53LX_CalibrationData_t CalibrationData; // Calibration Data
};

static int vl53lx_standby(void){
    #if DT_NODE_HAS_STATUS(TOF_XSHUT_NODE, okay)
        const struct device *gpio;

        // configure and set VL53LX_XSHUT_Pin
        gpio = device_get_binding(DT_LABEL(TOF_XSHUT_NODE));
        if (gpio == NULL) {
            LOG_ERR("Could not get pointer to device TOF_XSHUT_NODE.");
            return -EINVAL;
        }

        if (gpio_pin_configure(gpio, TOF_XSHUT_PIN, TOF_XSHUT_FLAGS) < 0) {
            LOG_ERR("Could not configure TOF_XSHUT_GPIO_PIN");
            return -EINVAL;
        }

        gpio_pin_set(gpio, TOF_XSHUT_PIN, 0);
    #endif

    return 0;
}

static int vl53lx_powerOn(void){
    #if DT_NODE_HAS_STATUS(TOF_XSHUT_NODE, okay)
        const struct device *gpio;

        // configure and set VL53LX_XSHUT_Pin
        gpio = device_get_binding(DT_LABEL(TOF_XSHUT_NODE));
        if (gpio == NULL) {
            LOG_ERR("Could not get pointer to device TOF_XSHUT_NODE.");
            return -EINVAL;
        }

        /*if (gpio_pin_configure(gpio, TOF_XSHUT_PIN, GPIO_OUTPUT_ACTIVE | TOF_XSHUT_FLAGS) < 0) {
            LOG_ERR("Could not configure TOF_XSHUT_GPIO_PIN");
            return -EINVAL;
        }*/

        //gpio_pin_set(gpio, TOF_XSHUT_PIN, 1);
    #endif

    return 0;
}


static int vl53lx_sample_fetch(const struct device *dev, enum sensor_channel chan){
    struct vl53lx_data *drv_data = dev->data;
    VL53LX_Error ret;
    uint8_t measurementDataReady = 0;

    do{
        ret = VL53LX_GetMeasurementDataReady(&drv_data->vl53lx, &measurementDataReady);
        if (ret) {
            LOG_ERR("VL53LX_GetMeasurementDataReady failed");
            goto exit;
        }
        if(measurementDataReady == 0){
            VL53LX_WaitUs(&drv_data->vl53lx, 100);
        }
    }while(measurementDataReady == 0);

    exit:
        return ret;
}

static int vl53lx_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val){
    struct vl53lx_data *drv_data = dev->data;
    VL53LX_Error ret = VL53LX_ERROR_NONE;
    VL53LX_MultiRangingData_t multiRangingData;

    ret = VL53LX_GetMultiRangingData(&drv_data->vl53lx, &multiRangingData);
    if (ret) {
        LOG_ERR("VL53LX_GetMeasurementDataReady failed");
        goto exit;
    }

    LOG_INF("NumberOfObjectsFound: {%d}", multiRangingData.NumberOfObjectsFound);
    LOG_INF("RangeMilliMeter: {%d}mm", multiRangingData.RangeData[0].RangeMilliMeter);

    if (chan == SENSOR_CHAN_DISTANCE) {
        if(multiRangingData.NumberOfObjectsFound > 0){
            val->val1 = multiRangingData.RangeData[0].RangeMilliMeter;
            val->val2 = 0;
        }
    }

    ret = VL53LX_ClearInterruptAndStartMeasurement(&drv_data->vl53lx);
    if (ret) {
        LOG_ERR("VL53LX_ClearInterruptAndStartMeasurement failed");
        goto exit;
    }

    exit:
        return ret;
}



int vl53lx_setup_single_shot(const struct device *dev){
    struct vl53lx_data *drv_data = dev->data;
    VL53LX_Error ret = VL53LX_ERROR_NONE;

    ret = VL53LX_GetCalibrationData(&drv_data->vl53lx, &drv_data->CalibrationData);
    if (ret) {
        LOG_ERR("VL53LX_GetCalibrationData failed");
        goto exit;
    }

    ret = VL53LX_SetCalibrationData(&drv_data->vl53lx, &drv_data->CalibrationData);
    if (ret) {
        LOG_ERR("VL53LX_SetCalibrationData failed");
        goto exit;
    }

    ret = VL53LX_SetDistanceMode(&drv_data->vl53lx, VL53LX_DISTANCEMODE_SHORT);
    if (ret) {
        LOG_ERR("VL53LX_SetDistanceMode failed");
        goto exit;
    }

    ret = VL53LX_StartMeasurement(&drv_data->vl53lx);
    if (ret) {
        LOG_ERR("VL53LX_StartMeasurement failed");
        goto exit;
    }

    exit:
        return ret;
}

static int vl53lx_init(const struct device *dev){
    struct vl53lx_data *drv_data = dev->data;
    VL53LX_Error ret = VL53LX_ERROR_NONE;
    uint8_t vl53lx_id = 0U;
    VL53LX_DeviceInfo_t vl53lx_dev_info;


    LOG_DBG("enter in %s", __func__);



    #if DT_INST_NODE_HAS_PROP(0, tof_interrupt_gpios)

    #endif

    drv_data->i2c = device_get_binding(DT_INST_BUS_LABEL(0));
    if (drv_data->i2c == NULL) {
          LOG_ERR("Could not get pointer to %s device.",
                  DT_INST_BUS_LABEL(0));
          return -EINVAL;
    }

    drv_data->vl53lx.i2c = drv_data->i2c;
    drv_data->vl53lx.I2cDevAddr = DT_INST_REG_ADDR(0);

    vl53lx_powerOn();
    k_sleep(K_MSEC(1)); // tBOOT is 1.2 ms max.

    ret = VL53LX_WaitDeviceBooted(&drv_data->vl53lx);
    if (ret) {
        LOG_ERR("WaitDeviceBooted failed");
        return -ENODEV;
    }

    // Get info from sensor (MUA not manatory?)
    #if 1
        (void)memset(&vl53lx_dev_info, 0, sizeof(VL53LX_DeviceInfo_t));

        ret = VL53LX_GetDeviceInfo(&drv_data->vl53lx, &vl53lx_dev_info);
        if (ret < 0) {
                LOG_ERR("Could not get info from device.");
                return -ENODEV;
        }

        LOG_DBG("VL53LX_GetDeviceInfo = %d", ret);
        LOG_DBG("   ProductType ID : %x",  vl53lx_dev_info.ProductType); // Product Type, VL53LX = 0xAA
        LOG_DBG("   ProductRevisionMajor : %d",
                    vl53lx_dev_info.ProductRevisionMajor);
        LOG_DBG("   ProductRevisionMinor : %d",
                    vl53lx_dev_info.ProductRevisionMinor);

        VL53LX_RdByte(&drv_data->vl53lx,
                     VL53LX_REG_MODEL_ID,
                     &vl53lx_id);

        if ((ret < 0) || (vl53lx_id != VL53LX_MODEL_ID)) {
                LOG_ERR("Issue on device identification");
                return -ENOTSUP;
        }
    #endif

    // sensor init
    ret = VL53LX_DataInit(&drv_data->vl53lx); 
    if (ret < 0) {
          LOG_ERR("VL53LX_DataInit return error (%d)", ret);
          return -ENOTSUP;
    }

    /*ret =  VL53LX_PerformRefSpadManagement(&drv_data->vl53lx);
    if (ret < 0) {
          LOG_ERR("VL53LX_PerformRefSpadManagement return error (%d)", ret);
          return -ENOTSUP;
    } HACK */
    
    ret = vl53lx_setup_single_shot(dev);
    if (ret < 0) {
          return -ENOTSUP;
    }

    return 0;
}

// Device tree entries
static const struct sensor_driver_api vl53lx_api_funcs = {
	.sample_fetch = vl53lx_sample_fetch,
	.channel_get = vl53lx_channel_get
};
//static const struct sensor_driver_api vl53lx_api_funcs = {};

static struct vl53lx_data vl53lx_driver;

DEVICE_DT_INST_DEFINE(0, vl53lx_init, device_pm_control_nop, &vl53lx_driver,
		    NULL, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
		    &vl53lx_api_funcs);
