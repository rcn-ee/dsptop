/*
 * Smart Reflex Sub-System (SRSS) module for Keystone devices
 *
 * Copyright (C) 2014 Texas Instruments.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/sysfs.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <asm/io.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/spinlock.h>

#define DRIVER_NAME "temperature_module_drv"
#define TEMP_KM_MAJOR_VERSION 1
#define TEMP_KM_MINOR_VERSION 0
#define TEMP_KM_PATCH_VERSION 1

/* un-comment the line below to enable printing for kernel debug messages */
//#define DEBUGSS_DRV_DEBUG

#ifdef DEBUGSS_DRV_DEBUG
#define __D(fmt, args...) dev_info(srss_device, "debug_info: " fmt, ## args)
#else
#define __D(fmt, args...)
#endif

#define __E(fmt, args...) dev_err(srss_device, "drv_error: " fmt, ## args)

/* Device variables */
static struct device* srss_device = NULL;
static struct clk		*srss_clk;

/* SRSS macro definitions */
#define SRSS_REG_MAP_SIZE         (0x400)

#define SRSS_TEMP_STAT0_REG       (0x28)
#define SRSS_TEMP_CTL0_REG        (0x1C)

#define ENABLED_MASK              ((0x3 << 8)|(0x3 << 24))
#define ERRORED_MASK              ((0x3 << 10)|(0x3 << 26))
#define READY0_MASK               (0x3 << 12)
#define READY1_MASK               (0x3 << 28)
#define SRSS_TEMP_ENABLE          (0x3)

#define SRSS_TIMEOUT              (10000)
#define SRSS_READ_DELAY           (1000)

#define SRSS0_INDEX               (0)
#define SRSS1_INDEX               (1)

/* Static variables */
#define NUM_SRSS_MODULES          (2)
#define MAX_SRSS_STATUS_MSG_SIZE  (32)

static DEFINE_SPINLOCK(io_lock);
static void __iomem	*srss_base;

static char srss_status[NUM_SRSS_MODULES][MAX_SRSS_STATUS_MSG_SIZE];
static int8_t srss_temp0[NUM_SRSS_MODULES];
static int8_t srss_temp1[NUM_SRSS_MODULES];

static void get_SRSS_temp_params(uint8_t srss_index)
{
	uint32_t regval;
	uint32_t i,j;
	char * srss_status_ptr = (char*)srss_status;
	srss_status_ptr = srss_status_ptr + 
                          (srss_index * MAX_SRSS_STATUS_MSG_SIZE);
	
	/* enable temp monitors in SRSS */
	regval = ioread32(srss_base + (SRSS_REG_MAP_SIZE*srss_index)
                         + SRSS_TEMP_CTL0_REG);
	regval |= SRSS_TEMP_ENABLE;

        spin_lock(&io_lock);
        iowrite32(regval, srss_base + (SRSS_REG_MAP_SIZE*srss_index)
                         + SRSS_TEMP_CTL0_REG);
        spin_unlock(&io_lock);

	regval = ioread32(srss_base + (SRSS_REG_MAP_SIZE*srss_index)
                         + SRSS_TEMP_CTL0_REG);
	
	/* Add delay, before checking whether SRSS temperature monitoring 
        is enabled */ 
	for (j=0; j<SRSS_READ_DELAY; j++);
		
	/* check for enable state */
	regval = ioread32(srss_base + (SRSS_REG_MAP_SIZE*srss_index)
                         + SRSS_TEMP_STAT0_REG);

	if(0 == (regval & ENABLED_MASK)) {
		sprintf(srss_status_ptr, "not_enabled");
		return;
	}
	
	/* check for error */
	if( regval & ERRORED_MASK) {
		sprintf(srss_status_ptr, "temp_mon_error");
		return;
	}
		
	for (i = 0; i< SRSS_TIMEOUT; i++) {
		regval = ioread32(srss_base + (SRSS_REG_MAP_SIZE*srss_index)
                                 + SRSS_TEMP_STAT0_REG);
		if(regval & ERRORED_MASK) {
			sprintf(srss_status_ptr, "temp_mon_error");
			return;
		}
		
		if((regval & READY0_MASK) && (regval & READY1_MASK)) {
			sprintf(srss_status_ptr, "temp_valid");
			/* Get SRSS temperature0 and temperature1 values */
                        srss_temp0[srss_index] = regval & 0xff;
			srss_temp1[srss_index] = ((regval >> 16) & 0xff);
			break;
		} else { 
			/* Add delay, before trying to read the 
                        temperature again */
                        for (j=0; j<SRSS_READ_DELAY; j++);
		}
	}

	if(SRSS_TIMEOUT == i) {
		sprintf(srss_status_ptr, "timeout");
		return;
	}

        return;
}

static ssize_t srss_show_name(struct device* dev, 
                              struct device_attribute* attr, 
                              char* buf)
{
        ssize_t byte_count;

        __D("%s:\n",__FUNCTION__);

        byte_count = sprintf(buf,"SRSStempmod\n");

        return byte_count;
}

static ssize_t read_srss0_temperature0(struct device* dev, 
                                       struct device_attribute* attr, 
                                       char* buf)
{
        ssize_t byte_count;

        __D("%s:\n",__FUNCTION__);

        /* Get current SRSS0 parameters */
        get_SRSS_temp_params(SRSS0_INDEX);

        byte_count = sprintf(buf,"%d\n",srss_temp0[SRSS0_INDEX]);

        return byte_count;
}

static ssize_t read_srss0_temperature1(struct device* dev, 
                                       struct device_attribute* attr, 
                                       char* buf)
{
        ssize_t byte_count;

        __D("%s:\n",__FUNCTION__);

        /* Get current SRSS0 parameters */
        get_SRSS_temp_params(SRSS0_INDEX);

        byte_count = sprintf(buf,"%d\n",srss_temp1[SRSS0_INDEX]);

        return byte_count;
}

static ssize_t get_srss0_status(struct device* dev, 
                                struct device_attribute* attr, 
                                char* buf)
{
        ssize_t byte_count;
        char * srss_status_ptr = (char*)srss_status;

        __D("%s:\n",__FUNCTION__);

        /* Get current SRSS0 parameters */
        get_SRSS_temp_params(SRSS0_INDEX);

        srss_status_ptr = srss_status_ptr + (SRSS0_INDEX * 
                          MAX_SRSS_STATUS_MSG_SIZE);
        byte_count = sprintf(buf,"%s\n",srss_status_ptr);

        return byte_count;
}

static ssize_t read_srss1_temperature0(struct device* dev, 
                                       struct device_attribute* attr, 
                                       char* buf)
{
        ssize_t byte_count;

        __D("%s:\n",__FUNCTION__);

        /* Get current SRSS1 parameters */
        get_SRSS_temp_params(SRSS1_INDEX);

        byte_count = sprintf(buf,"%d\n",srss_temp0[SRSS1_INDEX]);

        return byte_count;
}

static ssize_t read_srss1_temperature1(struct device* dev, 
                                       struct device_attribute* attr, 
                                       char* buf)
{
        ssize_t byte_count;

        __D("%s:\n",__FUNCTION__);

        /* Get current SRSS1 parameters */
        get_SRSS_temp_params(SRSS1_INDEX);

        byte_count = sprintf(buf,"%d\n",srss_temp1[SRSS1_INDEX]);

        return byte_count;
}

static ssize_t get_srss1_status(struct device* dev, 
                                struct device_attribute* attr, 
                                char* buf)
{
        ssize_t byte_count;
        char * srss_status_ptr = (char*)srss_status;

        __D("%s:\n",__FUNCTION__);

        /* Get current SRSS1 parameters */
        get_SRSS_temp_params(SRSS1_INDEX);

        srss_status_ptr = srss_status_ptr + (SRSS1_INDEX * 
                          MAX_SRSS_STATUS_MSG_SIZE);
        byte_count = sprintf(buf,"%s\n",srss_status_ptr);

        return byte_count;
}

/* Declare the sysfs entries. The macros create instances of dev_attr_name, 
dev_attr_srss0_temp0, dev_attr_srss0_temp1, dev_attr_srss0_status, 
dev_attr_srss1_temp0, dev_attr_srss1_temp1 and dev_attr_srss1_status */
static DEVICE_ATTR(name, S_IRUGO, srss_show_name, NULL);
static DEVICE_ATTR(srss0_temp0, S_IRUGO, read_srss0_temperature0, NULL);
static DEVICE_ATTR(srss0_temp1, S_IRUGO, read_srss0_temperature1, NULL);
static DEVICE_ATTR(srss0_status, S_IRUGO, get_srss0_status, NULL);
static DEVICE_ATTR(srss1_temp0, S_IRUGO, read_srss1_temperature0, NULL);
static DEVICE_ATTR(srss1_temp1, S_IRUGO, read_srss1_temperature1, NULL);
static DEVICE_ATTR(srss1_status, S_IRUGO, get_srss1_status, NULL);

static int keystone_srss_probe(struct platform_device *pdev)
{
	uint8_t i;
        char * srss_status_ptr = (char*)srss_status;
	int32_t retval = 0;
        struct resource  *srss_mem;

        __D("%s:\n",__FUNCTION__);

	srss_device = &pdev->dev;

	/* Prepare srss clock */
	srss_clk = devm_clk_get(srss_device, "srssclock");
	if (WARN_ON(IS_ERR(srss_clk)))
		return PTR_ERR(srss_clk);

	clk_prepare_enable(srss_clk);

	srss_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	srss_base = devm_ioremap_resource(srss_device, srss_mem);
	if (IS_ERR(srss_base)) {
		__E("failed to get SRSS register map from the device tree\n");
		return PTR_ERR(srss_base);
        }

	/* Register SRSS device with HWMON class */
	srss_device = hwmon_device_register(&pdev->dev);
	if (IS_ERR(srss_device)) {
		__E("failed to register HWMON device\n");
		retval = PTR_ERR(srss_device);
		return -1;
	}

	/* Now we can create the sysfs endpoints (ignore errors) */
	retval = device_create_file(srss_device, &dev_attr_name);
	if (retval < 0) {
		__E("failed to create temperature module name /sys endpoint\n");
	}
	retval = device_create_file(srss_device, &dev_attr_srss0_temp0);
	if (retval < 0) {
		__E("failed to create SRSS0 temperature0 /sys endpoint\n");
	}
	retval = device_create_file(srss_device, &dev_attr_srss0_temp1);
	if (retval < 0) {
		__E("failed to create SRSS0 temperature1 /sys endpoint\n");
	}
        retval = device_create_file(srss_device, &dev_attr_srss0_status);
        if (retval < 0) {
		__E("failed to create SRSS0 status /sys endpoint\n");
        }
	retval = device_create_file(srss_device, &dev_attr_srss1_temp0);
	if (retval < 0) {
		__E("failed to create SRSS1 temperature0 /sys endpoint\n");
	}
	retval = device_create_file(srss_device, &dev_attr_srss1_temp1);
	if (retval < 0) {
		__E("failed to create SRSS1 temperature1 /sys endpoint\n");
	}
        retval = device_create_file(srss_device, &dev_attr_srss1_status);
        if (retval < 0) {
		__E("failed to create SRSS1 status /sys endpoint\n");
        }

	/* Initialize SRSS status and temp values to default */
	for(i=0; i<NUM_SRSS_MODULES; i++) {
		srss_temp0[i] = -127;
		srss_temp1[i] = -127;
	}

	for(i=0; i<NUM_SRSS_MODULES; i++) {
		srss_status_ptr = srss_status_ptr + (i * 
                                  MAX_SRSS_STATUS_MSG_SIZE);
		sprintf(srss_status_ptr, "not_enabled");
	}

	return retval;
}

static int keystone_srss_remove(struct platform_device *pdev)
{
        __D("%s:\n",__FUNCTION__);

	device_remove_file(srss_device, &dev_attr_name);
        device_remove_file(srss_device, &dev_attr_srss0_temp0);
	device_remove_file(srss_device, &dev_attr_srss0_temp1);
        device_remove_file(srss_device, &dev_attr_srss0_status);
	device_remove_file(srss_device, &dev_attr_srss1_temp0);
	device_remove_file(srss_device, &dev_attr_srss1_temp1);
        device_remove_file(srss_device, &dev_attr_srss1_status);

	/* Unregister SRSS device with HWMON class */
	hwmon_device_unregister(srss_device);

	/* Un-prepare srss clock */
	clk_disable_unprepare(srss_clk);

	return 0;
}

static const struct of_device_id keystone_srss_of_match[] = {
	{ .compatible = "ti,keystone-srss", },
	{},
};
MODULE_DEVICE_TABLE(of, keystone_srss_of_match);

static struct platform_driver platform_srss_driver = {
	.driver = {
		.name = "smartreflex",
		.owner	= THIS_MODULE,
		.of_match_table = keystone_srss_of_match,
	},
	.probe = keystone_srss_probe,
	.remove = keystone_srss_remove,
};

module_platform_driver(platform_srss_driver);

/* Module information */
MODULE_AUTHOR("Texas Instruments Incorporated");
MODULE_DESCRIPTION("Smart Reflex Sub-System (SRSS) module for Keystone devices");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
