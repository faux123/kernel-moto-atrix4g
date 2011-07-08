/*
 *  drivers/misc/bcm4750.h
 *
 *  Supports for bcm4750 GPS chip
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 * Copyright (C) 2010 Motorola, Inc.
 *
 * Revision History:
 * Author      Date             Comment
 * Motorola    2010-10-11       <Power Management for Broadcom GPS Chip 4750>
 */

 #ifndef __BCM4750_H
 #define __BCM4750_H

 /* chip operation */
 #define OP_RESET 			1001
 #define OP_STANDBY			1002
 #define OP_POWER_EN		1003
 /* chip mod */
 #define CHIP_MOD_STANDBY    0
 #define CHIP_MOD_POWERON    1
 #define BCM4750_MODE_SIZE	 4

struct gps_platform_data {
    int reset_pin;
	int power_pin;
};

struct bcm4750{
	struct gps_platform_data *pdata;
};
#endif



