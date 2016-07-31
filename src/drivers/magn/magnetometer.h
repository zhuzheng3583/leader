/*******************************Copyright (c)***************************
**
**  							   SIMULATE
**
**------------------File Info-------------------------------------------
** File name:            	magnetometer.h
** Latest Version:      	V1.0.0
** Latest modified Date:	2015/06/07
** Modified by:
** Descriptions:
**
**----------------------------------------------------------------------
** Created by:           	Zhu Zheng <happyzhull@163.com>
** Created date:         	2015/06/07
** Descriptions:
**
***********************************************************************/
#pragma once
#include "type.h"
#include "device.h"

#define HMC_CONFIG_A 	0x00 	// r/w
#define HMC_CONFIG_B 	0x01 	// r/w
#define HMC_MODE 		0x02 	// r/w
#define HMC_DATA_X_MSB 	0x03	// r
#define HMC_DATA_X_LSB 	0x04	// r
#define HMC_DATA_Z_MSB 	0x05	// r
#define HMC_DATA_Z_LSB 	0x06	// r
#define HMC_DATA_Y_MSB 	0x07	// r
#define HMC_DATA_Y_LSB 	0x08	// r
#define HMC_STATUS 		0x09	// r
#define HMC_IDENTE_A	0x0A	// r
#define HMC_IDENTE_B	0x0B	// r
#define HMC_IDENTE_C	0x0C	// r


struct mag_device {
	U8 magslaveaddr;
	struct i2c_device *i2c_dev;

	struct device dev;
};

struct mag_platform {
	PCSTR name;
	S32 id;
	U8 magslaveaddr;
	struct i2c_device *i2c_dev;
};

struct mag_device *mag_dev_constructe(struct mag_platform *plat);
S32 mag_dev_destroy(struct mag_device *mag_dev);



S8 mag_i2c_write(U8 reg, U8 len, U8 *buf);
S8 mag_i2c_read(U8 reg, U8 len, U8 *buf);


/***********************************************************************
** End of file
***********************************************************************/

