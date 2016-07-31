/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:		zhuzheng<happyzhull@163.com>
** Created date:	2016/07/31
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#pragma once
#include "leader_type.h"
#include "leader_misc.h"

#include "core.h"
#include "interrupt.h"
#include "device.h"
#include "dma.h"
#include "i2c.h"

#include "packet.h"

#define ADDR_CONFIG_A 	0x00 	// r/w
#define ADDR_CONFIG_B 	0x01 	// r/w
#define ADDR_MODE 		0x02 	// r/w
#define ADDR_DATA_X_MSB 	0x03	// r
#define ADDR_DATA_X_LSB 	0x04	// r
#define ADDR_DATA_Z_MSB 	0x05	// r
#define ADDR_DATA_Z_LSB 	0x06	// r
#define ADDR_DATA_Y_MSB 	0x07	// r
#define ADDR_DATA_Y_LSB 	0x08	// r
#define ADDR_STATUS 		0x09	// r

#define ADDR_ID_A	0x0A	// r
#define ADDR_ID_B	0x0B	// r
#define ADDR_ID_C	0x0C	// r
#define ID_A_WHO_AM_I			'H'
#define ID_B_WHO_AM_I			'4'
#define ID_C_WHO_AM_I			'3'


#define SLAVE_ADDRESS 0x3C


/**
 * Calibration PROM as reported by the device.
 */
#pragma pack(push,1)
struct prom_s {
	u16 factory_setup;
	u16 c1_pres_sens;
	u16 c2_pres_offset;
	u16 c3_temp_coeff_pres_sens;
	u16 c4_temp_coeff_pres_offset;
	u16 c5_reference_temp;
	u16 c6_temp_coeff_temp;
	u16 serial_and_crc;
};
#pragma pack(pop)


namespace driver {

class hmc5883 : public device, public interrupt
{
public:
    hmc5883(PCSTR name, s32 id);
    ~hmc5883(void);

public:
	u8 _slave_addr;
	i2c *_i2c;

public:
    s32 probe(i2c *pi2c, u8 slave_addr);
    s32 remove(void);


public:
	void init(void);
	s32 read_raw(void);

	s32 read_reg8(u8 reg);
	s32 write_reg8(u8 reg, u8 data);
	s32 read_reg(u8 reg, u8 len, u8 *buf);
	s32 write_reg(u8 reg, u8 len, u8 *buf);
};
}

