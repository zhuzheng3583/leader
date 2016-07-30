/*******************************Copyright (c)***************************
**
** Porject name:	LeaderUAV-Plus
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2015/08/28
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
#include "spi.h"

#include "packet.h"


#define ADDR_RESET_CMD			0x1E	/* write to this address to reset chip */
#define ADDR_CMD_CONVERT_D1		0x48	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D2		0x58	/* write to this address to start temperature conversion */
#define ADDR_DATA				0x00	/* address of 3 bytes / 32bit pressure data */
#define ADDR_PROM_SETUP			0xA0	/* address of 8x 2 bytes factory and calibration data */
#define ADDR_PROM_C1			0xA2	/* address of 6x 2 bytes calibration data */
#define ADDR_PROM_C2			0xA4
#define ADDR_PROM_C3			0xA6
#define ADDR_PROM_C4			0xA8
#define ADDR_PROM_C5			0xAA
#define ADDR_PROM_C6			0xAC
#define ADDR_PROM_CRC 			0xAE


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

class ms5611 : public device, public interrupt
{
public:
    ms5611(PCSTR name, s32 id);
    ~ms5611(void);

public:
	spi *_spi;
	gpio *_gpio_cs;

	struct prom_s _prom; 				//用于存放PROM中的8组校准数据

    s32 _temperature, _pressure;
    f32 _altitude;

public:
    s32 probe(spi *pspi, gpio *gpio_cs);
    s32 remove(void);

public:
    virtual s32 open(s32 flags);
    virtual s32 read(u8 *buf, u32 count);
    virtual s32 close(void);

public:
	void init(void);
    s32 measure(data_baro_t *data);

    void reset(void);
	void read_prom(void);
    s32 crc4(u16 *prom);

	s32 read_raw(u8 cmd_osr);

	s32 cmd_write_byte(u8 cmd, s32 ms);
	s32 reg_read_byte(u8 reg);
	s32 reg_write_byte(u8 reg, u8 data);
	s32 reg_read(u8 reg, u8 len, u8 *buf);
	s32 reg_write(u8 reg, u8 len, u8 *buf);
};
}

