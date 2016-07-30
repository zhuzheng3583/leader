/*******************************Copyright (c)***************************
**
** Porject name:	LeaderUAV-Plus
** Created by:		zhuzheng<happyzhull@163.com>
** Created date:	2015/08/28
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#include "ms5611.h"
#include <math.h>

namespace driver {

ms5611::ms5611(PCSTR name, s32 id) :
    device(name, id)
{
    _prom.factory_setup = 0;
	_prom.c1_pres_sens = 0;
	_prom.c2_pres_offset = 0;
	_prom.c3_temp_coeff_pres_sens = 0;
	_prom.c4_temp_coeff_pres_offset = 0;
	_prom.c5_reference_temp = 0;
	_prom.c6_temp_coeff_temp = 0;
	_prom.serial_and_crc = 0;
}

ms5611::~ms5611(void)
{

}

s32 ms5611::probe(spi *pspi, gpio *gpio_cs)
{
    ASSERT((pspi != NULL) && (gpio_cs != NULL));

	if (device::probe() < 0) {
		ERR("%s: failed to probe.\n", _name);
		goto fail0;
	}

    _gpio_cs = gpio_cs;
    _spi = pspi;

	_gpio_cs->set_direction_output();
    _gpio_cs->set_value(true);

    ms5611::init();
    while (1) {
        ms5611::measure();
    }

    return 0;

fail0:
    return -1;
}


void ms5611::reset(void)
{
    ms5611::cmd_write_byte(ADDR_RESET_CMD);
    core::mdelay(10);
}

/**
 * MS5611 crc4 cribbed from the datasheet
 */
s32 ms5611::crc4(u16 *prom)
{
	s16 cnt;
	u16 rem;
	u16 crc_read;
	u8 bit;

	rem = 0x00;
	/* save the read crc */
	crc_read = prom[7];
	/* remove CRC byte */
	prom[7] = (0xFF00 & (prom[7]));

	for (cnt = 0; cnt < 16; cnt++) {
		/* uneven bytes */
		if (cnt & 1) {
			rem ^= (u8)((prom[cnt >> 1]) & 0x00FF);
		} else {
			rem ^= (u8)(prom[cnt >> 1] >> 8);
		}

		for (bit = 8; bit > 0; bit--) {
			if (rem & 0x8000) {
				rem = (rem << 1) ^ 0x3000;
			} else {
				rem = (rem << 1);
			}
		}
	}
	/* final 4 bit remainder is CRC value */
	rem = (0x000F & (rem >> 12));
	prom[7] = crc_read;
	/* return true if CRCs match */
	if ((0x000F & crc_read) != (rem ^ 0x00)) {
        return -1;
    }

    return 0;
}

void ms5611::read_prom(void)
{
	/* read and convert PROM words */
    u8 buf[2] = { 0 };
    ms5611::reg_read(ADDR_PROM_SETUP, 2, buf);
    _prom.factory_setup = buf[0] << 8 | buf[1];
    ms5611::reg_read(ADDR_PROM_C1, 2, buf);
    _prom.c1_pres_sens = buf[0] << 8 | buf[1];
    ms5611::reg_read(ADDR_PROM_C2, 2, buf);
    _prom.c2_pres_offset = buf[0] << 8 | buf[1];
    ms5611::reg_read(ADDR_PROM_C3, 2, buf);
    _prom.c3_temp_coeff_pres_sens = buf[0] << 8 | buf[1];
    ms5611::reg_read(ADDR_PROM_C4, 2, buf);
    _prom.c4_temp_coeff_pres_offset = buf[0] << 8 | buf[1];
    ms5611::reg_read(ADDR_PROM_C5, 2, buf);
    _prom.c5_reference_temp = buf[0] << 8 | buf[1];
    ms5611::reg_read(ADDR_PROM_C6, 2, buf);
    _prom.c6_temp_coeff_temp = buf[0] << 8 | buf[1];
    ms5611::reg_read(ADDR_PROM_CRC, 2, buf);
    _prom.serial_and_crc = buf[0] << 8 | buf[1];

    u16 tmp = 0;
    ms5611::reg_read(ADDR_RESET_CMD, 2, (u8 *)&tmp);
	/* calculate CRC and return success/failure accordingly */
	s32 ret = ms5611::crc4((u16 *)&_prom);
    if (ret != 0) {
		ERR("crc failed");
    }

    //return ret;
}

u32 ms5611::read_raw(u8 cmd_osr)
{
    union cvt {
        u8 byte[4];
        u32 raw;
    };

    ms5611::cmd_write_byte(cmd_osr);
    
    union cvt data;
    u8 buf[3] = { 0 };
    /* read the most recent measurement */
    s32 ret = ms5611::reg_read(ADDR_DATA, sizeof(buf), buf);
    if (ret == 0) {
        /* fetch the raw value */
        data.byte[0] = buf[2];
        data.byte[1] = buf[1];
        data.byte[2] = buf[0];
        data.byte[3] = 0;

        return (data.raw);
        //(u32)((buf[0]<<16) | (buf[1]<<8) | (buf[2]));
    }
}



s32 ms5611::get_temperature(void)
{
    return _temperature;
}

s32 ms5611::get_pressure(void)
{
    return _pressure;
}

f32 ms5611::get_altitude(void)                             
{
    return _altitude;
}

//Maximum values for calculation results:
//PMIN = 10mbar PMAX = 1200mbar
//TMIN = -40°C TMAX = 85°C TREF = 20°C
s32 ms5611::measure(void)
{
    s32 dt;                        //实际和参考温度之间的差异
    u32 d2_temp;                    //用于存放温度
	d2_temp = ms5611::read_raw(ADDR_CMD_CONVERT_D2);
	dt = (s32)d2_temp - (s32)_prom.c5_reference_temp * 256;
	_temperature = 2000 + dt * _prom.c6_temp_coeff_temp / 8388608;

    u32 d1_pres;                    //用于存放压力
    s64 off, sens;                  //实际温度抵消、实际温度灵敏度
    s64 aux, off2, sens2, temp2;    //温度校验值
    d1_pres = ms5611::read_raw(ADDR_CMD_CONVERT_D1);
	off = ((s64)_prom.c2_pres_offset * 65536) + ((s64)_prom.c4_temp_coeff_pres_offset * dt / 128);
	sens = ((s64)_prom.c1_pres_sens * 32768) + ((s64)_prom.c3_temp_coeff_pres_sens * dt / 256);

    if(_temperature < 2000)// second order temperature compensation when under 20 degrees C
    {
		temp2 = (dt * dt) / 0x80000000;
		aux = (_temperature - 2000) * (_temperature - 2000);
		off2 = 2.5 * aux;
		sens2 = 1.25 * aux;
		if(_temperature < -1500)
		{
			aux = (_temperature + 1500) * (_temperature + 1500);
			off2 = off2 + 7 * aux;
			sens2 = sens2 + 5.5 * aux;
		}
	} else { //(Temperature > 2000)
        temp2 = 0;
		off2 = 0;
		sens2 = 0;
	}	
	_temperature = _temperature - temp2;
	off = off - off2;
	sens = sens - sens2;
    
    _pressure = (((d1_pres * sens) / 2097152 - off) / 32768);
    
    f64 tmp;    	
    tmp = (_pressure / 101325.0);
    tmp = pow(tmp, 0.190295);               
    _altitude = 44330 * (1.0 - tmp);
  
    return 0;
}

void ms5611::init(void)
{
	ms5611::reset();
	ms5611::read_prom();
}

s32 ms5611::cmd_write_byte(u8 cmd)
{
    s32 ret = 0;
    //cmd = cmd & SPI_WRITE_CMD /*WRITE_CMD*/;
    _gpio_cs->set_value(false);
    struct spi_msg msgs = {
        .buf = &cmd,
        .len = sizeof(cmd),
        .flags = 0,
    };
    ret = _spi->transfer(&msgs);
    if (ret < 0) {
        ERR("failed to spi transfer.\n");
        goto fail0;
    }

    /*
	 * Wait for PROM contents to be in the device (2.8 ms) in the case we are
	 * called immediately after reset.
	 */
    core::mdelay(10);
    _gpio_cs->set_value(true);

    return 0;

fail0:
    _gpio_cs->set_value(true);
    return -1;
}


s32 ms5611::reg_read_byte(u8 reg)
{
    s32 ret = 0;
    u8 data = 0;
    ms5611::reg_read(reg, 1, &data);
    if (ret < 0) {
        return -1;
    }

    return ((s32)data);
}


s32 ms5611::reg_write_byte(u8 reg, u8 data)
{
    s32 ret = 0;
    ret = ms5611::reg_write(reg, 1, &data);
    if (ret < 0) {
        return -1;
    }

    return 0;
}

s32 ms5611::reg_read(u8 reg, u8 len, u8 *buf)
{
    s32 ret = 0;
    //reg = reg | READ_CMD/*SPI_READ_CMD*/ ;
    _gpio_cs->set_value(false);
    struct spi_msg msgs[] = {
        [0] = {
            .buf = &reg,
            .len = sizeof(reg),
            .flags = 0,
        },
        [1] = {
            .buf = buf,
            .len = len,
            .flags = SPI_M_RD,
        },
    };
    ret = _spi->transfer(&msgs[0]);
    if (ret < 0) {
        goto fail0;
    }
    ret = _spi->transfer(&msgs[1]);
    if (ret < 0) {
        goto fail1;
    }
    
    _gpio_cs->set_value(true);
    return 0;

fail1:
fail0:
    _gpio_cs->set_value(true);
    return -1;
}

s32 ms5611::reg_write(u8 reg, u8 len, u8 *buf)
{
    s32 ret = 0;
   //reg = reg & WRITE_CMD /*SPI_WRITE_CMD*/ ;
    _gpio_cs->set_value(false);
    struct spi_msg msgs[] = {
        [0] = {
            .buf = &reg,
            .len = sizeof(reg),
            .flags = 0,
        },
        [1] = {
            .buf = buf,
            .len = len,
            .flags = 0,
        },
    };
    ret = _spi->transfer(&msgs[0]);
    if (ret < 0) {
        goto fail0;
    }
    ret = _spi->transfer(&msgs[1]);
    if (ret < 0) {
        goto fail1;
    }
    //core::mdelay(10);
    _gpio_cs->set_value(true);

    return 0;

fail1:
fail0:
    _gpio_cs->set_value(true);
    return -1;
}

}



