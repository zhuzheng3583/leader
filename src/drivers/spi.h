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
#pragma once
#include "leader_type.h"
#include "leader_misc.h"

#include "core.h"
#include "interrupt.h"
#include "device.h"
#include "dma.h"
#include "gpio.h"

#define	SPI_NBITS_SINGLE	    0x01 /* 1bit transfer */
#define	SPI_NBITS_DUAL		0x02 /* 2bits transfer */
#define	SPI_NBITS_QUAD		0x04 /* 4bits transfer */

#define SPI_M_RD                0x0001

struct spi_msg {
    void    *buf;
    u32     len;
    u16     flags;  
};

namespace driver {

class spi : public device, public interrupt
{
public:
	spi(PCSTR name, s32 id);
	~spi(void);
    
public:
    u32  _speed_hz;
    u8	_bits_per_word;
    u16	_delay_usecs;
    u32	_cs_change:1;

public:
    dma *_dmatx;
    dma *_dmarx;
    s32 _dma_tx_id;
    s32 _dma_rx_id;

    s32 _eventtx;
    s32 _eventrx;

    
public:
    s32 probe(void);
    s32 remove(void);
    
public:
    s32 tx(u8 *buf, u32 size);
    s32 rx(u8 *buf, u32 size);
    s32 transfer(struct spi_msg *msg);
    
    s32 reset(void);
    s32 self_test(void);

public:
    void cs_init(void);
    void cs_deinit(void);
    void cs_active(void);
    void cs_deactive(void);
    
	void set_speed(u32 spiclk);
};

}
/***********************************************************************
** End of file
***********************************************************************/

