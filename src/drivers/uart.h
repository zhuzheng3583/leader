/*******************************Copyright (c)***************************
** 
** Porject name:	leader
** Created by:		zhuzheng<happyzhull@163.com>
** Created date:	2016/04/05
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

namespace driver {

class uart : public device, public irq_handler
{
public:
    uart(PCSTR name, s32 id);
    ~uart(void);

public:
    u32 _baudrate; 

    dma *_dmatx;
    dma *_dmarx;
    s32 _dma_tx_id;
    s32 _dma_rx_id;
    
    s32 _flag_tx;
    s32 _flag_rx;
    
public:
    s32 probe(void);
    s32 remove(void);

    void set_baudrate(u32 baudrate)	{ _baudrate = baudrate; }
    u32 get_baudrate(void)			{ return _baudrate; }

    s32 recv(u8 *buf, u32 count);
    s32 send(u8 *buf, u32 count);

    s32 self_test(void);

    
public:
    virtual s32 open(s32 flags);
    virtual s32 read(u8 *buf, u32 count);
    virtual s32 write(u8 *buf, u32 count);
    virtual s32 close(void);

    
public:
    virtual void isr(void); 
};

}
/***********************************************************************
** End of file
***********************************************************************/
