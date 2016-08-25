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
#include "uart_int.h"
#include "core.h"
#include "dma.h"

namespace driver {

class uart_dma : public uart_int
{
public:
    uart_dma(PCSTR devname, s32 devid);
    ~uart_dma(void);

public:
    dma *_dmatx;
    dma *_dmarx;
    s32 _dma_tx_id;
    s32 _dma_rx_id;
    
public:
    virtual s32 probe(void);
    virtual s32 remove(void);

    inline s32 recv(u8 *buf, u32 count);
    inline s32 send(u8 *buf, u32 count);

public:
    virtual s32 read(u8 *buf, u32 size);
    virtual s32 write(u8 *buf, u32 size);
};

}
/***********************************************************************
** End of file
***********************************************************************/
