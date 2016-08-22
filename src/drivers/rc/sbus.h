/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2016/08/22
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#pragma once
#include "device.h"

namespace driver {

class sbus : public device
{
public:
    sbus(PCSTR name, s32 id);
    ~sbus(void);

public:
    s32 probe(void);
    s32 remove(void);

public:
    s32 init(void);
    s32 reset(void);
    void measure(void);
};

}
/***********************************************************************
** End of file
***********************************************************************/
