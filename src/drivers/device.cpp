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
#include "device.h"

namespace driver {

device::device(void) : 
	_name(NULL),
    _id(-1),
    _handle(NULL),
    _devname(NULL),
    _devid(-1),
    _devhandle(NULL),
    _irq(-1),
    _probed(0),
    _opened(0)
{
	/* TODO register*/
	
}

device::device(PCSTR name, s32 id) :
	_name(name),
    _id(id),
    _handle(NULL),
    _devname(name),
    _devid(id),
    _devhandle(NULL),
    _irq(-1),
    _probed(0),
    _opened(0)
{
	
}

device::~device(void)
{
	/* TODO unregister*/
}

s32 device::probe(void)
{
	if (_probed != 0) {
		ERR("%s: Has been probed.\n", _name);
		return -1;
	}
	
	_probed = 1;
	return 0;
}

s32 device::is_probed(void)
{
	if (_probed != 0) {
		return 0;
	}

	ERR("%s: Has no been probed.\n", _name);
	return -1;
}

s32 device::remove(void)
{
	if (_probed == 0) {
		ERR("%s: Has been removed.\n", _name);
		return -1;
	}

	_probed = 0;
	_handle = NULL;
	
	return 0;
}

s32 device::open(s32 flags)
{
	if (_opened != 0) {
		ERR("%s: Has been opened.\n", _name);
		return -1;
	}
	
	_opened = 1;
	return 0;
}

s32 device::close(void)
{
	if (_opened == 0) {
		ERR("%s: Has been close.\n", _name);
		return -1;
	}

	_opened = 0;
	_handle = NULL;
	
	return 0;
}

s32 device::read(u8 *buf, u32 size)
{
	WRN("Called to device base class, Please check...\n");
	return 0;
}

s32 device::write(u8 *buf, u32 size)
{
	WRN("Called to device base class, Please check...\n");
	return 0;
}



s32 device::ioctl(enum ioctl_cmd cmd, u32 arg)
{
	WRN("Called to device base class, Please check...\n");
	return 0;
}

s32 device::seek(s32 offset, enum seek_mode mode)
{
	WRN("Called to device base class, Please check...\n");
	return 0;
}

s32 device::tell(void)
{
	WRN("Called to device base class, Please check...\n");
	return 0;
}

s32 device::flush(void)
{
	WRN("Called to device base class, Please check...\n");
	return 0;

}

}

/***********************************************************************
** End of file
***********************************************************************/
