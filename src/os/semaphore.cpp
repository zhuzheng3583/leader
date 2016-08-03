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
#include "os/semaphore.h"

#include "includes.h"

namespace os {

semaphore::semaphore(void) :
    _semcnt(0)
{

}

semaphore::~semaphore(void)
{

}

BOOL semaphore::create(PCSTR name, u32 semcnt)
{
    _name = name;
    _semcnt = semcnt;
	//osSemaphoreDef(name);
	_handle = (HANDLE)osSemaphoreCreate (NULL, (int32_t)semcnt);
    if (_handle == NULL) {
		ERR("%s: _handle = %d.\n", _name, _handle);
		kernel::on_error(ERR_OPERATION_FAILED, this);
		return false;
    }

    DBG("%s: semaphore create success.\n", _name);
    return true;
}

BOOL semaphore::s_delete(void)
{
	osStatus status = osOK;
	status = osSemaphoreDelete(osSemaphoreId(_handle));
	if (status != osOK) {
		ERR("%s: status = %d.\n", _name, status);
		kernel::on_error(ERR_OPERATION_FAILED, this);
		return false;
	}
	
	return true;
}

BOOL semaphore::pend(s32 timeoutms)
{
	osStatus status = osOK;
	status = (osStatus)osSemaphoreWait(osSemaphoreId(_handle), (uint32_t)timeoutms);
	if (status != osOK) {
		ERR("%s: status = %d.\n", _name, status);
		kernel::on_error(ERR_OPERATION_FAILED, this);
		return false;
	}
	
	return true;
}

BOOL semaphore::post(s32 timeoutms)
{
	osStatus status = osOK;
	status = osSemaphoreRelease(osSemaphoreId(_handle));
	if (status != osOK) {
		ERR("%s: status = %d.\n", _name, status);
		kernel::on_error(ERR_OPERATION_FAILED, this);
		return false;
	}
	
	return true;
}

void semaphore::reset(void)
{

}

}
/***********************************************************************
** End of file
***********************************************************************/
