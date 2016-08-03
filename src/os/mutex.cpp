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
#include "os/mutex.h"


namespace os {

mutex::mutex(void)
{

}

mutex::~mutex(void)
{

}

BOOL mutex::create(PCSTR name)
{
    _name = name;
	_handle =(HANDLE)osMutexCreate(NULL);
    if (_handle == NULL) {
		ERR("%s: _handle = %d.\n", _name, _handle);
		kernel::on_error(ERR_OPERATION_FAILED, this);
		return false;
    }

    DBG("%s: mutex create success.\n", _name);
    return true;
}

BOOL mutex::m_delete(void)
{
	osStatus status = osOK;
	status = osMutexDelete(osMutexId(_handle));
	if (status != osOK) {
		ERR("%s: status = %d.\n", _name, status);
		kernel::on_error(ERR_OPERATION_FAILED, this);
		return false;
	}
	
	return true;
}

BOOL mutex::pend(s32 timeoutms)
{
	osStatus status = osOK;
	status = (osStatus)osMutexWait(osMutexId(_handle), (uint32_t)timeoutms);
	if (status != osOK) {
		ERR("%s: status = %d.\n", _name, status);
		kernel::on_error(ERR_OPERATION_FAILED, this);
		return false;
	}
	
	return true;
}

BOOL mutex::post(s32 timeoutms)
{
	osStatus status = osOK;
	status = (osStatus)osMutexRelease(osMutexId(_handle));
	if (status != osOK) {
		ERR("%s: status = %d.\n", _name, status);
		kernel::on_error(ERR_OPERATION_FAILED, this);
		return false;
	}
	
	return true;
}

}
/***********************************************************************
** End of file
***********************************************************************/