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
#include "mutex.h"


namespace os {

mutex::mutex(void)
{

}

mutex::~mutex(void)
{

}

BOOL mutex::create(PCSTR os_name)
{
    _os_name = os_name;
	_os_handle =(HANDLE)osMutexCreate(NULL);
    if (_os_handle == NULL) {
		ERR("%s: _handle = %d.\n", _os_name, _os_handle);
		kernel::on_error(ERR_OPERATION_FAILED, this);
		return false;
    }

    DBG("%s: mutex create success.\n", _os_name);
    return true;
}

BOOL mutex::m_delete(void)
{
	osStatus status = osOK;
	status = osMutexDelete(osMutexId(_os_handle));
	if (status != osOK) {
		ERR("%s: status = %d.\n", _os_name, status);
		kernel::on_error(ERR_OPERATION_FAILED, this);
		return false;
	}

	return true;
}

BOOL mutex::pend(s32 timeoutms)
{
	osStatus status = osOK;
	status = (osStatus)osMutexWait(osMutexId(_os_handle), (uint32_t)timeoutms);
	if (status != osOK) {
		ERR("%s: status = %d.\n", _os_name, status);
		kernel::on_error(ERR_OPERATION_FAILED, this);
		return false;
	}

	return true;
}

BOOL mutex::post(s32 timeoutms)
{
	osStatus status = osOK;
	status = (osStatus)osMutexRelease(osMutexId(_os_handle));
	if (status != osOK) {
		ERR("%s: status = %d.\n", _os_name, status);
		kernel::on_error(ERR_OPERATION_FAILED, this);
		return false;
	}

	return true;
}

}
/***********************************************************************
** End of file
***********************************************************************/