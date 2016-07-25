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
#include "os/msgque.h"

#include "includes.h"
#include "cmsis_os.h"

namespace os {

msgque::msgque(void) :
    _msgcnt(0)
{

}

msgque::~msgque(void)
{

}

BOOL msgque::create(PCSTR name, u32 msgcnt)
{
    _name = name;
    _msgcnt = msgcnt;
	
	/* osMessageQId  */
	const osMessageQDef_t params = {
		.queue_sz = (uint32_t)msgcnt,    ///< number of elements in the queue
		.item_sz = (uint32_t)sizeof(void *), ///< size of an item
	};
	_handle = (HANDLE)osMessageCreate(&params, NULL);
	if (_handle == NULL) {
		ERR("%s: _handle = %d.\n", _name, _handle);
		kernel::on_error(ERR_OPERATION_FAILED, this);
		return false;
	}

	return true;
}

BOOL msgque::q_delete(void)
{
	return false;
}

BOOL msgque::pend(void *pmsg, u32 *psize, s32 timeoutms)
{
	osEvent event;
	event.status = osOK;
	event = osMessageGet (osMessageQId(_handle), (uint32_t)timeoutms);
	if (event.status != osEventMessage) {
		ERR("%s: status = %d.\n", _name, event.status);
		kernel::on_error(ERR_OPERATION_FAILED, this);
		return false;
	}
	*(u32 *)pmsg = (u32)event.value.v;

	return true;
}

BOOL msgque::post(void *pmsg, u32 size, s32 timeoutms)
{
	osStatus status = osOK;
	status = osMessagePut ((osMessageQId)_handle, (uint32_t)pmsg, (uint32_t)timeoutms);
	if (status != osOK) {
		ERR("%s: status = %d.\n", _name, status);
		kernel::on_error(ERR_OPERATION_FAILED, this);
		return false;
	}
	
	return true;
}

void msgque::reset(void)
{

}

}
/***********************************************************************
** End of file
***********************************************************************/