/*******************************Copyright (c)***************************
**
** Porject name:	leader
** Created by:	zhuzheng<happyzhull@163.com>
** Created date:	2016/07/23
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#include "logger.h"
#include "leader_system.h"

#include <string.h>

#define LOG_BUFFER_SIZE (4 * 1024)

using namespace driver;
using namespace os;

namespace app {

logger::logger(void)
{
	_params.name = "logger";
	_params.priority = 0;
	_params.stacksize = 512;
	_params.func = (void *)thread::func;
	_params.parg = this;

	_level = -1;
	_buf_threshold = 128;

	_buf_size = LOG_BUFFER_SIZE;
	_pcircbuf = new circbuf(_buf_size);
}

logger::~logger(void)
{
	_level = -2;
	delete _pcircbuf;
	_pcircbuf = NULL;
}

void logger::attach(device *pdev)
{
	_pdev = pdev;
}

void logger::detach(void)
{
	_pdev = NULL;
}

BOOL logger::create(struct thread_params *pparams)
{
    BOOL b = false;
    _pmutex = new mutex;
    b = _pmutex->create("log_mutex");
    if (b < 0) {
        goto err;
    }

    return thread::create(pparams);

err:
    delete _pmutex;
    return false;
}

BOOL logger::t_delete(void)
{
    delete _pmutex;
    return thread::t_delete();
}

s32 logger::vprintf(PCSTR fmt, va_list ap)
{
	u8 str[FMT_MAX_CNT];
	s32 count = 0;

	//if (_pmutex != NULL) { _pmutex->pend(WAIT_FOREVER); }
	// count = snprintf(str, FMT_MAX_CNT, fmt, ap);
	// count = sprintf(str, fmt, ap);
	count = vsnprintf((char *)str, FMT_MAX_CNT, fmt, ap);
	str[count-1] = '\r';
	str[count-0] = '\n';
	count = _pcircbuf->mem_producer((u8 *)str, count+1);
    //if (_pmutex != NULL) { _pmutex->post(WAIT_FOREVER); }

	return count;
}

void logger::flush(void)
{
	//INIT_CRITICAL_SECTION();
	//ENTER_CRITICAL_SECTION();
	//Interrupt::disable_all_irq();
	//if (_pcircbuf->get_used_size() > _buf_threshold) {
		//_pcircbuf->dev_consumer_to_empty(_pdev);
		//_pcircbuf->dev_consumer(_pdev, _pcircbuf->get_used_size());
	//}
    //Interrupt::enable_all_irq();
	//EXIT_CRITICAL_SECTION();
  
    _pcircbuf->dev_consumer(_pdev, _pcircbuf->get_used_size());
}

int32_t logger::self_test(void)
{
    INF("self test: check circbuf.\n");
    uint32_t testcnt = 500;

    for (u32 n = 0; n < testcnt; n++)
    {
        INF("%u.\n", n);
        logger::flush();
    }

    for (u32 n = 0; n < testcnt; n++)
    {
        ERR("%u.\n", n);
        WRN("%u.\n", n);
        INF("%u.\n", n);
        DBG("%u.\n", n);
		//INF("xxx111111111111111111111122222222222222222222222222xxx.\n");
        logger::flush();
    }

	for (u32 n = 0; /*n < testcnt*/; n++) {
		INF("%d-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d.\n",
			n, n, n, n, n, n, n, n, n, n, n, n, n, n, n, n);
        logger::flush();
        core::mdelay(100);
    }


    return true;
}


void logger::run(void *parg)
{
	for ( ; ; )
	{
		logger::flush();
		msleep(300);
	}

}

}
/***********************************************************************
** End of file
***********************************************************************/

