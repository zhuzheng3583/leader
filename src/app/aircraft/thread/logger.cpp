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

#include "system_uav.h"

#include <string.h>

#define LOG_BUFFER_SIZE (8 * 1024)

using namespace driver;
using namespace os;

namespace app {

Logger::Logger(Fops *pfile, uint32_t bufsize)
{
	m_param.name = "logger";
	m_param.priority = 10;
	m_param.stackbase = NULL;
	m_param.stacksize = 512;
	m_param.run = Task::func;
	m_param.parg = this;

	m_level = -1; //
	m_log_buf_threshold = 128;

    Logger::attach(pfile);
    if (bufsize == NULL) {
        m_log_buf_size = LOG_BUFFER_SIZE;
    } else {
        m_log_buf_size = bufsize;
    }
	m_pcircbuf = new Circbuf(m_log_buf_size);
}

Logger::~Logger(void)
{
	m_level = -2;
	delete m_pcircbuf;
	m_pcircbuf = NULL;
	Logger::detach();

}

BOOL Logger::create(struct task_param *pparam)
{
    BOOL b = false;
    m_pmutex = new Mutex;
    b = m_pmutex->create("log_mutex");
    if (b < 0) {
        goto err;
    }

    return Task::create(pparam);

err:
    delete m_pmutex;
    return false;
}

BOOL Logger::t_delete(void)
{
    delete m_pmutex;
    return Task::t_delete();
}


int32_t Logger::vprintf(PCSTR fmt, va_list ap)
{
	uint8_t str[FMT_MAX_CNT];
	int32_t count = 0;

	//if (m_pmutex != NULL) { m_pmutex->pend(WAIT_FOREVER); }
	// count = snprintf(str, FMT_MAX_CNT, fmt, ap);
	// count = sprintf(str, fmt, ap);
	count = vsnprintf((char *)str, FMT_MAX_CNT, fmt, ap);
	str[count-1] = '\r';
	str[count-0] = '\n';
	count = m_pcircbuf->mem_producer((uint8_t *)str, count+1);
    //if (m_pmutex != NULL) { m_pmutex->post(WAIT_FOREVER); }

	return count;
}

void Logger::flush(void)
{
	//INIT_CRITICAL_SECTION();
	//ENTER_CRITICAL_SECTION();
	//Interrupt::disable_all_irq();
	if (m_pcircbuf->get_used_size() > m_log_buf_threshold) {
		m_pcircbuf->dev_consumer_to_empty(m_pfile);
	}
    //Interrupt::enable_all_irq();
	//EXIT_CRITICAL_SECTION();
}

int32_t Logger::self_test(void)
{
    INF("self test: check circbuf.\n");
    uint32_t testcnt = 500;

    for (uint32_t n = 0; n < testcnt; n++)
    {
        INF("%u.\n", n);
        Logger::flush();
    }

    for (uint32_t n = 0; n < testcnt; n++)
    {
        ERR("%u.\n", n);
        WRN("%u.\n", n);
        INF("%u.\n", n);
        DBG("%u.\n", n);
		//INF("xxx111111111111111111111122222222222222222222222222xxx.\n");
        Logger::flush();
    }

    return true;
}


void Logger::run(void *parg)
{
	for ( ; ; )
	{
		Logger::flush();
		msleep(300);
	}

}

}
/***********************************************************************
** End of file
***********************************************************************/

