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

#pragma once
#include "type_leader.h"
#include "misc_leader.h"
#include "task.h"

#include "fops.h"
#include "circbuf.h"

#include "mutex.h"

#define FMT_MAX_CNT							256

using namespace driver;
using namespace os;

namespace app {

class Logger : public Task
{
public:
	Logger(Fops *pfile, uint32_t bufsize);
	~Logger(void);

public:
	Circbuf *m_pcircbuf;
	Fops	*m_pfile;

	uint32_t 	m_log_buf_size;
	uint32_t	m_log_buf_threshold;
	int32_t		m_level;

	Mutex	*m_pmutex;

public:
	BOOL create(struct task_param *pparam);
	BOOL t_delete(void);

	void attach(Fops *pfile)	{ m_pfile = pfile; }
	void detach(void)			{ m_pfile = NULL; }

	int32_t vprintf(PCSTR fmt, va_list ap);

	void flush(void);

	int32_t no_os_self_test(void);	// 检查环形缓冲区是否工作正常,及时刷缓冲区
	int32_t self_test(void);		// 通过任务调度刷新缓冲区

public:
	void set_log_buf_threshold(int32_t threshold)	{ m_log_buf_threshold = threshold; }
	int32_t get_log_buf_threshold(void)				{ return m_log_buf_threshold; }

	void set_log_level(int32_t level)				{ m_level = level; }
	int32_t get_log_level(void) 					{ return m_level; }


public:
	virtual void run(void *parg);
};

}
/***********************************************************************
** End of file
***********************************************************************/

