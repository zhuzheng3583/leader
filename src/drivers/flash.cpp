/*******************************Copyright (c)***************************
** 
** Porject name:	leader
** Created by:		zhuzheng<happyzhull@163.com>
** Created date:	2016/04/06
** Modified by:
** Modified date:
** Descriptions:
**
***********************************************************************/
#include "flash.h"

//#include "core.h"

namespace driver {


flash::flash(PCSTR name, s32 id) :
	device(name, id),
    _base(0),
    _offset(0)
{

}

flash::~flash(void)
{

}

s32 flash::probe(void)
{
	if (device::probe() < 0) {
		ERR("%s: failed to probe.\n", _name);
		goto fail0;
	}
	
	INF("%s: probe success.\n", _name);
	return 0;

fail0:
	return -1;	
}

s32 flash::remove(void)
{
	if (device::remove() < 0) {
		goto fail0;
	}

	return 0;

fail0:
    return -1;	
}



s32 flash::open(s32 flags)
{
	/* Unlock the Program memory */
	HAL_FLASH_Unlock();

	/* Clear all FLASH flags */
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
	/* Unlock the Program memory */
	HAL_FLASH_Lock();
	
	return 0;

fail:
    return -1;
}

s32 flash::close(void)
{
	HAL_FLASH_Lock(); 
	return 0;
}

s32 flash::erase(u32 page_start_addr, u32 page_n)
{
	u32 page_error = 0;

	ASSERT((_offset % FLASH_PAGE_SIZE == 0));
	/*Variable used for Erase procedure*/
	FLASH_EraseInitTypeDef EraseInitStruct;
  	/* Erase the user Flash area (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) */
  	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  	EraseInitStruct.PageAddress = page_start_addr;//FLASH_USER_START_ADDR;
  	EraseInitStruct.NbPages = page_n;//(FLASH_USER_END_ADDR - FLASH_USER_START_ADDR)/FLASH_PAGE_SIZE;

  	if (HAL_FLASHEx_Erase(&EraseInitStruct, &page_error) != HAL_OK) { 
	/*
		Error occurred while page erase. 
		User can add here some code to deal with this error. 
		PageError will contain the faulty page and then to know the code error on this page,
		user can call function 'HAL_FLASH_GetError()'
	*/
    	INF("%s: failed to erase.\n", _name);
		return -1;
	}	

	return 0;
}

s32 flash::read(u8 *buf, u32 count)
{
	s32 ret = 0;
	u32 start_addr = 0;
	u32 end_addr = 0;
	u32 cur_addr = 0;
	u32 page_n = 0;
	u8 *data = (u8 *)buf;
	u32 readcnt = 0;
	
	/* Unlock the Flash to enable the flash control register access */ 
	HAL_FLASH_Unlock();

	ASSERT((_offset % FLASH_PAGE_SIZE == 0));
	start_addr = FLASH_BASE + _offset;
	end_addr = start_addr + count;
	ASSERT((start_addr >= FLASH_USER_START_ADDR) && (end_addr <= FLASH_USER_END_ADDR));

	cur_addr = start_addr;
    for (readcnt = 0; readcnt < count; ) {
		data[readcnt++] = *(volatile u8*)cur_addr++;
		_offset++;
    }

	/* Lock the Flash to disable the flash control register access (recommended
	to protect the FLASH memory against possible unwanted operation) */
	HAL_FLASH_Lock(); 

	return readcnt;
}

s32 flash::write(u8 *buf, u32 count)
{
	s32 ret = 0;
	u32 start_addr = 0;
	u32 end_addr = 0;
	u32 cur_addr = 0;
	u32 page_n = 0;
	u8 *data = (u8 *)buf;
	u32 writecnt = 0;
	
	/* Unlock the Flash to enable the flash control register access */ 
	HAL_FLASH_Unlock();

	ASSERT((_offset % FLASH_PAGE_SIZE == 0));
	start_addr = FLASH_BASE + _offset;
	end_addr = start_addr + count;
	ASSERT((start_addr >= FLASH_USER_START_ADDR) && (end_addr <= FLASH_USER_END_ADDR));

	if (count % FLASH_PAGE_SIZE == 0) {
		page_n = count / FLASH_PAGE_SIZE;
	} else {
		page_n = count / FLASH_PAGE_SIZE + 1;
	}
	ret = flash::erase(start_addr, page_n);
	if (ret < 0) {
		ERR("%s: failed to flash::erase.\n", _name);
	}

	cur_addr = start_addr;
    for (writecnt = 0; writecnt < count; ) {
	    	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, cur_addr, *((u32 *)(data + writecnt))) == HAL_OK) {
			_offset += 4;
			cur_addr += 4;
			writecnt += 4;
	    	} else {
			/* Error occurred while writing data in Flash memory. 
			User can add here some code to deal with this error */
			ERR("%s: failed to flash::erase.\n", _name);
			return writecnt;
	    	}
    }

	/* Lock the Flash to disable the flash control register access (recommended
	to protect the FLASH memory against possible unwanted operation) */
	HAL_FLASH_Lock(); 

	return writecnt;
}



s32 flash::seek(s32 offset, enum seek_mode mode)
{
	switch (mode) {
	case SEEK_SET_M:
		_offset = offset;
		break;
	case SEEK_CUR_M:
		
		break;
	case SEEK_END_M:

		break;
	default:
        break;
	}
	return 0;
}

s32 flash::tell(void)
{
	return _offset;
}

s32 flash::self_test(void)
{
   	s32 ret = 0;
	u8 wbuf[16] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
			0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0xAB };
	u8 rbuf[16] = { 0 };

	flash::open(NULL);
	flash::seek(OFFSET_FLASH_PAGE_63, SEEK_SET_M);
	flash::write(wbuf, 16);
	flash::seek(OFFSET_FLASH_PAGE_63, SEEK_SET_M);
	flash::read(rbuf, 16);
	ret = memcmp((char const *)wbuf, (char const *)rbuf, 16);
	if(ret != 0) {
		INF("%s: failed to flash::self_test.\n", _name);
		CAPTURE_ERR();
	}
	
    return ret;
}

}
/***********************************************************************
** End of file
***********************************************************************/

