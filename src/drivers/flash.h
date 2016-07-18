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
#pragma once
#include "leader_type.h"
#include "leader_misc.h"

#include "device.h"

#define FLASH_USER_START_ADDR OFFSET_TO_ADDR_FLASH_PAGE(OFFSET_FLASH_PAGE_32)   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR   OFFSET_TO_ADDR_FLASH_PAGE(OFFSET_FLASH_PAGE_127)   /* End @ of user Flash area */
#define FLASH_PAGE_SIZE	    0x800                /* Size of page : 2 Kbytes */

#define OFFSET_TO_ADDR_FLASH_PAGE(offset)   ((u32)((offset) + FLASH_BASE))
#define ADDR_FLASH_PAGE_TO_OFFSET(addr)     ((u32)((addr) - FLASH_BASE))

/* Base address of the Flash pages */
#define OFFSET_FLASH_PAGE_0     ((u32)0x00000000) /* Base @ of Page 0, 2 Kbytes */
#define OFFSET_FLASH_PAGE_1     ((u32)0x00000800) /* Base @ of Page 1, 2 Kbytes */
#define OFFSET_FLASH_PAGE_2     ((u32)0x00001000) /* Base @ of Page 2, 2 Kbytes */
#define OFFSET_FLASH_PAGE_3     ((u32)0x00001800) /* Base @ of Page 3, 2 Kbytes */
#define OFFSET_FLASH_PAGE_4     ((u32)0x00002000) /* Base @ of Page 4, 2 Kbytes */
#define OFFSET_FLASH_PAGE_5     ((u32)0x00002800) /* Base @ of Page 5, 2 Kbytes */
#define OFFSET_FLASH_PAGE_6     ((u32)0x00003000) /* Base @ of Page 6, 2 Kbytes */
#define OFFSET_FLASH_PAGE_7     ((u32)0x00003800) /* Base @ of Page 7, 2 Kbytes */
#define OFFSET_FLASH_PAGE_8     ((u32)0x00004000) /* Base @ of Page 8, 2 Kbytes */
#define OFFSET_FLASH_PAGE_9     ((u32)0x00004800) /* Base @ of Page 9, 2 Kbytes */
#define OFFSET_FLASH_PAGE_10    ((u32)0x00005000) /* Base @ of Page 10, 2 Kbytes */
#define OFFSET_FLASH_PAGE_11    ((u32)0x00005800) /* Base @ of Page 11, 2 Kbytes */
#define OFFSET_FLASH_PAGE_12    ((u32)0x00006000) /* Base @ of Page 12, 2 Kbytes */
#define OFFSET_FLASH_PAGE_13    ((u32)0x00006800) /* Base @ of Page 13, 2 Kbytes */
#define OFFSET_FLASH_PAGE_14    ((u32)0x00007000) /* Base @ of Page 14, 2 Kbytes */
#define OFFSET_FLASH_PAGE_15    ((u32)0x00007800) /* Base @ of Page 15, 2 Kbytes */
#define OFFSET_FLASH_PAGE_16    ((u32)0x00008000) /* Base @ of Page 16, 2 Kbytes */
#define OFFSET_FLASH_PAGE_17    ((u32)0x00008800) /* Base @ of Page 17, 2 Kbytes */
#define OFFSET_FLASH_PAGE_18    ((u32)0x00009000) /* Base @ of Page 18, 2 Kbytes */
#define OFFSET_FLASH_PAGE_19    ((u32)0x00009800) /* Base @ of Page 19, 2 Kbytes */
#define OFFSET_FLASH_PAGE_20    ((u32)0x0000A000) /* Base @ of Page 20, 2 Kbytes */
#define OFFSET_FLASH_PAGE_21    ((u32)0x0000A800) /* Base @ of Page 21, 2 Kbytes  */
#define OFFSET_FLASH_PAGE_22    ((u32)0x0000B000) /* Base @ of Page 22, 2 Kbytes  */
#define OFFSET_FLASH_PAGE_23    ((u32)0x0000B800) /* Base @ of Page 23, 2 Kbytes */
#define OFFSET_FLASH_PAGE_24    ((u32)0x0000C000) /* Base @ of Page 24, 2 Kbytes */
#define OFFSET_FLASH_PAGE_25    ((u32)0x0000C800) /* Base @ of Page 25, 2 Kbytes */
#define OFFSET_FLASH_PAGE_26    ((u32)0x0000D000) /* Base @ of Page 26, 2 Kbytes */
#define OFFSET_FLASH_PAGE_27    ((u32)0x0000D800) /* Base @ of Page 27, 2 Kbytes */
#define OFFSET_FLASH_PAGE_28    ((u32)0x0000E000) /* Base @ of Page 28, 2 Kbytes */
#define OFFSET_FLASH_PAGE_29    ((u32)0x0000E800) /* Base @ of Page 29, 2 Kbytes */
#define OFFSET_FLASH_PAGE_30    ((u32)0x0000F000) /* Base @ of Page 30, 2 Kbytes */
#define OFFSET_FLASH_PAGE_31    ((u32)0x0000F800) /* Base @ of Page 31, 2 Kbytes */
#define OFFSET_FLASH_PAGE_32    ((u32)0x00010000) /* Base @ of Page 32, 2 Kbytes */
#define OFFSET_FLASH_PAGE_33    ((u32)0x00010800) /* Base @ of Page 33, 2 Kbytes */
#define OFFSET_FLASH_PAGE_34    ((u32)0x00011000) /* Base @ of Page 34, 2 Kbytes */
#define OFFSET_FLASH_PAGE_35    ((u32)0x00011800) /* Base @ of Page 35, 2 Kbytes */
#define OFFSET_FLASH_PAGE_36    ((u32)0x00012000) /* Base @ of Page 36, 2 Kbytes */
#define OFFSET_FLASH_PAGE_37    ((u32)0x00012800) /* Base @ of Page 37, 2 Kbytes */
#define OFFSET_FLASH_PAGE_38    ((u32)0x00013000) /* Base @ of Page 38, 2 Kbytes */
#define OFFSET_FLASH_PAGE_39    ((u32)0x00013800) /* Base @ of Page 39, 2 Kbytes */
#define OFFSET_FLASH_PAGE_40    ((u32)0x00014000) /* Base @ of Page 40, 2 Kbytes */
#define OFFSET_FLASH_PAGE_41    ((u32)0x00014800) /* Base @ of Page 41, 2 Kbytes */
#define OFFSET_FLASH_PAGE_42    ((u32)0x00015000) /* Base @ of Page 42, 2 Kbytes */
#define OFFSET_FLASH_PAGE_43    ((u32)0x00015800) /* Base @ of Page 43, 2 Kbytes */
#define OFFSET_FLASH_PAGE_44    ((u32)0x00016000) /* Base @ of Page 44, 2 Kbytes */
#define OFFSET_FLASH_PAGE_45    ((u32)0x00016800) /* Base @ of Page 45, 2 Kbytes */
#define OFFSET_FLASH_PAGE_46    ((u32)0x00017000) /* Base @ of Page 46, 2 Kbytes */
#define OFFSET_FLASH_PAGE_47    ((u32)0x00017800) /* Base @ of Page 47, 2 Kbytes */
#define OFFSET_FLASH_PAGE_48    ((u32)0x00018000) /* Base @ of Page 48, 2 Kbytes */
#define OFFSET_FLASH_PAGE_49    ((u32)0x00018800) /* Base @ of Page 49, 2 Kbytes */
#define OFFSET_FLASH_PAGE_50    ((u32)0x00019000) /* Base @ of Page 50, 2 Kbytes */
#define OFFSET_FLASH_PAGE_51    ((u32)0x00019800) /* Base @ of Page 51, 2 Kbytes */
#define OFFSET_FLASH_PAGE_52    ((u32)0x0001A000) /* Base @ of Page 52, 2 Kbytes */
#define OFFSET_FLASH_PAGE_53    ((u32)0x0001A800) /* Base @ of Page 53, 2 Kbytes  */
#define OFFSET_FLASH_PAGE_54    ((u32)0x0001B000) /* Base @ of Page 54, 2 Kbytes  */
#define OFFSET_FLASH_PAGE_55    ((u32)0x0001B800) /* Base @ of Page 55, 2 Kbytes */
#define OFFSET_FLASH_PAGE_56    ((u32)0x0001C000) /* Base @ of Page 56, 2 Kbytes */
#define OFFSET_FLASH_PAGE_57    ((u32)0x0001C800) /* Base @ of Page 57, 2 Kbytes */
#define OFFSET_FLASH_PAGE_58    ((u32)0x0001D000) /* Base @ of Page 58, 2 Kbytes */
#define OFFSET_FLASH_PAGE_59    ((u32)0x0001D800) /* Base @ of Page 59, 2 Kbytes */
#define OFFSET_FLASH_PAGE_60    ((u32)0x0001E000) /* Base @ of Page 60, 2 Kbytes */
#define OFFSET_FLASH_PAGE_61    ((u32)0x0001E800) /* Base @ of Page 61, 2 Kbytes */
#define OFFSET_FLASH_PAGE_62    ((u32)0x0001F000) /* Base @ of Page 62, 2 Kbytes */
#define OFFSET_FLASH_PAGE_63    ((u32)0x0001F800) /* Base @ of Page 63, 2 Kbytes */
#define OFFSET_FLASH_PAGE_64    ((u32)0x00020000) /* Base @ of Page 64, 2 Kbytes */
#define OFFSET_FLASH_PAGE_65    ((u32)0x00020800) /* Base @ of Page 65, 2 Kbytes */
#define OFFSET_FLASH_PAGE_66    ((u32)0x00021000) /* Base @ of Page 66, 2 Kbytes */
#define OFFSET_FLASH_PAGE_67    ((u32)0x00021800) /* Base @ of Page 67, 2 Kbytes */
#define OFFSET_FLASH_PAGE_68    ((u32)0x00022000) /* Base @ of Page 68, 2 Kbytes */
#define OFFSET_FLASH_PAGE_69    ((u32)0x00022800) /* Base @ of Page 69, 2 Kbytes */
#define OFFSET_FLASH_PAGE_70    ((u32)0x00023000) /* Base @ of Page 70, 2 Kbytes */
#define OFFSET_FLASH_PAGE_71    ((u32)0x00023800) /* Base @ of Page 71, 2 Kbytes */
#define OFFSET_FLASH_PAGE_72    ((u32)0x00024000) /* Base @ of Page 72, 2 Kbytes */
#define OFFSET_FLASH_PAGE_73    ((u32)0x00024800) /* Base @ of Page 73, 2 Kbytes */
#define OFFSET_FLASH_PAGE_74    ((u32)0x00025000) /* Base @ of Page 74, 2 Kbytes */
#define OFFSET_FLASH_PAGE_75    ((u32)0x00025800) /* Base @ of Page 75, 2 Kbytes */
#define OFFSET_FLASH_PAGE_76    ((u32)0x00026000) /* Base @ of Page 76, 2 Kbytes */
#define OFFSET_FLASH_PAGE_77    ((u32)0x00026800) /* Base @ of Page 77, 2 Kbytes */
#define OFFSET_FLASH_PAGE_78    ((u32)0x00027000) /* Base @ of Page 78, 2 Kbytes */
#define OFFSET_FLASH_PAGE_79    ((u32)0x00027800) /* Base @ of Page 79, 2 Kbytes */
#define OFFSET_FLASH_PAGE_80    ((u32)0x00028000) /* Base @ of Page 80, 2 Kbytes */
#define OFFSET_FLASH_PAGE_81    ((u32)0x00028800) /* Base @ of Page 81, 2 Kbytes */
#define OFFSET_FLASH_PAGE_82    ((u32)0x00029000) /* Base @ of Page 82, 2 Kbytes */
#define OFFSET_FLASH_PAGE_83    ((u32)0x00029800) /* Base @ of Page 83, 2 Kbytes */
#define OFFSET_FLASH_PAGE_84    ((u32)0x0002A000) /* Base @ of Page 84, 2 Kbytes */
#define OFFSET_FLASH_PAGE_85    ((u32)0x0002A800) /* Base @ of Page 85, 2 Kbytes  */
#define OFFSET_FLASH_PAGE_86    ((u32)0x0002B000) /* Base @ of Page 86, 2 Kbytes  */
#define OFFSET_FLASH_PAGE_87    ((u32)0x0002B800) /* Base @ of Page 87, 2 Kbytes */
#define OFFSET_FLASH_PAGE_88    ((u32)0x0002C000) /* Base @ of Page 88, 2 Kbytes */
#define OFFSET_FLASH_PAGE_89    ((u32)0x0002C800) /* Base @ of Page 89, 2 Kbytes */
#define OFFSET_FLASH_PAGE_90    ((u32)0x0002D000) /* Base @ of Page 90, 2 Kbytes */
#define OFFSET_FLASH_PAGE_91    ((u32)0x0002D800) /* Base @ of Page 91, 2 Kbytes */
#define OFFSET_FLASH_PAGE_92    ((u32)0x0002E000) /* Base @ of Page 92, 2 Kbytes */
#define OFFSET_FLASH_PAGE_93    ((u32)0x0002E800) /* Base @ of Page 93, 2 Kbytes */
#define OFFSET_FLASH_PAGE_94    ((u32)0x0002F000) /* Base @ of Page 94, 2 Kbytes */
#define OFFSET_FLASH_PAGE_95    ((u32)0x0002F800) /* Base @ of Page 95, 2 Kbytes */
#define OFFSET_FLASH_PAGE_96    ((u32)0x00030000) /* Base @ of Page 96, 2 Kbytes */
#define OFFSET_FLASH_PAGE_97    ((u32)0x00030800) /* Base @ of Page 97, 2 Kbytes */
#define OFFSET_FLASH_PAGE_98    ((u32)0x00031000) /* Base @ of Page 98, 2 Kbytes */
#define OFFSET_FLASH_PAGE_99    ((u32)0x00031800) /* Base @ of Page 99, 2 Kbytes */
#define OFFSET_FLASH_PAGE_100   ((u32)0x00032000) /* Base @ of Page 100, 2 Kbytes */
#define OFFSET_FLASH_PAGE_101   ((u32)0x00032800) /* Base @ of Page 101, 2 Kbytes */
#define OFFSET_FLASH_PAGE_102   ((u32)0x00033000) /* Base @ of Page 102, 2 Kbytes */
#define OFFSET_FLASH_PAGE_103   ((u32)0x00033800) /* Base @ of Page 103, 2 Kbytes */
#define OFFSET_FLASH_PAGE_104   ((u32)0x00034000) /* Base @ of Page 104, 2 Kbytes */
#define OFFSET_FLASH_PAGE_105   ((u32)0x00034800) /* Base @ of Page 105, 2 Kbytes */
#define OFFSET_FLASH_PAGE_106   ((u32)0x00035000) /* Base @ of Page 106, 2 Kbytes */
#define OFFSET_FLASH_PAGE_107   ((u32)0x00035800) /* Base @ of Page 107, 2 Kbytes */
#define OFFSET_FLASH_PAGE_108   ((u32)0x00036000) /* Base @ of Page 108, 2 Kbytes */
#define OFFSET_FLASH_PAGE_109   ((u32)0x00036800) /* Base @ of Page 109, 2 Kbytes */
#define OFFSET_FLASH_PAGE_110   ((u32)0x00037000) /* Base @ of Page 110, 2 Kbytes */
#define OFFSET_FLASH_PAGE_111   ((u32)0x00037800) /* Base @ of Page 111, 2 Kbytes */
#define OFFSET_FLASH_PAGE_112   ((u32)0x00038000) /* Base @ of Page 112, 2 Kbytes */
#define OFFSET_FLASH_PAGE_113   ((u32)0x00038800) /* Base @ of Page 113, 2 Kbytes */
#define OFFSET_FLASH_PAGE_114   ((u32)0x00039000) /* Base @ of Page 114, 2 Kbytes */
#define OFFSET_FLASH_PAGE_115   ((u32)0x00039800) /* Base @ of Page 115, 2 Kbytes */
#define OFFSET_FLASH_PAGE_116   ((u32)0x0003A000) /* Base @ of Page 116, 2 Kbytes */
#define OFFSET_FLASH_PAGE_117   ((u32)0x0003A800) /* Base @ of Page 117, 2 Kbytes  */
#define OFFSET_FLASH_PAGE_118   ((u32)0x0003B000) /* Base @ of Page 118, 2 Kbytes  */
#define OFFSET_FLASH_PAGE_119   ((u32)0x0003B800) /* Base @ of Page 119, 2 Kbytes */
#define OFFSET_FLASH_PAGE_120   ((u32)0x0003C000) /* Base @ of Page 120, 2 Kbytes */
#define OFFSET_FLASH_PAGE_121   ((u32)0x0003C800) /* Base @ of Page 121, 2 Kbytes */
#define OFFSET_FLASH_PAGE_122   ((u32)0x0003D000) /* Base @ of Page 122, 2 Kbytes */
#define OFFSET_FLASH_PAGE_123   ((u32)0x0003D800) /* Base @ of Page 123, 2 Kbytes */
#define OFFSET_FLASH_PAGE_124   ((u32)0x0003E000) /* Base @ of Page 124, 2 Kbytes */
#define OFFSET_FLASH_PAGE_125   ((u32)0x0003E800) /* Base @ of Page 125, 2 Kbytes */
#define OFFSET_FLASH_PAGE_126   ((u32)0x0003F000) /* Base @ of Page 126, 2 Kbytes */
#define OFFSET_FLASH_PAGE_127   ((u32)0x0003F800) /* Base @ of Page 127, 2 Kbytes */

#define DATA_32                 ((u32)0x12345678)

namespace driver {

class flash : public device
{
public:
    flash(PCSTR name, s32 id);
    ~flash(void);

protected:
    u32 _base;
    u32 _offset;
    
public:
    s32 probe(void);
    s32 remove(void);

protected:
    inline s32 erase(u32 page_addr, u32 page_n);

public:
    s32 self_test(void);

public:
    virtual s32 open(s32 flags);
    virtual s32 read(u8 *buf, u32 count);
    virtual s32 write(u8 *buf, u32 count);
    virtual s32 close(void);

    	virtual s32 seek(s32 offset, enum seek_mode mode);
	virtual s32 tell(void);
};

}
/***********************************************************************
** End of file
***********************************************************************/

