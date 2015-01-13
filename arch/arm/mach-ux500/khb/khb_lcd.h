/*****************************************************************************
** COPYRIGHT(C) : Samsung Electronics Co.Ltd, 2002-2006 ALL RIGHTS RESERVED
** AUTHOR               : KyoungHOON Kim (khoonk)
******************************************************************************/

#ifndef _KHB_LCD_H_
#define _KHB_LCD_H_
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/types.h>
#include <linux/dma-mapping.h>


#ifdef CONFIG_PM
#define PM_DEBUG 1
#endif
 
/*****************************************************************************
** Macro definitions 
******************************************************************************/

#define CONFIG_FB_LCD_WVGA

//#define CONFIG_LCD_COLOR_DEPTH_16
#undef CONFIG_LCD_COLOR_DEPTH_16


#if defined(CONFIG_FB_LCD_WVGA)
#define PANEL_W                                         (480)
#define PANEL_H                                         (800)
#elif defined(CONFIG_FB_LCD_WQVGA)
#define PANEL_W                     (240)
#define PANEL_H                     (400)
#endif


#define COLOR_RED                               0xff0000
#define COLOR_GREEN                             0x00ff00
#define COLOR_BLUE                              0x0000ff
#define COLOR_WHITE                             0xffffff
#define COLOR_BLACK                             0x000000


/*****************************************************************************
** externs
******************************************************************************/
#if defined(CONFIG_LCD_COLOR_DEPTH_16)
	extern unsigned short*  fbpMem;
#else
	extern unsigned int*    fbpMem;
#endif

extern int fd_line_length;

/*****************************************************************************
** forward declarations
******************************************************************************/
void lcd_PutPixel(int x, int y, unsigned int color);
void lcd_ClearScr(unsigned int color);
void lcd_Line(int x1, int y1, int x2, int y2, unsigned int color);
void lcd_Rectangle(int x1, int y1, int x2, int y2, unsigned int color);
void lcd_FilledRectangle(int x1, int y1, int x2, int y2, unsigned int color);
void lcd_draw_font(int x,int y,int fgc,int bgc,unsigned char len, unsigned char * data, unsigned char fontSize);

#endif /* #ifndef _KHB_LCD_H_  */
