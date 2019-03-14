/* CH376芯片 硬件抽象层 V1.0 */
/* 提供I/O接口子程序 */

#ifndef	__CH376_HAL_H__
#define __CH376_HAL_H__

#ifdef __CX51__
#ifndef __C51__
#define __C51__		1
#endif
#endif

#ifdef __C51__
#define	BIG_ENDIAN		1
#endif

#ifdef __C51__
#pragma NOAREGS
#endif

#ifdef __C51__
#include	<reg52.h>
#endif

#if defined(ARDUINO)
  #if ARDUINO < 100 
    #include <WProgram.h>
  #else 
    #include <Arduino.h> 
  #endif

  #define FORCE_INLINE __attribute__((always_inline)) inline
#endif

#include "CH376_include.h"
#include "../fastio.h"
//#include "pins.h"

typedef enum em_storage_type {
  EMST_SDCARD,
  EMST_USB_DISK
}EM_STORAGE_TYPE;


/* 附加的USB操作状态定义 */
#define		ERR_USB_UNKNOWN		0xFA	/* 未知错误,不应该发生的情况,需检查硬件或者程序错误 */

/*
#define SCK_PIN          52
#define MISO_PIN         50
#define MOSI_PIN         51
#define CH376_SPI_SS_PIN       53

// The following three pins must not be redefined for hardware SPI.
#define CH376_SPI_MOSI_PIN MOSI_PIN       // SPI Master Out Slave In pin
#define CH376_SPI_MISO_PIN MISO_PIN       // SPI Master In Slave Out pin
#define CH376_SPI_SCK_PIN SCK_PIN         // SPI Clock pin
*/

//#define SPI_INIT_RATE 0 // 0 最高速 

//#define	SPI_IF_TRANS	0x80	/* SPI字节传输完成标志,在SPSR的位7 */

#define CH376_INT_WIRE	/* 假定CH376的INT#引脚,如果未连接那么也可以通过查询串口中断状态码实现 */
//#define CH376_SPI_BZ  // 是否启用 busy 管脚查询  

//const int CH376_INT_PIN = 32;
//const int CH376_BUSY_PIN = 35;
/*
#define CH376_INT_PIN 82
#define CH376_BUSY_PIN 18
#define CH376_RESET_PIN 81
*/

// 与平台有关的延时函数
void	CH376Delayus( UINT8 us ); /* 延时指定微秒时间,根据单片机主频调整,不精确 */
void	CH376Delayms( UINT8 ms ); /* 延时指定毫秒时间,根据单片机主频调整,不精确 */


// 底层通信接口有关的函数实现
void	CH376_PORT_INIT( void );  		/* CH376通讯接口初始化 */

void	xEndCH376Cmd( void );			/* 结束CH376命令,仅用于SPI接口方式 */

void	xWriteCH376Cmd( UINT8 mCmd );	/* 向CH376写命令 */

void	xWriteCH376Data( UINT8 mData );	/* 向CH376写数据 */

UINT8	xReadCH376Data( void );			/* 从CH376读数据 */

//UINT8	CH376_init();			/* 初始化CH376 */

UINT8	CH376_init( EM_STORAGE_TYPE storage_type);			/* 初始化CH376 */

UINT8	Query376Interrupt( void );		/* 查询CH376中断(INT#引脚为低电平) */

#endif

