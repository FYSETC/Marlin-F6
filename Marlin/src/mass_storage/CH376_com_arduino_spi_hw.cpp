/* CH376芯片 硬件标准异步串口连接的硬件抽象层 V1.0 */
/* 提供I/O接口子程序 */

#include "../MarlinConfig.h"
#include "CH376_hal.h"
#include "CH376_debug.h"
#include "../pins.h"


// Only support 1280 and 2560 at the moment!
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

#ifdef FYS_STORAGE_SPI

// SPI RATE :  top rate:0 old:2
#ifdef FYS_STORAGE_USBMODE
#define SPI_INIT_RATE 2 
#elif defined(FYS_STORAGE_SDCARD)
#define SPI_INIT_RATE 0
#endif

#define	SPI_IF_TRANS	0x80	/* SPI字节传输完成标志,在SPSR的位7 */

#define CH376_SPI_BZ  // 是否启用 busy 管脚查询  

#define		INT_PIN_INIT	DDRD &= ~_BV(PIND5)
#define		INT_PIN_SET(x)	if(x)PORTD|=_BV(PIND5);else PORTD&=~_BV(PIND5)
#define		INT_PIN_VAL	((bool)(PIND&_BV(PIND5)))

/**
 * Initialize hardware SPI
 * Set SCK rate to F_CPU/pow(2, 1 + spiRate) for spiRate [0,6]
 */   
static void spiInit(uint8_t spiRate) {
  // See avr processor documentation
  SPCR = _BV(SPE) | _BV(MSTR) /*| _BV(CPOL) | _BV(CPHA)*/ | (spiRate >> 1);
  SPSR = spiRate & 1 || spiRate == 6 ? 0 : _BV(SPI2X);
}

void  CH376_PORT_INIT( void )  /* 由于使用SPI读写时序,所以进行初始化 */
{
  SET_INPUT(CH376_INT_PIN);
  SET_INPUT(CH376_BUSY_PIN);
  SET_OUTPUT(CH376_RESET_PIN);  

  OUT_WRITE(CH376_SPI_SS_PIN, HIGH);
  SET_INPUT_PULLUP(CH376_SPI_MISO_PIN);
  SET_OUTPUT(CH376_SPI_MOSI_PIN);
  SET_OUTPUT(CH376_SPI_SCK_PIN);

  // 硬件复位之后需要100 ms 的延时
  WRITE(CH376_RESET_PIN,HIGH);
  CH376Delayms(10);
  WRITE(CH376_RESET_PIN,LOW);
  CH376Delayms(100);

  // SS must be in output mode even it is not chip select
  //pinMode(CH376_SPI_SS_PIN,OUTPUT);
  // set SS high - may be chip select for another SPI device
  //#if SET_SPI_SS_HIGH
  //  digitalWrite(CH376_SPI_SS_PIN, HIGH);
  //#endif  // SET_SPI_SS_HIGH
  // set SCK rate for initialization commands
  /*
  // 如果是硬件SPI接口,那么可使用mode3(CPOL=1&CPHA=1)或mode0(CPOL=0&CPHA=0),CH376在时钟上升沿采样输入,下降沿输出,数据位是高位在前 
  CH376_SPI_SCS = 1;  // 禁止SPI片选
  // 对于双向I/O引脚模拟SPI接口,那么必须在此设置SPI_SCS,SPI_SCK,SPI_SDI为输出方向,SPI_SDO为输入方向
  SPCR = 0x5C;  // 设置SPI模式3, DORD=0(MSB first), CPOL=1, CPHA=1, CH376也支持SPI模式0
  */
  spiInit(SPI_INIT_RATE);  
}

void  mDelay0_5uS( void )  /* 至少延时0.5uS,根据单片机主频调整 */
{
  UINT8 i =10; // old:20 - 15
  while(i--);
}

UINT8 Spi376Exchange( UINT8 d )  /* 硬件SPI输出且输入8个位数据 */
{  
  /*
  // 为了提高速度,可以将该子程序做成宏以减少子程序调用层次
  SPDR = d;  // 先将数据写入SPI数据寄存器,然后查询SPI状态寄存器以等待SPI字节传输完成 
  while ( ( SPSR & SPI_IF_TRANS ) == 0 );  // 查询SPI状态寄存器以等待SPI字节传输完成 
  SPSR &= ~ SPI_IF_TRANS;  // 清除SPI字节传输完成标志,有的单片机会自动清除 
  return( SPDR );  // 先查询SPI状态寄存器以等待SPI字节传输完成,然后从SPI数据寄存器读出数据 
  */
  CH376DebugOut("USB-!");
  SPDR = d;
  while (!TEST(SPSR, SPIF)) { /* Intentionally left empty */ }
  CH376DebugOut("USB!");
  SPSR &= ~ SPI_IF_TRANS;  // 清除SPI字节传输完成标志,有的单片机会自动清除 
  return( SPDR );
}


//#define	xEndCH376Cmd( )  /* 结束CH376命令,仅用于SPI接口方式 */

void xEndCH376Cmd()
{
  WRITE(CH376_SPI_SS_PIN, HIGH);
}

void	xWriteCH376Cmd( UINT8 mCmd )  /* 向CH376写命令 */
{
  #ifdef	CH376_SPI_BZ
    UINT8 i;
  #endif

  /*
  CH376_SPI_SCS = 1;  // 防止之前未通过xEndCH376Cmd禁止SPI片选 
  // 对于双向I/O引脚模拟SPI接口,那么必须确保已经设置SPI_SCS,SPI_SCK,SPI_SDI为输出方向,SPI_SDO为输入方向
  CH376_SPI_SCS = 0;  // SPI片选有效
  Spi376Exchange( mCmd );  // 发出命令码 
  */
  
  WRITE(CH376_SPI_SS_PIN, HIGH);
  WRITE(CH376_SPI_SS_PIN, LOW);
  CH376DebugOut("USB 1!");
  Spi376Exchange( mCmd );  // 发出命令码 
  CH376DebugOut("USB 2!");
    
  #ifdef	CH376_SPI_BZ  
    // SPI忙状态查询,等待CH376不忙,或者下面一行的延时1.5uS代替
    for ( i = 30; i != 0 && READ(CH376_BUSY_PIN)==HIGH; -- i );  
  #else
    // 延时1.5uS确保读写周期大于1.5uS,或者用上面一行的状态查询代替
    mDelay0_5uS( ); mDelay0_5uS( ); mDelay0_5uS( );  
  #endif

  CH376DebugOut("USB 3!");
}

void	xWriteCH376Data( UINT8 mData )  /* 向CH376写数据 */
{
  Spi376Exchange( mData );
  mDelay0_5uS( );  /* 确保读写周期大于0.6uS */
  CH376DebugOut("USB 4!");
}

UINT8	xReadCH376Data( void )  /* 从CH376读数据 */
{
 	mDelay0_5uS( );  /* 确保读写周期大于0.6uS */
	return( Spi376Exchange( 0xFF ) );
}

/* 查询CH376中断(INT#低电平) */
UINT8	Query376Interrupt( void )
{
  return (READ(CH376_INT_PIN)==HIGH)?FALSE:TRUE;
}

UINT8	mInitCH376(   EM_STORAGE_TYPE storage_type )  /* 初始化CH376 */
{
	UINT8	res;
	
	/* 接口硬件初始化 */
	CH376_PORT_INIT( );  

	/* 测试单片机与CH376之间的通讯接口 */
	// 会将数据取反，再发送回来
	CH376DebugOut("USB checking exist!");
	xWriteCH376Cmd( CMD11_CHECK_EXIST );  
	xWriteCH376Data( 0x65 );
	CH376Delayus( 20 );
	res = xReadCH376Data( );
 	xEndCH376Cmd( );  	
 	if ( res != 0x9A ) { 
 	  /* 通讯接口不正常,可能原因有:
 	  // 接口连接异常,其它设备影响(片选不唯一),
 	  // 串口波特率,一直在复位,晶振不工作 */
 	  CH376DebugOutErr(res);
 	  return( ERR_USB_UNKNOWN ); 
 	}

  /* 设备USB工作模式 */
  //#ifdef FYS_STORAGE_USBMODE
  if(storage_type == EMST_USB_DISK) {
    CH376DebugOut("USB setting USB mode!");
  	xWriteCH376Cmd( CMD11_SET_USB_MODE );  
  	xWriteCH376Data( 0x06 );
  	CH376Delayus( 20 );  // 异步串口方式不需要
  	res = xReadCH376Data( );
  	xEndCH376Cmd( );
  	CH376DebugOutPair("USB set:", res);
	}
	//#endif

  //#ifdef FYS_STORAGE_SDCARD
  if(storage_type == EMST_SDCARD) {
  	CH376DebugOut("USB setting SDCARD mode!");
  	xWriteCH376Cmd( CMD11_SET_USB_MODE );  
  	xWriteCH376Data( 0x03 );
  	CH376Delayus( 20 );  // 异步串口方式不需要
  	res = xReadCH376Data( );
  	xEndCH376Cmd( );
  	CH376DebugOutPair("USB set:", res);
	}
	//#endif
	
	if ( res == CMD_RET_SUCCESS ) return( USB_INT_SUCCESS );
	else return( ERR_USB_UNKNOWN );  /* 设置模式错误 */
}

/* 初始化CH376 */
UINT8	CH376_init( EM_STORAGE_TYPE storage_type)  
{
	CH376Delayms( 100 );  /* 延时100毫秒 */
	mInitSTDIO( );  /* DEBUG串口初始化 为了让计算机通过串口监控演示过程 */
	CH376DebugOut( "USB Start" );

	return mInitCH376(storage_type);
}

#endif

#endif


