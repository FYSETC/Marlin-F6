/* CH376芯片 硬件标准异步串口连接的硬件抽象层 V1.0 */
/* 提供I/O接口子程序 */

#include <HardwareSerial.h>
#include "../MarlinConfig.h"
#include "CH376_hal.h"
#include "CH376_debug.h"


// 其他Arduino芯片暂时不管
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

#ifdef FYS_STORAGE_UART

/* 本例中的硬件连接方式如下(实际应用电路可以参照修改下述定义及子程序) */
/* 单片机的引脚    CH376芯片的引脚
      TXD                  RXD
      RXD                  TXD       */

//#define CH376_INT_WIRE			INT0	/* 假定CH376的INT#引脚,如果未连接那么也可以通过查询串口中断状态码实现 */

#define	UART_INIT_BAUDRATE	9600	/* 默认通讯波特率9600bps,建议通过硬件引脚设定直接选择更高的CH376的默认通讯波特率 */
//#define	UART_WORK_BAUDRATE	115200	/* 正式通讯波特率 */

void	CH376_PORT_INIT( void )  /* 由于使用异步串口读写时序,所以进行初始化 */
{
  Serial3.begin(UART_INIT_BAUDRATE);
}

#ifdef	UART_WORK_BAUDRATE
void	SET_WORK_BAUDRATE( void )  /* 将单片机切换到正式通讯波特率 */
{
  Serial3.begin(UART_INIT_BAUDRATE);
}
#endif

//#define	xEndCH376Cmd( )  /* 结束CH376命令,仅用于SPI接口方式 */

void xEndCH376Cmd()
{

}

void	xWriteCH376Cmd( UINT8 mCmd )  /* 向CH376写命令 */
{
  // geo-f: 原始程序在发送完一个字节后是等待其发送完成的，
  // 但这个 write 函数好像并没有等待，而是放在了发送队列中
  Serial3.write(SER_SYNC_CODE1);
  //Serial3.flush();
  Serial3.write(SER_SYNC_CODE2);
  //Serial3.flush();
  Serial3.write(mCmd);
  //Serial3.flush();
}

void	xWriteCH376Data( UINT8 mData )  /* 向CH376写数据 */
{
  Serial3.write(mData);
  //Serial3.flush();
}

UINT8	xReadCH376Data( void )  /* 从CH376读数据 */
{
  for ( UINT32 i = 0; i < 500000; i ++ ) {
    if (Serial3.available()) { /* 串口接收到 */
      return (UINT8)Serial3.read();
    }
  }
  
  return ( 0 ); /* 不应该发生的情况 */
}

/* 查询CH376中断(INT#低电平) */
UINT8	Query376Interrupt( void )
{
  #ifdef	CH376_INT_WIRE
  	return( CH376_INT_WIRE ? FALSE : TRUE );  /* 如果连接了CH376的中断引脚则直接查询中断引脚 */
  #else
    // 376机制是怎么样的？
    // 是直接等待返回，还是等待中断来临之后，再去读CH376中断的状态，这样就忽略了回来的数据，这样就需要把这里回来的
    // 数据先清除掉，因为要去读CH376中断的状态，会返回中断状态码的，下午搞清楚在看
    if(Serial3.available()) {
      return( TRUE );
    }
    else return( FALSE );
  #endif
}

UINT8	mInitCH376( void )  /* 初始化CH376 */
{
  UINT8 res;
  CH376_PORT_INIT( );  /* 接口硬件初始化 */
  xWriteCH376Cmd( CMD11_CHECK_EXIST );  /* 测试单片机与CH376之间的通讯接口 */
  xWriteCH376Data( 0x65 );
  res = xReadCH376Data( );
  xEndCH376Cmd( );  // 异步串口方式不需要
  if ( res != 0x9A ) { // 通讯接口不正常,可能原因有:接口连接异常,其它设备影响(片选不唯一),串口波特率,一直在复位,晶振不工作
    CH376DebugOut("USB not exist !");
    return( ERR_USB_UNKNOWN );  
  }
  
  // geo-f 20181017
  xWriteCH376Cmd( CMD_RESET_ALL ); 
  CH376DebugOut("The USB module has been reset !");
  delay(200);
  
#ifdef	UART_WORK_BAUDRATE
    xWriteCH376Cmd( CMD21_SET_BAUDRATE );  /* 设置串口通讯波特率 */
  #if	UART_WORK_BAUDRATE >= 6000000/256
      xWriteCH376Data( 0x03 );
      xWriteCH376Data( 256 - 6000000/UART_WORK_BAUDRATE );
  #else
      xWriteCH376Data( 0x02 );
      xWriteCH376Data( 256 - 750000/UART_WORK_BAUDRATE );
  #endif
    SET_WORK_BAUDRATE( );  /* 将单片机切换到正式通讯波特率 */
    res = xReadCH376Data( );
  //  xEndCH376Cmd( );  // 异步串口方式不需要
    if ( res != CMD_RET_SUCCESS ) return( ERR_USB_UNKNOWN );  /* 通讯波特率切换失败,建议通过硬件复位CH376后重试 */
#endif

  xWriteCH376Cmd( CMD11_SET_USB_MODE );  /* 设备USB工作模式 */
  xWriteCH376Data( 0x05 );
//  CH376Delayus( 20 );  // 异步串口方式不需要
  res = xReadCH376Data( );
//  xEndCH376Cmd( );  // 异步串口方式不需要
  if ( res == CMD_RET_SUCCESS ) return( USB_INT_SUCCESS );
  else {
    CH376DebugOut("USB mode setting wrong !");
    return( ERR_USB_UNKNOWN );  /* 设置模式错误 */
  }
}

UINT8	CH376_init( void )  /* 初始化CH376 */
{
  UINT8     i, s;

	CH376Delayms( 100 );  /* 延时100毫秒 */
	mInitSTDIO( );  /* 为了让计算机通过串口监控演示过程 */
	CH376DebugOut( "Start\n" );

	//s = mInitCH376( );  /* 初始化CH376 */
	//mStopIfError( s );
	return mInitCH376();
}

#endif
#endif

