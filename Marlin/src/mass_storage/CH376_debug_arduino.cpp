/* 调试 */
/* 提供printf子程序 */

#include	"CH376_debug.h"

// 其他Arduino芯片暂时不管
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

#ifdef FYS_STORAGE_SUPPORT

#include "../serial.h"

/* 检查操作状态,如果错误则显示错误代码并停机,应该替换为实际的处理措施,例如显示错误信息,等待用户确认后重试等 */
void	mStopIfError( UINT8 iError )
{
	if ( iError == USB_INT_SUCCESS ) return;  /* 操作成功 */
  //Serial.print( "Error:0x" ); /* 显示错误 */
  //Serial.println( (UINT16)iError,16);
  SERIAL_ECHO("USB Error:0x");
  SERIAL_PRINTLN((UINT16)iError,16);

  #ifdef CH376_DEBUG
  	while ( 1 ) {
      /*LED_OUT_ACT( );*/  /* LED闪烁 */
  		CH376Delayms( 200 );
      /*LED_OUT_INACT( );*/
  		CH376Delayms( 200 );
  	}
  #else
    // 硬件复位之后需要100 ms 的延时
    // geo-f:comment 20190228
    //WRITE(CH376_RESET_PIN,HIGH);
    //CH376Delayms(10);
    //WRITE(CH376_RESET_PIN,LOW);
    //CH376Delayms(100);
	#endif		
}

/* 为printf和getkey输入输出初始化串口 */
void	mInitSTDIO( void )
{
//  Serial.begin(115200); 
}

void CH376DebugOut(const char* inf) {
  //Serial.println( inf ); /* 显示错误 */
  #ifdef CH376_DEBUG
    SERIAL_ECHOLN(inf);
  #endif
}

void CH376DebugOutErr(UINT16 err) {
  #ifdef CH376_DEBUG
    SERIAL_ECHO("Error:0x");
    SERIAL_PRINTLN(err,16);
  #endif
}

void CH376DebugOutPair(const char *s,UINT16 err) {
  #ifdef CH376_DEBUG    
    SERIAL_ECHO(s);
    SERIAL_PRINTLN(err,16);
  #endif
}



#endif
#endif
