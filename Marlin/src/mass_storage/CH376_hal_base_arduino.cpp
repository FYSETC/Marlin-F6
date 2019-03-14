/* CH376芯片 硬件抽象层 V1.0 */
/* 提供基本子程序 */

#include	"CH376_hal.h"

// 其他Arduino芯片暂时不管
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

//#include "delay.h"

// 延时指定微秒时间
//#define CH376Delayus(us) DELAY_US(us) // 这样做需要在调用的文件中 #include "delay.h"
void	CH376Delayus( UINT8 us )
{
	//DELAY_US(us);
	//while ( us -- );  /* 24MHz MCS51 */
	while ( us -- ) {   // ATMEGA 2560 16M
    _NOP();_NOP();_NOP();_NOP();
    _NOP();_NOP();_NOP();_NOP();
    _NOP();_NOP();_NOP();_NOP();
    _NOP();_NOP();_NOP();_NOP();
	}
}


// 延时指定毫秒时间
void	CH376Delayms( UINT8 ms )
{
	delay(ms);
}

#endif

