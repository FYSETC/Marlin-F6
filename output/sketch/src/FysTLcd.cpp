#include "FysTLcd.h"
#include "HardwareSerial.h"
#include "delay.h"
#include "serial.h"

#ifdef FYSTLCD_V1

  bool FysTLcd::test = false; //geo-f
  volatile bool FysTLcd::ftRecvData = false;
  uint8_t FysTLcd::ftDataLen = 0;
  uint16_t FysTLcd::ftAddr = 0;
  uint8_t FysTLcd::ftData[128];


  void FysTLcd::ftBegin() {
    uint16_t baud_setting = (F_CPU / 4 / FYSTLCD_SER_BAUD - 1) / 2;
    FYSTLCD_UCSRxA = 1 << FYSTLCD_U2Xx;
    #if F_CPU == 16000000UL&&FYSTLCD_SER_BAUD==57600
      FYSTLCD_UCSRxA = 0;
      baud_setting = (F_CPU / 8 / FYSTLCD_SER_BAUD - 1) / 2;
    #endif
    FYSTLCD_UBRRxH = baud_setting >> 8;
    FYSTLCD_UBRRxL = baud_setting;
    #if defined(__AVR_ATmega8__)
      FYSTLCD_UCSRxC = 0x86; // select UCSRC register (shared with UBRRH)
    #else
      FYSTLCD_UCSRxC = 0x06;
    #endif
    SBI(FYSTLCD_UCSRxB, FYSTLCD_RXENx);
    SBI(FYSTLCD_UCSRxB, FYSTLCD_TXENx);
    SBI(FYSTLCD_UCSRxB, FYSTLCD_RXCIEx);
    CBI(FYSTLCD_UCSRxB, FYSTLCD_UDRIEx);
  }

  void FysTLcd::ftEnd() {
    CBI(FYSTLCD_UCSRxB, FYSTLCD_RXENx);
    CBI(FYSTLCD_UCSRxB, FYSTLCD_TXENx);
    CBI(FYSTLCD_UCSRxB, FYSTLCD_RXCIEx);
    CBI(FYSTLCD_UCSRxB, FYSTLCD_UDRIEx);
  }

  bool FysTLcd::waitData(uint8_t*dat) {
    uint16_t t=10000;
    while (t && !ftRecvData) {
      t--;
      DELAY_US(10);
    }
    ftRecvData = false;
    if (t) {
      for (t = 0; t<ftDataLen; t++)dat[t] = ftData[t];
      return true;
    }
    return false;
  }

  void FysTLcd::ftRxIrq(u8 dat)
  {
    static bool ifGetData = false;
    static int8_t num = 0, n = 0, step = 0;

    #ifdef FYSTLCD_FYS_CUSTOMIZED
    
      if(dat==FYSTLCD_FRAM_HEAD1&&step == 0)step = 1;
      else if(step==1) {
        if(dat==FYSTLCD_READ_VAR)step=2;
        //else if(dat==FYSTLCD_WRITE_VAR)step=3;// we don't care if send success
        else step=0;
        num=0;
      }
      else if(step==2) {
        ++num;
        switch(num) {
          case 1:
            ftAddr = dat;
            break;
          case 2:
            ftAddr = (ftAddr << 8) | dat;
            break;
          case 3:
            n=dat<<1;
            break;
          default:
            if (n > 0) {
              ftData[num - 4] = dat;
              n--;
              ifGetData = true;
            }
            break;
        }
        if (ifGetData&&n == 0) {
          ftRecvData = true;
          ifGetData = false;
          step = 0;
          ftDataLen = num - 3;
          num = 0;
        }
      }
      else {
        num = 0;
        step = 0;
        n = 0;
      }          

    #else // FYSTLCD_FYS_CUSTOMIZED

      if (dat == FYSTLCD_FRAM_HEAD1&&step == 0)step = 1;
      else if (dat == FYSTLCD_FRAM_HEAD2&&step == 1)step = 2;
      else if (step == 2)step++;
      else if (step == 3) {
        switch (dat){
          case FYSTLCD_READ_REG:step = 5; break;
          case FYSTLCD_READ_VAR:step = 6;  break;
          default:step = 0;  break;
        }
        num = 0;
      }
      else if (step == 5) {// register data    
        num++;
        switch (num) {
          case 0x01:// address
            ftAddr = dat;
            break;
          case 0x02:// data length
            n = dat;
            break;
          default:
            if (n > 0)
            {
                ftData[num - 3] = dat;
                n--;
                ifGetData = true;
            }
            break;
        }
        if (ifGetData&&n == 0) {
          ftRecvData = true;
          ifGetData = false;
          step = 0;
          ftDataLen = num - 2;
          num = 0;
        }
      }
      else if (step == 6) {
        ++num;
        switch (num) {
          case 1:
            ftAddr = dat;
            break;
          case 2:
            ftAddr = (ftAddr << 8) | dat;
            break;
          case 3:
            n = dat << 1;
            break;
          default:
            if (n > 0) {
              ftData[num - 4] = dat;
              n--;
              ifGetData = true;
            }
            break;
        }
        if (ifGetData&&n == 0) {
          ftRecvData = true;
          ifGetData = false;
          step = 0;// one frame data received
          ftDataLen = num - 3;
          num = 0;
        }
      }
      else {
        num = 0;
        step = 0;
      }
    #endif
  }

  ISR(USART2_RX_vect) {
    uint8_t dat = FYSTLCD_UDRx;
    FysTLcd::ftRxIrq(dat);
  }

  void FysTLcd::ftWriteVar(const uint16_t& varAddr, const uint8_t *varData, uint8_t len) {
    if (len == 0)return;

    #if defined(FYSTLCD_FYS_CUSTOMIZED)
      ftWrite(FYSTLCD_FRAM_HEAD1);
      ftWrite(FYSTLCD_WRITE_VAR);
      ftWrite((uint8_t)(varAddr >> 8));
      ftWrite((uint8_t)varAddr);
      ftWrite(len>>1);
    #else
      ftWrite(FYSTLCD_FRAM_HEAD1);
      ftWrite(FYSTLCD_FRAM_HEAD2);
      ftWrite(len + 3);
      ftWrite(FYSTLCD_WRITE_VAR);
      ftWrite((uint8_t)(varAddr >> 8));
      ftWrite((uint8_t)varAddr);

      if(test){
        SERIAL_PRINT(FYSTLCD_FRAM_HEAD1,16); // geo-f:test
        SERIAL_ECHO(' '); // geo-f:test
        delay(2);
        SERIAL_PRINT(FYSTLCD_FRAM_HEAD2,16); // geo-f:test
        SERIAL_ECHO(' '); // geo-f:test
        delay(2);
        SERIAL_PRINT(len + 3,16); // geo-f:test
        SERIAL_ECHO(' '); // geo-f:test
        delay(2);
        SERIAL_PRINT(FYSTLCD_WRITE_VAR,16); // geo-f:test
        SERIAL_ECHO(' '); // geo-f:test
        delay(2);
        SERIAL_PRINT((uint8_t)(varAddr >> 8),16); // geo-f:test
        SERIAL_ECHO(' '); // geo-f:test
        delay(2);
        SERIAL_PRINT((uint8_t)varAddr,16); // geo-f:test
        SERIAL_ECHO(' '); // geo-f:test
        delay(2);
      }
    #endif
    
    while (len-- > 0) {
      if(test){
        SERIAL_PRINT(*varData,16); // geo-f:test
        delay(2);
        SERIAL_ECHO(' '); // geo-f:test
      }
      ftWrite(*varData++);
    }
  }

  void FysTLcd::ftWriteVar(const uint16_t& varAddr, const uint16_t& data) {
    #if defined(FYSTLCD_FYS_CUSTOMIZED)
      ftWrite(FYSTLCD_FRAM_HEAD1);
      ftWrite(FYSTLCD_WRITE_VAR);
      ftWrite((uint8_t)(varAddr >> 8));
      ftWrite((uint8_t)varAddr);
      ftWrite(0x01);
    #else
      ftWrite(FYSTLCD_FRAM_HEAD1);
      ftWrite(FYSTLCD_FRAM_HEAD2);
      ftWrite(0x05);
      ftWrite(FYSTLCD_WRITE_VAR);
      ftWrite((uint8_t)(varAddr >> 8));
      ftWrite((uint8_t)varAddr);
    #endif
    ftWrite((uint8_t)(data >> 8));
    ftWrite((uint8_t)data);
  }

  bool FysTLcd::ftReadVar(const uint16_t& varAddr, uint8_t*tdata, uint8_t len){
    ftRecvData = false;
    #if defined(FYSTLCD_FYS_CUSTOMIZED)
      ftWrite(FYSTLCD_FRAM_HEAD1);
      ftWrite(FYSTLCD_READ_VAR);
      ftWrite(uint8_t(varAddr >> 8));
      ftWrite(uint8_t(varAddr));
    #else
      ftWrite(FYSTLCD_FRAM_HEAD1);
      ftWrite(FYSTLCD_FRAM_HEAD2);
      ftWrite(0x04);
      ftWrite(FYSTLCD_READ_VAR);
      ftWrite(uint8_t(varAddr >> 8));
      ftWrite(uint8_t(varAddr));
    #endif
    ftWrite(len >> 1);
    return waitData(tdata);
  }

  void FysTLcd::ftReadVar(const uint16_t& varAddr, uint8_t len) {
    ftRecvData = false;
    #if defined(FYSTLCD_FYS_CUSTOMIZED)
      ftWrite(FYSTLCD_FRAM_HEAD1);
      ftWrite(FYSTLCD_READ_VAR);
      ftWrite(uint8_t(varAddr >> 8));
      ftWrite(uint8_t(varAddr));
    #else
      ftWrite(FYSTLCD_FRAM_HEAD1);
      ftWrite(FYSTLCD_FRAM_HEAD2);
      ftWrite(0x04);
      ftWrite(FYSTLCD_READ_VAR);
      ftWrite(uint8_t(varAddr >> 8));
      ftWrite(uint8_t(varAddr));
    #endif
    ftWrite(len >> 1);
  }

  void FysTLcd::ftReset() {
    uint8_t cmd[4] = {0x55,0xAA,0x5A,0x5A};
    ftWriteVar(0x0004,cmd,4);
  }

  void FysTLcd::ftSetPage(uint16_t page){
    uint8_t cmd[4] = { 0x5A, 0x01, (page >> 8) & 0xFF, page & 0xFF };//5A A5 07 82 00 84 5A 01 00 01
    ftWriteVar(0x0084, cmd, 4);
  }

  uint16_t FysTLcd::ftGetPage() {
      uint8_t cmd[4] = { 0 };

      if (ftReadVar(0x0014, cmd, 2)) {
        uint16_t t = cmd[0];
        return (t << 8) | cmd[1];
      }
      return 0xFFFF;
  }

  void FysTLcd::ftSetBrightness(uint8_t val) {
    if (val > FYSTLCD_BRIGHTNESS_MAX)val = FYSTLCD_BRIGHTNESS_MAX;
    uint8_t cmd[2] = { val, 0x00 };
    ftWriteVar(0x0082, cmd, 2);
  }

  void FysTLcd::ftPlayMusic(uint8_t id, uint8_t segments, uint8_t volume) { 
    uint8_t cmd[4] = { id, segments, volume, 0x80 };
    ftWriteVar(0x00A0, cmd, 4);
  }

  void FysTLcd::ftDownloadJPEG(uint16_t varAddr,uint16_t x,uint16_t y){
    uint8_t cmd[8] = { 0x5A, 0x01, (varAddr>>8)&0xFF, varAddr&0xFF ,(x>>8)&0xFF,x&0xFF,(y>>8)&0xFF,y&0xFF};
    ftWriteVar(0x00A6, cmd, 8);
  }

  void FysTLcd::ftPuts(const uint16_t& varAddr, const char*str, uint8_t len) {
    if (len == 0)
    while (str[len])len++;
    
    #if defined(FYSTLCD_FYS_CUSTOMIZED)
      len++;
      len >>= 1;
      ftWrite(FYSTLCD_FRAM_HEAD1);
      ftWrite(FYSTLCD_WRITE_VAR);
      ftWrite((uint8_t)(varAddr >> 8));
      ftWrite((uint8_t)varAddr);
      ftWrite(len);
      len <<= 1;
      while (len--) {
        if(*str)ftWrite(*str++);
        else ftWrite(0);
      }
    #else
      ftWrite(FYSTLCD_FRAM_HEAD1);
      ftWrite(FYSTLCD_FRAM_HEAD2);
      ftWrite(len + 3);
      ftWrite(FYSTLCD_WRITE_VAR);
      //SERIAL_ECHO("varAddr:"); // geo-f:test
      //SERIAL_PRINT((uint8_t)(varAddr >> 8),16); // geo-f:test
      //SERIAL_PRINTLN((uint8_t)varAddr,16); // geo-f:test
      ftWrite((uint8_t)(varAddr >> 8));
      ftWrite((uint8_t)varAddr);
      while (len-- > 0) {
        if(*str) {
          //SERIAL_ECHO(*str); // geo-f:test
          ftWrite(*str++);
          
        }
        else {
          //SERIAL_ECHO("x");
          ftWrite(0);
        }
      }
    #endif
  }

  void FysTLcd::ftPutsPGM(const uint16_t& varAddr, const char*str, uint8_t len) {
    const char*t = str;
    char tlen = 0, ch;
    while (pgm_read_byte(t))
    {
        tlen++;
        t++;
    }
    if (len == 0)len = tlen;

    #if defined(FYSTLCD_FYS_CUSTOMIZED)
      len++;
      len >>= 1;
      ftWrite(FYSTLCD_FRAM_HEAD1);
      ftWrite(FYSTLCD_WRITE_VAR);
      ftWrite((uint8_t)(varAddr >> 8));
      ftWrite((uint8_t)varAddr);
      ftWrite(len);
      len <<= 1;
      ch = pgm_read_byte(str);
      while (len-- > 0) {
        if (ch) {
          ftWrite(ch);
          str++;
          ch = pgm_read_byte(str);
        }
        else ftWrite(0);
      }
    #else
      ftWrite(FYSTLCD_FRAM_HEAD1);
      ftWrite(FYSTLCD_FRAM_HEAD2);
      ftWrite(len + 3);
      ftWrite(FYSTLCD_WRITE_VAR);
      ftWrite((uint8_t)(varAddr >> 8));
      ftWrite((uint8_t)varAddr);
      ch = pgm_read_byte(str);
      while (len-- > 0) {
        if (ch) {
          ftWrite(ch);
          str++;
          ch = pgm_read_byte(str);
        }
        else ftWrite(0);
      }
    #endif
  }

  void FysTLcd::ftPutTime(millis_t& pt) {
    uint8_t t;
    t = pt / 3600;
    if (t >= 100) {
      cdt[n++] = t / 100 + '0';
      t %= 100;
    }
    if (t >= 10) {
      cdt[n++] = t / 10 + '0';
      t %= 10;
    }
    cdt[n++] = t + '0';
    cdt[n++] = ':';
    pt %= 3600;
    t = pt / 60;
    if (t >= 10) {
      cdt[n++] = t / 10 + '0';
      t %= 10;
    }
    cdt[n++] = t + '0';
    cdt[n++] = ':';
    t = pt % 60;
    if (t >= 10) {
      cdt[n++] = t / 10 + '0';
      t %= 10;
    }
    cdt[n++] = t + '0';
    for (; n < 8; n++)cdt[n] = ' ';
  }

  void FysTLcd::ftPutPGM(const char* str){
    while (char ch = pgm_read_byte(str++)) cdt[n++] = ch;
    for (; n < 8; n++)cdt[n] = ' ';
  }

  void FysTLcd::ftSendCmdFromString(const char*tar) {
    char c, ch, tt = 0;
    for (; (*tar >= 'A'&&*tar <= 'F') \
            || (*tar >= 'a'&&*tar <= 'f') \
            || (*tar >= '0'&&*tar <= '9') \
            || *tar == ' '; tar++) {
      MYSERIAL0.write(*tar);
      if (*tar == ' ')continue;
      if (*tar >= 'A'&&*tar <= 'F')
        ch = *tar - 'A' + 10;
      else if (*tar >= 'a'&&*tar <= 'f')
        ch = *tar - 'a' + 10;
      else
        ch = *tar - '0';
      if (tt == 0) {
        tt = 1;
        c = ch << 4;
      }
      else
      if (tt == 1) {
        tt = 2;
        c |= ch;
      }
      if (tt == 2) {
        tt = 0;
        ftWrite(c);
      }
    }
  }

  const char* FysTLcd::ftCmdString(uint8_t& stringLen) {
    if (ftDataLen<4) {
      stringLen = 0;
      return nullptr;
    }
    else {
      stringLen = ftDataLen - 2;
      if (ftData[stringLen - 1] == 0xFF)stringLen--;
      return (const char*)ftData;
    }
  }

#endif //FYSTLCD_V1

