#ifndef _FYSTLCD_H_
#define _FYSTLCD_H_

  #ifndef HardwareSerial_h
    #define HardwareSerial_h //trick standard serial
  #endif

  #include "Marlin.h"

  #ifdef FYSTLCD_V1
  
    typedef enum em_popup_page_type{
      EPPT_INFO_POPUP, 		
    	EPPT_INFO_WAITING,
    	EPPT_INFO_OPTION
    } EM_POPUP_PAGE_TYPE;

    //#define       FYSTLCD_FYS_CUSTOMIZED 

    /*--------------------FYSTLCD protocal--------------------*/

    #define FYSTLCD_WRITE_REG               0x80 
    #define FYSTLCD_READ_REG                0x81

    #ifdef FYSTLCD_FYS_CUSTOMIZED
      #define FYSTLCD_FRAM_HEAD1              0xCA
      #define FYSTLCD_WRITE_VAR               0xA1 
      #define FYSTLCD_READ_VAR                0xA2
    #else
      #define FYSTLCD_FRAM_HEAD1              0x5A
      #define FYSTLCD_FRAM_HEAD2              0xA5
      #define FYSTLCD_WRITE_VAR               0x82 
      #define FYSTLCD_READ_VAR                0x83   
    #endif

    /*--------------------serial define--------------------*/
    #define _TNAME(X,Y,Z)                   X##Y##Z
    #define TNAME(X,Y,Z)                    _TNAME(X,Y,Z)
    #define FYSTLCD_SERIAL_RX_VECT          TNAME(USART,FYSTLCD_SER_PORT,_RX_vect)
    #define FYSTLCD_UCSRxA                  TNAME(UCSR,FYSTLCD_SER_PORT,A)
    #define FYSTLCD_UCSRxB                  TNAME(UCSR,FYSTLCD_SER_PORT,B)
    #define FYSTLCD_UCSRxC                  TNAME(UCSR,FYSTLCD_SER_PORT,C)
    #define FYSTLCD_UBRRxH                  TNAME(UBRR,FYSTLCD_SER_PORT,H)
    #define FYSTLCD_UBRRxL                  TNAME(UBRR,FYSTLCD_SER_PORT,L)
    #define FYSTLCD_UDRx                    TNAME(UDR,FYSTLCD_SER_PORT,)
    #define FYSTLCD_U2Xx                    TNAME(U2X,FYSTLCD_SER_PORT,)
    #define FYSTLCD_RXENx                   TNAME(RXEN,FYSTLCD_SER_PORT,)
    #define FYSTLCD_TXENx                   TNAME(TXEN,FYSTLCD_SER_PORT,)
    #define FYSTLCD_TXCx                    TNAME(TXC,FYSTLCD_SER_PORT,)
    #define FYSTLCD_RXCIEx                  TNAME(RXCIE,FYSTLCD_SER_PORT,)
    #define FYSTLCD_UDRIEx                  TNAME(UDRIE,FYSTLCD_SER_PORT,)
    #define FYSTLCD_UDREx                   TNAME(UDRE,FYSTLCD_SER_PORT,)

    
    #define FYSTLCD_SER_BAUD                115200
    #define FYSTLCD_SER_PORT                2
    #define FYSTLCD_FILENAME_LEN            32      // filename length
    #define FYSTLCD_IMAGE_SIZE              250 
    #define FYSTLCD_DOT_TEN_MUL             10.0    // 1 decimal
    #define FYSTLCD_DOT_HUND_MUL            100.0   // 2 decimal
    #define FYSTLCD_BRIGHTNESS_MAX          0x64    //0x64 T5==64 old==40


    class FysTLcd
    {
    private:
        volatile static bool ftRecvData;
        static uint8_t ftDataLen;
        static uint8_t ftData[128];
        static inline void ftWrite(uint8_t dat)
        {
            while (!(FYSTLCD_UCSRxA&(1 << FYSTLCD_UDREx)));
            FYSTLCD_UDRx = dat;
        }
        static bool waitData(uint8_t*dat);
        static void ftWriteVar(const uint16_t& varAddr, const uint16_t& data);
        static void ftWriteVar(const uint16_t& varAddr, const uint8_t *data, uint8_t len = 2);
        static void ftReadVar(const uint16_t& varAddr, uint8_t len = 2);
        static bool ftReadVar(const uint16_t& varAddr, uint8_t*data, uint8_t len = 2);
        
    public:
        inline void ftWrite1(uint8_t dat) {
            while (!(FYSTLCD_UCSRxA&(1 << FYSTLCD_UDREx)));
            FYSTLCD_UDRx = dat;
        }
        
        static bool test; //geo-f
        static uint16_t ftAddr;
        static void ftBegin();
        static void ftEnd();
        static bool ftAvailableCmd(){ bool ifCmd = ftRecvData; ftRecvData = false; return ifCmd; }
        static inline uint16_t ftCmdVal16(){ uint16_t val = ftData[0]; val = (val << 8) | ftData[1]; return val; }
        static inline uint32_t ftCmdVal32(){ uint32_t val = ftData[0]; val = (val << 8) | ftData[1]; val = (val << 8) | ftData[2]; val = (val << 8) | ftData[3]; return val; }
        static const char* ftCmdString(uint8_t& stringLen);
        static void ftRxIrq(u8 dat);

        static void ftReset();
        static void ftSetBrightness(uint8_t val);
        static void ftSetPage(uint16_t page);
        static void ftPlayMusic(uint8_t id, uint8_t segments, uint8_t volume);
        static void ftDownloadJPEG(uint16_t varAddr,uint16_t x,uint16_t y);
        static void ftPuts(const uint16_t& ftAddr, const char*str, uint8_t len = 0);
        static void ftPutsPGM(const uint16_t& ftAddr, const char*str, uint8_t len = 0);
        static uint16_t ftGetPage();
        static void ftSendCmdFromString(const char*tar); //调试用

    private:
        uint8_t n;
        uint8_t cdt[39];
        uint16_t cAddr;
        inline void ftPauseReceive(){ CBI(FYSTLCD_UCSRxB, FYSTLCD_RXCIEx); }
        inline void ftResumeReceive(){ SBI(FYSTLCD_UCSRxB, FYSTLCD_RXCIEx); }

    public:
        inline void ftCmdStart(const uint16_t& ftAddr) {
          cAddr = ftAddr;
          memset(cdt, 0, 39);
          n = 0;
        }
        inline void ftCmdSend(uint8_t jump=0) {
          if(test){
            SERIAL_ECHOLNPAIR("n:",n);
            SERIAL_ECHOLNPAIR("cAddr:",cAddr);
            SERIAL_ECHOLN("ftWriteVar:");
          }
          
          ftWriteVar(cAddr, cdt, n);
          memset(cdt, 0, n);
          if (jump)cAddr += jump;
          else cAddr += (n >> 1);
          n = 0;
        }
        
        inline bool ftCmdReceive(uint8_t num) {
          bool ifGet=ftReadVar(cAddr, cdt, num);
          cAddr += (num >> 1);
          n = 0;
          return ifGet;
        }
        
        inline void ftCmdClear(){ 
          memset(cdt, 0, 39);
        }
        
        inline void ftCmdJump(uint8_t byteNum) { n += byteNum; }
        
        inline void ftCmdPut8(const uint8_t& r) {
          cdt[n++] = (uint8_t)r;
        }
        
        inline void ftCmdPut16(const int16_t& r) {
          cdt[n++] = (r >> 8) & 0xFF;
          cdt[n++] = (uint8_t)r;
        }
        
        inline void ftCmdPutF16(const float& r) {
          uint16_t t = r* FYSTLCD_DOT_TEN_MUL;
          cdt[n++] = (t >> 8) & 0xFF;
          cdt[n++] = (uint8_t)t;
        }
        
        inline void ftCmdPutF16_2(const float& r) {
          uint16_t t = r* FYSTLCD_DOT_HUND_MUL;
          cdt[n++] = (t >> 8) & 0xFF;
          cdt[n++] = (uint8_t)t;
        }
        inline void ftCmdPutI32(const int32_t& r) {
          cdt[n++] = (r >> 24) & 0xFF;
          cdt[n++] = (r >> 16) & 0xFF;
          cdt[n++] = (r >> 8) & 0xFF;
          cdt[n++] = (uint8_t)r;
        }
        
        inline void ftCmdPutF32(const float& r) {
          int32_t t = r*FYSTLCD_DOT_TEN_MUL;
          ftCmdPutI32(t);
        }

        inline void ftCmdGetI16(int16_t& r) {
          r = cdt[n++];
          r = ((r << 8) | cdt[n++]);
        }
        
        inline void ftCmdGetF16(float& r) {
          int16_t t;
          ftCmdGetI16(t);
          r = t / FYSTLCD_DOT_TEN_MUL;
        }
        
        inline void ftCmdGetI32(int32_t& r) {
          r = cdt[n++];
          r = ((r << 8) | cdt[n++]);
          r = ((r << 8) | cdt[n++]);
          r = ((r << 8) | cdt[n++]);
        }
        
        inline void ftCmdGetF32(float& r) {
          int32_t t;
          ftCmdGetI32(t);
          r = t / FYSTLCD_DOT_TEN_MUL;
        }
        
        void ftPutTime(millis_t& pt);
        void ftPutPGM(const char* str);

    public:
        FysTLcd():n(0){}
        ~FysTLcd(){ }
    };
  #endif //FYSTLCD_V1

#endif //FYSTLCD_H
