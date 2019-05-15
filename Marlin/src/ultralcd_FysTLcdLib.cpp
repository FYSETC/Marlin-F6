#include "temperature.h"
#include "ultralcd.h"
//#include "utility.h"
#ifdef FYSTLCD_V1
#include "FysTLcdAddrLayout.h"
#include "Marlin.h"
#include "language.h"
#include "sd/cardreader.h"
#include "mass_storage/cardusbdiskreader.h"
#include "temperature.h"
#include "stepper.h"
#include "configuration_store.h"
#include "parser.h"
#include <avr/wdt.h>

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "power_loss_recovery.h"
#endif

#if DISABLED(PRINTCOUNTER)
  #include "stopwatch.h"
#else // PRINTCOUNTER
  #include "printcounter.h"
  #include "duration_t.h"
#endif

static uint8_t dynamicIcon = 0;

typedef void(*generalVoidFun)();

static uint8_t ftState = 0x00;
#define FTSTATE_LCD_OLD_CARDSTATUS  0x01    //sdcard status
#define FTSTATE_NEED_SAVE_PARAM     0x02    // need to save paras to eeprom
#define FTSTATE_UPDATE_NOW          0x04    // update immediatelly
#define FTSTATE_SERVO_STATUS        0x08    // servo status
#define FTSTATE_AUTOPID_ING         0x10    // auto pid
#define FTSTATE_ACTIVE_WARNING      0x20    //

#define TEMPERTURE_PREHEAT_CHOISE_E1_PLA    0
#define TEMPERTURE_PREHEAT_CHOISE_E1_ABS    1
#define TEMPERTURE_PREHEAT_CHOISE_E1_PVA    2
#define TEMPERTURE_PREHEAT_CHOISE_E1_FLEX   3
#define TEMPERTURE_PREHEAT_CHOISE_E1_PET    4
#define TEMPERTURE_PREHEAT_CHOISE_E1_HIPS   5
#define TEMPERTURE_PREHEAT_CHOISE_E1_PP     6
#define TEMPERTURE_PREHEAT_CHOISE_E1_CUSTOM 7
#define TEMPERTURE_PREHEAT_CHOISE_E2_PLA    8
#define TEMPERTURE_PREHEAT_CHOISE_E2_ABS    9
#define TEMPERTURE_PREHEAT_CHOISE_E2_PVA    10
#define TEMPERTURE_PREHEAT_CHOISE_E2_FLEX   11
#define TEMPERTURE_PREHEAT_CHOISE_E2_PET    12
#define TEMPERTURE_PREHEAT_CHOISE_E2_HIPS   13
#define TEMPERTURE_PREHEAT_CHOISE_E2_PP     14
#define TEMPERTURE_PREHEAT_CHOISE_E2_CUSTOM 15
#define TEMPERTURE_PREHEAT_CHOISE_BED_PLA    16
#define TEMPERTURE_PREHEAT_CHOISE_BED_ABS    17
#define TEMPERTURE_PREHEAT_CHOISE_BED_PVA    18
#define TEMPERTURE_PREHEAT_CHOISE_BED_FLEX   19
#define TEMPERTURE_PREHEAT_CHOISE_BED_PET    20
#define TEMPERTURE_PREHEAT_CHOISE_BED_HIPS   21
#define TEMPERTURE_PREHEAT_CHOISE_BED_PP     22
#define TEMPERTURE_PREHEAT_CHOISE_BED_CUSTOM 23

char tempChoice = 0;
char optionId = 0;
uint16_t currentPageId = 0xFFFF, retPageId = 0xFFFF;
uint16_t dwinFileWindowTopIndex = 0;

int16_t lcd_preheat_hotend_temp[FILAMENT_OPE_CHOICES] = { PREHEAT_1_TEMP_HOTEND, \
                                                          PREHEAT_2_TEMP_HOTEND, \
                                                          PREHEAT_3_TEMP_HOTEND, \
                                                          PREHEAT_4_TEMP_HOTEND, \
                                                          PREHEAT_5_TEMP_HOTEND, \
                                                          PREHEAT_6_TEMP_HOTEND, \
                                                          PREHEAT_7_TEMP_HOTEND, \
                                                          PREHEAT_8_TEMP_HOTEND, \
                                                          };
int16_t lcd_preheat_bed_temp[FILAMENT_OPE_CHOICES] = {  PREHEAT_1_TEMP_BED, \
                                                        PREHEAT_2_TEMP_BED, \
                                                        PREHEAT_3_TEMP_BED, \
                                                        PREHEAT_4_TEMP_BED, \ 
                                                        PREHEAT_5_TEMP_BED, \ 
                                                        PREHEAT_6_TEMP_BED, \ 
                                                        PREHEAT_7_TEMP_BED, \ 
                                                        PREHEAT_8_TEMP_BED, \ 
                                                        };
int16_t lcd_preheat_fan_speed[FILAMENT_OPE_CHOICES] = { PREHEAT_1_FAN_SPEED, \
                                                        PREHEAT_2_FAN_SPEED, \
                                                        PREHEAT_3_FAN_SPEED, \
                                                        PREHEAT_4_FAN_SPEED, \
                                                        PREHEAT_5_FAN_SPEED, \
                                                        PREHEAT_6_FAN_SPEED, \
                                                        PREHEAT_7_FAN_SPEED, \
                                                        PREHEAT_8_FAN_SPEED, \
                                                        };
float movDis, movFeedrate;
generalVoidFun periodFun = nullptr;
static FysTLcd myFysTLcd;

#define MOVE_E_LENGTH_EACH_TIME 2.0
#define MOVE_E_FEEDRATE 3.0
#define MOVE_XYZ_FEEDRATE 50.0

#if (ENABLED(SDSUPPORT) || ENABLED(FYS_STORAGE_SUPPORT)) && PIN_EXISTS(SD_DETECT)
  //bool sd_status;
  uint8_t lcd_sd_status;
#endif

static int16_t oldFanSpeed = 0;
static uint16_t oldFeedratePercentage = 0;

static void sendParam_Tune();
static void readParam_Tune();
static void sendParam_Tune_fan();
static void readParam_Tune_fan();
static void sendParam_Motor();
static void readParam_Motor();
static void sendActiveExtrudersParam();
static void readActiveExtrudersParam();
#if HOTENDS > 1
  static void sendParam_ExtrudersOffset();
  static void readParam_ExtrudersOffset();
  static void sendParam_ExtrudersTemp();
  static void readParam_ExtrudersTemp();
  static void sendParam_ExtrudersMotor();
  static void readParam_ExtrudersMotor();
#endif
static void sendParam_Leveling();
static void readParam_Leveling();
static void sendParam_Temp();
static void readParam_Temp();
static void sendParam_Material();
static void readParam_Material();
static void sendParam_TMC2130();
static void readParam_TMC2130();
static void sendParam_system();
static void readParam_system();
static void lcd_period_report(int16_t s);
static void lcd_period_prompt_report();
static void lcd_period_task(millis_t& tNow);
static void lcd_event();
static void lcd_check();
static void dwin_on_cmd(millis_t& tNow);
static void lcd_init_datas();

void lcd_init() {
  #if defined (SDSUPPORT) && PIN_EXISTS(SD_DETECT)
    pinMode(SD_DETECT_PIN, INPUT);
    WRITE(SD_DETECT_PIN, HIGH);
  #endif
  
  FysTLcd::ftBegin();
  
  lcd_startup_music();

  lcd_set_page(0);

  /*
  #if defined (SDSUPPORT) && PIN_EXISTS(SD_DETECT)
    if (IS_SD_INSERTED)
    {
        //ftState |= FTSTATE_LCD_OLD_CARDSTATUS;
        myFysTLcd.ftCmdStart(VARADDR_STATUS_SD);
        myFysTLcd.ftCmdPut16(1);
        myFysTLcd.ftCmdSend();
    }
    else
    {
        myFysTLcd.ftCmdStart(VARADDR_STATUS_SD);
        myFysTLcd.ftCmdPut16(0);
        myFysTLcd.ftCmdSend();
    }
    dwinFileWindowTopIndex = 0;
  #endif
  */
    
  #if FYSTLCD_PAGE_EXIST(MAIN)
    retPageId = FTPAGE(MAIN);
    currentPageId = FTPAGE(MAIN);
  #endif

  /*
  #if ENABLED(SDSUPPORT) && PIN_EXISTS(SD_DETECT)
    sd_status = IS_SD_INSERTED;
  #endif // SDSUPPORT && SD_DETECT_PIN
  */

  #if ENABLED(SDSUPPORT) && PIN_EXISTS(SD_DETECT)
    SET_INPUT_PULLUP(SD_DETECT_PIN);
    lcd_sd_status = 2; // UNKNOWN
    dwinFileWindowTopIndex = 0;
  #endif

  lcd_init_datas();// so NO zero data in screen
}

static void lcd_init_datas() {
  movDis = 0.1;
  movFeedrate = 30.0;

  int16_t tval = movDis*FYSTLCD_DOT_TEN_MUL;
  myFysTLcd.ftCmdStart(VARADDR_MOVE_DIS_SIGN);
  myFysTLcd.ftCmdPut16(tval);
  myFysTLcd.ftCmdSend();
  tval = movFeedrate*FYSTLCD_DOT_TEN_MUL;
  myFysTLcd.ftCmdStart(VARADDR_MOVE_SPEED_SIGN);
  myFysTLcd.ftCmdPut16(tval);
  myFysTLcd.ftCmdSend();

  myLcdEvt |= ((uint16_t)0x0001 << LCDEVT_DETAIL_EXTRUDER);
  
  FysTLcd::ftPuts(VARADDR_VERSION_DATE, SHORT_BUILD_VERSION, ATTACH_STR_LEN);
  FysTLcd::ftPuts(VARADDR_SERIAL_NUMBER, SERIAL_NUMBER, ATTACH_STR_LEN);

  #if ENABLED(PRINTCOUNTER)
    printStatistics state = print_job_timer.getStats();
    myFysTLcd.ftCmdStart(VARADDR_INFO_PRINT);
    myFysTLcd.ftCmdPut16(state.totalPrints);
    myFysTLcd.ftCmdSend();
    char buffer[21];
    duration_t elapsed = state.printTime;
    elapsed.toString(buffer);
    FysTLcd::ftPuts(VARADDR_INFO_PRINT_ACC_TIME, buffer, ATTACH_STR_LEN);
  #endif
  
  millis_t pt=0;
  myFysTLcd.ftCmdStart(VARADDR_PRINT_TIME);
  myFysTLcd.ftPutTime(pt);
  myFysTLcd.ftCmdSend();
}

#define START_UP_END 99

static void lcd_start(millis_t& tNow)  {
  static millis_t period = 30; // 8ms
  static uint16_t pic_num = 0;

  if(myLcdEvt & ((uint16_t)0x0001 << LCDEVT_IF_CONTINE_PRINT)) {
    pic_num=START_UP_END+2;
    return;
  }

  if(pic_num>START_UP_END+1)  {
    return;
  }
  
  if (tNow > period) {
    if(pic_num>START_UP_END)  {
      #if FYSTLCD_PAGE_EXIST(MAIN)
        lcd_set_page_force(FTPAGE(MAIN));
      #endif  
      pic_num++;
      return;
    }
    
    myFysTLcd.ftCmdStart(VARADDR_START_UP);   
    myFysTLcd.ftCmdPut16(pic_num);
    myFysTLcd.ftCmdSend();
    period = tNow + 30;
    pic_num++;
  }
}

void lcd_update()
{
    millis_t t = millis();
    lcd_event();
    lcd_check();
    dwin_on_cmd(t);
    lcd_period_task(t);    
    
    #if ENABLED(POWER_LOSS_RECOVERY)
    if (job_recovery_commands_count && job_recovery_phase == JOB_RECOVERY_IDLE) {
      //lcd_goto_screen(lcd_job_recovery_menu);
      job_recovery_phase = JOB_RECOVERY_MAYBE; // Waiting for a response
      myLcdEvt |= ((uint16_t)0x0001 << LCDEVT_IF_CONTINE_PRINT);
    }
    #endif

    // deal with startup pages
    lcd_start(t);    
}

static void lcd_event() {
  if (myLcdEvt == 0x0000) return;
  uint8_t n;
  char sdFileName[FYSTLCD_FILENAME_LEN],*t;
  for (n = 0; n < 16; n++){
    if (myLcdEvt&(1 << n)){
      myLcdEvt &= ~(uint16_t)(1 << n);
      break;
    }
  }
  switch (n) {
    case LCDEVT_IF_CONTINE_PRINT:
      #if FYSTLCD_PAGE_EXIST(POWER_LOSS_RECOVERY)
        lcd_set_page(FTPAGE(POWER_LOSS_RECOVERY));
      #endif    
      break;
    
    case LCDEVT_READY_CONTINE_PRINT:
      #if ENABLED(SDSUPPORT) || ENABLED(FYS_STORAGE_SUPPORT)
        if (card.longFilename[0]) strncpy(sdFileName, card.longFilename, FYSTLCD_FILENAME_LEN);
        else strncpy(sdFileName, card.filename, FYSTLCD_FILENAME_LEN);
        t = strchr(sdFileName, '.');
        while (*t)*t++ = '\0';
        FysTLcd::ftPuts(VARADDR_PRINTFILE_NAME, sdFileName, FYSTLCD_FILENAME_LEN);
      #endif
      break;
    
    case LCDEVT_AUTOPID_STOP:
      ftState &= ~FTSTATE_AUTOPID_ING;
      
    case LCDEVT_DETAIL_EXTRUDER:
      ftState |= FTSTATE_UPDATE_NOW;
      sendActiveExtrudersParam();
      break;
    
    case LCDEVT_LEVELING_COMPLETE:          
      #if FYSTLCD_PAGE_EXIST(AUTO_LEVELING_COMPLETE)&&!FYSTLCD_PAGE_EXIST(AUTO_LEVELING)
      	sendParam_Leveling();
        if( !IS_SD_PRINTING && !print_job_timer.isRunning() ) 
        { 
           lcd_set_page(FTPAGE(MAIN));
        }
      #endif
			
      disable_X(); 
      disable_Y();  
      break;
    
    case LCDEVT_PRINTING_COMPLETE:
      #if FYSTLCD_PAGE_EXIST(ACCOMPLISH_PRINT)
        lcd_set_page(FTPAGE(ACCOMPLISH_PRINT));
      #elif FYSTLCD_PAGE_EXIST(MAIN)
        lcd_set_page(FTPAGE(MAIN));
      #endif   
      break;
  }
}

static void lcd_check() {   

  #if defined(SDSUPPORT) && PIN_EXISTS(SD_DETECT)
    /*
    const bool sd_statusNow = IS_SD_INSERTED;
    if (sd_statusNow != sd_status) {
        if (!sd_statusNow) {
          //ftState |= FTSTATE_LCD_OLD_CARDSTATUS;
          SERIAL_ECHOPGM("SD card inserted.");
          card.initsd();
          if(card.cardOK) {
              myFysTLcd.ftCmdStart(VARADDR_STATUS_SD);
              myFysTLcd.ftCmdPut16(1);
              myFysTLcd.ftCmdSend();
          }
        }
        else {
          bool changePage = false
            #if FYSTLCD_PAGE_EXIST(FILELIST)
              ||currentPageId == PAGENUM_FILELIST
            #endif
            #if FYSTLCD_PAGE_EXIST(ACCOMPLISH_PRINT)
              ||currentPageId==PAGENUM_ACCOMPLISH_PRINT
            #endif
              ;
          if (changePage) {
            #if FYSTLCD_PAGE_EXIST(MAIN)
              lcd_set_page(FTPAGE(MAIN));
            #endif
          }
          card.release();
          //ftState &= ~FTSTATE_LCD_OLD_CARDSTATUS;
          SERIAL_ECHOPGM("SD card removed.");
          myFysTLcd.ftCmdStart(VARADDR_STATUS_SD);
          myFysTLcd.ftCmdJump(2);
          myFysTLcd.ftCmdSend();
        }
        
        sd_status = sd_statusNow;
        dwinFileWindowTopIndex = 0;
    }*/

    const uint8_t sd_status = IS_SD_INSERTED;
    if (sd_status != lcd_sd_status) {

      uint8_t old_sd_status = lcd_sd_status; // prevent re-entry to this block!
      lcd_sd_status = sd_status;

      if (sd_status) {
        safe_delay(500); // Some boards need a delay to get settled
        card.initsd();
        if (old_sd_status == 2) {
          card.beginautostart();  // Initial boot
        }
        else {
          //set_status_P(PSTR(MSG_SD_INSERTED));
          if(card.cardOK) {
            myFysTLcd.ftCmdStart(VARADDR_STATUS_SD);
            myFysTLcd.ftCmdPut16(1);
            myFysTLcd.ftCmdSend();
          }
        }
      }
      else {
        card.release();
        if (old_sd_status != 2) {
          //set_status_P(PSTR(MSG_SD_REMOVED));
          myFysTLcd.ftCmdStart(VARADDR_STATUS_SD);
          myFysTLcd.ftCmdPut16(0);
          myFysTLcd.ftCmdSend();
          //if (!on_status_screen()) return_to_status();
          bool changePage = false
            #if FYSTLCD_PAGE_EXIST(FILELIST)
              ||currentPageId == PAGENUM_FILELIST
            #endif
            #if FYSTLCD_PAGE_EXIST(ACCOMPLISH_PRINT)
              ||currentPageId==PAGENUM_ACCOMPLISH_PRINT
            #endif
              ;
          if (changePage) {
            #if FYSTLCD_PAGE_EXIST(MAIN)
              lcd_set_page(FTPAGE(MAIN));
            #endif
          }
        }
      }

      //refresh();
      //init_lcd(); // May revive the LCD if static electricity killed it
      dwinFileWindowTopIndex = 0;
    }
  #endif

  #if FAN_COUNT > 0
    if (oldFanSpeed != fanSpeeds[active_extruder]) {
      oldFanSpeed=fanSpeeds[active_extruder];
      myFysTLcd.ftCmdStart(VARADDR_TUNE_FAN_SPEED);
      myFysTLcd.ftCmdPut16(oldFanSpeed);
      myFysTLcd.ftCmdSend();
    }
  #endif
  
  if (oldFeedratePercentage != feedrate_percentage) {
    oldFeedratePercentage = feedrate_percentage;
    myFysTLcd.ftCmdStart(VARADDR_TUNE_PRINT_PERCENTAGE);
    myFysTLcd.ftCmdPut16(oldFeedratePercentage);
    myFysTLcd.ftCmdSend();
  }
}

static void lcd_period_task(millis_t& tNow)  {
  static millis_t period = 1000;
  
  if ((ftState&FTSTATE_NEED_SAVE_PARAM) \
      && (commands_in_queue < BUFSIZE)) {
    enqueue_and_echo_commands_P(PSTR("M500"));
    ftState &= ~FTSTATE_NEED_SAVE_PARAM;
  }
  
  if (tNow > period || (ftState&FTSTATE_UPDATE_NOW)) {
    if (periodFun) periodFun();
    
    millis_t distance = 0;
    
	  #if defined(FYS_ACTIVE_TIME_OVER)
      if (tNow<previous_move_ms) {
        previous_move_ms = tNow;
        distance = max_inactive_time;
      }
      else {
        distance = tNow - previous_move_ms;
        if (distance > max_inactive_time)distance = 0;
        else distance = max_inactive_time - distance;
      }
      
      if (distance<21000) {
        if (!(ftState&FTSTATE_ACTIVE_WARNING)) {
          ftState |= FTSTATE_ACTIVE_WARNING;
          dwin_popup_shutdown();
        }
      }
      else if (ftState&FTSTATE_ACTIVE_WARNING) {
        ftState &= ~FTSTATE_ACTIVE_WARNING;
        dwin_popup_shutdown();
      }
	  #endif
	
    lcd_period_report(distance / 1000);
    period = tNow + 1000;
    ftState &= ~FTSTATE_UPDATE_NOW;
  }

  #if defined(VARADDR_PROMPT_DATA)&&VARADDR_PROMPT_DATA>0
    static millis_t prompt = 200;
    if (tNow > prompt) {
      lcd_period_prompt_report();
      prompt = tNow + 200;
    }
  #endif
}
static void lcd_save() {
  if (commands_in_queue < BUFSIZE) {
    enqueue_and_echo_commands_P(PSTR("M500"));
    ftState &= ~FTSTATE_NEED_SAVE_PARAM;
  }
  else 
    ftState |= FTSTATE_NEED_SAVE_PARAM;
}

static inline void dwin_pid_auto_tune() {
  char str[30];
  readActiveExtrudersParam();
  sprintf(str, "M303 E%d C5 S%d U1", active_extruder, thermalManager.target_temperature[active_extruder]);
  enqueue_and_echo_command(str);
  ftState |= FTSTATE_AUTOPID_ING;
}

static void moveAxis(AxisEnum axis, float val) {
//  current_position[axis] += val;
//  if (current_position[axis] < 0&&axis<E_AXIS)current_position[axis] = 0;
//  planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], movFeedrate>0.0 ? movFeedrate : pgm_read_float(&homing_feedrate_mm_s[axis]), active_extruder);

		if(planner.movesplanned()!=0) return; // geo-f:add 20180716

    if(axis==E_AXIS) {           
      if(thermalManager.degHotend(active_extruder) > 180) {
        // Get the new position
        current_position[axis] += val;

        // geo-f : add 20180607 , FLYING_BEAR
        movFeedrate = MOVE_E_FEEDRATE;
      }
      else {
        return;
      }
    }
    else {
      float min = current_position[axis] - 1000,
            max = current_position[axis] + 1000;

      #if HAS_SOFTWARE_ENDSTOPS
        // Limit to software endstops, if enabled
        if (soft_endstops_enabled) {
          #if ENABLED(MIN_SOFTWARE_ENDSTOPS)
            min = soft_endstop_min[axis];
          #endif
          #if ENABLED(MAX_SOFTWARE_ENDSTOPS)
            max = soft_endstop_max[axis];
          #endif
        }
      #endif
  

      // Get the new position
      current_position[axis] += val;

      // Limit only when trying to move towards the limit
      NOLESS(current_position[axis], min);
      NOMORE(current_position[axis], max);
      
      movFeedrate = MOVE_XYZ_FEEDRATE;
    }

    planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], movFeedrate>0.0 ? movFeedrate : pgm_read_float(&homing_feedrate_mm_s[axis]), active_extruder);
}

static void manualLevelingMove(float x, float y) {
  current_position[Z_AXIS] = Z_CLEARANCE_DEPLOY_PROBE;
  planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], MMM_TO_MMS(HOMING_FEEDRATE_Z), active_extruder);
  current_position[X_AXIS] = x;
  current_position[Y_AXIS] = y;
  planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], MMM_TO_MMS(HOMING_FEEDRATE_XY), active_extruder);
  current_position[Z_AXIS] = 0;
  planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], MMM_TO_MMS(HOMING_FEEDRATE_Z), active_extruder);
}

static int16_t filament_temp_preheat(bool ifHeatBed = false) {
    int16_t tempe, tempb;
    uint8_t extruder_index = 0;
    if (tempChoice < FILAMENT_OPE_CHOICES){
      extruder_index = 0;
      tempe = lcd_preheat_hotend_temp[tempChoice];
      //SERIAL_ECHOLNPAIR("tempb1:", tempb);
    }
    #if EXTRUDERS>1
      else if(tempChoice < FILAMENT_OPE_CHOICES*2){
        extruder_index = 1;
        tempe = lcd_preheat_hotend_temp[tempChoice-FILAMENT_OPE_CHOICES];
        tempb = lcd_preheat_bed_temp[tempChoice-FILAMENT_OPE_CHOICES];
      }
    #endif
    #if HAS_TEMP_BED
      else if(tempChoice < FILAMENT_OPE_CHOICES*3){
        extruder_index = 0;
        tempb = lcd_preheat_bed_temp[tempChoice-FILAMENT_OPE_CHOICES*2];
      }
    #endif
    else{
      extruder_index = 0;
      tempe = 260;
      tempb = 100;
      //SERIAL_ECHOLNPAIR("tempb2:", tempb);
    }
    
    if (ifHeatBed) { 
      #if HAS_TEMP_BED
        thermalManager.setTargetBed(tempb);
      #endif
    }
    else { 
      thermalManager.setTargetHotend(tempe, extruder_index);
    }
    
    #if FAN_COUNT > 0
    if(extruder_index<FAN_COUNT) {
      #if EXTRUDERS>1
        fanSpeeds[extruder_index] = lcd_preheat_fan_speed[tempChoice-FILAMENT_OPE_CHOICES];
      #else
        fanSpeeds[extruder_index] = lcd_preheat_fan_speed[tempChoice];
      #endif
    }
    #endif

    myFysTLcd.ftCmdStart(VARADDR_ACTIVE_EXTRUDER_HOTEND);
    myFysTLcd.ftCmdPut16(tempe);    
    myFysTLcd.ftCmdSend();

    #if HAS_TEMP_BED
      if (ifHeatBed) {
        myFysTLcd.ftCmdStart(VARADDR_ACTIVE_EXTRUDER_BED);
        myFysTLcd.ftCmdPut16(tempb);    
        myFysTLcd.ftCmdSend();
      }
    #endif

    #if EXTRUDERS>1
      myFysTLcd.ftCmdStart(VARADDR_ACTIVE_EXTRUDERS_HOTEND);
      myFysTLcd.ftCmdPut16(tempe);      
      myFysTLcd.ftCmdSend();
    #endif

    return tempe;
}

static void filament_load() {
  int16_t tempe = filament_temp_preheat(false) - 5;
  uint8_t extru_index = 0;
  if (tempChoice < FILAMENT_OPE_CHOICES){
    extru_index = 0;
  }
  #if EXTRUDERS>1
    else if(tempChoice < FILAMENT_OPE_CHOICES*2){
      extru_index = 1;
    }
  #endif
  if (!planner.is_full() && thermalManager.degHotend(extru_index) > tempe){
    #if FYSTLCD_PAGE_EXIST(FILAMENT_LOADING)
      lcd_set_page(FTPAGE(FILAMENT_LOADING));
    #endif      
    current_position[E_AXIS] += MOVE_E_LENGTH_EACH_TIME;
    planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 
        MOVE_E_FEEDRATE, extru_index);
  }
}

static void filament_unload() {
  int16_t tempe = filament_temp_preheat(false) - 5;
  uint8_t extru_index = 0;
  if (tempChoice < FILAMENT_OPE_CHOICES){
    extru_index = 0;
  }
  #if EXTRUDERS>1
    else if(tempChoice < FILAMENT_OPE_CHOICES*2){
      extru_index = 1;
    }
  #endif
  if (!planner.is_full() && thermalManager.degHotend(extru_index) > tempe){
    #if FYSTLCD_PAGE_EXIST(FILAMENT_UNLOADING)
      lcd_set_page(FTPAGE(FILAMENT_UNLOADING));
    #endif 
    current_position[E_AXIS] -= MOVE_E_LENGTH_EACH_TIME;
    planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], 
        MOVE_E_FEEDRATE, extru_index);
  }
}


// valid ：是否更新标志
static void dwin_update_file_list(bool valid) {  
  #if ENABLED(SDSUPPORT) || ENABLED(FYS_STORAGE_SUPPORT)

    uint8_t fileWindowStartIndex=0;
    if(card.isIndir()) {
      FysTLcd::ftPuts(VARADDR_FILES_NAME[0], "..", FYSTLCD_FILENAME_LEN);
      fileWindowStartIndex = 1;
    }
  
    char sdFileName[FYSTLCD_FILENAME_LEN + 1] = { 0 };
    for (uint8_t i = fileWindowStartIndex; i<FILE_WINDOW_SIZE; i++) {
        if (sdFileName[0] || sdFileName[1]) {
          memset(sdFileName, 0, FYSTLCD_FILENAME_LEN+1);            
        }
        
        if (valid) {
          if ((i-fileWindowStartIndex) < dwinFileWindowTopIndex) {
            card.getfilename(dwinFileWindowTopIndex - 1 - (i-fileWindowStartIndex));
            if(card.longFilename[0]!=0) {
              strncpy(sdFileName, card.longFilename, FYSTLCD_FILENAME_LEN);
            }
            else{
              strncpy(sdFileName, card.filename, FYSTLCD_FILENAME_LEN);
            }

            //SERIAL_PROTOCOLLN(card.longFilename);
            SERIAL_PROTOCOLLN(sdFileName);            
          }
        }
        
        FysTLcd::ftPuts(VARADDR_FILES_NAME[i], sdFileName, FYSTLCD_FILENAME_LEN);
    }
  #endif
}

static void dwin_file_select(const char *name,const char *longname) {    
  #if ENABLED(SDSUPPORT) || ENABLED(FYS_STORAGE_SUPPORT)
    if (card.sdprinting) return; 
  
    char sdFileName[FYSTLCD_FILENAME_LEN + 1] = { 0 };    
    //card.getfilename(sd_num - 1);                  
    if(longname[0]!=0) {
      strncpy(sdFileName, longname, FYSTLCD_FILENAME_LEN);
    }
    else {
      strncpy(sdFileName, name, FYSTLCD_FILENAME_LEN);
    }    
    
    if (card.isFileOpen()) card.closefile();
    card.openFile(name, true);
    #if !defined(FILE_PRINT_NEED_CONRIRM)
      card.startFileprint();
      print_job_timer.start();
    #endif
    
    //char*t = strchr(sdFileName, '.');
    //while (*t)*t++ = '\0';
    FysTLcd::ftPuts(VARADDR_PRINTFILE_NAME, sdFileName, FYSTLCD_FILENAME_LEN);
  #endif
}

void lcd_showFilename() {
  #if ENABLED(SDSUPPORT) || ENABLED(FYS_STORAGE_SUPPORT)
    char sdFileName[FYSTLCD_FILENAME_LEN + 1] = { 0 };    
                  
    if(card.longFilename[0]!=0) {
      strncpy(sdFileName, card.longFilename, FYSTLCD_FILENAME_LEN);
    }
    else {
      strncpy(sdFileName, card.filename, FYSTLCD_FILENAME_LEN);
    }
    
    char*t = strchr(sdFileName, '.');
    while (*t)*t++ = '\0';
    FysTLcd::ftPuts(VARADDR_PRINTFILE_NAME, sdFileName, FYSTLCD_FILENAME_LEN);
  #endif
}

static void reportCmdContent(uint16_t& tar) {
  for (uint8_t i = 0; i<4; i++) {
    char c = (tar >> ((3 - i) << 2)) & 0x0F;
    if (c > 9)MYSERIAL0.write(c - 10 + 'A');
    else MYSERIAL0.write(c + '0');
  }
}

static void dwin_on_cmd_tool(uint16_t tval) {
  switch (tval) {
    case VARADDR_TOOL_TUNEPID_ENTER:
      sendActiveExtrudersParam();
      ftState |= FTSTATE_UPDATE_NOW;
      #if FYSTLCD_PAGE_EXIST(TUNE_PID)
        lcd_set_page(FTPAGE(TUNE_PID));
      #endif
      break;
    case VARADDR_TOOL_TUNEPID_APPLY:
      readActiveExtrudersParam();
      break;
    case VARADDR_TOOL_TUNEPID_SAVE:
      readActiveExtrudersParam();
      lcd_save();
      break;
    case VARVAL_TOOL_HOME_ALL:
      enqueue_and_echo_commands_P(PSTR("G28"));
      break;
    case VARVAL_TOOL_HOME_X:
      enqueue_and_echo_commands_P(PSTR("G28 X0"));
      break;
    case VARVAL_TOOL_HOME_Y:
      enqueue_and_echo_commands_P(PSTR("G28 Y0"));
      break;
    case VARVAL_TOOL_HOME_Z:
      enqueue_and_echo_commands_P(PSTR("G28 Z0"));
      break;
    case VARVAL_TOOL_HOME_XY:
      enqueue_and_echo_commands_P(PSTR("G28 X0 Y0"));
      break;
    case VARVAL_TOOL_SHUTDOWN:
  		#if ENABLED(FYS_POWER_STATUS)&&PIN_EXISTS(PS_ON)
        powerStatus = POWER_DOWN_DOING;
  		#endif
      break;
      
    case VARVAL_TOOL_COOLDOWN:
      for (uint8_t e = 0; e<EXTRUDERS; e++) {
        thermalManager.setTargetHotend(0, e);
      }
      #if HAS_TEMP_BED
        thermalManager.setTargetBed(0);
      #endif
      break;
    
    case VARVAL_TOOL_PREHEAT_E1: 
      #if FYSTLCD_PAGE_EXIST(TEMP_PREHEAT_E1)
        lcd_set_page(FTPAGE(TEMP_PREHEAT_E1));
      #endif      
      break;
    case VARVAL_TOOL_PREHEAT_E2:
      #if EXTRUDERS>1
        #if FYSTLCD_PAGE_EXIST(TEMP_PREHEAT_E2)
          lcd_set_page(FTPAGE(TEMP_PREHEAT_E2));
        #endif 
      #endif
      break;
    
    case VARVAL_TOOL_PREHEAT_BED:
      #if HAS_TEMP_BED
        #if FYSTLCD_PAGE_EXIST(TEMP_PREHEAT_BED)
          lcd_set_page(FTPAGE(TEMP_PREHEAT_BED));
        #endif 
      #endif
      break;
    
    case VARVAL_TOOL_PREHEAT_PLA:
      tempChoice = TEMPERTURE_PREHEAT_CHOISE_E1_PLA;
      filament_temp_preheat();
      sendActiveExtrudersParam();
      break;
    case VARVAL_TOOL_PREHEAT_ABS:
      tempChoice = TEMPERTURE_PREHEAT_CHOISE_E1_ABS;
      filament_temp_preheat();
      sendActiveExtrudersParam();
      break;
    case VARVAL_TOOL_PREHEAT_PVA:
      tempChoice = TEMPERTURE_PREHEAT_CHOISE_E1_PVA;
      filament_temp_preheat();
      sendActiveExtrudersParam();
      break;
    case VARVAL_TOOL_PREHEAT_FLEX:
      tempChoice = TEMPERTURE_PREHEAT_CHOISE_E1_FLEX;
      filament_temp_preheat();
      sendActiveExtrudersParam();
      break;
    case VARVAL_TOOL_PREHEAT_PET:
      tempChoice = TEMPERTURE_PREHEAT_CHOISE_E1_PET;
      filament_temp_preheat();
      sendActiveExtrudersParam();
      break;
    case VARVAL_TOOL_PREHEAT_HIPS:
      tempChoice = TEMPERTURE_PREHEAT_CHOISE_E1_HIPS;
      filament_temp_preheat();
      sendActiveExtrudersParam();
      break;
    case VARVAL_TOOL_PREHEAT_PP:
      tempChoice = TEMPERTURE_PREHEAT_CHOISE_E1_PP;
      filament_temp_preheat();
      sendActiveExtrudersParam();
      break;
    case VARVAL_TOOL_PREHEAT_CUSTOM:
      tempChoice = TEMPERTURE_PREHEAT_CHOISE_E1_CUSTOM;
      #if FYSTLCD_PAGE_EXIST(TEMP_PREHEAT_CUSTOM)
        lcd_set_page(FTPAGE(TEMP_PREHEAT_CUSTOM));
      #endif      
      break;
      
    case VARVAL_TOOL_PREHEAT_EXTRUDER2_PLA:
      tempChoice = TEMPERTURE_PREHEAT_CHOISE_E2_PLA;
      filament_temp_preheat();
      sendActiveExtrudersParam();
      break;
    case VARVAL_TOOL_PREHEAT_EXTRUDER2_ABS:
      tempChoice = TEMPERTURE_PREHEAT_CHOISE_E2_ABS;
      filament_temp_preheat();
      sendActiveExtrudersParam();
      break;
    case VARVAL_TOOL_PREHEAT_EXTRUDER2_PVA:
      tempChoice = TEMPERTURE_PREHEAT_CHOISE_E2_PVA;
      filament_temp_preheat();
      sendActiveExtrudersParam();
      break;
    case VARVAL_TOOL_PREHEAT_EXTRUDER2_FLEX:
      tempChoice = TEMPERTURE_PREHEAT_CHOISE_E2_FLEX;
      filament_temp_preheat();
      sendActiveExtrudersParam();
      break;
    case VARVAL_TOOL_PREHEAT_EXTRUDER2_PET:
      tempChoice = TEMPERTURE_PREHEAT_CHOISE_E2_PET;
      filament_temp_preheat();
      sendActiveExtrudersParam();
      break;
    case VARVAL_TOOL_PREHEAT_EXTRUDER2_HIPS:
      tempChoice = TEMPERTURE_PREHEAT_CHOISE_E2_HIPS;
      filament_temp_preheat();
      sendActiveExtrudersParam();
      break;
    case VARVAL_TOOL_PREHEAT_EXTRUDER2_PP:
      tempChoice = TEMPERTURE_PREHEAT_CHOISE_E2_PP;
      filament_temp_preheat();
      sendActiveExtrudersParam();
      break;
    case VARVAL_TOOL_PREHEAT_EXTRUDER2_CUSTOM:
      tempChoice = TEMPERTURE_PREHEAT_CHOISE_E2_CUSTOM;
      #if FYSTLCD_PAGE_EXIST(TEMP_PREHEAT_CUSTOM)
        lcd_set_page(FTPAGE(TEMP_PREHEAT_CUSTOM));
      #endif      
      break;

    case VARVAL_TOOL_PREHEAT_BED_PLA:
      tempChoice = TEMPERTURE_PREHEAT_CHOISE_BED_PLA;
      filament_temp_preheat(true);
      sendActiveExtrudersParam();
      break;
    case VARVAL_TOOL_PREHEAT_BED_ABS:
      tempChoice = TEMPERTURE_PREHEAT_CHOISE_BED_ABS;
      filament_temp_preheat(true);
      sendActiveExtrudersParam();
      break;
    case VARVAL_TOOL_PREHEAT_BED_PVA:
      tempChoice = TEMPERTURE_PREHEAT_CHOISE_BED_PVA;
      filament_temp_preheat(true);
      sendActiveExtrudersParam();
      break;
    case VARVAL_TOOL_PREHEAT_BED_FLEX:
      tempChoice = TEMPERTURE_PREHEAT_CHOISE_BED_FLEX;
      filament_temp_preheat(true);
      sendActiveExtrudersParam();
      break;
    case VARVAL_TOOL_PREHEAT_BED_PET:
      tempChoice = TEMPERTURE_PREHEAT_CHOISE_BED_PET;
      filament_temp_preheat(true);
      sendActiveExtrudersParam();
      break;
    case VARVAL_TOOL_PREHEAT_BED_HIPS:
      tempChoice = TEMPERTURE_PREHEAT_CHOISE_BED_HIPS;
      filament_temp_preheat(true);
      sendActiveExtrudersParam();
      break;
    case VARVAL_TOOL_PREHEAT_BED_PP:
      tempChoice = TEMPERTURE_PREHEAT_CHOISE_BED_PP;
      filament_temp_preheat(true);
      sendActiveExtrudersParam();
      break;
    case VARVAL_TOOL_PREHEAT_BED_CUSTOM:
      tempChoice = TEMPERTURE_PREHEAT_CHOISE_BED_CUSTOM;
      #if FYSTLCD_PAGE_EXIST(TEMP_PREHEAT_CUSTOM)
        lcd_set_page(FTPAGE(TEMP_PREHEAT_CUSTOM));
      #endif      
      break;

    case VARVAL_TOOL_PREHEAT_CUSTOM_APPLY:
      // get the temperture
      myFysTLcd.ftCmdStart(VARADDR_TUNE_PREHEAT_CUSTOM);
      if (myFysTLcd.ftCmdReceive(2)) {        
        switch(tempChoice) { 
          case TEMPERTURE_PREHEAT_CHOISE_E1_CUSTOM: 
          case TEMPERTURE_PREHEAT_CHOISE_E2_CUSTOM:
            myFysTLcd.ftCmdGetI16(lcd_preheat_hotend_temp[FILAMENT_OPE_CHOICES-1]); 
            filament_temp_preheat();
            break;

          case TEMPERTURE_PREHEAT_CHOISE_BED_CUSTOM:
            myFysTLcd.ftCmdGetI16(lcd_preheat_bed_temp[FILAMENT_OPE_CHOICES-1]); 
            filament_temp_preheat(true);
            break;
        }       
        SERIAL_ECHOLNPAIR("tempChoice", tempChoice);
        SERIAL_ECHOLNPAIR("lcd_preheat_bed_temp", lcd_preheat_bed_temp[FILAMENT_OPE_CHOICES-1]);
      }
      
      sendActiveExtrudersParam();

      #if FYSTLCD_PAGE_EXIST(TEMP_PREHEAT)
        lcd_set_page(FTPAGE(TEMP_PREHEAT));
      #endif  
      break;

    case VARVAL_TOOL_PREHEAT_CUSTOM_CANCEL:
      switch(tempChoice) { 
          case TEMPERTURE_PREHEAT_CHOISE_E1_CUSTOM: 
            #if FYSTLCD_PAGE_EXIST(TEMP_PREHEAT_E1)
              lcd_set_page(FTPAGE(TEMP_PREHEAT_E1));
            #endif      
            break;
          case TEMPERTURE_PREHEAT_CHOISE_E2_CUSTOM:
            #if FYSTLCD_PAGE_EXIST(TEMP_PREHEAT_E2)
              lcd_set_page(FTPAGE(TEMP_PREHEAT_E2));
            #endif 
            break;

          case TEMPERTURE_PREHEAT_CHOISE_BED_CUSTOM:
            #if FYSTLCD_PAGE_EXIST(TEMP_PREHEAT_BED)
              lcd_set_page(FTPAGE(TEMP_PREHEAT_BED));
            #endif 
            break;
        }       
      break;
      
    case VARVAL_TOOL_M999:
      Running = true;
      MYSERIAL0.flush();
      SERIAL_ECHOLNPGM("M999 ok.");
      break;
    case VARVAL_TOOL_COOLDOWN_ACTIVE:
      thermalManager.setTargetHotend(0, active_extruder);
      break;
    case VARVAL_TOOL_AUTOPID:
      dwin_pid_auto_tune();
      break;
    case VARVAL_TOOL_LOCK_AXIS:
      myFysTLcd.ftCmdStart(VARADDR_STATUS_AXIS_LOCK);
      if (X_ENABLE_READ == X_ENABLE_ON) {
        disable_all_steppers();
        myFysTLcd.ftCmdJump(2);
      }
      else {
        enable_all_steppers();
        myFysTLcd.ftCmdPut16(1);
      }
      myFysTLcd.ftCmdSend();
      break;
    #if FAN_COUNT>0
      case VARVAL_TOOL_FAN_SWITCH:
        myFysTLcd.ftCmdStart(VARADDR_STATUS_FAN);
        if (fanSpeeds[active_extruder]>0) {
          fanSpeeds[active_extruder]=0;
          myFysTLcd.ftCmdJump(2);
        }
        else {
          fanSpeeds[active_extruder]=255;
          myFysTLcd.ftCmdPut16(1);
        }
        myFysTLcd.ftCmdSend();
        break;
    #endif
    
    #if HAS_SERVOS
      case VARVAL_TOOL_SERVO_SWITCH:
        myFysTLcd.ftCmdStart(VARADDR_STATUS_SERVO);
        if(ftState&FTSTATE_SERVO_STATUS) {
          ftState&=~FTSTATE_SERVO_STATUS;
          servo[0].move(90);
          myFysTLcd.ftCmdJump(2);
        }
        else {
          ftState|=FTSTATE_SERVO_STATUS;
          servo[0].move(10);
          myFysTLcd.ftCmdPut16(1);
        }
        myFysTLcd.ftCmdSend();
        break;
      case VARVAL_TOOL_SERVO_RESET:
        servo[0].move(160);
        break;
    #endif
    
    case VARVAL_TOOL_EXCHANGE_T:
      #if EXTRUDERS>1
        if (active_extruder + 1 < EXTRUDERS) {
          char str[3] = { 'T', active_extruder + '1', 0 };
          enqueue_and_echo_command(str);
        }
        else {
          char str[3] = { 'T', '0', 0 };
          enqueue_and_echo_command(str);
        }
      #endif
      break;
    case VARVAL_TOOL_POPINFO_CONFIRM:
      wait_for_user = false;
      dwin_popup_shutdown();
      break;
      
    case VARVAL_TOOL_AUTO_STOP_FILAMENT:
      if (periodFun == filament_load || periodFun == filament_unload) {
        periodFun = nullptr;
        tempChoice = 0;
      }
      stepper.quick_stop();
      thermalManager.setTargetHotend(0, 0);
      #if EXTRUDERS>1
        thermalManager.setTargetHotend(0, 1);
      #endif
      sendActiveExtrudersParam();
      #if FYSTLCD_PAGE_EXIST(FILAMENT)
        lcd_set_page(FTPAGE(FILAMENT));
      #endif
      break;
      
    case VARVAL_TOOL_AUTO_LEVELING:
      //#if HAS_BED_PROBE
      //  zprobe_zoffset -=0.3;
      //#endif
      dwin_popup(PSTR("\n    Leveling is in progress."),EPPT_INFO_WAITING); // geo-f : comment 20180608
      enqueue_and_echo_commands_P(PSTR("G28"));
      enqueue_and_echo_commands_P(PSTR("G29"));
      break;
        
    case VARVAL_TOOL_AUTO_LEVELING_BREAK:
      break;                        

    case VARVAL_TOOL_LEFTMOVE_X_0_1:
     	moveAxis(X_AXIS, -0.1);
      break;
    case VARVAL_TOOL_RIGHTMOVE_X_0_1:
     	moveAxis(X_AXIS, 0.1);
      break;
    case VARVAL_TOOL_LEFTMOVE_Y_0_1:
      moveAxis(Y_AXIS, -0.1);
      break;
    case VARVAL_TOOL_RIGHTMOVE_Y_0_1:
      moveAxis(Y_AXIS, 0.1);
      break;
    case VARVAL_TOOL_LEFTMOVE_Z_0_1:
      moveAxis(Z_AXIS, -0.1);
      break;
    case VARVAL_TOOL_RIGHTMOVE_Z_0_1:
      moveAxis(Z_AXIS, 0.1);
      break;
    case VARVAL_TOOL_LEFTMOVE_E_0_1:
     	moveAxis(E_AXIS, -0.1);
      break;
    case VARVAL_TOOL_RIGHTMOVE_E_0_1:
      moveAxis(E_AXIS, 0.1);
      break;
    case VARVAL_TOOL_LEFTMOVE_X_1:
     	moveAxis(X_AXIS, -1);
      break;
    case VARVAL_TOOL_RIGHTMOVE_X_1:
     	moveAxis(X_AXIS, 1);
      break;
    case VARVAL_TOOL_LEFTMOVE_Y_1:
      moveAxis(Y_AXIS, -1);
      break;
    case VARVAL_TOOL_RIGHTMOVE_Y_1:
      moveAxis(Y_AXIS, 1);
      break;
    case VARVAL_TOOL_LEFTMOVE_Z_1:
      moveAxis(Z_AXIS, -1);
      break;
    case VARVAL_TOOL_RIGHTMOVE_Z_1:
      moveAxis(Z_AXIS, 1);
      break;
    case VARVAL_TOOL_LEFTMOVE_E_1:
     	moveAxis(E_AXIS, -1);
      break;
    case VARVAL_TOOL_RIGHTMOVE_E_1:
      moveAxis(E_AXIS, 1);
      break;
    case VARVAL_TOOL_LEFTMOVE_X_10:
     	moveAxis(X_AXIS, -10);
      break;
    case VARVAL_TOOL_RIGHTMOVE_X_10:
     	moveAxis(X_AXIS, 10);
      break;
    case VARVAL_TOOL_LEFTMOVE_Y_10:
      moveAxis(Y_AXIS, -10);
      break;
    case VARVAL_TOOL_RIGHTMOVE_Y_10:
      moveAxis(Y_AXIS, 10);
      break;
    case VARVAL_TOOL_LEFTMOVE_Z_10:
      moveAxis(Z_AXIS, -10);
      break;
    case VARVAL_TOOL_RIGHTMOVE_Z_10:
      moveAxis(Z_AXIS, 10);
      break;
    case VARVAL_TOOL_LEFTMOVE_E_10:
     	moveAxis(E_AXIS, -10);
      break;
    case VARVAL_TOOL_RIGHTMOVE_E_10:
      moveAxis(E_AXIS, 10);
      break;
    case VARVAL_TOOL_ENTER_PAPER_HEIGHT:
      #if FYSTLCD_PAGE_EXIST(TOOL_PAPERHEIGHT)
        lcd_set_page(FTPAGE(TOOL_PAPERHEIGHT));
      #endif
      break;
    case VARVAL_TOOL_RESET:
      wdt_enable(WDTO_30MS);
      while (1) {};
      break;
    case VARVAL_TOOL_OPTION_LEFT:
      dwin_popup_shutdown();
      switch (optionId) {
        #if ENABLED(ADVANCED_PAUSE_FEATURE)
        case 1:
            advanced_pause_menu_response = ADVANCED_PAUSE_RESPONSE_RESUME_PRINT;
            break;
        #endif
        default:
            break;
      }
      break;
    case VARVAL_TOOL_OPTION_RIGHT:
      dwin_popup_shutdown();
      switch (optionId) {
        #if ENABLED(ADVANCED_PAUSE_FEATURE)
        case 1:
          advanced_pause_menu_response = ADVANCED_PAUSE_RESPONSE_EXTRUDE_MORE;
          break;
        #endif
        default:
          break;
      }
      break;
    case VARVAL_TOOL_COOLDOWN_HOTEND:
      thermalManager.setTargetHotend(0, active_extruder);
      myFysTLcd.ftCmdStart(VARADDR_TUNE_PREHEAT_HOTEND_TEMP);
      myFysTLcd.ftCmdJump(2);
      myFysTLcd.ftCmdSend();
      break;
    case VARVAL_TOOL_COOLDOWN_BED:
      #if HAS_HEATED_BED
        thermalManager.setTargetBed(0);
        myFysTLcd.ftCmdStart(VARADDR_TUNE_PREHEAT_BED_TEMP);
        myFysTLcd.ftCmdJump(2);
        myFysTLcd.ftCmdSend();
      #endif
      break;
    case VARVAL_TOOL_EMERGENCY_STOP_MOTOR: 
      #if ENABLED(FYS_POWERBREAK_STEPPER_STATUS)
        unsigned char _sreg = SREG; 
        sei();
        powerBreakStepperStatus = 1;
        while (powerBreakStepperStatus == 1);
        powerBreakStepperStatus = 0;
        disable_all_steppers();
        SREG = _sreg;
      #endif
      break;
    }
}

#if ENABLED(POWER_LOSS_RECOVERY)

  static void lcd_power_loss_recovery_resume() {
    char cmd[20];

    // Return to status now
    //lcd_return_to_status();
    #if FYSTLCD_PAGE_EXIST(PRINT)
      lcd_set_page(FTPAGE(PRINT));
    #endif
    
    delay(20);

    // Turn leveling off and home
    enqueue_and_echo_commands_P(PSTR("M420 S0\nG28 R0"
      #if ENABLED(MARLIN_DEV_MODE)
        " S"
      #elif !IS_KINEMATIC
        " X Y"
      #endif
    ));

    #if HAS_HEATED_BED
      const int16_t bt = job_recovery_info.target_temperature_bed;
      if (bt) {
        // Restore the bed temperature
        sprintf_P(cmd, PSTR("M190 S%i"), bt);
        enqueue_and_echo_command(cmd);
      }
    #endif

    // Restore all hotend temperatures
    HOTEND_LOOP() {
      const int16_t et = job_recovery_info.target_temperature[e];
      if (et) {
        #if HOTENDS > 1
          sprintf_P(cmd, PSTR("T%i"), e);
          enqueue_and_echo_command(cmd);
        #endif
        sprintf_P(cmd, PSTR("M109 S%i"), et);
        enqueue_and_echo_command(cmd);
      }
    }

    #if HOTENDS > 1
      sprintf_P(cmd, PSTR("T%i"), job_recovery_info.active_hotend);
      enqueue_and_echo_command(cmd);
    #endif

    // Restore print cooling fan speeds
    for (uint8_t i = 0; i < FAN_COUNT; i++) {
      int16_t f = job_recovery_info.fanSpeeds[i];
      if (f) {
        sprintf_P(cmd, PSTR("M106 P%i S%i"), i, f);
        enqueue_and_echo_command(cmd);
      }
    }

    // Start draining the job recovery command queue
    job_recovery_phase = JOB_RECOVERY_YES;
  }

  static void lcd_power_loss_recovery_cancel() {
    card.removeJobRecoveryFile();
    card.autostart_index = 0;
    //lcd_return_to_status();
    #if FYSTLCD_PAGE_EXIST(MAIN)
      lcd_set_page(FTPAGE(MAIN));
    #endif
    delay(20);
  }

#endif // POWER_LOSS_RECOVERY

#if ENABLED(SDSUPPORT) || ENABLED(FYS_STORAGE_SUPPORT)

  void lcd_sdcard_pause() {
    card.pauseSDPrint();
    print_job_timer.pause();
    #if ENABLED(PARK_HEAD_ON_PAUSE)
      enqueue_and_echo_commands_P(PSTR("M125"));
    #endif
    //lcd_reset_status();
  }

  void lcd_sdcard_resume() {
    #if ENABLED(PARK_HEAD_ON_PAUSE)
      enqueue_and_echo_commands_P(PSTR("M24"));
    #else
      card.startFileprint();
      print_job_timer.start();
    #endif
    //lcd_reset_status();
  }

  bool abort_sd_printing; // =false

  void lcd_sdcard_stop() {
    wait_for_heatup = wait_for_user = false;
    abort_sd_printing = true;
    //SERIAL_ECHOLNPGM("stop!");
    //lcd_setstatusPGM(PSTR(MSG_PRINT_ABORTED), -1);
    //lcd_return_to_status();
  }

#endif // SDSUPPORT || FYS_STORAGE_SUPPORT


static void dwin_on_cmd_print(uint16_t tval)
{
  #if ENABLED(SDSUPPORT) || ENABLED(FYS_STORAGE_SUPPORT)
    if (card.cardOK) {      
      switch (tval) {
        case VARVAL_PRINT_FILELIST:
          if(abort_sd_printing) return ;

          if (print_job_timer.isRunning() || print_job_timer.isPaused()) {
            #if FYSTLCD_PAGE_EXIST(PRINT)
              lcd_set_page(FTPAGE(PRINT));             
            #endif                
          }
          else {          
            #if ENABLED(USB_FLASH_DRIVE_SUPPORT)
              //Sd2Card::resetState();
              card.initsd();
              if(!card.cardOK) {
                SERIAL_ECHOLN("SD card reset state and init fail");
                #if FYSTLCD_PAGE_EXIST(FILE_INSERT_CARD)
                  lcd_set_page(FTPAGE(FILE_INSERT_CARD));
                #endif
                break;
              }
            #endif           
            #if FYSTLCD_PAGE_EXIST(FILELIST)
              lcd_set_page(FTPAGE(FILELIST));
            #endif
            card.setroot();
            dwinFileWindowTopIndex = card.getnrfilenames();
            //card.getWorkDirName();
            dwin_update_file_list(true);

            //SERIAL_ECHOLNPAIR("root index:", dwinFileWindowTopIndex);
          }
          break;

        case VARVAL_PRINT_FILELIST_DOWNPAGE:
        case VARVAL_PRINT_FILELIST_UPPAGE: {
          uint8_t realSize=FILE_WINDOW_SIZE;
          uint16_t fileCnt = card.getnrfilenames();
          if(card.isIndir()) realSize-=1; 
          if (tval == VARVAL_PRINT_FILELIST_DOWNPAGE)
          {                  
            if (dwinFileWindowTopIndex > realSize)
            {
              dwinFileWindowTopIndex -= realSize;
            }
            else
            {
              dwinFileWindowTopIndex = fileCnt;
            }
          }
          else
          {
            if (dwinFileWindowTopIndex+ realSize <= fileCnt)
            {
              dwinFileWindowTopIndex += realSize;
            }
            else
            {
              dwinFileWindowTopIndex = fileCnt%realSize;
              if(dwinFileWindowTopIndex==0) 
              {
                dwinFileWindowTopIndex = realSize;
              }
            }
          }
          dwin_update_file_list(true);      
        }
        break;
        
        case VARVAL_PRINT_RESUME_PRINT:
          if (card.isFileOpen()) {
            if (!card.sdprinting) {                                  
              lcd_sdcard_resume();
            }
          }
          break;

        case VARVAL_PRINT_PAUSE:
          if (card.isFileOpen()) {
            if (card.sdprinting) {                                  
              lcd_sdcard_pause();
            }
          }
          break;
            
        case VARVAL_PRINT_STOP:
          if (card.isFileOpen()) {                           
            #if FYSTLCD_PAGE_EXIST(MAIN)
              lcd_set_page(FTPAGE(MAIN));
            #endif
            lcd_sdcard_stop();              
            dynamicIcon = 0;
          }
          break;
            
        case VARVAL_PRINT_TUNE_ENTER:
          sendParam_Tune();
          #if FYSTLCD_PAGE_EXIST(PRINT_TUNE)
            lcd_set_page(FTPAGE(PRINT_TUNE));
          #endif
          break;
            
        case VARVAL_PRINT_TUNE_APPLY:
          retPageId = PAGENUM_PRINT;
          currentPageId = PAGENUM_PRINT;
          readParam_Tune();
          #if ENABLED(BABYSTEPPING)
            settings.save();
          #endif
          break;

        case VARVAL_PRINT_TUNE_FAN_ENTER:
          sendParam_Tune_fan();
          #if FYSTLCD_PAGE_EXIST(PRINT_TUNE_FAN)
            lcd_set_page(FTPAGE(PRINT_TUNE_FAN));
          #endif
          break;
            
        case VARVAL_PRINT_TUNE_FAN_APPLY:
          retPageId = PAGENUM_PRINT;
          currentPageId = PAGENUM_PRINT;
          readParam_Tune_fan();
          #if ENABLED(BABYSTEPPING)
            settings.save();
          #endif
          break;

        #if ENABLED(POWER_LOSS_RECOVERY)
          case VARVAL_PRINT_CONTINUE: 
            lcd_power_loss_recovery_resume();
            SERIAL_PROTOCOLPGM("Power-loss continue");
            break;              
          case VARVAL_PRINT_CANCEL: 
            SERIAL_PROTOCOLPGM("Power-loss cancel");
            lcd_power_loss_recovery_cancel();
            break;
        #endif
        
        #if defined(FILE_PRINT_NEED_CONRIRM)
          case VARVAL_PRINT_CONFIRM:
            card.startFileprint();
            print_job_timer.start();
            dynamicIcon = 0;
            #if FYSTLCD_PAGE_EXIST(PRINT)
            lcd_set_page(FTPAGE(PRINT));
            #endif
            break;
        #endif
        
        case VARVAL_PRINT_SD_REFRESH:
          card.initsd();
          if (card.cardOK) {
            dwinFileWindowTopIndex = card.getnrfilenames();
            card.getWorkDirName();
            dwin_update_file_list(true);
            SERIAL_ECHOLNPGM("Refresh ok.");
          }
          else {
            dwinFileWindowTopIndex = 0;
            dwin_update_file_list(false);
          }
          break;
            
        case VARVAL_PRINT_REPRINT:
          #if ENABLED(FYS_RECORD_CURRENT_PRINT_FILE)
            if (card.isFileOpen())card.closefile();
            card.openFile(currentPrintFile, true);
            if (!card.isFileOpen())
            {
                lcd_popup("Target file open fail.");
                return;
            }
            card.startFileprint();
            print_job_timer.start();
          #endif

          #if FYSTLCD_PAGE_EXIST(PRINT)
            lcd_set_page(FTPAGE(PRINT));
          #endif
          break;

        case VARVAL_PRINT_FILE_HOME:
          while(card.isIndir()) card.updir();
          #if FYSTLCD_PAGE_EXIST(MAIN)
            lcd_set_page(FTPAGE(MAIN));
          #endif
          break;
            
        default:
            for (uint8_t i = 0; i < FILE_WINDOW_SIZE; i++) {
              if (tval == VARVAL_PRINT_FILECHOOSE[i]) {                        
                if(card.isIndir()) {
                  if(dwinFileWindowTopIndex+1<=i) break;
                }
                else {
                  if(dwinFileWindowTopIndex<=i) break;
                }
                
              	if (card.isIndir() && 0 == i) { // meaning ".."
              		card.updir();
              		dwinFileWindowTopIndex = card.get_num_Files();
              		dwin_update_file_list(true);       
              		SERIAL_ECHOLNPAIR("root index:", dwinFileWindowTopIndex);
              	}
              	else {
                  uint8_t realIndex;
                  if(card.isIndir()) {
                    realIndex = dwinFileWindowTopIndex - i;
                  }
                  else {
                    realIndex = dwinFileWindowTopIndex - i - 1;
                  }
                                    
              		//if (card.getfilename(realIndex)) // handles empty directory (no action)
              		//{
              		  card.getfilename(realIndex);
              			if (card.filenameIsDir) {
              				// Build the new directory, and ensure it's terminated with a forward slash
              				//m_acCurrentDirectory.cat(m_oFileInfo.fileName);
              				//m_acCurrentDirectory.cat('/');
              				card.chdir(card.filename);                				
              				dwinFileWindowTopIndex = card.get_num_Files();
              				SERIAL_ECHOLNPAIR("isDir index:", dwinFileWindowTopIndex);
              				dwin_update_file_list(true);
              			}
              			else {              			  
              			  SERIAL_ECHOLNPAIR("file select:", card.filename);
                      #if defined(FILE_PRINT_NEED_CONRIRM)
                      
                        #if FYSTLCD_PAGE_EXIST(PRINTFILE_CONFIRM)
                         lcd_set_page(FTPAGE(PRINTFILE_CONFIRM));
                        #endif
                        
                      #elif FYSTLCD_PAGE_EXIST(PRINT)
                        lcd_set_page(FTPAGE(PRINT));
                      #endif

                      dwin_file_select(card.filename,card.longFilename);                                                
                      break;
              			}
              		}
              	//}                                                       
              }
            }
            break;
      }
    }
    else {
      #if ENABLED(USB_FLASH_DRIVE_SUPPORT)
        Sd2Card::resetState();        
      #endif
      card.initsd();
      if(!card.cardOK) {
        // geo-f : we need to call idle many times to init it , magic!
        #if ENABLED(USB_FLASH_DRIVE_SUPPORT)
          for(uint8_t i=0;i<30;i++) {
            Sd2Card::idle();   
            safe_delay(10);
          }
          card.initsd();
          if(!card.cardOK) {        
            #if FYSTLCD_PAGE_EXIST(FILE_INSERT_CARD)
              lcd_set_page(FTPAGE(FILE_INSERT_CARD));
            #endif

            return;
          }    
        #else 
          #if FYSTLCD_PAGE_EXIST(FILE_INSERT_CARD)
            lcd_set_page(FTPAGE(FILE_INSERT_CARD));
          #endif
          return;
        #endif
      }
      
      switch (tval) {
        case VARVAL_PRINT_FILELIST:
          if (print_job_timer.isRunning() || print_job_timer.isPaused()) {
            #if FYSTLCD_PAGE_EXIST(PRINT)
              lcd_set_page(FTPAGE(PRINT));
            #endif                
          }
          else {
            #if ENABLED(USB_FLASH_DRIVE_SUPPORT)
              Sd2Card::resetState();
              card.initsd();
              if(!card.cardOK) {
                SERIAL_ECHOLN("SD card reset state and init fail");
                #if FYSTLCD_PAGE_EXIST(FILE_INSERT_CARD)
                  lcd_set_page(FTPAGE(FILE_INSERT_CARD));
                #endif
                break;
              }
            #endif
            #if FYSTLCD_PAGE_EXIST(FILELIST)
              lcd_set_page(FTPAGE(FILELIST));
            #endif
            card.setroot();
            dwinFileWindowTopIndex = card.getnrfilenames();
            //card.getWorkDirName();
            dwin_update_file_list(true);
            //SERIAL_ECHOLNPAIR("root index:", dwinFileWindowTopIndex);
          }
          break;
        case VARVAL_PRINT_SD_REFRESH:
          if (card.cardOK) {
            dwinFileWindowTopIndex = card.getnrfilenames();
            card.getWorkDirName();
            dwin_update_file_list(true);
          }
          else {
            dwinFileWindowTopIndex = 0;
            dwin_update_file_list(false);
          }
          break;

        #if ENABLED(POWER_LOSS_RECOVERY)
        case VARVAL_PRINT_CANCEL:
          lcd_power_loss_recovery_cancel();
          break;
        #endif
      }
    }
  #endif
}

static void dwin_on_cmd_setting(uint16_t tval) {
  switch (tval) {
    case VARVAL_SETTING_MOTOR_ENTER:
        sendParam_Motor();
        #if HOTENDS > 1
          sendParam_ExtrudersMotor();
        #endif
        #if FYSTLCD_PAGE_EXIST(SETTING_MOTOR)
          lcd_set_page_force(FTPAGE(SETTING_MOTOR));
        #endif
        break;
    case VARVAL_SETTING_MOTOR_APPLY:
        readParam_Motor();
        #if HOTENDS > 1
          readParam_ExtrudersMotor();
        #endif  
        break;
    case VARVAL_SETTING_MOTOR_SAVE:
        readParam_Motor();
        #if HOTENDS > 1
          readParam_ExtrudersMotor();
        #endif
        lcd_save();
        #if FYSTLCD_PAGE_EXIST(SETTING)
          lcd_set_page(FTPAGE(SETTING));
        #endif
        break;
    case VARVAL_SETTING_LEVELING_ENTER:
        sendParam_Leveling();
        #if FYSTLCD_PAGE_EXIST(SETTING_LEVELING)
          lcd_set_page(FTPAGE(SETTING_LEVELING));
        #endif
        break;
    case VARVAL_SETTING_LEVELING_APPLY:
        readParam_Leveling();
        #if FYSTLCD_PAGE_EXIST(SETTING)
          lcd_set_page(FTPAGE(SETTING));
        #endif
        break;
    case VARVAL_SETTING_LEVELING_SAVE:
        readParam_Leveling();
        lcd_save();
        break;
    case VARVAL_SETTING_TEMP_PARA_ENTER:
        sendParam_Temp();
        #if HOTENDS > 1
          sendParam_ExtrudersTemp();
        #endif
        #if FYSTLCD_PAGE_EXIST(SETTING_TEMP_PARA)
          lcd_set_page(FTPAGE(SETTING_TEMP_PARA));
        #endif
        break;
    case VARVAL_SETTING_TEMP_PARA_APPLY:
        readParam_Temp();
        #if HOTENDS > 1
          readParam_ExtrudersTemp();
        #endif
        #if FYSTLCD_PAGE_EXIST(SETTING)
          lcd_set_page(FTPAGE(SETTING));
        #endif
        break;
    case VARVAL_SETTING_TEMP_PARA_SAVE:
        readParam_Temp();
        #if HOTENDS > 1
          readParam_ExtrudersTemp();
        #endif
        lcd_save();
        #if FYSTLCD_PAGE_EXIST(SETTING)
          lcd_set_page(FTPAGE(SETTING));
        #endif
        break;
    case VARVAL_SETTING_MATERIAL_ENTER:
        sendParam_Material();
        #if FYSTLCD_PAGE_EXIST(SETTING_MATERIAL)
        lcd_set_page(FTPAGE(SETTING_MATERIAL));
        #endif
        break;
    case VARVAL_SETTING_MATERIAL_APPLY:
        readParam_Material();
        break;
    case VARVAL_SETTING_MATERIAL_SAVE:
        readParam_Material();
        lcd_save();
        break;
    case VARVAL_SETTING_TMC2130_ENTER:
        sendParam_TMC2130();
        #if FYSTLCD_PAGE_EXIST(SETTING_TMC2130)
        lcd_set_page(FTPAGE(SETTING_TMC2130));
        #endif
        break;
    case VARVAL_SETTING_TMC2130_APPLY:
        readParam_TMC2130();
        break;
    case VARVAL_SETTING_TMC2130_SAVE:
        readParam_TMC2130();
        lcd_save();
        break;
    case VARVAL_SETTINGS_SAVE:
        lcd_save();
        break;
    case VARVAL_SETTINGS_RESET:
        settings.reset();
        break;
    case VARVAL_SETTINGS_RESETSAVE:
        settings.reset();
        lcd_save();
        break;
    case VARVAL_SETTINGS_SYSTEM:
        sendParam_system();
        #if FYSTLCD_PAGE_EXIST(SETTING_SYSTEM)
          lcd_set_page(FTPAGE(SETTING_SYSTEM));
        #endif
        break;
    case VARVAL_SETTINGS_SYSTEM_APPLY:
        readParam_system();
        break;
    case VARVAL_SETTINGS_SYSTEM_SAVE:
        readParam_system();
        lcd_save();
        break;
  }
}

static void dwin_on_cmd(millis_t& tNow) {
  if (!FysTLcd::ftAvailableCmd()) return;
  
	#if defined(FYS_ACTIVE_TIME_OVER)
    previous_move_ms = tNow;
	#endif

  uint16_t tval = FysTLcd::ftCmdVal16();

  // geo-f 
  /*
  SERIAL_ECHOPGM(" Addr:");
  reportCmdContent(FysTLcd::ftAddr);
  SERIAL_ECHOPGM(" Val:");
  reportCmdContent(tval);
  */
  
  uint8_t cmd[2];
  switch (FysTLcd::ftAddr) {
  case VARADDR_TOOL:
      dwin_on_cmd_tool(tval);
      break;
      
  case VARADDR_PRINT: 
      dwin_on_cmd_print(tval);
      break;
    
  case VARADDR_SETTING:
      dwin_on_cmd_setting(tval);
      break;
      
  #if HOTENDS > 1
    case VARADDR_EXTRUDERS:
      switch (tval) {
        case VARVAL_EXTRUDERS_OFFSET_ENTER:
          sendParam_ExtrudersOffset();
          #if FYSTLCD_PAGE_EXIST(SETTING_EXTRUDERS_OFFSET)
          lcd_set_page(FTPAGE(SETTING_EXTRUDERS_OFFSET));
          #endif
          break;
        case VARVAL_EXTRUDERS_OFFSET_APPLY:
          readParam_ExtrudersOffset();
          break;
        case VARVAL_EXTRUDERS_OFFSET_SAVE:
          readParam_ExtrudersOffset();
          lcd_save();
          break;
        case VARVAL_EXTRUDERS_MOTOR_ENTER:
          sendParam_ExtrudersMotor();
          #if FYSTLCD_PAGE_EXIST(SETTING_EXTRUDERS_MOTOR)
          lcd_set_page(FTPAGE(SETTING_EXTRUDERS_MOTOR));
          #endif
          break;
        case VARVAL_EXTRUDERS_MOTOR_APPLY:
          readParam_ExtrudersMotor();
          break;
        case VARVAL_EXTRUDERS_MOTOR_SAVE:
          readParam_ExtrudersMotor();
          lcd_save();
          break;
        case VARVAL_EXTRUDERS_TEMP_ENTER:
          sendParam_ExtrudersTemp();
          #if FYSTLCD_PAGE_EXIST(SETTING_EXTRUDERS_TEMP)
          lcd_set_page(FTPAGE(SETTING_EXTRUDERS_TEMP));
          #endif
          break;
        case VARVAL_EXTRUDERS_TEMP_APPLY:
          readParam_ExtrudersTemp();
          #if FYSTLCD_PAGE_EXIST(SETTING)
            lcd_set_page(FTPAGE(SETTING));
          #endif
          break;
        case VARVAL_EXTRUDERS_TEMP_SAVE:
          readParam_ExtrudersTemp();
          lcd_save();
          #if FYSTLCD_PAGE_EXIST(SETTING)
            lcd_set_page(FTPAGE(SETTING));
          #endif
          break;
      }
      break;
  #endif
  
  case VARADDR_FILAMENT_AUTO_ADD:
    switch (tval) {
      case VARVAL_FILAMENT_OPE_EXTRU1:
        #if FYSTLCD_PAGE_EXIST(FILAMENT_LOAD_1)
          lcd_set_page(FTPAGE(FILAMENT_LOAD_1));
        #endif
        break;
      case VARVAL_FILAMENT_OPE_EXTRU2:          
        #if EXTRUDERS>1
          #if FYSTLCD_PAGE_EXIST(FILAMENT_LOAD_2)
            lcd_set_page(FTPAGE(FILAMENT_LOAD_2));
          #endif
        #endif
        break;
        
      case VARVAL_FILAMENT_OPE_EXTRU1_PLA:
        tempChoice = TEMPERTURE_PREHEAT_CHOISE_E1_PLA;
        periodFun = filament_load;
        break;
      case VARVAL_FILAMENT_OPE_EXTRU1_ABS:
        tempChoice = TEMPERTURE_PREHEAT_CHOISE_E1_ABS;
        periodFun = filament_load;
        break;
      case VARVAL_FILAMENT_OPE_EXTRU1_PET:
        tempChoice = TEMPERTURE_PREHEAT_CHOISE_E1_PET;
        periodFun = filament_load;
        break;
      case VARVAL_FILAMENT_OPE_EXTRU1_FLEX:
        tempChoice = TEMPERTURE_PREHEAT_CHOISE_E1_FLEX;
        periodFun = filament_load;
        break;

      #if EXTRUDERS>1
        case VARVAL_FILAMENT_OPE_EXTRU2_PLA:
          tempChoice = TEMPERTURE_PREHEAT_CHOISE_E2_PLA;
          periodFun = filament_load;
          break;
        case VARVAL_FILAMENT_OPE_EXTRU2_ABS:
          tempChoice = TEMPERTURE_PREHEAT_CHOISE_E2_ABS;
          periodFun = filament_load;
          break;
        case VARVAL_FILAMENT_OPE_EXTRU2_PET:
          tempChoice = TEMPERTURE_PREHEAT_CHOISE_E2_PLA;
          periodFun = filament_load;
          break;
        case VARVAL_FILAMENT_OPE_EXTRU2_FLEX:
          tempChoice = TEMPERTURE_PREHEAT_CHOISE_E2_PLA;
          periodFun = filament_load;          
          break;      
      #endif
    }      
    break;
    
  case VARADDR_FILAMENT_AUTO_REMOVE:
    switch (tval) {
      case VARVAL_FILAMENT_OPE_EXTRU1:
        #if FYSTLCD_PAGE_EXIST(FILAMENT_UNLOAD_1)
          lcd_set_page(FTPAGE(FILAMENT_UNLOAD_1));
        #endif
        break;
      case VARVAL_FILAMENT_OPE_EXTRU2:          
        #if EXTRUDERS>1
          #if FYSTLCD_PAGE_EXIST(FILAMENT_UNLOAD_2)
            lcd_set_page(FTPAGE(FILAMENT_UNLOAD_2));
          #endif
        #endif
        break;

      case VARVAL_FILAMENT_OPE_EXTRU1_PLA:
        tempChoice = TEMPERTURE_PREHEAT_CHOISE_E1_PLA;
        periodFun = filament_unload;
        break;
      case VARVAL_FILAMENT_OPE_EXTRU1_ABS:
        tempChoice = TEMPERTURE_PREHEAT_CHOISE_E1_ABS;
        periodFun = filament_unload;
        break;
      case VARVAL_FILAMENT_OPE_EXTRU1_PET:
        tempChoice = TEMPERTURE_PREHEAT_CHOISE_E1_PET;
        periodFun = filament_unload;
        break;
      case VARVAL_FILAMENT_OPE_EXTRU1_FLEX:
        tempChoice = TEMPERTURE_PREHEAT_CHOISE_E1_FLEX;
        periodFun = filament_unload;
        break;

      #if EXTRUDERS>1
        case VARVAL_FILAMENT_OPE_EXTRU2_PLA:
          tempChoice = TEMPERTURE_PREHEAT_CHOISE_E2_PLA;
          periodFun = filament_unload;
          break;
        case VARVAL_FILAMENT_OPE_EXTRU2_ABS:
          tempChoice = TEMPERTURE_PREHEAT_CHOISE_E2_ABS;
          periodFun = filament_unload;
          break;
        case VARVAL_FILAMENT_OPE_EXTRU2_PET:
          tempChoice = TEMPERTURE_PREHEAT_CHOISE_E2_PLA;
          periodFun = filament_unload;
          break;
        case VARVAL_FILAMENT_OPE_EXTRU2_FLEX:
          tempChoice = TEMPERTURE_PREHEAT_CHOISE_E2_PLA;
          periodFun = filament_unload;          
          break;      
      #endif   
    }      
    break;
    
  case VARADDR_MOVE_DIS:
      movDis = tval;
      movDis /= FYSTLCD_DOT_TEN_MUL;
      myFysTLcd.ftCmdStart(VARADDR_MOVE_DIS_SIGN);
      myFysTLcd.ftCmdPut16(tval);
      myFysTLcd.ftCmdSend();
      break;
      
  case VARADDR_MOVE_SPEED:
      movFeedrate = tval;
      movFeedrate /= FYSTLCD_DOT_TEN_MUL;
      myFysTLcd.ftCmdStart(VARADDR_MOVE_SPEED_SIGN);
      myFysTLcd.ftCmdPut16(tval);
      myFysTLcd.ftCmdSend();
      break;
      
  case VARADDR_JUMP_PAGE:
      retPageId = tval; 
      currentPageId = tval;
      //SERIAL_ECHOPGM("Page to"); // geo-f
      //MYSERIAL0.println((int)tval);
      break;
      
  case VARADDR_TUNE_PRINT_PERCENTAGE:
      oldFeedratePercentage = tval;
      feedrate_percentage = tval;
      break;
  case VARADDR_TUNE_FAN_SPEED:
    #if FAN_COUNT > 0
      oldFanSpeed=tval;
      fanSpeeds[active_extruder] = tval;
    #endif
    break;
  case VARADDR_TUNE_PREHEAT_HOTEND_TEMP:
    thermalManager.setTargetHotend(tval, 0);
    break;
  case VARADDR_TUNE_PREHEAT_BED_TEMP:
    #if HAS_HEATED_BED
      thermalManager.setTargetBed(tval);
    #endif
    break;
  case VARADDR_TUNE_PREHEAT_HOTEND2_SELECT:      
    #if EXTRUDERS>1
      #if FYSTLCD_PAGE_EXIST(FILAMENT_PURGE2)
        lcd_set_page(FTPAGE(FILAMENT_PURGE2));
      #endif
    #endif
    break;
  case VARADDR_TUNE_PREHEAT_HOTEND2_TEMP:
    #if EXTRUDERS>1
      thermalManager.setTargetHotend(tval, 1);
    #endif
    break;

  case VARADDR_TUNE_MOVE_E:
    switch(tval) {
      case VARVAL_TUNE_FORWARD_E:      
        moveAxis(E_AXIS, 5);
        break;
      case VARVAL_TUNE_BACKWARD_E:
        moveAxis(E_AXIS, -5);
        break;
    }
    break;

  default:
    break;;
  }
}

#if defined(VARADDR_PROMPT_DATA)&&VARADDR_PROMPT_DATA>0
static void lcd_period_prompt_report() {
  /*******************
  *   10A0 X_MIN_ENDSTOP 
  *   10A1 X_MAX_ENDSTOP 
  *   10A2 Y_MIN_ENDSTOP 
  *   10A3 Y_MAX_ENDSTOP 
  *   10A4 Z_MIN_ENDSTOP 
  *   10A5 Z_MAX_ENDSTOP 
  *   10A6 reserved
  *   10A7 FAN ico 
  *   10A8 PRINT ico 
  *   10A9 reserved
  *   10AA autoPid ico 
  ********************/
  myFysTLcd.ftCmdStart(VARADDR_PROMPT_DATA);
  #if HAS_X_MIN //10A0
  if(READ(X_MIN_PIN) ^ X_MIN_ENDSTOP_INVERTING)myFysTLcd.ftCmdPut16(1);
  else myFysTLcd.ftCmdJump(2);
  #else
  myFysTLcd.ftCmdJump(2);
  #endif

  #if HAS_X_MAX //10A1
  if (READ(X_MAX_PIN) ^ X_MAX_ENDSTOP_INVERTING)myFysTLcd.ftCmdPut16(1);
  else myFysTLcd.ftCmdJump(2);
  #else
  myFysTLcd.ftCmdJump(2);
  #endif

  #if HAS_Y_MIN //10A2
  if(READ(Y_MIN_PIN) ^ Y_MIN_ENDSTOP_INVERTING)myFysTLcd.ftCmdPut16(1);
  else myFysTLcd.ftCmdJump(2);
  #else
  myFysTLcd.ftCmdJump(2);
  #endif

  #if HAS_Y_MAX //10A3
  if(READ(Y_MAX_PIN) ^ Y_MAX_ENDSTOP_INVERTING)myFysTLcd.ftCmdPut16(1);
  else myFysTLcd.ftCmdJump(2);
  #else
  myFysTLcd.ftCmdJump(2);
  #endif

  #if HAS_Z_MIN //10A4
  if(READ(Z_MIN_PIN) ^ Z_MIN_ENDSTOP_INVERTING)myFysTLcd.ftCmdPut16(1);
  else myFysTLcd.ftCmdJump(2);
  #else
  myFysTLcd.ftCmdJump(2);
  #endif

  #if HAS_Z_MAX //10A5
  if(READ(Z_MAX_PIN) ^ Z_MAX_ENDSTOP_INVERTING)myFysTLcd.ftCmdPut16(1);
  else myFysTLcd.ftCmdJump(2);
  #else
  myFysTLcd.ftCmdJump(2);
  #endif

  myFysTLcd.ftCmdJump(2);//10A6 reserved
  
  dynamicIcon++;
  if (dynamicIcon > 9)dynamicIcon = 0;
  
  #if FAN_COUNT > 0
    if (fanSpeeds[active_extruder] > 0)
        myFysTLcd.ftCmdPut16(dynamicIcon);
    else
        myFysTLcd.ftCmdJump(2);
  #else
    myFysTLcd.ftCmdJump(2);
  #endif

  #if ENABLED(SDSUPPORT) || ENABLED(FYS_STORAGE_SUPPORT)
    if (card.sdprinting)//10A8 print ico
      myFysTLcd.ftCmdPut16(dynamicIcon);
    else
  #endif
  
  myFysTLcd.ftCmdJump(2);
  
  myFysTLcd.ftCmdJump(2);//10A9 reserved
  if (ftState&FTSTATE_AUTOPID_ING) {
    myFysTLcd.ftCmdPut16(dynamicIcon);//10AA auto PID ico
  }
  else {
    myFysTLcd.ftCmdJump(2);
  }
  myFysTLcd.ftCmdSend();
}
#endif

static void lcd_period_report(int16_t s) {
  uint8_t i;

  myFysTLcd.ftCmdStart(VARADDR_PERIOD_DATA);
  
  //myFysTLcd.ftCmdPutF16(thermalManager.degHotend(0));
  myFysTLcd.ftCmdPut16((int16_t)thermalManager.degHotend(0));
  
  #if HAS_FAN0
    myFysTLcd.ftCmdPut16(fanSpeeds[0]);
  #else
    myFysTLcd.ftCmdJump(2);
  #endif
  
  #if HAS_TEMP_BED
   //myFysTLcd.ftCmdPutF16(thermalManager.degBed());
   myFysTLcd.ftCmdPut16((int16_t)thermalManager.degBed());
  #else
   myFysTLcd.ftCmdJump(2);
  #endif
  myFysTLcd.ftCmdPut16(feedrate_percentage);

  // 
  for (i = 0; i < 4;i++){
    myFysTLcd.ftCmdPutF16(current_position[i]);
  }

  #if ENABLED(SDSUPPORT) || ENABLED(FYS_STORAGE_SUPPORT)
    static int progress = 0;
    if (IS_SD_PRINTING)progress = card.percentDone();
    if (progress> 100)progress = 0;
    myFysTLcd.ftCmdPut16(progress);
  #else
    myFysTLcd.ftCmdJump(2);
  #endif
  
  #if defined(FYS_ACTIVE_TIME_OVER)
    myFysTLcd.ftCmdPut16(s);
  #else
    myFysTLcd.ftCmdJump(2);
  #endif
  myFysTLcd.ftCmdSend();
  
  // geo-f : zprope offset
  #if HAS_BED_PROBE
    myFysTLcd.ftCmdStart(VARADDR_ZOFFSET_DATA);
    myFysTLcd.ftCmdPutF16_2(zprobe_zoffset);
    myFysTLcd.ftCmdSend();
  #endif
  
  #if EXTRUDERS>1
    myFysTLcd.ftCmdStart(VARADDR_EXTRUDERS_TEMP);
    for (uint8_t e = 1; e < EXTRUDERS; e++) {
//      myFysTLcd.ftCmdPutF16(thermalManager.degHotend(e));
      myFysTLcd.ftCmdPut16((int16_t)thermalManager.degHotend(e));
    }
    myFysTLcd.ftCmdSend();
 #endif

  #if HAS_FAN1    
    myFysTLcd.ftCmdStart(VARADDR_FANS);
    myFysTLcd.ftCmdPut16(fanSpeeds[1]);
    myFysTLcd.ftCmdSend();      
  #endif

  #if ENABLED(SDSUPPORT) || ENABLED(FYS_STORAGE_SUPPORT)
  
    if (IS_SD_PRINTING) {
      myFysTLcd.ftCmdStart(VARADDR_PRINT_TIME);
      millis_t pt = print_job_timer.duration();
      card.percentDone();
      float total = float(pt) / (float)card.percentDone();
      static float smoothTotal = 0;
      myFysTLcd.ftPutTime(pt);
      myFysTLcd.ftCmdSend(8);
      smoothTotal = (smoothTotal * 999L + total) / 1000L;
      if (isinf(smoothTotal))smoothTotal = total;
      if (pt < 120) {
        myFysTLcd.ftPutPGM(PSTR("Unknown."));
        smoothTotal = total;
      }
      else {
        pt = smoothTotal;
        myFysTLcd.ftPutTime(pt);
      }
      myFysTLcd.ftCmdSend();
    }
    
  #endif
}

static void sendActiveExtrudersParam() {
  myFysTLcd.ftCmdStart(VARADDR_ACTIVE_EXTRUDER_PARAM);
  //int n = active_extruder;
  myFysTLcd.ftCmdPut16(0);
  myFysTLcd.ftCmdPut16(thermalManager.target_temperature[0]);
  myFysTLcd.ftCmdPutF32(PID_PARAM(Kp, 0));
  myFysTLcd.ftCmdPutF32(unscalePID_i(PID_PARAM(Ki, 0)));
  myFysTLcd.ftCmdPutF32(unscalePID_d(PID_PARAM(Kd, 0)));
  #if ENABLED(PID_EXTRUSION_SCALING)
    myFysTLcd.ftCmdPutF32(PID_PARAM(Kc, 0));
  #else
    myFysTLcd.ftCmdJump(4);
  #endif
  
  #if HAS_TEMP_BED
    myFysTLcd.ftCmdPut16(thermalManager.target_temperature_bed);
  #else
    myFysTLcd.ftCmdJump(2);
  #endif
  myFysTLcd.ftCmdSend();

  #if EXTRUDERS == 2
    myFysTLcd.ftCmdStart(VARADDR_ACTIVE_EXTRUDERS_PARAM);
    myFysTLcd.ftCmdPut16(1);
    myFysTLcd.ftCmdPut16(thermalManager.target_temperature[1]);
    myFysTLcd.ftCmdPutF32(PID_PARAM(Kp, 1));
    myFysTLcd.ftCmdPutF32(unscalePID_i(PID_PARAM(Ki, 1)));
    myFysTLcd.ftCmdPutF32(unscalePID_d(PID_PARAM(Kd, 1)));
    #if ENABLED(PID_EXTRUSION_SCALING)
      myFysTLcd.ftCmdPutF32(PID_PARAM(Kc, 1));
    #else
      myFysTLcd.ftCmdJump(4);
    #endif

    myFysTLcd.ftCmdSend();
  #endif
}

static void readActiveExtrudersParam() {
  myFysTLcd.ftCmdStart(VARADDR_ACTIVE_EXTRUDER_PARAM);
  if (myFysTLcd.ftCmdReceive(22)) {
    myFysTLcd.ftCmdJump(2);// ignore hotend number
    int16_t t;
    myFysTLcd.ftCmdGetI16(t);
    myFysTLcd.ftCmdGetF32(PID_PARAM(Kp, 0));
    myFysTLcd.ftCmdGetF32(PID_PARAM(Ki, 0));
    PID_PARAM(Ki, 0) = scalePID_i(PID_PARAM(Ki, 0));
    myFysTLcd.ftCmdGetF32(PID_PARAM(Kd, 0));
    PID_PARAM(Kd, 0) = scalePID_d(PID_PARAM(Kd, 0));
    #if ENABLED(PID_EXTRUSION_SCALING)
      myFysTLcd.ftCmdGetF32(PID_PARAM(Kc, 0));
    #else
      myFysTLcd.ftCmdJump(4);
    #endif
    thermalManager.setTargetHotend(t, 0);

    #if HAS_TEMP_BED
      myFysTLcd.ftCmdGetI16(t);
      thermalManager.setTargetBed(t);
    #endif
  }

  #if EXTRUDERS == 2

    myFysTLcd.ftCmdStart(VARADDR_ACTIVE_EXTRUDERS_PARAM);
    if (myFysTLcd.ftCmdReceive(20)) {
      myFysTLcd.ftCmdJump(2);// ignore hotend number
      int16_t t;
      myFysTLcd.ftCmdGetI16(t);
      myFysTLcd.ftCmdGetF32(PID_PARAM(Kp, 1));
      myFysTLcd.ftCmdGetF32(PID_PARAM(Ki, 1));
      PID_PARAM(Ki, 1) = scalePID_i(PID_PARAM(Ki, 1));
      myFysTLcd.ftCmdGetF32(PID_PARAM(Kd, 1));
      PID_PARAM(Kd, 1) = scalePID_d(PID_PARAM(Kd, 1));
      #if ENABLED(PID_EXTRUSION_SCALING)
        myFysTLcd.ftCmdGetF32(PID_PARAM(Kc, 1));
      #else
        myFysTLcd.ftCmdJump(4);
      #endif
      thermalManager.setTargetHotend(t, 1);
    }
  
  #endif
}

static void sendParam_Tune(){
  uint8_t e;
  myFysTLcd.ftCmdStart(VARADDR_PARAM_TUNE);
  #if HAS_TEMP_ADC_0
    myFysTLcd.ftCmdPut16(thermalManager.target_temperature[0]);
  #else
    myFysTLcd.ftCmdJump(2);
  #endif
  #if HAS_TEMP_ADC_1
    myFysTLcd.ftCmdPut16(thermalManager.target_temperature[1]);
  #else
    myFysTLcd.ftCmdJump(2);
  #endif
  #if HAS_TEMP_ADC_2
    myFysTLcd.ftCmdPut16(thermalManager.target_temperature[2]);
  #else
    myFysTLcd.ftCmdJump(2);
  #endif
  #if HAS_TEMP_ADC_3
    myFysTLcd.ftCmdPut16(thermalManager.target_temperature[3]);
  #else
    myFysTLcd.ftCmdJump(2);
  #endif
  #if HAS_TEMP_ADC_4
    myFysTLcd.ftCmdPut16(thermalManager.target_temperature[4]);
  #else
    myFysTLcd.ftCmdJump(2);
  #endif
  #if HAS_TEMP_BED
    myFysTLcd.ftCmdPut16(thermalManager.target_temperature_bed);
  #else
    myFysTLcd.ftCmdJump(2);
  #endif

  // VARADDR_PARAM_TUNE + 0x06
  // move to sendParam_Tune_fan
  //#if FAN_COUNT > 0
  //  for(e=0;e<FAN_COUNT;e++)
  //  {
  //      myFysTLcd.ftCmdPut16(fanSpeeds[e]);
  //  }
  //  for (; e<5; e++)myFysTLcd.ftCmdJump(2);
  //#else
    myFysTLcd.ftCmdJump(10);
  //#endif

  // VARADDR_PARAM_TUNE + 0x0b
  myFysTLcd.ftCmdPut16(feedrate_percentage);
  for (e = 0; e < EXTRUDERS; e++)
  {
      myFysTLcd.ftCmdPut16(planner.flow_percentage[e]); 
  }
  for (; e < 5; e++) myFysTLcd.ftCmdJump(2);
  
  myFysTLcd.ftCmdSend();
}

static void readParam_Tune()
{
    int16_t t;
    myFysTLcd.ftCmdStart(VARADDR_PARAM_TUNE);
    if (myFysTLcd.ftCmdReceive(32)) {
      #if HAS_TEMP_ADC_0
        myFysTLcd.ftCmdGetI16(t);
        thermalManager.setTargetHotend(t, 0);
      #else
        myFysTLcd.ftCmdJump(2);
      #endif
      #if HAS_TEMP_ADC_1
        myFysTLcd.ftCmdGetI16(t);
        thermalManager.setTargetHotend(t, 1);
      #else
        myFysTLcd.ftCmdJump(2);
      #endif
      #if HAS_TEMP_ADC_2
        myFysTLcd.ftCmdGetI16(t);
        thermalManager.setTargetHotend(t,2);
      #else
        myFysTLcd.ftCmdJump(2);
      #endif
      #if HAS_TEMP_ADC_3
        myFysTLcd.ftCmdGetI16(t);
        thermalManager.setTargetHotend(t,3);
      #else
        myFysTLcd.ftCmdJump(2);
      #endif
      #if HAS_TEMP_ADC_4
        myFysTLcd.ftCmdGetI16(t);
        thermalManager.setTargetHotend(t, 4);
      #else
        myFysTLcd.ftCmdJump(2);
      #endif
      #if HAS_TEMP_BED
        myFysTLcd.ftCmdGetI16(t);
        thermalManager.setTargetBed(t);
      #else
        myFysTLcd.ftCmdJump(2);
      #endif

      // move to readParam_Tune_fan
      uint8_t e;
      //#if FAN_COUNT > 0
      //  for (e = 0; e<FAN_COUNT; e++) myFysTLcd.ftCmdGetI16(fanSpeeds[e]);
      //  for (; e<5; e++) myFysTLcd.ftCmdJump(2);
      //#else
      myFysTLcd.ftCmdJump(10);
      //#endif
      
      myFysTLcd.ftCmdGetI16(feedrate_percentage);
      for (e = 0; e < EXTRUDERS; e++) {         
		    myFysTLcd.ftCmdGetI16(planner.flow_percentage[e]);
		    planner.refresh_e_factor(e);
      }
      for (; e<5; e++)myFysTLcd.ftCmdJump(2);		
    }
}

static void sendParam_Tune_fan(){
  uint8_t e;
  myFysTLcd.ftCmdStart(VARADDR_PARAM_TUNE+6);  
  // VARADDR_PARAM_TUNE + 6
  #if FAN_COUNT > 0
    for(e=0;e<FAN_COUNT;e++) {
        myFysTLcd.ftCmdPut16(fanSpeeds[e]);
    }
    for (; e<5; e++)myFysTLcd.ftCmdJump(2);
  #else
    myFysTLcd.ftCmdJump(10);
  #endif

  myFysTLcd.ftCmdSend();
}

static void readParam_Tune_fan()
{
    int16_t t;
    myFysTLcd.ftCmdStart(VARADDR_PARAM_TUNE+6);
    if (myFysTLcd.ftCmdReceive(10)) {      
      uint8_t e;
      #if FAN_COUNT > 0
        for (e = 0; e<FAN_COUNT; e++) myFysTLcd.ftCmdGetI16(fanSpeeds[e]);
        for (; e<5; e++) myFysTLcd.ftCmdJump(2);
      #else
        myFysTLcd.ftCmdJump(10);
      #endif	
    }
}

static void sendParam_Motor() {    
  myFysTLcd.ftCmdStart(VARADDR_PARAM_MOTOR);
  for (uint8_t i = 0; i < 4; i++)
  {
      myFysTLcd.ftCmdPutF32(planner.axis_steps_per_mm[i]);
  }
  for (uint8_t i = 0; i < 4; i++)
  {
      myFysTLcd.ftCmdPutF32(planner.max_feedrate_mm_s[i]);
  }
  myFysTLcd.ftCmdSend();

  for (uint8_t i = 0; i < 4; i++)
  {
      myFysTLcd.ftCmdPutI32((int32_t)planner.max_acceleration_mm_per_s2[i]);
  }
  myFysTLcd.ftCmdPutF32(planner.acceleration);
  myFysTLcd.ftCmdPutF32(planner.retract_acceleration);
  myFysTLcd.ftCmdPutF32(planner.travel_acceleration);
  myFysTLcd.ftCmdPutF32(planner.min_feedrate_mm_s);
  myFysTLcd.ftCmdSend();

  //myFysTLcd.ftCmdPutI32((int32_t)planner.min_segment_time);
  myFysTLcd.ftCmdPutI32((int32_t)planner.min_segment_time_us);
  myFysTLcd.ftCmdPutF32(planner.min_travel_feedrate_mm_s);
  for(uint8_t i=0;i<4;i++)
  {
      myFysTLcd.ftCmdPutF32(planner.max_jerk[i]);
  }
  myFysTLcd.ftCmdSend();

#if HAS_HOME_OFFSET
  #if ENABLED(DELTA)
  myFysTLcd.ftCmdJump(8);
  float f=DELTA_HEIGHT + home_offset[Z_AXIS];
  myFysTLcd.ftCmdPutF32(f);
  #else
  for(uint8_t i=0;i<3;i++)
  {
      myFysTLcd.ftCmdPutF32(home_offset[i]);
  }
  #endif
#else
  myFysTLcd.ftCmdJump(12);
#endif
#if HAS_MOTOR_CURRENT_PWM
  for (uint8_t q = 0; q<3;q++)
  {
      myFysTLcd.ftCmdPutI32((int32_t)stepper.motor_current_setting[q]);
  }
  myFysTLcd.ftCmdPutI32(MOTOR_CURRENT_PWM_RANGE);
#else
  myFysTLcd.ftCmdJump(16);
#endif
  myFysTLcd.ftCmdSend();
}
static void readParam_Motor() {   
  myFysTLcd.ftCmdStart(VARADDR_PARAM_MOTOR);
  if (myFysTLcd.ftCmdReceive(32))
  {
      for (uint8_t i = 0; i < 4; i++)
      {
          myFysTLcd.ftCmdGetF32(planner.axis_steps_per_mm[i]);
      }
			planner.refresh_positioning(); // geo-f:20180726
      for (uint8_t i = 0; i < 4; i++)
      {
          myFysTLcd.ftCmdGetF32(planner.max_feedrate_mm_s[i]);
      }
  }
  myFysTLcd.ftCmdClear();
  if (myFysTLcd.ftCmdReceive(32))
  {
      int32_t t;
      for (uint8_t i = 0; i < 4; i++)
      {
          myFysTLcd.ftCmdGetI32(t);
          planner.max_acceleration_mm_per_s2[i] = t;
      }
      myFysTLcd.ftCmdGetF32(planner.acceleration);
      myFysTLcd.ftCmdGetF32(planner.retract_acceleration);
      myFysTLcd.ftCmdGetF32(planner.travel_acceleration);
      myFysTLcd.ftCmdGetF32(planner.min_feedrate_mm_s);
  }
  myFysTLcd.ftCmdClear();
  if (myFysTLcd.ftCmdReceive(24))
  {
      int32_t t;
      myFysTLcd.ftCmdGetI32(t);
      //planner.min_segment_time = t;
      planner.min_segment_time_us = t; 
      myFysTLcd.ftCmdGetF32(planner.min_travel_feedrate_mm_s);
      myFysTLcd.ftCmdGetF32(planner.max_jerk[X_AXIS]);
      myFysTLcd.ftCmdGetF32(planner.max_jerk[Y_AXIS]);
      myFysTLcd.ftCmdGetF32(planner.max_jerk[Z_AXIS]);
      myFysTLcd.ftCmdGetF32(planner.max_jerk[E_AXIS]);
  }
  myFysTLcd.ftCmdClear();
  if (myFysTLcd.ftCmdReceive(24))
  {
#if HAS_HOME_OFFSET
  #if ENABLED(DELTA)
      myFysTLcd.ftCmdJump(8);
      myFysTLcd.ftCmdGetF32(home_offset[Z_AXIS]);
      home_offset[Z_AXIS] -= DELTA_HEIGHT;
  #else
      for (uint8_t i = 0; i < 3; i++)
      {
          myFysTLcd.ftCmdGetF32(home_offset[i]);
      }
  #endif
#else
      myFysTLcd.ftCmdJump(12);
#endif
#if HAS_MOTOR_CURRENT_PWM
      int32_t t;
      for (uint8_t q = 0; q<3; q++)
      {
          myFysTLcd.ftCmdGetI32(t);
          stepper.motor_current_setting[q]=t;
      }
#else
      myFysTLcd.ftCmdJump(12);
#endif
  }
}

#if HOTENDS > 1
static void sendParam_ExtrudersOffset() {
  uint8_t e;
  myFysTLcd.ftCmdStart(VARADDR_PARAM_EXTRUDERS_OFFSET);
  #if EXTRUDERS==5
    for (e = 1; e < 4; e++)
    {
        for (uint8_t axis = 0; axis < 3; axis++)
        {
            myFysTLcd.ftCmdPutF32(hotend_offset[axis][e]);
        }
    }
    myFysTLcd.ftCmdSend(); 
    for (uint8_t axis = 0; axis < 3; axis++)
    {
        myFysTLcd.ftCmdPutF32(hotend_offset[axis][4]);
    }
    myFysTLcd.ftCmdSend(); 
  #else
    for (e = 1; e < EXTRUDERS; e++)
    {
        for (uint8_t axis = 0; axis < 3; axis++)
        {
            myFysTLcd.ftCmdPutF32(hotend_offset[axis][e]);
        }
    }
    myFysTLcd.ftCmdSend();
  #endif
}
static void readParam_ExtrudersOffset()
{
  myFysTLcd.ftCmdStart(VARADDR_PARAM_EXTRUDERS_OFFSET);
  #if EXTRUDERS==5
    if (myFysTLcd.ftCmdReceive(36))
    {
        for (uint8_t e = 1; e < 4; e++)
        for (uint8_t axis = 0; axis < 3; axis++)
        {
            myFysTLcd.ftCmdGetF32(hotend_offset[axis][e]);
        }
    }
    myFysTLcd.ftCmdClear();
    if (myFysTLcd.ftCmdReceive(12))
    {
        for (uint8_t axis = 0; axis < 3; axis++)
        {
            myFysTLcd.ftCmdGetF32(hotend_offset[axis][4]);
        }
    }
  #else
    if (myFysTLcd.ftCmdReceive((EXTRUDERS - 1) * 12))
    {
        for (uint8_t e = 1; e < EXTRUDERS; e++)
        for (uint8_t axis = 0; axis < 3; axis++)
        {
            myFysTLcd.ftCmdGetF32(hotend_offset[axis][e]);
        }
    }
  #endif
}
static void sendParam_ExtrudersMotor()
{
    myFysTLcd.ftCmdStart(VARADDR_PARAM_EXTRUDERS_MOTOR);
#if EXTRUDERS==5
    for (uint8_t e = 1; e < 4; e++)
    {
        myFysTLcd.ftCmdPutF32(planner.axis_steps_per_mm[3 + e]);
        myFysTLcd.ftCmdPutF32(planner.max_feedrate_mm_s[3 + e]);
        myFysTLcd.ftCmdPutF32(planner.max_acceleration_mm_per_s2[3 + e]);
    }
    myFysTLcd.ftCmdSend();
    myFysTLcd.ftCmdPutF32(planner.axis_steps_per_mm[7]);
    myFysTLcd.ftCmdPutF32(planner.max_feedrate_mm_s[7]);
    myFysTLcd.ftCmdPutF32(planner.max_acceleration_mm_per_s2[7]);
    myFysTLcd.ftCmdSend();
#else
    for (uint8_t e = 1; e < EXTRUDERS; e++)
    {
        myFysTLcd.ftCmdPutF32(planner.axis_steps_per_mm[3 + e]);
        myFysTLcd.ftCmdPutF32(planner.max_feedrate_mm_s[3 + e]);
        myFysTLcd.ftCmdPutF32(planner.max_acceleration_mm_per_s2[3 + e]);
    }
    myFysTLcd.ftCmdSend();
#endif
}
static void readParam_ExtrudersMotor()
{
    int32_t t;
    myFysTLcd.ftCmdStart(VARADDR_PARAM_EXTRUDERS_MOTOR);
#if EXTRUDERS==5
    if (myFysTLcd.ftCmdReceive(36))
    {
        for (uint8_t e = 1; e < 4; e++)
        {
            myFysTLcd.ftCmdGetF32(planner.axis_steps_per_mm[3 + e]);
            myFysTLcd.ftCmdGetF32(planner.max_feedrate_mm_s[3 + e]);
            myFysTLcd.ftCmdGetI32(t);
            planner.max_acceleration_mm_per_s2[3 + e] = t;
        }
    }
    myFysTLcd.ftCmdClear();
    if (myFysTLcd.ftCmdReceive(12))
    {
        myFysTLcd.ftCmdGetF32(planner.axis_steps_per_mm[7]);
        myFysTLcd.ftCmdGetF32(planner.max_feedrate_mm_s[7]);
        myFysTLcd.ftCmdGetI32(t);
        planner.max_acceleration_mm_per_s2[7] = t;
    }
#else
    if (myFysTLcd.ftCmdReceive((EXTRUDERS - 1) * 12))
    {
        for (uint8_t e = 1; e < EXTRUDERS; e++)
        {
            myFysTLcd.ftCmdGetF32(planner.axis_steps_per_mm[3 + e]);
            myFysTLcd.ftCmdGetF32(planner.max_feedrate_mm_s[3 + e]);
            myFysTLcd.ftCmdGetI32(t);
            planner.max_acceleration_mm_per_s2[3 + e] = t;
        }
    }
#endif
}
static void sendParam_ExtrudersTemp() {
  uint8_t e;
  myFysTLcd.ftCmdStart(VARADDR_PARAM_EXTRUDERS_TEMP);
  #if EXTRUDERS>3
    for (e = 1; e < 3; e++) {
      myFysTLcd.ftCmdPutF32(PID_PARAM(Kp, e));
      myFysTLcd.ftCmdPutF32(unscalePID_i(PID_PARAM(Ki, e)));
      myFysTLcd.ftCmdPutF32(unscalePID_d(PID_PARAM(Kd, e)));
      #if ENABLED(PID_EXTRUSION_SCALING)
        myFysTLcd.ftCmdPutF32(PID_PARAM(Kc, e));
      #else
        myFysTLcd.ftCmdJump(4);
      #endif
    }
    myFysTLcd.ftCmdSend();
    for (; e < EXTRUDERS; e++) {
      myFysTLcd.ftCmdPutF32(PID_PARAM(Kp, e));
      myFysTLcd.ftCmdPutF32(unscalePID_i(PID_PARAM(Ki, e)));
      myFysTLcd.ftCmdPutF32(unscalePID_d(PID_PARAM(Kd, e)));
      #if ENABLED(PID_EXTRUSION_SCALING)
        myFysTLcd.ftCmdPutF32(PID_PARAM(Kc, e));
      #else
        myFysTLcd.ftCmdJump(4);
      #endif
    }
    myFysTLcd.ftCmdSend();
  #else
    for (e = 1; e < EXTRUDERS; e++)
    {
        myFysTLcd.ftCmdPutF32(PID_PARAM(Kp, e));
        myFysTLcd.ftCmdPutF32(unscalePID_i(PID_PARAM(Ki, e)));
        myFysTLcd.ftCmdPutF32(unscalePID_d(PID_PARAM(Kd, e)));
        #if ENABLED(PID_EXTRUSION_SCALING)
        myFysTLcd.ftCmdPutF32(PID_PARAM(Kc, e));
        #else
        myFysTLcd.ftCmdJump(4);
        #endif
    }
    myFysTLcd.ftCmdSend();
  #endif
}
static void readParam_ExtrudersTemp()
{
  uint8_t e;
  myFysTLcd.ftCmdStart(VARADDR_PARAM_EXTRUDERS_TEMP);
  #if EXTRUDERS>3
    if (myFysTLcd.ftCmdReceive(32)) {
        for (e = 1; e < 3; e++)
        {
            myFysTLcd.ftCmdGetF32(PID_PARAM(Kp, e));
            myFysTLcd.ftCmdGetF32(PID_PARAM(Ki, e));
            PID_PARAM(Ki, e) = scalePID_i(PID_PARAM(Ki, e));
            myFysTLcd.ftCmdGetF32(PID_PARAM(Kd, e));
            PID_PARAM(Kd, e) = scalePID_d(PID_PARAM(Kd, e));
        #if ENABLED(PID_EXTRUSION_SCALING)
            myFysTLcd.ftCmdGetF32(PID_PARAM(Kc, e));
        #else
            myFysTLcd.ftCmdJump(4);
        #endif
        }
    }
    myFysTLcd.ftCmdClear();
    if (myFysTLcd.ftCmdReceive(32))
    {
        for (; e < EXTRUDERS; e++)
        {
            myFysTLcd.ftCmdGetF32(PID_PARAM(Kp, e));
            myFysTLcd.ftCmdGetF32(PID_PARAM(Ki, e));
            PID_PARAM(Ki, e) = scalePID_i(PID_PARAM(Ki, e));
            myFysTLcd.ftCmdGetF32(PID_PARAM(Kd, e));
            PID_PARAM(Kd, e) = scalePID_d(PID_PARAM(Kd, e));
        #if ENABLED(PID_EXTRUSION_SCALING)
            myFysTLcd.ftCmdGetF32(PID_PARAM(Kc, e));
        #else
            myFysTLcd.ftCmdJump(4);
        #endif
        }
    }
  #else
    if (myFysTLcd.ftCmdReceive(16*(EXTRUDERS-1)))
    {
        for (e = 1; e < EXTRUDERS; e++)
        {
            myFysTLcd.ftCmdGetF32(PID_PARAM(Kp, e));
            myFysTLcd.ftCmdGetF32(PID_PARAM(Ki, e));
            PID_PARAM(Ki, e) = scalePID_i(PID_PARAM(Ki, e));
            myFysTLcd.ftCmdGetF32(PID_PARAM(Kd, e));
            PID_PARAM(Kd, e) = scalePID_d(PID_PARAM(Kd, e));
        #if ENABLED(PID_EXTRUSION_SCALING)
            myFysTLcd.ftCmdGetF32(PID_PARAM(Kc, e));
        #else
            myFysTLcd.ftCmdJump(4);
        #endif
        }
    }
  #endif
}
#endif

static void sendParam_Leveling()
{
    myFysTLcd.ftCmdStart(VARADDR_PARAM_LEVELING);
    // Global Leveling
#if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
    myFysTLcd.ftCmdPutF32(planner.z_fade_height);
#elif ABL_PLANAR // geo-f : add
        myFysTLcd.ftCmdPutF32(zprobe_zoffset); // geo-f : add
#else
    myFysTLcd.ftCmdJump(4);
#endif

    myFysTLcd.ftCmdSend();
// Planar Bed Leveling matrix
#if ABL_PLANAR
    for (char i = 0; i < 9; i++)
    {
        myFysTLcd.ftCmdPutF32(planner.bed_level_matrix.matrix[i]);
    }
#else
    myFysTLcd.ftCmdJump(36);
#endif
    myFysTLcd.ftCmdSend();
}
static void readParam_Leveling()
{
    myFysTLcd.ftCmdStart(VARADDR_PARAM_LEVELING);
    if (myFysTLcd.ftCmdReceive(4))
    {
        // Global Leveling
#if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
        myFysTLcd.ftCmdGetF32(planner.z_fade_height);
#endif
    }
    if (myFysTLcd.ftCmdReceive(36))
    {
        // Planar Bed Leveling matrix
#if ABL_PLANAR
        for (char i = 0; i < 9; i++)
        {
            myFysTLcd.ftCmdGetF32(planner.bed_level_matrix.matrix[i]);
        }
#endif
    }
}

static void sendParam_Temp() {
  myFysTLcd.ftCmdStart(VARADDR_PARAM_TEMP);
  #if ENABLED(PIDTEMP)
    myFysTLcd.ftCmdPutF32(PID_PARAM(Kp, 0));
    myFysTLcd.ftCmdPutF32(unscalePID_i(PID_PARAM(Ki, 0)));
    myFysTLcd.ftCmdPutF32(unscalePID_d(PID_PARAM(Kd, 0)));
    #if ENABLED(PID_EXTRUSION_SCALING)
    myFysTLcd.ftCmdPutF32(PID_PARAM(Kc, 0));
    #else
    myFysTLcd.ftCmdJump(4);
    #endif
  #else
    myFysTLcd.ftCmdJump(16);
  #endif //!PIDTEMP
    myFysTLcd.ftCmdSend();

  #if DISABLED(PIDTEMPBED)
    myFysTLcd.ftCmdJump(16);
  #else
    myFysTLcd.ftCmdPutF32(thermalManager.bedKp);
    myFysTLcd.ftCmdPutF32(unscalePID_i(thermalManager.bedKi));
    myFysTLcd.ftCmdPutF32(unscalePID_d(thermalManager.bedKd));
    myFysTLcd.ftCmdJump(4);
  #endif
  myFysTLcd.ftCmdSend();
}
static void readParam_Temp()
{
    myFysTLcd.ftCmdStart(VARADDR_PARAM_TEMP);
    if (myFysTLcd.ftCmdReceive(32)) {
      #if ENABLED(PIDTEMP)
        myFysTLcd.ftCmdGetF32(PID_PARAM(Kp, 0));
        myFysTLcd.ftCmdGetF32(PID_PARAM(Ki, 0));
        PID_PARAM(Ki, 0) = scalePID_i(PID_PARAM(Ki, 0));
        myFysTLcd.ftCmdGetF32(PID_PARAM(Kd, 0));
        PID_PARAM(Kd, 0) = scalePID_d(PID_PARAM(Kd, 0));
        #if ENABLED(PID_EXTRUSION_SCALING)
        myFysTLcd.ftCmdGetF32(PID_PARAM(Kc, 0));
        #else
        myFysTLcd.ftCmdJump(4);
        #endif
      #else
        myFysTLcd.ftCmdJump(16);
      #endif //!PIDTEMP

      #if DISABLED(PIDTEMPBED)
        myFysTLcd.ftCmdJump(16);
      #else
        myFysTLcd.ftCmdGetF32(thermalManager.bedKp);
        myFysTLcd.ftCmdGetF32(thermalManager.bedKi);
        thermalManager.bedKi = scalePID_i(thermalManager.bedKi);
        myFysTLcd.ftCmdGetF32(thermalManager.bedKd);
        thermalManager.bedKd = scalePID_d(thermalManager.bedKd);
        myFysTLcd.ftCmdJump(4);
      #endif
    }
    myFysTLcd.ftCmdClear();
		
		thermalManager.updatePID();
}

static void sendParam_Material()
{
    myFysTLcd.ftCmdStart(VARADDR_PARAM_MATERIAL);
#if ENABLED(FWRETRACT)
    myFysTLcd.ftCmdPut16(autoretract_enabled);
    myFysTLcd.ftCmdPutF32(retract_length);
    #if EXTRUDERS > 1
    myFysTLcd.ftCmdPutF32(retract_length_swap);
    #else
    myFysTLcd.ftCmdJump(4);
    #endif
    myFysTLcd.ftCmdPutF32(retract_feedrate_mm_s);
    myFysTLcd.ftCmdPutF32(retract_zlift);
    myFysTLcd.ftCmdPutF32(retract_recover_length);
    #if EXTRUDERS > 1
    myFysTLcd.ftCmdPutF32(retract_recover_length_swap);
    #else
    myFysTLcd.ftCmdJump(4);
    #endif
    myFysTLcd.ftCmdPutF32(retract_recover_feedrate_mm_s);
#else
    myFysTLcd.ftCmdJump(30);
#endif // FWRETRACT
    //myFysTLcd.ftCmdPut16(volumetric_enabled); 
	myFysTLcd.ftCmdPut16(parser.volumetric_enabled);
    myFysTLcd.ftCmdSend();

    uint8_t e;
    //for (e = 0; e<EXTRUDERS; e++)myFysTLcd.ftCmdPutF32(filament_size[e]); 
    for (e = 0; e<EXTRUDERS; e++)myFysTLcd.ftCmdPutF32(planner.filament_size[e]); 
    for (; e < 5; e++)myFysTLcd.ftCmdJump(4);

// Linear Advance
#if ENABLED(LIN_ADVANCE)
    myFysTLcd.ftCmdPutF32(planner.extruder_advance_k);
    myFysTLcd.ftCmdPutF32(planner.advance_ed_ratio);
#else
    myFysTLcd.ftCmdJump(8);
#endif
    myFysTLcd.ftCmdSend();
}
static void readParam_Material()
{
    myFysTLcd.ftCmdStart(VARADDR_PARAM_MATERIAL);
    if(myFysTLcd.ftCmdReceive(32))
    {
        int16_t t;
    #if ENABLED(FWRETRACT)
        myFysTLcd.ftCmdGetI16(t);
        autoretract_enabled=t;
        myFysTLcd.ftCmdGetF32(retract_length);
        #if EXTRUDERS > 1
        myFysTLcd.ftCmdGetF32(retract_length_swap);
        #else
        myFysTLcd.ftCmdJump(4);
        #endif
        myFysTLcd.ftCmdGetF32(retract_feedrate_mm_s);
        myFysTLcd.ftCmdGetF32(retract_zlift);
        myFysTLcd.ftCmdGetF32(retract_recover_length);
        #if EXTRUDERS > 1
        myFysTLcd.ftCmdGetF32(retract_recover_length_swap);
        #else
        myFysTLcd.ftCmdJump(4);
        #endif
        myFysTLcd.ftCmdGetF32(retract_recover_feedrate_mm_s);
    #else
        myFysTLcd.ftCmdJump(30);
    #endif // FWRETRACT
        myFysTLcd.ftCmdGetI16(t);
        //volumetric_enabled = t; 
        parser.volumetric_enabled = t;
    }
    myFysTLcd.ftCmdClear();
    if(myFysTLcd.ftCmdReceive(28))
    {
        uint8_t e;
        //for (e = 0; e<EXTRUDERS; e++)myFysTLcd.ftCmdGetF32(filament_size[e]);
        for (e = 0; e<EXTRUDERS; e++)myFysTLcd.ftCmdGetF32(planner.filament_size[e]);
        for (; e < 5; e++)myFysTLcd.ftCmdJump(4);
        // Linear Advance
    #if ENABLED(LIN_ADVANCE)
        myFysTLcd.ftCmdGetF32(planner.extruder_advance_k);
        myFysTLcd.ftCmdGetF32(planner.advance_ed_ratio);
    #endif
    }
}

static void sendParam_TMC2130()
{
    int16_t t;
    myFysTLcd.ftCmdStart(VARADDR_PARAM_TMC2130);
#if ENABLED(HAVE_TMC2130)
    #if ENABLED(X_IS_TMC2130)
    t = stepperX.getCurrent();
    myFysTLcd.ftCmdPut16(t);
    #else
    myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(Y_IS_TMC2130)
    t=stepperY.getCurrent();
    myFysTLcd.ftCmdPut16(t);
    #else
    myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(Z_IS_TMC2130)
    t=stepperZ.getCurrent();
    myFysTLcd.ftCmdPut16(t);
    #else
    myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(E0_IS_TMC2130)
    t=stepperE0.getCurrent();
    myFysTLcd.ftCmdPut16(t);
    #else
    myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(X2_IS_TMC2130)
    t=stepperX2.getCurrent();
    myFysTLcd.ftCmdPut16(t);
    #else
    myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(Y2_IS_TMC2130)
    t=stepperY2.getCurrent();
    myFysTLcd.ftCmdPut16(t);
    #else
    myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(Z2_IS_TMC2130)
    t=stepperZ2.getCurrent();
    myFysTLcd.ftCmdPut16(t);
    #else
    myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(E1_IS_TMC2130)
    t=stepperE1.getCurrent();
    myFysTLcd.ftCmdPut16(t);
    #else
    myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(E2_IS_TMC2130)
    t=stepperE2.getCurrent();
    myFysTLcd.ftCmdPut16(t);
    #else
    myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(E3_IS_TMC2130)
    t=stepperE3.getCurrent();
    myFysTLcd.ftCmdPut16(t);
    #else
    myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(E4_IS_TMC2130)
    t=stepperE4.getCurrent();
    myFysTLcd.ftCmdPut16(t);
    #else
    myFysTLcd.ftCmdJump(2);
    #endif
#else
    myFysTLcd.ftCmdJump(22);
#endif
    myFysTLcd.ftCmdSend();
}
static void readParam_TMC2130()
{
    int16_t t;
    myFysTLcd.ftCmdStart(VARADDR_PARAM_TMC2130);
    if (myFysTLcd.ftCmdReceive(22))
    {
	#if ENABLED(HAVE_TMC2130)
    #if ENABLED(X_IS_TMC2130)
        myFysTLcd.ftCmdGetI16(t);
        stepperX.setCurrent(t, R_SENSE, HOLD_MULTIPLIER);
    #else
        myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(Y_IS_TMC2130)
        myFysTLcd.ftCmdGetI16(t);
        stepperY.setCurrent(t, R_SENSE, HOLD_MULTIPLIER);
    #else
        myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(Z_IS_TMC2130)
        myFysTLcd.ftCmdGetI16(t);
        stepperZ.setCurrent(t, R_SENSE, HOLD_MULTIPLIER);
    #else
        myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(X2_IS_TMC2130)
        myFysTLcd.ftCmdGetI16(t);
        stepperX2.setCurrent(t, R_SENSE, HOLD_MULTIPLIER);
    #else
        myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(Y2_IS_TMC2130)
        myFysTLcd.ftCmdGetI16(t)
        stepperY2.setCurrent(t, R_SENSE, HOLD_MULTIPLIER);
    #else
        myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(Z2_IS_TMC2130)
        myFysTLcd.ftCmdGetI16(t);
        stepperZ2.setCurrent(t, R_SENSE, HOLD_MULTIPLIER);
    #else
        myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(E0_IS_TMC2130)
        myFysTLcd.ftCmdGetI16(t);
        stepperE0.setCurrent(t, R_SENSE, HOLD_MULTIPLIER);
    #else
        myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(E1_IS_TMC2130)
        myFysTLcd.ftCmdGetI16(t);
        stepperE1.setCurrent(t, R_SENSE, HOLD_MULTIPLIER);
    #else
        myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(E2_IS_TMC2130)
        myFysTLcd.ftCmdGetI16(t);
        stepperE2.setCurrent(t, R_SENSE, HOLD_MULTIPLIER);
    #else
        myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(E3_IS_TMC2130)
        myFysTLcd.ftCmdGetI16(t);
        stepperE3.setCurrent(t, R_SENSE, HOLD_MULTIPLIER);
    #else
        myFysTLcd.ftCmdJump(2);
    #endif
    #if ENABLED(E4_IS_TMC2130)
        myFysTLcd.ftCmdGetI16(t);
        stepperE4.setCurrent(t, R_SENSE, HOLD_MULTIPLIER);
    #else
        myFysTLcd.ftCmdJump(2);
    #endif
#else
        myFysTLcd.ftCmdJump(22);
#endif
    }
}
static void sendParam_system()
{
    myFysTLcd.ftCmdStart(VARADDR_PARAM_SYSTEM);
#ifdef FYS_ENERGY_CONSERVE_HEIGHT
    myFysTLcd.ftCmdPutF16(zEnergyHeight);
#else
    myFysTLcd.ftCmdJump(2);
#endif
#if PIN_EXISTS(PS_ON)&&defined(FYS_ACTIVE_TIME_OVER)
    int16_t t=max_inactive_time/1000UL;
    myFysTLcd.ftCmdPut16(t);
#else
    myFysTLcd.ftCmdJump(2);
#endif
    myFysTLcd.ftCmdSend();
}
static void readParam_system()
{
    myFysTLcd.ftCmdStart(VARADDR_PARAM_SYSTEM);
    if (myFysTLcd.ftCmdReceive(4))
    {
      #ifdef FYS_ENERGY_CONSERVE_HEIGHT
        myFysTLcd.ftCmdGetF16(zEnergyHeight);
#else
        myFysTLcd.ftCmdJump(2);
#endif
#if PIN_EXISTS(PS_ON)&&defined(FYS_ACTIVE_TIME_OVER)
        int16_t t;
        myFysTLcd.ftCmdGetI16(t);
        max_inactive_time = (millis_t)t* 1000UL;
#else
        myFysTLcd.ftCmdJump(2);
#endif
    }
}

/*
void kill_screen(const char* msg) 
{
  char str[INFO_POPUP_LEN + 1], i, j, ch;
  if (msg)
  for (j = 0; j < INFOS_NUM; j++) {
    memset(str, 0, INFO_POPUP_LEN);
    i = 0;
    while (ch = *msg) {
      if (ch == '\n') {
          msg++;
          break;
      }
      str[i++] = ch;
      msg++;
      if (i >= INFO_POPUP_LEN)break;
    }
    FysTLcd::ftPuts(VARADDR_POP_INFOS[j], str, INFO_POPUP_LEN);
  }
     
  #if FYSTLCD_PAGE_EXIST(INFO_WAITING)
    lcd_pop_page(FTPAGE(INFO_WAITING));
  #endif           
}
*/

void kill_screen(const char* message) 
{
  char str[INFO_POPUP_LEN + 1], i, j, ch;
  char *pMsg = (char *)message;
  if (pMsg)
  for (j = 0; j < INFOS_NUM; j++) {
    memset(str, 0, INFO_POPUP_LEN);
    i = 0;
    while (ch = pgm_read_byte(pMsg)) {
      if (ch == '\n') {
          pMsg++;
          break;
      }
      str[i++] = ch;
      pMsg++;
      if (i >= INFO_POPUP_LEN) break;
    }
    SERIAL_ECHOLN(str);
    FysTLcd::ftPuts(VARADDR_POP_INFOS[j], str, INFO_POPUP_LEN);
  }
  
  #if FYSTLCD_PAGE_EXIST(INFO_WAITING)
    lcd_pop_page(FTPAGE(INFO_WAITING));
  #endif           
}

void dwin_popup(const char* msg, EM_POPUP_PAGE_TYPE pageChoose, char funid)
{
    char str[INFO_POPUP_LEN + 1], i, j, ch;
    if (msg)
    for (j = 0; j < INFOS_NUM; j++) {
      memset(str, 0, INFO_POPUP_LEN);
      i = 0;
      while (ch = pgm_read_byte(msg)) {
        if (ch == '\n') {
            msg++;
            break;
        }
        str[i++] = ch;
        msg++;
        if (i >= INFO_POPUP_LEN)break;
      }
      FysTLcd::ftPuts(VARADDR_POP_INFOS[j], str, INFO_POPUP_LEN);
    }
    
    switch (pageChoose) {
      case EPPT_INFO_POPUP:
        #if FYSTLCD_PAGE_EXIST(INFO_POPUP)
          lcd_pop_page(FTPAGE(INFO_POPUP));
        #endif
        break;
      case EPPT_INFO_WAITING:
        #if FYSTLCD_PAGE_EXIST(INFO_WAITING)
          lcd_pop_page(FTPAGE(INFO_WAITING));
        #endif
        break;
      case EPPT_INFO_OPTION:
        switch (funid) {
          #if ENABLED(ADVANCED_PAUSE_FEATURE)
            case 1:
                optionId = 1;
                advanced_pause_menu_response = ADVANCED_PAUSE_RESPONSE_WAIT_FOR;
                FysTLcd::ftPutsPGM(VARADDR_QUESTION_LEFT, PSTR("Resume"),8);
                FysTLcd::ftPutsPGM(VARADDR_QUESTION_RIGHT, PSTR("Extrude"),8);
                break;
          #endif
          default:
              break;
        }
        #if FYSTLCD_PAGE_EXIST(INFO_OPTION)
          lcd_pop_page(FTPAGE(INFO_OPTION));
        #endif
        break;
      default:
          break;
    }
}

void dwin_popup_shutdown() {
  if (
      #if FYSTLCD_PAGE_EXIST(INFO_POPUP)
        currentPageId == FTPAGE(INFO_POPUP) || 
      #endif
      #if FYSTLCD_PAGE_EXIST(INFO_WAITING)
        currentPageId == FTPAGE(INFO_WAITING) ||
      #endif
      #if FYSTLCD_PAGE_EXIST(INFO_OPTION)
        currentPageId == FTPAGE(INFO_OPTION) ||
      #endif
      0) {
    lcd_pop_page(retPageId);
  }
}

void lcd_startup_music() {
  FysTLcd::ftPlayMusic(0x00, 0x0a, 0xFF);
  delay(100);
}

void lcd_shutDown() {
  FysTLcd::ftPlayMusic(0x05, 0x02, 0xFF);
  lcd_set_page(0x00);
}

void lcd_setstatus(const char* message, const bool persist)
{
  (void)persist;
  #ifdef FYS_WIFI_ESP3D
    while (*message == ' ')message++;
    bool tf = false;
    const char*t;
    char n, d;
    for (n = 0, d = 0, t = message; (*t >= '0'&&*t <= '9') || *t == '.'; t++) {
      if (*t == '.') {
        if (tf) {
            d++;
            tf = false;
        }
        else break;
      }
      else {
        if (!tf) {
          tf = true;
          n++;
        }
      }
    }
    if (n == 4 && d == 3) {
      FysTLcd::ftPuts(VARADDR_WIFI_IP, message, ATTACH_STR_LEN);
      return;
    }
    
    const char* str = "SSID";
    for (t = message; *t; t++) {
      if (*t == str[0]) {
        const char *r=t;
        for(d=0;*r&&d<4;r++,d++)
        if (*r != str[d])break;
        if (d == 4) {
          t = r;
          t++;
          break;
        }
      }
    }
    if (*t) {
      FysTLcd::ftPuts(VARADDR_WIFI_SSID, t, ATTACH_STR_LEN);
      myFysTLcd.ftCmdStart(VARADDR_STATUS_WIFI);
      myFysTLcd.ftCmdPut16(1);
      myFysTLcd.ftCmdSend();
      return;
    }
	#else
    (void)message;
	#endif
}

void lcd_set_return_page_print() { 
	#if FYSTLCD_PAGE_EXIST(PRINT)
		lcd_set_return_page(FTPAGE(PRINT));
	#endif
}

void lcd_set_page_print() { 
	#if FYSTLCD_PAGE_EXIST(PRINT)
		lcd_set_page(FTPAGE(PRINT));
	#endif
}

void lcd_set_page_main() { 
	#if FYSTLCD_PAGE_EXIST(MAIN)
		lcd_set_page(FTPAGE(MAIN));
	#endif
}

  
#endif //FYSTLCD_V1

