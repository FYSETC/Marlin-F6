#include "FysTLcd.h"


//*************** 1010---1060: file list ****************/
static const uint16_t VARADDR_FILES_NAME[] = { 0x1010, 0x1020, 0x1030, 0x1040, 0x1050 };
static const uint8_t FILE_WINDOW_SIZE = sizeof(VARADDR_FILES_NAME) / sizeof(VARADDR_FILES_NAME[0]);

/************* 0x1070---0x109F status data ***************/
#define     VARADDR_STATUS_SD                       0x1070  
#define     VARADDR_STATUS_AXIS_LOCK                0x1071  
#define     VARADDR_STATUS_FAN                      0x1072  
#define     VARADDR_STATUS_SERVO                    0x1073  
#define     VARADDR_STATUS_WIFI                     0x1074  


/****************0x10A0---0x11FF period data****************/
#define     INFOS_NUM                               4
#define     VARADDR_PROMPT_DATA                     0x10A0  
#define     VARADDR_PRINTFILE_NAME                  0x10D0  
#define     VARADDR_PERIOD_DATA                     0x1100  
#define     VARADDR_ZOFFSET_DATA                    0x1110  
#define     VARADDR_EXTRUDERS_TEMP                  0x1111  // max end: 0x1114 , 5 extruders 
#define     VARADDR_FANS                            0x1116 

#define     VARADDR_ACTIVE_EXTRUDER_PARAM           0x1120  
#define     VARADDR_ACTIVE_EXTRUDER_HOTEND          0x1121
#define     VARADDR_ACTIVE_EXTRUDER_BED             0x112A
#define     VARADDR_ACTIVE_EXTRUDERS_PARAM          0x1130
#define     VARADDR_ACTIVE_EXTRUDERS_HOTEND         0x1131
#define     VARADDR_PRINT_TIME                      0x1150  
#define     VARADDR_START_UP                        0x1160 

static const uint16_t VARADDR_POP_INFOS[INFOS_NUM] = { 0x1180, 0x1190, 0x11A0, 0x11B0 };
#define     INFO_POPUP_LEN                          0x20


/****************0x1200---0x16FF parameters****************/
#define     VARADDR_PARAM_TUNE                      0x1200  // 1200~1205:temp 1206~120A:fan
#define     VARADDR_PARAM_MOTOR                     0x1280  
#define     VARADDR_PARAM_LEVELING                  0x1380  
#define     VARADDR_PARAM_TEMP                      0x1400  
#define     VARADDR_PARAM_MATERIAL                  0x1481  
#define     VARADDR_PARAM_TMC2130                   0x1500  
#define     VARADDR_PARAM_EXTRUDERS_OFFSET          0x1540  
#define     VARADDR_PARAM_EXTRUDERS_MOTOR           0x1560 
#define     VARADDR_PARAM_EXTRUDERS_TEMP            0x1580  
#define     VARADDR_PARAM_SYSTEM                    0x15A0  


/****************0x1700---0x1720 auto return****************/
#define     VARADDR_TOOL                            0x1700  

#define     VARADDR_TOOL_TUNEPID_ENTER              0x0001  
#define     VARADDR_TOOL_TUNEPID_APPLY              0x0002  
#define     VARADDR_TOOL_TUNEPID_SAVE               0x0003  
#define     VARVAL_TOOL_HOME_ALL                    0x0004  
#define     VARVAL_TOOL_HOME_X                      0x0005  
#define     VARVAL_TOOL_HOME_Y                      0x0006  
#define     VARVAL_TOOL_HOME_Z                      0x0007  
#define     VARVAL_TOOL_SHUTDOWN                    0x0008  
#define     VARVAL_TOOL_COOLDOWN                    0x0009
#define     VARVAL_TOOL_M999                        0x000C  
#define     VARVAL_TOOL_COOLDOWN_ACTIVE             0x000D  
#define     VARVAL_TOOL_AUTOPID                     0x000E  
#define     VARVAL_TOOL_LOCK_AXIS                   0x000F  
#define     VARVAL_TOOL_FAN_SWITCH                  0x0010  
#define     VARVAL_TOOL_SERVO_SWITCH                0x0011  
#define     VARVAL_TOOL_SERVO_RESET                 0x0012  
#define     VARVAL_TOOL_EXCHANGE_T                  0x0013  
#define     VARVAL_TOOL_POPINFO_CONFIRM             0x0014  
#define     VARVAL_TOOL_AUTO_STOP_FILAMENT          0x0015  
#define     VARVAL_TOOL_AUTO_LEVELING               0x0016  
#define     VARVAL_TOOL_AUTO_LEVELING_BREAK         0x0017  

#define     VARVAL_TOOL_LEFTMOVE_X_0_1              0x0018  
#define     VARVAL_TOOL_RIGHTMOVE_X_0_1             0x0019  
#define     VARVAL_TOOL_LEFTMOVE_Y_0_1              0x001A  
#define     VARVAL_TOOL_RIGHTMOVE_Y_0_1             0x001B  
#define     VARVAL_TOOL_LEFTMOVE_Z_0_1              0x001C  
#define     VARVAL_TOOL_RIGHTMOVE_Z_0_1             0x001D  
#define     VARVAL_TOOL_LEFTMOVE_E_0_1              0x001E  
#define     VARVAL_TOOL_RIGHTMOVE_E_0_1             0x001F
#define     VARVAL_TOOL_LEFTMOVE_X_1                0x0020  
#define     VARVAL_TOOL_RIGHTMOVE_X_1               0x0021  
#define     VARVAL_TOOL_LEFTMOVE_Y_1                0x0022  
#define     VARVAL_TOOL_RIGHTMOVE_Y_1               0x0023  
#define     VARVAL_TOOL_LEFTMOVE_Z_1                0x0024  
#define     VARVAL_TOOL_RIGHTMOVE_Z_1               0x0025  
#define     VARVAL_TOOL_LEFTMOVE_E_1                0x0026  
#define     VARVAL_TOOL_RIGHTMOVE_E_1               0x0027
#define     VARVAL_TOOL_LEFTMOVE_X_10               0x0028  
#define     VARVAL_TOOL_RIGHTMOVE_X_10              0x0029  
#define     VARVAL_TOOL_LEFTMOVE_Y_10               0x002A  
#define     VARVAL_TOOL_RIGHTMOVE_Y_10              0x002B  
#define     VARVAL_TOOL_LEFTMOVE_Z_10               0x002C  
#define     VARVAL_TOOL_RIGHTMOVE_Z_10              0x002D  
#define     VARVAL_TOOL_LEFTMOVE_E_10               0x002E  
#define     VARVAL_TOOL_RIGHTMOVE_E_10              0x002F

#define     VARVAL_TOOL_ENTER_PAPER_HEIGHT          0x0031
#define     VARVAL_TOOL_RESET                       0x0032  
#define     VARVAL_TOOL_HOME_XY                     0x0033  
#define     VARVAL_TOOL_OPTION_LEFT                 0x0034  
#define     VARVAL_TOOL_OPTION_RIGHT                0x0035  
#define     VARVAL_TOOL_COOLDOWN_HOTEND             0x0037  
#define     VARVAL_TOOL_COOLDOWN_BED                0x0038  
#define     VARVAL_TOOL_EMERGENCY_STOP_MOTOR        0x0039  

#define     VARVAL_TOOL_PREHEAT_E1                  0x003D  
#define     VARVAL_TOOL_PREHEAT_E2                  0x003E 
#define     VARVAL_TOOL_PREHEAT_BED                 0x003F  //

#define     VARVAL_TOOL_PREHEAT_PLA                 0x0040  
#define     VARVAL_TOOL_PREHEAT_ABS                 0x0041 
#define     VARVAL_TOOL_PREHEAT_PVA                 0x0042  //
#define     VARVAL_TOOL_PREHEAT_FLEX                0x0043  //
#define     VARVAL_TOOL_PREHEAT_PET                 0x0044 
#define     VARVAL_TOOL_PREHEAT_HIPS                0x0045 
#define     VARVAL_TOOL_PREHEAT_PP                  0x0046 
#define     VARVAL_TOOL_PREHEAT_CUSTOM              0x0047

#define     VARVAL_TOOL_PREHEAT_EXTRUDER2_PLA       0x0049  //
#define     VARVAL_TOOL_PREHEAT_EXTRUDER2_ABS       0x004A  //
#define     VARVAL_TOOL_PREHEAT_EXTRUDER2_PVA       0x004B  //
#define     VARVAL_TOOL_PREHEAT_EXTRUDER2_FLEX      0x004C  //
#define     VARVAL_TOOL_PREHEAT_EXTRUDER2_PET       0x004D 
#define     VARVAL_TOOL_PREHEAT_EXTRUDER2_HIPS      0x004E 
#define     VARVAL_TOOL_PREHEAT_EXTRUDER2_PP        0x004F 
#define     VARVAL_TOOL_PREHEAT_EXTRUDER2_CUSTOM    0x0050

#define     VARVAL_TOOL_PREHEAT_BED_PLA             0x0052  //
#define     VARVAL_TOOL_PREHEAT_BED_ABS             0x0053  //
#define     VARVAL_TOOL_PREHEAT_BED_PVA             0x0054  //
#define     VARVAL_TOOL_PREHEAT_BED_FLEX            0x0055  //
#define     VARVAL_TOOL_PREHEAT_BED_PET             0x0056 
#define     VARVAL_TOOL_PREHEAT_BED_HIPS            0x0057 
#define     VARVAL_TOOL_PREHEAT_BED_PP              0x0058 
#define     VARVAL_TOOL_PREHEAT_BED_CUSTOM          0x0059


#define     VARVAL_TOOL_PREHEAT_CUSTOM_APPLY        0x0060
#define     VARVAL_TOOL_PREHEAT_CUSTOM_CANCEL       0x0061


// -----------------------About print
#define     VARADDR_PRINT                           0x1701  

#define     VARVAL_PRINT_FILELIST                   0x0001  
#define     VARVAL_PRINT_FILELIST_UPPAGE            0x0002 
#define     VARVAL_PRINT_FILELIST_DOWNPAGE          0x0003 

static const uint16_t VARVAL_PRINT_FILECHOOSE[] = { 0x0004, 0x0005, 0x0006, 0x0007, 0x0008, 0x0009, 0x000A, 0x000B, 0x000C, 0x000D };//选中文件列表上某一个文件
#define     VARVAL_PRINT_RESUME_PRINT               0x0021  
#define     VARVAL_PRINT_PAUSE                      0x0022  
#define     VARVAL_PRINT_STOP                       0x0023  
#define     VARVAL_PRINT_TUNE_ENTER                 0x0024  
#define     VARVAL_PRINT_TUNE_APPLY                 0x0025  
#define     VARVAL_PRINT_CONTINUE                   0x0026  
#define     VARVAL_PRINT_CANCEL                     0x0027  
#define     VARVAL_PRINT_CONFIRM                    0x0028  
#define     VARVAL_PRINT_SD_REFRESH                 0x0029  
#define     VARVAL_PRINT_REPRINT                    0x002A  
#define     VARVAL_PRINT_FILE_HOME                  0x002B  
#define     VARVAL_PRINT_TUNE_FAN_ENTER             0x002C
#define     VARVAL_PRINT_TUNE_FAN_APPLY             0x002D

// ---------------------- About setting
#define     VARADDR_SETTING                         0x1702  

#define     VARVAL_SETTING_MOTOR_ENTER              0x0001  
#define     VARVAL_SETTING_MOTOR_APPLY              0x0002  
#define     VARVAL_SETTING_MOTOR_SAVE               0x0003  
#define     VARVAL_SETTING_LEVELING_ENTER           0x0004  
#define     VARVAL_SETTING_LEVELING_APPLY           0x0005  
#define     VARVAL_SETTING_LEVELING_SAVE            0x0006  
#define     VARVAL_SETTING_TEMP_PARA_ENTER          0x0007  
#define     VARVAL_SETTING_TEMP_PARA_APPLY          0x0008  
#define     VARVAL_SETTING_TEMP_PARA_SAVE           0x0009  
#define     VARVAL_SETTING_MATERIAL_ENTER           0x000A  
#define     VARVAL_SETTING_MATERIAL_APPLY           0x000B  
#define     VARVAL_SETTING_MATERIAL_SAVE            0x000C  
#define     VARVAL_SETTING_TMC2130_ENTER            0x000D  
#define     VARVAL_SETTING_TMC2130_APPLY            0x000E  
#define     VARVAL_SETTING_TMC2130_SAVE             0x000F  
#define     VARVAL_SETTINGS_SAVE                    0x00C0  
#define     VARVAL_SETTINGS_RESET                   0x00C1  
#define     VARVAL_SETTINGS_RESETSAVE               0x00C2  
#define     VARVAL_SETTINGS_SYSTEM                  0x00C3  
#define     VARVAL_SETTINGS_SYSTEM_APPLY            0x00C4  
#define     VARVAL_SETTINGS_SYSTEM_SAVE             0x00C5  

// ---------------------- About extruders
#define     VARADDR_EXTRUDERS                       0x1703  

#define     VARVAL_EXTRUDERS_OFFSET_ENTER           0x0001  
#define     VARVAL_EXTRUDERS_OFFSET_APPLY           0x0002  
#define     VARVAL_EXTRUDERS_OFFSET_SAVE            0x0003  
#define     VARVAL_EXTRUDERS_MOTOR_ENTER            0x0004  
#define     VARVAL_EXTRUDERS_MOTOR_APPLY            0x0005  
#define     VARVAL_EXTRUDERS_MOTOR_SAVE             0x0006  
#define     VARVAL_EXTRUDERS_TEMP_ENTER             0x0007  
#define     VARVAL_EXTRUDERS_TEMP_APPLY             0x0008  
#define     VARVAL_EXTRUDERS_TEMP_SAVE              0x0009  

// ---------------------- Others
#define     VARADDR_MOVE_DIS                        0x1704  
#define     VARADDR_MOVE_SPEED                      0x1705  

#define     VARADDR_FILAMENT_AUTO_ADD               0x1706  
#define     VARADDR_FILAMENT_AUTO_REMOVE            0x1707  
#define     VARVAL_FILAMENT_OPE_EXTRU1              0x0001  //
#define     VARVAL_FILAMENT_OPE_EXTRU2              0x0002  //
#define     VARVAL_FILAMENT_OPE_EXTRU1_PLA          0x0003  //
#define     VARVAL_FILAMENT_OPE_EXTRU1_ABS          0x0004  //
#define     VARVAL_FILAMENT_OPE_EXTRU1_PET          0x0005  //
#define     VARVAL_FILAMENT_OPE_EXTRU1_FLEX         0x0006  //

#define     VARVAL_FILAMENT_OPE_EXTRU2_PLA          0x0007  //
#define     VARVAL_FILAMENT_OPE_EXTRU2_ABS          0x0008  //
#define     VARVAL_FILAMENT_OPE_EXTRU2_PET          0x0009  //
#define     VARVAL_FILAMENT_OPE_EXTRU2_FLEX         0x000A  //


#define     VARADDR_JUMP_PAGE                       0x1710  

#define     VARADDR_TUNE_PRINT_PERCENTAGE           0x1711  
#define     VARADDR_TUNE_FAN_SPEED                  0x1712  
#define     VARADDR_TUNE_PREHEAT_HOTEND_TEMP        0x1713  
#define     VARADDR_TUNE_PREHEAT_BED_TEMP           0x1714  
#define     VARADDR_TUNE_PREHEAT_HOTEND2_SELECT     0x1715  
#define     VARADDR_TUNE_PREHEAT_HOTEND2_TEMP       0x1716 
#define     VARADDR_TUNE_PREHEAT_CUSTOM             0x1717
#define     VARADDR_TUNE_MOVE_E                     0x1720  
#define     VARVAL_TUNE_FORWARD_E                   0X0001
#define     VARVAL_TUNE_BACKWARD_E                  0X0002



/****************0x1721---0x17E0 additional****************/

#define     VARADDR_MOVE_DIS_SIGN                   0x1721  
#define     VARADDR_MOVE_SPEED_SIGN                 0x1722
#define     VARADDR_QUESTION_LEFT                   0x1730  
#define     VARADDR_QUESTION_RIGHT                  0x1738  
#define     VARADDR_WIFI_SSID                       0x1748  
#define     VARADDR_WIFI_PASSWORD                   0x1750  
#define     VARADDR_WIFI_IP                         0x1758  
#define     VARADDR_WIFI_DEV_MAC                    0x1760  

#define     VARADDR_VERSION_DATE                    0x1762  
#define     VARADDR_SERIAL_NUMBER                   0x176A  
#define     VARADDR_INFO_PRINT                      0x1780 
#define     VARADDR_INFO_PRINT_ACC_TIME             0x1788 

#define     ATTACH_STR_LEN                          0x14    
#define     SERIAL_NUMBER                           "010-2323-4545"

#define     PAGENUM_MAIN                            1
#define     PAGENUM_TEMP_PREHEAT                    10
#define     PAGENUM_TEMP_PREHEAT_E1                 12
#define     PAGENUM_TEMP_PREHEAT_E2                 14
#define     PAGENUM_TEMP_PREHEAT_BED                16
#define     PAGENUM_TEMP_PREHEAT_CUSTOM             18
#define     PAGENUM_FILELIST                        71
#define     PAGENUM_PRINT                           73
#define     PAGENUM_PRINT_TUNE                      75
#define     PAGENUM_PRINT_TUNE_FAN                  77
#define     PAGENUM_FILE_INSERT_CARD                80
#define     PAGENUM_POWER_LOSS_RECOVERY             100
#define     PAGENUM_INFO_OPTION                     112 
#define     PAGENUM_INFO_POPUP                      114
#define     PAGENUM_INFO_WAITING                    116
#define     PAGENUM_AUTO_LEVELING_COMPLETE          0
#define     PAGENUM_LEVELING_METHOD                 0
#define     PAGENUM_TUNE_PID                        0
#define     PAGENUM_PRINTFILE_CONFIRM               0
#define     PAGENUM_ACCOMPLISH_PRINT                0
#define     PAGENUM_MANUAL_LEVELING                 0
#define     PAGENUM_POPUP_SHUTDOWN_WARNING          0
#define     PAGENUM_TOOL_CALIBRATION                0
#define     PAGENUM_TOOL_PAPERHEIGHT                0
#define     PAGENUM_FILAMENT                        130
#define     PAGENUM_FILAMENT_LOAD_1                 142
#define     PAGENUM_FILAMENT_LOAD_2                 144
#define     PAGENUM_FILAMENT_LOADING                148
#define     PAGENUM_FILAMENT_UNLOAD_1               152
#define     PAGENUM_FILAMENT_UNLOAD_2               154
#define     PAGENUM_FILAMENT_UNLOADING              158
#define     PAGENUM_FILAMENT_PURGE2                 160
#define     PAGENUM_SETTING                         210
#define     PAGENUM_SETTING_MOTOR                   212
#define     PAGENUM_SETTING_TEMP_PARA               214
#define     PAGENUM_SETTING_EXTRUDERS_OFFSET        0
#define     PAGENUM_SETTING_EXTRUDERS_MOTOR         0
#define     PAGENUM_SETTING_EXTRUDERS_TEMP          0
#define     PAGENUM_SETTING_LEVELING                0
#define     PAGENUM_SETTING_TMC2130                 0
#define     PAGENUM_SETTING_MATERIAL                0
#define     PAGENUM_SETTING_SYSTEM                  0



#define     FYSTLCD_PAGE_EXIST(x)   (defined(PAGENUM_##x)&&PAGENUM_##x>0)
#define     FTPAGE(x)   PAGENUM_##x
