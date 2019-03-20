/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef _USBREADER_H_
#define _USBREADER_H_

#include "../MarlinConfig.h"

#if ENABLED(FYS_STORAGE_SUPPORT)

#define SD_RESORT ENABLED(SDCARD_SORT_ALPHA) && ENABLED(SDSORT_DYNAMIC_RAM)

#define MAX_DIR_DEPTH 10          // Maximum folder depth

#include "cardusbfile.h"
#include "../sd/SdFatConfig.h"

// 这些定义是从 sdfat.h 拷贝过来
// use the gnu style oflag in open()
/** open() oflag for reading */
uint8_t const O_READ = 0X01;
/** open() oflag - same as O_READ */
uint8_t const O_RDONLY = O_READ;
/** open() oflag for write */
uint8_t const O_WRITE = 0X02;
/** open() oflag - same as O_WRITE */
uint8_t const O_WRONLY = O_WRITE;
/** open() oflag for reading and writing */
uint8_t const O_RDWR = (O_READ | O_WRITE);
/** open() oflag mask for access modes */
uint8_t const O_ACCMODE = (O_READ | O_WRITE);
/** The file offset shall be set to the end of the file prior to each write. */
uint8_t const O_APPEND = 0X04;
/** synchronous writes - call sync() after each write */
uint8_t const O_SYNC = 0X08;
/** create the file if nonexistent */
uint8_t const O_CREAT = 0X10;
/** If O_CREAT and O_EXCL are set, open() shall fail if the file exists */
uint8_t const O_EXCL = 0X20;
/** truncate the file to zero length */
uint8_t const O_TRUNC = 0X40;

//#define USB_READER_DEBUG

class USBReader {
public:
  USBReader();

  void initsd();
  void write_command(char *buf);
  #ifdef FYS_ULTILCD2_COMPATIBLE
    bool write_string(char* buffer);
    FORCE_INLINE int16_t fgets(char* str, int16_t num) { return file.fgets(str, num, NULL); }
    //FORCE_INLINE int errorCode() { return sd2card.errorCode(); }
    FORCE_INLINE int errorCode() { return 0; } // 暂时先用这个
    FORCE_INLINE bool atRoot() { return workDirDepth == 0; }
    //FORCE_INLINE void clearError() { sd2card.error(0); }
    FORCE_INLINE void clearError() {  } // 暂时先用这个
    FORCE_INLINE uint32_t getFilePos() { return sdpos; }
    FORCE_INLINE uint32_t getFileSize() { return filesize; }
  #endif
  //files auto[0-9].g on the sd card are performed in a row
  //this is to delay autostart and hence the initialisaiton of the sd card to some seconds after the normal init, so the device is available quick after a reset
  
  void beginautostart();
  void checkautostart();

  void openFile(char * const path, const bool read, const bool subcall=false);
  void openLogFile(char * const path);
  void removeFile(const char * const name);
  void closefile(const bool store_location=false);
  void release();
  void openAndPrintFile(const char *name);
  void startFileprint();
  void stopSDPrint(
    #if SD_RESORT
      const bool re_sort=false
    #endif
  );
  void getStatus();
  void printingHasFinished();
  void printFilename();

  #if ENABLED(LONG_FILENAME_HOST_SUPPORT)
    void printLongPath(char *path);
  #endif

  void getfilename(uint16_t nr, const char* const match=NULL);
  void getLongnameFromShort();
  uint16_t getnrfilenames();

  void getAbsFilename(char *t);
  void getAbsDir(char *t,const char* const append=NULL);

  void ls();
  void chdir(const char *relpath);
  int8_t updir();
  int8_t isIndir();
  void setroot();

  const char* diveToFile(USBFile*& curDir, const char * const path, const bool echo);

  uint16_t get_num_Files();

  #if ENABLED(SDCARD_SORT_ALPHA)
    void presort();
    void getfilename_sorted(const uint16_t nr);
    #if ENABLED(SDSORT_GCODE)
      FORCE_INLINE void setSortOn(bool b) { sort_alpha = b; presort(); }
      FORCE_INLINE void setSortFolders(int i) { sort_folders = i; presort(); }
      //FORCE_INLINE void setSortReverse(bool b) { sort_reverse = b; }
    #endif
  #endif

  #if ENABLED(POWER_LOSS_RECOVERY)
    void openJobRecoveryFile(const bool read);
    void closeJobRecoveryFile();
    bool jobRecoverFileExists();
    int16_t saveJobRecoveryInfo();
    int16_t loadJobRecoveryInfo();
    void removeJobRecoveryFile();
  #endif

  FORCE_INLINE void pauseSDPrint() { sdprinting = false; }
  FORCE_INLINE bool isFileOpen() { return file.isOpen(); }
  #if ENABLED(FYS_PRINT_IMAGE_PREVIEW)
    FORCE_INLINE bool isImageFileOpen() { return imageFile.isOpen(); }
    FORCE_INLINE bool eofImageFile() { return imageSdpos >= imageFilesize; }
    FORCE_INLINE int16_t getImageChar() { imageSdpos = imageFile.curPosition(); return (int16_t)imageFile.read(); }
    void lsDiveIn(const char *prepend, SdFile parent, const char * const match=NULL);
    void getShortNameFromLong(const char * const longName);
    void openImageFile(char * const path, const bool read, const bool subcall=false);
  #endif
  FORCE_INLINE bool eof() { return sdpos >= filesize; }
  FORCE_INLINE int16_t get() { sdpos = file.curPosition(); return (int16_t)file.read(); }
  FORCE_INLINE void setIndex(const uint32_t index) { sdpos = index; file.seekSet(index); }
  FORCE_INLINE uint32_t getIndex() { return sdpos; }
  FORCE_INLINE uint8_t percentDone() { return (isFileOpen() && filesize) ? sdpos / ((filesize + 99) / 100) : 0; }
  FORCE_INLINE char* getWorkDirName() { workDir.getFilename(filename); return filename; }

  #if ENABLED(AUTO_REPORT_SD_STATUS)
    void auto_report_sd_status(void);
    FORCE_INLINE void set_auto_report_interval(uint8_t v) {
      NOMORE(v, 60);
      auto_report_sd_interval = v;
      next_sd_report_ms = millis() + 1000UL * v;
    }
  #endif

  FORCE_INLINE char* longest_filename() { return longFilename[0] ? longFilename : filename; }

public:
  bool saving, logging, sdprinting, cardOK, filenameIsDir;
  char filename[FILENAME_LENGTH], longFilename[LONG_FILENAME_LENGTH];
  char filenameorigin[FILENAME_LENGTH];
  #if ENABLED(FYS_PRINT_IMAGE_PREVIEW)
    char filenameImage[FILENAME_LENGTH], longFilenameImage[LONG_FILENAME_LENGTH];
    char absFilenameImage[50]; // geo-f : to record the IMAGE ABS path short name
  #endif
  int8_t autostart_index;
  EM_STORAGE_TYPE storageType;
private:
  //SdFile root, workDir, workDirParents[MAX_DIR_DEPTH];
  //uint8_t workDirDepth;
  // workDir为当前打开的相对目录，只会在 updir 和 chdir 函数中变更，初始为根目录
  // workDirParents 为当前打开的绝对目录，不包括根目录
  //char root[FILENAME_LENGTH], workDir[FILENAME_LENGTH], workDirParents[MAX_DIR_DEPTH][FILENAME_LENGTH];
  //uint8_t workDirDepth;
  // 
  USBFile root, workDir, workDirParents[MAX_DIR_DEPTH];
  uint8_t workDirDepth;

  // Sort files and folders alphabetically.
  #if ENABLED(SDCARD_SORT_ALPHA)
    uint16_t sort_count;        // Count of sorted items in the current directory
    #if ENABLED(SDSORT_GCODE)
      bool sort_alpha;          // Flag to enable / disable the feature
      int sort_folders;         // Flag to enable / disable folder sorting
      //bool sort_reverse;      // Flag to enable / disable reverse sorting
    #endif

    // By default the sort index is static
    #if ENABLED(SDSORT_DYNAMIC_RAM)
      uint8_t *sort_order;
    #else
      uint8_t sort_order[SDSORT_LIMIT];
    #endif

    #if ENABLED(SDSORT_USES_RAM) && ENABLED(SDSORT_CACHE_NAMES) && DISABLED(SDSORT_DYNAMIC_RAM)
      #define SORTED_LONGNAME_MAXLEN ((SDSORT_CACHE_VFATS) * (FILENAME_LENGTH) + 1)
    #else
      #define SORTED_LONGNAME_MAXLEN LONG_FILENAME_LENGTH
    #endif

    // Cache filenames to speed up SD menus.
    #if ENABLED(SDSORT_USES_RAM)

      // If using dynamic ram for names, allocate on the heap.
      #if ENABLED(SDSORT_CACHE_NAMES)
        #if ENABLED(SDSORT_DYNAMIC_RAM)
          char **sortshort, **sortnames;
        #else
          char sortshort[SDSORT_LIMIT][FILENAME_LENGTH];
          char sortnames[SDSORT_LIMIT][SORTED_LONGNAME_MAXLEN];
        #endif
      #elif DISABLED(SDSORT_USES_STACK)
        char sortnames[SDSORT_LIMIT][SORTED_LONGNAME_MAXLEN];
      #endif

      // Folder sorting uses an isDir array when caching items.
      #if HAS_FOLDER_SORTING
        #if ENABLED(SDSORT_DYNAMIC_RAM)
          uint8_t *isDir;
        #elif ENABLED(SDSORT_CACHE_NAMES) || DISABLED(SDSORT_USES_STACK)
          uint8_t isDir[(SDSORT_LIMIT+7)>>3];
        #endif
      #endif

    #endif // SDSORT_USES_RAM

  #endif // SDCARD_SORT_ALPHA

  //Sd2Card sd2card;
  //SdVolume volume;
  //SdFile file;
  USBFile file; // 用于保存当前正在打印的文件

  #if ENABLED(POWER_LOSS_RECOVERY)
    USBFile jobRecoveryFile;
    uint32_t jobRecoveryFilesize, jobRecoveryFilesdpos;
  #endif

  #if ENABLED(FYS_PRINT_IMAGE_PREVIEW)
    USBFile imageFile;
  #endif

  // 以下变量主要用于分支打印，即在打印一个文件的中途去调用另一个文件进行打印
  #define SD_PROCEDURE_DEPTH 1
  #define MAXPATHNAMELENGTH (FILENAME_LENGTH*MAX_DIR_DEPTH + MAX_DIR_DEPTH + 1)
  uint8_t file_subcall_ctr;
  uint32_t filespos[SD_PROCEDURE_DEPTH];
  char proc_filenames[SD_PROCEDURE_DEPTH][MAXPATHNAMELENGTH];
  uint32_t filesize, sdpos;

  #if ENABLED(FYS_PRINT_IMAGE_PREVIEW)
    uint32_t imageFilesize, imageSdpos;
  #endif

  LsAction lsAction; //stored for recursion.
  uint16_t nrFiles; //counter for the files in the current directory and recycled as position counter for getting the nrFiles'th name in the directory.
  char* diveDirName;
  void lsDive(const char *prepend, USBFile parent, const char * const match=NULL);
  UINT8	LsDive( UINT8 index , const char * const match=NULL);

  #if ENABLED(SDCARD_SORT_ALPHA)
    void flush_presort();
  #endif

  #if ENABLED(AUTO_REPORT_SD_STATUS)
    static uint8_t auto_report_sd_interval;
    static millis_t next_sd_report_ms;
  #endif
};

/*
#if PIN_EXISTS(SD_DETECT)
  #if ENABLED(SD_DETECT_INVERTED)
    #define IS_SD_INSERTED (READ(SD_DETECT_PIN) == HIGH)
  #else
    #define IS_SD_INSERTED (READ(SD_DETECT_PIN) == LOW)
  #endif
#else
  // No card detect line? Assume the card is inserted.
  #define IS_SD_INSERTED true
#endif
*/

//extern CardReader card;
extern USBReader card;

#endif // FYS_STORAGE_SUPPORT

#if ENABLED(SDSUPPORT)
  #define IS_SD_PRINTING (card.sdprinting)
  #define IS_SD_FILE_OPEN (card.isFileOpen())
#elif ENABLED(FYS_STORAGE_SUPPORT)
  #define IS_SD_PRINTING (card.sdprinting)
  #define IS_SD_FILE_OPEN (card.isFileOpen())
#else
  #define IS_SD_PRINTING (false)
  #define IS_SD_FILE_OPEN (false)
#endif

#endif // _CARDREADER_H_

