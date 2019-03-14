#ifndef _USBFILE_H_
#define _USBFILE_H_

#include <stdint.h>
#include "CH376_hal.h"

class USBFile {
public:
  USBFile();
  //USBFile(const char* path, uint8_t oflag);
  ~USBFile() { }
  void init();
  void rewind(); 
  void getFilename(char * const filename);
  void setFilename(char * const filename);
  UINT32 getDirStartClust();
  void setDirStartClust(UINT32 clust);
  UINT8 getAttr();
  void setAttr(UINT8 at);
  bool isDir();
  UINT32 fileSize();
  UINT32 curPosition();
  bool seekSet(UINT32 pos);
  bool open(USBFile* dirFile, const char* path, uint8_t oflag);
  bool isOpen();
  void setFileOpenState(bool flag);
  INT16 read();
  INT16 read(void* buf, uint16_t nbyte);
  INT16 fgets(char* str, INT16 num, char* delim);
  void write(UINT8 *pData);
  int16_t write(const void* buf, uint16_t nbyte);
  bool close();
  int8_t readDir(FAT_DIR_INFO* dir, char* longFilename);
  bool remove(USBFile* dirFile, const char* path);

public :
  bool writeError;
  
private:
  UINT32 dirStartClust;  /* 文件所在目录的起始簇号 */
  UINT32 size;         /* 文件长度 */
  UINT32 curPosition_;  //当前文件的位置
  UINT8 name[8+1+3+1];    /* 文件名,共8+3字节,分隔符,结束符,因为未包含上级目录名所以是相对路径 */
  UINT8 attr;    
  bool is_open; // 标记文件是否打开  
};

#endif
