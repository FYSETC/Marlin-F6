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

#include "../MarlinConfig.h"

#if ENABLED(FYS_STORAGE_SUPPORT)

#include "cardusbdiskreader.h"

//#include "Marlin.h"
#include "../ultralcd.h"
#include "../stepper.h"
#include "../language.h"
#include "../printcounter.h"

#include "CH376_hal.h"
#include "CH376_file_sys.h"
#include "CH376_debug.h"

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "power_loss_recovery.h"
#endif

USBReader::USBReader() {
  #if ENABLED(SDCARD_SORT_ALPHA)
    sort_count = 0;
    #if ENABLED(SDSORT_GCODE)
      sort_alpha = true;
      sort_folders = FOLDER_SORTING;
      //sort_reverse = false;
    #endif
  #endif
  sdprinting = cardOK = saving = logging = false;
  filesize = 0;
  sdpos = 0;
  file_subcall_ctr = 0;

  #if ENABLED(FYS_PRINT_IMAGE_PREVIEW)
    imageFilesize=0; 
    imageSdpos=0;
  #endif

  workDirDepth = 0;
  ZERO(workDirParents);

  // Disable autostart until card is initialized
  autostart_index = -1;

  //power to SD reader
  #if SDPOWER > -1
    OUT_WRITE(SDPOWER, HIGH);
  #endif

  // storage type
  #ifdef FYS_STORAGE_SDCARD
    storageType = EMST_SDCARD;
  #endif
  #ifdef FYS_STORAGE_USBMODE
  storageType = EMST_USB_DISK;
  #endif
}

// 一般目录读出来的是不带.和结束符0的字符数组，所以要转为8.3格式的字串
char *createFilename(char *buffer, char *pDirName) { //buffer > 12characters
	char *pNameBuf = buffer;
	for ( UINT8 i= 0; i < 11; i ++ ) {  /* 复制文件名,长度为11个字符 */
		if ( pDirName[i] != 0x20 ) {  /* 有效字符 */ // 跳过空格
			if ( i == 8 ) {  /* 处理扩展名 */
				*pNameBuf = '.';  /* 分隔符 */
				pNameBuf ++;
			}
			*pNameBuf = pDirName[ i ];  /* 复制文件名的一个字符 */
			pNameBuf ++;
		}
	}
	*pNameBuf = 0;  /* 当前文件名完整路径的结束符 */

  return buffer;
}


#if ENABLED(FYS_PRINT_IMAGE_PREVIEW)

  /**
   * Dive into a folder and recurse depth-first to perform a pre-set operation lsAction:
   *   LS_Count       - Add +1 to nrFiles for every file within the parent
   *   LS_GetFilename - Get the filename of the file indexed by nrFile_index
   */
  
  void USBReader::lsDiveIn(const char *prepend, SdFile parent, const char * const match/*=NULL*/) {
    dir_t p;
    uint8_t cnt = 0;
  
    // Read the next entry from a directory
    while (parent.readDir(&p, longFilenameImage) > 0) {
  
      // If the entry is a directory and the action is LS_SerialPrint
      //if (DIR_IS_SUBDIR(&p) && lsAction != LS_Count && lsAction != LS_GetFilename) {
      if (DIR_IS_SUBDIR(&p)) { 
  
        // Get the short name for the item, which we know is a folder
        char dosFilename[FILENAME_LENGTH];
        createFilename(dosFilename, p);
  
        // Allocate enough stack space for the full path to a folder, trailing slash, and nul
        const bool prepend_is_empty = (!prepend || prepend[0] == '\0');
        const int len = (prepend_is_empty ? 1 : strlen(prepend)) + strlen(dosFilename) + 1 + 1;
        char path[len];
  
        // Append the FOLDERNAME12/ to the passed string.
        // It contains the full path to the "parent" argument.
        // We now have the full path to the item in this folder.
        strcpy(path, prepend_is_empty ? "/" : prepend); // root slash if prepend is empty
        strcat(path, dosFilename); // FILENAME_LENGTH-1 characters maximum
        strcat(path, "/");       // 1 character
  
        // Serial.print(path);
  
        // Get a new directory object using the full path
        // and dive recursively into it.
        SdFile dir;
        if (!dir.open(&parent, dosFilename, O_READ)) {
        }
        lsDiveIn(path, dir,match);
        // close() is done automatically by destructor of SdFile
      }
      else {
        uint8_t pn0 = p.name[0];
        if (pn0 == DIR_NAME_FREE) break;
        if (pn0 == DIR_NAME_DELETED || pn0 == '.') continue;
        if (longFilename[0] == '.') continue;
  
        if (!DIR_IS_FILE_OR_SUBDIR(&p) || (p.attributes & DIR_ATT_HIDDEN)) continue;
  
        filenameIsDir = DIR_IS_SUBDIR(&p);
  
        //if (!filenameIsDir && (/*p.name[8] != 'J' ||*/ p.name[9] == '~')) continue;
  
        switch (lsAction) {  // 1 based file count
          case LS_Count:
            nrFiles++;
            break;
  
          case LS_GetFilename:
          	//SERIAL_PROTOCOL("24\n");
    				// get the longname absolute dir
            char path[80];
    				sprintf_P(path,PSTR("%s%s"),prepend,longFilenameImage);
            SERIAL_ECHOLNPAIR("match:",match);
            SERIAL_ECHOLNPAIR("path:",path);
            createFilename(filenameImage, p);
            SERIAL_ECHOLNPAIR("filename:",filenameImage);
    				if (NULL != match ) {
    					if (strcasecmp(match, filenameImage) == 0 || strcasecmp(match, longFilenameImage) == 0) {
                //createFilename(filename, p);
    						sprintf_P(absFilenameImage,PSTR("%s%s"),prepend,filenameImage);
    						strlwr(absFilenameImage);
                SERIAL_ECHOLNPAIR("absFilenameImage",absFilenameImage);
    						return;
    					}
    				}
    				break;
        }
  
      }
    } // while readDir
  }

  // the short name will be stored in shortFileName[50]
  void USBReader::getShortNameFromLong(const char * const longName) {
  	lsAction=LS_GetFilename;
  	root.rewind();
  	lsDiveIn("",root,longName);
  }

  void USBReader::openImageFile(char * const path, const bool read, const bool subcall/*=false*/) {
  
    if (!cardOK) return;
  
    if (isImageFileOpen()) {
      imageFile.close();
    }

    uint32_t imageFileSize;
  
    SdFile *curDir;
    const char * const fname = diveToFile(curDir, path, false);
    if (!fname) return;
  
    if (read) {
      if (imageFile.open(curDir, fname, O_READ)) {
        imageFilesize = imageFile.fileSize();
        SERIAL_PROTOCOLPAIR(MSG_SD_FILE_OPENED, fname);
        SERIAL_PROTOCOLLNPAIR(MSG_SD_SIZE, imageFilesize);
        SERIAL_PROTOCOLLNPGM(MSG_SD_FILE_SELECTED);
  
        //if (longFilename[0]) {
        //  SERIAL_PROTOCOLPAIR(MSG_SD_FILE_LONG_NAME, longFilename);
        //}
      }
      else {
        SERIAL_PROTOCOLPAIR(MSG_SD_OPEN_FILE_FAIL, fname);
        SERIAL_PROTOCOLCHAR('.');
        SERIAL_EOL();
      }
    }
  }

#endif

// 现在的LsDive方法需要保存临时数据，暂定最大为40个目录，后续进行优化
// 而且暂时不支持 LS_SerialPrint
typedef struct _LS_RECORD {
  UINT32  DirStartClust;  /* 文件所在目录的起始簇号 */
//  UINT32  Size;         /* 文件长度 */
  UINT8 Name[8+1+3+1];    /* 文件名,共8+3字节,分隔符,结束符,因为未包含上级目录名所以是相对路径 */
  UINT8 Attr;             /* 文件属性 */
} LS_RECORD;

#define	MAX_FILE_COUNT		40 // 暂定最大为40个目录，后续进行优化
LS_RECORD  DirStruct[ MAX_FILE_COUNT ]; /* 仅保存目录结构 */
UINT16  fileCnt=0;
UINT16  nrFile_index=0;
UINT16  dirCnt=0;

/* 例子:列举指定序号的目录下的所有文件 */
// 输入参数index是指目录在DirStruct结构中的序号
// 如果是获取 nr 则包括了 当前目录，而不是只包括目录下的项目
UINT8	USBReader::LsDive( UINT8 index , const char * const match/*=NULL*/){		
  UINT8			s;
  P_FAT_DIR_INFO  pDir;
  UINT32	CurrentDirStartClust;  /* 保存当前目录的起始簇号,用于加快文件枚举和打开速度 */
  UINT8   buf[64];

  //
  #ifdef USB_READER_DEBUG
    SERIAL_ECHO("index num:");
    SERIAL_PRINTLN(index,10);
    SERIAL_ECHO("workDir.DirStartClust:");
    SERIAL_ECHOLN(DirStruct[ index ].DirStartClust);
  #endif
  
  // 将当前目录的上级目录的起始簇号设置为当前簇号,相当于打开上级目录
  // 在同一个目录中的所有文件都保有着目录起始簇号
  /*
  if(DirStruct[ index ].DirStartClust==0) {
  	CH376WriteVar32( VAR_START_CLUSTER, DirStruct[ index ].DirStartClust );

    char * const p = &DirStruct[ index ].Name[0];
  	#ifdef USB_READER_DEBUG
  	  SERIAL_ECHO("List Directory: ");	 
      CH376DebugOut( p );  // 显示当前要列举的目录名
    #endif

    // 打开目录,仅为了获取目录的起始簇号以提高速度
  //	s = CH376FileOpen( DirStruct[ index ].Name );
  	s = CH376FileOpen( p );
  	if ( s == USB_INT_SUCCESS ) return( ERR_FOUND_NAME );  // 应该是打开了目录,但是返回结果是打开了文件 
  	else if ( s != ERR_OPEN_DIR ) return( s );
  	CurrentDirStartClust = CH376ReadVar32( VAR_START_CLUSTER );  // 获取目录的起始簇号 
  	CH376FileClose( FALSE );  // 对于根目录一定要关闭

    #ifdef USB_READER_DEBUG
      SERIAL_ECHO("clust num:");
      SERIAL_ECHO(DirStruct[ index ].DirStartClust);
      SERIAL_ECHO(" root clust:");
      SERIAL_ECHO(root.getDirStartClust());
      SERIAL_ECHO("-");
      SERIAL_ECHOLN(CurrentDirStartClust);
    #endif
  }
  else {
    CurrentDirStartClust = DirStruct[ index ].DirStartClust;
  }
  */
  
  // 使用通配符开始枚举
	//CH376WriteVar32( VAR_START_CLUSTER, CurrentDirStartClust );  /* 当前目录的起始簇号,相当于打开当前目录 */
	CH376WriteVar32( VAR_START_CLUSTER, DirStruct[ index ].DirStartClust );  /* 当前目录的起始簇号,相当于打开当前目录 */
	CH376SetFileName( "*" );  /* 设置将要操作的文件的文件名,通配符支持所有文件和子目录 */
	xWriteCH376Cmd( CMD0H_FILE_OPEN );  /* 枚举文件和目录 */
	xEndCH376Cmd( );

  // 第一个目录跳过不处理
	s = Wait376Interrupt( );
	if ( s == USB_INT_DISK_READ ) {  /* 请求数据读出 */
    xWriteCH376Cmd( CMD0H_FILE_ENUM_GO );  /* 继续枚举文件和目录,先发出下一个命令再分析上行读出的数据可以让CH376与单片机分别同时工作,提高速度 */
	  xEndCH376Cmd( );
	}
  else { // 此目录已经枚举完
		if ( s == ERR_MISS_FILE ) { /* 没有找到更多的匹配文件 */
      s = USB_INT_SUCCESS;   
      #ifdef USB_READER_DEBUG
        CH376DebugOut( "List Directory END" );
      #endif
    }
		return s;
	}

	#ifdef USB_READER_DEBUG
    SERIAL_ECHO(" root clust:");
    SERIAL_ECHO(root.getDirStartClust());
  #endif
  
	while ( 1 ) {
		s = Wait376Interrupt( );
		if ( s == USB_INT_DISK_READ ) {  /* 请求数据读出 */
      /* 在文件枚举过程中,不能执行其它可能产生中断的操作命令,
      例如,如果需要获取长文件名,那么可以将枚举出的文件名保存并在枚举结束后获取其长文件名 */

      /* 读取枚举到的文件的FAT_DIR_INFO结构,返回长度总是sizeof( FAT_DIR_INFO ) */
			CH376ReadBlock( buf );  
			//xWriteCH376Cmd( CMD0H_FILE_ENUM_GO );  /* 继续枚举文件和目录,先发出下一个命令再分析上行读出的数据可以让CH376与单片机分别同时工作,提高速度 */
			//xEndCH376Cmd( );
      
			pDir = (P_FAT_DIR_INFO)buf;  /* 当前文件目录信息 */
			if ( pDir->DIR_Name[0] != '.' ) {  /* 不是本级或者上级目录名则继续,否则必须丢弃不处理 */
				if ( pDir->DIR_Name[0] == 0x05 ) pDir->DIR_Name[0] = 0xE5;  /* 特殊字符替换 */
        
        // 文件名结构缓冲区必须足够，如果不够 我们再想想还有什么办法可以去掉这个，看能不能所有目录都是从头搜索的
				if ( dirCnt < MAX_FILE_COUNT ) {  

          filenameIsDir = (pDir->DIR_Attr & ATTR_DIRECTORY)?true:false;

          // 如果是目录名，我们需要记录起始簇号等
					if ( filenameIsDir ) { 
          	#ifdef USB_READER_DEBUG
              SERIAL_ECHO(" root clust:");
              SERIAL_ECHO(root.getDirStartClust());
            #endif

					  // geo-f:20190228,我们由于只进行一层目录的枚举，下面三个就不需要记录了
            //createFilename(DirStruct[dirCnt].Name, pDir->DIR_Name);
  					//DirStruct[dirCnt].DirStartClust = CurrentDirStartClust;  /* 记录当前目录的起始簇号,用于加快文件打开速度 */
	  				//DirStruct[dirCnt].Attr = pDir->DIR_Attr;  /* 记录文件属性 */

            #ifdef USB_READER_DEBUG
              SERIAL_ECHO(" root clust:");
              SERIAL_ECHO(root.getDirStartClust());
            #endif
            
            // 子目录计数
            dirCnt ++;
            #ifdef USB_READER_DEBUG
              SERIAL_ECHO("dirCnt:");
              SERIAL_PRINTLN(dirCnt,10);
            #endif
          }

          #ifdef USB_READER_DEBUG
            CH376DebugOut( pDir->DIR_Name );
          #endif

          // 如果不是 gcode 文件，我们直接继续枚举
          if (!filenameIsDir && (pDir->DIR_Name[8] != 'G' || pDir->DIR_Name[9] == '~')) {
       			xWriteCH376Cmd( CMD0H_FILE_ENUM_GO );  /* 继续枚举文件和目录,先发出下一个命令再分析上行读出的数据可以让CH376与单片机分别同时工作,提高速度 */
      			xEndCH376Cmd( );
            continue;
          }

          #ifdef USB_READER_DEBUG
            SERIAL_ECHO(" root clust:");
            SERIAL_ECHO(root.getDirStartClust());
          #endif

          switch (lsAction) {  // 1 based file count
            case LS_Count:
              nrFiles++;
              #ifdef USB_READER_DEBUG
                SERIAL_ECHOLNPAIR("nrFiles:",nrFiles);
              #endif
              break;
/*
            case LS_SerialPrint:
              createFilename(filename, p);
              if (prepend) SERIAL_PROTOCOL(prepend);
              SERIAL_PROTOCOL(filename);
              SERIAL_PROTOCOLCHAR(' ');
              SERIAL_PROTOCOLLN(p.fileSize);
              break;
*/
            case LS_GetFilename:
              #ifdef USB_READER_DEBUG
                SERIAL_ECHO(" rootX clust:");
                SERIAL_ECHOLN(root.getDirStartClust());
              #endif
              memset(filenameorigin,0,sizeof(filenameorigin));
              strncpy(filenameorigin,pDir->DIR_Name,11);
              #ifdef USB_READER_DEBUG
                SERIAL_ECHO(" rootX clust:");
                SERIAL_ECHOLN(root.getDirStartClust());
              #endif
              //strcpy(filenameorigin,pDir->DIR_Name);
              createFilename(filename, pDir->DIR_Name);
              #ifdef USB_READER_DEBUG
                SERIAL_ECHOLNPAIR("fileCnt:",fileCnt);
                SERIAL_ECHOLNPAIR("filename:",filename);
                SERIAL_ECHO(" rootX clust:");
                SERIAL_ECHOLN(root.getDirStartClust());
              #endif
              if (match != NULL) {
                if (strcasecmp(match, filename) == 0) return;
              }
              else if (fileCnt == nrFile_index) {
                #ifdef USB_READER_DEBUG
                  SERIAL_ECHO(" rootX clust:");
                  SERIAL_ECHO(root.getDirStartClust());
                  SERIAL_ECHOLN("return");                  
                #endif
                dirCnt = 0; // 不再循环
                return USB_INT_SUCCESS;  // 0 based index                
              }
              
              fileCnt++;
              break;
          }
				}
				else {  /* 文件名结构缓冲区太小,结构数量不足 */
				  #ifdef USB_READER_DEBUG
					  CH376DebugOut( "USB dir Structure Full" );
					#endif
					s = Wait376Interrupt( );
					CH376EndDirInfo( );  /* 获取完FAT_DIR_INFO结构 */
					break;  /* 强行终止枚举 */
				}
			}

			xWriteCH376Cmd( CMD0H_FILE_ENUM_GO );  /* 继续枚举文件和目录,先发出下一个命令再分析上行读出的数据可以让CH376与单片机分别同时工作,提高速度 */
			xEndCH376Cmd( );
		}
		else { // 此目录已经枚举完
			if ( s == ERR_MISS_FILE ) { /* 没有找到更多的匹配文件 */
        s = USB_INT_SUCCESS;   
        #ifdef USB_READER_DEBUG
          CH376DebugOut( "List Directory END" );
        #endif
      }
			break;
		}
	}
  /*	if ( s == USB_INT_SUCCESS ) return( s );*/  /* 操作成功 */
	return( s );
}

// 这个还没有完善，暂时先不提供，此函数只有 M20 用到
void	USBReader::ls( void ){
/*
	UINT8	s;
	UINT16	DirCntOld;

  lsAction = LS_SerialPrint;

  // 从根目录开始找起
  memset(DirStruct,0,sizeof(LS_RECORD)*MAX_FILE_COUNT);
	DirStruct[ 0 ].Name[0] = '/';  // 根目录,是完整路径名,除根目录是绝对路径之外都是相对路径 
	DirStruct[ 0 ].Name[1] = 0;
	DirStruct[ 0 ].DirStartClust = 0;  // 根目录的起始簇号 
	DirStruct[ 0 ].Attr = ATTR_DIRECTORY;  // 根目录也是目录,作为第一个记录保存

  // 枚举目录,记录保存到结构中,DirCnt会在枚举到为目录时增加
	for ( DirCntOld = 0, dirCnt = 1; DirCntOld < dirCnt; DirCntOld ++ ) {  
		s = LsDive( DirCntOld );  
		if ( s != USB_INT_SUCCESS ) mStopIfError( s );
	}

	CH376FileClose( FALSE );  // 关闭 
*/
}


#if ENABLED(LONG_FILENAME_HOST_SUPPORT)

  /**
   * Get a long pretty path based on a DOS 8.3 path
   */
  void USBReader::printLongPath(char *path) {
    lsAction = LS_GetFilename;

    int i, pathLen = strlen(path);

    // SERIAL_ECHOPGM("Full Path: "); SERIAL_ECHOLN(path);

    // Zero out slashes to make segments
    for (i = 0; i < pathLen; i++) if (path[i] == '/') path[i] = '\0';

    SdFile diveDir = root; // start from the root for segment 1
    for (i = 0; i < pathLen;) {

      if (path[i] == '\0') i++; // move past a single nul

      char *segment = &path[i]; // The segment after most slashes

      // If a segment is empty (extra-slash) then exit
      if (!*segment) break;

      // Go to the next segment
      while (path[++i]) { }

      // SERIAL_ECHOPGM("Looking for segment: "); SERIAL_ECHOLN(segment);

      // Find the item, setting the long filename
      diveDir.rewind();
      lsDive(NULL, diveDir, segment);

      // Print /LongNamePart to serial output
      SERIAL_PROTOCOLCHAR('/');
      SERIAL_PROTOCOL(longFilename[0] ? longFilename : "???");

      // If the filename was printed then that's it
      if (!filenameIsDir) break;

      // SERIAL_ECHOPGM("Opening dir: "); SERIAL_ECHOLN(segment);

      // Open the sub-item as the new dive parent
      SdFile dir;
      if (!dir.open(&diveDir, segment, O_READ)) {
        SERIAL_EOL();
        SERIAL_ECHO_START();
        SERIAL_ECHOPGM(MSG_SD_CANT_OPEN_SUBDIR);
        SERIAL_ECHO(segment);
        break;
      }

      diveDir.close();
      diveDir = dir;

    } // while i<pathLen

    SERIAL_EOL();
  }

#endif // LONG_FILENAME_HOST_SUPPORT

/**
 * Echo the DOS 8.3 filename (and long filename, if any)
 */
void USBReader::printFilename() {
  if (file.isOpen()) {
    char dosFilename[FILENAME_LENGTH];
    file.getFilename(dosFilename);
    SERIAL_ECHO(dosFilename);
    #if ENABLED(LONG_FILENAME_HOST_SUPPORT)
      getfilename(0, dosFilename);
      if (longFilename[0]) {
        SERIAL_ECHO(' ');
        SERIAL_ECHO(longFilename);
      }
    #endif
  }
  else
    SERIAL_ECHOPGM("(no file)");

  SERIAL_EOL();
}

void USBReader::initsd() {
  // 初始化 USB
  UINT8 i = 0,s;
  
  cardOK = false;

  // 底层驱动的初始化，即sd2card初始化，usb设备则是 CH376等设备的初始化
  if(ERR_USB_UNKNOWN==CH376_init(storageType)) {
    SERIAL_ECHOLN("USB init fail");
    return;
  }

  CH376Delayms( 100 );
  
  // root 这个根目录文件 SdFile 的初始化，包括卷积 volume 的初始化
  // 层级是这样的 SdFile -> SdBaseFile -> volume -> sd2card
  // 其中 SdFile 只是 SdBaseFile 加入一些 print 功能，这两个为一层
  // volume 层 保存了一个 sd2card 的指针 用于具体的 硬件操作
  // 我们也创建一个 UsbFile 的类用来操作
  // 这个也相当于 CH376_file_sys 了

  // 我们需要去将 USB 挂载起来
  // 检查U盘是否连接,等待U盘插入,对于SD卡,可以由单片机直接查询SD卡座的插拔状态引脚
  //#ifdef FYS_STORAGE_USBMODE
  if(storageType == EMST_USB_DISK) {
  	SERIAL_ECHOLN("USB connecting");
  	while ( CH376DiskConnect( ) != USB_INT_SUCCESS && i<3 ) {  // 等待0.15秒钟
  		CH376Delayms( 50 );
  		i++;
  		SERIAL_ECHOLN("USB connecting");
  	}
  	if(i==3) {
  	  SERIAL_ECHOLN("USB connect fail");
  	  return;
  	}
  }
	//#endif	

  // 延时,可选操作,有的USB存储器需要几十毫秒的延时
  CH376Delayms( 200 );

  for ( i = 0; i < 100; i ++ ) {  /* 最长等待时间,100*50mS */
		CH376Delayms( 50 );
		SERIAL_ECHOLN( "USB Mouting" );
		s = CH376DiskMount( );  /* 初始化磁盘并测试磁盘是否就绪 */
		//SERIAL_ECHOLNPAIR("s:", s);
		if ( s == USB_INT_SUCCESS ) break;  /* 准备好 */
		else if ( s == ERR_DISK_DISCON ) break;  /* 检测到断开,重新检测并计时 */
		if ( CH376GetDiskStatus( ) >= DEF_DISK_MOUNTED && i >= 5 ) {
		  break;  /* 有的U盘总是返回未准备好,不过可以忽略,只要其建立连接MOUNTED且尝试5*50mS */
		}
    //UINT8 st = CH376GetDiskStatus();
    //SERIAL_ECHOLNPAIR("st:", st);
		//if ( st >= DEF_DISK_MOUNTED && i >= 5 ) break;
  }

  /* 检测到断开,重新检测并计时 */
  if ( s == ERR_DISK_DISCON ) {  
    SERIAL_ECHOLN( "USB Device gone\n" );
    return ;
  }

  /* 未知USB设备,例如USB键盘、打印机等 */
	if ( CH376GetDiskStatus( ) < DEF_DISK_MOUNTED ) {  
		SERIAL_ECHOLN( "Unknown USB device\n" );
		//goto UnknownUsbDevice;
		return;
	}

  cardOK = true;
  SERIAL_ECHO_START();
  SERIAL_ECHOLNPGM(MSG_SD_CARD_OK);
}

void USBReader::release() {
  sdprinting = false;
  cardOK = false;
}

void USBReader::openAndPrintFile(const char *name) {
  char cmd[4 + strlen(name) + 1]; // Room for "M23 ", filename, and null
  sprintf_P(cmd, PSTR("M23 %s"), name);
  for (char *c = &cmd[4]; *c; c++) *c = tolower(*c);
  enqueue_and_echo_command_now(cmd);
  enqueue_and_echo_commands_P(PSTR("M24"));
}

void USBReader::startFileprint() {
  if (cardOK) {
    sdprinting = true;
    #if SD_RESORT
      flush_presort();
    #endif
  }
}

void USBReader::stopSDPrint(
  #if SD_RESORT
    const bool re_sort/*=false*/
  #endif
) {
  #if ENABLED(ADVANCED_PAUSE_FEATURE)
    did_pause_print = 0;
  #endif
  sdprinting = false;
  
  if (isFileOpen()) file.close();
  file.init(); // geo-f:add 20190228
  #if SD_RESORT
    if (re_sort) presort();
  #endif
}

void USBReader::openLogFile(char * const path) {
  logging = true;
  openFile(path, false);
}

// 获取 SdFile 中的文件名放入 dst 指向的位置，并在 dst 末尾放入 ‘/’
void appendAtom(USBFile &file, char *& dst, uint8_t &cnt) {
  file.getFilename(dst);
  while (*dst && cnt < MAXPATHNAMELENGTH) { dst++; cnt++; }
  if (cnt < MAXPATHNAMELENGTH) { *dst = '/'; dst++; cnt++; }
}

// 这里只有从 lcd 中打开了某一个文件之后，所有的路径才会有赋值，才能获取到文件的绝对路径
void USBReader::getAbsFilename(char *t) {
  *t++ = '/';                                               // Root folder // workDirParents不包括 根目录所以加入'/'
  uint8_t cnt = 1;

  for (uint8_t i = 0; i < workDirDepth; i++)                // Loop to current work dir
    appendAtom(workDirParents[i], t, cnt);

  if (cnt < MAXPATHNAMELENGTH - (FILENAME_LENGTH)) {
    appendAtom(file, t, cnt);
    --t;
  }
  *t = '\0';
}

void USBReader::getAbsDir(char *t,const char* const append/*=NULL*/) {
  *t++ = '/';                                               // Root folder // workDirParents不包括 根目录所以加入'/'
  uint8_t cnt = 1;

  for (uint8_t i = 0; i < workDirDepth; i++)                // Loop to current work dir
    appendAtom(workDirParents[i], t, cnt);

  if(append!=NULL) {
    char *app = (char *)append;
    if (cnt < MAXPATHNAMELENGTH - (FILENAME_LENGTH)) {
      while (*app && cnt < MAXPATHNAMELENGTH) { *t++ = *app++; cnt++; }
    }
  }

  *t = '\0';
}


void USBReader::openFile(char * const path, const bool read, const bool subcall/*=false*/) {

  if (!cardOK) return;

  uint8_t doing = 0;
  if (isFileOpen()) {                     // Replacing current file or doing a subroutine
    if (subcall) {
      if (file_subcall_ctr > SD_PROCEDURE_DEPTH - 1) {
        SERIAL_ERROR_START();
        SERIAL_ERRORPGM("trying to call sub-gcode files with too many levels. MAX level is:");
        SERIAL_ERRORLN((int)SD_PROCEDURE_DEPTH);
        kill(PSTR(MSG_KILLED));
        return;
      }

      // Store current filename (based on workDirParents) and position
      getAbsFilename(proc_filenames[file_subcall_ctr]);
      filespos[file_subcall_ctr] = sdpos;

      SERIAL_ECHO_START();
      SERIAL_ECHOPAIR("SUBROUTINE CALL target:\"", path);
      SERIAL_ECHOPAIR("\" parent:\"", proc_filenames[file_subcall_ctr]);
      SERIAL_ECHOLNPAIR("\" pos", sdpos);
      file_subcall_ctr++;
    }
    else
      doing = 1;
  }
  else if (subcall) {     // Returning from a subcall?
    SERIAL_ECHO_START();
    SERIAL_ECHOLNPGM("END SUBROUTINE");
  }
  else {                  // Opening fresh file
    doing = 2;
    file_subcall_ctr = 0; // Reset procedure depth in case user cancels print while in procedure
  }

  if (doing) {
    SERIAL_ECHO_START();
    SERIAL_ECHOPGM("Now ");
    serialprintPGM(doing == 1 ? PSTR("doing") : PSTR("fresh"));
    SERIAL_ECHOLNPAIR(" file: ", path);
  }

  stopSDPrint();

  // 通过路径获取到文件所属的目录 curDir，并返回文件名
  // 相当于 CH376FileOpenDir 功能，不过，在文件打开的过程中，获取了一些文件的属性并记录了
  // 而 CH376FileOpenDir 只是单纯的去打开目录了
  USBFile *curDir;
  const char * const fname = diveToFile(curDir, path, false);
  if (!fname) return;

  // 最终目标是为了打开最终的文件
  if (read) {
    if (file.open(curDir, fname, O_READ)) {
      filesize = file.fileSize();
      sdpos = 0;
      SERIAL_PROTOCOLPAIR(MSG_SD_FILE_OPENED, fname);
      SERIAL_PROTOCOLLNPAIR(MSG_SD_SIZE, filesize);
      SERIAL_PROTOCOLLNPGM(MSG_SD_FILE_SELECTED);

      // USB 暂时没有长名
      //getfilename(0, fname);
      //lcd_setstatus(longFilename[0] ? longFilename : fname);
      lcd_setstatus(fname);
      
      //if (longFilename[0]) {
      //  SERIAL_PROTOCOLPAIR(MSG_SD_FILE_LONG_NAME, longFilename);
      //}
    }
    else {
      SERIAL_PROTOCOLPAIR(MSG_SD_OPEN_FILE_FAIL, fname);
      SERIAL_PROTOCOLCHAR('.');
      SERIAL_EOL();
    }
	  #ifdef FYS_START_UP_100_FEEDRATE
    feedrate_percentage = 100;
    #endif
  }
  else { //write
    if (!file.open(curDir, fname, O_CREAT | O_APPEND | O_WRITE | O_TRUNC)) {
      SERIAL_PROTOCOLPAIR(MSG_SD_OPEN_FILE_FAIL, fname);
      SERIAL_PROTOCOLCHAR('.');
      SERIAL_EOL();
    }
    else {
      saving = true;
      SERIAL_PROTOCOLLNPAIR(MSG_SD_WRITE_TO_FILE, path);
      lcd_setstatus(fname);
    }
  }
}

void USBReader::removeFile(const char * const name) {
  if (!cardOK) return;

  stopSDPrint();

  USBFile *curDir;
  const char * const fname = diveToFile(curDir, name, false);
  if (!fname) return;

  if (file.remove(curDir, fname)) {
    SERIAL_PROTOCOLPGM("File deleted:");
    SERIAL_PROTOCOLLN(fname);
    sdpos = 0;
    #if ENABLED(SDCARD_SORT_ALPHA)
      presort();
    #endif
  }
  else {
    SERIAL_PROTOCOLPGM("Deletion failed, File: ");
    SERIAL_PROTOCOL(fname);
    SERIAL_PROTOCOLCHAR('.');
  }
}

void USBReader::getStatus() {
  if (cardOK && sdprinting) {
    SERIAL_PROTOCOLPGM(MSG_SD_PRINTING_BYTE);
    SERIAL_PROTOCOL(sdpos);
    SERIAL_PROTOCOLCHAR('/');
    SERIAL_PROTOCOLLN(filesize);
  }
  else
    SERIAL_PROTOCOLLNPGM(MSG_SD_NOT_PRINTING);
}

void USBReader::write_command(char *buf) {
  char* begin = buf;
  char* npos = NULL;
  char* end = buf + strlen(buf) - 1;

  file.writeError = false;
  if ((npos = strchr(buf, 'N')) != NULL) {
    begin = strchr(npos, ' ') + 1;
    end = strchr(npos, '*') - 1;
  }
  end[1] = '\r';
  end[2] = '\n';
  end[3] = '\0';
  file.write(begin);
  if (file.writeError) {
    SERIAL_ERROR_START();
    SERIAL_ERRORLNPGM(MSG_SD_ERR_WRITE_TO_FILE);
  }
}

#ifdef FYS_ULTILCD2_COMPATIBLE
bool USBReader::write_string(char* buffer)
{
    file.write(buffer);
    return file.writeError;
}
#endif

//
// Run the next autostart file. Called:
// - On boot after successful card init
// - After finishing the previous autostart file
// - From the LCD command to run the autostart file
//

void USBReader::checkautostart() {

  if (autostart_index < 0 || sdprinting) return;

  if (!cardOK) initsd();

/*
  if (cardOK
    #if ENABLED(POWER_LOSS_RECOVERY)
      && !jobRecoverFileExists() // Don't run auto#.g when a resume file exists
    #endif
  ) {
    char autoname[10];
    sprintf_P(autoname, PSTR("auto%i.g"), int(autostart_index));

    //dir_t p;
    //root.rewind();
    //while (root.readDir(&p, NULL) > 0) {
    //  for (int8_t i = (int8_t)strlen((char*)p.name); i--;) p.name[i] = tolower(p.name[i]);
    //  if (p.name[9] != '~' && strncmp((char*)p.name, autoname, 5) == 0) {
    //    openAndPrintFile(autoname);
    //    autostart_index++;
    //    return;
    //  }
    //}

    // 暂时先不去搜索文件夹了，只打印根目录下的 autoname
    if(openFile(autoname, true)) {
      openAndPrintFile(autoname);
      autostart_index++;    
      return;
    }
  }
  */
  autostart_index = -1;
}




void USBReader::beginautostart() {
  autostart_index = 0;
  setroot();
}

void USBReader::closefile(const bool store_location) {
  //file.sync();
  //file.close();

  file.close();
    
  saving = logging = false;

  if (store_location) {
    //future: store printer state, filename and position for continuing a stopped print
    // so one can unplug the printer and continue printing the next day.
  }
}

/**
 * Get the name of a file in the current directory by index
 * with optional name to match.
 */
void USBReader::getfilename(uint16_t nr, const char * const match/*=NULL*/) {
  #if ENABLED(SDSORT_CACHE_NAMES)
    if (match != NULL) {
      while (nr < sort_count) {
        if (strcasecmp(match, sortshort[nr]) == 0) break;
        nr++;
      }
    }
    if (nr < sort_count) {
      strcpy(filename, sortshort[nr]);
      strcpy(longFilename, sortnames[nr]);
      filenameIsDir = TEST(isDir[nr>>3], nr & 0x07);
      return;
    }
  #endif // SDSORT_CACHE_NAMES

  #ifdef USB_READER_DEBUG
    SERIAL_ECHOLN("");
    SERIAL_ECHOLNPAIR("getfilename:",nr);
  #endif
  
  lsAction = LS_GetFilename;
  nrFile_index = nr;
  fileCnt = 0;    // 获取文件名需要清零此计数
  workDir.rewind(); // 工作目录，会随着 lcd 的操作而变化

  //lsDive(NULL, workDir, match);

  // 从此工作目录开始找
  memset(DirStruct,0,sizeof(LS_RECORD)*MAX_FILE_COUNT);
  workDir.getFilename(DirStruct[0].Name);
	DirStruct[ 0 ].DirStartClust = workDir.getDirStartClust();
	DirStruct[ 0 ].Attr = ATTR_DIRECTORY;  /* 根目录也是目录,作为第一个记录保存 */

  #ifdef USB_READER_DEBUG
	  SERIAL_ECHO("root.DirStartClust:");
    SERIAL_ECHOLN(root.getDirStartClust());
  #endif

  // 枚举目录,记录保存到结构中,DirCnt会在枚举到为目录时增加
  UINT8 s;
  UINT16 DirCntOld=0; // geo-f:增加=0 20190228
  /*
	for ( DirCntOld = 0, dirCnt = 1; DirCntOld < dirCnt; DirCntOld ++ ) {  
		s = LsDive( DirCntOld );  
		if ( s != USB_INT_SUCCESS ) mStopIfError( s );
	}*/
	// 只找当前目录
	s = LsDive( DirCntOld );  
	if ( s != USB_INT_SUCCESS ) {
	  mStopIfError( s );
	  cardOK = false;
	}

  #ifdef USB_READER_DEBUG
	  SERIAL_ECHO("root.DirStartClust:");
    SERIAL_ECHOLN(root.getDirStartClust());
  #endif

	CH376FileClose( FALSE );  /* 关闭 */

  #ifdef USB_READER_DEBUG
  	SERIAL_ECHOLN(filename);
  #endif
}

// the short name will be stored in shortFileName[50]
void USBReader::getLongnameFromShort() {
  UINT8 nameAbsPath[LONG_FILENAME_LENGTH];
  UINT8 LongNameBuf[ LONG_NAME_BUF_LEN ];

  // 不获取文件夹长名，会出错
  char*t = strchr(filename, '.');
  if(t==NULL) return;

  #ifdef USB_READER_DEBUG
    SERIAL_PROTOCOLLN("getLongname");
  #endif
  
  memset(longFilename,0,LONG_FILENAME_LENGTH);
  memset(LongNameBuf,0,LONG_NAME_BUF_LEN);
  
  getAbsDir(nameAbsPath,filename);
  
  //SERIAL_PROTOCOLLN((char*)nameAbsPath);
  UINT8 s = CH376GetLongName( nameAbsPath, LongNameBuf );  /* 由短文件名或者目录(文件夹)名获得相应的长文件名 */
	if ( s == USB_INT_SUCCESS ) {
		//SERIAL_PROTOCOLLN( (char*)LongNameBuf );

    UINT8 i=0,j=0;
		for (; j<LONG_NAME_BUF_LEN; j+=2 ) {  /* */
			//SERIAL_CHAR(LongNameBuf[j] );  /* 英文UNICODE字符可以打印输出 */			
			if ( *(PUINT16)(&LongNameBuf[j]) == 0 ) {    
			  //SERIAL_ECHOLN("break");
			  break;  /* 结束 */
			}
			
			longFilename[i]=LongNameBuf[j];
			i++;
		}
		//SERIAL_ECHO("\n");
	}
	else {
	  CH376DebugOutErr( s );
	}
	//CH376FileClose( FALSE );  /* 关闭 */
}


// 此函数是获取 workdir 下的文件数量不是 root 下的 ,只获取这一级目录，下一级不获取
uint16_t USBReader::getnrfilenames() {
  #ifdef USB_READER_DEBUG
    SERIAL_ECHOLN("");
    SERIAL_ECHO("getnrfilenames:");
  #endif

  lsAction = LS_Count;
  nrFiles = 0;
  workDir.rewind();
  //lsDive(NULL, workDir);

  // 从此工作目录开始找
  memset(DirStruct,0,sizeof(LS_RECORD)*MAX_FILE_COUNT);
  workDir.getFilename(DirStruct[0].Name);
	DirStruct[ 0 ].DirStartClust = workDir.getDirStartClust();
	DirStruct[ 0 ].Attr = ATTR_DIRECTORY;  /* 根目录也是目录,作为第一个记录保存 */

  #ifdef USB_READER_DEBUG
    SERIAL_ECHO("workDir.DirStartClust:");
    SERIAL_ECHOLN(workDir.getDirStartClust());
    SERIAL_ECHO("workDir.Name:");
    char *const p = &DirStruct[0].Name[0];
    SERIAL_ECHOLN(p);
  #endif
  
  UINT8 s;
  UINT16 DirCntOld=0;// geo-f:20190228 增加=0
  /*
	for ( DirCntOld = 0, dirCnt = 1; DirCntOld < dirCnt; DirCntOld ++ ) {  
		s = LsDive( DirCntOld );  		
		if ( s != USB_INT_SUCCESS ) mStopIfError( s );
	}
	*/
	// 只找当前目录
	s = LsDive( DirCntOld );  		
  if ( s != USB_INT_SUCCESS ) {    
    mStopIfError( s );
    cardOK = false; // geo-f:20190228
  }

  #ifdef USB_READER_DEBUG
    SERIAL_ECHO("workDir.DirStartClust:");
    SERIAL_ECHOLN(workDir.getDirStartClust());
    SERIAL_ECHO("root.DirStartClust:");
    SERIAL_ECHOLN(root.getDirStartClust());
  #endif
  
  CH376FileClose( FALSE );  /* 关闭一下 */

  #ifdef USB_READER_DEBUG
    SERIAL_ECHOLN(nrFiles);
  #endif

  dirCnt = 0; // geo-f:add20190228
  
  return nrFiles;
}

/**
 * Dive to the given file path, with optional echo.
 * On exit set curDir and return the name part of the path.
 * A NULL result indicates an unrecoverable error.
 * 一层一层打开目录
 */
 /*
const char* USBReader::diveToFile(USBFile*& curDir, const char * const path, const bool echo) {
  USBFile myDir;
  if (path[0] != '/') { curDir = &workDir; return path; }

  curDir = &root;
  const char *dirname_start = &path[1];
  while (dirname_start) {
    char * const dirname_end = strchr(dirname_start, '/');
    if (dirname_end <= dirname_start) break;
    const uint8_t len = dirname_end - dirname_start;
    char dosSubdirname[len + 1];
    strncpy(dosSubdirname, dirname_start, len);
    dosSubdirname[len] = 0;

    if (echo) SERIAL_ECHOLN(dosSubdirname);

    if (!myDir.open(curDir, dosSubdirname, O_READ)) {
      SERIAL_PROTOCOLPAIR(MSG_SD_OPEN_FILE_FAIL, dosSubdirname);
      SERIAL_PROTOCOLCHAR('.');
      SERIAL_EOL();
      return NULL;
    }
    curDir = &myDir;
    dirname_start = dirname_end + 1;
  }
  return dirname_start;
}
*/

const char* USBReader::diveToFile(USBFile*& curDir, const char * const path, const bool echo) {
  USBFile myDir;
  if (path[0] != '/') { curDir = &workDir; return path; }

  curDir = &root;
  const char *dirname_start = &path[1];
  while (dirname_start) {
    char * const dirname_end = strchr(dirname_start, '/');
    if (dirname_end <= dirname_start) break;
    const uint8_t len = dirname_end - dirname_start;
    char dosSubdirname[len + 1];
    strncpy(dosSubdirname, dirname_start, len);
    dosSubdirname[len] = 0;

    if (echo) SERIAL_ECHOLN(dosSubdirname);

    if (!myDir.open(curDir, dosSubdirname, O_READ)) {
      SERIAL_PROTOCOLPAIR(MSG_SD_OPEN_FILE_FAIL, dosSubdirname);
      SERIAL_PROTOCOLCHAR('.');
      SERIAL_EOL();
      return NULL;
    }
    curDir = &myDir;
    dirname_start = dirname_end + 1;
  }
  return dirname_start;
}


// 这里的主要工作是 
// 1、查看工作目录是否打开，如果打开就以这个目录为父目录 - 工作目录字节是否为空，如果不为空，则作为前置父目录
// 2、
void USBReader::chdir(const char * relpath) {
/*
  SdFile newDir;
  SdFile *parent = workDir.isOpen() ? &workDir : &root;

  if (newDir.open(parent, relpath, O_READ)) {
    workDir = newDir;
    if (workDirDepth < MAX_DIR_DEPTH)
      workDirParents[workDirDepth++] = workDir;
    #if ENABLED(SDCARD_SORT_ALPHA)
      presort();
    #endif
  }
  else {
    SERIAL_ECHO_START();
    SERIAL_ECHOPGM(MSG_SD_CANT_ENTER_SUBDIR);
    SERIAL_ECHOLN(relpath);
  }
*/
  UINT8 na[13]="";
  USBFile newDir;
  USBFile *parent = workDir.isOpen() ? &workDir : &root;
  /*
  USBFile *parent;
  if(workDir.isOpen()) {
    parent = &workDir;
    SERIAL_ECHOLN("99"); // 如果这里运行了 那么是 getnrfilenames函数改变了 workDir 的 dirStartClust？
  }
  else {
    parent = &root;
    SERIAL_ECHOLN("66");

    SERIAL_ECHO("parent.DirStartClust:");
    SERIAL_ECHOLN(parent->getDirStartClust());

    SERIAL_ECHO("root.DirStartClust:");
    SERIAL_ECHOLN(root.getDirStartClust());
  }
  */
  if (newDir.open(parent, relpath, O_READ)) {
    workDir = newDir;
    /*
    SERIAL_ECHO("workDir.DirStartClust:");
    SERIAL_ECHOLN(workDir.getDirStartClust());
    SERIAL_ECHO("workDir.name:");
    workDir.getFilename(na);
    char *const p = na;
    SERIAL_ECHOLN(p);
    */
    
    if (workDirDepth < MAX_DIR_DEPTH)
      workDirParents[workDirDepth++] = workDir;
    #if ENABLED(SDCARD_SORT_ALPHA)
      presort();
    #endif
  }
  else {
    SERIAL_ECHO_START();
    SERIAL_ECHOPGM(MSG_SD_CANT_ENTER_SUBDIR);
    SERIAL_ECHOLN(relpath);
    cardOK = false; // geo-f:add 20190227
  }

  /*
  // 组建需要访问的绝对路径,其实不用，CH376FileOpenPath 不需要绝对目录
  const bool  work_dir_parents_empty=(workDirDepth==0);
  const int len = work_dir_parents_empty ? 1 : (workDirDepth+1)*FILENAME_LENGTH + 1 + 1; // 加上 ‘/’ 的位置 和结束符
  char pathAbs[len];
  getAbsDir(pathAbs,relpath);

  // 尝试打开目录，这里最主要的是获取到 pathAbs 的 dirStartClust 便于后面的遍历
  // 这里应该变成一个 open 函数，一个USBFile open一次就可以获得 所有的属性
  UINT8 s = CH376FileOpenPath(pathAbs);
  if (s == ERR_OPEN_DIR) { // 不应该是 USB_INT_SUCCESS，必须是目录
    workDir.setDirStartClust(CH376ReadVar32( VAR_START_CLUSTER ));
    workDir.setFilename(relpath);
    workDir.setAttr(ATTR_DIRECTORY);
    if (workDirDepth < MAX_DIR_DEPTH)
      workDirParents[workDirDepth++] = workDir;
    #if ENABLED(SDCARD_SORT_ALPHA)
      presort();
    #endif
  }
  else {
    SERIAL_ECHO_START();
    SERIAL_ECHOPGM(MSG_SD_CANT_ENTER_SUBDIR);
    SERIAL_ECHOLN(relpath);
  }
  */
}

int8_t USBReader::updir() {
  if (workDirDepth > 0) {                                               // At least 1 dir has been saved
    workDir = --workDirDepth ? workDirParents[workDirDepth - 1] : root; // Use parent, or root if none
    #if ENABLED(SDCARD_SORT_ALPHA)
      presort();
    #endif
  }
  return workDirDepth;
}

int8_t USBReader::isIndir() {
  return workDirDepth>0;
}

void USBReader::setroot() {
  /*if (!workDir.openRoot(&volume)) {
    SERIAL_ECHOLNPGM(MSG_SD_WORKDIR_FAIL);
  }*/

  // 这里的根目录在sd那边和 其他的Sdfile 好像没什么区别，我们的话root需要设定文件名
  root.setFilename("/");
  
  // 这里应该有问题，我们需要重载 ‘=’ ？ // 只要没有动态内存申请函数，用默认拷贝构造函数即可
  //workDir = root; 

  // 20190228:这里是不是应该把workDir 打开一下？这样 / 根目录就打开了（好像根目录不需要打开吧，再想想）
  // 应该是要打开 workDir 这样 root 的 clust 就变成了 2了 试试
  USBFile newDir;
  if (newDir.open(&root, "/", O_READ)) {
    workDir = newDir;
  }
  //workDir.open(USBFile * dirFile, "/", O_READ);

  //SERIAL_ECHO("root.DirStartClust:");
  //SERIAL_ECHOLN(root.getDirStartClust());
  //SERIAL_ECHO("workDir.DirStartClust:");
  //SERIAL_ECHOLN(workDir.getDirStartClust());

  
  #if ENABLED(SDCARD_SORT_ALPHA)
    presort();
  #endif
}

#if ENABLED(SDCARD_SORT_ALPHA)

  /**
   * Get the name of a file in the current directory by sort-index
   */
  void USBReader::getfilename_sorted(const uint16_t nr) {
    getfilename(
      #if ENABLED(SDSORT_GCODE)
        sort_alpha &&
      #endif
      (nr < sort_count) ? sort_order[nr] : nr
    );
  }

  /**
   * Read all the files and produce a sort key
   *
   * We can do this in 3 ways...
   *  - Minimal RAM: Read two filenames at a time sorting along...
   *  - Some RAM: Buffer the directory just for this sort
   *  - Most RAM: Buffer the directory and return filenames from RAM
   */
  void USBReader::presort() {

    // Throw away old sort index
    flush_presort();

    // Sorting may be turned off
    #if ENABLED(SDSORT_GCODE)
      if (!sort_alpha) return;
    #endif

    // If there are files, sort up to the limit
    uint16_t fileCnt = getnrfilenames();
    if (fileCnt > 0) {

      // Never sort more than the max allowed
      // If you use folders to organize, 20 may be enough
      if (fileCnt > SDSORT_LIMIT) fileCnt = SDSORT_LIMIT;

      // Sort order is always needed. May be static or dynamic.
      #if ENABLED(SDSORT_DYNAMIC_RAM)
        sort_order = new uint8_t[fileCnt];
      #endif

      // Use RAM to store the entire directory during pre-sort.
      // SDSORT_LIMIT should be set to prevent over-allocation.
      #if ENABLED(SDSORT_USES_RAM)

        // If using dynamic ram for names, allocate on the heap.
        #if ENABLED(SDSORT_CACHE_NAMES)
          #if ENABLED(SDSORT_DYNAMIC_RAM)
            sortshort = new char*[fileCnt];
            sortnames = new char*[fileCnt];
          #endif
        #elif ENABLED(SDSORT_USES_STACK)
          char sortnames[fileCnt][SORTED_LONGNAME_MAXLEN];
        #endif

        // Folder sorting needs 1 bit per entry for flags.
        #if HAS_FOLDER_SORTING
          #if ENABLED(SDSORT_DYNAMIC_RAM)
            isDir = new uint8_t[(fileCnt + 7) >> 3];
          #elif ENABLED(SDSORT_USES_STACK)
            uint8_t isDir[(fileCnt + 7) >> 3];
          #endif
        #endif

      #else // !SDSORT_USES_RAM

        // By default re-read the names from SD for every compare
        // retaining only two filenames at a time. This is very
        // slow but is safest and uses minimal RAM.
        char name1[LONG_FILENAME_LENGTH + 1];

      #endif

      if (fileCnt > 1) {

        // Init sort order.
        for (uint16_t i = 0; i < fileCnt; i++) {
          sort_order[i] = i;
          // If using RAM then read all filenames now.
          #if ENABLED(SDSORT_USES_RAM)
            getfilename(i);
            #if ENABLED(SDSORT_DYNAMIC_RAM)
              // Use dynamic method to copy long filename
              sortnames[i] = strdup(longest_filename());
              #if ENABLED(SDSORT_CACHE_NAMES)
                // When caching also store the short name, since
                // we're replacing the getfilename() behavior.
                sortshort[i] = strdup(filename);
              #endif
            #else
              // Copy filenames into the static array
              #if SORTED_LONGNAME_MAXLEN != LONG_FILENAME_LENGTH
                strncpy(sortnames[i], longest_filename(), SORTED_LONGNAME_MAXLEN);
                sortnames[i][SORTED_LONGNAME_MAXLEN - 1] = '\0';
              #else
                strncpy(sortnames[i], longest_filename(), SORTED_LONGNAME_MAXLEN);
              #endif
              #if ENABLED(SDSORT_CACHE_NAMES)
                strcpy(sortshort[i], filename);
              #endif
            #endif
            // char out[30];
            // sprintf_P(out, PSTR("---- %i %s %s"), i, filenameIsDir ? "D" : " ", sortnames[i]);
            // SERIAL_ECHOLN(out);
            #if HAS_FOLDER_SORTING
              const uint16_t bit = i & 0x07, ind = i >> 3;
              if (bit == 0) isDir[ind] = 0x00;
              if (filenameIsDir) isDir[ind] |= _BV(bit);
            #endif
          #endif
        }

        // Bubble Sort
        for (uint16_t i = fileCnt; --i;) {
          bool didSwap = false;
          for (uint16_t j = 0; j < i; ++j) {
            const uint16_t o1 = sort_order[j], o2 = sort_order[j + 1];

            // Compare names from the array or just the two buffered names
            #if ENABLED(SDSORT_USES_RAM)
              #define _SORT_CMP_NODIR() (strcasecmp(sortnames[o1], sortnames[o2]) > 0)
            #else
              #define _SORT_CMP_NODIR() (strcasecmp(name1, name2) > 0)
            #endif

            #if HAS_FOLDER_SORTING
              #if ENABLED(SDSORT_USES_RAM)
                // Folder sorting needs an index and bit to test for folder-ness.
                const uint8_t ind1 = o1 >> 3, bit1 = o1 & 0x07,
                              ind2 = o2 >> 3, bit2 = o2 & 0x07;
                #define _SORT_CMP_DIR(fs) \
                  (((isDir[ind1] & _BV(bit1)) != 0) == ((isDir[ind2] & _BV(bit2)) != 0) \
                    ? _SORT_CMP_NODIR() \
                    : (isDir[fs > 0 ? ind1 : ind2] & (fs > 0 ? _BV(bit1) : _BV(bit2))) != 0)
              #else
                #define _SORT_CMP_DIR(fs) ((dir1 == filenameIsDir) ? _SORT_CMP_NODIR() : (fs > 0 ? dir1 : !dir1))
              #endif
            #endif

            // The most economical method reads names as-needed
            // throughout the loop. Slow if there are many.
            #if DISABLED(SDSORT_USES_RAM)
              getfilename(o1);
              strcpy(name1, longest_filename()); // save (or getfilename below will trounce it)
              #if HAS_FOLDER_SORTING
                bool dir1 = filenameIsDir;
              #endif
              getfilename(o2);
              char *name2 = longest_filename(); // use the string in-place
            #endif // !SDSORT_USES_RAM

            // Sort the current pair according to settings.
            if (
              #if HAS_FOLDER_SORTING
                #if ENABLED(SDSORT_GCODE)
                  sort_folders ? _SORT_CMP_DIR(sort_folders) : _SORT_CMP_NODIR()
                #else
                  _SORT_CMP_DIR(FOLDER_SORTING)
                #endif
              #else
                _SORT_CMP_NODIR()
              #endif
            ) {
              sort_order[j] = o2;
              sort_order[j + 1] = o1;
              didSwap = true;
            }
          }
          if (!didSwap) break;
        }
        // Using RAM but not keeping names around
        #if ENABLED(SDSORT_USES_RAM) && DISABLED(SDSORT_CACHE_NAMES)
          #if ENABLED(SDSORT_DYNAMIC_RAM)
            for (uint16_t i = 0; i < fileCnt; ++i) free(sortnames[i]);
            #if HAS_FOLDER_SORTING
              free(isDir);
            #endif
          #endif
        #endif
      }
      else {
        sort_order[0] = 0;
        #if ENABLED(SDSORT_USES_RAM) && ENABLED(SDSORT_CACHE_NAMES)
          getfilename(0);
          #if ENABLED(SDSORT_DYNAMIC_RAM)
            sortnames = new char*[1];
            sortnames[0] = strdup(longest_filename()); // malloc
            #if ENABLED(SDSORT_CACHE_NAMES)
              sortshort = new char*[1];
              sortshort[0] = strdup(filename);       // malloc
            #endif
            isDir = new uint8_t[1];
          #else
            #if SORTED_LONGNAME_MAXLEN != LONG_FILENAME_LENGTH
              strncpy(sortnames[0], longest_filename(), SORTED_LONGNAME_MAXLEN);
              sortnames[0][SORTED_LONGNAME_MAXLEN - 1] = '\0';
            #else
              strncpy(sortnames[0], longest_filename(), SORTED_LONGNAME_MAXLEN);
            #endif
            #if ENABLED(SDSORT_CACHE_NAMES)
              strcpy(sortshort[0], filename);
            #endif
          #endif
          isDir[0] = filenameIsDir ? 0x01 : 0x00;
        #endif
      }

      sort_count = fileCnt;
    }
  }

  void USBReader::flush_presort() {
    if (sort_count > 0) {
      #if ENABLED(SDSORT_DYNAMIC_RAM)
        delete sort_order;
        #if ENABLED(SDSORT_CACHE_NAMES)
          for (uint8_t i = 0; i < sort_count; ++i) {
            free(sortshort[i]); // strdup
            free(sortnames[i]); // strdup
          }
          delete sortshort;
          delete sortnames;
        #endif
      #endif
      sort_count = 0;
    }
  }

#endif // SDCARD_SORT_ALPHA

// 此函数是获取 workdir 下的文件数量不是 root 下的 ,只获取这一级目录，下一级不获取
uint16_t USBReader::get_num_Files() {
  return
    #if ENABLED(SDCARD_SORT_ALPHA) && SDSORT_USES_RAM && SDSORT_CACHE_NAMES
      nrFiles // no need to access the SD card for filenames
    #else
      getnrfilenames()
    #endif
  ;
}

void USBReader::printingHasFinished() {
  planner.synchronize();
  file.close();
  if (file_subcall_ctr > 0) { // Heading up to a parent file that called current as a procedure.
    file_subcall_ctr--;
    openFile(proc_filenames[file_subcall_ctr], true, true);
    setIndex(filespos[file_subcall_ctr]);
    startFileprint();
  }
  else {
    sdprinting = false;

    #if ENABLED(POWER_LOSS_RECOVERY)
      removeJobRecoveryFile();
    #endif

    #if ENABLED(SD_FINISHED_STEPPERRELEASE) && defined(SD_FINISHED_RELEASECOMMAND)
      planner.finish_and_disable();
    #endif
    print_job_timer.stop();
    if (print_job_timer.duration() > 60)
      enqueue_and_echo_commands_P(PSTR("M31"));
    #if ENABLED(SDCARD_SORT_ALPHA)
      presort();
    #endif
    #if ENABLED(ULTRA_LCD) && ENABLED(LCD_SET_PROGRESS_MANUALLY)
      progress_bar_percent = 0;
    #endif
    #if ENABLED(SD_REPRINT_LAST_SELECTED_FILE)
      lcd_reselect_last_file();
    #endif
  }
}

#if ENABLED(AUTO_REPORT_SD_STATUS)
  uint8_t USBReader::auto_report_sd_interval = 0;
  millis_t USBReader::next_sd_report_ms;

  void USBReader::auto_report_sd_status() {
    millis_t current_ms = millis();
    if (auto_report_sd_interval && ELAPSED(current_ms, next_sd_report_ms)) {
      next_sd_report_ms = current_ms + 1000UL * auto_report_sd_interval;
      getStatus();
    }
  }
#endif // AUTO_REPORT_SD_STATUS

#if ENABLED(POWER_LOSS_RECOVERY)

  char job_recovery_file_name[13] = "rcvy.bin";

  void USBReader::openJobRecoveryFile(const bool read) {
    if (!cardOK) {
      SERIAL_ECHOLN("open job card bad");
      return;
    }
    if (jobRecoveryFile.isOpen()) {
      SERIAL_ECHOLN("open job already open");
      return;
    }

    // 通过路径获取到文件所属的目录 curDir，并返回文件名
    // 相当于 CH376FileOpenDir 功能，不过，在文件打开的过程中，获取了一些文件的属性并记录了
    // 而 CH376FileOpenDir 只是单纯的去打开目录了
    USBFile *curDir = &root;
    //const char * const fname = diveToFile(curDir, job_recovery_file_name, false);
    //if (!fname) return;
    const char * const fname = &job_recovery_file_name[0];

    // 最终目标是为了打开最终的文件
    if (read) {
      if (jobRecoveryFile.open(curDir, fname, O_READ)) {
        //jobRecoveryFilesize = jobRecoveryFile.fileSize();
        //jobRecoveryFilesdpos = 0;
        SERIAL_PROTOCOLPAIR(MSG_SD_FILE_OPENED, fname);
        SERIAL_PROTOCOLLNPAIR(MSG_SD_SIZE, filesize);
        SERIAL_PROTOCOLLNPGM(MSG_SD_FILE_SELECTED);
      }
      else {
        SERIAL_PROTOCOLPAIR(MSG_SD_OPEN_FILE_FAIL, fname);
        SERIAL_PROTOCOLCHAR('.');
        SERIAL_EOL();
      }
    }
    else { //write
      // 如果已经存在了，我们就直接打开文件就行，如果不存在我们就建立文件
      if (!jobRecoveryFile.open(curDir, fname, O_WRITE)) {
        UINT8 s = CH376FileCreatePath("/rcvy.bin");
        if ( s != USB_INT_SUCCESS ) {
          SERIAL_PROTOCOLPAIR("create file fail:", job_recovery_file_name);
          SERIAL_PROTOCOLCHAR('.');
          SERIAL_EOL();
        }
      }
      /*
      UINT8 s = CH376FileCreatePath("/rcvy.bin");
      if ( s != USB_INT_SUCCESS ) {
        SERIAL_PROTOCOLPAIR("create file fail:", job_recovery_file_name);
        SERIAL_PROTOCOLCHAR('.');
        SERIAL_EOL();
      }*/
    }
    /*
    if (!jobRecoveryFile.open(&root, job_recovery_file_name, read ? O_READ : O_CREAT | O_WRITE | O_TRUNC | O_SYNC)) {
      SERIAL_PROTOCOLPAIR(MSG_SD_OPEN_FILE_FAIL, job_recovery_file_name);
      SERIAL_PROTOCOLCHAR('.');
      SERIAL_EOL();
    }
    else if (!read)
      SERIAL_PROTOCOLLNPAIR(MSG_SD_WRITE_TO_FILE, job_recovery_file_name);
    */
  }

  void USBReader::closeJobRecoveryFile() { jobRecoveryFile.close(); }

  bool USBReader::jobRecoverFileExists() {
    const bool exists = jobRecoveryFile.open(&root, job_recovery_file_name, O_READ);
    if (exists) {
      jobRecoveryFile.close();
      #if ENABLED(DEBUG_POWER_LOSS_RECOVERY)
        SERIAL_PROTOCOLLNPGM("---- jobRecoverFileExists. ----");
      #endif
    }
    #if ENABLED(DEBUG_POWER_LOSS_RECOVERY)
    else
      SERIAL_PROTOCOLLNPGM("---- jobRecoverFile not Exists. ----");
    #endif
    //jobRecoveryFile.close();
    return exists;
  }

  int16_t USBReader::saveJobRecoveryInfo() {
    #if ENABLED(DEBUG_POWER_LOSS_RECOVERY)
      SERIAL_PROTOCOLLNPGM("---- saveJobRecoveryInfo. ----");
    #endif
    jobRecoveryFile.seekSet(0);
    const int16_t ret = jobRecoveryFile.write(&job_recovery_info, sizeof(job_recovery_info));
    #if ENABLED(DEBUG_POWER_LOSS_RECOVERY)
      if (ret == -1) SERIAL_PROTOCOLLNPGM("Power-loss file write failed.");
    #endif
    return ret;
  }

  int16_t USBReader::loadJobRecoveryInfo() {
    return jobRecoveryFile.read(&job_recovery_info, sizeof(job_recovery_info));
  }

  void USBReader::removeJobRecoveryFile() {
    #if ENABLED(DEBUG_POWER_LOSS_RECOVERY)
      SERIAL_PROTOCOLLNPGM("---- removeJobRecoveryFile. ----");
    #endif
    job_recovery_info.valid_head = job_recovery_info.valid_foot = job_recovery_commands_count = 0;
    if (jobRecoverFileExists()) {
      //closefile();
      jobRecoveryFile.close();
      //removeFile(job_recovery_file_name);
      jobRecoveryFile.remove(&root, job_recovery_file_name);
      #if ENABLED(DEBUG_POWER_LOSS_RECOVERY)
        SERIAL_PROTOCOLPGM("Power-loss file delete");
        serialprintPGM(jobRecoverFileExists() ? PSTR(" failed.\n") : PSTR("d.\n"));
      #endif
    }
  }

#endif // POWER_LOSS_RECOVERY


#endif // FYS_STORAGE_SUPPORT
