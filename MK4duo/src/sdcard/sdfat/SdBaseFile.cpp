/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 Alberto Cotronei @MagoKimbra
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * Arduino SdFat Library
 * Copyright (C) 2009 by William Greiman
 *
 * This file is part of the Arduino Sd2Card Library
 */

#include "../../../MK4duo.h"

#if HAS_SD_SUPPORT

#include "SdBaseFile.h"

int8_t RFstricmp(PGM_P s1, PGM_P s2) {
  while(*s1 && (tolower(*s1) == tolower(*s2)))
    s1++, s2++;
  return (const uint8_t)tolower(*s1) - (const uint8_t)tolower(*s2);
}

int8_t RFstrnicmp(PGM_P s1, PGM_P s2, size_t n) {
  while(n--) {
    if(tolower(*s1) != tolower(*s2))
      return (uint8_t)tolower(*s1) - (uint8_t)tolower(*s2);
    s1++;
    s2++;
  }
  return 0;
}

/** Public Parameters */
SdBaseFile* SdBaseFile::cwd_ = 0;   // Pointer to Current Working Directory

// callback function for date/time
void (*SdBaseFile::dateTime_)(uint16_t* date, uint16_t* time) = 0;

// add a cluster to a file
bool SdBaseFile::addCluster() {
  if (!vol_->allocContiguous(1, &curCluster_)) return false;
  // if first cluster of file link to directory entry
  if (firstCluster_ == 0) {
    firstCluster_ = curCluster_;
    flags_ |= F_FILE_DIR_DIRTY;
  }
  return true;
}

// Add a cluster to a directory file and zero the cluster.
// return with first block of cluster in the cache
cache_t* SdBaseFile::addDirCluster() {
  uint32_t block;
  cache_t* pc;
  // max folder size
  if (fileSize_ / sizeof(dir_t) >= 0xFFFF) return 0;

  if (!addCluster()) return 0;

  block = vol_->clusterStartBlock(curCluster_);
  pc = vol_->cacheFetch(block, SdVolume::CACHE_RESERVE_FOR_WRITE);
  if (!pc) return 0;

  memset(pc, 0, 512);
  // zero rest of clusters
  for (uint8_t i = 1; i < vol_->blocksPerCluster_; i++) {
    if (!vol_->writeBlock(block + i, pc->data)) return 0;
  }

  // Increase directory file size by cluster size
  fileSize_ += 512UL*vol_->blocksPerCluster_;
  return pc;
}

// cache a file's directory entry
// return pointer to cached entry or null for failure
dir_t* SdBaseFile::cacheDirEntry(uint8_t action) {
  cache_t* pc;
  pc = vol_->cacheFetch(dirBlock_, action);
  if (!pc) return 0;
  return pc->dir + dirIndex_;
}

/** Close a file and force cached data and directory information
 *  to be written to the storage device.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 * Reasons for failure include no file is open or an I/O error.
 */
bool SdBaseFile::close() {
  bool rtn = sync();
  type_ = FAT_FILE_TYPE_CLOSED;
  return rtn;
}

/** Check for contiguous file and return its raw block range.
 *
 * \param[out] bgnBlock the first block address for the file.
 * \param[out] endBlock the last  block address for the file.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 * Reasons for failure include file is not contiguous, file has zero length
 * or an I/O error occurred.
 */
bool SdBaseFile::contiguousRange(uint32_t* bgnBlock, uint32_t* endBlock) {
  // error if no blocks
  if (firstCluster_ == 0) return false;

  for (uint32_t c = firstCluster_; ; c++) {
    uint32_t next;
    if (!vol_->fatGet(c, &next)) return false;

    // check for contiguous
    if (next != (c + 1)) {
      // error if not end of chain
      if (!vol_->isEOC(next)) return false;

      *bgnBlock = vol_->clusterStartBlock(firstCluster_);
      *endBlock = vol_->clusterStartBlock(c) + vol_->blocksPerCluster_ - 1;
      return true;
    }
  }

  return false;
}

/** Create and open a new contiguous file of a specified size.
 *
 * \note This function only supports short DOS 8.3 names.
 * See open() for more information.
 *
 * \param[in] dirFile The directory where the file will be created.
 * \param[in] path A path with a valid DOS 8.3 file name.
 * \param[in] size The desired file size.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 * Reasons for failure include \a path contains
 * an invalid DOS 8.3 file name, the FAT volume has not been initialized,
 * a file is already open, the file already exists, the root
 * directory is full or an I/O error.
 *
 */
bool SdBaseFile::createContiguous(SdBaseFile* dirFile, const char * path, uint32_t size) {
  uint32_t count;
  // don't allow zero length file
  if (size == 0) return false;

  if (!open(dirFile, path, O_CREAT | O_EXCL | O_RDWR)) return false;

  // calculate number of clusters needed
  count = ((size - 1) >> (vol_->clusterSizeShift_ + 9)) + 1;

  // allocate clusters
  if (!vol_->allocContiguous(count, &firstCluster_)) {
    remove();
    return false;
  }

  fileSize_ = size;

  // insure sync() will update dir entry
  flags_ |= F_FILE_DIR_DIRTY;

  return sync();

}

/** Return a file's directory entry.
 *
 * \param[out] dir Location for return of the file's directory entry.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
bool SdBaseFile::dirEntry(dir_t* dir) {
  dir_t* p;
  // make sure fields on SD are correct
  if (!sync()) return false;

  // read entry
  p = cacheDirEntry(SdVolume::CACHE_FOR_READ);
  if (!p) return false;

  // copy to caller's struct
  memcpy(dir, p, sizeof(dir_t));
  return true;

}

/** Format the name field of \a dir into the 13 byte array
 * \a name in standard 8.3 short name format.
 *
 * \param[in] dir The directory structure containing the name.
 * \param[out] name A 13 byte char array for the formatted name.
 */
void SdBaseFile::dirName(const dir_t& dir, char* name) {
  uint8_t j = 0;
  for (uint8_t i = 0; i < 11; i++) {
    if (dir.name[i] == ' ') continue;
    if (i == 8) name[j++] = '.';
    name[j++] = dir.name[i];
  }
  name[j] = 0;
}

/** Test for the existence of a file in a directory
 *
 * \param[in] name Name of the file to be tested for.
 *
 * The calling instance must be an open directory file.
 *
 * dirFile.exists("TOFIND.TXT") searches for "TOFIND.TXT" in  the directory
 * dirFile.
 *
 * \return true if the file exists else false.
 */
bool SdBaseFile::exists(const char * name) {
  SdBaseFile file;
  return file.open(this, name, FILE_READ);
}

/**
 * Get a string from a file.
 *
 * fgets() reads bytes from a file into the array pointed to by \a str, until
 * \a num - 1 bytes are read, or a delimiter is read and transferred to \a str,
 * or end-of-file is encountered. The string is then terminated
 * with a null byte.
 *
 * fgets() deletes CR, '\\r', from the string.  This insures only a '\\n'
 * terminates the string for Windows text files which use CRLF for newline.
 *
 * \param[out] str Pointer to the array where the string is stored.
 * \param[in] num Maximum number of characters to be read
 * (including the final null byte). Usually the length
 * of the array \a str is used.
 * \param[in] delim Optional set of delimiters. The default is "\n".
 *
 * \return For success fgets() returns the length of the string in \a str.
 * If no data is read, fgets() returns zero for EOF or -1 if an error occurred.
 **/
int16_t SdBaseFile::fgets(char* str, int16_t num, char* delim) {
  char ch;
  int16_t n = 0;
  int16_t r = -1;
  while ((n + 1) < num && (r = read(&ch, 1)) == 1) {
    // delete CR
    if (ch == '\r') continue;
    str[n++] = ch;
    if (!delim) {
      if (ch == '\n') break;
    }
    else {
      if (strchr(delim, ch)) break;
    }
  }
  if (r < 0) {
    // read error
    return -1;
  }
  str[n] = '\0';
  return n;
}

/** Get a file's name
 *
 * \param[out] name An array of 13 characters for the file's name.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
bool SdBaseFile::getFilename(char* name) {
  if (!isOpen()) return false;

  if (isRoot()) {
    name[0] = '/';
    name[1] = '\0';
    return true;
  }
  // cache entry
  dir_t* p = cacheDirEntry(SdVolume::CACHE_FOR_READ);
  if (!p) return false;

  // format name
  dirName(*p, name);
  return true;
}


void SdBaseFile::getpos(FatPos_t* pos) {
  pos->position = curPosition_;
  pos->cluster = curCluster_;
}

uint8_t SdBaseFile::lsRecursive(SdBaseFile* parent, uint8_t level, char* findFilename, SdBaseFile* pParentFound, bool isJson) {
  dir_t *p = NULL;
  //uint8_t cnt=0;
  //char *oldpathend = pathend;
  #if ENABLED(JSON_OUTPUT)
    bool firstFile = true;
  #endif

  parent->rewind();

  while ((p = parent->getLongFilename(p, card.tempLongFilename))) {
    //HAL::pingWatchdog();
    if (! (DIR_IS_FILE(p) || DIR_IS_SUBDIR(p))) continue;
    if (strcmp(card.tempLongFilename, "..") == 0) continue;
    if (card.tempLongFilename[0] == '.') continue; // MAC CRAP
    if (DIR_IS_SUBDIR(p)) {
      if (level >= SD_MAX_FOLDER_DEPTH) continue; // can't go deeper
      if (level < SD_MAX_FOLDER_DEPTH && findFilename == NULL) {
        if (level && !isJson) {
          SERIAL_TXT(card.fileName);
          SERIAL_CHR('/');
        }
        #if ENABLED(JSON_OUTPUT)
          if (isJson) {
            if (!firstFile) SERIAL_CHR(',');
            SERIAL_CHR('"'); SERIAL_CHR('*');
            SDCard::printEscapeChars(card.tempLongFilename);
            SERIAL_CHR('"');
            firstFile = false;
          }
          else {
            SERIAL_TXT(card.tempLongFilename);
            SERIAL_CHR('/'); SERIAL_EOL(); // End with / to mark it as directory entry, so we can see empty directories.
          }
        #else
          SERIAL_TXT(card.tempLongFilename);
          SERIAL_CHR('/'); SERIAL_EOL();// End with / to mark it as directory entry, so we can see empty directories.
        #endif
      }
      SdBaseFile next;
      char *tmp;

      if(level) strcat(card.fileName, "/");

      strcat(card.fileName, card.tempLongFilename);
      uint16_t index = (parent->curPosition()-31) >> 5;

      if(!isJson && next.open(parent, index, FILE_READ)) {
        if (next.lsRecursive(&next, level + 1, findFilename, pParentFound, false))
          return true;
      }
      parent->seekSet(32 * (index + 1));
      if ((tmp = strrchr(card.fileName, '/')) != NULL)
        *tmp = 0;
      else
        *card.fileName = 0;
    }
    else {
      if (findFilename != NULL) {
        int8_t cFullname;
        cFullname = strlen(card.fileName);
        if (RFstrnicmp(card.fileName, findFilename, cFullname) == 0) {
          if (cFullname > 0)
            cFullname++;
          if (RFstricmp(card.tempLongFilename, findFilename + cFullname) == 0) {
            if (pParentFound != NULL)
              *pParentFound = *parent;
            return true;
          }
        }
      }
      else {
        if(level && !isJson) {
          SERIAL_TXT(card.fileName);
          SERIAL_CHR('/');
        }
        #if ENABLED(JSON_OUTPUT)
          if (isJson) {
            if (!firstFile) SERIAL_CHR(',');
            SERIAL_CHR('"');
            SDCard::printEscapeChars(card.tempLongFilename);
            SERIAL_CHR('"');
            firstFile = false;
          }
          else
        #endif
        {
          SERIAL_TXT(card.tempLongFilename);
          #if ENABLED(SD_EXTENDED_DIR)
            SERIAL_MV(" ", (long) p->fileSize);
          #endif
          SERIAL_EOL();
        }
      }
    }
  }
  return false;
}


/** List directory contents.
 * list to indicate subdirectory level.
 */
void SdBaseFile::ls() {
  SdBaseFile parent;
  rewind();
  *card.fileName = 0;
  pathend = card.fileName;
  parent = *this;
  lsRecursive(&parent, 0, NULL, NULL, false);
}

#if ENABLED(JSON_OUTPUT)

  void SdBaseFile::lsJSON() {
    SdBaseFile parent;
    rewind();
    *card.fileName = 0;
    parent = *this;
    lsRecursive(&parent, 0, NULL, NULL, true);
  }

#endif

// saves 32 bytes on stack for ls recursion
// return 0 - EOF, 1 - normal file, or 2 - directory
int8_t SdBaseFile::lsPrintNext(uint8_t flags, uint8_t indent) {
  dir_t dir;
  //uint8_t w = 0;
  while (1) {
    if (read(&dir, sizeof(dir)) != sizeof(dir)) return 0;
    if (dir.name[0] == DIR_NAME_FREE) return 0;

    // skip deleted entry and entries for . and  ..
    if (dir.name[0] != DIR_NAME_DELETED && dir.name[0] != '.'
      && DIR_IS_FILE_OR_SUBDIR(&dir)) break;
  }
  // indent for dir level
  for (uint8_t i = 0; i < indent; i++) SERIAL_CHR(' ');

  printDirName(dir, flags & (LS_DATE | LS_SIZE) ? 14 : 0, true);

  // print modify date/time if requested
  if (flags & LS_DATE) {
    SERIAL_CHR(' ');
    printFatDate(dir.lastWriteDate);
    SERIAL_CHR(' ');
    printFatTime(dir.lastWriteTime);
  }
  // print size if requested
  if (!DIR_IS_SUBDIR(&dir) && (flags & LS_SIZE)) {
    SERIAL_CHR(' ');
    SERIAL_VAL(dir.fileSize);
  }
  SERIAL_EOL();
  return DIR_IS_FILE(&dir) ? 1 : 2;
}

// format directory name field from a 8.3 name string

bool SdBaseFile::make83Name(PGM_P str, uint8_t* name, const char** ptr) {
  uint8_t c;
  uint8_t n = 7;  // max index for part before dot
  uint8_t i = 0;
  // blank fill name and extension
  while (i < 11) name[i++] = ' ';
  i = 0;
  while (*str != '\0' && *str != '/') {
    c = *str++;
    if (c == '.') {
      if (n == 10) return false;
      n = 10;  // max index for full 8.3 name
      i = 8;   // place for extension
    }
    else {
      // illegal FAT characters
      #if ENABLED(__AVR__)
        // store chars in flash
        PGM_P p = PSTR("|<>^+=?/[];,*\"\\");
        uint8_t b;
        while ((b = pgm_read_byte(p++))) {
          if (b == c) return false;
        }
      #else  // __AVR__
        // store chars in RAM
        if (strchr("|<>^+=?/[];,*\"\\", c)) return false;
      #endif  // __AVR__

      // check size and only allow ASCII printable characters
      if (i > n || c < 0x20 || c > 0x7E) {
        c = '_';
      }
      // only upper case allowed in 8.3 names - convert lower to upper
      name[i++] = c < 'a' || c > 'z' ?  c : c + ('A' - 'a');
    }
  }
  *ptr = str;
  // must have a file name, extension is optional
  return name[0] != ' ';

}

/** Make a new directory.
 *
 * \param[in] parent An open SdFat instance for the directory that will contain
 * the new directory.
 *
 * \param[in] path A path with a valid 8.3 DOS name for the new directory.
 *
 * \param[in] pFlag Create missing parent directories if true.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 * Reasons for failure include this file is already open, \a parent is not a
 * directory, \a path is invalid or already exists in \a parent.
 */
bool SdBaseFile::mkdir(SdBaseFile* parent, const char * path, bool pFlag) {
  uint8_t dname[LONG_FILENAME_LENGTH + 1];
  SdBaseFile newParent;

  if (openParentReturnFile(parent, path, dname, &newParent, pFlag))
    return mkdir(&newParent, dname);

  return false;
}

bool SdBaseFile::mkdir(SdBaseFile* parent, const uint8_t *dname) {
  dir_t d;

  if (!parent->isDir()) return false;

  // create a normal file
  if (!open(parent, dname, O_CREAT | O_EXCL | O_RDWR, true)) return false;

  // make entry for '.'
  memset(&d, 0, sizeof(d));
  d.creationDate = FAT_DEFAULT_DATE;
  d.creationTime = FAT_DEFAULT_TIME;
  d.lastAccessDate = d.creationDate;
  d.lastWriteDate = d.creationDate;
  d.lastWriteTime = d.creationTime;

  d.name[0] = '.';
  d.attributes = DIR_ATT_DIRECTORY;
  for (uint8_t i = 1; i < 11; i++) d.name[i] = ' ';

  if (write(&d, sizeof(dir_t)) < 0) return false;

  sync();

  // make entry for '..'
  d.name[1] = '.';
  if (parent->isRoot()) {
    d.firstClusterLow = 0;
    d.firstClusterHigh = 0;
  }
  else {
    d.firstClusterLow = parent->firstCluster_ & 0xFFFF;
    d.firstClusterHigh = parent->firstCluster_ >> 16;
  }
  if (write(&d, sizeof(dir_t)) < 0)
    return false;
  sync();
  memset(&d, 0, sizeof(dir_t));
  if (write(&d, sizeof(dir_t)) < 0)
    return false;
  sync();

  type_ = FAT_FILE_TYPE_SUBDIR;
  flags_ |= F_FILE_DIR_DIRTY;
  return true;
}

/** Open a file in the current working directory.
 *
 * \param[in] path A path with a valid 8.3 DOS name for a file to be opened.
 *
 * \param[in] oflag Values for \a oflag are constructed by a bitwise-inclusive
 * OR of open flags. see SdBaseFile::open(SdBaseFile*, const char*, uint8_t).
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
bool SdBaseFile::open(PGM_P path, uint8_t oflag) {
  return open(cwd_, path, oflag);
}

/** Open a file or directory by name.
 *
 * \param[in] dirFile An open SdFat instance for the directory containing the
 * file to be opened.
 *
 * \param[in] path A path with a valid 8.3 DOS name for a file to be opened.
 *
 * \param[in] oflag Values for \a oflag are constructed by a bitwise-inclusive
 * OR of flags from the following list
 *
 * O_READ - Open for reading.
 *
 * O_RDONLY - Same as O_READ.
 *
 * O_WRITE - Open for writing.
 *
 * O_WRONLY - Same as O_WRITE.
 *
 * O_RDWR - Open for reading and writing.
 *
 * O_APPEND - If set, the file offset shall be set to the end of the
 * file prior to each write.
 *
 * O_AT_END - Set the initial position at the end of the file.
 *
 * O_CREAT - If the file exists, this flag has no effect except as noted
 * under O_EXCL below. Otherwise, the file shall be created
 *
 * O_EXCL - If O_CREAT and O_EXCL are set, open() shall fail if the file exists.
 *
 * O_SYNC - Call sync() after each write.  This flag should not be used with
 * write(uint8_t), write_P(PGM_P), writelnmkdir_P(PGM_P), or the Arduino Print class.
 * These functions do character at a time writes so sync() will be called
 * after each byte.
 *
 * O_TRUNC - If the file exists and is a regular file, and the file is
 * successfully opened and is not read only, its length shall be truncated to 0.
 *
 * WARNING: A given file must not be opened by more than one SdBaseFile object
 * of file corruption may occur.
 *
 * \note Directory files must be opened read only.  Write and truncation is
 * not allowed for directory files.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 * Reasons for failure include this file is already open, \a dirFile is not
 * a directory, \a path is invalid, the file does not exist
 * or can't be opened in the access mode specified by oflag.
 */

 bool SdBaseFile::openParentReturnFile(SdBaseFile* dirFile, const char * path, uint8_t *dname, SdBaseFile *newParent, bool bMakeDirs) {
  SdBaseFile dir1, dir2;
  SdBaseFile *parent = dirFile;
  //dir_t *pEntry;
  SdBaseFile *sub = &dir1;
  char *p;
  //bool bFound;

  *dname = 0;

  if (!dirFile) return false;

  // error if already open
  if (isOpen()) return false;

  if (*path == '/') {
    while (*path == '/') path++;
    if (!dirFile->isRoot()) {
      if (!dir2.openRoot(dirFile->vol_)) return false;
      parent = &dir2;
    }
  }

  // Traverse the Long Directory Name Path until we get to the LEAF (long file name)
  while ((p = strchr(path, '/')) != NULL) {
    int8_t cb = p-path;

    memcpy(dname, path, cb);
    *(dname + cb) = 0;

    if (*(p + 1) == 0) {
      *newParent = *parent;
      return true;
    }

    if (!sub->open(parent, dname, FILE_READ, false)) {
      if (!bMakeDirs)
         return false;
      if (!sub->mkdir(parent, dname)) {
        return false;
      }
    }

    if (parent != dirFile) parent->close();
    parent = sub;
    sub = parent != &dir1 ? &dir1 : &dir2;
    path = p+1;
  }

  strcpy((char *)dname, path);

  *newParent = *parent;
  return true;

}

bool SdBaseFile::open(SdBaseFile* dirFile, const char * path, uint8_t oflag) {
  uint8_t dname[LONG_FILENAME_LENGTH + 1];
  SdBaseFile parent;

  if (openParentReturnFile(dirFile, path, dname, &parent, false)) {
    if (*dname == 0) return true;
    return open(&parent, dname, oflag, false);
  }

  return false;
}


uint8_t SdBaseFile::lfn_checksum(const unsigned char *pFCBName) {
  int i;
  unsigned char sum = 0;

  for (i = 11; i; i--)
    sum = ((sum & 1) << 7) + (sum >> 1) + *pFCBName++;

  return sum;
}

// open with filename in dname
bool SdBaseFile::open(SdBaseFile* dirFile, const uint8_t *dname, uint8_t oflag, bool bDir) {
  bool emptyFound = false;
  uint8_t index = 0;
  dir_t tempDir, *p = NULL;
  PGM_P tempPtr;
  char newName[SHORT_FILENAME_LENGTH + 2];
  bool bShortName = false;
  int8_t cVFATNeeded = -1, cVFATFoundCur;
  uint32_t wIndexPos = 0;
  uint8_t cbFilename;
  char *Filename = (char *)dname;

  vol_ = dirFile->vol_;
  dirFile->rewind();
  // search for file

  if (oflag & O_CREAT) {
    int8_t cb = strlen((char *)dname);
    bShortName = cb < 9;
    cVFATNeeded = (cb / 13) + (cb % 13 == 0 ? 0 : 1);
  }

  while ((p = dirFile->getLongFilename(p, card.tempLongFilename))) {
    //HAL::pingWatchdog();
    index = (0xF & ((dirFile->curPosition_ - 31) >> 5));
    if (RFstricmp(card.tempLongFilename, (char *)dname) == 0) {
      if (oflag & O_EXCL) return false;
      return openCachedEntry(index, oflag);
    }
  }

  // don't create unless O_CREAT and O_WRITE
  if (!(oflag & O_CREAT) || !(oflag & O_WRITE)) return false;

  dirFile->findSpace(&tempDir, cVFATNeeded, &cVFATFoundCur, &wIndexPos);
  if (wIndexPos != 0) {
    emptyFound = true;
    index = wIndexPos >> 5;
  }
  else {
    // only 512 entries allowed in FAT16 Root Fixed dir
    if (dirFile->type() == FAT_FILE_TYPE_ROOT_FIXED && (dirFile->curPosition_ >> 5) >= 512)
      return false;
    cVFATFoundCur = cVFATNeeded + 1;
    if (dirFile->curPosition_ > 0)
      wIndexPos = dirFile->curPosition_-32;
  }
  p = &tempDir;

  dirFile->flags_ |= O_WRITE;
  dirFile->seekSet(wIndexPos);

  // Create LONG FILE NAMES and LONG DIRECTORIES HERE
  // FILL IN MULTIPLE dir_t enteries..
  // DO a test and and make all files created have a long file name of "XXXXXXX <shortname>"
  if (!bShortName) {
    char *pExt, szExt[5];

    // Generate 8.3 from longfile name
    if ((pExt = strchr((char *)dname, '.')) != NULL) {
      strncpy(szExt, pExt, 4);
      szExt[4] = 0;
      if (pExt > (char*)dname + 6)
        pExt = (char*)dname + 6;
    }
    else {
      szExt[0] = 0;
      pExt = (char*)dname+6;
    }
    uint8_t cb = pExt-(char *)dname;
    memcpy(newName, dname, cb);
    newName[cb] = 0;
    strcat(newName, "~1");
    strcat(newName, szExt);
  }
  else
    strcpy(newName, (char *)dname);

  uint8_t checksum;

  make83Name(newName, (uint8_t *)p->name, &tempPtr);
  checksum = lfn_checksum((unsigned char *)p->name);
  cbFilename = strlen(Filename);

  // Write Long File Name VFAT entries to file
  for (uint8_t iBlk = cVFATNeeded; iBlk > 0; iBlk--) {
    vfat_t *VFAT = (vfat_t *)p;
    uint8_t n;

    n = (iBlk - 1) * 13;
    memset(p, 0, sizeof(*p));
    p->attributes = DIR_ATT_LONG_NAME;
    VFAT->sequenceNumber = iBlk | (iBlk == cVFATNeeded ? 0x40 : 0);

    uint16_t *pName = VFAT->name1;

    for(int8_t i = 0; i < 13; i++) {
      if (n+i > cbFilename)
        *pName++ = 0xFFFF;
      else
        *pName++ = (uint16_t)Filename[n + i];
      if (i == 4)
        pName = VFAT->name2;
      else if (i == 10)
        pName = VFAT->name3;
    }
    VFAT->checksum = checksum;
    if (dirFile->write(p, sizeof(dir_t)) < 0)
      return false;
    dirFile->sync();
  }
  // END WRITING LONG FILE NAME BLK

  // Start 8.3 file init
  // initialize as empty file
  memset(p, 0, sizeof(*p));

  make83Name(newName, (uint8_t *)p->name, &tempPtr);

  p->attributes = bDir ? DIR_ATT_DIRECTORY : DIR_ATT_ARCHIVE;

  p->creationDate = FAT_DEFAULT_DATE;
  p->creationTime = FAT_DEFAULT_TIME;
  p->lastAccessDate = p->creationDate;
  p->lastWriteDate = p->creationDate;
  p->lastWriteTime = p->creationTime;

  if (dirFile->write(p, sizeof(dir_t)) < 0) return false;

  dirFile->sync();

  memset(p, 0, sizeof(*p));

  if (emptyFound)
    p->name[0] = DIR_NAME_DELETED;

  for(int8_t i = 0; i < cVFATFoundCur - cVFATNeeded; i++) {
    if (dirFile->write(p, sizeof(dir_t)) < 0) return false;
    dirFile->sync();
  }

  return open(dirFile, (wIndexPos >> 5) + (cVFATNeeded), oflag & ~O_EXCL);

}

/** Open a file by index.
 *
 * \param[in] dirFile An open SdFat instance for the directory.
 *
 * \param[in] index The \a index of the directory entry for the file to be
 * opened.  The value for \a index is (directory file position)/32.
 *
 * \param[in] oflag Values for \a oflag are constructed by a bitwise-inclusive
 * OR of flags O_READ, O_WRITE, O_TRUNC, and O_SYNC.
 *
 * See open() by path for definition of flags.
 * \return true for success or false for failure.
 */
bool SdBaseFile::open(SdBaseFile* dirFile, uint16_t index, uint8_t oflag) {
  dir_t* p;

  vol_ = dirFile->vol_;

  // error if already open
  if (isOpen() || !dirFile) return false;

  // don't open existing file if O_EXCL - user call error
  if (oflag & O_EXCL) return false;

  // seek to location of entry
  if (!dirFile->seekSet(32 * index)) return false;

  // read entry into cache
  p = dirFile->readDirCache();
  if (!p) return false;

  // error if empty slot or '.' or '..'
  if (p->name[0] == DIR_NAME_FREE || p->name[0] == DIR_NAME_DELETED || p->name[0] == '.')
    return false;

  // open cached entry
  return openCachedEntry(index & 0xF, oflag);

}

// open a cached directory entry. Assumes vol_ is initialized
bool SdBaseFile::openCachedEntry(uint8_t dirIndex, uint8_t oflag) {
  // location of entry in cache
  dir_t* p = &vol_->cacheAddress()->dir[dirIndex];

  // write or truncate is an error for a directory or read-only file
  if (p->attributes & (DIR_ATT_READ_ONLY | DIR_ATT_DIRECTORY)) {
//    if (oflag & (O_WRITE | O_TRUNC))
//      return false;
  }
  // remember location of directory entry on SD
  dirBlock_ = vol_->cacheBlockNumber();
  dirIndex_ = dirIndex;

  // copy first cluster number for directory fields
  firstCluster_ = (uint32_t)p->firstClusterHigh << 16;
  firstCluster_ |= p->firstClusterLow;

  // make sure it is a normal file or subdirectory
  if (DIR_IS_FILE(p)) {
    fileSize_ = p->fileSize;
    type_ = FAT_FILE_TYPE_NORMAL;
  }
  else if (DIR_IS_SUBDIR(p)) {
    if (!setDirSize()) {
      fileSize_= 0;
    }
    type_ = FAT_FILE_TYPE_SUBDIR;
  }
  else {
    type_ = FAT_FILE_TYPE_CLOSED;
    return false;
  }
  // save open flags for read/write
  flags_ = oflag & F_OFLAG;

  // set to start of file
  curCluster_ = 0;
  curPosition_ = 0;
  if ((oflag & O_TRUNC) && !truncate(0)) {
    type_ = FAT_FILE_TYPE_CLOSED;
    return false;
  }

  return oflag & O_AT_END ? seekEnd(0) : true;

}

/** Open the next file or subdirectory in a directory.
 *
 * \param[in] dirFile An open SdFat instance for the directory containing the
 * file to be opened.
 *
 * \param[in] oflag Values for \a oflag are constructed by a bitwise-inclusive
 * OR of flags O_READ, O_WRITE, O_TRUNC, and O_SYNC.
 *
 * See open() by path for definition of flags.
 * \return true for success or false for failure.
 */
bool SdBaseFile::openNext(SdBaseFile* dirFile, uint8_t oflag) {
  dir_t* p;
  uint8_t index;

  if (!dirFile) return false;

  // error if already open
  if (isOpen()) return false;

  vol_ = dirFile->vol_;

  while (1) {
    index = 0xF & (dirFile->curPosition_ >> 5);

    // read entry into cache
    p = dirFile->readDirCache();
    if (!p) return false;

    // done if last entry
    if (p->name[0] == DIR_NAME_FREE) return false;

    // skip empty slot or '.' or '..'
    if (p->name[0] == DIR_NAME_DELETED || p->name[0] == '.') {
      continue;
    }
    // must be file or dir
    if (DIR_IS_FILE_OR_SUBDIR(p)) {
      return openCachedEntry(index, oflag);
    }
  }
}

/** Open a directory's parent directory.
 *
 * \param[in] dir Parent of this directory will be opened.  Must not be root.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
bool SdBaseFile::openParent(SdBaseFile* dir) {
  dir_t entry;
  dir_t* p;
  SdBaseFile file;
  uint32_t c;
  uint32_t cluster;
  uint32_t lbn;
  cache_t* pc;
  // error if already open or dir is root or dir is not a directory
  if (isOpen() || !dir || dir->isRoot() || !dir->isDir()) return false;

  vol_ = dir->vol_;
  // position to '..'
  if (!dir->seekSet(32)) return false;

  // read '..' entry
  if (dir->read(&entry, sizeof(entry)) != 32) return false;

  // verify it is '..'
  if (entry.name[0] != '.' || entry.name[1] != '.') return false;

  // start cluster for '..'
  cluster = entry.firstClusterLow;
  cluster |= (uint32_t)entry.firstClusterHigh << 16;
  if (cluster == 0) return openRoot(vol_);
  // start block for '..'
  lbn = vol_->clusterStartBlock(cluster);
  // first block of parent dir
  pc = vol_->cacheFetch(lbn, SdVolume::CACHE_FOR_READ);
  if (!pc) return false;

  p = &pc->dir[1];
  // verify name for '../..'
  if (p->name[0] != '.' || p->name[1] != '.') return false;

  // '..' is pointer to first cluster of parent. open '../..' to find parent
  if (p->firstClusterHigh == 0 && p->firstClusterLow == 0) {
    if (!file.openRoot(dir->volume())) return false;
  }
  else {
    if (!file.openCachedEntry(1, FILE_READ)) return false;
  }
  // search for parent in '../..'
  do {
    if (file.readDir(&entry) != 32) return false;
    c = entry.firstClusterLow;
    c |= (uint32_t)entry.firstClusterHigh << 16;
  } while (c != cluster);
  // open parent
  return open(&file, file.curPosition() / 32 - 1, FILE_READ);

}

/** Open a volume's root directory.
 *
 * \param[in] vol The FAT volume containing the root directory to be opened.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 * Reasons for failure include the file is already open, the FAT volume has
 * not been initialized or it a FAT12 volume.
 */
bool SdBaseFile::openRoot(SdVolume* vol) {
  // error if file is already open
  if (isOpen()) return false;

  vol_ = vol;
  if (vol->fatType() == 16 || (FAT12_SUPPORT && vol->fatType() == 12)) {
    type_ = FAT_FILE_TYPE_ROOT_FIXED;
    firstCluster_ = 0;
    fileSize_ = 32 * vol->rootDirEntryCount();
  }
  else if (vol->fatType() == 32) {
    type_ = FAT_FILE_TYPE_ROOT32;
    firstCluster_ = vol->rootDirStart();
    if (!setDirSize()) return false;
  }
  else
    return false;

  // read only
  flags_ = FILE_READ;

  // set to start of file
  curCluster_ = 0;
  curPosition_ = 0;

  // root has no directory entry
  dirBlock_ = 0;
  dirIndex_ = 0;
  return true;

}

/** Return the next available byte without consuming it.
 *
 * \return The byte if no error and not at eof else -1;
 */
int SdBaseFile::peek() {
  FatPos_t pos;
  getpos(&pos);
  int c = read();
  if (c >= 0) setpos(&pos);
  return c;
}

/** %Print the name field of a directory entry in 8.3 format.
 * \param[in] pr Print stream for output.
 * \param[in] dir The directory structure containing the name.
 * \param[in] width Blank fill name if length is less than \a width.
 * \param[in] printSlash Print '/' after directory names if true.
 */
void SdBaseFile::printDirName(const dir_t& dir, uint8_t width, bool printSlash) {
  uint8_t w = 0;
  for (uint8_t i = 0; i < 11; i++) {
    if (dir.name[i] == ' ')continue;
    if (i == 8) {
      SERIAL_CHR('.');
      w++;
    }
    SERIAL_CHR((char)dir.name[i]);
    w++;
  }
  if (DIR_IS_SUBDIR(&dir) && printSlash) {
    SERIAL_CHR('/');
    w++;
  }
  while (w < width) {
    SERIAL_CHR(' ');
    w++;
  }
}

// print uint8_t with width 2
static void print2u(uint8_t v) {
  if (v < 10) SERIAL_CHR('0');
  SERIAL_VAL((int)v);
}

/** Print a file's creation date and time
 *
 * \param[in] pr Print stream for output.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
bool SdBaseFile::printCreateDateTime() {
  dir_t dir;
  if (!dirEntry(&dir)) return false;
  printFatDate(dir.creationDate);
  SERIAL_CHR(' ');
  printFatTime(dir.creationTime);
  return true;
}


/** %Print a directory date field.
 *
 *  Format is yyyy-mm-dd.
 *
 * \param[in] pr Print stream for output.
 * \param[in] fatDate The date field from a directory entry.
 */
void SdBaseFile::printFatDate(uint16_t fatDate) {
  SERIAL_VAL((int)FAT_YEAR(fatDate));
  SERIAL_CHR('-');
  print2u(FAT_MONTH(fatDate));
  SERIAL_CHR('-');
  print2u(FAT_DAY(fatDate));
}


/** %Print a directory time field.
 *
 * Format is hh:mm:ss.
 *
 * \param[in] pr Print stream for output.
 * \param[in] fatTime The time field from a directory entry.
 */
void SdBaseFile::printFatTime(uint16_t fatTime) {
  print2u(FAT_HOUR(fatTime));
  SERIAL_CHR(':');
  print2u(FAT_MINUTE(fatTime));
  SERIAL_CHR(':');
  print2u(FAT_SECOND(fatTime));
}

/** Print a file's modify date and time
 *
 * \param[in] pr Print stream for output.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
bool SdBaseFile::printModifyDateTime() {
  dir_t dir;
  if (!dirEntry(&dir)) return false;
  printFatDate(dir.lastWriteDate);
  SERIAL_CHR(' ');
  printFatTime(dir.lastWriteTime);
  return true;
}
/** Template for SdBaseFile::printField() */

template <typename Type>

static int printFieldT(SdBaseFile* file, char sign, Type value, char term) {
  char buf[3*sizeof(Type) + 3];
  char* str = &buf[sizeof(buf)];

  if (term) {
    *--str = term;
    if (term == '\n') *--str = '\r';
  }

  do {
    Type m = value;
    value /= 10;
    *--str = '0' + m - 10*value;
  } while (value);

  if (sign) *--str = sign;
  
  return file->write(str, &buf[sizeof(buf)] - str);
}

/** Print a number followed by a field terminator.
 * \param[in] value The number to be printed.
 * \param[in] term The field terminator.  Use '\\n' for CR LF.
 * \return The number of bytes written or -1 if an error occurs.
 */
int SdBaseFile::printField(uint16_t value, char term) {
  return printFieldT(this, 0, value, term);
}

/** Print a number followed by a field terminator.
 * \param[in] value The number to be printed.
 * \param[in] term The field terminator.  Use '\\n' for CR LF.
 * \return The number of bytes written or -1 if an error occurs.
 */
int SdBaseFile::printField(int16_t value, char term) {
  char sign = 0;
  if (value < 0) {
    sign = '-';
    value = -value;
  }
  return printFieldT(this, sign, (uint16_t)value, term);
}

/** Print a number followed by a field terminator.
 * \param[in] value The number to be printed.
 * \param[in] term The field terminator.  Use '\\n' for CR LF.
 * \return The number of bytes written or -1 if an error occurs.
 */
int SdBaseFile::printField(uint32_t value, char term) {
  return printFieldT(this, 0, value, term);
}

/** Print a number followed by a field terminator.
 * \param[in] value The number to be printed.
 * \param[in] term The field terminator.  Use '\\n' for CR LF.
 * \return The number of bytes written or -1 if an error occurs.
 */
int SdBaseFile::printField(int32_t value, char term) {
  char sign = 0;
  if (value < 0) {
    sign = '-';
    value = -value;
  }
  return printFieldT(this, sign, (uint32_t)value, term);
}

//-----------------------------------------------------------------------------

/** Print a file's name
 *
 * \param[in] pr Print stream for output.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
bool SdBaseFile::printName() {
  char name[13];
  if (!getFilename(name)) return false;
  SERIAL_TXT(name);
  return true;
}

/** Read the next byte from a file.
 *
 * \return For success read returns the next byte in the file as an int.
 * If an error occurs or end of file is reached -1 is returned.
 */
int16_t SdBaseFile::read() {
  uint8_t b;
  return read(&b, 1) == 1 ? b : -1;
}

/** Read data from a file starting at the current position.
 *
 * \param[out] buf Pointer to the location that will receive the data.
 *
 * \param[in] nbyte Maximum number of bytes to read.
 *
 * \return For success read() returns the number of bytes read.
 * A value less than \a nbyte, including zero, will be returned
 * if end of file is reached.
 * If an error occurs, read() returns -1.  Possible errors include
 * read() called before a file has been opened, corrupt file system
 * or an I/O error occurred.
 */
int SdBaseFile::read(void* buf, size_t nbyte) {
  uint8_t blockOfCluster;
  uint8_t* dst = reinterpret_cast<uint8_t*>(buf);
  uint16_t offset;
  size_t toRead;
  uint32_t block;  // raw device block number
  cache_t* pc;

  // error if not open or write only
  if (!isOpen() || !(flags_ & FILE_READ)) return -1;

  // max bytes left in file
  if (nbyte >= (fileSize_ - curPosition_))
    nbyte = fileSize_ - curPosition_;

  // amount left to read
  toRead = nbyte;
  while (toRead > 0) {
    size_t n;
    offset = curPosition_ & 0x1FF;  // offset in block
    blockOfCluster = vol_->blockOfCluster(curPosition_);

    if (type_ == FAT_FILE_TYPE_ROOT_FIXED) {
      block = vol_->rootDirStart() + (curPosition_ >> 9);
    }
    else {
      if (offset == 0 && blockOfCluster == 0) {
        // start of new cluster
        if (curPosition_ == 0) {
          // use first cluster in file
          curCluster_ = firstCluster_;
        }
        else {
          // get next cluster from FAT
          if (!vol_->fatGet(curCluster_, &curCluster_)) return -1;
        }
      }
      block = vol_->clusterStartBlock(curCluster_) + blockOfCluster;
    }

    if (offset != 0 || toRead < 512 || block == vol_->cacheBlockNumber()) {
      // amount to be read from current block
      n = 512 - offset;
      if (n > toRead) n = toRead;
      // read block to cache and copy data to caller
      pc = vol_->cacheFetch(block, SdVolume::CACHE_FOR_READ);
      if (!pc) return -1;

      uint8_t* src = pc->data + offset;
      memcpy(dst, src, n);
    }
    else if (!USE_MULTI_BLOCK_SD_IO || toRead < 1024) {
      // read single block
      n = 512;
      if (!vol_->readBlock(block, dst)) return -1;
    }
    else {
      uint8_t nb = toRead >> 9;

      if (type_ != FAT_FILE_TYPE_ROOT_FIXED) {
        uint8_t mb = vol_->blocksPerCluster() - blockOfCluster;
        if (mb < nb) nb = mb;
      }

      n = 512 * nb;

      if (vol_->cacheBlockNumber() <= block
        && block < (vol_->cacheBlockNumber() + nb)) {
        // flush cache if a block is in the cache
        if (!vol_->cacheSync()) return -1;
      }

      if (!vol_->sdCard()->readStart(block)) return -1;

      for (uint8_t b = 0; b < nb; b++) {
        if (!vol_->sdCard()->readData(dst + b * 512)) return -1;
      }

      if (!vol_->sdCard()->readStop()) return -1;

    }

    dst += n;
    curPosition_ += n;
    toRead -= n;

  }

  return nbyte;

}

/** 
 * Convert the dir_t name field of the file (which contains blank fills)
 * into a proper filename string without spaces inside.
 *
 * buffer MUST be at least a 13 char array!
 */
void SdBaseFile::createFilename(char* buffer, const dir_t &dirEntry) {
  const uint8_t* src = dirEntry.name;
  for (uint8_t i = 0; i < 11; i++, src++) {
    if (*src == ' ') continue; // ignore spaces
    if (i == 8) *buffer++ = '.';
    *buffer++ = *src;
  }
  *buffer = 0; // close the string
}


/** Read the next directory entry from a directory file with the long filename
 *
 * \param[out] dir The dir_t struct that will receive the data.
 * \param[out] longFilename The long filename associated with the 8.3 name
 *
 * \return For success getLongFilename() returns a pointer to dir_t
 * A value of zero will be returned if end of file is reached.
 */
dir_t *SdBaseFile::getLongFilename(dir_t *dir, char *longFilename) {
  
  int16_t n;
  uint8_t bLastPart = true;
  uint8_t checksum  = 0;

  if (longFilename != NULL)
    *longFilename = 0;

  while (1) {
    //HAL::pingWatchdog();
    if (!(dir = readDirCache())) {
      return NULL;
    }

    if (dir->name[0] == DIR_NAME_FREE)
      return NULL;

    if (dir->name[0] == DIR_NAME_0xE5 || dir->name[0] == DIR_NAME_DELETED) {
      bLastPart = true;
      if (longFilename != NULL)
        *longFilename = 0;
      continue;
    }

    if (DIR_IS_LONG_NAME(dir)) {
     if (longFilename != NULL) {
        vfat_t *VFAT = (vfat_t*)dir;
        int8_t nSeq = VFAT->sequenceNumber & 0x1F;

        // Sanity check the VFAT entry. The first cluster is always set to zero. And the sequence number should be higher then 0
        if (VFAT->firstClusterLow == 0 && nSeq > 0 && nSeq <= MAX_VFAT_ENTRIES) {
          n = (nSeq - 1) * 13;

          longFilename[n + 0] = (char)VFAT->name1[0];
      		longFilename[n + 1] = (char)VFAT->name1[1];
      		longFilename[n + 2] = (char)VFAT->name1[2];
          longFilename[n + 3] = (char)VFAT->name1[3];
          longFilename[n + 4] = (char)VFAT->name1[4];
          longFilename[n + 5] = (char)VFAT->name2[0];
          longFilename[n + 6] = (char)VFAT->name2[1];
          longFilename[n + 7] = (char)VFAT->name2[2];
      		longFilename[n + 8] = (char)VFAT->name2[3];
      		longFilename[n + 9] = (char)VFAT->name2[4];
      		longFilename[n + 10] = (char)VFAT->name2[5];
      		longFilename[n + 11] = (char)VFAT->name3[0];
      		longFilename[n + 12] = (char)VFAT->name3[1];

          if (bLastPart) {
            checksum = VFAT->checksum;
            longFilename[n + 13] = 0;
          }
          bLastPart = false;
        }
      }
    }
    else {
      if (((dir->attributes & DIR_ATT_HIDDEN) || (dir->attributes & DIR_ATT_SYSTEM)) || (dir->name[0] == '.' && dir->name[1] != '.')) {
        bLastPart = true;
        if (longFilename != NULL)
          *longFilename = 0;
        continue;
      }
      if (DIR_IS_FILE(dir) || DIR_IS_SUBDIR(dir)) {
        if (longFilename && (bLastPart || checksum != lfn_checksum((unsigned char *)dir->name))) {
          createFilename(longFilename, *dir);
        }
        return dir;
      }
    }
  }
  return dir;
}

bool SdBaseFile::findSpace(dir_t *dir, int8_t cVFATNeeded, int8_t *pcVFATFound, uint32_t *pwIndexPos) {
  //int16_t n; // unused
  int8_t cVFATFound = 0;
  // if not a directory file or miss-positioned return an error
  if (!isDir()) return -1;

  rewind();

  while (1) {
    //HAL::pingWatchdog();
    dir = readDirCache();
    if (!dir) return false;
    // last entry if DIR_NAME_FREE
    if (dir->name[0] == DIR_NAME_FREE) return 0;
    // skip empty entries and entry for .  and ..
    if (dir->name[0] == DIR_NAME_0xE5 || dir->name[0] == DIR_NAME_DELETED) {
      if (DIR_IS_LONG_NAME(dir)) {
        //vfat_t *VFAT = (vfat_t*)dir; // unused
        cVFATFound++;
      }
      else {
        if (pwIndexPos != NULL && cVFATNeeded > 0 && cVFATFound >= cVFATNeeded && *pwIndexPos == 0) {
          *pwIndexPos = curPosition_-sizeof(dir_t)-(cVFATFound * sizeof(dir_t));
          *pcVFATFound = cVFATFound;
          return true;
        }
        cVFATFound++;
      }
    }
    else {
      cVFATFound = 0;
    }
  }
}


/** Read the next directory entry from a directory file.
 *
 * \param[out] dir The dir_t struct that will receive the data.
 *
 * \return For success readDir() returns the number of bytes read.
 * A value of zero will be returned if end of file is reached.
 * If an error occurs, readDir() returns -1.  Possible errors include
 * readDir() called before a directory has been opened, this is not
 * a directory file or an I/O error occurred.
 */

int8_t SdBaseFile::readDir(dir_t* dir) {
  int16_t n;
  // if not a directory file or miss-positioned return an error
  if (!isDir() || (0x1F & curPosition_)) return -1;

  while (1) {
    n = read(dir, sizeof(dir_t));
    if (n != sizeof(dir_t)) return n == 0 ? 0 : -1;
    // last entry if DIR_NAME_FREE
    if (dir->name[0] == DIR_NAME_FREE) return 0;
    // skip empty entries and entry for .  and ..
    if (dir->name[0] == DIR_NAME_DELETED || dir->name[0] == '.') continue;
    // return if normal file or subdirectory
    if (DIR_IS_FILE_OR_SUBDIR(dir)) return n;
  }
}

// Read next directory entry into the cache
// Assumes file is correctly positioned
dir_t* SdBaseFile::readDirCache() {
  uint8_t i;
  // error if not directory
  if (!isDir()) return 0;

  // index of entry in cache
  i = (curPosition_ >> 5) & 0xF;

  // use read to locate and cache block
  if (read() < 0) return 0;

  // advance to next entry
  curPosition_ += 31;

  // return pointer to entry
  return vol_->cacheAddress()->dir + i;

}

/** Remove a file.
 *
 * The directory entry and all data for the file are deleted.
 *
 * \note This function should not be used to delete the 8.3 version of a
 * file that has a long name. For example if a file has the long name
 * "New Text Document.txt" you should not delete the 8.3 name "NEWTEX~1.TXT".
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 * Reasons for failure include the file read-only, is a directory,
 * or an I/O error occurred.
 */
bool SdBaseFile::remove() {
  dir_t* d;
  // free any clusters - will FAIL if read-only or directory
  if (!truncate(0)) return false;

  // cache directory entry
  d = cacheDirEntry(SdVolume::CACHE_FOR_WRITE);
  if (!d) return false;

  // mark entry deleted
  d->name[0] = DIR_NAME_DELETED;

  // set this file closed
  type_ = FAT_FILE_TYPE_CLOSED;

  // write entry to SD
  return vol_->cacheSync();

}

/** Remove a file.
 *
 * The directory entry and all data for the file are deleted.
 *
 * \param[in] dirFile The directory that contains the file.
 * \param[in] path Path for the file to be removed.
 *
 * \note This function should not be used to delete the 8.3 version of a
 * file that has a long name. For example if a file has the long name
 * "New Text Document.txt" you should not delete the 8.3 name "NEWTEX~1.TXT".
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 * Reasons for failure include the file is a directory, is read only,
 * \a dirFile is not a directory, \a path is not found
 * or an I/O error occurred.
 */
bool SdBaseFile::remove(SdBaseFile* dirFile, PGM_P path) {
  SdBaseFile file;
  if (!file.open(dirFile, path, O_WRITE)) return false;
  return file.remove();
}

/** Rename a file or subdirectory.
 *
 * \param[in] dirFile Directory for the new path.
 * \param[in] newPath New path name for the file/directory.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 * Reasons for failure include \a dirFile is not open or is not a directory
 * file, newPath is invalid or already exists, or an I/O error occurs.
 */
bool SdBaseFile::rename(SdBaseFile* dirFile, PGM_P newPath) {
  dir_t entry;
  uint32_t dirCluster = 0;
  SdBaseFile file;
  cache_t* pc;
  dir_t* d;

  // must be an open file or subdirectory
  if (!(isFile() || isSubDir())) return false;

  // can't move file
  if (vol_ != dirFile->vol_) return false;

  // sync() and cache directory entry
  sync();
  d = cacheDirEntry(SdVolume::CACHE_FOR_WRITE);
  if (!d) return false;

  // save directory entry
  memcpy(&entry, d, sizeof(entry));

  // mark entry deleted
  d->name[0] = DIR_NAME_DELETED;

  // make directory entry for new path
  if (isFile()) {
    if (!file.open(dirFile, newPath, O_CREAT | O_EXCL | O_WRITE))
      goto restore;
  }
  else {
    // don't create missing path prefix components
    if (!file.mkdir(dirFile, newPath, false)) goto restore;
    // save cluster containing new dot dot
    dirCluster = file.firstCluster_;
  }
  // change to new directory entry
  dirBlock_ = file.dirBlock_;
  dirIndex_ = file.dirIndex_;

  // mark closed to avoid possible destructor close call
  file.type_ = FAT_FILE_TYPE_CLOSED;

  // cache new directory entry
  d = cacheDirEntry(SdVolume::CACHE_FOR_WRITE);
  if (!d) return false;

  // copy all but name field to new directory entry
  memcpy(&d->attributes, &entry.attributes, sizeof(entry) - sizeof(d->name));

  // update dot dot if directory
  if (dirCluster) {
    // get new dot dot
    uint32_t block = vol_->clusterStartBlock(dirCluster);
    pc = vol_->cacheFetch(block, SdVolume::CACHE_FOR_READ);
    if (!pc) return false;

    memcpy(&entry, &pc->dir[1], sizeof(entry));

    // free unused cluster
    if (!vol_->freeChain(dirCluster)) return false;

    // store new dot dot
    block = vol_->clusterStartBlock(firstCluster_);
    pc = vol_->cacheFetch(block, SdVolume::CACHE_FOR_WRITE);
    if (!pc) return false;

    memcpy(&pc->dir[1], &entry, sizeof(entry));
  }

  return vol_->cacheSync();

restore:
  d = cacheDirEntry(SdVolume::CACHE_FOR_WRITE);
  if (!d) return false;

  // restore entry
  d->name[0] = entry.name[0];
  vol_->cacheSync();

  return false;

}

/** Remove a directory file.
 *
 * The directory file will be removed only if it is empty and is not the
 * root directory.  rmdir() follows DOS and Windows and ignores the
 * read-only attribute for the directory.
 *
 * \note This function should not be used to delete the 8.3 version of a
 * directory that has a long name. For example if a directory has the
 * long name "New folder" you should not delete the 8.3 name "NEWFOL~1".
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 * Reasons for failure include the file is not a directory, is the root
 * directory, is not empty, or an I/O error occurred.
 */
bool SdBaseFile::rmdir() {
  // must be open subdirectory
  if (!isSubDir()) return false;

  rewind();

  // make sure directory is empty
  while (curPosition_ < fileSize_) {
    dir_t* p = readDirCache();
    if (!p) return false;

    // done if past last used entry
    if (p->name[0] == DIR_NAME_FREE) break;
    // skip empty slot, '.' or '..'
    if (p->name[0] == DIR_NAME_DELETED || p->name[0] == '.') continue;
    // error not empty
    if (DIR_IS_FILE_OR_SUBDIR(p)) return false;

  }

  // convert empty directory to normal file for remove
  type_ = FAT_FILE_TYPE_NORMAL;
  flags_ |= O_WRITE;
  return remove();

}

/** Recursively delete a directory and all contained files.
 *
 * This is like the Unix/Linux 'rm -rf *' if called with the root directory
 * hence the name.
 *
 * Warning - This will remove all contents of the directory including
 * subdirectories.  The directory will then be removed if it is not root.
 * The read-only attribute for files will be ignored.
 *
 * \note This function should not be used to delete the 8.3 version of
 * a directory that has a long name.  See remove() and rmdir().
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
bool SdBaseFile::rmRfStar() {
  uint32_t index;
  SdBaseFile f;

  rewind();

  while (curPosition_ < fileSize_) {
    // remember position
    index = curPosition_ / 32;

    dir_t* p = readDirCache();
    if (!p) return false;

    // done if past last entry
    if (p->name[0] == DIR_NAME_FREE) break;

    // skip empty slot or '.' or '..'
    if (p->name[0] == DIR_NAME_DELETED || p->name[0] == '.') continue;

    // skip if part of long file name or volume label in root
    if (!DIR_IS_FILE_OR_SUBDIR(p)) continue;

    if (!f.open(this, index, FILE_READ)) return false;

    if (f.isSubDir()) {
      // recursively delete
      if (!f.rmRfStar()) return false;
    }
    else {
      // ignore read-only
      f.flags_ |= O_WRITE;
      if (!f.remove()) return false;
    }
    // position to next entry if required
    if (curPosition_ != (32UL * (index + 1))) {
      if (!seekSet(32UL * (index + 1))) return false;
    }
  }

  // don't try to delete root
  if (!isRoot() && !rmdir()) return false;

  return true;

}

/**  Create a file object and open it in the current working directory.
 *
 * \param[in] path A path with a valid 8.3 DOS name for a file to be opened.
 *
 * \param[in] oflag Values for \a oflag are constructed by a bitwise-inclusive
 * OR of open flags. see SdBaseFile::open(SdBaseFile*, const char*, uint8_t).
 */
SdBaseFile::SdBaseFile(PGM_P path, uint8_t oflag) {
  type_ = FAT_FILE_TYPE_CLOSED;
  writeError = false;
  open(path, oflag);
}

/** Sets a file's position.
 *
 * \param[in] pos The new position in bytes from the beginning of the file.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
bool SdBaseFile::seekSet(uint32_t pos) {
  uint32_t nCur;
  uint32_t nNew;
  // error if file not open or seek past end of file
  if (!isOpen() || pos > fileSize_) return false;

  if (type_ == FAT_FILE_TYPE_ROOT_FIXED) {
    curPosition_ = pos;
    curCluster_ = 0;
    return true;
  }
  if (pos == 0) {
    // set position to start of file
    curCluster_ = 0;
    curPosition_ = 0;
    return true;
  }

  // calculate cluster index for cur and new position
  nCur = (curPosition_ - 1) >> (vol_->clusterSizeShift_ + 9);
  nNew = (pos - 1) >> (vol_->clusterSizeShift_ + 9);

  if (nNew < nCur || curPosition_ == 0) {
    // must follow chain from first cluster
    curCluster_ = firstCluster_;
  }
  else {
    // advance from curPosition
    nNew -= nCur;
  }

  while (nNew--) {
    if (!vol_->fatGet(curCluster_, &curCluster_)) return false;
  }

  curPosition_ = pos;

  return true;

}
// set fileSize_ for a directory
bool SdBaseFile::setDirSize() {
  uint16_t s = 0;
  uint32_t cluster = firstCluster_;
  do {
    if (!vol_->fatGet(cluster, &cluster)) return false;

    s += vol_->blocksPerCluster();
    // max size if a directory file is 4096 blocks
    if (s >= 4096) return false;

  } while (!vol_->isEOC(cluster));

  fileSize_ = 512L * s;
  return true;

}

//-----------------------------------------------------------------------------

void SdBaseFile::setpos(FatPos_t* pos) {
  curPosition_ = pos->position;
  curCluster_ = pos->cluster;
}

/** The sync() call causes all modified data and directory fields
 * to be written to the storage device.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 * Reasons for failure include a call to sync() before a file has been
 * opened or an I/O error.
 */
bool SdBaseFile::sync() {
  // only allow open files and directories
  if (!isOpen()) {
    writeError = true;
    return false;
  }

  if (flags_ & F_FILE_DIR_DIRTY) {
    dir_t* d = cacheDirEntry(SdVolume::CACHE_FOR_WRITE);
    // check for deleted by another open file object
    if (!d || d->name[0] == DIR_NAME_DELETED) {
      writeError = true;
      return false;
    }
    // do not set filesize for dir files
    if (!isDir()) d->fileSize = fileSize_;

    // update first cluster fields
    d->firstClusterLow = firstCluster_ & 0xFFFF;
    d->firstClusterHigh = firstCluster_ >> 16;

    // set modify time if user supplied a callback date/time function
    if (dateTime_) {
      dateTime_(&d->lastWriteDate, &d->lastWriteTime);
      d->lastAccessDate = d->lastWriteDate;
    }
    // clear directory dirty
    flags_ &= ~F_FILE_DIR_DIRTY;
  }
  return vol_->cacheSync();

}

/** Copy a file's timestamps
 *
 * \param[in] file File to copy timestamps from.
 *
 * \note
 * Modify and access timestamps may be overwritten if a date time callback
 * function has been set by dateTimeCallback().
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
bool SdBaseFile::timestamp(SdBaseFile* file) {
  dir_t* d;
  dir_t dir;

  // get timestamps
  if (!file->dirEntry(&dir)) return false;

  // update directory fields
  if (!sync()) return false;

  d = cacheDirEntry(SdVolume::CACHE_FOR_WRITE);
  if (!d) return false;

  // copy timestamps
  d->lastAccessDate = dir.lastAccessDate;
  d->creationDate = dir.creationDate;
  d->creationTime = dir.creationTime;
  d->creationTimeTenths = dir.creationTimeTenths;
  d->lastWriteDate = dir.lastWriteDate;
  d->lastWriteTime = dir.lastWriteTime;

  // write back entry
  return vol_->cacheSync();

}

/** Set a file's timestamps in its directory entry.
 *
 * \param[in] flags Values for \a flags are constructed by a bitwise-inclusive
 * OR of flags from the following list
 *
 * T_ACCESS - Set the file's last access date.
 *
 * T_CREATE - Set the file's creation date and time.
 *
 * T_WRITE - Set the file's last write/modification date and time.
 *
 * \param[in] year Valid range 1980 - 2107 inclusive.
 *
 * \param[in] month Valid range 1 - 12 inclusive.
 *
 * \param[in] day Valid range 1 - 31 inclusive.
 *
 * \param[in] hour Valid range 0 - 23 inclusive.
 *
 * \param[in] minute Valid range 0 - 59 inclusive.
 *
 * \param[in] second Valid range 0 - 59 inclusive
 *
 * \note It is possible to set an invalid date since there is no check for
 * the number of days in a month.
 *
 * \note
 * Modify and access timestamps may be overwritten if a date time callback
 * function has been set by dateTimeCallback().
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 */
bool SdBaseFile::timestamp(uint8_t flags, uint16_t year, uint8_t month,
                           uint8_t day, uint8_t hour, uint8_t minute, uint8_t second) {
  uint16_t dirDate;
  uint16_t dirTime;
  dir_t* d;

  if (!isOpen()
    || year < 1980
    || year > 2107
    || month < 1
    || month > 12
    || day < 1
    || day > 31
    || hour > 23
    || minute > 59
    || second > 59) return false;

  // update directory entry
  if (!sync()) return false;

  d = cacheDirEntry(SdVolume::CACHE_FOR_WRITE);
  if (!d) return false;

  dirDate = FAT_DATE(year, month, day);
  dirTime = FAT_TIME(hour, minute, second);

  if (flags & T_ACCESS)
    d->lastAccessDate = dirDate;

  if (flags & T_CREATE) {
    d->creationDate = dirDate;
    d->creationTime = dirTime;
    // seems to be units of 1/100 second not 1/10 as Microsoft states
    d->creationTimeTenths = second & 1 ? 100 : 0;
  }

  if (flags & T_WRITE) {
    d->lastWriteDate = dirDate;
    d->lastWriteTime = dirTime;
  }

  return vol_->cacheSync();

}

/** Truncate a file to a specified length.  The current file position
 * will be maintained if it is less than or equal to \a length otherwise
 * it will be set to end of file.
 *
 * \param[in] length The desired length for the file.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.
 * Reasons for failure include file is read only, file is a directory,
 * \a length is greater than the current file size or an I/O error occurs.
 */
bool SdBaseFile::truncate(uint32_t length) {
  uint32_t newPos;
  // error if not a normal file or read-only
  if (!isFile() || !(flags_ & O_WRITE)) return false;

  // error if length is greater than current size
  if (length > fileSize_) return false;

  // fileSize and length are zero - nothing to do
  if (fileSize_ == 0) return true;

  // remember position for seek after truncation
  newPos = curPosition_ > length ? length : curPosition_;

  // position to last cluster in truncated file
  if (!seekSet(length)) return false;

  if (length == 0) {
    // free all clusters
    if (!vol_->freeChain(firstCluster_)) return false;

    firstCluster_ = 0;
  }
  else {
    uint32_t toFree;
    if (!vol_->fatGet(curCluster_, &toFree)) return false;

    if (!vol_->isEOC(toFree)) {
      // free extra clusters
      if (!vol_->freeChain(toFree)) return false;
      // current cluster is end of chain
      if (!vol_->fatPutEOC(curCluster_)) return false;
    }
  }

  fileSize_ = length;

  // need to update directory entry
  flags_ |= F_FILE_DIR_DIRTY;

  if (!sync()) return false;

  // set file to correct position
  return seekSet(newPos);

}

/** Write data to an open file.
 *
 * \note Data is moved to the cache but may not be written to the
 * storage device until sync() is called.
 *
 * \param[in] buf Pointer to the location of the data to be written.
 *
 * \param[in] nbyte Number of bytes to write.
 *
 * \return For success write() returns the number of bytes written, always
 * \a nbyte.  If an error occurs, write() returns -1.  Possible errors
 * include write() is called before a file has been opened, write is called
 * for a read-only file, device is full, a corrupt file system or an I/O error.
 *
 */
int SdBaseFile::write(const void* buf, size_t nbyte) {
  // convert void* to uint8_t*  -  must be before goto statements
  const uint8_t* src = reinterpret_cast<const uint8_t*>(buf);
  cache_t* pc;
  uint8_t cacheOption;
  // number of bytes left to write  -  must be before goto statements
  size_t nToWrite = nbyte;
  size_t n;

  // error if not a normal file or is read-only
  if (/*!isFile() || */!(flags_ & O_WRITE)) {
    writeError = true;
    return -1;
  }

  // seek to end of file if append flag
  if ((flags_ & O_APPEND) && curPosition_ != fileSize_) {
    if (!seekEnd()) {
      writeError = true;
      return -1;
    }
  }

  while (nToWrite) {
    uint8_t blockOfCluster = vol_->blockOfCluster(curPosition_);
    uint16_t blockOffset = curPosition_ & 0x1FF;
    if (blockOfCluster == 0 && blockOffset == 0) {
      // start of new cluster
      if (curCluster_ != 0) {
        uint32_t next;
        if (!vol_->fatGet(curCluster_, &next)) {
          writeError = true;
          return -1;
        }
        if (vol_->isEOC(next)) {
          // add cluster if at end of chain
          if (!addCluster()) {
            writeError = true;
            return -1;
          }
        }
        else {
          curCluster_ = next;
        }
      } else {
        if (firstCluster_ == 0) {
          // allocate first cluster of file
          if (!addCluster()) {
            writeError = true;
            return -1;
          }
        }
        else {
          curCluster_ = firstCluster_;
        }
      }
    }
    // block for data write
    uint32_t block = type_ == FAT_FILE_TYPE_ROOT_FIXED ? vol_->rootDirStart() + (curPosition_ >> 9) : vol_->clusterStartBlock(curCluster_) + blockOfCluster;

    if (blockOffset != 0 || nToWrite < 512) {
      // partial block - must use cache
      // max space in block
      n = 512 - blockOffset;
      // lesser of space and amount to write
      if (n > nToWrite) n = nToWrite;

      if (blockOffset == 0 && curPosition_ >= fileSize_) {
        // start of new block don't need to read into cache
        cacheOption = SdVolume::CACHE_RESERVE_FOR_WRITE;
      }
      else {
        // rewrite part of block
        cacheOption = SdVolume::CACHE_FOR_WRITE;
      }

      pc = vol_->cacheFetch(block, cacheOption);
      if (!pc) {
        writeError = true;
        return -1;
      }

      uint8_t* dst = pc->data + blockOffset;
      memcpy(dst, src, n);
      if (512 == (n + blockOffset)) {
        if (!vol_->cacheWriteData()) {
          writeError = true;
          return -1;
        }
      }
    }
    else if (!USE_MULTI_BLOCK_SD_IO || nToWrite < 1024) {
      // use single block write command
      n = 512;
      if (vol_->cacheBlockNumber() == block)
        vol_->cacheInvalidate();
      if (!vol_->writeBlock(block, src)) {
        writeError = true;
        return -1;
      }
    }
    else {
      // use multiple block write command
      uint8_t maxBlocks = vol_->blocksPerCluster() - blockOfCluster;
      uint8_t nBlock = nToWrite >> 9;
      if (nBlock > maxBlocks) nBlock = maxBlocks;

      n = 512 * nBlock;
      if (!vol_->sdCard()->writeStart(block, nBlock)) {
        writeError = true;
        return -1;
      }
      for (uint8_t b = 0; b < nBlock; b++) {
        // invalidate cache if block is in cache
        if ((block + b) == vol_->cacheBlockNumber()) {
          vol_->cacheInvalidate();
        }
        if (!vol_->sdCard()->writeData(src + 512*b)) {
          writeError = true;
          return -1;
        }
      }
      if (!vol_->sdCard()->writeStop()) {
        writeError = true;
        return -1;
      }
    }

    curPosition_ += n;
    src += n;
    nToWrite -= n;
  }

  if (curPosition_ > fileSize_) {
    // update fileSize and insure sync will update dir entry
    fileSize_ = curPosition_;
    flags_ |= F_FILE_DIR_DIRTY;
  }
  else if (dateTime_ && nbyte) {
    // insure sync will update modified date and time
    flags_ |= F_FILE_DIR_DIRTY;
  }

  if (flags_ & O_SYNC) {
    if (!sync()) {
      writeError = true;
      return -1;
    }
  }

  return nbyte;

}

#endif // HAS_SD_SUPPORT
