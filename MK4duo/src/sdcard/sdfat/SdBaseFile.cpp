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

// print uint8_t with width 2
static void print2u(uint8_t v) {
  if (v < 10) SERIAL_CHR('0');
  SERIAL_VAL((int)v);
}

/** Public Parameters */
SdBaseFile* SdBaseFile::cwd_ = 0;   // Pointer to Current Working Directory

/** Public Function */
bool SdBaseFile::getFilename(char * const name) {
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

bool SdBaseFile::seekSet(const uint32_t pos) {
  uint32_t nCur, nNew;
  // error if file not open or seek past end of file
  if (!isOpen() || pos > fileSize_) return false;

  if (type_ == FAT_FILE_TYPE_ROOT_FIXED) {
    curPosition_ = pos;
    return true;
  }
  if (pos == 0) {
    curCluster_ = curPosition_ = 0;   // set position to start of file
    return true;
  }

  // calculate cluster index for cur and new position
  nCur = (curPosition_ - 1) >> (vol_->clusterSizeShift_ + 9);
  nNew = (pos - 1) >> (vol_->clusterSizeShift_ + 9);

  if (nNew < nCur || curPosition_ == 0)
    curCluster_ = firstCluster_;      // must follow chain from first cluster
  else
    nNew -= nCur;                     // advance from curPosition

  while (nNew--)
    if (!vol_->fatGet(curCluster_, &curCluster_)) return false;

  curPosition_ = pos;
  return true;
}

bool SdBaseFile::sync() {
  // only allow open files and directories
  if (!isOpen()) goto FAIL;

  if (flags_ & F_FILE_DIR_DIRTY) {
    dir_t* d = cacheDirEntry(SdVolume::CACHE_FOR_WRITE);
    // check for deleted by another open file object
    if (!d || d->name[0] == DIR_NAME_DELETED) goto FAIL;

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
  return vol_->cacheFlush();

FAIL:
  writeError = true;
  return false;
}

bool SdBaseFile::mkdir(SdBaseFile* parent, const char * path, bool pFlag) {
  uint8_t dname[LONG_FILENAME_LENGTH+1];
  SdBaseFile newParent;
  if (openParentReturnFile(parent, path, dname, &newParent, pFlag))
    return mkdir(&newParent, dname);
  return false;
}

bool SdBaseFile::open(const char * path, uint8_t oflag) {
  return open(cwd_, path, oflag);
}

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
  if (p->name[0] == DIR_NAME_FREE ||
      p->name[0] == DIR_NAME_DELETED || p->name[0] == '.') {
    return false;
  }
  // open cached entry
  return openCachedEntry(index & 0xF, oflag);
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
  return false;
}

bool SdBaseFile::openParent(SdBaseFile* dir) {
  dir_t entry;
  dir_t* p;
  SdBaseFile file;
  uint32_t c;
  uint32_t cluster;
  uint32_t lbn;
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
  if (!vol_->cacheRawBlock(lbn, SdVolume::CACHE_FOR_READ)) return false;

  p = &vol_->cacheBuffer_.dir[1];
  // verify name for '../..'
  if (p->name[0] != '.' || p->name[1] != '.') return false;
  // '..' is pointer to first cluster of parent. open '../..' to find parent
  if (p->firstClusterHigh == 0 && p->firstClusterLow == 0) {
    if (!file.openRoot(dir->volume())) return false;
  }
  else if (!file.openCachedEntry(1, FILE_READ))
    return false;

  // search for parent in '../..'
  do {
    if (file.readDir(&entry) != 32) return false;
    c = entry.firstClusterLow;
    c |= (uint32_t)entry.firstClusterHigh << 16;
  } while (c != cluster);

  // open parent
  return open(&file, file.curPosition() / 32 - 1, FILE_READ);
}

bool SdBaseFile::openRoot(SdVolume* vol) {
  // error if file is already open
  if (isOpen()) return false;

  if (vol->fatType() == 16 || (FAT12_SUPPORT && vol->fatType() == 12)) {
    type_ = FAT_FILE_TYPE_ROOT_FIXED;
    firstCluster_ = 0;
    fileSize_ = 32 * vol->rootDirEntryCount();
  }
  else if (vol->fatType() == 32) {
    type_ = FAT_FILE_TYPE_ROOT32;
    firstCluster_ = vol->rootDirStart();
    if (!vol->chainSize(firstCluster_, &fileSize_)) return false;
  }
  else // volume is not initialized, invalid, or FAT12 without support
    return false;

  vol_ = vol;
  // read only
  flags_ = O_READ;

  // set to start of file
  curCluster_ = curPosition_ = 0;

  // root has no directory entry
  dirBlock_ = dirIndex_ = 0;
  return true;
}

bool SdBaseFile::close() {
  bool rtn = sync();
  type_ = FAT_FILE_TYPE_CLOSED;
  return rtn;
}

int16_t SdBaseFile::read() {
  uint8_t b;
  return read(&b, 1) == 1 ? b : -1;
}

int16_t SdBaseFile::read(void* buf, uint16_t nbyte) {
  uint8_t* dst = reinterpret_cast<uint8_t*>(buf);
  uint16_t offset, toRead;
  uint32_t block;  // raw device block number

  // error if not open or write only
  if (!isOpen() || !(flags_ & O_READ)) return -1;

  // max bytes left in file
  NOMORE(nbyte, fileSize_ - curPosition_);

  // amount left to read
  toRead = nbyte;
  while (toRead > 0) {
    offset = curPosition_ & 0x1FF;  // offset in block
    if (type_ == FAT_FILE_TYPE_ROOT_FIXED) {
      block = vol_->rootDirStart() + (curPosition_ >> 9);
    }
    else {
      uint8_t blockOfCluster = vol_->blockOfCluster(curPosition_);
      if (offset == 0 && blockOfCluster == 0) {
        // start of new cluster
        if (curPosition_ == 0)
          curCluster_ = firstCluster_;                      // use first cluster in file
        else if (!vol_->fatGet(curCluster_, &curCluster_))  // get next cluster from FAT
          return -1;
      }
      block = vol_->clusterStartBlock(curCluster_) + blockOfCluster;
    }
    uint16_t n = toRead;

    // amount to be read from current block
    NOMORE(n, 512 - offset);

    // no buffering needed if n == 512
    if (n == 512 && block != vol_->cacheBlockNumber()) {
      if (!vol_->readBlock(block, dst)) return -1;
    }
    else {
      // read block to cache and copy data to caller
      if (!vol_->cacheRawBlock(block, SdVolume::CACHE_FOR_READ)) return -1;
      uint8_t* src = vol_->cache()->data + offset;
      memcpy(dst, src, n);
    }
    dst += n;
    curPosition_ += n;
    toRead -= n;
  }
  return nbyte;
}

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

int16_t SdBaseFile::write(const void * buf, uint16_t nbyte) {
  // convert void* to uint8_t*  -  must be before goto statements
  const uint8_t* src = reinterpret_cast<const uint8_t*>(buf);

  // number of bytes left to write  -  must be before goto statements
  uint16_t nToWrite = nbyte;

  // error if not a normal file or is read-only
  if (!isFile() || !(flags_ & O_WRITE)) goto FAIL;

  // seek to end of file if append flag
  if ((flags_ & O_APPEND) && curPosition_ != fileSize_) {
    if (!seekEnd()) goto FAIL;
  }

  while (nToWrite > 0) {
    uint8_t blockOfCluster = vol_->blockOfCluster(curPosition_);
    uint16_t blockOffset = curPosition_ & 0x1FF;
    if (blockOfCluster == 0 && blockOffset == 0) {
      // start of new cluster
      if (curCluster_ == 0) {
        if (firstCluster_ == 0) {
          // allocate first cluster of file
          if (!addCluster()) goto FAIL;
        }
        else {
          curCluster_ = firstCluster_;
        }
      }
      else {
        uint32_t next;
        if (!vol_->fatGet(curCluster_, &next)) goto FAIL;
        if (vol_->isEOC(next)) {
          // add cluster if at end of chain
          if (!addCluster()) goto FAIL;
        }
        else {
          curCluster_ = next;
        }
      }
    }
    // max space in block
    uint16_t n = 512 - blockOffset;

    // lesser of space and amount to write
    NOMORE(n, nToWrite);

    // block for data write
    uint32_t block = vol_->clusterStartBlock(curCluster_) + blockOfCluster;
    if (n == 512) {
      // full block - don't need to use cache
      if (vol_->cacheBlockNumber() == block) {
        // invalidate cache if block is in cache
        vol_->cacheSetBlockNumber(0xFFFFFFFF, false);
      }
      if (!vol_->writeBlock(block, src)) goto FAIL;
    }
    else {
      if (blockOffset == 0 && curPosition_ >= fileSize_) {
        // start of new block don't need to read into cache
        if (!vol_->cacheFlush()) goto FAIL;
        // set cache dirty and SD address of block
        vol_->cacheSetBlockNumber(block, true);
      }
      else {
        // rewrite part of block
        if (!vol_->cacheRawBlock(block, SdVolume::CACHE_FOR_WRITE)) goto FAIL;
      }
      uint8_t* dst = vol_->cache()->data + blockOffset;
      memcpy(dst, src, n);
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
    if (!sync()) goto FAIL;
  }
  return nbyte;

FAIL:
  // return for write error
  writeError = true;
  return -1;
}

bool SdBaseFile::remove() {
  dir_t* d;
  // free any clusters - will fail if read-only or directory
  if (!truncate(0)) return false;

  // cache directory entry
  d = cacheDirEntry(SdVolume::CACHE_FOR_WRITE);
  if (!d) return false;

  // mark entry deleted
  d->name[0] = DIR_NAME_DELETED;

  // set this file closed
  type_ = FAT_FILE_TYPE_CLOSED;

  // write entry to SD
  return vol_->cacheFlush();
  return true;
}

bool SdBaseFile::remove(SdBaseFile* dirFile, const char * path) {
  SdBaseFile file;
  return file.open(dirFile, path, O_WRITE) ? file.remove() : false;
}

void SdBaseFile::ls() {
  SdBaseFile parent;
  rewind();
  *card.fileName = 0;
  pathend = card.fileName;
  parent = *this;
  lsRecursive(&parent, 0, NULL, NULL, false);
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
            SERIAL_MV(" ", (long)p->fileSize);
          #endif
          SERIAL_EOL();
        }
      }
    }
  }
  return false;
}

dir_t* SdBaseFile::getLongFilename(dir_t* dir, char* longFilename) {
  
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

void SdBaseFile::printFatDate(uint16_t fatDate) {
  SERIAL_VAL((int)FAT_YEAR(fatDate));
  SERIAL_CHR('-');
  print2u(FAT_MONTH(fatDate));
  SERIAL_CHR('-');
  print2u(FAT_DAY(fatDate));
}

void SdBaseFile::printFatTime(uint16_t fatTime) {
  print2u(FAT_HOUR(fatTime));
  SERIAL_CHR(':');
  print2u(FAT_MINUTE(fatTime));
  SERIAL_CHR(':');
  print2u(FAT_SECOND(fatTime));
}

bool SdBaseFile::printModifyDateTime() {
  dir_t dir;
  if (!dirEntry(&dir)) return false;
  printFatDate(dir.lastWriteDate);
  SERIAL_CHR(' ');
  printFatTime(dir.lastWriteTime);
  return true;
}

bool SdBaseFile::printName() {
  char name[FILENAME_LENGTH];
  if (!getFilename(name)) return false;
  SERIAL_TXT(name);
  return true;
}

/** Private Function */
void SdBaseFile::getpos(FatPos_t* pos) {
  pos->position = curPosition_;
  pos->cluster = curCluster_;
}

void SdBaseFile::setpos(FatPos_t* pos) {
  curPosition_ = pos->position;
  curCluster_ = pos->cluster;
}

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

void SdBaseFile::dirName(const dir_t& dir, char* name) {
  uint8_t j = 0;
  for (uint8_t i = 0; i < 11; i++) {
    if (dir.name[i] == ' ')continue;
    if (i == 8) name[j++] = '.';
    name[j++] = dir.name[i];
  }
  name[j] = 0;
}

bool SdBaseFile::exists(const char * name) {
  SdBaseFile file;
  return file.open(this, name, O_READ);
}

uint8_t SdBaseFile::lfn_checksum(const unsigned char *pFCBName) {
  int i;
  unsigned char sum = 0;

  for (i = 11; i; i--)
    sum = ((sum & 1) << 7) + (sum >> 1) + *pFCBName++;

  return sum;
}

bool SdBaseFile::openParentReturnFile(SdBaseFile* dirFile, const char * path, uint8_t *dname, SdBaseFile *newParent, bool bMakeDirs) {
  SdBaseFile dir1, dir2;
  SdBaseFile* parent = dirFile;
  SdBaseFile* sub = &dir1;
  char *p;

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
    int8_t cb = p - path;

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

int SdBaseFile::peek() {
  FatPos_t pos;
  getpos(&pos);
  int c = read();
  if (c >= 0) setpos(&pos);
  return c;
}

bool SdBaseFile::printCreateDateTime() {
  dir_t dir;
  if (!dirEntry(&dir)) return false;
  printFatDate(dir.creationDate);
  SERIAL_CHR(' ');
  printFatTime(dir.creationTime);
  return true;
}

/**
 * Template for SdBaseFile::printField()
 */
template <typename Type>
static int printFieldT(SdBaseFile* file, char sign, Type value, char term) {
  char buf[3*sizeof(Type) + 3];
  char* str = &buf[sizeof(buf)];

  if (term) {
    *--str = term;
    if (term == '\n') {
      *--str = '\r';
    }
  }
  do {
    Type m = value;
    value /= 10;
    *--str = '0' + m - 10*value;
  } while (value);
  if (sign) {
    *--str = sign;
  }
  return file->write(str, &buf[sizeof(buf)] - str);
}

int SdBaseFile::printField(uint16_t value, char term) {
  return printFieldT(this, 0, value, term);
}
int SdBaseFile::printField(int16_t value, char term) {
  char sign = 0;
  if (value < 0) {
    sign = '-';
    value = -value;
  }
  return printFieldT(this, sign, (uint16_t)value, term);
}
int SdBaseFile::printField(uint32_t value, char term) {
  return printFieldT(this, 0, value, term);
}
int SdBaseFile::printField(int32_t value, char term) {
  char sign = 0;
  if (value < 0) {
    sign = '-';
    value = -value;
  }
  return printFieldT(this, sign, (uint32_t)value, term);
}

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

    if (!f.open(this, index, O_READ)) return false;
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
    if (curPosition_ != (32 * (index + 1))) {
      if (!seekSet(32 * (index + 1))) return false;
    }
  }
  // don't try to delete root
  if (!isRoot()) {
    if (!rmdir()) return false;
  }
  return true;
}

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
  return vol_->cacheFlush();
}

bool SdBaseFile::timestamp(uint8_t flags, uint16_t year, uint8_t month,
                           uint8_t day, uint8_t hour, uint8_t minute, uint8_t second) {
  uint16_t dirDate, dirTime;
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
      || second > 59) {
    return false;
  }
  // update directory entry
  if (!sync()) return false;

  d = cacheDirEntry(SdVolume::CACHE_FOR_WRITE);
  if (!d) return false;

  dirDate = FAT_DATE(year, month, day);
  dirTime = FAT_TIME(hour, minute, second);
  if (flags & T_ACCESS) {
    d->lastAccessDate = dirDate;
  }
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
  return vol_->cacheFlush();
}

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

// callback function for date/time
void (*SdBaseFile::dateTime_)(uint16_t* date, uint16_t* time) = 0;

bool SdBaseFile::addCluster() {
  if (!vol_->allocContiguous(1, &curCluster_)) return false;

  // if first cluster of file link to directory entry
  if (firstCluster_ == 0) {
    firstCluster_ = curCluster_;
    flags_ |= F_FILE_DIR_DIRTY;
  }
  return true;
}

bool SdBaseFile::addDirCluster() {
  uint32_t block;
  // max folder size
  if (fileSize_ / sizeof(dir_t) >= 0xFFFF) return false;

  if (!addCluster()) return false;
  if (!vol_->cacheFlush()) return false;

  block = vol_->clusterStartBlock(curCluster_);

  // set cache to first block of cluster
  vol_->cacheSetBlockNumber(block, true);

  // zero first block of cluster
  memset(vol_->cacheBuffer_.data, 0, 512);

  // zero rest of cluster
  for (uint8_t i = 1; i < vol_->blocksPerCluster_; i++) {
    if (!vol_->writeBlock(block + i, vol_->cacheBuffer_.data)) return false;
  }
  // Increase directory file size by cluster size
  fileSize_ += 512UL << vol_->clusterSizeShift_;
  return true;
}

dir_t* SdBaseFile::cacheDirEntry(uint8_t action) {
  if (!vol_->cacheRawBlock(dirBlock_, action)) return NULL;
  return vol_->cache()->dir + dirIndex_;
}

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

bool SdBaseFile::make83Name(const char* str, uint8_t* name, const char** ptr) {
  uint8_t n = 7,                      // Max index until a dot is found
          i = 11;
  while (i) name[--i] = ' ';          // Set whole FILENAME.EXT to spaces
  while (*str && *str != '/') {       // For each character, until nul or '/'
    uint8_t c = *str++;               // Get char and advance
    if (c == '.') {                   // For a dot...
      if (n == 10) return false;      // Already moved the max index? fail!
      n = 10;                         // Move the max index for full 8.3 name
      i = 8;                          // Move up to the extension place
    }
    else {
      // Fail for illegal characters
      PGM_P p = PSTR("|<>^+=?/[];,*\"\\");
      while (uint8_t b = pgm_read_byte(p++)) if (b == c) return false;
      if (i > n || c < 0x21 || c == 0x7F) return false;           // Check size, non-printable characters
      name[i++] = (c < 'a' || c > 'z') ? (c) : (c + ('A' - 'a')); // Uppercase required for 8.3 name
    }
  }
  *ptr = str;                         // Set passed pointer to the end
  return name[0] != ' ';              // Return true if any name was set
}

bool SdBaseFile::mkdir(SdBaseFile* parent, const uint8_t * dname) {
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

  if (write(&d, sizeof(dir_t)) < 0) return false;

  sync();

  memset(&d, 0, sizeof(dir_t));

  if (write(&d, sizeof(dir_t)) < 0) return false;

  sync();

  type_ = FAT_FILE_TYPE_SUBDIR;
  flags_ |= F_FILE_DIR_DIRTY;
  return true;
}

void SdBaseFile::createFilename(char* buffer, const dir_t &dirEntry) {
  const uint8_t* src = dirEntry.name;
  for (uint8_t i = 0; i < 11; i++, src++) {
    if (*src == ' ') continue; // ignore spaces
    if (i == 8) *buffer++ = '.';
    *buffer++ = *src;
  }
  *buffer = 0; // close the string
}

bool SdBaseFile::findSpace(dir_t* dir, int8_t cVFATNeeded, int8_t* pcVFATFound, uint32_t* pwIndexPos) {
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
    if (dirFile->type() == FAT_FILE_TYPE_ROOT_FIXED && (dirFile->curPosition_ >> 5) >= 512) return false;
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
  else {
    strcpy(newName, (char *)dname);
  }

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
    if (dirFile->write(p, sizeof(dir_t)) < 0) return false;
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

// open a cached directory entry. Assumes vol_ is initialized
bool SdBaseFile::openCachedEntry(uint8_t dirIndex, uint8_t oflag) {
  // location of entry in cache
  dir_t* p = &vol_->cache()->dir[dirIndex];

  // write or truncate is an error for a directory or read-only file
  if (p->attributes & (DIR_ATT_READ_ONLY | DIR_ATT_DIRECTORY)) {
    if (oflag & (O_WRITE | O_TRUNC)) goto FAIL;
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
    if (!vol_->chainSize(firstCluster_, &fileSize_)) goto FAIL;
    type_ = FAT_FILE_TYPE_SUBDIR;
  }
  else
    goto FAIL;

  // save open flags for read/write
  flags_ = oflag & (O_ACCMODE | O_SYNC | O_APPEND);

  // set to start of file
  curCluster_ = 0;
  curPosition_ = 0;
  if ((oflag & O_TRUNC) && !truncate(0)) return false;
  return oflag & O_AT_END ? seekEnd(0) : true;

FAIL:
  type_ = FAT_FILE_TYPE_CLOSED;
  return false;
}

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
  return vol_->cache()->dir + i;
}

/**
 * Create a file object and open it in the current working directory.
 *
 * \param[in] path A path with a valid 8.3 DOS name for a file to be opened.
 *
 * \param[in] oflag Values for \a oflag are constructed by a bitwise-inclusive
 * OR of open flags. see SdBaseFile::open(SdBaseFile*, const char*, uint8_t).
 */
SdBaseFile::SdBaseFile(const char* path, uint8_t oflag) {
  type_ = FAT_FILE_TYPE_CLOSED;
  writeError = false;
  open(path, oflag);
}

#endif // HAS_SD_SUPPORT
