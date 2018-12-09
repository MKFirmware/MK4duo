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

#include "SdVolume.h"

cache_t  SdVolume::cacheBuffer_;       // 512 byte cache for Sd2Card
uint32_t SdVolume::cacheBlockNumber_;  // current block number
uint8_t  SdVolume::cacheStatus_;       // status of cache block
uint32_t SdVolume::cacheFatOffset_;    // offset for mirrored FAT
#if USE_SEPARATE_FAT_CACHE
  cache_t  SdVolume::cacheFatBuffer_;       // 512 byte cache for FAT
  uint32_t SdVolume::cacheFatBlockNumber_;  // current Fat block number
  uint8_t  SdVolume::cacheFatStatus_;       // status of cache Fatblock
#endif  // USE_SEPARATE_FAT_CACHE
Sd2Card* SdVolume::sdCard_;            // pointer to SD card object

// find a contiguous group of clusters
bool SdVolume::allocContiguous(uint32_t count, uint32_t* curCluster) {
  // start of group
  uint32_t bgnCluster;
  // end of group
  uint32_t endCluster;
  // last cluster of FAT
  uint32_t fatEnd = clusterCount_ + 1;

  // flag to save place to start next search
  bool setStart;

  // set search start cluster
  if (*curCluster) {
    // try to make file contiguous
    bgnCluster = *curCluster + 1;

    // don't save new start location
    setStart = false;
  }
  else {
    // start at likely place for free cluster
    bgnCluster = allocSearchStart_;

    // save next search start if one cluster
    setStart = count == 1;
  }
  // end of group
  endCluster = bgnCluster;

  // search the FAT for free clusters
  for (uint32_t n = 0;; n++, endCluster++) {
    // can't find space checked all clusters
    if (n >= clusterCount_) return false;

    // past end - start from beginning of FAT
    if (endCluster > fatEnd) {
      bgnCluster = endCluster = 2;
    }

    uint32_t f;
    if (!fatGet(endCluster, &f)) return false;

    if (f != 0) {
      // cluster in use try next cluster as bgnCluster
      bgnCluster = endCluster + 1;
    }
    else if ((endCluster - bgnCluster + 1) == count) {
      // done - found space
      break;
    }
  }

  // mark end of chain
  if (!fatPutEOC(endCluster)) return false;

  // link clusters
  while (endCluster > bgnCluster) {
    if (!fatPut(endCluster - 1, endCluster)) return false;
    endCluster--;
  }

  if (*curCluster != 0 && !fatPut(*curCluster, bgnCluster)) return false;

  // return first cluster number to caller
  *curCluster = bgnCluster;
  // remember possible next free cluster
  if (setStart) allocSearchStart_ = bgnCluster + 1;

  return true;

}

#if USE_SEPARATE_FAT_CACHE

  cache_t* SdVolume::cacheFetch(uint32_t blockNumber, uint8_t options) {
    return cacheFetchData(blockNumber, options);
  }

  cache_t* SdVolume::cacheFetchData(uint32_t blockNumber, uint8_t options) {
    if (cacheBlockNumber_ != blockNumber) {
      if (!cacheWriteData()) return 0;

      if (!(options & CACHE_OPTION_NO_READ)) {
        if (!sdCard_->readBlock(blockNumber, cacheBuffer_.data)) return 0;
      }
      cacheStatus_ = 0;
      cacheBlockNumber_ = blockNumber;
    }

    cacheStatus_ |= options & CACHE_STATUS_MASK;

    return &cacheBuffer_;

  }

  cache_t* SdVolume::cacheFetchFat(uint32_t blockNumber, uint8_t options) {
    if (cacheFatBlockNumber_ != blockNumber) {
      if (!cacheWriteFat()) return 0;

      if (!(options & CACHE_OPTION_NO_READ)) {
        if (!sdCard_->readBlock(blockNumber, cacheFatBuffer_.data)) return 0;
      }

      cacheFatStatus_ = 0;
      cacheFatBlockNumber_ = blockNumber;
    }

    cacheFatStatus_ |= options & CACHE_STATUS_MASK;

    return &cacheFatBuffer_;

  }

  bool SdVolume::cacheSync() {
    return cacheWriteData() && cacheWriteFat();
  }

  bool SdVolume::cacheWriteData() {
    if (cacheStatus_ & CACHE_STATUS_DIRTY) {
      if (!sdCard_->writeBlock(cacheBlockNumber_, cacheBuffer_.data)) return false;
      cacheStatus_ &= ~CACHE_STATUS_DIRTY;
    }
    return true;
  }

  bool SdVolume::cacheWriteFat() {
    if (cacheFatStatus_ & CACHE_STATUS_DIRTY) {
      if (!sdCard_->writeBlock(cacheFatBlockNumber_, cacheFatBuffer_.data)) return false;

      // mirror second FAT
      if (cacheFatOffset_) {
        uint32_t lbn = cacheFatBlockNumber_ + cacheFatOffset_;
        if (!sdCard_->writeBlock(lbn, cacheFatBuffer_.data)) return false;
      }
      cacheFatStatus_ &= ~CACHE_STATUS_DIRTY;
    }
    return true;
  }

#else  // USE_SEPARATE_FAT_CACHE

  cache_t* SdVolume::cacheFetch(uint32_t blockNumber, uint8_t options) {
    if (cacheBlockNumber_ != blockNumber) {
      if (!cacheSync()) return 0;

      if (!(options & CACHE_OPTION_NO_READ)) {
        if (!sdCard_->readBlock(blockNumber, cacheBuffer_.data)) return 0;
      }

      cacheStatus_ = 0;
      cacheBlockNumber_ = blockNumber;
    }

    cacheStatus_ |= options & CACHE_STATUS_MASK;

    return &cacheBuffer_;

  }

  cache_t* SdVolume::cacheFetchFat(uint32_t blockNumber, uint8_t options) {
    return cacheFetch(blockNumber, options | CACHE_STATUS_FAT_BLOCK);
  }

  bool SdVolume::cacheSync() {
    if (cacheStatus_ & CACHE_STATUS_DIRTY) {
      if (!sdCard_->writeBlock(cacheBlockNumber_, cacheBuffer_.data)) return false;

      // mirror second FAT
      if ((cacheStatus_ & CACHE_STATUS_FAT_BLOCK) && cacheFatOffset_) {
        uint32_t lbn = cacheBlockNumber_ + cacheFatOffset_;
        if (!sdCard_->writeBlock(lbn, cacheBuffer_.data)) return false;
      }
      cacheStatus_ &= ~CACHE_STATUS_DIRTY;
    }
    return true;
  }

  bool SdVolume::cacheWriteData() {
    return cacheSync();
  }

#endif  // USE_SEPARATE_FAT_CACHE

void SdVolume::cacheInvalidate() {
    cacheBlockNumber_ = 0xFFFFFFFF;
    cacheStatus_ = 0;
}

uint32_t SdVolume::clusterStartBlock(uint32_t cluster) const {
  return dataStartBlock_ + ((cluster - 2) * blocksPerCluster_);
}

// Fetch a FAT entry
bool SdVolume::fatGet(uint32_t cluster, uint32_t* value) {
  uint32_t lba;
  cache_t* pc;

  // error if reserved cluster of beyond FAT

  if (cluster < 2  || cluster > (clusterCount_ + 1)) return false;

  if (FAT12_SUPPORT && fatType_ == 12) {
    uint16_t index = cluster;
    index += index >> 1;
    lba = fatStartBlock_ + (index >> 9);
    pc = cacheFetchFat(lba, CACHE_FOR_READ);
    if (!pc) return false;

    index &= 0x1FF;
    uint16_t tmp = pc->data[index];
    index++;
    if (index == 512) {
      pc = cacheFetchFat(lba + 1, CACHE_FOR_READ);
      if (!pc) return false;
      index = 0;
    }
    tmp |= pc->data[index] << 8;
    *value = cluster & 1 ? tmp >> 4 : tmp & 0xFFF;
    return true;
  }

  if (fatType_ == 16)
    lba = fatStartBlock_ + (cluster >> 8);
  else if (fatType_ == 32)
    lba = fatStartBlock_ + (cluster >> 7);
  else
    return false;

  pc = cacheFetchFat(lba, CACHE_FOR_READ);
  if (!pc) return false;

  if (fatType_ == 16)
    *value = pc->fat16[cluster & 0xFF];
  else
    *value = pc->fat32[cluster & 0x7F] & FAT32MASK;

  return true;

}

// Store a FAT entry
bool SdVolume::fatPut(uint32_t cluster, uint32_t value) {
  uint32_t lba;
  cache_t* pc;
  // error if reserved cluster of beyond FAT
  if (cluster < 2 || cluster > (clusterCount_ + 1)) return false;

  if (FAT12_SUPPORT && fatType_ == 12) {
    uint16_t index = cluster;
    index += index >> 1;
    lba = fatStartBlock_ + (index >> 9);
    pc = cacheFetchFat(lba, CACHE_FOR_WRITE);
    if (!pc) return false;

    index &= 0x1FF;
    uint8_t tmp = value;
    if (cluster & 1) {
      tmp = (pc->data[index] & 0xF) | tmp << 4;
    }
    pc->data[index] = tmp;

    index++;
    if (index == 512) {
      lba++;
      index = 0;
      pc = cacheFetchFat(lba, CACHE_FOR_WRITE);
      if (!pc) return false;
    }

    tmp = value >> 4;
    if (!(cluster & 1)) {
      tmp = ((pc->data[index] & 0xF0)) | tmp >> 4;
    }
    pc->data[index] = tmp;
    return true;
  }

  if (fatType_ == 16)
    lba = fatStartBlock_ + (cluster >> 8);
  else if (fatType_ == 32)
    lba = fatStartBlock_ + (cluster >> 7);
  else
    return false;

  pc = cacheFetchFat(lba, CACHE_FOR_WRITE);
  if (!pc) return false;

  // store entry
  if (fatType_ == 16)
    pc->fat16[cluster & 0xFF] = value;
  else
    pc->fat32[cluster & 0x7F] = value;

  return true;

}

// free a cluster chain
bool SdVolume::freeChain(uint32_t cluster) {
  uint32_t next;

  // clear free cluster location
  allocSearchStart_ = 2;

  do {
    if (!fatGet(cluster, &next)) return false;

    // free cluster
    if (!fatPut(cluster, 0)) return false;

    cluster = next;
  } while (!isEOC(cluster));

  return true;

}

/** Initialize a FAT volume.
 *
 * \param[in] dev The SD card where the volume is located.
 *
 * \param[in] part The partition to be used.  Legal values for \a part are
 * 1-4 to use the corresponding partition on a device formatted with
 * a MBR, Master Boot Record, or zero if the device is formatted as
 * a super floppy with the FAT boot sector in block zero.
 *
 * \return The value one, true, is returned for success and
 * the value zero, false, is returned for failure.  Reasons for
 * failure include not finding a valid partition, not finding a valid
 * FAT file system in the specified partition or an I/O error.
 */
bool SdVolume::init(Sd2Card* dev, uint8_t part) {
  uint32_t totalBlocks;
  uint32_t volumeStartBlock = 0;
  fat32_boot_t* fbs;
  cache_t* pc;
  sdCard_ = dev;
  fatType_ = 0;
  allocSearchStart_ = 2;
  cacheStatus_ = 0;  // cacheSync() will write block if true
  cacheBlockNumber_ = 0xFFFFFFFF;
  cacheFatOffset_ = 0;

  // if part == 0 assume super floppy with FAT boot sector in block zero
  // if part > 0 assume mbr volume with partition table
  if (part) {
    if (part > 4) return false;
    pc = cacheFetch(volumeStartBlock, CACHE_FOR_READ);
    if (!pc) return false;

    part_t* p = &pc->mbr.part[part-1];
    if ((p->boot & 0x7F) !=0 || p->totalSectors < 100 || p->firstSector == 0)
      return false;

    volumeStartBlock = p->firstSector;
  }

  pc = cacheFetch(volumeStartBlock, CACHE_FOR_READ);
  if (!pc) return false;

  fbs = &(pc->fbs32);
  if (fbs->bytesPerSector != 512 || fbs->fatCount == 0 || fbs->reservedSectorCount == 0 || fbs->sectorsPerCluster == 0)
    return false;

  fatCount_ = fbs->fatCount;
  blocksPerCluster_ = fbs->sectorsPerCluster;
  // determine shift that is same as multiply by blocksPerCluster_
  clusterSizeShift_ = 0;
  while (blocksPerCluster_ != (1 << clusterSizeShift_)) {
    // error if not power of 2
    if (clusterSizeShift_++ > 7) return false;
  }

  blocksPerFat_ = fbs->sectorsPerFat16 ? fbs->sectorsPerFat16 : fbs->sectorsPerFat32;

  if (fatCount_ > 0) cacheFatOffset_ = blocksPerFat_;
  fatStartBlock_ = volumeStartBlock + fbs->reservedSectorCount;

  // count for FAT16 zero for FAT32
  rootDirEntryCount_ = fbs->rootDirEntryCount;

  // directory start for FAT16 dataStart for FAT32
  rootDirStart_ = fatStartBlock_ + fbs->fatCount * blocksPerFat_;

  // data start for FAT16 and FAT32
  dataStartBlock_ = rootDirStart_ + ((32 * fbs->rootDirEntryCount + 511) / 512);

  // total blocks for FAT16 or FAT32
  totalBlocks = fbs->totalSectors16 ? fbs->totalSectors16 : fbs->totalSectors32;

  // total data blocks
  clusterCount_ = totalBlocks - (dataStartBlock_ - volumeStartBlock);

  // divide by cluster size to get cluster count
  clusterCount_ >>= clusterSizeShift_;

  // FAT type is determined by cluster count
  if (clusterCount_ < 4085) {
    fatType_ = 12;
    if (!FAT12_SUPPORT) return false;
  }
  else if (clusterCount_ < 65525)
    fatType_ = 16;
  else {
    rootDirStart_ = fbs->fat32RootCluster;
    fatType_ = 32;
  }

  return true;

}

#endif  // HAS_SD_SUPPORT
