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
#pragma once

/**
 * Arduino SdFat Library
 * Copyright (C) 2009 by William Greiman
 *
 * This file is part of the Arduino Sd2Card Library
 */

#if ENABLED(USB_FLASH_DRIVE_SUPPORT)
  #include "usbflashdrive/Sd2Card.h"
#else
  #include "Sd2Card.h"
#endif

#include "SdFatConfig.h"
#include <stdint.h>

//==============================================================================
// SdVolume class
/**
 * \brief Cache for an SD data block
 */
union cache_t {
  uint8_t         data[512];  // Used to access cached file data blocks.
  uint16_t        fat16[256]; // Used to access cached FAT16 entries.
  uint32_t        fat32[128]; // Used to access cached FAT32 entries.
  dir_t           dir[16];    // Used to access cached directory entries.
  mbr_t           mbr;        // Used to access a cached Master Boot Record.
  fat_boot_t      fbs;        // Used to access to a cached FAT boot sector.
  fat32_boot_t    fbs32;      // Used to access to a cached FAT32 boot sector.
  fat32_fsinfo_t  fsinfo;     // Used to access to a cached FAT32 FSINFO sector.
};

/**
 * \class SdVolume
 * \brief Access FAT16 and FAT32 volumes on SD and SDHC cards.
 */
class SdVolume {

  public:

    /** Create an instance of SdVolume */
    SdVolume(void) :allocSearchStart_(2), fatType_(0) {}

    /**
     * Clear the cache and returns a pointer to the cache.  Used by the WaveRP
     * recorder to do raw write to the SD card.  Not for normal apps.
     * \return A pointer to the cache buffer or zero if an error occurs.
     */
    static uint8_t* cacheClear(void) {
      cacheBlockNumber_ = 0XFFFFFFFF;
      return cacheBuffer_.data;
    }

    /**
     * Initialize a FAT volume.  Try partition one first then try super
     * floppy format.
     *
     * \param[in] dev The Sd2Card where the volume is located.
     *
     * \return The value one, true, is returned for success and
     * the value zero, false, is returned for failure.  Reasons for
     * failure include not finding a valid partition, not finding a valid
     * FAT file system or an I/O error.
     */
    bool init(Sd2Card* dev) { return init(dev, 1) ? true : init(dev, 0);}
    bool init(Sd2Card* dev, uint8_t part);

    // inline functions that return volume info
    /** \return The volume's cluster size in blocks. */
    uint8_t blocksPerCluster(void) const { return blocksPerCluster_; }
    /** \return The number of blocks in one FAT. */
    uint32_t blocksPerFat(void)  const { return blocksPerFat_; }
    /** \return The total number of clusters in the volume. */
    uint32_t clusterCount(void) const { return clusterCount_; }
    /** \return The shift count required to multiply by blocksPerCluster. */
    uint8_t clusterSizeShift(void) const { return clusterSizeShift_; }
    /** \return The logical block number for the start of file data. */
    uint32_t dataStartBlock(void) const { return dataStartBlock_; }
    /** \return The number of FAT structures on the volume. */
    uint8_t fatCount(void) const { return fatCount_; }
    /** \return The logical block number for the start of the first FAT. */
    uint32_t fatStartBlock(void) const { return fatStartBlock_; }
    /** \return The FAT type of the volume. Values are 12, 16 or 32. */
    uint8_t fatType(void) const { return fatType_; }
    /** \return The number of entries in the root directory for FAT16 volumes. */
    uint32_t rootDirEntryCount(void) const { return rootDirEntryCount_; }
    /** \return The logical block number for the start of the root directory
         on FAT16 volumes or the first cluster number on FAT32 volumes. */
    uint32_t rootDirStart(void) const { return rootDirStart_; }
    /** return a pointer to the Sd2Card object for this volume */
    static Sd2Card* sdCard(void) { return sdCard_; }
  
  private:
    // Allow SdBaseFile access to SdVolume private data.
    friend class SdBaseFile;
  
    uint32_t allocSearchStart_;   // start cluster for alloc search
    uint8_t blocksPerCluster_;    // cluster size in blocks
    uint32_t blocksPerFat_;       // FAT size in blocks
    uint32_t clusterCount_;       // clusters in one FAT
    uint8_t clusterSizeShift_;    // shift to convert cluster count to block count
    uint32_t dataStartBlock_;     // first data block number
    uint8_t fatCount_;            // number of FATs on volume
    uint32_t fatStartBlock_;      // start block for first FAT
    uint8_t fatType_;             // volume type (12, 16, OR 32)
    uint16_t rootDirEntryCount_;  // number of entries in FAT16 root dir
    uint32_t rootDirStart_;       // root start block for FAT16, cluster for FAT32
  
  // block caches
  // use of static functions save a bit of flash - maybe not worth complexity
  //
    static const uint8_t CACHE_STATUS_DIRTY = 1;
    static const uint8_t CACHE_STATUS_FAT_BLOCK = 2;
    static const uint8_t CACHE_STATUS_MASK = CACHE_STATUS_DIRTY | CACHE_STATUS_FAT_BLOCK;
    static const uint8_t CACHE_OPTION_NO_READ = 4;
    // value for option argument in cacheFetch to indicate read from cache
    static uint8_t const CACHE_FOR_READ = 0;
    // value for option argument in cacheFetch to indicate write to cache
    static uint8_t const CACHE_FOR_WRITE = CACHE_STATUS_DIRTY;
    // reserve cache block with no read
    static uint8_t const CACHE_RESERVE_FOR_WRITE = CACHE_STATUS_DIRTY | CACHE_OPTION_NO_READ;

    static cache_t cacheBuffer_;        // 512 byte cache for device blocks
    static uint32_t cacheBlockNumber_;  // Logical number of block in the cache
    static uint32_t cacheFatOffset_;    // offset for mirrored FAT
    static uint8_t cacheStatus_;        // status of cache block
    #if USE_SEPARATE_FAT_CACHE
      static cache_t cacheFatBuffer_;       // 512 byte cache for FAT
      static uint32_t cacheFatBlockNumber_; // current Fat block number
      static uint8_t  cacheFatStatus_;      // status of cache Fatblock
    #endif  // USE_SEPARATE_FAT_CACHE
    static Sd2Card* sdCard_;            // Sd2Card object for cache

    cache_t *cacheAddress(void) { return &cacheBuffer_; }
    uint32_t cacheBlockNumber(void) { return cacheBlockNumber_; }
    static cache_t* cacheFetch(uint32_t blockNumber, uint8_t options);
    static cache_t* cacheFetchData(uint32_t blockNumber, uint8_t options);
    static cache_t* cacheFetchFat(uint32_t blockNumber, uint8_t options);
    static void cacheInvalidate();
    static bool cacheSync();
    static bool cacheWriteData();
    static bool cacheWriteFat();
  
    bool allocContiguous(uint32_t count, uint32_t* curCluster);
    uint8_t blockOfCluster(uint32_t position) const { return (position >> 9) & (blocksPerCluster_ - 1); }
    uint32_t clusterStartBlock(uint32_t cluster) const;
    bool fatGet(uint32_t cluster, uint32_t* value);
    bool fatPut(uint32_t cluster, uint32_t value);
    bool fatPutEOC(uint32_t cluster) { return fatPut(cluster, 0x0FFFFFFF); }
    bool freeChain(uint32_t cluster);
    bool isEOC(uint32_t cluster) const {
      if (FAT12_SUPPORT && fatType_ == 12) return  cluster >= FAT12EOC_MIN;
      if (fatType_ == 16) return cluster >= FAT16EOC_MIN;
      return  cluster >= FAT32EOC_MIN;
    }
    bool readBlock(uint32_t block, uint8_t* dst) { return sdCard_->readBlock(block, dst); }
    bool writeBlock(uint32_t block, const uint8_t* dst) { return sdCard_->writeBlock(block, dst); }

};
