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
 * \file
 * \brief SdBaseFile class
 */

/**
 * Arduino SdFat Library
 * Copyright (C) 2009 by William Greiman
 *
 * This file is part of the Arduino Sd2Card Library
 */

#include "SdVolume.h"

#include <stdint.h>

/**
 * \struct FatPos_t
 * \brief internal type for istream
 * do not use in user apps
 */
struct FatPos_t {
  uint32_t position;  // stream byte position
  uint32_t cluster;   // cluster of position
  FatPos_t() : position(0), cluster(0) {}
};

// use the gnu style oflag in open()
uint8_t const O_READ = 0x01,                    // open() oflag for reading
              O_RDONLY = O_READ,                // open() oflag - same as O_IN
              O_WRITE = 0x02,                   // open() oflag for write
              O_WRONLY = O_WRITE,               // open() oflag - same as O_WRITE
              O_RDWR = (O_READ | O_WRITE),      // open() oflag for reading and writing
              O_ACCMODE = (O_READ | O_WRITE),   // open() oflag mask for access modes
              O_APPEND = 0x04,                  // The file offset shall be set to the end of the file prior to each write.
              O_SYNC = 0x08,                    // Synchronous writes - call sync() after each write
              O_TRUNC = 0x10,                   // Truncate the file to zero length
              O_AT_END = 0x20,                  // Set the initial position at the end of the file
              O_CREAT = 0x40,                   // Create the file if nonexistent
              O_EXCL = 0x80;                    // If O_CREAT and O_EXCL are set, open() shall fail if the file exists

// SdBaseFile class static and const definitions

// flags for ls()
uint8_t const LS_DATE = 1,    // ls() flag to print modify date
              LS_SIZE = 2,    // ls() flag to print file size
              LS_R = 4;       // ls() flag for recursive list of subdirectories


// flags for timestamp
uint8_t const T_ACCESS = 1,   // Set the file's last access date
              T_CREATE = 2,   // Set the file's creation date and time
              T_WRITE = 4;    // Set the file's write date and time

// values for type_
uint8_t const FAT_FILE_TYPE_CLOSED = 0,                           // This file has not been opened.
              FAT_FILE_TYPE_NORMAL = 1,                           // A normal file
              FAT_FILE_TYPE_ROOT_FIXED = 2,                       // A FAT12 or FAT16 root directory
              FAT_FILE_TYPE_ROOT32 = 3,                           // A FAT32 root directory
              FAT_FILE_TYPE_SUBDIR = 4,                           // A subdirectory file
              FAT_FILE_TYPE_MIN_DIR = FAT_FILE_TYPE_ROOT_FIXED;   // Test value for directory type

/**
 * date field for FAT directory entry
 * \param[in] year [1980,2107]
 * \param[in] month [1,12]
 * \param[in] day [1,31]
 *
 * \return Packed date for dir_t entry.
 */
static inline uint16_t FAT_DATE(uint16_t year, uint8_t month, uint8_t day) { return (year - 1980) << 9 | month << 5 | day; }

/**
 * year part of FAT directory date field
 * \param[in] fatDate Date in packed dir format.
 *
 * \return Extracted year [1980,2107]
 */
static inline uint16_t FAT_YEAR(uint16_t fatDate) { return 1980 + (fatDate >> 9); }

/**
 * month part of FAT directory date field
 * \param[in] fatDate Date in packed dir format.
 *
 * \return Extracted month [1,12]
 */
static inline uint8_t FAT_MONTH(uint16_t fatDate) { return (fatDate >> 5) & 0xF; }

/**
 * day part of FAT directory date field
 * \param[in] fatDate Date in packed dir format.
 *
 * \return Extracted day [1,31]
 */
static inline uint8_t FAT_DAY(uint16_t fatDate) { return fatDate & 0x1F; }

/**
 * time field for FAT directory entry
 * \param[in] hour [0,23]
 * \param[in] minute [0,59]
 * \param[in] second [0,59]
 *
 * \return Packed time for dir_t entry.
 */
static inline uint16_t FAT_TIME(uint8_t hour, uint8_t minute, uint8_t second) { return hour << 11 | minute << 5 | second >> 1; }

/**
 * hour part of FAT directory time field
 * \param[in] fatTime Time in packed dir format.
 *
 * \return Extracted hour [0,23]
 */
static inline uint8_t FAT_HOUR(uint16_t fatTime) { return fatTime >> 11; }

/**
 * minute part of FAT directory time field
 * \param[in] fatTime Time in packed dir format.
 *
 * \return Extracted minute [0,59]
 */
static inline uint8_t FAT_MINUTE(uint16_t fatTime) { return (fatTime >> 5) & 0x3F; }

/**
 * second part of FAT directory time field
 * Note second/2 is stored in packed time.
 *
 * \param[in] fatTime Time in packed dir format.
 *
 * \return Extracted second [0,58]
 */
static inline uint8_t FAT_SECOND(uint16_t fatTime) { return 2 * (fatTime & 0x1F); }

// Default date for file timestamps is 1 Jan 2000
uint16_t const FAT_DEFAULT_DATE = ((2000 - 1980) << 9) | (1 << 5) | 1;
// Default time for file timestamp is 1 am
uint16_t const FAT_DEFAULT_TIME = (1 << 11);

/**
 * \class SdBaseFile
 * \brief Base class for SdFile with Print and C++ streams.
 */
class SdBaseFile {

  public: /** Constructor */

    SdBaseFile() : writeError(false), type_(FAT_FILE_TYPE_CLOSED) {}
    SdBaseFile(const char* path, uint8_t oflag);
    ~SdBaseFile() { if (isOpen()) close(); }

  public: /** Public Parameters */

    SdVolume* vol_;           // volume where file is located

    /**
     * writeError is set to true if an error occurs during a write().
     * Set writeError to false before calling print() and/or write() and check
     * for true after calls to print() and/or write().
     */
    bool writeError;

    // allow SdFat to set cwd_
    friend class SdFat;

    // global pointer to cwd dir
    static SdBaseFile* cwd_;

  private: /** Private Parameters */

    // sync of directory entry required
    static uint8_t const F_FILE_DIR_DIRTY = 0x80;

    char *    pathend;

    uint8_t   flags_,         // See above for definition of flags_ bits
              fstate_,        // error and eof indicator
              type_,          // type of file see above for values
              dirIndex_;      // index of directory entry in dirBlock

    uint32_t  curCluster_,    // cluster for current file position
              curPosition_,   // current file position in bytes from beginning
              dirBlock_,      // block for this files directory entry
              fileSize_,      // file size in bytes
              firstCluster_;  // first cluster of file

  public: /** Public Function */

    /**
     * return The total number of bytes in a file or directory.
     */
    uint32_t fileSize() const { return fileSize_; }

    /**
     * return number of bytes available from the current position to EOF
     */
    uint32_t available() { return fileSize() - curPosition(); }

    /**
     * return The current cluster number for a file or directory.
     */
    uint32_t curCluster() const { return curCluster_; }

    /**
     * return The current position for a file or directory.
     */
    uint32_t curPosition() const { return curPosition_; }

    /**
     * Get a file's name
     *
     * \param[out] name An array of 13 characters for the file's name.
     *
     * \return true for success, false for failure.
     */
    bool getFilename(char * const name);

    /**
     * Sets a file's position.
     *
     * \param[in] pos The new position in bytes from the beginning of the file.
     *
     * \return true for success, false for failure.
     */
    bool seekSet(const uint32_t pos);

    /**
     * Set the files position to current position + \a pos. See seekSet().
     * param[in] offset The new position in bytes from the current position.
     * return true for success or false for failure.
     */
    bool seekCur(int32_t offset) { return seekSet(curPosition_ + offset); }

    /**
     * Set the files position to end-of-file + \a offset. See seekSet().
     * param[in] offset The new position in bytes from end-of-file.
     * return true for success or false for failure.
     */
    bool seekEnd(int32_t offset = 0) { return seekSet(fileSize_ + offset); }

    /**
     * The sync() call causes all modified data and directory fields
     * to be written to the storage device.
     *
     * \return true for success, false for failure.
     * Reasons for failure include a call to sync() before a file has been
     * opened or an I/O error.
     */
    bool sync();

    /**
     * return True if this is a directory else false.
     */
    bool isDir() const { return type_ >= FAT_FILE_TYPE_MIN_DIR; }

    /**
     * return True if this is a normal file else false.
     */
    bool isFile() const { return type_ == FAT_FILE_TYPE_NORMAL; }

    /**
     * return True if this is an open file/directory else false.
     */
    bool isOpen() const { return type_ != FAT_FILE_TYPE_CLOSED; }

    /**
     * return True if this is a subdirectory else false.
     */
    bool isSubDir() const { return type_ == FAT_FILE_TYPE_SUBDIR; }

    /**
     * return True if this is the root directory.
     */
    bool isRoot() const {
      return type_ == FAT_FILE_TYPE_ROOT_FIXED || type_ == FAT_FILE_TYPE_ROOT32;
    }

    /**
     * Make a new directory.
     *
     * \param[in] parent An open SdFat instance for the directory that will contain
     * the new directory.
     *
     * \param[in] path A path with a valid 8.3 DOS name for the new directory.
     *
     * \param[in] pFlag Create missing parent directories if true.
     *
     * \return true for success, false for failure.
     * Reasons for failure include this file is already open, \a parent is not a
     * directory, \a path is invalid or already exists in \a parent.
     */
    bool mkdir(SdBaseFile* dir, const char * path, bool pFlag=true);
    // alias for backward compactability
    bool makeDir(SdBaseFile* dir, const char * path) { return mkdir(dir, path, false); }

    /**
     * Open a file in the current working directory.
     *
     * \param[in] path A path with a valid 8.3 DOS name for a file to be opened.
     *
     * \param[in] oflag Values for \a oflag are constructed by a bitwise-inclusive
     * OR of open flags. see SdBaseFile::open(SdBaseFile*, const char*, uint8_t).
     *
     * \return The value one, true, is returned for success and
     * the value zero, false, is returned for failure.
     */
    bool open(const char * path, uint8_t oflag=O_READ);

    /**
     * Open a file by index.
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
    bool open(SdBaseFile* dirFile, uint16_t index, uint8_t oflag=O_READ);

    bool open(SdBaseFile* dirFile, const char * path, uint8_t oflag=O_READ);

    /**
     * Open the next file or subdirectory in a directory.
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
    bool openNext(SdBaseFile* dirFile, uint8_t oflag);

    /**
     * Open a directory's parent directory.
     *
     * \param[in] dir Parent of this directory will be opened.  Must not be root.
     *
     * \return true for success, false for failure.
     */
    bool openParent(SdBaseFile* dir);

    /**
     * Open a volume's root directory.
     *
     * \param[in] vol The FAT volume containing the root directory to be opened.
     *
     * \return true for success, false for failure.
     * Reasons for failure include the file is already open, the FAT volume has
     * not been initialized or it a FAT12 volume.
     */
    bool openRoot(SdVolume* vol);

    /**
     * Close a file and force cached data and directory information
     *  to be written to the storage device.
     *
     * \return true for success, false for failure.
     * Reasons for failure include no file is open or an I/O error.
     */
    bool close();

    /**
     * Read the next byte from a file.
     *
     * \return For success read returns the next byte in the file as an int.
     * If an error occurs or end of file is reached -1 is returned.
     */
    int16_t read();

    /**
     * Read data from a file starting at the current position.
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
    int16_t read(void* buf, uint16_t nbyte);

    /**
     * Read the next entry in a directory.
     *
     * \param[out] dir The dir_t struct that will receive the data.
     *
     * \return For success readDir() returns the number of bytes read.
     * A value of zero will be returned if end of file is reached.
     * If an error occurs, readDir() returns -1.  Possible errors include
     * readDir() called before a directory has been opened, this is not
     * a directory file or an I/O error occurred.
     */
    int8_t readDir(dir_t* dir);
    int8_t readDir(dir_t& dir) { return readDir(&dir); }

    /**
     * Write data to an open file.
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
    int16_t write(const void* buf, uint16_t nbyte);

    /**
     * Remove a file.
     *
     * The directory entry and all data for the file are deleted.
     *
     * \note This function should not be used to delete the 8.3 version of a
     * file that has a long name. For example if a file has the long name
     * "New Text Document.txt" you should not delete the 8.3 name "NEWTEX~1.TXT".
     *
     * \return true for success, false for failure.
     * Reasons for failure include the file read-only, is a directory,
     * or an I/O error occurred.
     */
    bool remove();

    /**
     * Remove a file.
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
     * \return true for success, false for failure.
     * Reasons for failure include the file is a directory, is read only,
     * \a dirFile is not a directory, \a path is not found
     * or an I/O error occurred.
     */
    static bool remove(SdBaseFile* dirFile, const char * path);
    
    /**
     * Set the file's current position to zero.
     */
    void rewind() { seekSet(0); }

    /**
     * List directory contents.
     * list to indicate subdirectory level.
     */
    void ls();

    uint8_t lsRecursive(SdBaseFile *parent, uint8_t level, char *findFilename, SdBaseFile *pParentFound, bool isJson);

    /** Read the next directory entry from a directory file with the long filename
     *
     * \param[out] dir The dir_t struct that will receive the data.
     * \param[out] longFilename The long filename associated with the 8.3 name
     *
     * \return For success getLongFilename() returns a pointer to dir_t
     * A value of zero will be returned if end of file is reached.
     */
    dir_t* getLongFilename(dir_t *dir, char *longFilename);

    /**
     * %Print a directory date field.
     *
     *  Format is yyyy-mm-dd.
     *
     * \param[in] pr Print stream for output.
     * \param[in] fatDate The date field from a directory entry.
     */
    static void printFatDate(uint16_t fatDate);

    /**
     * %Print a directory time field.
     *
     * Format is hh:mm:ss.
     *
     * \param[in] pr Print stream for output.
     * \param[in] fatTime The time field from a directory entry.
     */
    static void printFatTime(uint16_t fatTime);

    /**
     * Print a file's modify date and time
     *
     * \param[in] pr Print stream for output.
     *
     * \return The value one, true, is returned for success and
     * the value zero, false, is returned for failure.
     */
    bool printModifyDateTime();

    /**
     * Print a file's name to Serial
     *
     * \return true for success, false for failure.
     */
    bool printName();

  private: /** Private Function */

    /**
     * return SdVolume that contains this file.
     */
    SdVolume* volume() const { return vol_; }

    /**
     * get position for streams
     * param[out] pos struct to receive position
     */
    void getpos(FatPos_t* pos);

    /**
     * set position for streams
     * param[out] pos struct with value for new position
     */
    void setpos(FatPos_t* pos);

    /** Set the date/time callback function
     *
     * \param[in] dateTime The user's call back function.  The callback
     * function is of the form:
     *
     * \code
     * void dateTime(uint16_t* date, uint16_t* time) {
     *   uint16_t year;
     *   uint8_t month, day, hour, minute, second;
     *
     *   // User gets date and time from GPS or real-time clock here
     *
     *   // return date using FAT_DATE macro to format fields
     *   *date = FAT_DATE(year, month, day);
     *
     *   // return time using FAT_TIME macro to format fields
     *   *time = FAT_TIME(hour, minute, second);
     * }
     * \endcode
     *
     * Sets the function that is called when a file is created or when
     * a file's directory entry is modified by sync(). All timestamps,
     * access, creation, and modify, are set when a file is created.
     * sync() maintains the last access date and last modify date/time.
     *
     * See the timestamp() function.
     */
    static void dateTimeCallback(
      void (*dateTime)(uint16_t* date, uint16_t* time)) {
      dateTime_ = dateTime;
    }

    /**
     * Cancel the date/time callback function.
     */
    static void dateTimeCallbackCancel() { dateTime_ = 0; }

    /**
     * Return a file's directory entry.
     *
     * \param[out] dir Location for return of the file's directory entry.
     *
     * \return true for success, false for failure.
     */
    bool dirEntry(dir_t* dir);

    /**
     * Format the name field of \a dir into the 13 byte array
     * \a name in standard 8.3 short name format.
     *
     * \param[in] dir The directory structure containing the name.
     * \param[out] name A 13 byte char array for the formatted name.
     */
    static void dirName(const dir_t& dir, char* name);

    /**
     * Test for the existence of a file in a directory
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
    bool exists(const char * name);

    /**
     * return The first cluster number for a file or directory.
     */
    uint32_t firstCluster() const { return firstCluster_; }

    uint8_t lfn_checksum(const unsigned char *pFCBName);

    /**
     * Open a file or directory by name.
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
    bool openParentReturnFile(SdBaseFile* dirFile, PGM_P path, uint8_t *dname, SdBaseFile *newParent, bool bMakeDirs);

    /**
     * Return the next available byte without consuming it.
     *
     * \return The byte if no error and not at eof else -1;
     */
    int peek();

    /**
     * Print a file's creation date and time
     *
     * \param[in] pr Print stream for output.
     *
     * \return The value one, true, is returned for success and
     * the value zero, false, is returned for failure.
     */
    bool printCreateDateTime();

    /**
     * Print a number followed by a field terminator.
     * \param[in] value The number to be printed.
     * \param[in] term The field terminator.  Use '\\n' for CR LF.
     * \return The number of bytes written or -1 if an error occurs.
     */
    int printField(int16_t value, char term);
    int printField(uint16_t value, char term);
    int printField(int32_t value, char term);
    int printField(uint32_t value, char term);

    /**
     * Remove a directory file.
     *
     * The directory file will be removed only if it is empty and is not the
     * root directory.  rmdir() follows DOS and Windows and ignores the
     * read-only attribute for the directory.
     *
     * \note This function should not be used to delete the 8.3 version of a
     * directory that has a long name. For example if a directory has the
     * long name "New folder" you should not delete the 8.3 name "NEWFOL~1".
     *
     * \return true for success, false for failure.
     * Reasons for failure include the file is not a directory, is the root
     * directory, is not empty, or an I/O error occurred.
     */
    bool rmdir();
    // for backward compatibility
    bool rmDir() {return rmdir();}

    /**
     * Recursively delete a directory and all contained files.
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
     * \return true for success, false for failure.
     */
    bool rmRfStar();

    /**
     * Copy a file's timestamps
     *
     * \param[in] file File to copy timestamps from.
     *
     * \note
     * Modify and access timestamps may be overwritten if a date time callback
     * function has been set by dateTimeCallback().
     *
     * \return true for success, false for failure.
     */
    bool timestamp(SdBaseFile* file);

    /**
     * Set a file's timestamps in its directory entry.
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
     * \return true for success, false for failure.
     */
    bool timestamp(uint8_t flag, uint16_t year, uint8_t month, uint8_t day,
                   uint8_t hour, uint8_t minute, uint8_t second);

    /**
     * Type of file.  You should use isFile() or isDir() instead of type()
     * if possible.
     *
     * \return The file or directory type.
     */
    uint8_t type() const {return type_;}

    /**
     * Truncate a file to a specified length.  The current file position
     * will be maintained if it is less than or equal to \a length otherwise
     * it will be set to end of file.
     *
     * \param[in] length The desired length for the file.
     *
     * \return true for success, false for failure.
     * Reasons for failure include file is read only, file is a directory,
     * \a length is greater than the current file size or an I/O error occurs.
     */
    bool truncate(uint32_t size);

    // data time callback function
    static void (*dateTime_)(uint16_t* date, uint16_t* time);

    // add a cluster to a file
    bool addCluster();

    // Add a cluster to a directory file and zero the cluster.
    // return with first block of cluster in the cache
    bool addDirCluster();

    // cache a file's directory entry
    // return pointer to cached entry or null for failure
    dir_t* cacheDirEntry(uint8_t action);

    // saves 32 bytes on stack for ls recursion
    // return 0 - EOF, 1 - normal file, or 2 - directory
    int8_t lsPrintNext(uint8_t flags, uint8_t indent);

    // Format directory name field from a 8.3 name string
    static bool make83Name(PGM_P str, uint8_t* name, const char** ptr);

    bool mkdir(SdBaseFile* parent, const uint8_t *dname);

    /** 
     * Convert the dir_t name field of the file (which contains blank fills)
     * into a proper filename string without spaces inside.
     *
     * buffer MUST be at least a 13 char array!
     */
    static void createFilename(char* buffer, const dir_t &dirEntry);

    bool findSpace(dir_t *dir, int8_t cVFATNeeded, int8_t *pcVFATFound, uint32_t *pwIndexPos);

    // open with filename in dname
    bool open(SdBaseFile* dirFile, const uint8_t *dname, uint8_t oflag, bool bDir);

    // open a cached directory entry. Assumes vol_ is initialized
    bool openCachedEntry(uint8_t cacheIndex, uint8_t oflags);

    /**
     * %Print the name field of a directory entry in 8.3 format.
     * \param[in] pr Print stream for output.
     * \param[in] dir The directory structure containing the name.
     * \param[in] width Blank fill name if length is less than \a width.
     * \param[in] printSlash Print '/' after directory names if true.
     */
    static void printDirName(const dir_t& dir, uint8_t width, bool printSlash);

    // Read next directory entry into the cache
    // Assumes file is correctly positioned
    dir_t* readDirCache();

};
