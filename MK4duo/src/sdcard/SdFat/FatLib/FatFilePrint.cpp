/**
 * Copyright (c) 2011-2018 Bill Greiman
 * This file is part of the SdFat library for SD memory cards.
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */
#include "../../../../mk4duo.h"
#include <math.h>
#include "FatFile.h"
#include "FmtNumber.h"

#if HAS_SD_SUPPORT

//------------------------------------------------------------------------------
// print uint8_t with width 2
static void print2u(uint8_t v) {
  char c0 = '?';
  char c1 = '?';
  if (v < 100) {
    c1 = v/10;
    c0 = v - 10*c1 + '0';
    c1 += '0';
  }
  SERIAL_CHR(c1);
  SERIAL_CHR(c0);
}
//------------------------------------------------------------------------------
static void printU32(uint32_t v) {
  char buf[11];
  char* ptr = buf + sizeof(buf);
  *--ptr = 0;
  SERIAL_TXT(fmtDec(v, ptr));
}
//------------------------------------------------------------------------------
static void printHex(uint8_t w, uint16_t h) {
  char buf[5];
  char* ptr = buf + sizeof(buf);
  *--ptr = 0;
  for (uint8_t i = 0; i < w; i++) {
    char c = h & 0XF;
    *--ptr = c < 10 ? c + '0' : c + 'A' - 10;
    h >>= 4;
  }
  SERIAL_TXT(ptr);
}
//------------------------------------------------------------------------------
void FatFile::dmpFile(uint32_t pos, size_t n) {
  char text[17];
  text[16] = 0;
  if (n >= 0XFFF0) {
    n = 0XFFF0;
  }
  if (!seekSet(pos)) {
    return;
  }
  for (size_t i = 0; i <= n; i++) {
    if ((i & 15) == 0) {
      if (i) {
        SERIAL_CHR(' ');
        SERIAL_TXT(text);
        if (i == n) {
          break;
        }
      }
      SERIAL_CHR('\r');
      SERIAL_CHR('\n');
      if (i >= n) {
        break;
      }
      printHex(4, i);
      SERIAL_CHR(' ');
    }
    int16_t h = read();
    if (h < 0) {
      break;
    }
    SERIAL_CHR(' ');
    printHex(2, h);
    text[i&15] = ' ' <= h && h < 0X7F ? h : '.';
  }
  SERIAL_EOL();
}
//------------------------------------------------------------------------------
/*
bool FatFile::ls(uint8_t flags, uint8_t indent) {
  FatFile file;
  if (!isDir() || getError()) {
    DBG_FAIL_MACRO;
    goto fail;
  }
  rewind();
  while (file.openNext(this, O_RDONLY)) {
    if (!file.isHidden() || (flags & LS_A)) {
    // indent for dir level
      for (uint8_t i = 0; i < indent; i++) {
        SERIAL_TXT(' ');
      }
      if (flags & LS_DATE) {
        file.printModifyDateTime();
        SERIAL_TXT(' ');
      }
      if (flags & LS_SIZE) {
        file.printFileSize();
        SERIAL_TXT(' ');
      }
      file.printName();
      if (file.isDir()) {
        SERIAL_TXT('/');
      }
      SERIAL_EOL();
      if ((flags & LS_R) && file.isDir()) {
        if (!file.ls(flags, indent + 2)) {
          DBG_FAIL_MACRO;
          goto fail;
        }
      }
    }
    file.close();
  }
  if (getError()) {
    DBG_FAIL_MACRO;
    goto fail;
  }
  return true;

 fail:
  return false;
}
*/

inline int8_t RFstricmp(const char* s1, const char* s2) {
  while(*s1 && (tolower(*s1) == tolower(*s2))) s1++, s2++;
  return (const uint8_t)tolower(*s1) - (const uint8_t)tolower(*s2);
}

inline int8_t RFstrnicmp(const char* s1, const char* s2, size_t n) {
  while(n--) {
    if(tolower(*s1) != tolower(*s2))
      return (uint8_t)tolower(*s1) - (uint8_t)tolower(*s2);
    s1++;
    s2++;
  }
  return 0;
}

void FatFile::lsRecursive(uint8_t level, bool isJson) {

  FatFile file;
  #ifdef JSON_OUTPUT
    bool firstFile = true;
  #endif
  rewind();

  while (file.openNext(this, O_READ)) {
    file.getName(card.tempLongFilename, LONG_FILENAME_LENGTH);
    if (file.isHidden()) {
      file.close();
      continue;
    }
    // if (! (file.isFile() || file.isDir())) continue;
    if (strcmp(card.tempLongFilename, "..") == 0) {
      file.close();
      continue;
    }
    if (card.tempLongFilename[0] == '.') {
      file.close();
      continue; // MAC CRAP
    }
    if (file.isDir()) {
      if (level >= SD_MAX_FOLDER_DEPTH) {
        file.close();
        continue; // can't go deeper
      }
      if (level && !isJson) {
        SERIAL_TXT(card.fileName);
        SERIAL_CHR('/');
      }
      #ifdef JSON_OUTPUT
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
        SERIAL_CHR('/'); SERIAL_EOL(); // End with / to mark it as directory entry, so we can see empty directories.
      #endif
      char* tmp;
      // Add directory name
      if (level)
        strcat(card.fileName, "/");
      strcat(card.fileName, card.tempLongFilename);
      if (!isJson) {
        file.lsRecursive(level + 1, false);
      }
      // remove added directory name
      if ((tmp = strrchr(card.fileName, '/')) != NULL)
        *tmp = 0;
      else
        *card.fileName = 0;
    }
    else { // is filename
      if (level && !isJson) {
        SERIAL_TXT(card.fileName);
        SERIAL_CHR('/');
      }
      #ifdef JSON_OUTPUT
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
        #ifdef SD_EXTENDED_DIR
          SERIAL_MV(" ", (long)file.fileSize());
        #endif
        SERIAL_EOL();
      }
    }
    file.close();
  }
}

//------------------------------------------------------------------------------
/** List directory contents.
 *
 * \param[in] pr Print stream for list.
 *
 * \param[in] flags The inclusive OR of
 *
 * LS_DATE - %Print file modification date
 *
 * LS_SIZE - %Print file size.
 *
 * LS_R - Recursive list of subdirectories.
 *
 * \param[in] indent Amount of space before file name. Used for recursive
 * list to indicate subdirectory level.
 */
void FatFile::ls(uint8_t flags, uint8_t indent) {
  *card.fileName = 0;
  lsRecursive(0, false);
}

#ifdef JSON_OUTPUT
  void FatFile::lsJSON() {
    *card.fileName = 0;
    lsRecursive(0, true);
  }
#endif

//------------------------------------------------------------------------------
bool FatFile::printCreateDateTime() {
  dir_t dir;
  if (!dirEntry(&dir)) {
    DBG_FAIL_MACRO;
    goto fail;
  }
  printFatDate(dir.creationDate);
  SERIAL_CHR(' ');
  printFatTime(dir.creationTime);
  return true;

fail:
  return false;
}
//------------------------------------------------------------------------------
void FatFile::printFatDate(uint16_t fatDate) {
  printU32(FAT_YEAR(fatDate));
  SERIAL_CHR('-');
  print2u(FAT_MONTH(fatDate));
  SERIAL_CHR('-');
  print2u(FAT_DAY(fatDate));
}
//------------------------------------------------------------------------------
void FatFile::printFatTime(uint16_t fatTime) {
  print2u(FAT_HOUR(fatTime));
  SERIAL_CHR(':');
  print2u(FAT_MINUTE(fatTime));
  SERIAL_CHR(':');
  print2u(FAT_SECOND(fatTime));
}
//------------------------------------------------------------------------------
/** Template for FatFile::printField() */
template <typename Type>
static int printFieldT(FatFile* file, char sign, Type value, char term) {
  char buf[3*sizeof(Type) + 3];
  char* str = &buf[sizeof(buf)];

  if (term) {
    *--str = term;
    if (term == '\n') {
      *--str = '\r';
    }
  }
#ifdef OLD_FMT
  do {
    Type m = value;
    value /= 10;
    *--str = '0' + m - 10*value;
  } while (value);
#else  // OLD_FMT
  str = fmtDec(value, str);
#endif  // OLD_FMT
  if (sign) {
    *--str = sign;
  }
  return file->write(str, &buf[sizeof(buf)] - str);
}
//------------------------------------------------------------------------------

int FatFile::printField(float value, char term, uint8_t prec) {
  char buf[24];
  char* str = &buf[sizeof(buf)];
  if (term) {
    *--str = term;
    if (term == '\n') {
      *--str = '\r';
    }
  }
  str = fmtFloat(value, str, prec);
  return write(str, buf + sizeof(buf) - str);
}
//------------------------------------------------------------------------------
int FatFile::printField(uint16_t value, char term) {
  return printFieldT(this, 0, value, term);
}
//------------------------------------------------------------------------------
int FatFile::printField(int16_t value, char term) {
  char sign = 0;
  if (value < 0) {
    sign = '-';
    value = -value;
  }
  return printFieldT(this, sign, (uint16_t)value, term);
}
//------------------------------------------------------------------------------
int FatFile::printField(uint32_t value, char term) {
  return printFieldT(this, 0, value, term);
}
//------------------------------------------------------------------------------
int FatFile::printField(int32_t value, char term) {
  char sign = 0;
  if (value < 0) {
    sign = '-';
    value = -value;
  }
  return printFieldT(this, sign, (uint32_t)value, term);
}
//------------------------------------------------------------------------------
bool FatFile::printModifyDateTime() {
  dir_t dir;
  if (!dirEntry(&dir)) {
    DBG_FAIL_MACRO;
    goto fail;
  }
  printFatDate(dir.lastWriteDate);
  SERIAL_CHR(' ');
  printFatTime(dir.lastWriteTime);
  return true;

fail:
  return false;
}
//------------------------------------------------------------------------------
void FatFile::printFileSize() {
  char buf[11];
  char *ptr = buf + sizeof(buf);
  *--ptr = 0;
  ptr = fmtDec(fileSize(), ptr);
  while (ptr > buf) {
    *--ptr = ' ';
  }
  SERIAL_TXT(buf);
}

#endif