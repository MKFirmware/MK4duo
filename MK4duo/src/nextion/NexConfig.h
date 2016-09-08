/**
 * @file NexConfig.h
 *
 * Options for user can be found here. 
 *
 * @author  Wu Pengfei (email:<pengfei.wu@itead.cc>)
 * @date    2015/8/13
 * @copyright 
 * Copyright (C) 2014-2015 ITEAD Intelligent Systems Co., Ltd. \n
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */
 
#include "../../base.h"

#ifndef __NEXCONFIG_H__
#define __NEXCONFIG_H__

#if NEXTION_SERIAL > 0
  #if NEXTION_SERIAL == 1
    #define nexSerial Serial1
  #elif NEXTION_SERIAL == 2
    #define nexSerial Serial2
  #elif NEXTION_SERIAL == 3
    #define nexSerial Serial3
  #endif
#else
  #define nexSerial Serial1
#endif

#define dbSerialPrint(a)    {}
#define dbSerialPrintln(a)  {}
#define dbSerialBegin(a)    {}

#endif //__NEXCONFIG_H__
