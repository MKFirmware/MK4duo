/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
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

#define DEC 10
#define HEX 16
#define OCT  8
#define BIN  2
#define BYTE 0

#ifndef SERIAL_PORT_1
  #define SERIAL_PORT_1 0
#endif

#ifndef RX_BUFFER_SIZE
  #define RX_BUFFER_SIZE 128
#endif

#ifndef TX_BUFFER_SIZE
  #define TX_BUFFER_SIZE 32
#endif

// Templated type selector
template<bool b, typename T, typename F> struct TypeSelector { typedef T type;} ;
template<typename T, typename F> struct TypeSelector<false, T, F> { typedef F type; };

constexpr bool
  bSERIAL_XON_XOFF = (false
    #if ENABLED(SERIAL_XON_XOFF)
      || true
    #endif
  ),
  bEMERGENCY_PARSER = (false
    #if ENABLED(EMERGENCY_PARSER)
      || true
    #endif
  ),
  bSERIAL_STATS_DROPPED_RX = (false
    #if ENABLED(SERIAL_STATS_DROPPED_RX)
      || true
    #endif
  ),
  bSERIAL_STATS_RX_BUFFER_OVERRUNS = (false
    #if ENABLED(SERIAL_STATS_RX_BUFFER_OVERRUNS)
      || true
    #endif
  ),
  bSERIAL_STATS_RX_FRAMING_ERRORS = (false
    #if ENABLED(SERIAL_STATS_RX_FRAMING_ERRORS)
      || true
    #endif
  ),
  bSERIAL_STATS_MAX_RX_QUEUED = (false
    #if ENABLED(SERIAL_STATS_MAX_RX_QUEUED)
      || true
    #endif
  );

template <uint8_t serial>
struct MK4duoSerialHostCfg {
  static constexpr int PORT               = serial;
  static constexpr unsigned int RX_SIZE   = RX_BUFFER_SIZE;
  static constexpr unsigned int TX_SIZE   = TX_BUFFER_SIZE;
  static constexpr bool XONOFF            = bSERIAL_XON_XOFF;
  static constexpr bool EMERGENCYPARSER   = bEMERGENCY_PARSER;
  static constexpr bool DROPPED_RX        = bSERIAL_STATS_DROPPED_RX;
  static constexpr bool RX_OVERRUNS       = bSERIAL_STATS_RX_BUFFER_OVERRUNS;
  static constexpr bool RX_FRAMING_ERRORS = bSERIAL_STATS_RX_FRAMING_ERRORS;
  static constexpr bool MAX_RX_QUEUED     = bSERIAL_STATS_MAX_RX_QUEUED;
};

template <uint8_t serial>
struct MK4duoSerialCfg {
  static constexpr int PORT               = serial;
  static constexpr unsigned int RX_SIZE   = 32;
  static constexpr unsigned int TX_SIZE   = 32;
  static constexpr bool XONOFF            = false;
  static constexpr bool EMERGENCYPARSER   = false;
  static constexpr bool DROPPED_RX        = false;
  static constexpr bool RX_OVERRUNS       = false;
  static constexpr bool RX_FRAMING_ERRORS = false;
  static constexpr bool MAX_RX_QUEUED     = false;
};
