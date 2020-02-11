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

/**
 * mcode
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#if MECH(DELTA) && ENABLED(PROBE_SMART_EFFECTOR) && PIN_EXISTS(SMART_EFFECTOR)

#define CODE_M672

#define M672_PROGBYTE    105  // Magic byte to start programming custom sensitivity
#define M672_ERASEBYTE   131	// Magic byte to clear custom sensitivity

void M672_send(uint8_t b) {    // bit rate requirement: 1KHz +/- 30%
  for (uint8_t bits = 0; bits < 14; bits++) {
    switch (bits) {
      default: { OUT_WRITE(SMART_EFFECTOR_PIN, !!(b & 0x80)); b <<= 1;  break;  } // send bit, shift next into place
      case  7:
      case 12: { OUT_WRITE(SMART_EFFECTOR_PIN, !!(b & 0x80));           break;  } // send bit. no shift
      case  8:
      case 13: { OUT_WRITE(SMART_EFFECTOR_PIN,  !(b & 0x80)); b <<= 1;  break;  } // send inverted previous bit
      case  0: case  1:                                                           // 00
      case  3: { OUT_WRITE(SMART_EFFECTOR_PIN, LOW);                    break;  } // 0010
      case  2: { OUT_WRITE(SMART_EFFECTOR_PIN, HIGH);                   break;  } // 001
    }
    DELAY_US(1000);
  }
}

/**
 * M672 - Set/reset Duet Smart Effector sensitivity
 *
 *  One of these is required:
 *    S<sensitivity> - 0-255
 *    R              - Reset sensitivity to default
 */
inline void gcode_M672() {

  if (parser.seen('R')) {
    M672_send(M672_ERASEBYTE);
    M672_send(M672_ERASEBYTE);
  }
  else if (parser.seenval('S')) {
    const int8_t M672_sensitivity = parser.value_byte();
    M672_send(M672_PROGBYTE);
    M672_send(M672_sensitivity);
    M672_send(255 - M672_sensitivity);
  }
  else {
    SERIAL_EM("!'S' or 'R' parameter required.");
    return;
  }

  OUT_WRITE(SMART_EFFECTOR_PIN, LOW);

}

#endif
