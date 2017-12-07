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
 * mcode
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#if HAS_EXT_ENCODER

  #define CODE_M512

  /**
   * M512: Print Extruder Encoder status Pin
   */
  inline void gcode_M512(void) {

    #if HAS_E0_ENC
      SERIAL_EMV("Enc0 signal:", (int16_t)READ_ENCODER(E0_ENC_PIN));
    #endif
    #if HAS_E1_ENC
      SERIAL_EMV("Enc1 signal:", (int16_t)READ_ENCODER(E1_ENC_PIN));
    #endif
    #if HAS_E2_ENC
      SERIAL_EMV("Enc2 signal:", (int16_t)READ_ENCODER(E2_ENC_PIN));
    #endif
    #if HAS_E3_ENC
      SERIAL_EMV("Enc3 signal:", (int16_t)READ_ENCODER(E3_ENC_PIN));
    #endif
    #if HAS_E4_ENC
      SERIAL_EMV("Enc4 signal:", (int16_t)READ_ENCODER(E4_ENC_PIN));
    #endif
    #if HAS_E5_ENC
      SERIAL_EMV("Enc5 signal:", (int16_t)READ_ENCODER(E5_ENC_PIN));
    #endif

  }

#endif
