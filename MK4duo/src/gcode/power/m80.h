/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2017 Alberto Cotronei @MagoKimbra
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

#if HAS_POWER_SWITCH

  #define CODE_M80

  /**
   * M80   : Turn on Power Supply
   * M80 S : Report the current state and exit
   */
  inline void gcode_M80(void) {

    // S: Report the current power supply state and exit
    if (parser.seen('S')) {
      SERIAL_PS(powerManager.powersupply_on ? PSTR("PS:1\n") : PSTR("PS:0\n"));
      return;
    }

    powerManager.power_on();

    // If you have a switch on suicide pin, this is useful
    // if you want to start another print with suicide feature after
    // a print without suicide...
    #if HAS(SUICIDE)
      OUT_WRITE(SUICIDE_PIN, HIGH);
    #endif

    #if ENABLED(HAVE_TMC2130)
      delay(100);
      tmc2130_init(); // Settings only stick when the driver has power
    #endif

    LCD_MESSAGEPGM(WELCOME_MSG);

    #if ENABLED(LASER) && ENABLED(LASER_PERIPHERALS)
      laser.peripherals_on();
      laser.wait_for_peripherals();
    #endif
  }

#endif // HAS_POWER_SWITCH
