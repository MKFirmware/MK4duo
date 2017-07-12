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
 * gcode.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#define M81

/**
 * M81: Turn off Power, including Power Supply, if there is one.
 *
 *      This code should ALWAYS be available for EMERGENCY SHUTDOWN!
 */
inline void gcode_M81() {
  thermalManager.disable_all_heaters();
  thermalManager.disable_all_coolers();
  stepper.finish_and_disable();

  #if FAN_COUNT > 0
    LOOP_FAN() printer.fanSpeeds[f] = 0;
    #if ENABLED(PROBING_FANS_OFF)
      printer.fans_paused = false;
      ZERO(printer.paused_fanSpeeds);
    #endif
  #endif

  #if ENABLED(LASER)
    laser.extinguish();
    #if ENABLED(LASER_PERIPHERALS)
      laser.peripherals_off();
    #endif
  #endif

  #if ENABLED(CNCROUTER)
    disable_cncrouter();
  #endif

  printer.safe_delay(1000); // Wait 1 second before switching off

  #if HAS(SUICIDE)
    stepper.synchronize();
    suicide();
  #elif HAS_POWER_SWITCH
    powerManager.power_off();
  #endif

  LCD_MESSAGEPGM(MACHINE_NAME " " MSG_OFF ".");

}
