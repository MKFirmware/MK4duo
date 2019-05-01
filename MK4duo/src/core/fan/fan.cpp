/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
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
 * fan.cpp
 *
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"

#if FAN_COUNT > 0
  Fan fans[FAN_COUNT];
#endif

/**
 * Initialize Fans
 */
void Fan::init() {

  speed               = 0;
  paused_speed        = 0;
  scaled_speed        = 128;
  kickstart           = 0;

  setIdle(false);

  if (printer.isRunning()) return; // All running not reinitialize

  if (data.pin > 0) HAL::pinMode(data.pin, isHWinvert() ? OUTPUT_HIGH : OUTPUT_LOW);

}

void Fan::set_auto_monitor(const int8_t h) {
  if (WITHIN(h, 0, HOTENDS - 1) || h == 7)
    SBI(data.auto_monitor, (uint8_t)h);
  else if (h == -1)
    data.auto_monitor = 0;
  else
    SERIAL_EM(MSG_INVALID_HOTEND);
  spin();
}

void Fan::set_output_pwm() {
  const uint8_t new_Speed = isHWinvert() ? 255 - actual_speed() : actual_speed();
  HAL::analogWrite(data.pin, new_Speed, data.freq);
}

void Fan::spin() {

  static millis_s controller_fan_ms = 0;

  const millis_s ms = millis();

  if (data.auto_monitor != 0) {

    // Check for Hotend temperature
    LOOP_HOTEND() {
      if (TEST(data.auto_monitor, h)) {
        if (hotends[h].current_temperature > data.trigger_temperature) {
          speed = data.max_speed;
          break;
        }
        else
          speed = data.min_speed;
      }
    }

    // Check for Controller fan
    if (TEST(data.auto_monitor, 7)) {

      // Check Heaters
      if (thermalManager.heaters_isActive()) controller_fan_ms = ms;

      #if HAS_MCU_TEMPERATURE
        // Check MSU
        if (thermalManager.mcu_current_temperature >= 50) controller_fan_ms = ms;
      #endif

      // Check Motors
      if (X_ENABLE_READ() == X_ENABLE_ON || Y_ENABLE_READ() == Y_ENABLE_ON || Z_ENABLE_READ() == Z_ENABLE_ON
        || E0_ENABLE_READ() == E_ENABLE_ON // If any of the drivers are enabled...
        #if DRIVER_EXTRUDERS > 1
          || E1_ENABLE_READ() == E_ENABLE_ON
          #if HAS_X2_ENABLE
            || X2_ENABLE_READ() == X_ENABLE_ON
          #endif
          #if DRIVER_EXTRUDERS > 2
            || E2_ENABLE_READ() == E_ENABLE_ON
            #if DRIVER_EXTRUDERS > 3
              || E3_ENABLE_READ() == E_ENABLE_ON
              #if DRIVER_EXTRUDERS > 4
                || E4_ENABLE_READ() == E_ENABLE_ON
                #if DRIVER_EXTRUDERS > 5
                  || E5_ENABLE_READ() == E_ENABLE_ON
                #endif
              #endif
            #endif
          #endif
        #endif
      ) {
        controller_fan_ms = ms;
      }

      // Fan off if no steppers or heaters have been enabled for CONTROLLERFAN_SECS seconds
      if (!controller_fan_ms || expired(&controller_fan_ms, millis_s(CONTROLLERFAN_SECS * 1000U))) {
        controller_fan_ms = 0;
        speed = data.min_speed;
      }
      else
        speed = data.max_speed;
    }

  }

  speed = speed ? constrain(speed, data.min_speed, data.max_speed) : 0;

}

void Fan::print_M106() {
  bool found_auto = false;
  SERIAL_LM(CFG, "Fans: P<Fan> U<Pin> L<Min Speed> X<Max Speed> F<Freq> I<Hardware Inverted 0-1> H<Auto mode> T<Trig Temp>");
  SERIAL_SMV(CFG, "  M106 P", (int)data.ID);
  SERIAL_MV(" U", data.pin);
  SERIAL_MV(" L", data.min_speed);
  SERIAL_MV(" X", data.max_speed);
  SERIAL_MV(" F", data.freq);
  SERIAL_MV(" I", isHWinvert());
  SERIAL_MSG(" H");
  LOOP_HOTEND() {
    if (TEST(data.auto_monitor, h)) {
      SERIAL_VAL((int)h);
      SERIAL_MV(" T", data.trigger_temperature);
      found_auto = true;
      break;
    }
  }
  if (!found_auto) {
    if (TEST(data.auto_monitor, 7))
      SERIAL_CHR('7');
    else
      SERIAL_MSG("-1");
  }
  SERIAL_EOL();
}

#if ENABLED(TACHOMETRIC)
  void tacho_interrupt0() { fans[0].tacho.interrupt(); }
  #if FAN_COUNT > 1
    void tacho_interrupt1() { fans[1].tacho.interrupt(); }
    #if FAN_COUNT > 2
      void tacho_interrupt2() { fans[2].tacho.interrupt(); }
      #if FAN_COUNT > 3
        void tacho_interrupt3() { fans[3].tacho.interrupt(); }
        #if FAN_COUNT > 4
          void tacho_interrupt4() { fans[4].tacho.interrupt(); }
          #if FAN_COUNT > 5
            void tacho_interrupt5() { fans[5].tacho.interrupt(); }
          #endif
        #endif
      #endif
    #endif
  #endif
#endif