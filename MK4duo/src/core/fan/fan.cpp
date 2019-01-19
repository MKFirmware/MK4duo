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
 * fan.cpp
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"

#if FAN_COUNT > 0
  Fan fans[FAN_COUNT];
#endif

/**
 * Initialize Fans
 */
void Fan::init() {

  Speed               = 0;
  paused_Speed        = 0;
  scaled_Speed        = 128;
  Kickstart           = 0;

  setIdle(false);

  if (printer.isRunning()) return; // All running not reinitialize

  if (data.pin > 0) HAL::pinMode(data.pin, isHWInverted() ? OUTPUT_HIGH : OUTPUT_LOW);

}

void Fan::setAutoMonitored(const int8_t h) {
  if (WITHIN(h, 0, HOTENDS - 1) || h == 7)
    SBI(data.autoMonitored, (uint8_t)h);
  else      
    data.autoMonitored = 0;
  spin();
}

void Fan::setOutputPwm() {
  const uint8_t new_Speed = isHWInverted() ? 255 - actual_Speed() : actual_Speed();
  HAL::analogWrite(data.pin, new_Speed, data.freq);
}

void Fan::spin() {

  static watch_t controller_fan_watch(CONTROLLERFAN_SECS * 1000UL);

  if (data.autoMonitored != 0) {

    // Check for Hotend temperature
    LOOP_HOTEND() {
      if (TEST(data.autoMonitored, h)) {
        if (heaters[h].current_temperature > data.triggerTemperature) {
          Speed = data.max_Speed;
          break;
        }
        else
          Speed = data.min_Speed;
      }
    }

    // Check for Controller fan
    if (TEST(data.autoMonitored, 7)) {

      // Check Heaters
      if (thermalManager.heaters_isActive()) controller_fan_watch.start();

      #if HAS_MCU_TEMPERATURE
        // Check MSU
        if (thermalManager.mcu_current_temperature >= 50) controller_fan_watch.start();
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
        controller_fan_watch.start();
      }

      // Fan off if no steppers or heaters have been enabled for CONTROLLERFAN_SECS seconds
      Speed = controller_fan_watch.elapsed() ? data.min_Speed : data.max_Speed;
    }

  }

  Speed = Speed ? constrain(Speed, data.min_Speed, data.max_Speed) : 0;

}

void Fan::print_M106() {
  bool found_auto = false;
  SERIAL_LM(CFG, "Fans: P<Fan> U<Pin> L<Min Speed> X<Max Speed> F<Freq> I<Hardware Inverted 0-1> H<Auto mode> T<Trig Temp>");
  SERIAL_SMV(CFG, "  M106 P", (int)data.ID);
  SERIAL_MV(" U", data.pin);
  SERIAL_MV(" L", data.min_Speed);
  SERIAL_MV(" X", data.max_Speed);
  SERIAL_MV(" F", data.freq);
  SERIAL_MV(" I", isHWInverted());
  SERIAL_MSG(" H");
  LOOP_HOTEND() {
    if (TEST(data.autoMonitored, h)) {
      SERIAL_VAL((int)h);
      SERIAL_MV(" T", data.triggerTemperature);
      found_auto = true;
      break;
    }
  }
  if (!found_auto) {
    if (TEST(data.autoMonitored, 7))
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