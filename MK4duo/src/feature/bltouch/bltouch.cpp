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
 * bltouch.cpp
 *
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"

#if ENABLED(BLTOUCH)

  Bltouch bltouch;

  /** Public Function */
  void Bltouch::init() {
    // Make sure any BLTouch error condition is cleared
    move(BLTOUCH_RESET);
    set_deployed(true);
    set_deployed(false);
  }

  bool Bltouch::test() {
    #if HAS_Z_PROBE_PIN
      return READ(Z_PROBE_PIN) != endstops.isLogic(Z_PROBE);
    #else
      return READ(Z_MIN_PIN) != endstops.isLogic(Z_MIN);
    #endif
  }

  // returns false for ok and true for failure
  bool Bltouch::set_deployed(const bool deploy) {
    if (deploy && test()) {     // If BL-Touch says it's triggered
      move(BLTOUCH_RESET);      // try to reset it.
      move(BLTOUCH_DEPLOY);     // Also needs to deploy and stow to
      move(BLTOUCH_STOW);       // clear the triggered condition.
      printer.safe_delay(1500); // wait for internal self test to complete
                                //   measured completion time was 0.65 seconds
                                //   after reset, deploy & stow sequence

      if (test()) {             // If it still claims to be triggered...
        SERIAL_LM(ER, MSG_STOP_BLTOUCH);
        sound.feedback(false);
        printer.Stop();
        return true;
      }
    }

    move(deploy ? BLTOUCH_DEPLOY : BLTOUCH_STOW);

    #if ENABLED(DEBUG_FEATURE)
      if (printer.debugFeature()) {
        SERIAL_MV("bltouch.set_deployed(", deploy);
        SERIAL_CHR(')'); SERIAL_EOL();
      }
    #endif

    return false;
  }

  /** Private Function */
  void Bltouch::move(const int angle) {
    MOVE_SERVO(Z_PROBE_SERVO_NR, angle);  // Give the BL-Touch the move and wait
    printer.safe_delay(BLTOUCH_DELAY);
  }

#endif // BLTOUCH
