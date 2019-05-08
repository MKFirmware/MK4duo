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
  #if ENABLED(BLTOUCH_FORCE_5V_MODE)
    cmd_5V_mode();
  #endif
  clear();
}

bool Bltouch::test() {
  #if HAS_Z_PROBE_PIN
    return HAL::digitalRead(Z_PROBE_PIN) != endstops.isLogic(Z_PROBE);
  #else
    return HAL::digitalRead(Z_MIN_PIN) != endstops.isLogic(Z_MIN);
  #endif
}

bool Bltouch::deploy() {
  // Do a DEPLOY
  if (printer.debugFeature()) DEBUG_EM("BLTouch DEPLOY requested");

  // Attempt to DEPLOY, wait for DEPLOY_DELAY or ALARM
  if (deploy_query_alarm()) {
    // The deploy might have failed or the probe is already triggered (nozzle too low?)
    if (printer.debugFeature()) DEBUG_EM("BLTouch ALARM or TRIGGER after DEPLOY, recovering");

    clear();                            // Get the probe into start condition

    // Last attempt to DEPLOY
    if (deploy_query_alarm()) {
      // The deploy might have failed or the probe is actually triggered (nozzle too low?) again
      if (printer.debugFeature()) DEBUG_EM("BLTouch Recovery Failed");

      SERIAL_LM(ER, MSG_STOP_BLTOUCH);  // Tell the user something is wrong, needs action
      printer.stop();                   // but it's not too bad, no need to kill, allow restart

      return true;                      // Tell our caller we goofed in case he cares to know
    }
  }

  // Now the probe is ready to issue a 10ms pulse when the pin goes up.
  // The trigger STOW (see motion.cpp for example) will pull up the probes pin as soon as the pulse
  // is registered.

  if (printer.debugFeature()) DEBUG_EM("bltouch.deploy() end");

  return false; // report success to caller
}

bool Bltouch::stow() {
  // Do a STOW
  if (printer.debugFeature()) DEBUG_EM("BLTouch STOW requested");

  // A STOW will clear a triggered condition in the probe (10ms pulse).
  // At the moment that we come in here, we might (pulse) or will (SW mode) see the trigger on the pin.
  // So even though we know a STOW will be ignored if an ALARM condition is active, we will STOW.
  // Note: If the probe is deployed AND in an ALARM condition, this STOW will not pull up the pin
  // and the ALARM condition will still be there. --> ANTClabs should change this behaviour maybe

  // Attempt to STOW, wait for STOW_DELAY or ALARM
  if (stow_query_alarm()) {
    // The stow might have failed
    if (printer.debugFeature()) DEBUG_EM("BLTouch ALARM or TRIGGER after STOW, recovering");

    cmd_reset();                          // This RESET will then also pull up the pin. If it doesn't
                                          // work and the pin is still down, there will no longer be
                                          // an ALARM condition though.
                                          // But one more STOW will catch that
    // Last attempt to STOW
    if (stow_query_alarm()) {             // so if there is now STILL an ALARM condition:

      if (printer.debugFeature()) DEBUG_EM("BLTouch Recovery Failed");

      SERIAL_LM(ER, MSG_STOP_BLTOUCH);    // Tell the user something is wrong, needs action
      printer.stop();                     // but it's not too bad, no need to kill, allow restart

      return true;                        // Tell our caller we goofed in case he cares to know
    }
  }

  if (printer.debugFeature()) DEBUG_EM("bltouch.stow() end");

  return false; // report success to caller
}

bool Bltouch::status() {
  /**
   * Return a TRUE for "YES, it is DEPLOYED"
   * This function will ensure switch state is reset after execution
   * This may change pin position in some scenarios, specifically
   * if the pin has been triggered but not yet stowed.
   */

  if (printer.debugFeature()) DEBUG_EM("BLTouch STATUS requested");

  cmd_SW_mode();
  const bool trig = test();         // If triggered in SW mode, the pin is up, it is STOWED

  if (printer.debugFeature()) DEBUG_EMV("BLTouch is ", (int)trig);

  cmd_reset();                      // turn off the SW Mode
  if (trig) stow();
  else deploy();                    // and reset any triggered signal, restore state
  return !trig;
}

/** Private Functions */
void Bltouch::clear() {
  cmd_reset();  // RESET or RESET_SW will clear an alarm condition but...
                // ...it will not clear a triggered condition in SW mode when the pin is currently up
                // ANTClabs <-- CODE ERROR
  stow();       // STOW will pull up the pin and clear any triggered condition unless it fails, don't care
  deploy();     // DEPLOY to test the probe. Could fail, don't care
  stow();       // STOW to be ready for meaningful work. Could fail, don't care
}

bool Bltouch::command(const BLTCommand cmd, const millis_s &ms) {
  if (printer.debugFeature()) SERIAL_EMV("BLTouch Command :", cmd);
  MOVE_SERVO(Z_PROBE_SERVO_NR, cmd);
  printer.safe_delay(MAX(ms, BLTOUCH_DELAY));
  return test();
}

#endif // BLTOUCH
