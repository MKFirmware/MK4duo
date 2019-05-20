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

BLTouch bltouch;

/** Public Function */
void BLTouch::init() {
  #if ENABLED(BLTOUCH_FORCE_5V_MODE)
    // BLTOUCH < V3.0 and clones: This will be ignored
    // BLTOUCH V3.0: SET_5V_MODE.
    //               This mode will stay active until manual SET_OD_MODE or power cycle
    // BLTOUCH V3.1: SET_5V_MODE. If not the probe will default to the eeprom settings configured by the user
    deploy();
    cmd_mode_5V();
    cmd_mode_store();
    cmd_mode_5V();
    stow();
  #endif
  cmd_reset();
  cmd_stow();
}

void BLTouch::test() {

  bool probe_logic, deploy_state, stow_state;

  SERIAL_EM("BLTouch test.");
  SERIAL_EMV(".  Using index:  ", 0);
  SERIAL_EMV(".  Deploy angle: ", servo[0].angle[0]);
  SERIAL_EMV(".  Stow angle:   ", servo[0].angle[1]);

  #if HAS_Z_PROBE_PIN

    #define PROBE_TEST_PIN Z_PROBE_PIN
    probe_logic = endstops.isLogic(Z_PROBE);
    SERIAL_EMV(". BLTouch uses Z_MIN_PROBE_PIN: ", PROBE_TEST_PIN);
    SERIAL_EM(". Uses Z_PROBE_ENDSTOP_LOGIC (ignores Z_MIN_ENDSTOP_LOGIC)");
    SERIAL_ELOGIC(". Z_MIN_ENDSTOP_LOGIC:", probe_logic);

  #elif HAS_Z_MIN

    #define PROBE_TEST_PIN Z_MIN_PIN
    probe_logic = endstops.isLogic(Z_MIN);
    SERIAL_EMV(". BLTouch uses Z_MIN pin: ", PROBE_TEST_PIN);
    SERIAL_EM(". Uses Z_MIN_ENDSTOP_LOGIC (ignores Z_PROBE_ENDSTOP_LOGIC)");
    SERIAL_ELOGIC(". Z_MIN_ENDSTOP_LOGIC:", probe_logic);

  #endif

  cmd_reset();
  cmd_stow();

  // DEPLOY and STOW 4 times and see if the signal follows
  uint8_t i = 0;
  SERIAL_EM(". Deploy & stow 4 times");
  do {
    cmd_deploy();
    deploy_state = HAL::digitalRead(PROBE_TEST_PIN);
    cmd_stow();
    stow_state = HAL::digitalRead(PROBE_TEST_PIN);
  } while (++i < 4);

  if (probe_logic != deploy_state) SERIAL_EM("WARNING: INVERTING setting probably backwards.");

  if (deploy_state != stow_state) {
    SERIAL_EM("= BLTouch clone detected");
    if (deploy_state) {
      SERIAL_EM(".  DEPLOYED state: HIGH (logic 1)");
      SERIAL_EM(".  STOWED (triggered) state: LOW (logic 0)");
    }
    else {
      SERIAL_EM(".  DEPLOYED state: LOW (logic 0)");
      SERIAL_EM(".  STOWED (triggered) state: HIGH (logic 1)");
    }
    SERIAL_EM("FAIL: BLTOUCH enabled - Set up this device as a Servo Probe with INVERTING set to 'true'.");
    return;
  }

  // Ask the user for a trigger event and measure the pulse width.
  cmd_deploy();
  SERIAL_EM("** Please trigger probe within 30 sec **");
  uint16_t probe_counter = 0;

  // Wait 30 seconds for user to trigger probe
  for (uint16_t j = 0; j < 500 * 30 && probe_counter == 0 ; j++) {

    printer.safe_delay(2);

    if (0 == j % (500 * 1)) printer.reset_move_ms();          // Keep steppers powered

    if (deploy_state != HAL::digitalRead(PROBE_TEST_PIN)) {   // probe triggered

      for (probe_counter = 1; probe_counter < 15 && deploy_state != HAL::digitalRead(PROBE_TEST_PIN); ++probe_counter)
        printer.safe_delay(2);

      SERIAL_EMV(". Pulse width (+/- 4mS): ", probe_counter * 2);

      if (probe_counter >= 4) {
        if (probe_counter == 15) SERIAL_MSG("= BLTouch V3.1");
        else SERIAL_MSG("= BLTouch pre V3.1 or compatible probe");
        SERIAL_EM(" detected.");
      }
      else SERIAL_EM("FAIL: Noise detected - please re-run test");

      cmd_stow();

      return;

    }  // pulse detected

  } // for loop waiting for trigger

  if (probe_counter == 0) SERIAL_LM(ER, " Trigger not detected");
}

bool BLTouch::triggered() {
  #if HAS_Z_PROBE_PIN
    return HAL::digitalRead(Z_PROBE_PIN) != endstops.isLogic(Z_PROBE);
  #else
    return HAL::digitalRead(Z_MIN_PIN) != endstops.isLogic(Z_MIN);
  #endif
}

bool BLTouch::deploy() {
  // Do a DEPLOY
  if (printer.debugFeature()) DEBUG_EM(">>> bltouch.deploy() start");

  // Attempt to DEPLOY, wait for DEPLOY_DELAY or ALARM
  if (cmd_deploy_alarm()) {
    // The deploy might have failed or the probe is already triggered (nozzle too low?)
    if (printer.debugFeature()) DEBUG_EM("BLTouch ALARM or TRIGGER after DEPLOY, recovering");

    clear();                            // Get the probe into start condition

    // Last attempt to DEPLOY
    if (cmd_deploy_alarm()) {
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

  if (printer.debugFeature()) DEBUG_EM("<<< bltouch.deploy() end");

  return false; // report success to caller
}

bool BLTouch::stow() {
  // Do a STOW
  if (printer.debugFeature()) DEBUG_EM(">>> bltouch.stow() start");

  // A STOW will clear a triggered condition in the probe (10ms pulse).
  // At the moment that we come in here, we might (pulse) or will (SW mode) see the trigger on the pin.
  // So even though we know a STOW will be ignored if an ALARM condition is active, we will STOW.
  // Note: If the probe is deployed AND in an ALARM condition, this STOW will not pull up the pin
  // and the ALARM condition will still be there. --> ANTClabs should change this behaviour maybe

  // Attempt to STOW, wait for STOW_DELAY or ALARM
  if (cmd_stow_alarm()) {
    // The stow might have failed
    if (printer.debugFeature()) DEBUG_EM("BLTouch ALARM or TRIGGER after STOW, recovering");

    cmd_reset();                        // This RESET will then also pull up the pin. If it doesn't
                                        // work and the pin is still down, there will no longer be
                                        // an ALARM condition though.
                                        // But one more STOW will catch that
    // Last attempt to STOW
    if (cmd_stow_alarm()) {             // so if there is now STILL an ALARM condition:

      if (printer.debugFeature()) DEBUG_EM("BLTouch Recovery Failed");

      SERIAL_LM(ER, MSG_STOP_BLTOUCH);  // Tell the user something is wrong, needs action
      printer.stop();                   // but it's not too bad, no need to kill, allow restart

      return true;                      // Tell our caller we goofed in case he cares to know
    }
  }

  if (printer.debugFeature()) DEBUG_EM("<<< bltouch.stow() end");

  return false; // report success to caller
}

bool BLTouch::status() {
  // Return a TRUE for "YES, it is DEPLOYED"
  // This function will ensure switch state is reset after execution
  if (printer.debugFeature()) DEBUG_EM("BLTouch STATUS requested");

  cmd_mode_SW();
  const bool trig = triggered();        // If triggered in mode SW, the pin is up, it is STOWED

  if (printer.debugFeature()) DEBUG_ELOGIC("BLTouch is ", trig);

  if (trig) stow(); else deploy();      // Turn off mode SW, reset any trigger
  return !trig;
}

/** Private Functions */
void BLTouch::clear() {
  cmd_reset();  // RESET or RESET_SW will clear an alarm condition but...
                // ...it will not clear a triggered condition in SW mode when the pin is currently up
                // ANTClabs <-- CODE ERROR
  stow();       // STOW will pull up the pin and clear any triggered condition unless it fails, don't care
  deploy();     // DEPLOY to test the probe. Could fail, don't care
  stow();       // STOW to be ready for meaningful work. Could fail, don't care
}

bool BLTouch::command(const BLTCommand cmd, const millis_s ms/*=BLTOUCH_DELAY*/) {
  if (printer.debugFeature()) SERIAL_EMV("BLTouch Command :", cmd);
  MOVE_SERVO(Z_PROBE_SERVO_NR, cmd);
  printer.safe_delay(MAX(ms, BLTOUCH_DELAY));
  return triggered();
}

#endif // BLTOUCH
