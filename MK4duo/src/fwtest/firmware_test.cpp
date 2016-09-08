/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2016 Alberto Cotronei @MagoKimbra
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

#include "../../base.h"

#if ENABLED(FIRMWARE_TEST)

#include "firmware_test.h"

static char serial_answer;

void FirmwareTest() {
  SERIAL_EM("---------- FIRMWARE TEST --------------");
  SERIAL_EM("--------------- MK --------------------");
  SERIAL_EM(MSG_FWTEST_01);
  SERIAL_EM(MSG_FWTEST_02);
  SERIAL_EM(MSG_FWTEST_YES_NO);
  serial_answer = ' ';
  while (serial_answer!='y' && serial_answer!='Y' && serial_answer!='n' && serial_answer!='N') {
    serial_answer = MKSERIAL.read();
  }
  if (serial_answer=='y' || serial_answer=='Y') {
    SERIAL_EM(MSG_FWTEST_03);

    SERIAL_EM(" ");
    SERIAL_EM("***** ENDSTOP X *****");
    #if PIN_EXISTS(X_MIN) && (X_HOME_DIR == -1)
      if (!READ(X_MIN_PIN)^X_MIN_ENDSTOP_INVERTING) {
        SERIAL_M("MIN ENDSTOP X: ");
        SERIAL_EM(MSG_ENDSTOP_OPEN);
      }
      else {
        SERIAL_M("X ENDSTOP ");
        SERIAL_EM(MSG_FWTEST_ERROR);
        SERIAL_M(MSG_FWTEST_INVERT);
        SERIAL_M("#define X_MIN_ENDSTOP_LOGIC ");
        SERIAL_M(MSG_FWTEST_INTO);
        #if MECH(CARTESIAN)
          SERIAL_EM("Configuration_Cartesian.h");
        #elif MECH(COREXY)
          SERIAL_EM("Configuration_Core.h");
        #elif MECH(COREXZ)
          SERIAL_EM("Configuration_Core.h");
        #elif MECH(DELTA)
          SERIAL_EM("Configuration_Delta.h");
        #elif MECH(SCARA)
          SERIAL_EM("Configuration_Scara.h");
        #endif
        return;
      }
      SERIAL_M(MSG_FWTEST_PRESS);
      SERIAL_EM("X");
      SERIAL_EM(MSG_FWTEST_YES);
      serial_answer = ' ';
      while (serial_answer!='y' && serial_answer!='Y' && !(READ(X_MIN_PIN)^X_MIN_ENDSTOP_INVERTING)) {
        serial_answer = MKSERIAL.read();
      }
      if (READ(X_MIN_PIN)^X_MIN_ENDSTOP_INVERTING) {
        SERIAL_M("MIN ENDSTOP X: ");
        SERIAL_EM(MSG_ENDSTOP_HIT);
      }
      else {
        SERIAL_M("X ");
        SERIAL_EM(MSG_FWTEST_ENDSTOP_ERR);
        return;
      }
    #elif PIN_EXISTS(X_MAX) && X_HOME_DIR == 1
      if (!READ(X_MAX_PIN)^X_MAX_ENDSTOP_INVERTING) {
        SERIAL_M("MAX ENDSTOP X: ");
        SERIAL_EM(MSG_ENDSTOP_OPEN);
      }
      else {
        SERIAL_M("X ENDSTOP ");
        SERIAL_EM(MSG_FWTEST_ERROR);
        SERIAL_M(MSG_FWTEST_INVERT);
        SERIAL_M("#define X_MAX_ENDSTOP_LOGIC ");
        SERIAL_M(MSG_FWTEST_INTO);
        #if MECH(CARTESIAN)
          SERIAL_EM("Configuration_Cartesian.h");
        #elif MECH(COREXY)
          SERIAL_EM("Configuration_Core.h");
        #elif MECH(COREXZ)
          SERIAL_EM("Configuration_Core.h");
        #elif MECH(DELTA)
          SERIAL_EM("Configuration_Delta.h");
        #elif MECH(SCARA)
          SERIAL_EM("Configuration_Scara.h");
        #endif
        return;
      }
      SERIAL_V(MSG_FWTEST_PRESS);
      SERIAL_EM("X");
      SERIAL_EM(MSG_FWTEST_YES);
      serial_answer = ' ';
      while (serial_answer!='y' && serial_answer!='Y' && !(READ(X_MAX_PIN)^X_MAX_ENDSTOP_INVERTING)) {
        serial_answer = MKSERIAL.read();
      }
      if (READ(X_MAX_PIN)^X_MAX_ENDSTOP_INVERTING) {
        SERIAL_M("MAX ENDSTOP X: ");
        SERIAL_EM(MSG_ENDSTOP_HIT);
      }
      else {
        SERIAL_M("X ");
        SERIAL_EM(MSG_FWTEST_ENDSTOP_ERR);
        return;
      }
    #elif X_HOME_DIR == -1
      SERIAL_M(MSG_FWTEST_ERROR);
      SERIAL_M("!!! X_MIN_PIN ");
      SERIAL_EM(MSG_FWTEST_NDEF);
      return;
    #elif X_HOME_DIR == 1
      SERIAL_M(MSG_FWTEST_ERROR);
      SERIAL_M("!!! X_MAX_PIN ");
      SERIAL_EM(MSG_FWTEST_NDEF);
      return;
    #endif

    SERIAL_EM(" ");
    SERIAL_EM("***** ENDSTOP Y *****");
    #if PIN_EXISTS(Y_MIN) && Y_HOME_DIR == -1
      if (!READ(Y_MIN_PIN)^Y_MIN_ENDSTOP_INVERTING) {
        SERIAL_M("MIN ENDSTOP Y: ");
        SERIAL_EM(MSG_ENDSTOP_OPEN);
      }
      else {
        SERIAL_M("Y ENDSTOP ");
        SERIAL_EM(MSG_FWTEST_ERROR);
        SERIAL_M(MSG_FWTEST_INVERT);
        SERIAL_M("#define Y_MIN_ENDSTOP_LOGIC ");
        SERIAL_M(MSG_FWTEST_INTO);
        #if MECH(CARTESIAN)
          SERIAL_EM("Configuration_Cartesian.h");
        #elif MECH(COREXY)
          SERIAL_EM("Configuration_Core.h");
        #elif MECH(COREXZ)
          SERIAL_EM("Configuration_Core.h");
        #elif MECH(DELTA)
          SERIAL_EM("Configuration_Delta.h");
        #elif MECH(SCARA)
          SERIAL_EM("Configuration_Scara.h");
        #endif
        return;
      }
      SERIAL_V(MSG_FWTEST_PRESS);
      SERIAL_EM("Y");
      SERIAL_EM(MSG_FWTEST_YES);
      serial_answer = ' ';
      while (serial_answer!='y' && serial_answer!='Y' && !(READ(Y_MIN_PIN)^Y_MIN_ENDSTOP_INVERTING)) {
        serial_answer = MKSERIAL.read();
      }
      if (READ(Y_MIN_PIN)^Y_MIN_ENDSTOP_INVERTING) {
        SERIAL_M("MIN ENDSTOP Y: ");
        SERIAL_EM(MSG_ENDSTOP_HIT);
      }
      else {
        SERIAL_M("Y ");
        SERIAL_EM(MSG_FWTEST_ENDSTOP_ERR);
        return;
      }
    #elif PIN_EXISTS(Y_MAX) && Y_HOME_DIR == 1
      if (!READ(Y_MAX_PIN)^Y_MAX_ENDSTOP_INVERTING) {
        SERIAL_M("MAX ENDSTOP Y: ");
        SERIAL_EM(MSG_ENDSTOP_OPEN);
      }
      else {
        SERIAL_M("Y ENDSTOP ");
        SERIAL_EM(MSG_FWTEST_ERROR);
        SERIAL_M(MSG_FWTEST_INVERT);
        SERIAL_M("#define Y_MAX_ENDSTOP_LOGIC ");
        SERIAL_M(MSG_FWTEST_INTO);
        #if MECH(CARTESIAN)
          SERIAL_EM("Configuration_Cartesian.h");
        #elif MECH(COREXY)
          SERIAL_EM("Configuration_Core.h");
        #elif MECH(COREXZ)
          SERIAL_EM("Configuration_Core.h");
        #elif MECH(DELTA)
          SERIAL_EM("Configuration_Delta.h");
        #elif MECH(SCARA)
          SERIAL_EM("Configuration_Scara.h");
        #endif
        return;
      }
      SERIAL_V(MSG_FWTEST_PRESS);
      SERIAL_EM("Y");
      SERIAL_EM(MSG_FWTEST_YES);
      serial_answer = ' ';
      while (serial_answer!='y' && serial_answer!='Y' && !(READ(Y_MAX_PIN)^Y_MAX_ENDSTOP_INVERTING)) {
        serial_answer = MKSERIAL.read();
      }
      if (READ(Y_MAX_PIN)^Y_MAX_ENDSTOP_INVERTING) {
        SERIAL_M("MAX ENDSTOP Y: ");
        SERIAL_EM(MSG_ENDSTOP_HIT);
      }
      else {
        SERIAL_M("Y ");
        SERIAL_EM(MSG_FWTEST_ENDSTOP_ERR);
        return;
      }
    #elif Y_HOME_DIR == -1
      SERIAL_M(MSG_FWTEST_ERROR);
      SERIAL_M("!!! Y_MIN_PIN ");
      SERIAL_EM(MSG_FWTEST_NDEF);
      return;
    #elif Y_HOME_DIR == 1
      SERIAL_M(MSG_FWTEST_ERROR);
      SERIAL_M("!!! Y_MAX_PIN ");
      SERIAL_EM(MSG_FWTEST_NDEF);
      return;
    #endif

    SERIAL_EM(" ");
    SERIAL_EM("***** ENDSTOP Z *****");
    #if PIN_EXISTS(Z_MIN) && Z_HOME_DIR == -1
      if (!READ(Z_MIN_PIN)^Z_MIN_ENDSTOP_INVERTING) {
        SERIAL_M("MIN ENDSTOP Z: ");
        SERIAL_EM(MSG_ENDSTOP_OPEN);
      }
      else {
        SERIAL_M("Z ENDSTOP ");
        SERIAL_EM(MSG_FWTEST_ERROR);
        SERIAL_M(MSG_FWTEST_INVERT);
        SERIAL_M("#define Z_MIN_ENDSTOP_LOGIC ");
        SERIAL_M(MSG_FWTEST_INTO);
        #if MECH(CARTESIAN)
          SERIAL_EM("Configuration_Cartesian.h");
        #elif MECH(COREXY)
          SERIAL_EM("Configuration_Core.h");
        #elif MECH(COREXZ)
          SERIAL_EM("Configuration_Core.h");
        #elif MECH(DELTA)
          SERIAL_EM("Configuration_Delta.h");
        #elif MECH(SCARA)
          SERIAL_EM("Configuration_Scara.h");
        #endif
        return;
      }
      SERIAL_V(MSG_FWTEST_PRESS);
      SERIAL_EM("Z");
      SERIAL_EM(MSG_FWTEST_YES);
      serial_answer = ' ';
      while (serial_answer!='y' && serial_answer!='Y' && !(READ(Z_MIN_PIN)^Z_MIN_ENDSTOP_INVERTING)) {
        serial_answer = MKSERIAL.read();
      }
      if (READ(Z_MIN_PIN)^Z_MIN_ENDSTOP_INVERTING) {
        SERIAL_M("MIN ENDSTOP Z: ");
        SERIAL_EM(MSG_ENDSTOP_HIT);
      }
      else {
        SERIAL_M("Z ");
        SERIAL_EM(MSG_FWTEST_ENDSTOP_ERR);
        return;
      }
    #elif PIN_EXISTS(Z_MAX) && Z_HOME_DIR == 1
      if (!READ(Z_MAX_PIN)^Z_MAX_ENDSTOP_INVERTING) {
        SERIAL_M("MAX ENDSTOP Z: ");
        SERIAL_EM(MSG_ENDSTOP_OPEN);
      }
      else {
        SERIAL_M("Z ENDSTOP ");
        SERIAL_EM(MSG_FWTEST_ERROR);
        SERIAL_M(MSG_FWTEST_INVERT);
        SERIAL_M("#define Z_MAX_ENDSTOP_LOGIC ");
        SERIAL_M(MSG_FWTEST_INTO);
        #if MECH(CARTESIAN)
          SERIAL_EM("Configuration_Cartesian.h");
        #elif MECH(COREXY)
          SERIAL_EM("Configuration_Core.h");
        #elif MECH(COREXZ)
          SERIAL_EM("Configuration_Core.h");
        #elif MECH(DELTA)
          SERIAL_EM("Configuration_Delta.h");
        #elif MECH(SCARA)
          SERIAL_EM("Configuration_Scara.h");
        #endif
        return;
      }
      SERIAL_V(MSG_FWTEST_PRESS);
      SERIAL_EM("Z");
      SERIAL_EM(MSG_FWTEST_YES);
      serial_answer = ' ';
      while (serial_answer!='y' && serial_answer!='Y' && !(READ(Z_MAX_PIN)^Z_MAX_ENDSTOP_INVERTING)) {
        serial_answer = MKSERIAL.read();
      }
      if (READ(Z_MAX_PIN)^Z_MAX_ENDSTOP_INVERTING) {
        SERIAL_M("MAX ENDSTOP Z: ");
        SERIAL_EM(MSG_ENDSTOP_HIT);
      }
      else {
        SERIAL_M("Z ");
        SERIAL_EM(MSG_FWTEST_ENDSTOP_ERR);
        return;
      }
    #elif Z_HOME_DIR == -1
      SERIAL_M(MSG_FWTEST_ERROR);
      SERIAL_M("!!! Z_MIN_PIN ");
      SERIAL_EM(MSG_FWTEST_NDEF);
      return;
    #elif Z_HOME_DIR == 1
      SERIAL_M(MSG_FWTEST_ERROR);
      SERIAL_M("!!! Z_MAX_PIN ");
      SERIAL_EM(MSG_FWTEST_NDEF);
      return;
    #endif

    SERIAL_EM("ENDSTOP ");
    SERIAL_M(MSG_FWTEST_OK);
    SERIAL_EM(" ");
  }

  #if HAS(POWER_SWITCH)
    SET_OUTPUT(PS_ON_PIN);
    WRITE(PS_ON_PIN, PS_ON_AWAKE);
  #endif

  // Reset position to 0
  stepper.synchronize();
  for (int8_t i = 0; i < NUM_AXIS; i++) current_position[i] = 0;
  planner.set_position_mm(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

  SERIAL_EM("***** TEST MOTOR  *****");
  SERIAL_EM(MSG_FWTEST_ATTENTION);
  SERIAL_EM(MSG_FWTEST_YES);
  serial_answer = ' ';
  while (serial_answer!='y' && serial_answer!='Y') {
    serial_answer = MKSERIAL.read();
  }
  SERIAL_EM(MSG_FWTEST_04);
  SERIAL_EM(" ");
  SERIAL_EM("***** MOTOR X *****");
  destination[X_AXIS] = 10;
  prepare_move();
  stepper.synchronize();

  SERIAL_EM(MSG_FWTEST_XAXIS);
  SERIAL_EM(MSG_FWTEST_YES_NO);
  serial_answer = ' ';
  while (serial_answer!='y' && serial_answer!='Y' && serial_answer!='n' && serial_answer!='N') {
    serial_answer = MKSERIAL.read();
  }
  if (serial_answer=='y' || serial_answer=='Y') {
    SERIAL_EM("MOTOR X ");
    SERIAL_M(MSG_FWTEST_OK);
  }
  else {
    SERIAL_M(MSG_FWTEST_INVERT);
    SERIAL_M("#define INVERT_X_DIR ");
    SERIAL_M(MSG_FWTEST_INTO);
    #if MECH(CARTESIAN)
      SERIAL_EM("Configuration_Cartesian.h");
    #elif MECH(COREXY)
      SERIAL_EM("Configuration_Core.h");
    #elif MECH(COREXZ)
      SERIAL_EM("Configuration_Core.h");
    #elif MECH(DELTA)
      SERIAL_EM("Configuration_Delta.h");
    #elif MECH(SCARA)
      SERIAL_EM("Configuration_Scara.h");
    #endif
    return;
  }
  SERIAL_EM(" ");
  SERIAL_EM("***** MOTOR Y *****");
  destination[Y_AXIS] = 10;
  prepare_move();
  stepper.synchronize();
  SERIAL_EM(MSG_FWTEST_YAXIS);
  SERIAL_EM(MSG_FWTEST_YES_NO);
  serial_answer = ' ';
  while (serial_answer!='y' && serial_answer!='Y' && serial_answer!='n' && serial_answer!='N') {
    serial_answer = MKSERIAL.read();
  }
  if (serial_answer=='y' || serial_answer=='Y') {
    SERIAL_EM("MOTOR Y ");
    SERIAL_M(MSG_FWTEST_OK);
  }
  else {
    SERIAL_M(MSG_FWTEST_INVERT);
    SERIAL_M("#define INVERT_Y_DIR ");
    SERIAL_M(MSG_FWTEST_INTO);
    #if MECH(CARTESIAN)
      SERIAL_EM("Configuration_Cartesian.h");
    #elif MECH(COREXY)
      SERIAL_EM("Configuration_Core.h");
    #elif MECH(COREXZ)
      SERIAL_EM("Configuration_Core.h");
    #elif MECH(DELTA)
      SERIAL_EM("Configuration_Delta.h");
    #elif MECH(SCARA)
      SERIAL_EM("Configuration_Scara.h");
    #endif
    return;
  }
  SERIAL_EM(" ");
  SERIAL_EM("***** MOTOR Z *****");
  destination[Z_AXIS] = 10;
  prepare_move();
  stepper.synchronize();
  SERIAL_EM(MSG_FWTEST_ZAXIS);
  SERIAL_EM(MSG_FWTEST_YES_NO);
  serial_answer = ' ';
  while (serial_answer!='y' && serial_answer!='Y' && serial_answer!='n' && serial_answer!='N') {
    serial_answer = MKSERIAL.read();
  }
  if (serial_answer=='y' || serial_answer=='Y') {
    SERIAL_EM("MOTOR Z ");
    SERIAL_M(MSG_FWTEST_OK);
  }
  else {
    SERIAL_M(MSG_FWTEST_INVERT);
    SERIAL_M("#define INVERT_Z_DIR ");
    SERIAL_M(MSG_FWTEST_INTO);
    #if MECH(CARTESIAN)
      SERIAL_EM("Configuration_Cartesian.h");
    #elif MECH(COREXY)
      SERIAL_EM("Configuration_Core.h");
    #elif MECH(COREXZ)
      SERIAL_EM("Configuration_Core.h");
    #elif MECH(DELTA)
      SERIAL_EM("Configuration_Delta.h");
    #elif MECH(SCARA)
      SERIAL_EM("Configuration_Scara.h");
    #endif
    return;
  }
  SERIAL_EM("MOTOR ");
  SERIAL_M(MSG_FWTEST_OK);
  SERIAL_EM(" ");
  SERIAL_V(MSG_FWTEST_END);
}
#endif
