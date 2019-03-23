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
#pragma once

/**
 * sanitycheck.h
 *
 * Test configuration values for errors at compile-time.
 */

// Advanced motion
#if DISABLED(DEFAULT_STEPPER_DEACTIVE_TIME)
  #error "DEPENDENCY ERROR: Missing setting DEFAULT_STEPPER_DEACTIVE_TIME."
#endif

#if ENABLED(STEPPER_HIGH_LOW)
  #if DISABLED(STEPPER_HIGH_LOW_DELAY)
    #error "DEPENDENCY ERROR: Missing setting STEPPER_HIGH_LOW_DELAY."
  #endif
#endif

#if ENABLED(DIGIPOT_I2C)
  #if DISABLED(DIGIPOT_I2C_NUM_CHANNELS)
    #error "DEPENDENCY ERROR: Missing setting DIGIPOT_I2C_NUM_CHANNELS."
  #endif
  #if DISABLED(DIGIPOT_I2C_MOTOR_CURRENTS)
    #error "DEPENDENCY ERROR: Missing setting DIGIPOT_I2C_MOTOR_CURRENTS."
  #endif
#endif

#if HAVE_DRV(TMC26X)
  #if X_HAS_DRV(TMC26X)
    #if DISABLED(X_CURRENT)
      #error "DEPENDENCY ERROR: Missing setting X_CURRENT."
    #endif
    #if DISABLED(X_SENSE_RESISTOR)
      #error "DEPENDENCY ERROR: Missing setting X_SENSE_RESISTOR."
    #endif
    #if DISABLED(X_MICROSTEPS)
      #error "DEPENDENCY ERROR: Missing setting X_MICROSTEPS."
    #endif
  #endif
  #if Y_HAS_DRV(TMC26X)
    #if DISABLED(Y_CURRENT)
      #error "DEPENDENCY ERROR: Missing setting Y_CURRENT."
    #endif
    #if DISABLED(Y_SENSE_RESISTOR)
      #error "DEPENDENCY ERROR: Missing setting Y_SENSE_RESISTOR."
    #endif
    #if DISABLED(Y_MICROSTEPS)
      #error "DEPENDENCY ERROR: Missing setting Y_MICROSTEPS."
    #endif
  #endif
  #if Z_HAS_DRV(TMC26X)
    #if DISABLED(Z_CURRENT)
      #error "DEPENDENCY ERROR: Missing setting Z_CURRENT."
    #endif
    #if DISABLED(Z_SENSE_RESISTOR)
      #error "DEPENDENCY ERROR: Missing setting Z_SENSE_RESISTOR."
    #endif
    #if DISABLED(Z_MICROSTEPS)
      #error "DEPENDENCY ERROR: Missing setting Z_MICROSTEPS."
    #endif
  #endif
  #if E0_HAS_DRV(TMC26X)
    #if DISABLED(E0_CURRENT)
      #error "DEPENDENCY ERROR: Missing setting E0_CURRENT."
    #endif
    #if DISABLED(E0_SENSE_RESISTOR)
      #error "DEPENDENCY ERROR: Missing setting E0_SENSE_RESISTOR."
    #endif
    #if DISABLED(E0_MICROSTEPS)
      #error "DEPENDENCY ERROR: Missing setting E0_MICROSTEPS."
    #endif
  #endif
  #if E1_HAS_DRV(TMC26X)
    #if DISABLED(E1_CURRENT)
      #error "DEPENDENCY ERROR: Missing setting E1_CURRENT."
    #endif
    #if DISABLED(E1_SENSE_RESISTOR)
      #error "DEPENDENCY ERROR: Missing setting E1_SENSE_RESISTOR."
    #endif
    #if DISABLED(E1_MICROSTEPS)
      #error "DEPENDENCY ERROR: Missing setting E1_MICROSTEPS."
    #endif
  #endif
  #if E2_HAS_DRV(TMC26X)
    #if DISABLED(E2_CURRENT)
      #error "DEPENDENCY ERROR: Missing setting E2_CURRENT."
    #endif
    #if DISABLED(E2_SENSE_RESISTOR)
      #error "DEPENDENCY ERROR: Missing setting E2_SENSE_RESISTOR."
    #endif
    #if DISABLED(E2_MICROSTEPS)
      #error "DEPENDENCY ERROR: Missing setting E2_MICROSTEPS."
    #endif
  #endif
  #if E3_HAS_DRV(TMC26X)
    #if DISABLED(E3_CURRENT)
      #error "DEPENDENCY ERROR: Missing setting E3_CURRENT."
    #endif
    #if DISABLED(E3_SENSE_RESISTOR)
      #error "DEPENDENCY ERROR: Missing setting E3_SENSE_RESISTOR."
    #endif
    #if DISABLED(E3_MICROSTEPS)
      #error "DEPENDENCY ERROR: Missing setting E3_MICROSTEPS."
    #endif
  #endif
#endif
#if HAVE_DRV(L6470)
  #if X_HAS_DRV(L6470)
    #if DISABLED(X_MICROSTEPS)
      #error "DEPENDENCY ERROR: Missing setting X_MICROSTEPS."
    #endif
    #if DISABLED(X_K_VAL)
      #error "DEPENDENCY ERROR: Missing setting X_K_VAL."
    #endif
    #if DISABLED(X_OVERCURRENT)
      #error "DEPENDENCY ERROR: Missing setting X_OVERCURRENT."
    #endif
    #if DISABLED(X_STALLCURRENT)
      #error "DEPENDENCY ERROR: Missing setting X_STALLCURRENT."
    #endif
  #endif
  #if Y_HAS_DRV(L6470)
    #if DISABLED(Y_MICROSTEPS)
      #error "DEPENDENCY ERROR: Missing setting Y_MICROSTEPS."
    #endif
    #if DISABLED(Y_K_VAL)
      #error "DEPENDENCY ERROR: Missing setting Y_K_VAL."
    #endif
    #if DISABLED(Y_OVERCURRENT)
      #error "DEPENDENCY ERROR: Missing setting Y_OVERCURRENT."
    #endif
    #if DISABLED(Y_STALLCURRENT)
      #error "DEPENDENCY ERROR: Missing setting Y_STALLCURRENT."
    #endif
  #endif
  #if Z_HAS_DRV(L6470)
    #if DISABLED(Z_MICROSTEPS)
      #error "DEPENDENCY ERROR: Missing setting Z_MICROSTEPS."
    #endif
    #if DISABLED(Z_K_VAL)
      #error "DEPENDENCY ERROR: Missing setting Z_K_VAL."
    #endif
    #if DISABLED(Z_OVERCURRENT)
      #error "DEPENDENCY ERROR: Missing setting Z_OVERCURRENT."
    #endif
    #if DISABLED(Z_STALLCURRENT)
      #error "DEPENDENCY ERROR: Missing setting Z_STALLCURRENT."
    #endif
  #endif
  #if E0_HAS_DRV(L6470)
    #if DISABLED(E0_MICROSTEPS)
      #error "DEPENDENCY ERROR: Missing setting E0_MICROSTEPS."
    #endif
    #if DISABLED(E0_K_VAL)
      #error "DEPENDENCY ERROR: Missing setting E0_K_VAL."
    #endif
    #if DISABLED(E0_OVERCURRENT)
      #error "DEPENDENCY ERROR: Missing setting E0_OVERCURRENT."
    #endif
    #if DISABLED(E0_STALLCURRENT)
      #error "DEPENDENCY ERROR: Missing setting E0_STALLCURRENT."
    #endif
  #endif
  #if E1_HAS_DRV(L6470)
    #if DISABLED(E1_MICROSTEPS)
      #error "DEPENDENCY ERROR: Missing setting E1_MICROSTEPS."
    #endif
    #if DISABLED(E1_K_VAL)
      #error "DEPENDENCY ERROR: Missing setting E1_K_VAL."
    #endif
    #if DISABLED(E1_OVERCURRENT)
      #error "DEPENDENCY ERROR: Missing setting E1_OVERCURRENT."
    #endif
    #if DISABLED(E1_STALLCURRENT)
      #error "DEPENDENCY ERROR: Missing setting E1_STALLCURRENT."
    #endif
  #endif
  #if E2_HAS_DRV(L6470)
    #if DISABLED(E2_MICROSTEPS)
      #error "DEPENDENCY ERROR: Missing setting E2_MICROSTEPS."
    #endif
    #if DISABLED(E2_K_VAL)
      #error "DEPENDENCY ERROR: Missing setting E2_K_VAL."
    #endif
    #if DISABLED(E2_OVERCURRENT)
      #error "DEPENDENCY ERROR: Missing setting E2_OVERCURRENT."
    #endif
    #if DISABLED(E2_STALLCURRENT)
      #error "DEPENDENCY ERROR: Missing setting E2_STALLCURRENT."
    #endif
  #endif
  #if E3_HAS_DRV(L6470)
    #if DISABLED(E3_MICROSTEPS)
      #error "DEPENDENCY ERROR: Missing setting E3_MICROSTEPS."
    #endif
    #if DISABLED(E3_K_VAL)
      #error "DEPENDENCY ERROR: Missing setting E3_K_VAL."
    #endif
    #if DISABLED(E3_OVERCURRENT)
      #error "DEPENDENCY ERROR: Missing setting E3_OVERCURRENT."
    #endif
    #if DISABLED(E3_STALLCURRENT)
      #error "DEPENDENCY ERROR: Missing setting E3_STALLCURRENT."
    #endif
  #endif
#endif
