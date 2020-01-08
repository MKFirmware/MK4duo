/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
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

/**
 * Check existing CS pins against enabled TMC SPI drivers.
 */
#define INVALID_TMC_SPI(ST) (AXIS_HAS_SPI(ST) && !PIN_EXISTS(ST##_CS))
#if INVALID_TMC_SPI(X)
  #error "DEPENDENCY ERROR: An SPI driven TMC driver on X requires X_CS_PIN."
#elif INVALID_TMC_SPI(X2)
  #error "DEPENDENCY ERROR: An SPI driven TMC driver on X2 requires X2_CS_PIN."
#elif INVALID_TMC_SPI(Y)
  #error "DEPENDENCY ERROR: An SPI driven TMC driver on Y requires Y_CS_PIN."
#elif INVALID_TMC_SPI(Y2)
  #error "DEPENDENCY ERROR: An SPI driven TMC driver on Y2 requires Y2_CS_PIN."
#elif INVALID_TMC_SPI(Z)
  #error "DEPENDENCY ERROR: An SPI driven TMC driver on Z requires Z_CS_PIN."
#elif INVALID_TMC_SPI(Z2)
  #error "DEPENDENCY ERROR: An SPI driven TMC driver on Z2 requires Z2_CS_PIN."
#elif INVALID_TMC_SPI(Z3)
  #error "DEPENDENCY ERROR: An SPI driven TMC driver on Z3 requires Z3_CS_PIN."
#elif INVALID_TMC_SPI(E0)
  #error "DEPENDENCY ERROR: An SPI driven TMC driver on E0 requires E0_CS_PIN."
#elif INVALID_TMC_SPI(E1)
  #error "DEPENDENCY ERROR: An SPI driven TMC driver on E1 requires E1_CS_PIN."
#elif INVALID_TMC_SPI(E2)
  #error "DEPENDENCY ERROR: An SPI driven TMC driver on E2 requires E2_CS_PIN."
#elif INVALID_TMC_SPI(E3)
  #error "DEPENDENCY ERROR: An SPI driven TMC driver on E3 requires E3_CS_PIN."
#elif INVALID_TMC_SPI(E4)
  #error "DEPENDENCY ERROR: An SPI driven TMC driver on E4 requires E4_CS_PIN."
#elif INVALID_TMC_SPI(E5)
  #error "DEPENDENCY ERROR: An SPI driven TMC driver on E5 requires E5_CS_PIN."
#endif
#undef INVALID_TMC_SPI

/**
 * Check existing RX/TX pins against enable TMC UART drivers.
 */
#define INVALID_TMC_UART(ST) (AXIS_HAS_UART(ST) && !(defined(ST##_HARDWARE_SERIAL) || (PIN_EXISTS(ST##_SERIAL_RX) && PIN_EXISTS(ST##_SERIAL_TX))))
#if INVALID_TMC_UART(X)
  #error "DEPENDENCY ERROR: TMC2208 on X requires X_HARDWARE_SERIAL or X_SERIAL_(RX|TX)_PIN."
#elif INVALID_TMC_UART(X2)
  #error "DEPENDENCY ERROR: TMC2208 on X2 requires X2_HARDWARE_SERIAL or X2_SERIAL_(RX|TX)_PIN."
#elif INVALID_TMC_UART(Y)
  #error "DEPENDENCY ERROR: TMC2208 on Y requires Y_HARDWARE_SERIAL or Y_SERIAL_(RX|TX)_PIN."
#elif INVALID_TMC_UART(Y2)
  #error "DEPENDENCY ERROR: TMC2208 on Y2 requires Y2_HARDWARE_SERIAL or Y2_SERIAL_(RX|TX)_PIN."
#elif INVALID_TMC_UART(Z)
  #error "DEPENDENCY ERROR: TMC2208 on Z requires Z_HARDWARE_SERIAL or Z_SERIAL_(RX|TX)_PIN."
#elif INVALID_TMC_UART(Z2)
  #error "DEPENDENCY ERROR: TMC2208 on Z2 requires Z2_HARDWARE_SERIAL or Z2_SERIAL_(RX|TX)_PIN."
#elif INVALID_TMC_UART(Z3)
  #error "DEPENDENCY ERROR: TMC2208 on Z3 requires Z3_HARDWARE_SERIAL or Z3_SERIAL_(RX|TX)_PIN."
#elif INVALID_TMC_UART(E0)
  #error "DEPENDENCY ERROR: TMC2208 on E0 requires E0_HARDWARE_SERIAL or E0_SERIAL_(RX|TX)_PIN."
#elif INVALID_TMC_UART(E1)
  #error "DEPENDENCY ERROR: TMC2208 on E1 requires E1_HARDWARE_SERIAL or E1_SERIAL_(RX|TX)_PIN."
#elif INVALID_TMC_UART(E2)
  #error "DEPENDENCY ERROR: TMC2208 on E2 requires E2_HARDWARE_SERIAL or E2_SERIAL_(RX|TX)_PIN."
#elif INVALID_TMC_UART(E3)
  #error "DEPENDENCY ERROR: TMC2208 on E3 requires E3_HARDWARE_SERIAL or E3_SERIAL_(RX|TX)_PIN."
#elif INVALID_TMC_UART(E4)
  #error "DEPENDENCY ERROR: TMC2208 on E4 requires E4_HARDWARE_SERIAL or E4_SERIAL_(RX|TX)_PIN."
#elif INVALID_TMC_UART(E5)
  #error "DEPENDENCY ERROR: TMC2208 on E5 requires E5_HARDWARE_SERIAL or E5_SERIAL_(RX|TX)_PIN."
#endif
#undef INVALID_TMC_UART

#if ENABLED(HYBRID_THRESHOLD) && !TMC_HAS_STEALTHCHOP
  #error "DEPENDENCY ERROR: Enable STEALTHCHOP on axis to use HYBRID_THRESHOLD."
#endif

// Require STEALTHCHOP for SENSORLESS_HOMING on DELTA as the transition from spreadCycle to stealthChop
// is necessary in order to reset the stallGuard indication between the initial movement of all three
// towers to +Z and the individual homing of each tower. This restriction can be removed once a means of
// clearing the stallGuard activated status is found.
#if ENABLED(SENSORLESS_HOMING) && MECH(DELTA) && (!AXIS_HAS_STALLGUARD(X) || !AXIS_HAS_STALLGUARD(Y) || !AXIS_HAS_STALLGUARD(Z))
  #error "DEPENDENCY ERROR: SENSORLESS_HOMING on DELTA currently requires STEALTHCHOP on all axis."
#endif

// Sensorless homing is required for both combined steppers in an H-bot
#if CORE_IS_XY && X_HAS_SENSORLESS != Y_HAS_SENSORLESS
  #error "DEPENDENCY ERROR: CoreXY requires both X and Y to use sensorless homing if either does."
#elif CORE_IS_XZ && X_HAS_SENSORLESS != Z_HAS_SENSORLESS
  #error "DEPENDENCY ERROR: CoreXZ requires both X and Z to use sensorless homing if either does."
#elif CORE_IS_YZ && Y_HAS_SENSORLESS != Z_HAS_SENSORLESS
  #error "DEPENDENCY ERROR: CoreYZ requires both Y and Z to use sensorless homing if either does."
#endif

// Other TMC feature requirements
#if ENABLED(SENSORLESS_HOMING) && !TMC_HAS_STALLGUARD
  #error "SENSORLESS_HOMING requires TMC2130, TMC2160, TMC2660, or TMC5160 stepper drivers."
#elif ENABLED(SENSORLESS_PROBING) && !TMC_HAS_STALLGUARD
  #error "SENSORLESS_PROBING requires TMC2130, TMC2160, TMC2660, or TMC5160 stepper drivers."
#elif STEALTHCHOP_ENABLED && !TMC_HAS_STEALTHCHOP
  #error "STEALTHCHOP requires TMC2130, TMC2160, TMC2208, or TMC5160 stepper drivers."
#endif

#if ENABLED(TMC_Z_CALIBRATION) && !AXIS_HAS_TMC(Z) && !AXIS_HAS_TMC(Z2)
  #error "DEPENDENCY ERROR: TMC_Z_CALIBRATION requires at least one TMC driver on Z axis"
#endif

#if !HAS_TRINAMIC
  #if ENABLED(MONITOR_DRIVER_STATUS)
    #error "DEPENDENCY ERROR: MONITOR_DRIVER_STATUS requires at least one TMC driver"
  #elif ENABLED(TMC_DEBUG)
    #error "DEPENDENCY ERROR: TMC_DEBUG requires at least one TMC driver"
  #endif
#endif
