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
 * L64xx.h
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#if HAS_L64XX

#include <L6470.h>
#if L6470_LIBRARY_VERSION < 0x000800
  #error "L6470_LIBRARY_VERSION to 0.8.0 or newer."
#endif

#if HAVE_DRV(L6470)
  #define L64XX_MODEL_LIB L6470
#elif HAVE_DRV(L6474)
  #define L64XX_MODEL_LIB L6474
#elif HAVE_DRV(L6480)
  #define L64XX_MODEL_LIB L6480
#endif

#define dSPIN_STEP_CLOCK      0x58
#define dSPIN_STEP_CLOCK_FWD  dSPIN_STEP_CLOCK
#define dSPIN_STEP_CLOCK_REV  dSPIN_STEP_CLOCK+1

#define KVAL_HOLD_STEP_DOWN   1

union l64xx_flag_t {
  uint8_t all;
  struct {
    bool  spi_active      : 1;
    bool  spi_abort       : 1;
    bool  monitor_paused  : 1;
    bool  bit3            : 1;
    bool  bit4            : 1;
    bool  bit5            : 1;
    bool  bit6            : 1;
    bool  bit7            : 1;
  };
  l64xx_flag_t() { all = 0x00; }
};

class MKL64XX : public L64XX_MODEL_LIB {

  public: /** Constructor */

    MKL64XX(const uint8_t DRIVER_ID, const pin_t ss_pin) :
      L64XX_MODEL_LIB(ss_pin),
      id(DRIVER_ID)
      {}

  public: /** Public Parameters */

    const uint8_t id;

    uint8_t       dir_command,
                  is_otw,
                  otw_counter,
                  is_ot,
                  is_hi_Z,
                  com_counter;

    uint32_t      driver_status;

};

class L64XX_Manager : public L64XXHelper {

  public: /** Constructor */

    L64XX_Manager() {}

  public: /** Public Parameters */

    static l64xx_flag_t flag;

    typedef struct {
      uint8_t   STATUS_AXIS_LAYOUT;               // Copy of L6470_status_layout
      uint8_t   AXIS_OCD_TH_MAX;                  // Size of OCD_TH field
      uint8_t   AXIS_STALL_TH_MAX;                // Size of STALL_TH field
      float     AXIS_OCD_CURRENT_CONSTANT_INV;    // mA per count
      float     AXIS_STALL_CURRENT_CONSTANT_INV;  // mA per count
      uint8_t   L6470_AXIS_CONFIG,                // Address of the CONFIG register
                L6470_AXIS_STATUS;                // Address of the STATUS register
      uint16_t  L6470_ERROR_MASK,                 // STATUS_UVLO | STATUS_TH_WRN | STATUS_TH_SD  | STATUS_OCD | STATUS_STEP_LOSS_A | STATUS_STEP_LOSS_B
                L6474_ERROR_MASK,                 // STATUS_UVLO | STATUS_TH_WRN | STATUS_TH_SD  | STATUS_OCD
                STATUS_AXIS_RAW,                  // Copy of status register contents
                STATUS_AXIS,                      // Copy of status register contents but with all error bits active low
                STATUS_AXIS_OCD,                  // Overcurrent detected bit position
                STATUS_AXIS_SCK_MOD,              // Step clock mode is active bit position
                STATUS_AXIS_STEP_LOSS_A,          // Stall detected on A bridge bit position
                STATUS_AXIS_STEP_LOSS_B,          // Stall detected on B bridge bit position
                STATUS_AXIS_TH_SD,                // Thermal shutdown bit position
                STATUS_AXIS_TH_WRN,               // Thermal warning bit position
                STATUS_AXIS_UVLO,                 // Undervoltage lockout is active bit position
                STATUS_AXIS_WRONG_CMD,            // Last command not valid bit position
                STATUS_AXIS_CMD_ERR,              // Command error bit position
                STATUS_AXIS_NOTPERF_CMD;          // Last command not performed bit position
    } L64XX_shadow_t;

    static L64XX_shadow_t shadow;

  public: /** Public Function */

    static void create_l64();

    static void factory_parameters();

    static uint16_t get_stepper_status(Driver* drv);

    static void monitor_update(Driver* drv);
    static void monitor_driver();

    static void say_axis(Driver* drv, const uint8_t label=true);

    static void spi_init();
    static void transfer(uint8_t L6470_buf[], const uint8_t length);
    static uint8_t transfer_single(uint8_t data, int16_t ss_pin);
    static uint8_t transfer_chain(uint8_t data, int16_t ss_pin, uint8_t chain_position);

  private: /** Private Function */

    static void init_chip(L64XX* st, const int ms, const int oc, const int sc, const int mv, const int slew_rate);

    static void print_driver_err(Driver* drv, const char * const err=nullptr);

};

extern L64XX_Manager l64xxManager;

#endif
