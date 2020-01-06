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
 * fanmanager.h
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#include "fan/fan.h"

// Struct Fans data
struct fans_data_t {
  uint8_t   fans;
  uint16_t  frequency;
};

class FanManager {

  public: /** Constructor */

    FanManager() {}

  public: /** Public Parameters */

    static fans_data_t data;

    #if DISABLED(SOFTWARE_PDM)
      static uint8_t pwm_soft_count;
    #endif

  public: /** Public Function */

    /**
     * Initialize the Fans manager
     */
    static void init();

    /**
     * Fans Manager Spin
     */
    static void spin();

    /**
     * Create Object fan
     */
    static void create_object();

    /**
     * Initialize to the factory parameters
     */
    static void factory_parameters();

    /**
     * Change number fan
     */
    static void change_number_fan(const uint8_t f);

    /**
     * Set Fans pwm output
     */
    static void set_output_pwm();

    /**
     * Print all Fans data
     */
    static void print_parameters();

    /**
     * Report current speed
     */
    static void report_speed();

    /**
     * Print Fan data with M106
     */
    static void print_M106(const uint8_t f);

    /**
     * Check if there are heaters Active
     */
    static bool fans_isActive();

    /**
     * Set target fan from the P parameter
     *
     * Returns TRUE if the target is invalid
     */
    static bool get_target_fan(uint8_t& f);

  private: /** Private Function */

    /**
     * Fans Factory parameters
     */
    static void fans_factory_parameters(const uint8_t f);

};

extern FanManager fanManager;
