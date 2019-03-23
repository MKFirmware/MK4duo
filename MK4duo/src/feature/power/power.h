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

union flagpower_t {
  bool all;
  struct {
    bool  Logic   : 1;
    bool  Pullup  : 1;
    bool  bit2    : 1;
    bool  bit3    : 1;
    bool  bit4    : 1;
    bool  bit5    : 1;
    bool  bit6    : 1;
    bool  bit7    : 1;
  };
  flagpower_t() { all = false; }
};

#if HAS_POWER_SWITCH || HAS_POWER_CONSUMPTION_SENSOR || HAS_POWER_CHECK

  class Power {

    public: /** Constructor */

    Power() {};

    public: /** Public Parameters */

      static flagpower_t flag;

      #if HAS_POWER_CONSUMPTION_SENSOR
        static int16_t  current_raw_powconsumption;
        static float    consumption_meas;   // holds the power consumption as accurately measured
        static uint32_t startpower;
      #endif

    private: /** Private Parameters */

      #if HAS_POWER_SWITCH
        static bool powersupply_on;
        #if (POWER_TIMEOUT > 0)
          static watch_t watch_lastPowerOn;
        #endif
      #endif

    public: /** Public Function */

      #if HAS_POWER_SWITCH || HAS_POWER_CHECK

        /**
         * Initialize the Power switch and Power Check pins
         */
        static void init();

      #endif

      #if HAS_POWER_CHECK

        /**
         * Initialize Factory parameters
         */
        static void factory_parameters();

        /**
         * Setup Pullup
         */
        static void setup_pullup();

        /**
         * Print logical and pullup
         */
        static void report();

      #endif

      #if HAS_POWER_SWITCH

        static void spin();
        static void power_on();
        static void power_off();

        FORCE_INLINE static bool is_on() { return powersupply_on; }

      #endif

      #if HAS_POWER_CONSUMPTION_SENSOR
        static float  analog2voltage(),
                      analog2current(),
                      analog2power(),
                      raw_analog2voltage(),
                      analog2error(float current),
                      analog2efficiency(float watt);
      #endif

      // Flag bit 0 Set power check logic
      FORCE_INLINE static void setLogic(const bool logic) { flag.Logic = logic; }
      FORCE_INLINE static bool isLogic() { return flag.Logic; }

      // Flag bit 1 Set power check pullup
      FORCE_INLINE static void setPullup(const bool pullup) { flag.Pullup = pullup; }
      FORCE_INLINE static bool isPullup() { return flag.Pullup; }

    private: /** Private Function */

      #if HAS_POWER_SWITCH
        static bool is_power_needed();
      #endif

  };

  extern Power powerManager;

#endif // HAS_POWER_SWITCH || HAS_POWER_CONSUMPTION_SENSOR
