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

#ifndef _POWER_H_
#define _POWER_H_

#if HAS_POWER_SWITCH || HAS_POWER_CONSUMPTION_SENSOR || HAS_POWER_CHECK

  class Power {

    public: /** Constructor */

    Power() {};

    public: /** Public Parameters */

      #if HAS_POWER_CONSUMPTION_SENSOR
        static int16_t  current_raw_powconsumption;
        static float    consumption_meas;   // holds the power consumption as accurately measured
        static uint32_t consumption_hour,   // holds the power consumption per hour as accurately measured
                        startpower;
      #endif

    private: /** Private Parameters */

      static flagbyte_t  flag;

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
      FORCE_INLINE static void setLogic(const bool logic) { flag.bit0 = logic; }
      FORCE_INLINE static bool isLogic() { return flag.bit0; }

      // Flag bit 1 Set power check pullup
      FORCE_INLINE static void setPullup(const bool pullup) { flag.bit1 = pullup; }
      FORCE_INLINE static bool isPullup() { return flag.bit1; }

    private: /** Private Function */

      #if HAS_POWER_SWITCH
        static bool is_power_needed();
      #endif

  };

  extern Power powerManager;

#endif // HAS_POWER_SWITCH || HAS_POWER_CONSUMPTION_SENSOR

#endif /* _POWER_H_ */
