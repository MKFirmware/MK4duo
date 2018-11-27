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

/**
 * filrunout.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#ifndef _FILRUNOUT_H_
#define _FILRUNOUT_H_

#if HAS_FIL_RUNOUT_0

  class FilamentRunOut {

    public: /** Constructor */

      FilamentRunOut() {}

    public: /** Public Parameters */

      static flagbyte_t logic_flag,
                        pullup_flag;

    public: /** Public Function */

      /**
       * Initialize the filrunout pins
       */
      static void init();

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

      /**
       * Filrunout check
       */
      static void spin();

      FORCE_INLINE static void setLogic(const FilRunoutEnum filrunout, const bool logic) {
        SET_BIT(logic_flag._byte, filrunout, logic);
      }
      FORCE_INLINE static bool isLogic(const FilRunoutEnum filrunout) { return TEST(logic_flag._byte, filrunout); }

      FORCE_INLINE static void setPullup(const FilRunoutEnum filrunout, const bool pullup) {
        SET_BIT(pullup_flag._byte, filrunout, pullup);
      }
      FORCE_INLINE static bool isPullup(const FilRunoutEnum filrunout) { return TEST(pullup_flag._byte, filrunout); }

    private: /** Private Function */

      static bool read();

  };

  extern FilamentRunOut filamentrunout;

#endif // HAS_FIL_RUNOUT_0

#endif /* _FILRUNOUT_H_ */
