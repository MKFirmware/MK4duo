/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2017 Alberto Cotronei @MagoKimbra
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
 * tools.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#ifndef _TOOLS_H_
#define _TOOLS_H_

#if EXTRUDERS > 0

  class Tools {

    public: /** Constructor */

      Tools() {};

    public: /** Public Parameters */

      static uint8_t  active_extruder,
                      previous_extruder,
                      target_extruder,
                      active_driver;

      static bool     volumetric_enabled;

      static int16_t  flow_percentage[EXTRUDERS],       // Extrusion factor for each extruder
                      density_percentage[EXTRUDERS];    // Extrusion density factor for each extruder
      static float    filament_size[EXTRUDERS],         // cross-sectional area of filament (in millimeters), typically around 1.75 or 2.85, 0 disables the volumetric calculations for the tools.
                      volumetric_multiplier[EXTRUDERS]; // reciprocal of cross-sectional area of filament (in square millimeters), stored this way to reduce computational burden in planner

      #if ENABLED(FWRETRACT)
        static bool   autoretract_enabled,
                      retracted[EXTRUDERS],
                      retracted_swap[EXTRUDERS];
        static float  retract_length, retract_length_swap, retract_feedrate_mm_s, retract_zlift,
                      retract_recover_length, retract_recover_length_swap, retract_recover_feedrate_mm_s;
      #endif

      #if HAS_EXT_ENCODER
        static uint8_t  encLastSignal[EXTRUDERS];           // what was the last signal
        static int8_t   encLastDir[EXTRUDERS];
        static int32_t  encStepsSinceLastSignal[EXTRUDERS], // when was the last signal
                        encLastChangeAt[EXTRUDERS],
                        encErrorSteps[EXTRUDERS];
      #endif

  };

  extern Tools tools;

#endif

#endif /* _TOOLS_H_ */
