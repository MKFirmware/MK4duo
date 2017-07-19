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
 * tools.cpp
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#include "../../base.h"

#if EXTRUDERS > 0

  Extruder extruder;

  uint8_t Extruder::active    = 0,
          Extruder::previous  = 0,
          Extruder::target    = 0,
          Extruder::driver    = 0;

  #if ENABLED(VOLUMETRIC_DEFAULT_ON)
    bool  Extruder::volumetric_enabled = true;
  #else
    bool  Extruder::volumetric_enabled = false;
  #endif

  int16_t Extruder::flow_percentage[EXTRUDERS]        = ARRAY_BY_EXTRUDERS(100),
          Extruder::density_percentage[EXTRUDERS]     = ARRAY_BY_EXTRUDERS(100);
  float   Extruder::filament_size[EXTRUDERS]          = ARRAY_BY_EXTRUDERS(DEFAULT_NOMINAL_FILAMENT_DIA),
          Extruder::volumetric_multiplier[EXTRUDERS]  = ARRAY_BY_EXTRUDERS(1.0);

  #if ENABLED(FWRETRACT)
    bool  Extruder::autoretract_enabled           = false,
          Extruder::retracted[EXTRUDERS]          = ARRAY_BY_EXTRUDERS(false),
          Extruder::retracted_swap[EXTRUDERS]     = ARRAY_BY_EXTRUDERS(false);
    float Extruder::retract_length                = RETRACT_LENGTH,
          Extruder::retract_length_swap           = RETRACT_LENGTH_SWAP,
          Extruder::retract_feedrate_mm_s         = RETRACT_FEEDRATE,
          Extruder::retract_zlift                 = RETRACT_ZLIFT,
          Extruder::retract_recover_length        = RETRACT_RECOVER_LENGTH,
          Extruder::retract_recover_length_swap   = RETRACT_RECOVER_LENGTH_SWAP,
          Extruder::retract_recover_feedrate_mm_s = RETRACT_RECOVER_FEEDRATE;
  #endif

  #if HAS_EXT_ENCODER
    uint8_t Extruder::encLastSignal[EXTRUDERS]            = ARRAY_BY_EXTRUDERS(0);
    int8_t  Extruder::encLastDir[EXTRUDERS]               = ARRAY_BY_EXTRUDERS(1);
    int32_t Extruder::encStepsSinceLastSignal[EXTRUDERS]  = ARRAY_BY_EXTRUDERS(0),
            Extruder::encLastChangeAt[EXTRUDERS]          = ARRAY_BY_EXTRUDERS(0),
            Extruder::encErrorSteps[EXTRUDERS]            = ARRAY_BY_EXTRUDERS(ENC_ERROR_STEPS);
  #endif

#endif // EXTRUDERS > 0
