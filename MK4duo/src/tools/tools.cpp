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
 * tools.cpp
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#include "../../base.h"

#if EXTRUDERS > 0

  Tools tools;

  uint8_t Tools::active_extruder    = 0,
          Tools::previous_extruder  = 0,
          Tools::target_extruder    = 0,
          Tools::active_driver      = 0;

  #if ENABLED(VOLUMETRIC_DEFAULT_ON)
    bool  Tools::volumetric_enabled = true;
  #else
    bool  Tools::volumetric_enabled = false;
  #endif

  int16_t Tools::flow_percentage[EXTRUDERS]       = ARRAY_BY_EXTRUDERS(100),
          Tools::density_percentage[EXTRUDERS]    = ARRAY_BY_EXTRUDERS(100);
  float   Tools::filament_size[EXTRUDERS]         = ARRAY_BY_EXTRUDERS(DEFAULT_NOMINAL_FILAMENT_DIA),
          Tools::volumetric_multiplier[EXTRUDERS] = ARRAY_BY_EXTRUDERS(1.0);

  #if ENABLED(FWRETRACT)
    bool  Tools::autoretract_enabled            = false,
          Tools::retracted[EXTRUDERS]           = ARRAY_BY_EXTRUDERS(false),
          Tools::retracted_swap[EXTRUDERS]      = ARRAY_BY_EXTRUDERS(false);
    float Tools::retract_length                 = RETRACT_LENGTH,
          Tools::retract_length_swap            = RETRACT_LENGTH_SWAP,
          Tools::retract_feedrate_mm_s          = RETRACT_FEEDRATE,
          Tools::retract_zlift                  = RETRACT_ZLIFT,
          Tools::retract_recover_length         = RETRACT_RECOVER_LENGTH,
          Tools::retract_recover_length_swap    = RETRACT_RECOVER_LENGTH_SWAP,
          Tools::retract_recover_feedrate_mm_s  = RETRACT_RECOVER_FEEDRATE;
  #endif

  float   Tools::retract_acceleration[EXTRUDERS]  = ARRAY_BY_EXTRUDERS(0.0);

  #if HAS_EXT_ENCODER
    uint8_t Tools::encLastSignal[EXTRUDERS]           = ARRAY_BY_EXTRUDERS(0);
    int8_t  Tools::encLastDir[EXTRUDERS]              = ARRAY_BY_EXTRUDERS(1);
    int32_t Tools::encStepsSinceLastSignal[EXTRUDERS] = ARRAY_BY_EXTRUDERS(0),
            Tools::encLastChangeAt[EXTRUDERS]         = ARRAY_BY_EXTRUDERS(0),
            Tools::encErrorSteps[EXTRUDERS]           = ARRAY_BY_EXTRUDERS(ENC_ERROR_STEPS);
  #endif

#endif // EXTRUDERS > 0
