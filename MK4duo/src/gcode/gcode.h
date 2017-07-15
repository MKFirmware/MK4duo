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
 * gcode.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

// Calibrate Commands
#include "calibrate/g28.h"
#include "calibrate/g29_mbl.h"
#include "calibrate/g29_abl.h"
#include "calibrate/g42.h"
#include "calibrate/m48.h"

// Config Commands

// Control Commands
#include "control/m17.h"
#include "control/m42.h"

// Debug Commands
#include "debug/m43.h"

// Delta Commands
#include "delta/g33.h"

// Feature Commands
#include "feature/g12.h"
#include "feature/g27.h"
#include "feature/g60.h"
#include "feature/g61.h"

// Geometry Commands
#include "geometry/g92.h"

// Host Commands

// LCD Commands
#include "lcd/m0_m1.h"

// Mixing Commands

// Motion Commands
#include "motion/g0_g1.h"
#include "motion/g2_g3.h"
#include "motion/g4.h"
#include "motion/g5.h"
#include "motion/g10_g11.h"
#include "motion/g90.h"
#include "motion/g91.h"

// MultiMode Commands (Laser - CNC)
#include "multimode/g7.h"
#include "multimode/g17.h"
#include "multimode/m3_m4.h"
#include "multimode/m5.h"
#include "multimode/m6.h"

// Nextion Commands
#include "nextion/m35.h"

// Power Commands
#include "power/m80.h"
#include "power/m81.h"

// Probe Commands
#include "probe/g30.h"
#include "probe/g31.h"
#include "probe/g38.h"

// SDCard Commands
#include "sdcard/sdcard.h"

// Sensor Commands
#include "sensor/m70.h"

// Stats Commands
#include "stats/m31.h"
#include "stats/m75.h"
#include "stats/m76.h"
#include "stats/m77.h"
#include "stats/m78.h"

// Temperature Commands

// Units Commands
#include "units/g20.h"

// Da spostare ancora
#include "m_code.h"

// Table for G and M code
#include "table_gcode.h"
#include "table_mcode.h"

// T Commands
#include "t_code.h"
