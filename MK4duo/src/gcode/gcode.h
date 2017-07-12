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

#include "motion/g1.h"
#include "motion/g2.h"
#include "motion/g4.h"
#include "motion/g5.h"
#include "laser/g7.h"
#include "motion/g10.h"
#include "feature/g12.h"
#include "cnc/g17.h"
#include "units/g20.h"
#include "feature/g27.h"
#include "calibrate/g28.h"
#include "calibrate/g29_mbl.h"
#include "calibrate/g29_abl.h"
#include "probe/g30.h"
#include "probe/g31.h"
#include "delta/g33.h"
#include "probe/g38.h"
#include "calibrate/g42.h"
#include "feature/g60.h"
#include "feature/g61.h"
#include "motion/g90.h"
#include "motion/g91.h"
#include "geometry/g92.h"

#include "m_code.h"
#include "t_code.h"
#include "table_gcode.h"
#include "table_mcode.h"
