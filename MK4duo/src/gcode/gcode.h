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
#include "config/m92.h"
#include "config/m200.h"
#include "config/m201.h"
#include "config/m203.h"
#include "config/m204.h"
#include "config/m205.h"

// Control Commands
#include "control/m17.h"
#include "control/m18_m84.h"
#include "control/m42.h"
#include "control/m85.h"
#include "control/m106_m107.h"
#include "control/m112.h"
#include "control/m120.h"
#include "control/m121.h"
#include "control/m122.h"

// Debug Commands
#include "debug/m43.h"

// Delta Commands
#include "delta/g33.h"

// Feature Commands
#include "feature/g12.h"
#include "feature/g27.h"
#include "feature/g60.h"
#include "feature/g61.h"
#include "feature/m96_m97.h"
#include "feature/m98_m99.h"
#include "feature/m125.h"
#include "feature/m126_m127_m128_m129.h"
#include "feature/m150.h"

// Geometry Commands
#include "geometry/g92.h"

// Host Commands
#include "host/m110.h"
#include "host/m111.h"
#include "host/m113.h"
#include "host/m114.h"
#include "host/m115.h"
#include "host/m118.h"
#include "host/m119.h"

// LCD Commands
#include "lcd/m0_m1.h"
#include "lcd/m117.h"
#include "lcd/m145.h"

// Mixing Commands
#include "mixing/m163_m164_m165.h"

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
#include "multimode/g17_g18_g19.h"
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
#include "temperature/m104.h"
#include "temperature/m105.h"
#include "temperature/m108.h"
#include "temperature/m109.h"
#include "temperature/m140.h"
#include "temperature/m141.h"
#include "temperature/m142.h"
#include "temperature/m155.h"
#include "temperature/m190.h"
#include "temperature/m191.h"
#include "temperature/m192.h"

// Units Commands
#include "units/g20_g21.h"
#include "units/m82.h"
#include "units/m83.h"
#include "units/m149.h"

// Da spostare ancora
#include "m_code.h"

// Table for G and M code
#include "table_gcode.h"
#include "table_mcode.h"

// T Commands
#include "t_code.h"
