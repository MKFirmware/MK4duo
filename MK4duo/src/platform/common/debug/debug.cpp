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

#include "../../../../MK4duo.h"

#if ENABLED(DEBUG_FEATURE)

void Debug::log_machine_info() {
  DEBUG_SM(DEB, " Machine Type:");
  DEBUG_EM(MACHINE_TYPE);

  DEBUG_SM(DEB, "Probe:");
  #if HAS_PROBE_MANUALLY
    DEBUG_EM("PROBE_MANUALLY");
  #elif HAS_PROBE_FIX
    DEBUG_EM("PROBE_FIX_MOUNTED");
  #elif HAS_BLTOUCH
    DEBUG_EM("BLTOUCH");
  #elif HAS_SLED
    DEBUG_EM("PROBE_SLED");
  #elif HAS_ALLEN_KEY
    DEBUG_EM("ALLEN KEY");
  #elif HAS_Z_SERVO_PROBE
    DEBUG_EM("SERVO PROBE");
  #else
    DEBUG_EM("NONE");
  #endif

  #if HAS_BED_PROBE
    DEBUG_SM(DEB, " Probe Offset");
    DEBUG_MV(" X:", probe.data.offset.x);
    DEBUG_MV(" Y:", probe.data.offset.y);
    DEBUG_MV(" Z:", probe.data.offset.z);

    if (probe.data.offset.x > 0)
      DEBUG_MSG(" (Right");
    else if (probe.data.offset.x < 0)
      DEBUG_MSG(" (Left");
    else if (probe.data.offset.y != 0)
      DEBUG_MSG(" (Middle");
    else
      DEBUG_MSG(" (Aligned With");

    if (probe.data.offset.y > 0)
      DEBUG_MSG("-Back");
    else if (probe.data.offset.y < 0)
      DEBUG_MSG("-Front");
    else if (probe.data.offset.x != 0)
      DEBUG_MSG("-Center");

    if (probe.data.offset.z < 0)
      DEBUG_MSG(" & Below");
    else if (probe.data.offset.z > 0)
      DEBUG_MSG(" & Above");
    else
      DEBUG_MSG(" & Same Z as");
    DEBUG_EM(" Nozzle)");
  #endif

  #if HAS_ABL_OR_UBL
    DEBUG_SM(DEB, " Auto Bed Leveling:");
    #if ENABLED(AUTO_BED_LEVELING_LINEAR)
      DEBUG_MSG("LINEAR");
    #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
      DEBUG_MSG("BILINEAR");
    #elif ENABLED(AUTO_BED_LEVELING_3POINT)
      DEBUG_MSG("3POINT");
    #elif ENABLED(AUTO_BED_LEVELING_UBL)
      DEBUG_MSG("UBL");
    #endif
    if (bedlevel.flag.leveling_active) {
      DEBUG_EM(" (enabled)");
      #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
        if (bedlevel.z_fade_height)
          DEBUG_MV("Z Fade: ", bedlevel.z_fade_height);
      #endif
      #if ABL_PLANAR
        const xyz_pos_t diff = {
          planner.get_axis_position_mm(X_AXIS) - mechanics.position.x,
          planner.get_axis_position_mm(Y_AXIS) - mechanics.position.y,
          planner.get_axis_position_mm(Z_AXIS) - mechanics.position.z
        };
        DEBUG_MSG("ABL Adjustment X");
        if (diff.x > 0) DEBUG_CHR('+');
        DEBUG_VAL(diff.x);
        DEBUG_MSG(" Y");
        if (diff.y > 0) DEBUG_CHR('+');
        DEBUG_VAL(diff.y);
        DEBUG_MSG(" Z");
        if (diff.z > 0) DEBUG_CHR('+');
        DEBUG_VAL(diff.z);
      #else
        #if ENABLED(AUTO_BED_LEVELING_UBL)
          DEBUG_MSG("UBL Adjustment Z");
          const float rz = ubl.get_z_correction(mechanics.position.x, mechanics.position.y);
        #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
          DEBUG_MSG("ABL Adjustment Z");
          const float rz = abl.bilinear_z_offset(mechanics.position);
        #endif
        DEBUG_VAL(ftostr43sign(rz, '+'));
        #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
          if (bedlevel.z_fade_height) {
            DEBUG_MV(" (", ftostr43sign(rz * bedlevel.fade_scaling_factor_for_z(mechanics.position.z)));
            DEBUG_MSG("+)");
          }
        #endif
      #endif
    }
    else
      DEBUG_MSG(" (disabled)");

    DEBUG_EOL();

  #elif ENABLED(MESH_BED_LEVELING)

    DEBUG_SM(DEB, " Mesh Bed Leveling");
    if (bedlevel.flag.leveling_active) {
      DEBUG_EM(" (enabled)");
      DEBUG_MV("MBL Adjustment Z", ftostr43sign(mbl.get_z(mechanics.position
        #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
          , 1.0
        #endif
      )));
      DEBUG_CHR('+');
      #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
        if (bedlevel.z_fade_height) {
          DEBUG_MV(" (", ftostr43sign(mbl.get_z(mechanics.position,
            bedlevel.fade_scaling_factor_for_z(mechanics.position.z))));
          DEBUG_MSG("+)");
        }
      #endif
    }
    else
      DEBUG_MSG(" (disabled)");

    DEBUG_EOL();

  #endif

}

void Debug::print_xyz(const float &x, const float &y, const float &z, PGM_P const prefix/*=nullptr*/, PGM_P const suffix/*=nullptr*/) {
  SERIAL_STR(prefix);
  SERIAL_CHR('(');
  SERIAL_VAL(x);
  SERIAL_MSG(", ");
  SERIAL_VAL(y);
  SERIAL_MSG(", ");
  SERIAL_VAL(z);
  SERIAL_CHR(')');

  if (suffix) SERIAL_STR(suffix);
  else SERIAL_EOL();
}

#endif // DEBUG_FEATURE
