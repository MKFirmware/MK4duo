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

#include "../../../MK4duo.h"

#if ENABLED(DEBUG_FEATURE)

void Debug::log_machine_info() {
  DEBUG_SM(DEB, " Machine Type:");
  DEBUG_EM(MACHINE_TYPE);

  DEBUG_SM(DEB, "Probe:");
  #if ENABLED(PROBE_MANUALLY)
    DEBUG_EM("PROBE_MANUALLY");
  #elif ENABLED(Z_PROBE_FIX_MOUNTED)
    DEBUG_EM("Z_PROBE_FIX_MOUNTED");
  #elif ENABLED(BLTOUCH)
    DEBUG_EM("BLTOUCH");
  #elif ENABLED(Z_PROBE_SLED)
    DEBUG_EM("Z_PROBE_SLED");
  #elif ENABLED(Z_PROBE_ALLEN_KEY)
    DEBUG_EM("ALLEN KEY");
  #elif HAS_Z_SERVO_PROBE
    DEBUG_EM("SERVO PROBE");
  #else
    DEBUG_EM("NONE");
  #endif

  #if HAS_BED_PROBE
    DEBUG_SM(DEB, " Probe Offset");
    DEBUG_MV(" X:", probe.data.offset[X_AXIS]);
    DEBUG_MV(" Y:", probe.data.offset[Y_AXIS]);
    DEBUG_MV(" Z:", probe.data.offset[Z_AXIS]);

    if (probe.data.offset[X_AXIS] > 0)
      DEBUG_MSG(" (Right");
    else if (probe.data.offset[X_AXIS] < 0)
      DEBUG_MSG(" (Left");
    else if (probe.data.offset[Y_AXIS] != 0)
      DEBUG_MSG(" (Middle");
    else
      DEBUG_MSG(" (Aligned With");

    if (probe.data.offset[Y_AXIS] > 0)
      DEBUG_MSG("-Back");
    else if (probe.data.offset[Y_AXIS] < 0)
      DEBUG_MSG("-Front");
    else if (probe.data.offset[X_AXIS] != 0)
      DEBUG_MSG("-Center");

    if (probe.data.offset[Z_AXIS] < 0)
      DEBUG_MSG(" & Below");
    else if (probe.data.offset[Z_AXIS] > 0)
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
        const float diff[XYZ] = {
          planner.get_axis_position_mm(X_AXIS) - mechanics.current_position[X_AXIS],
          planner.get_axis_position_mm(Y_AXIS) - mechanics.current_position[Y_AXIS],
          planner.get_axis_position_mm(Z_AXIS) - mechanics.current_position[Z_AXIS]
        };
        DEBUG_MSG("ABL Adjustment X");
        if (diff[X_AXIS] > 0) DEBUG_CHR('+');
        DEBUG_VAL(diff[X_AXIS]);
        DEBUG_MSG(" Y");
        if (diff[Y_AXIS] > 0) DEBUG_CHR('+');
        DEBUG_VAL(diff[Y_AXIS]);
        DEBUG_MSG(" Z");
        if (diff[Z_AXIS] > 0) DEBUG_CHR('+');
        DEBUG_VAL(diff[Z_AXIS]);
      #else
        #if ENABLED(AUTO_BED_LEVELING_UBL)
          DEBUG_MSG("UBL Adjustment Z");
          const float rz = ubl.get_z_correction(mechanics.current_position[X_AXIS], mechanics.current_position[Y_AXIS]);
        #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
          DEBUG_MSG("ABL Adjustment Z");
          const float rz = abl.bilinear_z_offset(mechanics.current_position);
        #endif
        DEBUG_VAL(ftostr43sign(rz, '+'));
        #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
          if (bedlevel.z_fade_height) {
            DEBUG_MV(" (", ftostr43sign(rz * bedlevel.fade_scaling_factor_for_z(mechanics.current_position[Z_AXIS])));
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
      DEBUG_MV("MBL Adjustment Z", ftostr43sign(mbl.get_z(mechanics.current_position[X_AXIS], mechanics.current_position[Y_AXIS]
        #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
          , 1.0
        #endif
      )));
      DEBUG_CHR('+');
      #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
        if (bedlevel.z_fade_height) {
          DEBUG_MV(" (", ftostr43sign(mbl.get_z(mechanics.current_position[X_AXIS], mechanics.current_position[Y_AXIS],
            bedlevel.fade_scaling_factor_for_z(mechanics.current_position[Z_AXIS]))));
          DEBUG_MSG("+)");
        }
      #endif
    }
    else
      DEBUG_MSG(" (disabled)");

    DEBUG_EOL();

  #endif

}

void Debug::print_xyz(PGM_P prefix, PGM_P suffix, const float x, const float y, const float z) {
  Com::printPGM(prefix);
  Com::write('(');
  Com::print(x);
  Com::printPGM(PSTR(", "));
  Com::print(y);
  Com::printPGM(PSTR(", "));
  Com::print(z);
  Com::write(')');

  if (suffix) Com::printPGM(suffix);
  else Com::println();
}

void Debug::print_xyz(PGM_P prefix, PGM_P suffix, const float xyz[]) {
  print_xyz(prefix, suffix, xyz[X_AXIS], xyz[Y_AXIS], xyz[Z_AXIS]);
}

#if HAS_PLANAR
  void Debug::print_xyz(PGM_P prefix, PGM_P suffix, const vector_3 &xyz) {
    print_xyz(prefix, suffix, xyz.x, xyz.y, xyz.z);
  }
#endif

#endif // DEBUG_FEATURE
