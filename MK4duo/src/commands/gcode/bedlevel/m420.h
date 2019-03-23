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

/**
 * mcode
 *
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

#if HAS_LEVELING

#define CODE_M420

//#define M420_C_USE_MEAN

/**
 * M420: Enable/Disable Bed Leveling and/or set the Z fade height.
 *
 *    S[bool]   Turns leveling on or off
 *    Z[height] Sets the Z fade height (0 or none to disable)
 *    V[bool]   Verbose - Print the leveling grid
 *
 *  With AUTO_BED_LEVELING_UBL only:
 *
 *    L[index]  Load UBL mesh from index (0 is default)
 *    T[map]    0:Human-readable 1:CSV 2:"LCD" 4:Compact
 *
 * With mesh-based leveling only:
 *
 *    C         Center mesh on the mean of the lowest and highest
 */
inline void gcode_M420(void) {

  const bool  seen_S = parser.seen('S'),
              to_enable = seen_S ? parser.value_bool() : bedlevel.flag.leveling_active;

  // If disabling leveling do it right away
  // (Don't disable for just M420 or M420 V)
  if (seen_S && !to_enable) bedlevel.set_bed_leveling_enabled(false);

  const float oldpos[] = {
    mechanics.current_position[X_AXIS],
    mechanics.current_position[Y_AXIS],
    mechanics.current_position[Z_AXIS]
  };

  #if ENABLED(AUTO_BED_LEVELING_UBL)

    // L to load a mesh from the EEPROM
    if (parser.seen('L')) {

      bedlevel.set_bed_leveling_enabled(false);

      #if ENABLED(EEPROM_SETTINGS)

        const int8_t storage_slot = parser.has_value() ? parser.value_int() : ubl.storage_slot;
        const int16_t a = eeprom.calc_num_meshes();

        if (!a) {
          SERIAL_EM("?EEPROM storage not available.");
          return;
        }

        if (!WITHIN(storage_slot, 0, a - 1)) {
          SERIAL_EM("?Invalid storage slot.");
          SERIAL_EMV("?Use 0 to ", a - 1);
          return;
        }

        eeprom.load_mesh(storage_slot);
        ubl.storage_slot = storage_slot;

      #else

        SERIAL_EM("?EEPROM storage not available.");
        return;

      #endif
    }

    // L or V display the map info
    if (parser.seen('L') || parser.seen('V')) {
      ubl.display_map(parser.byteval('T'));
      SERIAL_MSG("Mesh is ");
      if (!ubl.mesh_is_valid()) SERIAL_MSG("in");
      SERIAL_EMV("valid\nStorage slot: ", ubl.storage_slot);
    }

  #endif // AUTO_BED_LEVELING_UBL

  const bool seenV = parser.seen('V');

  #if HAS_MESH

    if (bedlevel.leveling_is_valid()) {

      // Subtract the given value or the mean from all mesh values
      if (parser.seen('C')) {
        const float cval = parser.value_float();
        #if ENABLED(AUTO_BED_LEVELING_UBL)

          bedlevel.set_bed_leveling_enabled(false);
          ubl.adjust_mesh_to_mean(true, cval);

        #else

          #if ENABLED(M420_C_USE_MEAN)

            // Get the sum and average of all mesh values
            float mesh_sum = 0;
            for (uint8_t x = GRID_MAX_POINTS_X; x--;)
              for (uint8_t y = GRID_MAX_POINTS_Y; y--;)
                mesh_sum += Z_VALUES(x, y);
            const float zmean = mesh_sum / float(GRID_MAX_POINTS);

          #else

            // Find the low and high mesh values
            float lo_val = 100, hi_val = -100;
            for (uint8_t x = GRID_MAX_POINTS_X; x--;)
              for (uint8_t y = GRID_MAX_POINTS_Y; y--;) {
                const float z = Z_VALUES(x, y);
                NOMORE(lo_val, z);
                NOLESS(hi_val, z);
              }
            // Take the mean of the lowest and highest
            const float zmean = (lo_val + hi_val) / 2.0 + cval;

          #endif

          // If not very close to 0, adjust the mesh
          if (!NEAR_ZERO(zmean)) {
            bedlevel.set_bed_leveling_enabled(false);
            // Subtract the mean from all values
            for (uint8_t x = GRID_MAX_POINTS_X; x--;)
              for (uint8_t y = GRID_MAX_POINTS_Y; y--;)
                Z_VALUES(x, y) -= zmean;
            #if ENABLED(ABL_BILINEAR_SUBDIVISION)
              abl.virt_interpolate();
            #endif
          }

        #endif
      }
    }
    else if (to_enable || seenV) {
      SERIAL_LM(ER, "Invalid mesh.");
      goto EXIT_M420;
    }

  #endif // HAS_MESH

  // V to print the matrix or mesh
  if (seenV) {
    #if ABL_PLANAR
      bedlevel.matrix.debug(PSTR("Bed Level Correction Matrix:"));
    #else
      if (bedlevel.leveling_is_valid()) {
        #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
          abl.print_bilinear_leveling_grid();
          #if ENABLED(ABL_BILINEAR_SUBDIVISION)
            abl.print_bilinear_leveling_grid_virt();
          #endif
        #elif ENABLED(MESH_BED_LEVELING)
          SERIAL_EM("Mesh Bed Level data:");
          mbl.report_mesh();
        #endif
      }
    #endif
  }

  #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
    if (parser.seen('Z')) bedlevel.set_z_fade_height(parser.value_linear_units());
  #endif

  // Enable leveling if specified, or if previously active
  bedlevel.set_bed_leveling_enabled(to_enable);

EXIT_M420:

  // Error if leveling failed to enable or reenable
  if (to_enable && !bedlevel.flag.leveling_active)
    SERIAL_LM(ER, MSG_ERR_M420_FAILED);

  SERIAL_STR(ECHO);
  SERIAL_EONOFF("Bed Leveling ", bedlevel.flag.leveling_active);

  #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
    SERIAL_SM(ECHO, "Fade Height ");
    if (bedlevel.z_fade_height > 0.0)
      SERIAL_EV(bedlevel.z_fade_height);
    else
      SERIAL_EM(MSG_OFF);
  #endif

  // Report change in position
  if (memcmp(oldpos, mechanics.current_position, sizeof(oldpos)))
    mechanics.report_current_position();

}

#endif // HAS_LEVELING
