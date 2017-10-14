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
 * mcode
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#if HAS_LEVELING

  #define CODE_M420

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
   */
  inline void gcode_M420(void) {

    #if ENABLED(AUTO_BED_LEVELING_UBL)

      // L to load a mesh from the EEPROM
      if (parser.seen('L')) {

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
        ubl.display_map(0);  // Currently only supports one map type
        SERIAL_MV("ubl.mesh_is_valid = ", ubl.mesh_is_valid());
        SERIAL_MV("ubl.storage_slot = ", ubl.storage_slot);
      }

    #endif

    // V to print the matrix or mesh
    if (parser.seen('V')) {
      #if ABL_PLANAR
        bedlevel.matrix.debug(PSTR("Bed Level Correction Matrix:"));
      #else
        if (bedlevel.leveling_is_valid()) {
          #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
            bedlevel.print_bilinear_leveling_grid();
            #if ENABLED(ABL_BILINEAR_SUBDIVISION)
              bedlevel.print_bilinear_leveling_grid_virt();
            #endif
          #elif ENABLED(MESH_BED_LEVELING)
            SERIAL_EM("Mesh Bed Level data:");
            bedlevel.mesh_report();
          #endif
        }
      #endif
    }

    const bool to_enable = parser.boolval('S');
    if (parser.seen('S')) bedlevel.set_bed_leveling_enabled(to_enable);

    #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
      if (parser.seen('Z')) bedlevel.set_z_fade_height(parser.value_linear_units());
    #endif

    const bool new_status = bedlevel.leveling_active;

    if (to_enable && !new_status)
      SERIAL_LM(ER, MSG_ERR_M420_FAILED);

    SERIAL_LMV(ECHO, "Bed Leveling ", new_status ? MSG_ON : MSG_OFF);

    #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
      SERIAL_SM(ECHO, "Fade Height ");
      if (bedlevel.z_fade_height > 0.0)
        SERIAL_EV(bedlevel.z_fade_height);
      else
        SERIAL_EM(MSG_OFF);
    #endif
  }

#endif // HAS_LEVELING
