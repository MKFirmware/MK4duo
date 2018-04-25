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

#include "../../../../MK4duo.h"

#if ENABLED(AUTO_BED_LEVELING_UBL)

  #include "ubl.h"
  #include <math.h>

  unified_bed_leveling ubl;

  uint8_t ubl_cnt = 0;

  void unified_bed_leveling::echo_name() { SERIAL_MSG("Unified Bed Leveling"); }

  void unified_bed_leveling::report_current_mesh() {
    if (!bedlevel.leveling_is_valid()) return;
    SERIAL_LM(ECHO, "  G29 I99");
    for (uint8_t x = 0; x < GRID_MAX_POINTS_X; x++) {
      for (uint8_t y = 0;  y < GRID_MAX_POINTS_Y; y++) {
        if (!isnan(z_values[x][y])) {
          SERIAL_SMV(ECHO, "  M421 I", x);
          SERIAL_MV(" J", y);
          SERIAL_MV(" Z", z_values[x][y], 2);
          SERIAL_EOL();
          printer.safe_delay(75);
        }
      }
    }
  }

  void unified_bed_leveling::report_state() {
    echo_name();
    SERIAL_MSG(" System v" UBL_VERSION " ");
    if (!bedlevel.leveling_active) SERIAL_MSG("in");
    SERIAL_EM("active.");
    printer.safe_delay(50);
  }

  #if ENABLED(UBL_DEVEL_DEBUGGING)

    static void debug_echo_axis(const AxisEnum axis) {
      if (mechanics.current_position[axis] == mechanics.destination[axis])
        SERIAL_MSG("-------------");
      else
        SERIAL_VAL(mechanics.destination[X_AXIS], 6);
    }

    void debug_current_and_destination(const char *title) {

      // if the title message starts with a '!' it is so important, we are going to
      // ignore the status of the bedlevel.g26_debug_flag
      if (*title != '!' && !bedlevel.g26_debug_flag) return;

      const float de = mechanics.destination[E_AXIS] - mechanics.current_position[E_AXIS];

      if (de == 0.0) return; // Printing moves only

      const float dx = mechanics.destination[X_AXIS] - mechanics.current_position[X_AXIS],
                  dy = mechanics.destination[Y_AXIS] - mechanics.current_position[Y_AXIS],
                  xy_dist = HYPOT(dx, dy);

      if (xy_dist == 0.0) return;

      const float fpmm = de / xy_dist;
      SERIAL_MV("   fpmm=", fpmm, 6);

      SERIAL_MV("    current=( ", mechanics.current_position[X_AXIS], 6);
      SERIAL_MV(", ", mechanics.current_position[Y_AXIS], 6);
      SERIAL_MV(", ", mechanics.current_position[Z_AXIS], 6);
      SERIAL_MV(", ", mechanics.current_position[E_AXIS], 6);
      SERIAL_MSG(" )   destination=( ");
      debug_echo_axis(X_AXIS);
      SERIAL_MSG(", ");
      debug_echo_axis(Y_AXIS);
      SERIAL_MSG(", ");
      debug_echo_axis(Z_AXIS);
      SERIAL_MSG(", ");
      debug_echo_axis(E_AXIS);
      SERIAL_MSG(" )   ");
      SERIAL_STR(title);
      SERIAL_EOL();

    }

  #endif // UBL_DEVEL_DEBUGGING

  int8_t unified_bed_leveling::storage_slot;

  float unified_bed_leveling::z_values[GRID_MAX_POINTS_X][GRID_MAX_POINTS_Y];

  // 15 is the maximum nubmer of grid points supported + 1 safety margin for now,
  // until determinism prevails
  constexpr float unified_bed_leveling::_mesh_index_to_xpos[16],
                  unified_bed_leveling::_mesh_index_to_ypos[16];

  #if ENABLED(ULTIPANEL)
    bool unified_bed_leveling::lcd_map_control = false;
  #endif

  volatile int unified_bed_leveling::encoder_diff;

  unified_bed_leveling::unified_bed_leveling() {
    ubl_cnt++;  // Debug counter to ensure we only have one UBL object present in memory.  We can eliminate this (and all references to ubl_cnt) very soon.
    reset();
  }

  void unified_bed_leveling::reset() {
    const bool was_enabled = bedlevel.leveling_active;
    bedlevel.set_bed_leveling_enabled(false);
    storage_slot = -1;
    #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
      bedlevel.set_z_fade_height(10.0);
    #endif
    ZERO(z_values);
    if (was_enabled) mechanics.report_current_position();
  }

  void unified_bed_leveling::invalidate() {
    bedlevel.set_bed_leveling_enabled(false);
    set_all_mesh_points_to_value(NAN);
  }

  void unified_bed_leveling::set_all_mesh_points_to_value(const float value) {
    for (uint8_t x = 0; x < GRID_MAX_POINTS_X; x++) {
      for (uint8_t y = 0; y < GRID_MAX_POINTS_Y; y++) {
        z_values[x][y] = value;
      }
    }
  }

  
  static void serial_echo_xy(const uint8_t sp, const int16_t x, const int16_t y) {
    SERIAL_SP(sp);
    SERIAL_CHR('(');
    if (x < 100) { SERIAL_CHR(' '); if (x < 10) SERIAL_CHR(' '); }
    SERIAL_VAL(x);
    SERIAL_CHR(',');
    if (y < 100) { SERIAL_CHR(' '); if (y < 10) SERIAL_CHR(' '); }
    SERIAL_VAL(y);
    SERIAL_CHR(')');
    printer.safe_delay(5);
  }

  static void serial_echo_column_labels(const uint8_t sp) {
    SERIAL_SP(7);
    for (int8_t i = 0; i < GRID_MAX_POINTS_X; i++) {
      if (i < 10) SERIAL_CHR(' ');
      SERIAL_VAL(i);
      SERIAL_SP(sp);
    }
    printer.safe_delay(10);
  }

  /**
   * Produce one of these mesh maps:
   *   0: Human-readable
   *   1: CSV format for spreadsheet import
   *   2: TODO: Display on Graphical LCD
   *   4: Compact Human-Readable
   */
  void unified_bed_leveling::display_map(const int map_type) {
    printer.setSuspendAutoreport(true);

    constexpr uint8_t eachsp = 1 + 6 + 1,                           // [-3.567]
                      twixt = eachsp * (GRID_MAX_POINTS_X) - 9 * 2; // Leading 4sp, Coordinates 9sp each

    const bool human = !(map_type & 0x3), csv = map_type == 1, lcd = map_type == 2, comp = map_type & 0x4;

    SERIAL_MSG("\nBed Topography Report");
    if (human) {
      SERIAL_MSG(":\n\n");
      serial_echo_xy(4, UBL_MESH_MIN_X, UBL_MESH_MAX_Y);
      serial_echo_xy(twixt, UBL_MESH_MAX_X, UBL_MESH_MAX_Y);
      SERIAL_EOL();
      serial_echo_column_labels(eachsp - 2);
    }
    else {
      SERIAL_MSG(" for ");
      SERIAL_PS(csv ? PSTR("CSV:\n") : PSTR("LCD:\n"));
    }

    const float current_xi = get_cell_index_x(mechanics.current_position[X_AXIS] + (MESH_X_DIST) / 2.0),
                current_yi = get_cell_index_y(mechanics.current_position[Y_AXIS] + (MESH_Y_DIST) / 2.0);

    if (!lcd) SERIAL_EOL();
    for (int8_t j = GRID_MAX_POINTS_Y - 1; j >= 0; j--) {

      // Row Label (J index)
      if (human) {
        if (j < 10) SERIAL_CHR(' ');
        SERIAL_VAL(j);
        SERIAL_MSG(" |");
      }
      
      // Row Values (I indexes)
      for (uint8_t i = 0; i < GRID_MAX_POINTS_X; i++) {

        // Opening Brace or Space
        const bool is_current = i == current_xi && j == current_yi;
        if (human) SERIAL_CHR(is_current ? '[' : ' ');

        // Z Value at current I, J
        const float f = z_values[i][j];
        if (lcd) {
          // TODO: Display on Graphical LCD
        }
        else if (isnan(f))
          SERIAL_PS(human ? PSTR("  .   ") : PSTR("NAN"));
        else if (human || csv) {
          if (human && f >= 0.0) SERIAL_CHR(f > 0 ? '+' : ' '); // Space for positive ('-' for negative)
          SERIAL_VAL(f, 3);                                     // Positive: 5 digits, Negative: 6 digits
        }
        if (csv && i < GRID_MAX_POINTS_X - 1) SERIAL_CHR('\t');

        // Closing Brace or Space
        if (human) SERIAL_CHR(is_current ? ']' : ' ');

        printer.idle();
      }
      if (!lcd) SERIAL_EOL();

      // A blank line between rows (unless compact)
      if (j && human && !comp) SERIAL_EM("   |");
    }

    if (human) {
      serial_echo_column_labels(eachsp - 2);
      SERIAL_EOL();
      serial_echo_xy(4, UBL_MESH_MIN_X, UBL_MESH_MIN_Y);
      serial_echo_xy(twixt, UBL_MESH_MAX_X, UBL_MESH_MIN_Y);
      SERIAL_EOL();
      SERIAL_EOL();
    }

    printer.setSuspendAutoreport(false);
  }

  bool unified_bed_leveling::sanity_check() {
    uint8_t error_flag = 0;

    if (eeprom.calc_num_meshes() < 1) {
      SERIAL_EM("?Mesh too big for EEPROM.");
      error_flag++;
    }

    return !!error_flag;
  }

#endif // AUTO_BED_LEVELING_UBL
