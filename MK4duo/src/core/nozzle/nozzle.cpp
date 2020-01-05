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

#include "../../../MK4duo.h"
#include "sanitycheck.h"

Nozzle nozzle;

/** Public Parameters */
nozzle_data_t Nozzle::data;

/** Public Function */
void Nozzle::factory_parameters() {

  #if ENABLED(HOTEND_OFFSET_X) && ENABLED(HOTEND_OFFSET_Y) && ENABLED(HOTEND_OFFSET_Z)
    constexpr float HEoffset_X[] = HOTEND_OFFSET_X,
                    HEoffset_Y[] = HOTEND_OFFSET_Y,
                    HEoffset_Z[] = HOTEND_OFFSET_Z;
  #else
    constexpr float HEoffset_X[MAX_HOTEND] = { 0.0f },
                    HEoffset_Y[MAX_HOTEND] = { 0.0f },
                    HEoffset_Z[MAX_HOTEND] = { 0.0f },
  #endif

  static_assert(
    HEoffset_X[0] == 0 && HEoffset_Y[0] == 0 && HEoffset_Z[0] == 0,
    "Offsets for the first hotend must be 0.0."
  );
  LOOP_HOTEND() {
    data.hotend_offset[h].x = HEoffset_X[h];
    data.hotend_offset[h].y = HEoffset_Y[h];
    data.hotend_offset[h].z = HEoffset_Z[h];
  }

  #if ENABLED(NOZZLE_PARK_FEATURE)
    constexpr xyz_pos_t nozzle_park_point = NOZZLE_PARK_POINT;
    data.park_point = nozzle_park_point;
  #elif MAX_EXTRUDER > 1
    data.park_point.set(0, 0, TOOL_CHANGE_Z_RAISE);
  #endif

}

#if HAS_LCD
  void Nozzle::set_heating_message() {
    const bool heating = hotends[toolManager.target_hotend()]->isHeating();
    lcdui.status_printf_P(0, PSTR("H%i " S_FMT), toolManager.target_hotend(), heating ? GET_TEXT(MSG_HEATING) : GET_TEXT(MSG_COOLING));
  }
#endif

#if ENABLED(NOZZLE_PARK_FEATURE) || MAX_EXTRUDER > 1

  void Nozzle::print_M217() {
    #if ENABLED(NOZZLE_PARK_FEATURE)
      SERIAL_LM(CFG, "Nozzle Park: X<point> Y<point> Z<point>");
      SERIAL_SM(CFG, "  M217");
      SERIAL_MV(" X", LINEAR_UNIT(data.park_point.x));
      SERIAL_MV(" Y", LINEAR_UNIT(data.park_point.y));
      SERIAL_MV(" Z", LINEAR_UNIT(data.park_point.z));
      SERIAL_EOL();
    #else
      SERIAL_LM(CFG, "Z raise: Z<point>:");
      SERIAL_SM(CFG, "  M217");
      SERIAL_MV(" Z", LINEAR_UNIT(data.park_point.z));
      SERIAL_EOL();
    #endif
  }

#endif // ENABLED(NOZZLE_PARK_FEATURE) || MAX_EXTRUDER > 1

#if MAX_HOTEND > 1

  void Nozzle::print_M218() {
    SERIAL_LM(CFG, "Hotend offset (unit): T<Tool> X<offset> Y<offset> Z<offset>:");
    LOOP_HOTEND() {
      SERIAL_SMV(CFG, "  M218 T", (int)h);
      SERIAL_MV(" X", LINEAR_UNIT(data.hotend_offset[h].x), 3);
      SERIAL_MV(" Y", LINEAR_UNIT(data.hotend_offset[h].y), 3);
      SERIAL_MV(" Z", LINEAR_UNIT(data.hotend_offset[h].z), 3);
      SERIAL_EOL();
    }
  }

#endif

#if ENABLED(NOZZLE_CLEAN_FEATURE)

  void Nozzle::clean(const uint8_t &pattern, const uint8_t &strokes, const float &radius, const uint8_t &objects, const uint8_t cleans) {

    xyz_pos_t start = NOZZLE_CLEAN_START_POINT;
    xyz_pos_t end   = NOZZLE_CLEAN_END_POINT;

    if (pattern == 2) {
      if (!(cleans & (_BV(X_AXIS) | _BV(Y_AXIS)))) {
        SERIAL_EM("Warning: Clean Circle requires XY");
        return;
      }
      constexpr xyz_pos_t middle NOZZLE_CLEAN_CIRCLE_MIDDLE;
      end = middle;
    }
    else {
      if (!TEST(cleans, X_AXIS))  start.x = end.x = mechanics.position.x;
      if (!TEST(cleans, X_AXIS))  start.y = end.y = mechanics.position.y;
    }
    if (!TEST(cleans, Z_AXIS))    start.z = end.z = mechanics.position.z;

    #if MECH(DELTA)
      if (mechanics.position.z > mechanics.delta_clip_start_height)
        mechanics.do_blocking_move_to_z(mechanics.delta_clip_start_height);
    #endif

    switch (pattern) {
      case 1:   zigzag(start, end, strokes, objects); break;
      case 2:   circle(start, end, strokes, radius);  break;
      default:  stroke(start, end, strokes);
    }
  }

#endif // ENABLED(NOZZLE_CLEAN_FEATURE)

#if ENABLED(NOZZLE_PARK_FEATURE)

  void Nozzle::park(const uint8_t z_action, const xyz_pos_t &park_p/*=data.park_point*/) {

    const float fr_xy = NOZZLE_PARK_XY_FEEDRATE;
    const float fr_z  = NOZZLE_PARK_Z_FEEDRATE;

    switch (z_action) {
      case 1: // force Z-park height
        mechanics.do_blocking_move_to_z(park_p.z, fr_z);
        break;

      case 2: // Raise by Z-park height
        mechanics.do_blocking_move_to_z(MIN(mechanics.position.z + park_p.z, Z_MAX_BED), fr_z);
        break;

      default: // Raise to Z-park height if lower
        mechanics.do_blocking_move_to_z(MAX(park_p.z, mechanics.position.z), fr_z);
    }

    mechanics.do_blocking_move_to_xy(park_p.x, park_p.y, fr_xy);
  }

#endif // ENABLED(NOZZLE_PARK_FEATURE)

/** Private Function */
#if ENABLED(NOZZLE_CLEAN_FEATURE)

  void Nozzle::stroke(const xyz_pos_t &start, const xyz_pos_t &end, const uint8_t &strokes) {

    #if ENABLED(NOZZLE_CLEAN_GOBACK)
      // Store the current coords
      const xyz_pos_t initial = mechanics.position;
    #endif

    // Move to the starting point
    #if ENABLED(NOZZLE_CLEAN_NO_Z)
      mechanics.do_blocking_move_to_xy(start.x, start.y);
    #else
      mechanics.do_blocking_move_to(start.x, start.y, start.z);
    #endif

    // Start the stroke pattern
    for (uint8_t i = 0; i < (strokes >>1); i++) {
      mechanics.do_blocking_move_to_xy(end.x, end.y);
      mechanics.do_blocking_move_to_xy(start.x, start.y);
    }

    #if ENABLED(NOZZLE_CLEAN_GOBACK)
      // Move the nozzle to the initial point
      mechanics.do_blocking_move_to(initial);
    #endif
  }

  void Nozzle::zigzag(const xyz_pos_t &start, const xyz_pos_t &end, const uint8_t &strokes, const uint8_t &objects) {

    const float diffx = end.x - start.x,
                diffy = end.y - start.y;

    if (!diffx || !diffy) return;

    #if ENABLED(NOZZLE_CLEAN_GOBACK)
      // Store the current coords
      const xyz_pos_t initial = mechanics.position;
    #endif

    #if ENABLED(NOZZLE_CLEAN_NO_Z)
      mechanics.do_blocking_move_to_xy(start.x, start.y);
    #else
      mechanics.do_blocking_move_to(start.x, start.y, start.z);
    #endif

    const uint8_t zigs = objects << 1;
    const bool horiz = ABS(diffx) >= ABS(diffy);    // Do a horizontal wipe?
    const float P = (horiz ? diffx : diffy) / zigs;   // Period of each zig / zag
    const xyz_pos_t *side;

    for (uint8_t j = 0; j < strokes; j++) {
      for (int8_t i = 0; i < zigs; i++) {
        side = (i & 1) ? &end : &start;
        if (horiz)
          mechanics.do_blocking_move_to_xy(start.x + i * P, side->y);
        else
          mechanics.do_blocking_move_to_xy(side->x, start.y + i * P);
      }

      for (int8_t i = zigs; i >= 0; i--) {
        side = (i & 1) ? &end : &start;
        if (horiz)
          mechanics.do_blocking_move_to_xy(start.x + i * P, side->y);
        else
          mechanics.do_blocking_move_to_xy(side->x, start.y + i * P);
      }
    }

    #if ENABLED(NOZZLE_CLEAN_GOBACK)
      // Move the nozzle to the initial point
      mechanics.do_blocking_move_to(initial);
    #endif
  }

  void Nozzle::circle(const xyz_pos_t &start, const xyz_pos_t &middle, const uint8_t &strokes, const float &radius) {

    if (strokes == 0) return;

    #if ENABLED(NOZZLE_CLEAN_GOBACK)
      // Store the current coords
      const xyz_pos_t initial = mechanics.position;
    #endif

    #if ENABLED(NOZZLE_CLEAN_NO_Z)
      mechanics.do_blocking_move_to_xy(start.x, start.y);
    #else
      mechanics.do_blocking_move_to(start.x, start.y, start.z);
    #endif

    for (uint8_t s = 0; s < strokes; s++) {
      for (uint8_t i = 0; i < NOZZLE_CLEAN_CIRCLE_FN; i++) {
        mechanics.do_blocking_move_to_xy(
          middle.x + SIN((RADIANS(360) / NOZZLE_CLEAN_CIRCLE_FN) * i) * radius,
          middle.y + COS((RADIANS(360) / NOZZLE_CLEAN_CIRCLE_FN) * i) * radius
        );
      }
    }

    // Let's be safe
    mechanics.do_blocking_move_to_xy(start.x, start.y);

    #if ENABLED(NOZZLE_CLEAN_GOBACK)
      // Move the nozzle to the initial point
      mechanics.do_blocking_move_to(initial);
    #endif
  }

#endif // ENABLED(NOZZLE_CLEAN_FEATURE)
