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
 * advanced_pause.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#ifndef _ADVANCED_PAUSE_H_
#define _ADVANCED_PAUSE_H_

#if ENABLED(ADVANCED_PAUSE_FEATURE)

  extern AdvancedPauseMenuResponse advanced_pause_menu_response;

  extern bool move_away_flag;

  bool pause_print(const float &retract, const float &retract2, const float &z_lift, const float &x_pos, const float &y_pos,
                   const float &unload_length=0, const int16_t new_temp=0, const int8_t max_beep_count=0, const bool show_lcd=false);

  void wait_for_filament_reload(const int8_t max_beep_count=0);

  void resume_print(const float &load_length=0, const float &initial_extrude_length=0, const int8_t max_beep_count=0);

#endif // ENABLED(ADVANCED_PAUSE_FEATURE)
#endif /* _ADVANCED_PAUSE_H_ */
