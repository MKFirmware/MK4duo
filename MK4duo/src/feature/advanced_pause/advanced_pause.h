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
#pragma once

/**
 * advanced_pause.h
 *
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

#if ENABLED(ADVANCED_PAUSE_FEATURE)

typedef struct {
  float unload_length,
        load_length;
} advanced_pause_data_t;

#if ENABLED(DUAL_X_CARRIAGE)
  #define DXC_PARAMS , const int8_t DXC_ext=-1
  #define DXC_ARGS   , const int8_t DXC_ext
  #define DXC_PASS   , DXC_ext
#else
  #define DXC_PARAMS
  #define DXC_ARGS
  #define DXC_PASS
#endif

class AdvancedPause {

  public: /** Constructor */

    AdvancedPause() {}

  public: /** Public Parameters */

    static AdvancedPauseMenuResponseEnum menu_response;

    static advanced_pause_data_t data[EXTRUDERS];

    static uint8_t did_pause_print;

  public: /** Public Function */

    static void do_pause_e_move(const float &length, const float &fr);

    static bool pause_print(const float &retract, const point_t &park_point, const float &unload_length=0,
                            const bool show_lcd=false DXC_PARAMS);

    static void wait_for_confirmation(const bool is_reload=false, const int8_t max_beep_count=0 DXC_PARAMS);

    static void resume_print( const float &slow_load_length=0, const float &fast_load_length=0,
                              const float &purge_length=PAUSE_PARK_EXTRUDE_LENGTH, const int8_t max_beep_count=0 DXC_PARAMS);

    static bool load_filament(const float &slow_load_length=0, const float &fast_load_length=0,
                              const float &purge_length=0, const int8_t max_beep_count=0, const bool show_lcd=false,
                              const bool pause_for_user=false, const AdvancedPauseModeEnum mode=ADVANCED_PAUSE_MODE_PAUSE_PRINT DXC_PARAMS);

    static bool unload_filament(const float &unload_length, const bool show_lcd=false, const AdvancedPauseModeEnum mode=ADVANCED_PAUSE_MODE_PAUSE_PRINT);

  private: /** Private Function */

    static void show_continue_prompt(const bool is_reload);
    static bool ensure_safe_temperature(const AdvancedPauseModeEnum mode=ADVANCED_PAUSE_MODE_SAME);

    #if HAS_BUZZER
      static void filament_change_beep(const int8_t max_beep_count, const bool init=false);
    #endif

};

extern AdvancedPause advancedpause;

#endif // ENABLED(ADVANCED_PAUSE_FEATURE)
