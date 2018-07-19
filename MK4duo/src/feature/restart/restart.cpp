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

#include "../../../MK4duo.h"

#if HAS_SD_RESTART

  Restart restart;

  restart_job_t Restart::job_info;
  restart_phase Restart::job_phase = RESTART_IDLE;

  char    Restart::buffer_ring[BUFSIZE + APPEND_CMD_COUNT][MAX_CMD_SIZE];
  uint8_t Restart::count  = 0;

  void Restart::do_print_job() {

    memset(&job_info, 0, sizeof(job_info));
    ZERO(buffer_ring);

    if (!card.isOK()) card.mount();

    if (card.isOK()) {

      #if ENABLED(DEBUG_RESTART)
        SERIAL_EMV("Init restart infomation. Size: ", (int)sizeof(job_info));
      #endif

      if (card.exist_restart_file()) {
        card.open_restart_file(true);
        card.read_restart_data();
        card.close_restart_file();

        if ((job_info.valid_head) && job_info.valid_head == job_info.valid_foot) {

          uint8_t index = 0;
          char str_X[10], str_Y[10], str_Z[10], str_E[10];

          ZERO(str_X);
          ZERO(str_Y);
          ZERO(str_Z);
          ZERO(str_E);

          dtostrf(job_info.current_position[X_AXIS], 1, 3, str_X);
          dtostrf(job_info.current_position[Y_AXIS], 1, 3, str_Y);
          dtostrf(job_info.current_position[Z_AXIS], 1, 3, str_Z);
          dtostrf(job_info.current_position[E_AXIS], 1, 3, str_E);

          #if Z_HOME_DIR > 0
            sprintf_P(buffer_ring[index++], PSTR("G92 E%s"), str_E);
            sprintf_P(buffer_ring[index++], PSTR("G0 X%s Y%s Z%s"), str_X, str_Y, str_Z);
            strcpy(buffer_ring[index++], PSTR("M117 Printing..."));
          #else
            sprintf_P(buffer_ring[index++], PSTR("G92 Z%s E%s"), str_Z, str_E);
            sprintf_P(buffer_ring[index++], PSTR("G0 X%s Y%s Z%s"), str_X, str_Y, str_Z);
            strcpy(buffer_ring[index++], PSTR("M117 Printing..."));
          #endif

          uint8_t read = job_info.buffer_index_r, c = job_info.buffer_lenght;
          while (c--) {
            strcpy(buffer_ring[index++], job_info.buffer_ring[read]);
            read = (read + 1) % BUFSIZE;
          }

          count = index;

          #if ENABLED(DEBUG_RESTART)
            debug_info(true);
          #endif

          card.selectFile(job_info.fileName);
          card.setIndex(job_info.sdpos);

          // Auto Restart
          if (job_info.auto_restart) start_job();

        }
        else {
          if ((job_info.valid_head != 0) && (job_info.valid_head != job_info.valid_foot))
            LCD_MESSAGEPGM("RECOVERY INVALID DATA.");
          memset(&job_info, 0, sizeof(job_info));
        }
      }
    }
  }

  void Restart::start_job() {

    // Auto home
    #if Z_HOME_DIR > 0
      mechanics.home();
    #else
      mechanics.home(true, true, false);
      printer.setZHomed(true);
      stepper.enable_Z();
    #endif

    #if EXTRUDERS > 1
      tools.change(job_info.active_extruder, 0, true);
    #endif

    // Set leveling
    #if HAS_LEVELING
      #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
        bedlevel.set_z_fade_height(job_info.z_fade_height);
      #endif
      bedlevel.set_bed_leveling_enabled(job_info.leveling);
    #endif

    // Set temperature
    #if HEATER_COUNT > 0
      LOOP_HEATER() {
        heaters[h].target_temperature = job_info.target_temperature[h];
        thermalManager.wait_heater(&heaters[h], true);
      }
    #endif

    // Set fan
    #if FAN_COUNT > 0
      LOOP_FAN() fans[f].Speed = job_info.fan_speed[f];
    #endif

    job_phase = RESTART_YES;

    job_info.auto_restart = false;

    print_job_counter.resume(job_info.print_job_counter_elapsed);
    card.startFileprint();

  }

  void Restart::save_data(const bool force_save/*=false*/) {

    static watch_t save_restart_watch((SD_RESTART_FILE_SAVE_TIME) * 1000UL);

    if (save_restart_watch.elapsed() || force_save ||
        // Save on every new Z height
        (mechanics.current_position[Z_AXIS] > 0 && mechanics.current_position[Z_AXIS] > job_info.current_position[Z_AXIS])
    ) {

      if (!++job_info.valid_head) ++job_info.valid_head; // non-zero in sequence
      job_info.valid_foot = job_info.valid_head;

      // Mechanics state
      COPY_ARRAY(job_info.current_position, mechanics.current_position);

      #if HEATER_COUNT > 0
        LOOP_HEATER()
          job_info.target_temperature[h] = heaters[h].target_temperature;
      #endif

      #if FAN_COUNT > 0
        LOOP_FAN()
          job_info.fan_speed[f] = fans[f].Speed;
      #endif

      // Extruders
      #if EXTRUDERS > 1
        job_info.active_extruder = tools.active_extruder;
      #endif

      // Leveling      
      #if HAS_LEVELING
        job_info.leveling = bedlevel.leveling_active;
        job_info.z_fade_height = 
          #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
            bedlevel.z_fade_height;
          #else
            0.0;
          #endif
      #endif

      // Commands in the queue
      job_info.buffer_index_r = commands.buffer_index_r;
      job_info.buffer_lenght = commands.buffer_lenght;
      COPY_ARRAY(job_info.buffer_ring, commands.buffer_ring);

      // Elapsed print job time
      job_info.print_job_counter_elapsed = print_job_counter.duration() * 1000UL;

      // SD file e position
      if (!job_info.just_restart) {
        card.getAbsFilename(job_info.fileName);
        job_info.just_restart = true;
      }
      job_info.sdpos = card.getIndex();

      job_info.auto_restart = !force_save;

      #if ENABLED(DEBUG_RESTART)
        SERIAL_EM("Saving job_info");
        debug_info(false);
      #endif

      card.open_restart_file(false);
      (void)card.save_restart_data();
      save_restart_watch.start();
    }
  }

  #if ENABLED(DEBUG_RESTART)

    void Restart::debug_info(const bool restart) {

      SERIAL_MV("Valid Head:", (int)job_info.valid_head);
      SERIAL_EMV(" Valid Foot:", (int)job_info.valid_foot);
      if (job_info.valid_head) {
        if (job_info.valid_head == job_info.valid_foot) {
          SERIAL_MSG("current_position");
          LOOP_XYZE(i) SERIAL_MV(": ", job_info.current_position[i]);
          SERIAL_EOL();
          SERIAL_MSG("target_temperature");
          LOOP_HEATER() SERIAL_MV(": ", job_info.target_temperature[h]);
          SERIAL_EOL();
          SERIAL_MSG("fanSpeeds");
          LOOP_FAN() SERIAL_MV(": ", job_info.fan_speed[f]);
          SERIAL_EOL();
          #if HAS_LEVELING
            SERIAL_EMV("leveling: ", int(job_info.leveling));
            SERIAL_EMV(" z_fade_height: ", int(job_info.z_fade_height));
          #endif
          SERIAL_EMV("buffer_index_r: ", job_info.buffer_index_r);
          SERIAL_EMV("buffer_lenght: ", job_info.buffer_lenght);
          if (restart)
            for (uint8_t i = 0; i < count; i++) SERIAL_EMV("> ", buffer_ring[i]);
          else
            for (uint8_t i = 0; i < job_info.buffer_lenght; i++) SERIAL_EMV("> ", job_info.buffer_ring[i]);
          SERIAL_EMT("Filename: ", job_info.fileName);
          SERIAL_EMV("sdpos: ", job_info.sdpos);
          SERIAL_EMV("print_job_counter_elapsed: ", job_info.print_job_counter_elapsed);
          SERIAL_EMV("auto_restart: ", int(job_info.auto_restart));
        }
        else
          SERIAL_EM("INVALID DATA");
      }
    }

  #endif

#endif // HAS_SD_RESTART
