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

  SdFile Restart::file;

  restart_job_t Restart::job_info;

  bool Restart::enabled;

  void Restart::init_job() { memset(&job_info, 0, sizeof(job_info)); }

  void Restart::enable(const bool onoff) {
    enabled = onoff;
    changed();
  }

  void Restart::changed() {
    if (!enabled)
      purge_job();
    else if (IS_SD_PRINTING())
      save_job(true);
  }

  void Restart::check() {
    if (enabled) {
      if (!card.isOK()) card.mount();
      if (card.isOK()) {
        load_job();
        if (!valid()) return purge_job();
        commands.enqueue_and_echo_P(PSTR("M800 S"));
      }
    }
  }

  void Restart::purge_job() {
    init_job();
    card.delete_restart_file();
  }

  void Restart::load_job() {
    if (exists()) {
      open(true);
      (void)file.read(&job_info, sizeof(job_info));
      close();
    }
    #if ENABLED(DEBUG_POWER_LOSS_RECOVERY)
      debug(PSTR("Load"));
    #endif
  }

  void Restart::save_job(const bool force_save/*=false*/, const bool save_count/*=true*/) {

    static watch_t save_restart_watch((SD_RESTART_FILE_SAVE_TIME) * 1000UL);

    if (save_restart_watch.elapsed() || force_save ||
        // Save on every new Z height
        (mechanics.current_position[Z_AXIS] > job_info.current_position[Z_AXIS])
    ) {

      if (!++job_info.valid_head) ++job_info.valid_head; // non-zero in sequence
      job_info.valid_foot = job_info.valid_head;

      // Mechanics state
      COPY_ARRAY(job_info.current_position, mechanics.current_position);
      job_info.feedrate = uint16_t(MMS_TO_MMM(mechanics.feedrate_mm_s));

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
        job_info.leveling = bedlevel.flag.leveling_active;
        job_info.z_fade_height = 
          #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
            bedlevel.z_fade_height;
          #else
            0.0;
          #endif
      #endif

      // Commands in the queue
      job_info.buffer_head = commands.buffer_ring.head();
      job_info.buffer_count = save_count ? commands.buffer_ring.count() : 0;
      for (uint8_t index = 0; index < BUFSIZE; index++) {
        gcode_t temp_cmd;
        temp_cmd = commands.buffer_ring.peek(index);
        strncpy(job_info.buffer_ring[index], temp_cmd.gcode, sizeof(job_info.buffer_ring[index]) - 1);
      }

      // Elapsed print job time
      job_info.print_job_counter_elapsed = print_job_counter.duration() * 1000UL;

      // SD file e position
      if (!job_info.just_restart) {
        card.getAbsFilename(job_info.fileName);
        job_info.just_restart = true;
      }
      job_info.sdpos = card.getIndex();

      write_job();
    }
  }

  void Restart::resume_job() {

    char cmd[40], str1[16];

    #if HAS_LEVELING
      // Make sure leveling is off before any G92 and G28
      commands.process_now_P(PSTR("M420 S0 Z0"));
    #endif

    // Auto home
    #if Z_HOME_DIR > 0
      mechanics.home();
    #else
      mechanics.home_flag.ZHomed = true;
      stepper.enable_Z();
      dtostrf(job_info.current_position[Z_AXIS], 1, 3, str1);
      sprintf_P(cmd, PSTR("G92 Z%s"), str1);
      commands.process_now(cmd);
      mechanics.home(true, true, false);
    #endif

    // Select the previously active tool (with no_move)
    #if EXTRUDERS > 1
      tools.change(job_info.active_extruder, 0, true);
    #endif

    // Set temperature
    #if HEATER_COUNT > 0
      LOOP_HEATER() {
        heaters[h].setTarget(job_info.target_temperature[h]);
        thermalManager.wait_heater(&heaters[h], true);
      }
    #endif

    // Set fan
    #if FAN_COUNT > 0
      LOOP_FAN() fans[f].Speed = job_info.fan_speed[f];
    #endif

    // Set leveling
    #if HAS_LEVELING
      #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
        bedlevel.set_z_fade_height(job_info.z_fade_height);
      #endif
      bedlevel.set_bed_leveling_enabled(job_info.leveling);
    #endif

    // Restore E position
    dtostrf(job_info.current_position[E_AXIS], 1, 3, str1);
    sprintf_P(cmd, PSTR("G92 E%s"), str1);
    commands.process_now(cmd);

    #if Z_HOME_DIR > 0
      // Move back to the saved XYZ
      char str2[16], str3[16];
      dtostrf(job_info.current_position[X_AXIS], 1, 3, str1);
      dtostrf(job_info.current_position[Y_AXIS], 1, 3, str2);
      dtostrf(job_info.current_position[Z_AXIS], 1, 3, str3);
      sprintf_P(cmd, PSTR("G1 X%s Y%s Z%s F3000"), str1, str2, str3);
      commands.process_now(cmd);
    #else
      // Move back to the saved XY
      char str2[16];
      dtostrf(job_info.current_position[X_AXIS], 1, 3, str1);
      dtostrf(job_info.current_position[Y_AXIS], 1, 3, str2);
      sprintf_P(cmd, PSTR("G1 X%s Y%s F3000"), str1, str2);
      commands.process_now(cmd);
      // Move back to the saved Z
      dtostrf(job_info.current_position[Z_AXIS], 1, 3, str1);
      sprintf_P(cmd, PSTR("G1 Z%s F200"), str1);
      commands.process_now(cmd);
    #endif

    // Restore the feedrate
    sprintf_P(cmd, PSTR("G1 F%d"), job_info.feedrate);
    commands.process_now(cmd);

    uint8_t h = job_info.buffer_head, c = job_info.buffer_count;
    for (; c--; h = (h + 1) % BUFSIZE)
      commands.process_now(job_info.buffer_ring[h]);

    // Resume the SD file from the last position
    char *fn = job_info.fileName;
    while (*fn == '/') fn++;
    sprintf_P(cmd, PSTR("M23 %s"), fn);
    commands.process_now(cmd);
    sprintf_P(cmd, PSTR("M24 S%ld T%ld"), job_info.sdpos, job_info.print_job_counter_elapsed);
    commands.process_now(cmd);

  }

/** Private Function */
  void Restart::write_job() {

    #if ENABLED(DEBUG_RESTART)
      debug_info(PSTR("Write"));
    #endif

    open(false);
    file.rewind();
    const int16_t ret = file.write(&job_info, sizeof(job_info));
    #if ENABLED(DEBUG_RESTART)
      if (ret == -1) SERIAL_EM("Restart file write failed.");
    #endif
  }

  #if ENABLED(DEBUG_RESTART)

    void Restart::debug_info(PGM_P const prefix) {
      SERIAL_PS(prefix);
      SERIAL_MV("Job Recovery Info...\nvalid Head:", (int)job_info.valid_head);
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
          SERIAL_EMV("buffer_head: ", job_info.buffer_head);
          SERIAL_EMV("buffer_count: ", job_info.buffer_count);
          for (uint8_t i = 0; i < job_info.buffer_count; i++) SERIAL_EMT("> ", job_info.buffer_ring[i]);
          SERIAL_EMT("Filename: ", job_info.fileName);
          SERIAL_EMV("sdpos: ", job_info.sdpos);
          SERIAL_EMV("print_job_counter_elapsed: ", job_info.print_job_counter_elapsed);
        }
        else
          SERIAL_EM("INVALID DATA");
      }
      SERIAL_EM("---");
    }

  #endif

#endif // HAS_SD_RESTART
