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

#if HAS_SD_RESTART

Restart restart;

/** Public Parameters */
SdFile Restart::job_file;

restart_job_t Restart::job_info;

bool  Restart::enabled;

uint32_t  Restart::cmd_sdpos      = 0,
          Restart::sdpos[BUFSIZE] = { 0 };  

/** Public Function */
void Restart::enable(const bool onoff) {
  enabled = onoff;
  changed();
}

void Restart::changed() {
  if (!enabled)
    purge_job();
  else if (IS_SD_PRINTING())
    save_job();
}

void Restart::check() {
  if (enabled) {
    card.mount();
    if (card.isMounted()) {
      load_job();
      if (!valid()) return purge_job();
      commands.inject_P(PSTR("M800 S"));
    }
  }
}

void Restart::start_job() {
  card.getAbsFilename(job_info.fileName);
  cmd_sdpos = 0;
  ZERO(sdpos);
}

void Restart::purge_job() {
  clear_job();
  card.delete_restart_file();
}

void Restart::load_job() {
  if (exists()) {
    open(true);
    (void)job_file.read(&job_info, sizeof(job_info));
    close();
  }
  debug_info(PSTR("Load"));
}

void Restart::save_job(const bool force_save/*=true*/) {

  static short_timer_t save_restart_timer(millis());

  // Did Z change since the last call?
  if (save_restart_timer.expired((SD_RESTART_FILE_SAVE_TIME) * 1000) || force_save
      || mechanics.position.z > job_info.axis_position_mm.z
  ) {

    // Set Head and Foot to matching non-zero values
    if (!++job_info.valid_head) ++job_info.valid_head; // non-zero in sequence
    job_info.valid_foot = job_info.valid_head;

    // Mechanics state
    LOOP_XYZE(axis) job_info.axis_position_mm[axis] = planner.get_axis_position_mm(AxisEnum(axis));
    #if ENABLED(WORKSPACE_OFFSETS)
      job_info.home_offset = mechanics.data.home_offset;
      job_info.position_shift = mechanics.position_shift;
    #endif
    job_info.feedrate = uint16_t(MMS_TO_MMM(mechanics.feedrate_mm_s));

    // Heater
    #if HAS_HOTENDS
      LOOP_HOTEND()
        if (hotends[h]) job_info.target_temperature[h] = hotends[h]->deg_target();
    #endif
    #if HAS_BEDS
      LOOP_BED()
        if (beds[h]) job_info.bed_target_temperature[h] = beds[h]->deg_target();
    #endif
    #if HAS_CHAMBERS
      LOOP_CHAMBER()
        if (chambers[h]) job_info.chamber_target_temperature[h] = chambers[h]->deg_target();
    #endif
    #if HAS_FAN
      LOOP_FAN()
        if (fans[f]) job_info.fan_speed[f] = fans[f]->speed;
    #endif

    // Extruders
    #if MAX_EXTRUDER > 1
      job_info.active_extruder = toolManager.extruder.active;
    #endif

    LOOP_EXTRUDER() {
      job_info.flow_percentage[e]     = extruders[e]->flow_percentage;
      job_info.density_percentage[e]  = extruders[e]->density_percentage;
    }

    // Leveling      
    #if HAS_LEVELING
      job_info.leveling = bedlevel.flag.leveling_active;
      #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
        job_info.z_fade_height = bedlevel.z_fade_height;
      #else
        job_info.z_fade_height = 0.0;
      #endif
    #endif

    #if ENABLED(COLOR_MIXING_EXTRUDER) && HAS_GRADIENT_MIX
      memcpy(&job_info.gradient, &mixer.gradient, sizeof(job_info.gradient));
    #endif

    // Relative axis modes
    job_info.axis_relative_modes = mechanics.axis_relative_modes;

    // Elapsed print job time
    job_info.print_job_counter_elapsed = print_job_counter.duration();

    write_job();
  }
}

void Restart::resume_job() {

  #define RESTART_ZRAISE 2

  char cmd[MAX_CMD_SIZE + 16], str1[16], str2[16];

  // Save job_info.sdpos because stepper ISR overwrites it
  const uint32_t save_sdpos = job_info.sdpos;

  #if HAS_LCD
    lcdui.status_printf_P(0, GET_TEXT(MSG_RESUMING));
  #endif

  #if HAS_LEVELING
    // Make sure leveling is off before any G92 and G28
    commands.process_now_P(PSTR("M420 S0 Z0"));
  #endif

  // Reset E, raise Z, home XY...
  commands.process_now_P(PSTR("G92.9 E0"));
  #if Z_HOME_DIR > 0
    commands.process_now_P(G28_CMD);
  #else
    commands.process_now_P(PSTR("G92.9 Z0"));
    mechanics.home_flag.ZHomed = true;
    stepper.enable_Z();
    commands.process_now_P(PSTR("G1 Z" STRINGIFY(RESTART_ZRAISE)));
    commands.process_now_P(PSTR("G28 XY"));
  #endif

  // Select the previously active tool (with no_move)
  #if EXTRUDERS > 1
    sprintf_P(cmd, PSTR("T%i S"), job_info.active_extruder);
    commands.process_now(cmd);
  #endif

  // Set temperature
  #if HAS_CHAMBERS
    LOOP_CHAMBER() {
      if (chambers[h]) {
        chambers[h]->set_target_temp(job_info.chamber_target_temperature[h]);
        chambers[h]->wait_for_target(true);
      }
    }
  #endif
  #if HAS_BEDS
    LOOP_BED() {
      if (beds[h]) {
        beds[h]->set_target_temp(job_info.bed_target_temperature[h]);
        beds[h]->wait_for_target(true);
      }
    }
  #endif
  #if HAS_HOTENDS
    LOOP_HOTEND() {
      if (hotends[h]) {
        hotends[h]->set_target_temp(job_info.target_temperature[h]);
        hotends[h]->wait_for_target(true);
      }
    }
  #endif

  // Set fan
  #if HAS_FAN
    LOOP_FAN() {
      if (fans[f]) fans[f]->speed = job_info.fan_speed[f];
    }
  #endif

  LOOP_EXTRUDER() {
    extruders[e]->flow_percentage     = job_info.flow_percentage[e];
    extruders[e]->density_percentage  = job_info.density_percentage[e];
  }

  // Set leveling
  #if HAS_LEVELING
    if (job_info.z_fade_height || job_info.leveling) {
      sprintf_P(cmd, PSTR("M420 S%i Z%s"), int(job_info.leveling), dtostrf(job_info.z_fade_height, 1, 1, str1));
      commands.process_now(cmd);
    }
  #endif

  #if ENABLED(COLOR_MIXING_EXTRUDER) && HAS_GRADIENT_MIX
    memcpy(&mixer.gradient, &job_info.gradient, sizeof(job_info.gradient));
  #endif

  // Extrude and retract to clean the nozzle
  #if SD_RESTART_FILE_PURGE_LEN > 0
    commands.process_now_P(PSTR("G1 E" STRINGIFY(SD_RESTART_FILE_PURGE_LEN) " F200"));
  #endif
  #if SD_RESTART_FILE_RETRACT_LEN > 0
    sprintf_P(cmd, PSTR("G1 E%d F3000"), SD_RESTART_FILE_PURGE_LEN - (SD_RESTART_FILE_RETRACT_LEN));
    commands.process_now(cmd);
  #endif

  // For DELTA must inversetrasform coordinate
  #if MECH(DELTA)
    mechanics.InverseTransform(
      job_info.axis_position_mm.x,
      job_info.axis_position_mm.y,
      job_info.axis_position_mm.z,
      job_info.axis_position_mm
    );
  #endif

  #if Z_HOME_DIR > 0
    // Move back to the saved XYZ
    char str3[16];
    sprintf_P(cmd, PSTR("G1 X%s Y%s Z%s F3000"),
      dtostrf(job_info.axis_position_mm.x, 1, 3, str1),
      dtostrf(job_info.axis_position_mm.y, 1, 3, str2),
      dtostrf(job_info.axis_position_mm.z, 1, 3, str3)
    );
    commands.process_now(cmd);
  #else
    // Move back to the saved XY
    sprintf_P(cmd, PSTR("G1 X%s Y%s F3000"),
      dtostrf(job_info.axis_position_mm.x, 1, 3, str1),
      dtostrf(job_info.axis_position_mm.y, 1, 3, str2)
    );
    commands.process_now(cmd);
    // Move back to the saved Z
    dtostrf(job_info.axis_position_mm.z, 1, 3, str1);
    commands.process_now_P(PSTR("G1 Z0 F200"));
    sprintf_P(cmd, PSTR("G92.9 Z%s"), str1);
    commands.process_now(cmd);
  #endif

  // Un-retract
  #if SD_RESTART_FILE_PURGE_LEN > 0
    commands.process_now_P(PSTR("G1 E" STRINGIFY(SD_RESTART_FILE_PURGE_LEN) " F3000"));
  #endif

  // Restore the feedrate
  sprintf_P(cmd, PSTR("G1 F%d"), job_info.feedrate);
  commands.process_now(cmd);

  // Restore E position
  sprintf_P(cmd, PSTR("G92.9 E%s"), dtostrf(job_info.axis_position_mm.e, 1, 3, str1));
  commands.process_now(cmd);

  // Relative mode
  mechanics.axis_relative_modes = job_info.axis_relative_modes;

  #if ENABLED(WORKSPACE_OFFSETS)
    LOOP_XYZ(i) {
      mechanics.data.home_offset[i] = job_info.home_offset[i];
      mechanics.position_shift[i]   = job_info.position_shift[i];
      mechanics.update_workspace_offset((AxisEnum)i);
    }
  #endif

  // Resume the SD file from the last position
  char *fn = job_info.fileName;
  while (*fn == '/') fn++;
  sprintf_P(cmd, M23_CMD, fn);
  commands.process_now(cmd);
  sprintf_P(cmd, PSTR("M24 S%ld T%ld"), save_sdpos, job_info.print_job_counter_elapsed);
  commands.process_now(cmd);

}

/** Private Function */
void Restart::clear_job() { memset(&job_info, 0, sizeof(job_info)); }

void Restart::write_job() {
  bool failed = false;

  debug_info(PSTR("Write"));

  open(false);
  if (!job_file.seekSet(0)) failed = true;
  if (!failed && !job_file.write(&job_info, sizeof(job_info)) == sizeof(job_info))
    failed = true;
  close();
  if (failed) DEBUG_LM(DEB, " Restart file write failed.");

}

#if ENABLED(DEBUG_RESTART)

  void Restart::debug_info(PGM_P const prefix) {
    SERIAL_STR(prefix);
    SERIAL_MV("Job Restart Info...\nvalid Head:", (int)job_info.valid_head);
    SERIAL_EMV(" Valid Foot:", (int)job_info.valid_foot);
    if (job_info.valid_head) {
      if (job_info.valid_head == job_info.valid_foot) {
        SERIAL_MSG("position.x");
        LOOP_XYZE(i) SERIAL_MV(": ", job_info.axis_position_mm[i]);
        SERIAL_EOL();
        SERIAL_MSG("target_temperature");
        LOOP_HOTEND() SERIAL_MV(": ", job_info.target_temperature[h]);
        SERIAL_EOL();
        SERIAL_MSG("fanSpeeds");
        LOOP_FAN() SERIAL_MV(": ", job_info.fan_speed[f]);
        SERIAL_EOL();
        #if HAS_LEVELING
          SERIAL_EMV("leveling: ", int(job_info.leveling));
          SERIAL_EMV(" z_fade_height: ", int(job_info.z_fade_height));
        #endif
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
