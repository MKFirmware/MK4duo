/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2017 Alberto Cotronei @MagoKimbra
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
 * printer.cpp
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#include "../../base.h"
#include "printer.h"

#if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
  #include "../HAL/HAL_endstop_interrupts.h"
#endif

#if ENABLED(PCA9632)
  #include "../utility/pca9632.h"
#endif

const char axis_codes[XYZE] = {'X', 'Y', 'Z', 'E'};

Printer printer;

/**
 * Public Parameters
 */

bool    Printer::Running        = true,
        Printer::relative_mode  = false,
        Printer::pos_saved      = false;

volatile bool Printer::wait_for_user = false;

int16_t Printer::flow_percentage[EXTRUDERS]       = ARRAY_BY_EXTRUDERS(100),
        Printer::density_percentage[EXTRUDERS]    = ARRAY_BY_EXTRUDERS(100);

float   Printer::filament_size[EXTRUDERS]         = ARRAY_BY_EXTRUDERS(DEFAULT_NOMINAL_FILAMENT_DIA),
        Printer::volumetric_multiplier[EXTRUDERS] = ARRAY_BY_EXTRUDERS(1.0);

float   Printer::hotend_offset[XYZ][HOTENDS];

// Print status related
long    Printer::currentLayer  = 0,
        Printer::maxLayer      = -1;   // -1 = unknown
char    Printer::printName[21] = "";   // max. 20 chars + 0
float   Printer::progress      = 0.0;

// Extruders
uint8_t Printer::active_extruder    = 0,
        Printer::previous_extruder  = 0,
        Printer::target_extruder    = 0,
        Printer::active_driver      = 0;

uint8_t Printer::host_keepalive_interval = DEFAULT_KEEPALIVE_INTERVAL;

float   Printer::resume_position[XYZE];
bool    Printer::move_away_flag = false;

bool    Printer::axis_relative_modes[] = AXIS_RELATIVE_MODES,
          #if ENABLED(VOLUMETRIC_DEFAULT_ON)
            Printer::volumetric_enabled = true;
          #else
            Printer::volumetric_enabled = false;
          #endif

// Inactivity shutdown
millis_t  Printer::max_inactive_time      = 0;

// Interrupt Event
MK4duoInterruptEvent Printer::interruptEvent = INTERRUPT_EVENT_NONE;

// Printer mode
PrinterMode Printer::mode =
  #if ENABLED(PLOTTER)
    PRINTER_MODE_PLOTTER
  #elif ENABLED(SOLDER)
    PRINTER_MODE_SOLDER;
  #elif ENABLED(PICK_AND_PLACE)
    PRINTER_MODE_PICKER;
  #elif ENABLED(CNCROUTER)
    PRINTER_MODE_CNC;
  #elif ENABLED(LASER)
    PRINTER_MODE_LASER;
  #else
    PRINTER_MODE_FFF;
  #endif

// Print Job Timer
PrintCounter Printer::print_job_counter = PrintCounter();

#if HAS_SDSUPPORT
  bool Printer::sd_print_paused = false;
#endif

#if FAN_COUNT > 0
  int16_t Printer::fanSpeeds[FAN_COUNT];
  #if ENABLED(PROBING_FANS_OFF)
    bool Printer::fans_paused;
    int16_t Printer::paused_fanSpeeds[FAN_COUNT];
  #endif
#endif
#if HAS_CONTROLLERFAN
  uint8_t Printer::controller_fanSpeeds;
#endif
#if ENABLED(FAN_KICKSTART_TIME)
  uint8_t Printer::fanKickstart;
#endif

#if ENABLED(ADVANCED_PAUSE_FEATURE)
  AdvancedPauseMenuResponse Printer::advanced_pause_menu_response;
#endif

#if HAS_FIL_RUNOUT || HAS_EXT_ENCODER
  bool Printer::filament_ran_out = false;
#endif

#if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
  float Printer::motor_current[3 + DRIVER_EXTRUDERS];
#endif

#if HAS_CASE_LIGHT
  int   Printer::case_light_brightness;
  bool  Printer::case_light_on;
#endif

#if HAS_EXT_ENCODER
  int32_t Printer::encStepsSinceLastSignal[EXTRUDERS] = ARRAY_BY_EXTRUDERS(0);
  uint8_t Printer::encLastSignal[EXTRUDERS]           = ARRAY_BY_EXTRUDERS(0);
  int8_t  Printer::encLastDir[EXTRUDERS]              = ARRAY_BY_EXTRUDERS(1);
  int32_t Printer::encLastChangeAt[EXTRUDERS]         = ARRAY_BY_EXTRUDERS(0),
          Printer::encErrorSteps[EXTRUDERS]           = ARRAY_BY_EXTRUDERS(ENC_ERROR_STEPS);
#endif

/**
 * Public Function
 */

/**
 * MK4duo entry-point: Set up before the program loop
 *  - Set up Alligator Board
 *  - Set up the printer.kill pin, filament runout, power hold
 *  - Start the serial port
 *  - Print startup messages and diagnostics
 *  - Get EEPROM or default settings
 *  - Initialize managers for:
 *    • temperature
 *    • CNCROUTER
 *    • planner
 *    • watchdog
 *    • stepper
 *    • photo pin
 *    • laserbeam, laser and laser_raster
 *    • servos
 *    • LCD controller
 *    • Digipot I2C
 *    • Z probe sled
 *    • status LEDs
 */
void Printer::setup() {

  HAL::hwSetup();

  #if ENABLED(FILAMENT_RUNOUT_SENSOR)
    setup_filrunoutpin();
  #endif

  #if HAS_KILL
    SET_INPUT_PULLUP(KILL_PIN);
  #endif

  setup_powerhold();

  #if HAS_STEPPER_RESET
    disableStepperDrivers();
  #endif

  #if ENABLED(USE_WATCHDOG)
    watchdog_init();
  #endif

  SERIAL_INIT(BAUDRATE);
  SERIAL_L(START);

  // Check startup
  SERIAL_STR(INFO);
  HAL::showStartReason();

  SERIAL_LM(ECHO, BUILD_VERSION);

  #if ENABLED(STRING_DISTRIBUTION_DATE) && ENABLED(STRING_CONFIG_H_AUTHOR)
    SERIAL_LM(ECHO, MSG_CONFIGURATION_VER STRING_DISTRIBUTION_DATE MSG_AUTHOR STRING_CONFIG_H_AUTHOR);
    SERIAL_LM(ECHO, MSG_COMPILED __DATE__);
  #endif // STRING_DISTRIBUTION_DATE

  SERIAL_SMV(ECHO, MSG_FREE_MEMORY, HAL::getFreeRam());
  SERIAL_EMV(MSG_PLANNER_BUFFER_BYTES, (int)sizeof(block_t)*BLOCK_BUFFER_SIZE);

  // Send "ok" after commands by default
  commands.reset_send_ok();

  #if MECH(MUVE3D) && ENABLED(PROJECTOR_PORT) && ENABLED(PROJECTOR_BAUDRATE)
    DLPSerial.begin(PROJECTOR_BAUDRATE);
  #endif

  mechanics.Init();

  #if HAS_SDSUPPORT
    // loads custom configuration from SDCARD if available else uses defaults
    card.RetrieveSettings();
    HAL::delayMilliseconds(500);
  #endif

  // Loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
  #if !HAS_EEPROM_SD
    eeprom.Load_Settings();
  #endif

  #if ENABLED(WORKSPACE_OFFSETS)
    // Initialize current position based on home_offset
    COPY_ARRAY(mechanics.current_position, mechanics.home_offset);
  #else
    ZERO(mechanics.current_position);
  #endif

  // Vital to init stepper/planner equivalent for current_position
  mechanics.sync_plan_position();

  thermalManager.init();    // Initialize temperature loop

  #if ENABLED(CNCROUTER)
    cnc_init();
  #endif

  stepper.init();    // Initialize stepper, this enables interrupts!
  
  #if HAS_SERVOS
    servo_init();
  #endif

  #if HAS_PHOTOGRAPH
    OUT_WRITE(PHOTOGRAPH_PIN, LOW);
  #endif

  #if HAS_CASE_LIGHT
    case_light_on = CASE_LIGHT_DEFAULT_ON;
    case_light_brightness = CASE_LIGHT_DEFAULT_BRIGHTNESS;
    update_case_light();
  #endif

  #if HAS_DOOR
    #if ENABLED(DOOR_OPEN_PULLUP)
      SET_INPUT_PULLUP(DOOR_PIN);
    #else
      SET_INPUT(DOOR_PIN);
    #endif
  #endif

  #if HAS_POWER_CHECK && HAS_SDSUPPORT
    #if ENABLED(POWER_CHECK_PULLUP)
      SET_INPUT_PULLUP(POWER_CHECK_PIN);
    #else
      SET_INPUT(POWER_CHECK_PIN);
    #endif
  #endif

  #if HAS_BED_PROBE
    probe.set_enable(false);
  #endif

  #if HAS_STEPPER_RESET
    enableStepperDrivers();
  #endif

  #if ENABLED(DIGIPOT_I2C)
    digipot_i2c_init();
  #endif

  #if HAS_Z_PROBE_SLED
    OUT_WRITE(SLED_PIN, LOW); // turn it off
  #endif

  #if HAS_HOME
    SET_INPUT_PULLUP(HOME_PIN);
  #endif

  #if PIN_EXISTS(STAT_LED_RED_PIN)
    OUT_WRITE(STAT_LED_RED_PIN, LOW); // turn it off
  #endif

  #if PIN_EXISTS(STAT_LED_BLUE_PIN)
    OUT_WRITE(STAT_LED_BLUE_PIN, LOW); // turn it off
  #endif

  #if ENABLED(RGB_LED) || ENABLED(RGBW_LED)
    SET_OUTPUT(RGB_LED_R_PIN);
    SET_OUTPUT(RGB_LED_G_PIN);
    SET_OUTPUT(RGB_LED_B_PIN);
    #if ENABLED(RGBW_LED)
      SET_OUTPUT(RGB_LED_W_PIN);
    #endif
  #endif

  #if ENABLED(LASER)
    laser.Init();
  #endif

  #if ENABLED(FLOWMETER_SENSOR)
    #if ENABLED(MINFLOW_PROTECTION)
      flow_firstread = false;
    #endif
    flow_init();
  #endif

  #if ENABLED(RFID_MODULE)
    RFID_ON = RFID522.init();
    if (RFID_ON)
      SERIAL_EM("RFID CONNECT");
  #endif

  lcd_init();
  #if ENABLED(SHOW_BOOTSCREEN)
    #if ENABLED(DOGLCD)                           // On DOGM the first bootscreen is already drawn
      #if ENABLED(SHOW_CUSTOM_BOOTSCREEN)
        printer.safe_delay(CUSTOM_BOOTSCREEN_TIMEOUT);    // Custom boot screen pause
        lcd_bootscreen();                         // Show MK4duo boot screen
      #endif
      printer.safe_delay(BOOTSCREEN_TIMEOUT);
    #elif ENABLED(ULTRA_LCD)
      lcd_bootscreen();
      #if DISABLED(SDSUPPORT)
        lcd_init();
      #endif
    #endif
  #endif

  #if ENABLED(COLOR_MIXING_EXTRUDER) && MIXING_VIRTUAL_TOOLS > 1
    // Initialize mixing to 100% color 1
    for (uint8_t i = 0; i < MIXING_STEPPERS; i++)
      mixing_factor[i] = (i == 0) ? 1.0 : 0.0;
    for (uint8_t t = 0; t < MIXING_VIRTUAL_TOOLS; t++)
      for (uint8_t i = 0; i < MIXING_STEPPERS; i++)
        mixing_virtual_tool_mix[t][i] = mixing_factor[i];
  #endif

  #if ENABLED(BLTOUCH)
    // Make sure any BLTouch error condition is cleared
    probe.bltouch_command(BLTOUCH_RESET);
    probe.set_bltouch_deployed(true);
    probe.set_bltouch_deployed(false);
  #endif

  #if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
    setup_endstop_interrupts();
  #endif

  #if ENABLED(DELTA_HOME_ON_POWER)
    home_all_axes();
  #endif

}

void Printer::safe_delay(millis_t ms) {
  while (ms > 50) {
    ms -= 50;
    HAL::delayMilliseconds(50);
    thermalManager.manage_temp_controller();
  }
  HAL::delayMilliseconds(ms);
}

/**
 * Prepare to do endstop or probe moves
 * with custom feedrates.
 *
 *  - Save current feedrates
 *  - Reset the rate multiplier
 *  - Reset the command timeout
 *  - Enable the endstops (for endstop moves)
 */
void Printer::setup_for_endstop_or_probe_move() {
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) DEBUG_POS("setup_for_endstop_or_probe_move", mechanics.current_position);
  #endif
  mechanics.saved_feedrate_mm_s = mechanics.feedrate_mm_s;
  mechanics.saved_feedrate_percentage = mechanics.feedrate_percentage;
  mechanics.feedrate_percentage = 100;
  commands.refresh_cmd_timeout();
}

void Printer::clean_up_after_endstop_or_probe_move() {
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) DEBUG_POS("clean_up_after_endstop_or_probe_move", mechanics.current_position);
  #endif
  mechanics.feedrate_mm_s = mechanics.saved_feedrate_mm_s;
  mechanics.feedrate_percentage = mechanics.saved_feedrate_percentage;
  commands.refresh_cmd_timeout();
}

/**
 * Set XYZE mechanics.destination and mechanics.feedrate_mm_s from the current GCode command
 *
 *  - Set mechanics.destination from included axis codes
 *  - Set to current for missing axis codes
 *  - Set the mechanics.feedrate_mm_s, if included
 */
void Printer::get_destination_from_command() {

  #if ENABLED(IDLE_OOZING_PREVENT)
    if (parser.seen('E')) IDLE_OOZING_retract(false);
  #endif

  LOOP_XYZE(i) {
    if (parser.seen(axis_codes[i]))
      mechanics.destination[i] = parser.value_axis_units((AxisEnum)i) + (axis_relative_modes[i] || relative_mode ? mechanics.current_position[i] : 0);
    else
      mechanics.destination[i] = mechanics.current_position[i];
  }

  if (parser.linearval('F') > 0.0)
    mechanics.feedrate_mm_s = MMM_TO_MMS(parser.value_feedrate());

  if (parser.seen('P'))
    mechanics.destination[E_AXIS] = (parser.value_axis_units(E_AXIS) * density_percentage[previous_extruder] / 100) + mechanics.current_position[E_AXIS];

  if(!DEBUGGING(DRYRUN))
    print_job_counter.data.filamentUsed += (mechanics.destination[E_AXIS] - mechanics.current_position[E_AXIS]);

  #if ENABLED(COLOR_MIXING_EXTRUDER)
    gcode_get_mix();
  #endif

  #if ENABLED(RFID_MODULE)
    if(!DEBUGGING(DRYRUN))
      RFID522.RfidData[active_extruder].data.lenght -= (mechanics.destination[E_AXIS] - mechanics.current_position[E_AXIS]);
  #endif

  #if ENABLED(NEXTION) && ENABLED(NEXTION_GFX)
    #if MECH(DELTA)
      if((parser.seen('X') || parser.seen('Y')) && parser.seen('E'))
        gfx_line_to(mechanics.destination[X_AXIS] + (X_MAX_POS), mechanics.destination[Y_AXIS] + (Y_MAX_POS), mechanics.destination[Z_AXIS]);
      else
        gfx_cursor_to(mechanics.destination[X_AXIS] + (X_MAX_POS), mechanics.destination[Y_AXIS] + (Y_MAX_POS), mechanics.destination[Z_AXIS]);
    #else
      if((parser.seen('X') || parser.seen('Y')) && parser.seen('E'))
        gfx_line_to(mechanics.destination[X_AXIS], mechanics.destination[Y_AXIS], mechanics.destination[Z_AXIS]);
      else
        gfx_cursor_to(mechanics.destination[X_AXIS], mechanics.destination[Y_AXIS], mechanics.destination[Z_AXIS]);
    #endif
  #endif
}

/**
 * Set target_tool from the T parameter or the active_tool
 *
 * Returns TRUE if the target is invalid
 */
bool Printer::get_target_tool_from_command(const uint16_t code) {
  if (parser.seenval('T')) {
    const int8_t t = parser.value_byte();
    if (t >= EXTRUDERS) {
      SERIAL_SMV(ECHO, "M", code);
      SERIAL_EMV(" " MSG_INVALID_EXTRUDER, t);
      return true;
    }
    target_extruder = t;
  }
  else
    target_extruder = active_extruder;

  return false;
}

/**
 * Kill all activity and lock the machine.
 * After this the machine will need to be reset.
 */
void Printer::kill(const char* lcd_msg) {
  SERIAL_LM(ER, MSG_ERR_KILLED);

  thermalManager.disable_all_heaters();
  thermalManager.disable_all_coolers();
  stepper.disable_all_steppers();

  #if ENABLED(KILL_METHOD) && (KILL_METHOD == 1)
    HAL::resetHardware();
  #endif
  #if ENABLED(FLOWMETER_SENSOR) && ENABLED(MINFLOW_PROTECTION)
    flow_firstread = false;
  #endif

  #if ENABLED(ULTRA_LCD)
    kill_screen(lcd_msg);
  #else
    UNUSED(lcd_msg);
  #endif

  HAL::delayMilliseconds(600);  // Wait a short time (allows messages to get out before shutting down.
  cli(); // Stop interrupts

  HAL::delayMilliseconds(250);  // Wait to ensure all interrupts routines stopped
  thermalManager.disable_all_heaters(); // Turn off heaters again

  #if ENABLED(LASER)
    laser.Init();
    #if ENABLED(LASER_PERIPHERALS)
      laser.peripherals_off();
    #endif
  #endif

  #if ENABLED(CNCROUTER)
     disable_cncrouter();
  #endif

  #if HAS_POWER_SWITCH
    SET_INPUT(PS_ON_PIN);
  #endif

  #if HAS(SUICIDE)
    suicide();
  #endif

  while(1) {
    #if ENABLED(USE_WATCHDOG)
      watchdog_reset();
    #endif
  } // Wait for reset
}

/**
 * Turn off heaters and stop the print in progress
 * After a stop the machine may be resumed with M999
 */
void Printer::Stop() {
  #if ENABLED(FLOWMETER_SENSOR) && ENABLED(MINFLOW_PROTECTION)
    flow_firstread = false;
  #endif

  thermalManager.disable_all_heaters();
  thermalManager.disable_all_coolers();

  #if ENABLED(PROBING_FANS_OFF)
    if (fans_paused) fans_pause(false); // put things back the way they were
  #endif

  #if ENABLED(LASER)
    if (laser.diagnostics) SERIAL_EM("Laser set to off, Stop() called");
    laser.extinguish();
    #if ENABLED(LASER_PERIPHERALS)
      laser.peripherals_off();
    #endif
  #endif

  #if ENABLED(CNCROUTER)
     disable_cncrouter();
  #endif

  if (printer.IsRunning()) {
    Running = false;
    commands.save_last_gcode; // Save last g_code for restart
    SERIAL_LM(ER, MSG_ERR_STOPPED);
    SERIAL_STR(PAUSE);
    SERIAL_EOL();
    LCD_MESSAGEPGM(MSG_STOPPED);
  }
}

void Printer::quickstop_stepper() {
  stepper.quick_stop();
  stepper.synchronize();
  mechanics.set_current_from_steppers_for_axis(ALL_AXES);
  mechanics.sync_plan_position();
}

void Printer::calculate_volumetric_multipliers() {
  for (uint8_t e = 0; e < EXTRUDERS; e++)
    volumetric_multiplier[e] = calculate_volumetric_multiplier(filament_size[e]);
}

void Printer::idle(bool no_stepper_sleep/*=false*/) {

  static uint8_t cycle_1500ms = 15;

  // Start event periodical

  #if ENABLED(NEXTION)
    lcd_key_touch_update();
  #else
    lcd_update();
  #endif

  #if ENABLED(HOST_KEEPALIVE_FEATURE)
    host_keepalive();
  #endif

  #if ENABLED(AUTO_REPORT_TEMPERATURES) && (HAS_TEMP_HOTEND || HAS_TEMP_BED)
    thermalManager.auto_report_temperatures();
  #endif

  #if ENABLED(FLOWMETER_SENSOR)
    flowrate_manage();
  #endif

  #if ENABLED(CNCROUTER)
    cnc_manage();
  #endif

  manage_inactivity(no_stepper_sleep);

  handle_Interrupt_Event();

  print_job_counter.tick();

  if (HAL::execute_100ms) {
    // Event 100 Ms
    HAL::execute_100ms = false;
    thermalManager.manage_temp_controller();
    if (--cycle_1500ms == 0) {
      // Event 1500 Ms
      cycle_1500ms = 15;
      #if ENABLED(NEXTION)
        nextion_draw_update();
      #endif
    }
  }
}

/**
 * Manage several activities:
 *  - Check for Filament Runout
 *  - Keep the command buffer full
 *  - Check for maximum inactive time between commands
 *  - Check for maximum inactive time between stepper commands
 *  - Check if pin CHDK needs to go LOW
 *  - Check for KILL button held down
 *  - Check for HOME button held down
 *  - Check if cooling fan needs to be switched on
 *  - Check if an idle but hot extruder needs filament extruded (EXTRUDER_RUNOUT_PREVENT)
 *  - Check oozing prevent
 *  - Read o Write Rfid
 */
void Printer::manage_inactivity(bool ignore_stepper_queue/*=false*/) {

  #if HAS_FIL_RUNOUT && FILAMENT_RUNOUT_DOUBLE_CHECK > 0
    static bool filament_double_check = false;
    static millis_t filament_switch_time = 0;
    if ((IS_SD_PRINTING || print_job_counter.isRunning()) && READ(FIL_RUNOUT_PIN) == FIL_RUNOUT_PIN_INVERTING) {
      if (filament_double_check) {
        if (ELAPSED(millis(), filament_switch_time) {
          setInterruptEvent(INTERRUPT_EVENT_FIL_RUNOUT);
          filament_double_check = false;
        }
      }
      else {
        filament_double_check = true;
        filament_switch_time = millis() + FILAMENT_RUNOUT_DOUBLE_CHECK;
      }
    }
  #elif HAS_FIL_RUNOUT
    if ((IS_SD_PRINTING || print_job_counter.isRunning()) && READ(FIL_RUNOUT_PIN) == FIL_RUNOUT_PIN_INVERTING)
      setInterruptEvent(INTERRUPT_EVENT_FIL_RUNOUT);
  #endif

  commands.get_available_commands();

  const millis_t ms = millis();

  if (max_inactive_time && ELAPSED(ms, commands.previous_cmd_ms + max_inactive_time)) {
    SERIAL_LMT(ER, MSG_KILL_INACTIVE_TIME, parser.command_ptr);
    kill(PSTR(MSG_KILLED));
  }

  // Prevent steppers timing-out in the middle of M600
  #if ENABLED(ADVANCED_PAUSE_FEATURE) && ENABLED(PAUSE_PARK_NO_STEPPER_TIMEOUT)
    #define MOVE_AWAY_TEST !move_away_flag
  #else
    #define MOVE_AWAY_TEST true
  #endif

  #if ENABLED(FLOWMETER_SENSOR) && ENABLED(MINFLOW_PROTECTION)
    if (flow_firstread && print_job_counter.isRunning() && (get_flowrate() < (float)MINFLOW_PROTECTION)) {
      flow_firstread = false;
      kill(PSTR(MSG_KILLED));
    }
  #endif

  if (MOVE_AWAY_TEST && stepper.stepper_inactive_time && ELAPSED(ms, commands.previous_cmd_ms + stepper.stepper_inactive_time)
      && !ignore_stepper_queue && !planner.blocks_queued()) {
    #if ENABLED(DISABLE_INACTIVE_X)
      disable_X();
    #endif
    #if ENABLED(DISABLE_INACTIVE_Y)
      disable_Y();
    #endif
    #if ENABLED(DISABLE_INACTIVE_Z)
      disable_Z();
    #endif
    #if ENABLED(DISABLE_INACTIVE_E)
      stepper.disable_e_steppers();
    #endif
    #if ENABLED(LASER)
      if (laser.time / 60000 > 0) {
        laser.lifetime += laser.time / 60000; // convert to minutes
        laser.time = 0;
      }
      laser.extinguish();
      #if ENABLED(LASER_PERIPHERALS)
        laser.peripherals_off();
      #endif
    #endif
  }

  #if HAS_CHDK // Check if pin should be set to LOW after M240 set it to HIGH
    if (chdkActive && ELAPSED(ms, chdkHigh + CHDK_DELAY)) {
      chdkActive = false;
      WRITE(CHDK_PIN, LOW);
    }
  #endif

  #if HAS_KILL

    // Check if the kill button was pressed and wait just in case it was an accidental
    // key kill key press
    // -------------------------------------------------------------------------------
    static int killCount = 0;   // make the inactivity button a bit less responsive
    const int KILL_DELAY = 750;
    if (!READ(KILL_PIN))
       killCount++;
    else if (killCount > 0)
       killCount--;

    // Exceeded threshold and we can confirm that it was not accidental
    // KILL the machine
    // ----------------------------------------------------------------
    if (killCount >= KILL_DELAY) {
      SERIAL_LM(ER, MSG_KILL_BUTTON);
      kill(PSTR(MSG_KILLED));
    }
  #endif

  #if HAS_HOME
    // Check to see if we have to home, use poor man's debouncer
    // ---------------------------------------------------------
    static int homeDebounceCount = 0;   // poor man's debouncing count
    const int HOME_DEBOUNCE_DELAY = 750;
    if (!IS_SD_PRINTING && !READ(HOME_PIN)) {
      if (!homeDebounceCount) {
        commands.enqueue_and_echo_commands_P(PSTR("G28"));
        LCD_MESSAGEPGM(MSG_AUTO_HOME);
      }
      if (homeDebounceCount < HOME_DEBOUNCE_DELAY)
        homeDebounceCount++;
      else
        homeDebounceCount = 0;
    }
  #endif

  #if HAS_CONTROLLERFAN
    controllerFan(); // Check if fan should be turned on to cool stepper drivers down
  #endif

  #if HAS_POWER_SWITCH
    if (!powerManager.powersupply_on) powerManager.check(); // Check Power
  #endif

  #if ENABLED(EXTRUDER_RUNOUT_PREVENT)
    if (ELAPSED(ms, commands.previous_cmd_ms + (EXTRUDER_RUNOUT_SECONDS) * 1000UL)
      && thermalManager.degHotend(active_extruder) > EXTRUDER_RUNOUT_MINTEMP) {
      bool oldstatus = 0;
      #if ENABLED(DONDOLO_SINGLE_MOTOR)
        oldstatus = E0_ENABLE_READ;
        enable_E0();
      #else // !DONDOLO_SINGLE_MOTOR
        switch (active_extruder) {
          case 0: oldstatus = E0_ENABLE_READ; enable_E0(); break;
          #if DRIVER_EXTRUDERS > 1
            case 1: oldstatus = E1_ENABLE_READ; enable_E1(); break;
            #if DRIVER_EXTRUDERS > 2
              case 2: oldstatus = E2_ENABLE_READ; enable_E2(); break;
              #if DRIVER_EXTRUDERS > 3
                case 3: oldstatus = E3_ENABLE_READ; enable_E3(); break;
                #if DRIVER_EXTRUDERS > 4
                  case 4: oldstatus = E4_ENABLE_READ; enable_E4(); break;
                  #if DRIVER_EXTRUDERS > 5
                    case 5: oldstatus = E5_ENABLE_READ; enable_E5(); break;
                  #endif // DRIVER_EXTRUDERS > 5
                #endif // DRIVER_EXTRUDERS > 4
              #endif // DRIVER_EXTRUDERS > 3
            #endif // DRIVER_EXTRUDERS > 2
          #endif // DRIVER_EXTRUDERS > 1
        }
      #endif // !DONDOLO_SINGLE_MOTOR

      commands.previous_cmd_ms = ms; // commands.refresh_cmd_timeout()

      const float olde = mechanics.current_position[E_AXIS];
      mechanics.current_position[E_AXIS] += EXTRUDER_RUNOUT_EXTRUDE;
      planner.buffer_line_kinematic(mechanics.current_position, MMM_TO_MMS(EXTRUDER_RUNOUT_SPEED), active_extruder);
      mechanics.current_position[E_AXIS] = olde;
      mechanics.set_e_position_mm(olde);
      stepper.synchronize();
      #if ENABLED(DONDOLO_SINGLE_MOTOR)
        E0_ENABLE_WRITE(oldstatus);
      #else
        switch(active_extruder) {
          case 0: E0_ENABLE_WRITE(oldstatus); break;
          #if DRIVER_EXTRUDERS > 1
            case 1: E1_ENABLE_WRITE(oldstatus); break;
            #if DRIVER_EXTRUDERS > 2
              case 2: E2_ENABLE_WRITE(oldstatus); break;
              #if DRIVER_EXTRUDERS > 3
                case 3: E3_ENABLE_WRITE(oldstatus); break;
                #if DRIVER_EXTRUDERS > 4
                  case 4: E4_ENABLE_WRITE(oldstatus); break;
                  #if DRIVER_EXTRUDERS > 5
                    case 5: E5_ENABLE_WRITE(oldstatus); break;
                  #endif // DRIVER_EXTRUDERS > 5
                #endif // DRIVER_EXTRUDERS > 4
              #endif // DRIVER_EXTRUDERS > 3
            #endif // DRIVER_EXTRUDERS > 2
          #endif // DRIVER_EXTRUDERS > 1
        }
      #endif // !DONDOLO_SINGLE_MOTOR
    }
  #endif // EXTRUDER_RUNOUT_PREVENT

  #if ENABLED(DUAL_X_CARRIAGE)
    // handle delayed move timeout
    if (mechanics.delayed_move_time && ELAPSED(ms, mechanics.delayed_move_time + 1000UL) && printer.IsRunning()) {
      // travel moves have been received so enact them
      mechanics.delayed_move_time = 0xFFFFFFFFUL; // force moves to be done
      mechanics.set_destination_to_current();
      mechanics.prepare_move_to_destination();
    }
  #endif

  #if ENABLED(IDLE_OOZING_PREVENT)
    if (planner.blocks_queued()) axis_last_activity = millis();
    if (thermalManager.degHotend(active_extruder) > IDLE_OOZING_MINTEMP && !(DEBUGGING(DRYRUN)) && IDLE_OOZING_enabled) {
      #if ENABLED(FILAMENTCHANGEENABLE)
        if (!filament_changing)
      #endif
      {
        if (thermalManager.degTargetHotend(active_extruder) < IDLE_OOZING_MINTEMP) {
          IDLE_OOZING_retract(false);
        }
        else if ((millis() - axis_last_activity) >  IDLE_OOZING_SECONDS * 1000UL) {
          IDLE_OOZING_retract(true);
        }
      }
    }
  #endif

  #if ENABLED(RFID_MODULE)
    for (uint8_t e = 0; e < EXTRUDERS; e++) {
      if (Spool_must_read[e]) {
        if (RFID522.getID(e)) {
          Spool_ID[e] = RFID522.RfidDataID[e].Spool_ID;
          HAL::delayMilliseconds(200);
          if (RFID522.readBlock(e)) {
            Spool_must_read[e] = false;
            density_percentage[e] = RFID522.RfidData[e].data.density;
            filament_size[e] = RFID522.RfidData[e].data.size;
            calculate_volumetric_multipliers();
            RFID522.printInfo(e);
          }
        }
      }

      if (Spool_must_write[e]) {
        if (RFID522.getID(e)) {
          if (Spool_ID[e] == RFID522.RfidDataID[e].Spool_ID) {
            HAL::delayMilliseconds(200);
            if (RFID522.writeBlock(e)) {
              Spool_must_write[e] = false;
              SERIAL_SMV(INFO, "Spool on E", e);
              SERIAL_EM(" writed!");
              RFID522.printInfo(e);
            }
          }
        }
      }
    }
  #endif

  #if ENABLED(TEMP_STAT_LEDS)
    handle_status_leds();
  #endif

  #if ENABLED(HAVE_TMC2130)
    checkOverTemp();
  #endif

  planner.check_axes_activity();
}

#if ENABLED(PROBING_FANS_OFF)

  void Printer::fans_pause(const bool p) {
    if (p != fans_paused) {
      fans_paused = p;
      if (p)
        LOOP_FAN() {
          paused_fanSpeeds[f] = fanSpeeds[f];
          fanSpeeds[f] = 0;
        }
      else
        LOOP_FAN()
          fanSpeeds[f] = paused_fanSpeeds[f];
    }
  }

#endif // PROBING_FANS_OFF

void Printer::setInterruptEvent(const MK4duoInterruptEvent event) {
  if (interruptEvent == INTERRUPT_EVENT_NONE)
    interruptEvent = event;
}

void Printer::handle_Interrupt_Event() {

  if (interruptEvent == INTERRUPT_EVENT_NONE) return; // Exit if none Event

  const MK4duoInterruptEvent event = interruptEvent;
  interruptEvent = INTERRUPT_EVENT_NONE;

  switch(event) {
    #if HAS_FIL_RUNOUT || HAS_DAV_SYSTEM
      //case INTERRUPT_EVENT_FIL_RUNOUT:
      case INTERRUPT_EVENT_DAV_SYSTEM:
        if (!filament_ran_out) {
          filament_ran_out = true;
          commands.enqueue_and_echo_commands_P(PSTR(FILAMENT_RUNOUT_SCRIPT));
          SERIAL_LM(REQUEST_PAUSE, "End Filament detect");
          stepper.synchronize();
        }
        break;
    #endif

    #if HAS_EXT_ENCODER
      case INTERRUPT_EVENT_ENC_DETECT:
        if (!filament_ran_out && (IS_SD_PRINTING || print_job_counter.isRunning())) {
          filament_ran_out = true;
          stepper.synchronize();
          #if HAS_SDSUPPORT
            if (card.cardOK && card.isFileOpen() && IS_SD_PRINTING)
              gcode_M25();
            else
          #endif
          if (print_job_counter.isRunning()) {
            #if ENABLED(PARK_HEAD_ON_PAUSE)
              gcode_M125();
            #endif
          }
          SERIAL_LM(REQUEST_PAUSE, "Extruder jam detected");
        }
        break;
    #endif

    default:
      break;
  }
}

#if EXTRUDERS > 1 || ENABLED(COLOR_MIXING_EXTRUDER)

  void Printer::tool_change(const uint8_t tmp_extruder, const float fr_mm_s/*=0.0*/, bool no_move/*=false*/) {

    #if ENABLED(COLOR_MIXING_EXTRUDER) && MIXING_VIRTUAL_TOOLS > 1
      // T0-T15: Switch virtual tool by changing the mix
      if (tmp_extruder >= MIXING_VIRTUAL_TOOLS)
        return invalid_extruder_error(tmp_extruder);
    #else
      if (tmp_extruder >= EXTRUDERS)
        return invalid_extruder_error(tmp_extruder);
    #endif

    #if HOTENDS == 1

      #if ENABLED(COLOR_MIXING_EXTRUDER) && MIXING_VIRTUAL_TOOLS > 1

        // T0-Tnnn: Switch virtual tool by changing the mix
        for (uint8_t j = 0; j < MIXING_STEPPERS; j++)
          mixing_factor[j] = mixing_virtual_tool_mix[tmp_extruder][j];

        SERIAL_EMV(MSG_ACTIVE_COLOR, (int)tmp_extruder);

      #elif HAS_MKMULTI_TOOLS

        UNUSED(fr_mm_s);
        UNUSED(no_move);

        MK_multi_tool_change(tmp_extruder);

      #else

        UNUSED(fr_mm_s);
        UNUSED(no_move);

        // Set the new active extruder
        previous_extruder = active_extruder;
        active_driver = active_extruder = tmp_extruder;

      #endif

    #else // HOTENDS > 1

      const float old_feedrate_mm_s = fr_mm_s > 0.0 ? fr_mm_s : mechanics.feedrate_mm_s;

      mechanics.feedrate_mm_s = fr_mm_s > 0.0 ? fr_mm_s : XY_PROBE_FEEDRATE_MM_S;

      if (tmp_extruder != active_extruder) {
        if (!no_move && mechanics.axis_unhomed_error()) {
          SERIAL_EM("No move on toolchange");
          no_move = true;
        }

        // Save current position to mechanics.destination, for use later
        mechanics.set_destination_to_current();

        #if ENABLED(DUAL_X_CARRIAGE)

          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) {
              SERIAL_MSG("Dual X Carriage Mode ");
              switch (mechanics.dual_x_carriage_mode) {
                case DXC_DUPLICATION_MODE: SERIAL_EM("DXC_DUPLICATION_MODE"); break;
                case DXC_AUTO_PARK_MODE: SERIAL_EM("DXC_AUTO_PARK_MODE"); break;
                case DXC_FULL_CONTROL_MODE: SERIAL_EM("DXC_FULL_CONTROL_MODE"); break;
              }
            }
          #endif

          const float xhome = mechanics.x_home_pos(active_extruder);
          if (mechanics.dual_x_carriage_mode == DXC_AUTO_PARK_MODE
              && printer.IsRunning()
              && (mechanics.delayed_move_time || mechanics.current_position[X_AXIS] != xhome)
          ) {
            float raised_z = mechanics.current_position[Z_AXIS] + TOOLCHANGE_PARK_ZLIFT;
            #if ENABLED(MAX_SOFTWARE_ENDSTOPS)
              NOMORE(raised_z, endstops.soft_endstop_max[Z_AXIS]);
            #endif
            #if ENABLED(DEBUG_LEVELING_FEATURE)
              if (DEBUGGING(LEVELING)) {
                SERIAL_EMV("Raise to ", raised_z);
                SERIAL_EMV("MoveX to ", xhome);
                SERIAL_EMV("Lower to ", mechanics.current_position[Z_AXIS]);
              }
            #endif
            // Park old head: 1) raise 2) move to park position 3) lower
            for (uint8_t i = 0; i < 3; i++)
              planner.buffer_line(
                i == 0 ? mechanics.current_position[X_AXIS] : xhome,
                mechanics.current_position[Y_AXIS],
                i == 2 ? mechanics.current_position[Z_AXIS] : raised_z,
                mechanics.current_position[E_AXIS],
                mechanics.max_feedrate_mm_s[i == 1 ? X_AXIS : Z_AXIS],
                active_extruder
              );
            stepper.synchronize();
          }

          // apply Y & Z extruder offset (x offset is already used in determining home pos)
          mechanics.current_position[Y_AXIS] -= hotend_offset[Y_AXIS][active_extruder] - hotend_offset[Y_AXIS][tmp_extruder];
          mechanics.current_position[Z_AXIS] -= hotend_offset[Z_AXIS][active_extruder] - hotend_offset[Z_AXIS][tmp_extruder];

          // Activate the new extruder
          active_extruder = active_driver = tmp_extruder;

          // This function resets the max/min values - the current position may be overwritten below.
          mechanics.set_axis_is_at_home(X_AXIS);

          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) DEBUG_POS("New Extruder", mechanics.current_position);
          #endif

          // Only when auto-parking are carriages safe to move
          if (mechanics.dual_x_carriage_mode != DXC_AUTO_PARK_MODE) no_move = true;

          switch (mechanics.dual_x_carriage_mode) {
            case DXC_FULL_CONTROL_MODE:
              // New current position is the position of the activated hotend
              mechanics.current_position[X_AXIS] = LOGICAL_X_POSITION(mechanics.inactive_hotend_x_pos);
              // Save the inactive hotend's position (from the old mechanics.current_position)
              mechanics.inactive_hotend_x_pos = RAW_X_POSITION(mechanics.destination[X_AXIS]);
              break;
            case DXC_AUTO_PARK_MODE:
              // record raised toolhead position for use by unpark
              COPY_ARRAY(mechanics.raised_parked_position, mechanics.current_position);
              mechanics.raised_parked_position[Z_AXIS] += TOOLCHANGE_UNPARK_ZLIFT;
              #if ENABLED(MAX_SOFTWARE_ENDSTOPS)
                NOMORE(mechanics.raised_parked_position[Z_AXIS], endstops.soft_endstop_max[Z_AXIS]);
              #endif
              mechanics.active_hotend_parked = true;
              mechanics.delayed_move_time = 0;
              break;
            case DXC_DUPLICATION_MODE:
              // If the new hotend is the left one, set it "parked"
              // This triggers the second hotend to move into the duplication position
              mechanics.active_hotend_parked = (active_extruder == 0);

              if (mechanics.active_hotend_parked)
                mechanics.current_position[X_AXIS] = LOGICAL_X_POSITION(mechanics.inactive_hotend_x_pos);
              else
                mechanics.current_position[X_AXIS] = mechanics.destination[X_AXIS] + mechanics.duplicate_hotend_x_offset;
              mechanics.inactive_hotend_x_pos = RAW_X_POSITION(mechanics.destination[X_AXIS]);
              mechanics.hotend_duplication_enabled = false;
              #if ENABLED(DEBUG_LEVELING_FEATURE)
                if (DEBUGGING(LEVELING)) {
                  SERIAL_EMV("Set inactive_extruder_x_pos=", inactive_extruder_x_pos);
                  SERIAL_EM("Clear mechanics.hotend_duplication_enabled");
                }
              #endif
              break;
          }

          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) {
              SERIAL_EMV("Active hotend parked: ", mechanics.active_hotend_parked ? "yes" : "no");
              DEBUG_POS("New hotend (parked)", mechanics.current_position);
            }
          #endif

          // No extra case for HAS_ABL in DUAL_X_CARRIAGE. Does that mean they don't work together?
        #else // !DUAL_X_CARRIAGE

          #if HAS_DONDOLO
            // <0 if the new nozzle is higher, >0 if lower. A bigger raise when lower.
            float z_diff = hotend_offset[Z_AXIS][active_extruder] - hotend_offset[Z_AXIS][tmp_extruder],
                  z_raise = 0.3 + (z_diff > 0.0 ? z_diff : 0.0),
                  z_back  = 0.3 - (z_diff < 0.0 ? z_diff : 0.0);

            // Always raise by some amount (mechanics.destination copied from current_position earlier)
            mechanics.destination[Z_AXIS] += z_raise;
            planner.buffer_line_kinematic(mechanics.destination, mechanics.max_feedrate_mm_s[Z_AXIS], active_extruder);
            stepper.synchronize();

            move_extruder_servo(tmp_extruder);
            HAL::delayMilliseconds(500);

            // Move back down
            mechanics.destination[Z_AXIS] = mechanics.current_position[Z_AXIS] - z_back;
            planner.buffer_line_kinematic(mechanics.destination, mechanics.max_feedrate_mm_s[Z_AXIS], active_extruder);
            stepper.synchronize();
          #endif

          /**
           * Set current_position to the position of the new nozzle.
           * Offsets are based on linear distance, so we need to get
           * the resulting position in coordinate space.
           *
           * - With grid or 3-point leveling, offset XYZ by a tilted vector
           * - With mesh leveling, update Z for the new position
           * - Otherwise, just use the raw linear distance
           *
           * Software endstops are altered here too. Consider a case where:
           *   E0 at X=0 ... E1 at X=10
           * When we switch to E1 now X=10, but E1 can't move left.
           * To express this we apply the change in XY to the software endstops.
           * E1 can move farther right than E0, so the right limit is extended.
           *
           * Note that we don't adjust the Z software endstops. Why not?
           * Consider a case where Z=0 (here) and switching to E1 makes Z=1
           * because the bed is 1mm lower at the new position. As long as
           * the first nozzle is out of the way, the carriage should be
           * allowed to move 1mm lower. This technically "breaks" the
           * Z software endstop. But this is technically correct (and
           * there is no viable alternative).
           */
          #if ABL_PLANAR
            // Offset extruder, make sure to apply the bed level rotation matrix
            vector_3 tmp_offset_vec = vector_3(hotend_offset[X_AXIS][tmp_extruder],
                                               hotend_offset[Y_AXIS][tmp_extruder],
                                               0),
                     act_offset_vec = vector_3(hotend_offset[X_AXIS][active_extruder],
                                               hotend_offset[Y_AXIS][active_extruder],
                                               0),
                     offset_vec = tmp_offset_vec - act_offset_vec;

            #if ENABLED(DEBUG_LEVELING_FEATURE)
              if (DEBUGGING(LEVELING)) {
                tmp_offset_vec.debug("tmp_offset_vec");
                act_offset_vec.debug("act_offset_vec");
                offset_vec.debug("offset_vec (BEFORE)");
              }
            #endif

            offset_vec.apply_rotation(bedlevel.bed_level_matrix.transpose(bedlevel.bed_level_matrix));

            #if ENABLED(DEBUG_LEVELING_FEATURE)
              if (DEBUGGING(LEVELING)) offset_vec.debug("offset_vec (AFTER)");
            #endif

            // Adjustments to the current position
            float xydiff[2] = { offset_vec.x, offset_vec.y };
            mechanics.current_position[Z_AXIS] += offset_vec.z;

          #else // !ABL_PLANAR

            float xydiff[2] = {
              hotend_offset[X_AXIS][tmp_extruder] - hotend_offset[X_AXIS][active_extruder],
              hotend_offset[Y_AXIS][tmp_extruder] - hotend_offset[Y_AXIS][active_extruder]
            };

            #if ENABLED(MESH_BED_LEVELING)

              if (mbl.active()) {
                #if ENABLED(DEBUG_LEVELING_FEATURE)
                  if (DEBUGGING(LEVELING)) SERIAL_MV("Z before MBL: ", mechanics.current_position[Z_AXIS]);
                #endif
                float x2 = mechanics.current_position[X_AXIS] + xydiff[X_AXIS],
                      y2 = mechanics.current_position[Y_AXIS] + xydiff[Y_AXIS],
                      z1 = mechanics.current_position[Z_AXIS], z2 = z1;
                bedlevel.apply_leveling(mechanics.current_position[X_AXIS], mechanics.current_position[Y_AXIS], z1);
                bedlevel.apply_leveling(x2, y2, z2);
                mechanics.current_position[Z_AXIS] += z2 - z1;
                #if ENABLED(DEBUG_LEVELING_FEATURE)
                  if (DEBUGGING(LEVELING))
                    SERIAL_EMV(" after: ", mechanics.current_position[Z_AXIS]);
                #endif
              }

            #endif // MESH_BED_LEVELING

          #endif // !AUTO_BED_LEVELING_FEATURE

          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) {
              SERIAL_MV("Offset Tool XY by { ", xydiff[X_AXIS]);
              SERIAL_MV(", ", xydiff[Y_AXIS]);
              SERIAL_EM(" }");
            }
          #endif

          // The newly-selected extruder XY is actually at...
          mechanics.current_position[X_AXIS] += xydiff[X_AXIS];
          mechanics.current_position[Y_AXIS] += xydiff[Y_AXIS];
          #if ENABLED(WORKSPACE_OFFSETS) || ENABLED(DUAL_X_CARRIAGE)
            LOOP_XY(i) {
              mechanics.position_shift[i] += xydiff[i];
              endstops.update_software_endstops((AxisEnum)i);
            }
          #endif

          // Set the new active extruder
          previous_extruder = active_extruder;

          #if ENABLED(DONDOLO_SINGLE_MOTOR)
            active_extruder = tmp_extruder;
            active_driver = 0;
          #else
            active_extruder = active_driver = tmp_extruder;
          #endif

        #endif // !DUAL_X_CARRIAGE

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("Sync After Toolchange", mechanics.current_position);
        #endif

        // Tell the planner the new "current position"
        mechanics.sync_plan_position();

        // Move to the "old position" (move the extruder into place)
        if (!no_move && printer.IsRunning()) {
          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) DEBUG_POS("Move back", mechanics.destination);
          #endif
          mechanics.prepare_move_to_destination();
        }

      } // (tmp_extruder != active_extruder)

      stepper.synchronize();

      #if ENABLED(EXT_SOLENOID)
        disable_all_solenoids();
        enable_solenoid_on_active_extruder();
      #endif // EXT_SOLENOID

      mechanics.feedrate_mm_s = old_feedrate_mm_s;

    #endif // HOTENDS > 1

    SERIAL_LMV(ECHO, MSG_ACTIVE_DRIVER, (int)active_driver);
    SERIAL_LMV(ECHO, MSG_ACTIVE_EXTRUDER, (int)active_extruder);
  }

#endif // EXTRUDERS > 1

#if ENABLED(NPR2)

  void Printer::MK_multi_tool_change(const uint8_t &e) {

    if (e != old_color) {
      long csteps;
      stepper.synchronize(); // Finish all movement

      if (old_color == 99)
        csteps = (color_position[e]) * color_step_moltiplicator;
      else
        csteps = (color_position[e] - color_position[old_color]) * color_step_moltiplicator;

      if (csteps < 0) stepper.colorstep(-csteps, false);
      if (csteps > 0) stepper.colorstep(csteps, true);

      // Set the new active extruder
      previous_extruder = active_extruder;
      old_color = active_extruder = e;
      active_driver = 0;
      SERIAL_EMV(MSG_ACTIVE_COLOR, (int)active_extruder);
    }
  }

#elif ENABLED(MKSE6)

  void Printer::MK_multi_tool_change(const uint8_t e) {

    stepper.synchronize(); // Finish all movement

    const int angles[EXTRUDERS] = ARRAY_BY_EXTRUDERS_N (
      MKSE6_SERVOPOS_E0, MKSE6_SERVOPOS_E1,
      MKSE6_SERVOPOS_E2, MKSE6_SERVOPOS_E3,
      MKSE6_SERVOPOS_E4, MKSE6_SERVOPOS_E5
    );
    MOVE_SERVO(MKSE6_SERVO_INDEX, angles[e]);

    #if (MKSE6_SERVO_DELAY > 0)
      safe_delay(MKSE6_SERVO_DELAY);
    #endif

    // Set the new active extruder
    previous_extruder = active_extruder;
    active_extruder = e;
    active_driver = 0;
  }

#elif ENABLED(MKR4)

  void Printer::MK_multi_tool_change(const uint8_t &e) {

    stepper.synchronize(); // Finish all movement
    stepper.disable_e_steppers();

    #if (EXTRUDERS == 4) && HAS_E0E2 && HAS_E1E3 && (DRIVER_EXTRUDERS == 2)

      switch(e) {
        case 0:
          WRITE_RELE(E0E2_CHOICE_PIN, LOW);
          WRITE_RELE(E1E3_CHOICE_PIN, LOW);
          active_driver = 0;
          safe_delay(500); // 500 microseconds delay for relay
          enable_E0();
          break;
        case 1:
          WRITE_RELE(E0E2_CHOICE_PIN, LOW);
          WRITE_RELE(E1E3_CHOICE_PIN, LOW);
          active_driver = 1;
          safe_delay(500); // 500 microseconds delay for relay
          enable_E1();
          break;
        case 2:
          WRITE_RELE(E0E2_CHOICE_PIN, HIGH);
          WRITE_RELE(E1E3_CHOICE_PIN, LOW);
          active_driver = 0;
          safe_delay(500); // 500 microseconds delay for relay
          enable_E2();
          break;
        case 3:
          WRITE_RELE(E0E2_CHOICE_PIN, LOW);
          WRITE_RELE(E1E3_CHOICE_PIN, HIGH);
          active_driver = 1;
          safe_delay(500); // 500 microseconds delay for relay
          enable_E3();
          break;
      }

    #elif (EXTRUDERS == 3) && HAS_E0E2 && (DRIVER_EXTRUDERS == 2)

      switch(e) {
        case 0:
          WRITE_RELE(E0E2_CHOICE_PIN, LOW);
          active_driver = 0;
          safe_delay(500); // 500 microseconds delay for relay
          enable_E0();
          break;
        case 1:
          WRITE_RELE(E0E2_CHOICE_PIN, LOW);
          active_driver = 1;
          safe_delay(500); // 500 microseconds delay for relay
          enable_E1();
          break;
        case 2:
          WRITE_RELE(E0E2_CHOICE_PIN, HIGH);
          active_driver = 0;
          safe_delay(500); // 500 microseconds delay for relay
          enable_E0();
          break;
      }

    #elif (EXTRUDERS == 2) && HAS_E0E1 && (DRIVER_EXTRUDERS == 1)

      switch(e) {
        case 0:
          WRITE_RELE(E0E1_CHOICE_PIN, LOW);
          active_driver = 0;
          safe_delay(500); // 500 microseconds delay for relay
          enable_E0();
          break;
        case 1:
          WRITE_RELE(E0E1_CHOICE_PIN, HIGH);
          active_driver = 0;
          safe_delay(500); // 500 microseconds delay for relay
          enable_E0();
          break;
      }

    #endif // E0E1_CHOICE_PIN E0E2_CHOICE_PIN E1E3_CHOICE_PIN

    // Set the new active extruder
    previous_extruder = active_extruder;
    active_extruder = e;
  }

#elif ENABLED(MKR6) || ENABLED(MKR12)

  void Printer::MK_multi_tool_change(const uint8_t &e) {

    stepper.synchronize(); // Finish all movement
    stepper.disable_e_steppers();

    #if (EXTRUDERS == 2) && HAS_EX1 && (DRIVER_EXTRUDERS == 1)

      switch(e) {
        case 0:
          WRITE_RELE(EX1_CHOICE_PIN, LOW);
          active_driver = 0;
          safe_delay(500); // 500 microseconds delay for relay
          enable_E0();
          break;
        case 1:
          WRITE_RELE(EX1_CHOICE_PIN, HIGH);
          active_driver = 0;
          safe_delay(500); // 500 microseconds delay for relay
          enable_E0();
          break;
      }

    #elif (EXTRUDERS == 3) && HAS_EX1 && HAS_EX2 && (DRIVER_EXTRUDERS == 1)

      switch(e) {
        case 0:
          WRITE_RELE(EX1_CHOICE_PIN, LOW);
          WRITE_RELE(EX2_CHOICE_PIN, LOW);
          active_driver = 0;
          safe_delay(500); // 500 microseconds delay for relay
          enable_E0();
          break;
        case 1:
          WRITE_RELE(EX1_CHOICE_PIN, HIGH);
          WRITE_RELE(EX2_CHOICE_PIN, LOW);
          active_driver = 0;
          safe_delay(500); // 500 microseconds delay for relay
          enable_E0();
          break;
        case 2:
          WRITE_RELE(EX1_CHOICE_PIN, HIGH);
          WRITE_RELE(EX2_CHOICE_PIN, HIGH);
          active_driver = 0;
          safe_delay(500); // 500 microseconds delay for relay
          enable_E0();
          break;
      }

    #elif (EXTRUDERS > 3) && HAS_EX1 && HAS_EX2

      uint8_t multiply = e, driver;

      for (driver = 0; driver < DRIVER_EXTRUDERS; driver++) {
        if (multiply < 3) break;
        multiply -= 3;
      }

      switch(multiply) {
        case 0:
          WRITE_RELE(EX1_CHOICE_PIN, LOW);
          WRITE_RELE(EX2_CHOICE_PIN, LOW);
          active_driver = driver;
          safe_delay(500); // 500 microseconds delay for relay
          enable_E0();
          break;
        case 1:
          WRITE_RELE(EX1_CHOICE_PIN, HIGH);
          WRITE_RELE(EX2_CHOICE_PIN, LOW);
          active_driver = driver;
          safe_delay(500); // 500 microseconds delay for relay
          enable_E0();
          break;
        case 2:
          WRITE_RELE(EX1_CHOICE_PIN, HIGH);
          WRITE_RELE(EX2_CHOICE_PIN, HIGH);
          active_driver = driver;
          safe_delay(500); // 500 microseconds delay for relay
          enable_E0();
          break;
        default:
          SERIAL_LM(ER, "More Driver Extruders");
          break;
      }

    #endif

    // Set the new active extruder
    previous_extruder = active_extruder;
    active_extruder = e;
  }

#endif

#if ENABLED(CNCROUTER)

  void Printer::tool_change_cnc(uint8_t tool_id, bool wait/*=true*/, bool raise_z/*=true*/) {

    #if !ENABLED(CNCROUTER_AUTO_TOOL_CHANGE)
      unsigned long saved_speed;
      float saved_z;
    #endif

    if (tool_id != active_cnc_tool) {

      if (wait) {
        SERIAL_STR(PAUSE);
        SERIAL_EOL();
      }

      stepper.synchronize();

      #if !ENABLED(CNCROUTER_AUTO_TOOL_CHANGE)
        if (raise_z) {
          saved_speed = getCNCSpeed();
          saved_z = mechanics.current_position[Z_AXIS];
          mechanics.do_blocking_move_to_z(CNCROUTER_SAFE_Z);
        }
      #endif

      disable_cncrouter();
      safe_delay(300);

      if (wait) {
        // LCD click or M108 will clear this
        wait_for_user = true;

        KEEPALIVE_STATE(PAUSED_FOR_USER);

        #if HAS_BUZZER
          millis_t next_buzz = millis();
        #endif

        while (wait_for_user) {
          #if HAS_BUZZER
            if (millis() - next_buzz > 60000) {
              for (uint8_t i = 0; i < 3; i++) BUZZ(300, 1000);
              next_buzz = millis();
            }
          #endif
          idle(true);
        } // while (wait_for_user)
      } // if (wait)

      if (tool_id != CNC_M6_TOOL_ID) active_cnc_tool = tool_id;
      #if !ENABLED(CNCROUTER_AUTO_TOOL_CHANGE)
        else setCNCRouterSpeed(saved_speed);
        if (raise_z)
          mechanics.do_blocking_move_to_z(saved_z);
      #endif

      stepper.synchronize();

      if (wait) {
        KEEPALIVE_STATE(IN_HANDLER);

        SERIAL_STR(RESUME);
        SERIAL_EOL();
      }
    }
  }

#endif

#if HAS_SDSUPPORT

  /**
   * SD Stop & Store location
   */
  void Printer::stopSDPrint(const bool store_location) {
    if (card.cardOK && card.isFileOpen() && IS_SD_PRINTING) {
      if (store_location) SERIAL_EM("Close file and save restart.gcode");
      card.stopSDPrint(store_location);
      commands.clear_command_queue();
      quickstop_stepper();
      print_job_counter.stop();
      thermalManager.wait_for_heatup = false;
      thermalManager.disable_all_heaters();
      thermalManager.disable_all_coolers();
      #if FAN_COUNT > 0
        LOOP_FAN() fanSpeeds[f] = 0;
      #endif
      lcd_setstatus(MSG_PRINT_ABORTED, true);
      #if HAS_POWER_SWITCH
        powerManager.power_off();
      #endif
    }
  }

#endif // HAS_SDSUPPORT

#if HAS_TEMP_HOTEND

  #if DISABLED(MIN_COOLING_SLOPE_DEG)
    #define MIN_COOLING_SLOPE_DEG 1.50
  #endif
  #if DISABLED(MIN_COOLING_SLOPE_TIME)
    #define MIN_COOLING_SLOPE_TIME 60
  #endif

  void Printer::wait_heater(bool no_wait_for_cooling/*=true*/) {

    #if TEMP_RESIDENCY_TIME > 0
      millis_t residency_start_ms = 0;
      // Loop until the temperature has stabilized
      #define TEMP_CONDITIONS (!residency_start_ms || PENDING(now, residency_start_ms + (TEMP_RESIDENCY_TIME) * 1000UL))
    #else
      // Loop until the temperature is exactly on target
      #define TEMP_CONDITIONS (wants_to_cool ? thermalManager.isCoolingHotend(target_extruder) : thermalManager.isHeatingHotend(target_extruder))
    #endif

    float target_temp = -1.0, old_temp = 9999.0;
    bool wants_to_cool = false;
    thermalManager.wait_for_heatup = true;
    millis_t now, next_temp_ms = 0, next_cool_check_ms = 0;

    #if DISABLED(BUSY_WHILE_HEATING)
      KEEPALIVE_STATE(NOT_BUSY);
    #endif

    #if ENABLED(PRINTER_EVENT_LEDS)
      const float start_temp = thermalManager.degHotend(target_extruder);
      uint8_t old_blue = 0;
    #endif

    do {
      // Target temperature might be changed during the loop
      if (target_temp != thermalManager.degTargetHotend(target_extruder))
        target_temp = thermalManager.degTargetHotend(target_extruder);

      wants_to_cool = thermalManager.isCoolingHotend(target_extruder);

      // Exit if S<lower>, continue if S<higher>, R<lower>, or R<higher>
      if (no_wait_for_cooling && wants_to_cool) break;

      now = millis();
      if (ELAPSED(now, next_temp_ms)) { // Print temp & remaining time every 1s while waiting
        next_temp_ms = now + 1000UL;
        thermalManager.print_heaterstates();
        #if TEMP_RESIDENCY_TIME > 0
          SERIAL_MSG(MSG_W);
          if (residency_start_ms)
            SERIAL_VAL(long((((TEMP_RESIDENCY_TIME) * 1000UL) - (now - residency_start_ms)) / 1000UL));
          else
            SERIAL_CHR("?");
        #endif
        SERIAL_EOL();
      }

      idle();
      commands.refresh_cmd_timeout(); // to prevent stepper.stepper_inactive_time from running out

      const float temp = thermalManager.degHotend(target_extruder);

      #if ENABLED(PRINTER_EVENT_LEDS)
        // Gradually change LED strip from violet to red as nozzle heats up
        if (!wants_to_cool) {
          const uint8_t blue = map(constrain(temp, start_temp, target_temp), start_temp, target_temp, 255, 0);
          if (blue != old_blue) set_led_color(255, 0, (old_blue = blue));
        }
      #endif

      #if TEMP_RESIDENCY_TIME > 0

        float temp_diff = FABS(target_temp - temp);

        if (!residency_start_ms) {
          // Start the TEMP_RESIDENCY_TIME timer when we reach target temp for the first time.
          if (temp_diff < TEMP_WINDOW) residency_start_ms = now;
        }
        else if (temp_diff > TEMP_HYSTERESIS) {
          // Restart the timer whenever the temperature falls outside the hysteresis.
          residency_start_ms = now;
        }

      #endif

      // Prevent a wait-forever situation if R is misused i.e. M109 R0
      if (wants_to_cool) {
        // Break after MIN_COOLING_SLOPE_TIME seconds
        // if the temperature did not drop at least MIN_COOLING_SLOPE_DEG
        if (!next_cool_check_ms || ELAPSED(now, next_cool_check_ms)) {
          if (old_temp - temp < MIN_COOLING_SLOPE_DEG) break;
          next_cool_check_ms = now + 1000UL * MIN_COOLING_SLOPE_TIME;
          old_temp = temp;
        }
      }

    } while (thermalManager.wait_for_heatup && TEMP_CONDITIONS);

    if (thermalManager.wait_for_heatup) {
      LCD_MESSAGEPGM(MSG_HEATING_COMPLETE);
      #if ENABLED(PRINTER_EVENT_LEDS)
        #if ENABLED(RGBW_LED)
          set_led_color(0, 0, 0, 255);  // Turn on the WHITE LED
        #else
          set_led_color(255, 255, 255); // Set LEDs All On
        #endif
      #endif
    }

    #if DISABLED(BUSY_WHILE_HEATING)
      KEEPALIVE_STATE(IN_HANDLER);
    #endif
  }

#endif

#if HAS_TEMP_BED

  #if DISABLED(MIN_COOLING_SLOPE_DEG_BED)
    #define MIN_COOLING_SLOPE_DEG_BED 1.50
  #endif
  #if DISABLED(MIN_COOLING_SLOPE_TIME_BED)
    #define MIN_COOLING_SLOPE_TIME_BED 60
  #endif

  void Printer::wait_bed(bool no_wait_for_cooling/*=true*/) {

    #if TEMP_BED_RESIDENCY_TIME > 0
      millis_t residency_start_ms = 0;
      // Loop until the temperature has stabilized
      #define TEMP_BED_CONDITIONS (!residency_start_ms || PENDING(now, residency_start_ms + (TEMP_BED_RESIDENCY_TIME) * 1000UL))
    #else
      // Loop until the temperature is very close target
      #define TEMP_BED_CONDITIONS (wants_to_cool ? thermalManager.isCoolingBed() : thermalManager.isHeatingBed())
    #endif

    float target_temp = -1.0, old_temp = 9999.0;
    bool wants_to_cool = false;
    thermalManager.wait_for_heatup = true;
    millis_t now, next_temp_ms = 0, next_cool_check_ms = 0;

    #if DISABLED(BUSY_WHILE_HEATING)
      KEEPALIVE_STATE(NOT_BUSY);
    #endif

    target_extruder = active_extruder; // for print_heaterstates

    #if ENABLED(PRINTER_EVENT_LEDS)
      const float start_temp = thermalManager.degBed();
      uint8_t old_red = 255;
    #endif

    // Wait for temperature to come close enough
    do {
      // Target temperature might be changed during the loop
      if (target_temp != thermalManager.degTargetBed())
        target_temp = thermalManager.degTargetBed();

      wants_to_cool = thermalManager.isCoolingBed();

      // Exit if S<lower>, continue if S<higher>, R<lower>, or R<higher>
      if (no_wait_for_cooling && wants_to_cool) break;

      now = millis();
      if (ELAPSED(now, next_temp_ms)) { // Print Temp Reading every 1 second while heating up.
        next_temp_ms = now + 1000UL;
        thermalManager.print_heaterstates();
        #if TEMP_BED_RESIDENCY_TIME > 0
          SERIAL_MSG(MSG_W);
          if (residency_start_ms)
            SERIAL_VAL(long((((TEMP_BED_RESIDENCY_TIME) * 1000UL) - (now - residency_start_ms)) / 1000UL));
          else
            SERIAL_CHR("?");
        #endif
        SERIAL_EOL();
      }

      idle();
      commands.refresh_cmd_timeout(); // to prevent stepper.stepper_inactive_time from running out

      const float temp = thermalManager.degBed();

      #if ENABLED(PRINTER_EVENT_LEDS)
        // Gradually change LED strip from blue to violet as bed heats up
        if (!wants_to_cool) {
          const uint8_t red = map(constrain(temp, start_temp, target_temp), start_temp, target_temp, 0, 255);
          if (red != old_red) set_led_color((old_red = red), 0, 255);
        }
      #endif

      #if TEMP_BED_RESIDENCY_TIME > 0

        float temp_diff = FABS(target_temp - temp);

        if (!residency_start_ms) {
          // Start the TEMP_BED_RESIDENCY_TIME timer when we reach target temp for the first time.
          if (temp_diff < TEMP_BED_WINDOW) residency_start_ms = now;
        }
        else if (temp_diff > TEMP_BED_HYSTERESIS) {
          // Restart the timer whenever the temperature falls outside the hysteresis.
          residency_start_ms = now;
        }

      #endif

      // Prevent a wait-forever situation if R is misused i.e. M190 R0
      if (wants_to_cool) {
        // Break after MIN_COOLING_SLOPE_TIME_BED seconds
        // if the temperature did not drop at least MIN_COOLING_SLOPE_DEG_BED
        if (!next_cool_check_ms || ELAPSED(now, next_cool_check_ms)) {
          if (old_temp - temp < MIN_COOLING_SLOPE_DEG_BED) break;
          next_cool_check_ms = now + 1000UL * MIN_COOLING_SLOPE_TIME_BED;
          old_temp = temp;
        }
      }

    } while (thermalManager.wait_for_heatup && TEMP_BED_CONDITIONS);

    if (thermalManager.wait_for_heatup) LCD_MESSAGEPGM(MSG_BED_DONE);

    #if DISABLED(BUSY_WHILE_HEATING)
      KEEPALIVE_STATE(IN_HANDLER);
    #endif
  }

#endif // HAS_TEMP_BED

#if HAS_TEMP_CHAMBER

  void Printer::wait_chamber(bool no_wait_for_heating/*=true*/) {
    #if TEMP_CHAMBER_RESIDENCY_TIME > 0
      millis_t residency_start_ms = 0;
      // Loop until the temperature has stabilized
      #define TEMP_CHAMBER_CONDITIONS (!residency_start_ms || PENDING(now, residency_start_ms + (TEMP_CHAMBER_RESIDENCY_TIME) * 1000UL))
    #else
      // Loop until the temperature is very close target
      #define TEMP_CHAMBER_CONDITIONS (wants_to_heat ? thermalManager.isHeatingChamber() : thermalManager.isCoolingChamber())
    #endif

    float target_temp = -1;
    bool wants_to_heat;
    thermalManager.wait_for_heatup = true;
    millis_t now, next_temp_ms = 0;

    KEEPALIVE_STATE(WAIT_HEATER);

    // Wait for temperature to come close enough
    do {
      // Target temperature might be changed during the loop
      if (target_temp != thermalManager.degTargetChamber())
        target_temp = thermalManager.degTargetChamber();

      wants_to_heat = thermalManager.isHeatingChamber();

      // Exit if S<higher>, continue if S<lower>, R<higher>, or R<lower>
      if (no_wait_for_heating && wants_to_heat) break;

      now = millis();
      if (ELAPSED(now, next_temp_ms)) { // Print Temp Reading every 1 second while heating up.
        next_temp_ms = now + 1000UL;
        print_chamberstate();
        #if TEMP_CHAMBER_RESIDENCY_TIME > 0
          SERIAL_MSG(MSG_W);
          if (residency_start_ms) {
            long rem = (((TEMP_CHAMBER_RESIDENCY_TIME) * 1000UL) - (now - residency_start_ms)) / 1000UL;
            SERIAL_EV(rem);
          }
          else {
            SERIAL_EM("?");
          }
        #else
          SERIAL_EOL();
        #endif
      }

      idle();
      commands.refresh_cmd_timeout(); // to prevent stepper.stepper_inactive_time from running out

      #if TEMP_CHAMBER_RESIDENCY_TIME > 0

        float temp_diff = FABS(target_temp - thermalManager.degTargetChamber());

        if (!residency_start_ms) {
          // Start the TEMP_CHAMBER_RESIDENCY_TIME timer when we reach target temp for the first time.
          if (temp_diff < TEMP_CHAMBER_WINDOW) residency_start_ms = now;
        }
        else if (temp_diff > TEMP_CHAMBER_HYSTERESIS) {
          // Restart the timer whenever the temperature falls outside the hysteresis.
          residency_start_ms = now;
        }

      #endif //TEMP_CHAMBER_RESIDENCY_TIME > 0

    } while (thermalManager.wait_for_heatup && TEMP_CHAMBER_CONDITIONS);
    LCD_MESSAGEPGM(MSG_CHAMBER_DONE);
    KEEPALIVE_STATE(IN_HANDLER);
  }

#endif

#if HAS_TEMP_COOLER

  void Printer::wait_cooler(bool no_wait_for_heating/*=true*/) {
    #if TEMP_COOLER_RESIDENCY_TIME > 0
      millis_t residency_start_ms = 0;
      // Loop until the temperature has stabilized
      #define TEMP_COOLER_CONDITIONS (!residency_start_ms || PENDING(now, residency_start_ms + (TEMP_COOLER_RESIDENCY_TIME) * 1000UL))
    #else
      // Loop until the temperature is very close target
      #define TEMP_COOLER_CONDITIONS (wants_to_heat ? thermalManager.isHeatingCooler() : thermalManager.isCoolingCooler())
    #endif

    float target_temp = -1;
    bool wants_to_heat;
    thermalManager.wait_for_heatup = true;
    millis_t now, next_temp_ms = 0;

    KEEPALIVE_STATE(WAIT_HEATER);

    // Wait for temperature to come close enough
    do {
      // Target temperature might be changed during the loop
      if (target_temp != thermalManager.degTargetCooler())
        target_temp = thermalManager.degTargetCooler();

      wants_to_heat = thermalManager.isHeatingCooler();

      // Exit if S<higher>, continue if S<lower>, R<higher>, or R<lower>
      if (no_wait_for_heating && wants_to_heat) break;

      // Prevent a wait-forever situation if R is misused i.e. M190 C R50
      // Simply don't wait to heat a cooler over 25C
      if (wants_to_heat && target_temp > 25) break;

      now = millis();
      if (ELAPSED(now, next_temp_ms)) { //Print Temp Reading every 1 second while heating up.
        next_temp_ms = now + 1000UL;
        print_coolerstate();
        #if TEMP_COOLER_RESIDENCY_TIME > 0
          SERIAL_MSG(MSG_W);
          if (residency_start_ms) {
            long rem = (((TEMP_COOLER_RESIDENCY_TIME) * 1000UL) - (now - residency_start_ms)) / 1000UL;
            SERIAL_EV(rem);
          }
          else {
            SERIAL_EM("?");
          }
        #else
          SERIAL_EOL();
        #endif
      }

      idle();
      commands.refresh_cmd_timeout(); // to prevent stepper.stepper_inactive_time from running out

      #if TEMP_COOLER_RESIDENCY_TIME > 0

        float temp_diff = FABS(target_temp - thermalManager.degTargetCooler());

        if (!residency_start_ms) {
          // Start the TEMP_COOLER_RESIDENCY_TIME timer when we reach target temp for the first time.
          if (temp_diff < TEMP_COOLER_WINDOW) residency_start_ms = now;
        }
        else if (temp_diff > TEMP_COOLER_HYSTERESIS) {
          // Restart the timer whenever the temperature falls outside the hysteresis.
          residency_start_ms = now;
        }

      #endif //TEMP_COOLER_RESIDENCY_TIME > 0

    } while (thermalManager.wait_for_heatup && TEMP_COOLER_CONDITIONS);
    LCD_MESSAGEPGM(MSG_COOLER_DONE);
    KEEPALIVE_STATE(IN_HANDLER);
  }

#endif

#if ENABLED(ADVANCED_PAUSE_FEATURE)

  #if HAS_BUZZER

    void Printer::filament_change_beep(const int8_t max_beep_count, const bool init/*=false*/) {
      static millis_t next_buzz = 0;
      static int8_t runout_beep = 0;

      if (init) next_buzz = runout_beep = 0;

      const millis_t ms = millis();
      if (ELAPSED(ms, next_buzz)) {
        if (max_beep_count < 0 || runout_beep < max_beep_count + 5) { // Only beep as long as we're supposed to
          next_buzz = ms + ((max_beep_count < 0 || runout_beep < max_beep_count) ? 2500 : 400);
          BUZZ(300, 2000);
          runout_beep++;
        }
      }
    }

  #endif

  void Printer::ensure_safe_temperature() {
    bool heaters_heating = true;

    thermalManager.wait_for_heatup = true;
    while (thermalManager.wait_for_heatup && heaters_heating) {
      idle();
      heaters_heating = false;
      LOOP_HOTEND() {
        if (thermalManager.degTargetHotend(h) && abs(thermalManager.degHotend(h) - thermalManager.degTargetHotend(h)) > TEMP_HYSTERESIS) {
          heaters_heating = true;
          #if HAS_LCD
            lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_WAIT_FOR_NOZZLES_TO_HEAT);
          #endif
          break;
        }
      }
    }
  }

  bool Printer::pause_print(const float &retract, const float &retract2, const float &z_lift, const float &x_pos, const float &y_pos,
                          const float &unload_length/*=0*/, const int8_t max_beep_count/*=0*/, const bool show_lcd/*=false*/) {

    if (move_away_flag) return false; // already paused

    if (!DEBUGGING(DRYRUN) && (unload_length != 0 || retract != 0 || retract2 != 0)) {
      #if ENABLED(PREVENT_COLD_EXTRUSION)
        if (thermalManager.tooColdToExtrude(active_extruder)) {
          SERIAL_LM(ER, MSG_TOO_COLD_FOR_M600);
          return false;
        }
      #endif

      ensure_safe_temperature(); // wait for hotend to heat up before unloading
    }

    // Indicate that the printer is paused
    move_away_flag = true;

    // Pause the print job and timer
    #if HAS_SDSUPPORT
      if (card.sdprinting) {
        card.pauseSDPrint();
        sd_print_paused = true;
      }
    #endif
    print_job_counter.pause();

    // Show initial message and wait for synchronize steppers
    if (show_lcd) {
      #if HAS_LCD
        lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_INIT);
      #endif
    }
    stepper.synchronize();

    // Save current position
    COPY_ARRAY(resume_position, mechanics.current_position);
    mechanics.set_destination_to_current();

    if (retract) {
      // Initial retract before move to filament change position
      mechanics.destination[E_AXIS] += retract;
      RUNPLAN(PAUSE_PARK_RETRACT_FEEDRATE);
    }

    // Lift Z axis
    if (z_lift > 0) {
      mechanics.destination[Z_AXIS] += z_lift;
      NOMORE(mechanics.destination[Z_AXIS], Z_MAX_POS);
      RUNPLAN(PAUSE_PARK_Z_FEEDRATE);
    }

    // Move XY axes to filament exchange position
    mechanics.destination[X_AXIS] = x_pos;
    mechanics.destination[Y_AXIS] = y_pos;

    endstops.clamp_to_software_endstops(mechanics.destination);
    RUNPLAN(PAUSE_PARK_XY_FEEDRATE);
    stepper.synchronize();

    // Store in old temperature the target temperature for hotend and bed
    int16_t old_target_temperature[HOTENDS];
    LOOP_HOTEND() old_target_temperature[h] = thermalManager.target_temperature[h]; // Save nozzle temps

    // Second retract filament with Cool Down
    if (retract2) {
      // Cool Down hotend
      #if ENABLED(PAUSE_PARK_COOLDOWN_TEMP) && PAUSE_PARK_COOLDOWN_TEMP > 0
        lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_COOLDOWN);
        thermalManager.setTargetHotend(PAUSE_PARK_COOLDOWN_TEMP, active_extruder);
        wait_heater(false);
      #endif

      // Second retract filament
      mechanics.destination[E_AXIS] -= retract2;
      RUNPLAN(PAUSE_PARK_RETRACT_2_FEEDRATE);
      stepper.synchronize();
    }

    if (unload_length != 0) {
      if (show_lcd) {
        #if HAS_LCD
          lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_UNLOAD);
          idle();
        #endif
      }

      // Unload filament
      mechanics.destination[E_AXIS] += unload_length;
      RUNPLAN(PAUSE_PARK_UNLOAD_FEEDRATE);
      stepper.synchronize();
    }

    if (show_lcd) {
      #if HAS_LCD
        lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_INSERT);
      #endif
    }

    #if HAS_BUZZER
      filament_change_beep(max_beep_count, true);
    #else
      UNUSED(max_beep_count);
    #endif

    idle();

    // Disable extruders steppers for manual filament changing (only on boards that have separate ENABLE_PINS)
    #if E0_ENABLE_PIN != X_ENABLE_PIN && E1_ENABLE_PIN != Y_ENABLE_PIN
      stepper.disable_e_steppers();
      safe_delay(100);
    #endif

    // Start the heater idle timers
    const millis_t nozzle_timeout = (millis_t)(PAUSE_PARK_NOZZLE_TIMEOUT) * 1000UL;
    const millis_t bed_timeout    = (millis_t)(PAUSE_PARK_PRINTER_OFF) * 60000UL;

    LOOP_HOTEND() {
      thermalManager.start_heater_idle_timer(h, nozzle_timeout);
      thermalManager.setTargetHotend(old_target_temperature[h], h);
    }

    #if HAS_TEMP_BED && PAUSE_PARK_PRINTER_OFF > 0
      thermalManager.start_bed_idle_timer(bed_timeout);
    #endif

    return true;
  }

  void Printer::wait_for_filament_reload(const int8_t max_beep_count/*=0*/) {
    bool nozzle_timed_out = false,
         bed_timed_out = false;

    // Wait for filament insert by user and press button
    KEEPALIVE_STATE(PAUSED_FOR_USER);
    wait_for_user = true;    // LCD click or M108 will clear this
    while (wait_for_user) {

      #if HAS_BUZZER
        filament_change_beep(max_beep_count);
      #else
        UNUSED(max_beep_count);
      #endif

      if (!nozzle_timed_out)
        LOOP_HOTEND()
          nozzle_timed_out |= thermalManager.is_heater_idle(h);

      if (nozzle_timed_out) {

        #if HAS_LCD
          lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_CLICK_TO_HEAT_NOZZLE);
        #endif

        // Wait for LCD click or M108
        while (wait_for_user) {

          if (!bed_timed_out) {
            #if HAS_TEMP_BED && PAUSE_PARK_PRINTER_OFF > 0
              bed_timed_out = thermalManager.is_bed_idle();
            #endif
          }
          else {
            #if HAS_LCD
              lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_PRINTER_OFF);
            #endif
          }

          idle(true);
        }

        // Re-enable the bed if they timed out
        #if HAS_TEMP_BED && PAUSE_PARK_PRINTER_OFF > 0
          if (bed_timed_out) {
            thermalManager.reset_bed_idle_timer();
            #if HAS_LCD
              lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_WAIT_FOR_NOZZLES_TO_HEAT);
            #endif
            wait_bed();
          }
        #endif

        // Re-enable the heaters if they timed out
        LOOP_HOTEND() thermalManager.reset_heater_idle_timer(h);

        // Wait for the heaters to reach the target temperatures
        ensure_safe_temperature();

        #if HAS_LCD
          lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_INSERT);
        #endif

        // Start the heater idle timers
        const millis_t nozzle_timeout = (millis_t)(PAUSE_PARK_NOZZLE_TIMEOUT) * 1000UL;
        const millis_t bed_timeout    = (millis_t)(PAUSE_PARK_PRINTER_OFF) * 60000UL;

        LOOP_HOTEND()
          thermalManager.start_heater_idle_timer(h, nozzle_timeout);

        #if HAS_TEMP_BED && PAUSE_PARK_PRINTER_OFF > 0
          thermalManager.start_bed_idle_timer(bed_timeout);
        #endif

        wait_for_user = true; /* Wait for user to load filament */
        nozzle_timed_out = false;
        bed_timed_out = false;

        #if HAS_BUZZER
          filament_change_beep(max_beep_count, true);
        #endif
      }

      idle(true);
    }

    KEEPALIVE_STATE(IN_HANDLER);
  }

  void Printer::resume_print(const float &load_length/*=0*/, const float &initial_extrude_length/*=0*/, const int8_t max_beep_count/*=0*/) {
    bool  nozzle_timed_out  = false,
          bed_timed_out     = false;

    if (!move_away_flag) return;

    // Re-enable the heaters if they timed out
    #if HAS_TEMP_BED && PAUSE_PARK_PRINTER_OFF > 0
      bed_timed_out = thermalManager.is_bed_idle();
      thermalManager.reset_bed_idle_timer();
      if (bed_timed_out) wait_bed();
    #endif

    LOOP_HOTEND() {
      nozzle_timed_out |= thermalManager.is_heater_idle(h);
      thermalManager.reset_heater_idle_timer(h);
    }

    if (nozzle_timed_out) ensure_safe_temperature();

    #if HAS_BUZZER
      filament_change_beep(max_beep_count, true);
    #else
      UNUSED(max_beep_count);
    #endif

    if (load_length != 0) {
      #if HAS_LCD
        // Show "insert filament"
        if (nozzle_timed_out)
          lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_INSERT);
      #endif

      KEEPALIVE_STATE(PAUSED_FOR_USER);
      wait_for_user = true;    // LCD click or M108 will clear this
      while (wait_for_user && nozzle_timed_out) {
        #if HAS_BUZZER
          filament_change_beep(max_beep_count);
        #endif
        idle(true);
      }
      KEEPALIVE_STATE(IN_HANDLER);

      #if HAS_LCD
        // Show "load" message
        lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_LOAD);
      #endif

      // Load filament
      mechanics.destination[E_AXIS] += load_length;

      RUNPLAN(PAUSE_PARK_LOAD_FEEDRATE);
      stepper.synchronize();
    }

    #if HAS_LCD && ENABLED(PAUSE_PARK_EXTRUDE_LENGTH) && PAUSE_PARK_EXTRUDE_LENGTH > 0

      float extrude_length = initial_extrude_length;

      do {
        if (extrude_length > 0) {
          // "Wait for filament extrude"
          lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_EXTRUDE);

          // Extrude filament to get into hotend
          mechanics.destination[E_AXIS] += extrude_length;
          RUNPLAN(PAUSE_PARK_EXTRUDE_FEEDRATE);
          stepper.synchronize();
        }

        // Show "Extrude More" / "Resume" menu and wait for reply
        KEEPALIVE_STATE(PAUSED_FOR_USER);
        wait_for_user = false;
        lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_OPTION);
        while (advanced_pause_menu_response == ADVANCED_PAUSE_RESPONSE_WAIT_FOR) idle(true);
        KEEPALIVE_STATE(IN_HANDLER);

        extrude_length = PAUSE_PARK_EXTRUDE_LENGTH;

        // Keep looping if "Extrude More" was selected
      } while (advanced_pause_menu_response == ADVANCED_PAUSE_RESPONSE_EXTRUDE_MORE);

    #endif

    #if HAS_LCD
      // "Wait for print to resume"
      lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_RESUME);
    #endif

    // Set extruder to saved position
    mechanics.destination[E_AXIS] = mechanics.current_position[E_AXIS] = resume_position[E_AXIS];
    mechanics.set_e_position_mm(mechanics.current_position[E_AXIS]);

    #if IS_KINEMATIC
      // Move XYZ to starting position
      planner.buffer_line_kinematic(resume_position, PAUSE_PARK_XY_FEEDRATE, active_extruder);
    #else
      // Move XY to starting position, then Z
      mechanics.destination[X_AXIS] = resume_position[X_AXIS];
      mechanics.destination[Y_AXIS] = resume_position[Y_AXIS];
      RUNPLAN(PAUSE_PARK_XY_FEEDRATE);
      mechanics.destination[Z_AXIS] = resume_position[Z_AXIS];
      RUNPLAN(PAUSE_PARK_Z_FEEDRATE);
    #endif
    stepper.synchronize();

    #if ENABLED(FILAMENT_RUNOUT_SENSOR)
      filament_ran_out = false;
    #endif

    mechanics.set_current_to_destination();

    #if HAS_LCD
      // Show status screen
      lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_STATUS);
    #endif

    #if HAS_SDSUPPORT
      if (sd_print_paused) {
        card.startFileprint();
        sd_print_paused = false;
      }
    #endif

    move_away_flag = false;
  }

#endif // ADVANCED_PAUSE_FEATURE

#if HAS_CASE_LIGHT

  void Printer::update_case_light() {
    pinMode(CASE_LIGHT_PIN, OUTPUT);
    uint8_t case_light_bright = (uint8_t)case_light_brightness;
    if (case_light_on) {
      HAL::analogWrite(CASE_LIGHT_PIN, INVERT_CASE_LIGHT ? 255 - case_light_brightness : case_light_brightness );
      WRITE(CASE_LIGHT_PIN, INVERT_CASE_LIGHT ? LOW : HIGH);
    }
    else WRITE(CASE_LIGHT_PIN, INVERT_CASE_LIGHT ? HIGH : LOW);
  }
  
#endif

#if HAS_COLOR_LEDS

  void Printer::set_led_color(
    const uint8_t r, const uint8_t g, const uint8_t b
      #if ENABLED(RGBW_LED)
        , const uint8_t w/*=0*/
      #endif
  ) {

    #if ENABLED(BLINKM)

      // This variant uses i2c to send the RGB components to the device.
      SendColors(r, g, b);

    #endif

    #if ENABLED(RGB_LED) || ENABLED(RGBW_LED)

      // This variant uses 3 separate pins for the RGB components.
      // If the pins can do PWM then their intensity will be set.
      WRITE(RGB_LED_R_PIN, r ? HIGH : LOW);
      WRITE(RGB_LED_G_PIN, g ? HIGH : LOW);
      WRITE(RGB_LED_B_PIN, b ? HIGH : LOW);
      analogWrite(RGB_LED_R_PIN, r);
      analogWrite(RGB_LED_G_PIN, g);
      analogWrite(RGB_LED_B_PIN, b);

      #if ENABLED(RGBW_LED)
        WRITE(RGB_LED_W_PIN, w ? HIGH : LOW);
        analogWrite(RGB_LED_W_PIN, w);
      #endif

    #endif

    #if ENABLED(PCA9632)
      // Update I2C LED driver
      PCA9632_SetColor(r, g, b);
    #endif

  }

#endif // HAS_COLOR_LEDS

/**
 * Sensitive pin test for M42, M226
 */
bool Printer::pin_is_protected(uint8_t pin) {
  static const int8_t sensitive_pins[] PROGMEM = SENSITIVE_PINS;
  for (uint8_t i = 0; i < COUNT(sensitive_pins); i++)
    if (pin == (int8_t)pgm_read_byte(&sensitive_pins[i])) return true;
  return false;
}

void Printer::suicide() {
  #if HAS_SUICIDE
    OUT_WRITE(SUICIDE_PIN, LOW);
  #endif
}

/**
 * Private Function
 */
#if HAS_FIL_RUNOUT
  void Printer::setup_filrunoutpin() {
    #if ENABLED(ENDSTOPPULLUP_FIL_RUNOUT)
      SET_INPUT_PULLUP(FIL_RUNOUT_PIN);
    #else
      SET_INPUT(FIL_RUNOUT_PIN);
    #endif
  }
#endif

void Printer::setup_powerhold() {
  #if HAS_SUICIDE
    OUT_WRITE(SUICIDE_PIN, HIGH);
  #endif
  #if HAS_POWER_SWITCH
    #if ENABLED(PS_DEFAULT_OFF)
      powerManager.power_off();
    #else
      powerManager.power_on();
    #endif
  #endif
}

/**
 * Stepper Reset (RigidBoard, et.al.)
 */
#if HAS_STEPPER_RESET
  void Printer::disableStepperDrivers() {
    OUT_WRITE(STEPPER_RESET_PIN, LOW);  // drive it down to hold in reset motor driver chips
  }
  void Printer::enableStepperDrivers() { SET_INPUT(STEPPER_RESET_PIN); }  // set to input, which allows it to be pulled high by pullups
#endif

#if HAS_CONTROLLERFAN

  void Printer::controllerFan() {
    static millis_t lastMotorOn = 0,    // Last time a motor was turned on
                    nextMotorCheck = 0; // Last time the state was checked
    millis_t ms = millis();
    if (ELAPSED(ms, nextMotorCheck)) {
      nextMotorCheck = ms + 2500UL; // Not a time critical function, so only check every 2.5s
      if (X_ENABLE_READ == X_ENABLE_ON || Y_ENABLE_READ == Y_ENABLE_ON || Z_ENABLE_READ == Z_ENABLE_ON
        || E0_ENABLE_READ == E_ENABLE_ON // If any of the drivers are enabled...
        #if EXTRUDERS > 1
          || E1_ENABLE_READ == E_ENABLE_ON
          #if HAS_X2_ENABLE
            || X2_ENABLE_READ == X_ENABLE_ON
          #endif
          #if EXTRUDERS > 2
            || E2_ENABLE_READ == E_ENABLE_ON
            #if EXTRUDERS > 3
              || E3_ENABLE_READ == E_ENABLE_ON
              #if EXTRUDERS > 4
                || E4_ENABLE_READ == E_ENABLE_ON
                #if EXTRUDERS > 5
                  || E5_ENABLE_READ == E_ENABLE_ON
                #endif
              #endif
            #endif
          #endif
        #endif
      ) {
        lastMotorOn = ms; //... set time to NOW so the fan will turn on
      }

      // Fan off if no steppers have been enabled for CONTROLLERFAN_SECS seconds
      controller_fanSpeeds = (!lastMotorOn || ELAPSED(ms, lastMotorOn + (CONTROLLERFAN_SECS) * 1000UL)) ? 0 : CONTROLLERFAN_SPEED;
    }
  }

#endif // HAS_CONTROLLERFAN

float Printer::calculate_volumetric_multiplier(const float diameter) {
  if (!volumetric_enabled || diameter == 0) return 1.0;
  float d2 = diameter * 0.5;
  return 1.0 / (M_PI * d2 * d2);
}

void Printer::invalid_extruder_error(const uint8_t e) {
  SERIAL_SMV(ER, "T", (int)e);
  SERIAL_EM(" " MSG_INVALID_EXTRUDER);
}

#if ENABLED(IDLE_OOZING_PREVENT)

  void Printer::IDLE_OOZING_retract(bool retracting) {
    if (retracting && !IDLE_OOZING_retracted[active_extruder]) {
      float old_feedrate_mm_s = mechanics.feedrate_mm_s;
      mechanics.set_destination_to_current();
      mechanics.current_position[E_AXIS] += IDLE_OOZING_LENGTH / volumetric_multiplier[active_extruder];
      mechanics.feedrate_mm_s = IDLE_OOZING_FEEDRATE;
      mechanics.set_e_position_mm(mechanics.current_position[E_AXIS]);
      mechanics.prepare_move_to_destination();
      mechanics.feedrate_mm_s = old_feedrate_mm_s;
      IDLE_OOZING_retracted[active_extruder] = true;
      //SERIAL_EM("-");
    }
    else if (!retracting && IDLE_OOZING_retracted[active_extruder]) {
      float old_feedrate_mm_s = mechanics.feedrate_mm_s;
      mechanics.set_destination_to_current();
      mechanics.current_position[E_AXIS] -= (IDLE_OOZING_LENGTH+IDLE_OOZING_RECOVER_LENGTH) / volumetric_multiplier[active_extruder];
      mechanics.feedrate_mm_s = IDLE_OOZING_RECOVER_FEEDRATE;
      mechanics.set_e_position_mm(mechanics.current_position[E_AXIS]);
      mechanics.prepare_move_to_destination();
      mechanics.feedrate_mm_s = old_feedrate_mm_s;
      IDLE_OOZING_retracted[active_extruder] = false;
      //SERIAL_EM("+");
    }
  }

#endif

#if ENABLED(HOST_KEEPALIVE_FEATURE)

  MK4duoBusyState busy_state = NOT_BUSY;
  static millis_t next_busy_signal_ms = 0;

  /**
   * Output a "busy" message at regular intervals
   * while the machine is not accepting 
   */
  void Printer::host_keepalive() {
    const millis_t now = millis();
    if (host_keepalive_interval && busy_state != NOT_BUSY) {
      if (PENDING(now, next_busy_signal_ms)) return;
      switch (busy_state) {
        case IN_HANDLER:
        case IN_PROCESS:
          SERIAL_LM(BUSY, MSG_BUSY_PROCESSING);
          break;
        case WAIT_HEATER:
          SERIAL_LM(BUSY, MSG_BUSY_WAIT_HEATER);
          break;
        case DOOR_OPEN:
          SERIAL_LM(BUSY, MSG_BUSY_DOOR_OPEN);
          break;
        case PAUSED_FOR_USER:
          SERIAL_LM(BUSY, MSG_BUSY_PAUSED_FOR_USER);
          break;
        case PAUSED_FOR_INPUT:
          SERIAL_LM(BUSY, MSG_BUSY_PAUSED_FOR_INPUT);
          break;
        default:
          break;
      }
    }
    next_busy_signal_ms = now + host_keepalive_interval * 1000UL;
  }

#endif // HOST_KEEPALIVE_FEATURE

#if ENABLED(TEMP_STAT_LEDS)

  void Printer::handle_status_leds() {

    static bool red_led = false;
    static millis_t next_status_led_update_ms = 0;

    if (ELAPSED(millis(), next_status_led_update_ms)) {
      next_status_led_update_ms += 500; // Update every 0.5s
      float max_temp = 0.0;
        #if HAS_TEMP_BED
          max_temp = MAX3(max_temp, thermalManager.degTargetBed(), thermalManager.degBed());
        #endif
      LOOP_HOTEND()
        max_temp = MAX3(max_temp, thermalManager.degHotend(h), thermalManager.degTargetHotend(h));
      const bool new_led = (max_temp > 55.0) ? true : (max_temp < 54.0) ? false : red_led;
      if (new_led != red_led) {
        red_led = new_led;
        #if PIN_EXISTS(STAT_LED_RED)
          WRITE(STAT_LED_RED_PIN, new_led ? HIGH : LOW);
          #if PIN_EXISTS(STAT_LED_BLUE)
            WRITE(STAT_LED_BLUE_PIN, new_led ? LOW : HIGH);
          #endif
        #else
          WRITE(STAT_LED_BLUE_PIN, new_led ? HIGH : LOW);
        #endif

      }
    }
  }

#endif

#if ENABLED(HAVE_TMC2130)

  void automatic_current_control(TMC2130Stepper &st, String axisID) {
    // Check otpw even if we don't use automatic control. Allows for flag inspection.
    const bool is_otpw = st.checkOT();

    // Report if a warning was triggered
    static bool previous_otpw = false;
    if (is_otpw && !previous_otpw) {
      char timestamp[10];
      duration_t elapsed = printer.print_job_counter.duration();
      const bool has_days = (elapsed.value > 60*60*24L);
      (void)elapsed.toDigital(timestamp, has_days);
      SERIAL_TXT(timestamp);
      SERIAL_TXT(": ");
      SERIAL_TXT(axisID);
      SERIAL_EM(" driver overtemperature warning!");
    }
    previous_otpw = is_otpw;

    #if CURRENT_STEP > 0 && ENABLED(AUTOMATIC_CURRENT_CONTROL)
      // Return if user has not enabled current control start with M906 S1.
      if (!auto_current_control) return;

      /**
       * Decrease current if is_otpw is true.
       * Bail out if driver is disabled.
       * Increase current if OTPW has not been triggered yet.
       */
      uint16_t current = st.getCurrent();
      if (is_otpw) {
        st.setCurrent(current - CURRENT_STEP, R_SENSE, HOLD_MULTIPLIER);
        #if ENABLED(REPORT_CURRENT_CHANGE)
          SERIAL_TXT(axisID);
          SERIAL_MV(" current decreased to ", st.getCurrent());
        #endif
      }

      else if (!st.isEnabled())
        return;

      else if (!is_otpw && !st.getOTPW()) {
        current += CURRENT_STEP;
        if (current <= AUTO_ADJUST_MAX) {
          st.setCurrent(current, R_SENSE, HOLD_MULTIPLIER);
          #if ENABLED(REPORT_CURRENT_CHANGE)
            SERIAL_TXT(axisID);
            SERIAL_MV(" current increased to ", st.getCurrent());
          #endif
        }
      }
      SERIAL_EOL();
    #endif
  }

  void Printer::checkOverTemp() {

    static millis_t next_cOT = 0;
    if (ELAPSED(millis(), next_cOT)) {
      next_cOT = millis() + 5000;
      #if ENABLED(X_IS_TMC2130)
        automatic_current_control(stepperX, "X");
      #endif
      #if ENABLED(Y_IS_TMC2130)
        automatic_current_control(stepperY, "Y");
      #endif
      #if ENABLED(Z_IS_TMC2130)
        automatic_current_control(stepperZ, "Z");
      #endif
      #if ENABLED(X2_IS_TMC2130)
        automatic_current_control(stepperX2, "X2");
      #endif
      #if ENABLED(Y2_IS_TMC2130)
        automatic_current_control(stepperY2, "Y2");
      #endif
      #if ENABLED(Z2_IS_TMC2130)
        automatic_current_control(stepperZ2, "Z2");
      #endif
      #if ENABLED(E0_IS_TMC2130)
        automatic_current_control(stepperE0, "E0");
      #endif
      #if ENABLED(E1_IS_TMC2130)
        automatic_current_control(stepperE1, "E1");
      #endif
      #if ENABLED(E2_IS_TMC2130)
        automatic_current_control(stepperE2, "E2");
      #endif
      #if ENABLED(E3_IS_TMC2130)
        automatic_current_control(stepperE3, "E3");
      #endif
      #if ENABLED(E4_IS_TMC2130)
        automatic_current_control(stepperE4, "E4");
      #endif
      #if ENABLED(E5_IS_TMC2130)
        automatic_current_control(stepperE5, "E5");
      #endif
    }
  }

#endif // HAVE_TMC2130
