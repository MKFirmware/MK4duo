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
 * printer.cpp
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#include "../../MK4duo.h"

const char axis_codes[XYZE] = {'X', 'Y', 'Z', 'E'};

Printer printer;

bool  Printer::pos_saved              = false,
      Printer::relative_mode          = false,
      Printer::Running                = false,
      Printer::axis_relative_modes[]  = AXIS_RELATIVE_MODES;

volatile bool Printer::wait_for_user = false;

// Print status related
long    Printer::currentLayer  = 0,
        Printer::maxLayer      = -1;   // -1 = unknown

char    Printer::printName[21] = "";   // max. 20 chars + 0

uint8_t Printer::progress                 = 0,
        Printer::host_keepalive_interval  = DEFAULT_KEEPALIVE_INTERVAL;

// Inactivity shutdown
millis_t  Printer::max_inactive_time  = 0;

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

#if ENABLED(HOST_KEEPALIVE_FEATURE)
  Printer::MK4duoBusyState Printer::busy_state = NOT_BUSY;
#endif

bool Printer::filament_out = false;

#if ENABLED(RFID_MODULE)
  uint32_t  Printer::Spool_ID[EXTRUDERS] = ARRAY_BY_EXTRUDERS(0);
  bool      Printer::RFID_ON = false,
            Printer::Spool_must_read[EXTRUDERS]  = ARRAY_BY_EXTRUDERS(false),
            Printer::Spool_must_write[EXTRUDERS] = ARRAY_BY_EXTRUDERS(false);
#endif

#if ENABLED(NPR2)
  uint8_t Printer::old_color = 99;
#endif

#if ENABLED(G38_PROBE_TARGET)
  bool  Printer::G38_move         = false,
        Printer::G38_endstop_hit  = false;
#endif

#if ENABLED(BARICUDA)
  int Printer::baricuda_valve_pressure  = 0,
      Printer::baricuda_e_to_p_pressure = 0;
#endif

#if ENABLED(EASY_LOAD)
  bool Printer::allow_lengthy_extrude_once = false; // for load/unload
#endif

#if ENABLED(IDLE_OOZING_PREVENT)
  millis_t  Printer::axis_last_activity   = 0;
  bool      Printer::IDLE_OOZING_enabled  = true,
            Printer::IDLE_OOZING_retracted[EXTRUDERS] = ARRAY_BY_EXTRUDERS(false);
#endif

#if HAS_CHDK
  millis_t  Printer::chdkHigh   = 0;
  bool      Printer::chdkActive = false;
#endif

/**
 * Public Function
 */

/**
 * MK4duo entry-point: Set up before the program loop
 *  - Set up Hardware Board
 *  - Set up the kill pin, filament runout, power hold
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

  #if HAS_FIL_RUNOUT
    filamentrunout.Init();
  #endif

  #if HAS_KILL
    SET_INPUT_PULLUP(KILL_PIN);
  #endif

  setup_powerhold();

  #if HAS_STEPPER_RESET
    stepper.disableStepperDrivers();
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
    card.mount();
    // loads custom configuration from SDCARD if available else uses defaults
    card.RetrieveSettings();
  #endif

  // Loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
  eeprom.Load_Settings();

  #if ENABLED(WORKSPACE_OFFSETS)
    // Initialize current position based on home_offset
    COPY_ARRAY(mechanics.current_position, mechanics.home_offset);
  #else
    ZERO(mechanics.current_position);
  #endif

  // Vital to init stepper/planner equivalent for current_position
  mechanics.sync_plan_position();

  thermalManager.init();  // Initialize temperature loop

  #if ENABLED(CNCROUTER)
    cnc.init();
  #endif

  stepper.init(); // Initialize stepper, this enables interrupts!

  #if HAS_SERVOS
    servo_init(); // Initialize all Servo
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
    stepper.enableStepperDrivers();
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

  #if ENABLED(NEOPIXEL_LED)
    setup_neopixel();
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
      flowmeter.flow_firstread = false;
    #endif
    flowmeter.flow_init();
  #endif

  #if ENABLED(RFID_MODULE)
    RFID_ON = rfid522.init();
    if (RFID_ON)
      SERIAL_EM("RFID CONNECT");
  #endif

  lcd_init();
  #if ENABLED(SHOW_BOOTSCREEN)
    #if ENABLED(DOGLCD)                           // On DOGM the first bootscreen is already drawn
      #if ENABLED(SHOW_CUSTOM_BOOTSCREEN)
        safe_delay(CUSTOM_BOOTSCREEN_TIMEOUT);    // Custom boot screen pause
        lcd_bootscreen();                         // Show MK4duo boot screen
      #endif
      safe_delay(BOOTSCREEN_TIMEOUT);
    #elif ENABLED(ULTRA_LCD)
      lcd_bootscreen();
      #if DISABLED(SDSUPPORT)
        lcd_init();
      #endif
    #endif
  #endif

  #if ENABLED(COLOR_MIXING_EXTRUDER) && MIXING_VIRTUAL_TOOLS > 1
    mixing_tools_init();
  #endif

  #if ENABLED(BLTOUCH)
    // Make sure any BLTouch error condition is cleared
    probe.bltouch_command(BLTOUCH_RESET);
    probe.set_bltouch_deployed(true);
    probe.set_bltouch_deployed(false);
  #endif

  #if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
    endstops.setup_endstop_interrupts();
  #endif

  #if ENABLED(DELTA_HOME_ON_POWER)
    mechanics.Home(true);
  #endif

  #if FAN_COUNT > 0
    LOOP_FAN() fans[f].Speed = 0;
  #endif
}

/**
 * The main MK4duo program loop
 *
 *  - Save or log commands to SD
 *  - Process available commands (if not saving)
 *  - Call heater manager
 *  - Call Fans manager
 *  - Call inactivity manager
 *  - Call endstop manager
 *  - Call LCD update
 */
void Printer::loop() {

  commands.get_available_commands();

  #if HAS_SDSUPPORT
    card.checkautostart(false);
  #endif

  commands.advance_command_queue();

  endstops.report_state();
  idle();
}

void Printer::safe_delay(millis_t ms) {
  while (ms > 50) {
    ms -= 50;
    HAL::delayMilliseconds(50);
    if (HAL::Analog_is_ready) thermalManager.manage_temp_controller();
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
void Printer::bracket_probe_move(const bool before) {
  static float saved_feedrate_mm_s;
  static int16_t saved_feedrate_percentage;
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) DEBUG_POS("bracket_probe_move", mechanics.current_position);
  #endif
  if (before) {
    saved_feedrate_mm_s = mechanics.feedrate_mm_s;
    saved_feedrate_percentage = mechanics.feedrate_percentage;
    mechanics.feedrate_percentage = 100;
    commands.refresh_cmd_timeout();
  }
  else {
    mechanics.feedrate_mm_s = saved_feedrate_mm_s;
    mechanics.feedrate_percentage = saved_feedrate_percentage;
    commands.refresh_cmd_timeout();
  }
}

void Printer::setup_for_endstop_or_probe_move()       { bracket_probe_move(true); }
void Printer::clean_up_after_endstop_or_probe_move()  { bracket_probe_move(false); }

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
    mechanics.destination[E_AXIS] = (parser.value_axis_units(E_AXIS) * tools.density_percentage[tools.previous_extruder] / 100) + mechanics.current_position[E_AXIS];

  if(!DEBUGGING(DRYRUN))
    print_job_counter.data.filamentUsed += (mechanics.destination[E_AXIS] - mechanics.current_position[E_AXIS]);

  #if ENABLED(COLOR_MIXING_EXTRUDER)
    get_mix_from_command();
  #endif

  #if ENABLED(RFID_MODULE)
    if(!DEBUGGING(DRYRUN))
      rfid522.RfidData[tools.active_extruder].data.lenght -= (mechanics.destination[E_AXIS] - mechanics.current_position[E_AXIS]);
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
    tools.target_extruder = t;
  }
  else
    tools.target_extruder = tools.active_extruder;

  return false;
}

/**
 * Kill all activity and lock the machine.
 * After this the machine will need to be reset.
 */
void Printer::kill(const char* lcd_msg) {
  SERIAL_LM(ER, MSG_ERR_KILLED);

  thermalManager.disable_all_heaters();
  stepper.disable_all_steppers();

  #if ENABLED(KILL_METHOD) && (KILL_METHOD == 1)
    HAL::resetHardware();
  #endif
  #if ENABLED(FLOWMETER_SENSOR) && ENABLED(MINFLOW_PROTECTION)
    flowmeter.flow_firstread = false;
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
     cnc.disable_router();
  #endif

  #if HAS_POWER_SWITCH
    SET_INPUT(PS_ON_PIN);
  #endif

  #if HAS_SUICIDE
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
    flowmeter.flow_firstread = false;
  #endif

  thermalManager.disable_all_heaters();

  #if ENABLED(PROBING_FANS_OFF)
    LOOP_FAN() {
      if (fans[f].paused) fans[f].pause(false); // put things back the way they were
    }
  #endif

  #if ENABLED(LASER)
    if (laser.diagnostics) SERIAL_EM("Laser set to off, Stop() called");
    laser.extinguish();
    #if ENABLED(LASER_PERIPHERALS)
      laser.peripherals_off();
    #endif
  #endif

  #if ENABLED(CNCROUTER)
     cnc.disable_router();
  #endif

  if (IsRunning()) {
    Running = false;
    SERIAL_LM(ER, MSG_ERR_STOPPED);
    SERIAL_STR(PAUSE);
    SERIAL_EOL();
    LCD_MESSAGEPGM(MSG_STOPPED);
  }
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
    flowmeter.flowrate_manage();
  #endif

  #if ENABLED(CNCROUTER)
    cnc.manage();
  #endif

  manage_inactivity(no_stepper_sleep);

  handle_Interrupt_Event();

  print_job_counter.tick();

  #if FAN_COUNT > 0
    LOOP_FAN() fans[f].Check();
  #endif

  if (HAL::execute_100ms) {
    // Event 100 Ms
    HAL::execute_100ms = false;
    if (HAL::Analog_is_ready) thermalManager.manage_temp_controller();
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

  #if HAS_FIL_RUNOUT
    filamentrunout.Check();
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
    if (flowmeter.flow_firstread && print_job_counter.isRunning() && (flowmeter.flowrate < (float)MINFLOW_PROTECTION)) {
      flowmeter.flow_firstread = false;
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

  #if HAS_POWER_SWITCH
    if (!powerManager.powersupply_on) powerManager.check(); // Check Power
  #endif

  #if ENABLED(EXTRUDER_RUNOUT_PREVENT)
    if (ELAPSED(ms, commands.previous_cmd_ms + (EXTRUDER_RUNOUT_SECONDS) * 1000UL)
      && heaters[EXTRUDER_IDX].current_temperature > EXTRUDER_RUNOUT_MINTEMP) {
      bool oldstatus = 0;
      #if ENABLED(DONDOLO_SINGLE_MOTOR)
        oldstatus = E0_ENABLE_READ;
        enable_E0();
      #else // !DONDOLO_SINGLE_MOTOR
        switch (tools.active_extruder) {
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
      planner.buffer_line_kinematic(mechanics.current_position, MMM_TO_MMS(EXTRUDER_RUNOUT_SPEED), tools.active_extruder);
      mechanics.current_position[E_AXIS] = olde;
      mechanics.set_e_position_mm(olde);
      stepper.synchronize();
      #if ENABLED(DONDOLO_SINGLE_MOTOR)
        E0_ENABLE_WRITE(oldstatus);
      #else
        switch(tools.active_extruder) {
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
    if (mechanics.delayed_move_time && ELAPSED(ms, mechanics.delayed_move_time + 1000UL) && IsRunning()) {
      // travel moves have been received so enact them
      mechanics.delayed_move_time = 0xFFFFFFFFUL; // force moves to be done
      mechanics.set_destination_to_current();
      mechanics.prepare_move_to_destination();
    }
  #endif

  #if ENABLED(IDLE_OOZING_PREVENT)
    if (planner.blocks_queued()) axis_last_activity = millis();
    if (heaters[EXTRUDER_IDX].current_temperature > IDLE_OOZING_MINTEMP && !(DEBUGGING(DRYRUN)) && IDLE_OOZING_enabled) {
      #if ENABLED(FILAMENTCHANGEENABLE)
        if (!filament_changing)
      #endif
      {
        if (heaters[EXTRUDER_IDX].target_temperature < IDLE_OOZING_MINTEMP) {
          IDLE_OOZING_retract(false);
        }
        else if ((millis() - axis_last_activity) >  IDLE_OOZING_SECONDS * 1000UL) {
          IDLE_OOZING_retract(true);
        }
      }
    }
  #endif

  #if ENABLED(RFID_MODULE)
    for (int8_t e = 0; e < EXTRUDERS; e++) {
      if (Spool_must_read[e]) {
        if (rfid522.getID(e)) {
          Spool_ID[e] = rfid522.RfidDataID[e].Spool_ID;
          HAL::delayMilliseconds(200);
          if (rfid522.readBlock(e)) {
            Spool_must_read[e] = false;
            tools.density_percentage[e] = rfid522.RfidData[e].data.density;
            tools.filament_size[e] = rfid522.RfidData[e].data.size;
            calculate_volumetric_multipliers();
            rfid522.printInfo(e);
          }
        }
      }

      if (Spool_must_write[e]) {
        if (rfid522.getID(e)) {
          if (Spool_ID[e] == rfid522.RfidDataID[e].Spool_ID) {
            HAL::delayMilliseconds(200);
            if (rfid522.writeBlock(e)) {
              Spool_must_write[e] = false;
              SERIAL_SMV(INFO, "Spool on E", e);
              SERIAL_EM(" writed!");
              rfid522.printInfo(e);
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
    tmc2130_checkOverTemp();
  #endif

  planner.check_axes_activity();
}

void Printer::setInterruptEvent(const MK4duoInterruptEvent event) {
  if (interruptEvent == INTERRUPT_EVENT_NONE)
    interruptEvent = event;
}

void Printer::handle_Interrupt_Event() {

  if (interruptEvent == INTERRUPT_EVENT_NONE) return; // Exit if none Event

  const MK4duoInterruptEvent event = interruptEvent;
  interruptEvent = INTERRUPT_EVENT_NONE;

  switch(event) {
    #if HAS_FIL_RUNOUT
      case INTERRUPT_EVENT_FIL_RUNOUT:
        if (!filament_out && (IS_SD_PRINTING || print_job_counter.isRunning())) {
          filament_out = true;
          commands.enqueue_and_echo_commands_P(PSTR(FILAMENT_RUNOUT_SCRIPT));
          SERIAL_LM(REQUEST_PAUSE, "End Filament detect");
          stepper.synchronize();
        }
        break;
    #endif

    #if HAS_EXT_ENCODER
      case INTERRUPT_EVENT_ENC_DETECT:
        if (!filament_out && (IS_SD_PRINTING || print_job_counter.isRunning())) {
          filament_out = true;
          stepper.synchronize();

          #if ENABLED(ADVANCED_PAUSE_FEATURE)
            commands.enqueue_and_echo_commands_P(PSTR("M600"));
          #endif

          SERIAL_LM(REQUEST_PAUSE, "Extruder jam detected");
        }
        break;
    #endif

    default:
      break;
  }
}

/**
 * Sensitive pin test for M42, M226
 */
bool Printer::pin_is_protected(const Pin pin) {
  static const int8_t sensitive_pins[] PROGMEM = SENSITIVE_PINS;
  for (uint8_t i = 0; i < COUNT(sensitive_pins); i++)
    if (pin == pgm_read_byte(&sensitive_pins[i])) return true;
  return false;
}

void Printer::suicide() {
  #if HAS_SUICIDE
    OUT_WRITE(SUICIDE_PIN, LOW);
  #endif
}

char Printer::GetStatusCharacter(){
  return  print_job_counter.isRunning() ? 'P'   // Printing
        : print_job_counter.isPaused()  ? 'A'   // Paused / Stopped
        :                                 'I';  // Idle
}

/**
 * Private Function
 */

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

float Printer::calculate_volumetric_multiplier(const float diameter) {
  if (!tools.volumetric_enabled || diameter == 0) return 1.0;
  return 1.0 / CIRCLE_AREA(diameter * 0.5);
}
void Printer::calculate_volumetric_multipliers() {
  for (uint8_t e = 0; e < EXTRUDERS; e++)
    tools.volumetric_multiplier[e] = calculate_volumetric_multiplier(tools.filament_size[e]);
}

#if ENABLED(IDLE_OOZING_PREVENT)

  void Printer::IDLE_OOZING_retract(bool retracting) {
    if (retracting && !IDLE_OOZING_retracted[tools.active_extruder]) {
      float old_feedrate_mm_s = mechanics.feedrate_mm_s;
      mechanics.set_destination_to_current();
      mechanics.current_position[E_AXIS] += IDLE_OOZING_LENGTH / tools.volumetric_multiplier[tools.active_extruder];
      mechanics.feedrate_mm_s = IDLE_OOZING_FEEDRATE;
      mechanics.set_e_position_mm(mechanics.current_position[E_AXIS]);
      mechanics.prepare_move_to_destination();
      mechanics.feedrate_mm_s = old_feedrate_mm_s;
      IDLE_OOZING_retracted[tools.active_extruder] = true;
      //SERIAL_EM("-");
    }
    else if (!retracting && IDLE_OOZING_retracted[tools.active_extruder]) {
      float old_feedrate_mm_s = mechanics.feedrate_mm_s;
      mechanics.set_destination_to_current();
      mechanics.current_position[E_AXIS] -= (IDLE_OOZING_LENGTH+IDLE_OOZING_RECOVER_LENGTH) / tools.volumetric_multiplier[tools.active_extruder];
      mechanics.feedrate_mm_s = IDLE_OOZING_RECOVER_FEEDRATE;
      mechanics.set_e_position_mm(mechanics.current_position[E_AXIS]);
      mechanics.prepare_move_to_destination();
      mechanics.feedrate_mm_s = old_feedrate_mm_s;
      IDLE_OOZING_retracted[tools.active_extruder] = false;
      //SERIAL_EM("+");
    }
  }

#endif

#if ENABLED(HOST_KEEPALIVE_FEATURE)

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
          max_temp = MAX3(max_temp, heaters[BED_INDEX].target_temperature, heaters[BED_INDEX].current_temperature);
        #endif
      LOOP_HOTEND()
        max_temp = MAX3(max_temp, heaters[h].current_temperature, heaters[h].target_temperature);
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
