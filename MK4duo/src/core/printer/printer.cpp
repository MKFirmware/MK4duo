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

#include "../../../MK4duo.h"

const char axis_codes[XYZE] = {'X', 'Y', 'Z', 'E'};

Printer printer;

bool Printer::axis_relative_modes[] = AXIS_RELATIVE_MODES;

// Print status related
long    Printer::currentLayer  = 0,
        Printer::maxLayer      = -1;   // -1 = unknown

char    Printer::printName[21] = "";   // max. 20 chars + 0

uint8_t Printer::progress = 0;

// Inactivity shutdown
watch_t Printer::max_inactivity_watch,
        Printer::move_watch(DEFAULT_STEPPER_DEACTIVE_TIME * 1000UL);

#if ENABLED(HOST_KEEPALIVE_FEATURE)
  watch_t Printer::host_keepalive_watch(DEFAULT_KEEPALIVE_INTERVAL * 1000UL);
#endif

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

#if ENABLED(RFID_MODULE)
  uint32_t  Printer::Spool_ID[EXTRUDERS] = ARRAY_BY_EXTRUDERS(0);
  bool      Printer::RFID_ON = false,
            Printer::Spool_must_read[EXTRUDERS]  = ARRAY_BY_EXTRUDERS(false),
            Printer::Spool_must_write[EXTRUDERS] = ARRAY_BY_EXTRUDERS(false);
#endif

#if ENABLED(BARICUDA)
  int Printer::baricuda_valve_pressure  = 0,
      Printer::baricuda_e_to_p_pressure = 0;
#endif

#if ENABLED(IDLE_OOZING_PREVENT)
  millis_t  Printer::axis_last_activity   = 0;
  bool      Printer::IDLE_OOZING_enabled  = true,
            Printer::IDLE_OOZING_retracted[EXTRUDERS] = ARRAY_BY_EXTRUDERS(false);
#endif

#if HAS_CHDK
  watch_t Printer::chdk_watch(CHDK_DELAY);
  bool    Printer::chdkActive = false;
#endif

// Private

uint8_t   Printer::mk_debug_flag  = 0, // For debug
          Printer::mk_home_flag   = 0; // For Homed

uint16_t  Printer::mk_various_flag  = 0; // For various

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

  #if ENABLED(MB_SETUP)
    MB_SETUP;
  #endif

  setup_pinout();

  #if HAS_POWER_SWITCH
    #if PS_DEFAULT_OFF
      powerManager.power_off();
    #else
      powerManager.power_on();
    #endif
  #endif

  #if HAS_STEPPER_RESET
    stepper.disableStepperDrivers();
  #endif

  // Init Serial for HOST
  MKSERIAL.begin(BAUDRATE);
  HAL::delayMilliseconds(1000);
  SERIAL_L(START);

  #if HAVE_DRV(TMC2130)
    tmc_init_cs_pins();
  #endif
  #if HAVE_DRV(TMC2208)
    tmc2208_serial_begin();
  #endif

  #if MECH(MUVE3D) && ENABLED(PROJECTOR_PORT) && ENABLED(PROJECTOR_BAUDRATE)
    DLPSerial.begin(PROJECTOR_BAUDRATE);
  #endif

  // Check startup
  SERIAL_STR(INFO);
  HAL::showStartReason();

  // Init Watchdog
  watchdog.init();

  SERIAL_LM(ECHO, BUILD_VERSION);

  #if ENABLED(STRING_DISTRIBUTION_DATE) && ENABLED(STRING_CONFIG_H_AUTHOR)
    SERIAL_LM(ECHO, MSG_CONFIGURATION_VER STRING_DISTRIBUTION_DATE MSG_AUTHOR STRING_CONFIG_H_AUTHOR);
    SERIAL_LM(ECHO, MSG_COMPILED __DATE__);
  #endif // STRING_DISTRIBUTION_DATE

  SERIAL_SMV(ECHO, MSG_FREE_MEMORY, HAL::getFreeRam());
  SERIAL_EMV(MSG_PLANNER_BUFFER_BYTES, (int)sizeof(block_t)*BLOCK_BUFFER_SIZE);

  // Send "ok" after commands by default
  commands.setup();

  lcd_init();
  LCD_MESSAGEPGM(WELCOME_MSG);

  #if ENABLED(SHOW_BOOTSCREEN)
    #if ENABLED(DOGLCD) || ENABLED(ULTRA_LCD)
      lcd_bootscreen(); // Show MK4duo boot screen
    #endif
  #endif

  #if HAS_SDSUPPORT
    if (!card.isOK()) card.mount();
  #endif

  print_job_counter.init();

  mechanics.init();

  // Init endstops
  endstops.init();

  // Load data from EEPROM if available (or use defaults)
  // This also updates variables in the planner, elsewhere
  const bool eeprom_loaded = eeprom.Load_Settings();

  #if ENABLED(WORKSPACE_OFFSETS)
    // Initialize current position based on home_offset
    COPY_ARRAY(mechanics.current_position, mechanics.home_offset);
  #else
    ZERO(mechanics.current_position);
  #endif

  // Vital to init stepper/planner equivalent for current_position
  mechanics.sync_plan_position_mech_specific();

  thermalManager.init();  // Initialize temperature loop

  stepper.init(); // Initialize stepper, this enables interrupts!

  #if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
    externaldac.begin();
    externaldac.set_driver_current();
  #endif

  #if ENABLED(CNCROUTER)
    cnc.init();
  #endif

  #if HAS_SERVOS
    servo_init(); // Initialize all Servo
  #endif

  #if HAS_CASE_LIGHT
    caselight.update();
  #endif

  #if HAS_SOFTWARE_ENDSTOPS
    endstops.setSoftEndstop(true);
  #endif

  #if HAS_STEPPER_RESET
    stepper.enableStepperDrivers();
  #endif

  #if ENABLED(DIGIPOT_I2C)
    digipot_i2c_init();
  #endif

  #if HAS_COLOR_LEDS
    leds.setup();
  #endif

  #if ENABLED(LASER)
    laser.init();
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

  #if ENABLED(COLOR_MIXING_EXTRUDER) && MIXING_VIRTUAL_TOOLS > 1
    mixing_tools_init();
  #endif

  #if ENABLED(BLTOUCH)
    probe.bltouch_init();
  #endif

  // All Initialized set Running to true.
  setRunning(true);

  #if ENABLED(DELTA_HOME_ON_POWER)
    mechanics.home();
  #endif

  #if FAN_COUNT > 0
    LOOP_FAN() fans[f].Speed = 0;
  #endif

  if (!eeprom_loaded) lcd_eeprom_allert();

  #if HAS_SD_RESTART
    restart.do_print_job();
  #endif
}

/**
 * The main MK4duo program loop
 *
 *  - Save or log commands to SD
 *  - Process available commands (if not saving)
 *  - Call endstop manager
 *  - Call LCD update
 */
void Printer::loop() {

  printer.keepalive(NotBusy);

  #if HAS_SDSUPPORT

    card.checkautostart();

    if (isAbortSDprinting()) {
      setAbortSDprinting(false);

      #if HAS_SD_RESTART
        // Save Job for restart
        if (IS_SD_PRINTING) restart.save_data(true);
      #endif

      // Stop SD printing
      card.stopSDPrint();

      // Clear all command in quee
      commands.clear_queue();

      // Stop all stepper
      quickstop_stepper();

      // Auto home
      #if Z_HOME_DIR > 0
        mechanics.home();
      #else
        mechanics.home(true, true, false);
      #endif

      // Disabled Heaters and Fan
      thermalManager.disable_all_heaters();
      #if FAN_COUNT > 0
        LOOP_FAN() fans[f].Speed = 0;
      #endif

      // Stop printer job timer
      print_job_counter.stop();
    }

  #endif // HAS_SDSUPPORT

  commands.get_available();
  commands.advance_queue();
  endstops.report_state();
  idle();
}

void Printer::check_periodical_actions() {

  static uint8_t  cycle_1000ms = 10,  // Event 1.0 Second
                  cycle_2500ms = 25;  // Event 2.5 Second

  // Control interrupt events
  handle_interrupt_events();

  // Tick timer job counter
  print_job_counter.tick();

  // Event 100 Ms - 10Hz
  if (HAL::execute_100ms) {
    HAL::execute_100ms = false;
    planner.check_axes_activity();
    thermalManager.spin();

    // Event 1.0 Second
    if (--cycle_1000ms == 0) {
      cycle_1000ms = 10;
      if (!isSuspendAutoreport() && isAutoreportTemp()) {
        thermalManager.report_temperatures();
        SERIAL_EOL();
      }
      #if HAS_SDSUPPORT
        if (isAutoreportSD()) card.printStatus();
      #endif
      #if ENABLED(NEXTION)
        nextion_draw_update();
      #endif
      if (planner.cleaning_buffer_flag) {
        planner.cleaning_buffer_flag = false;
        #if ENABLED(SD_FINISHED_STEPPERRELEASE) && ENABLED(SD_FINISHED_RELEASECOMMAND)
          commands.enqueue_and_echo_P(PSTR(SD_FINISHED_RELEASECOMMAND));
        #endif
      }
    }

    // Event 2.5 Second
    if (--cycle_2500ms == 0) {
      cycle_2500ms = 25;
      #if FAN_COUNT > 0
        LOOP_FAN() fans[f].spin();
      #endif
      #if HAS_POWER_SWITCH
        powerManager.spin();
      #endif
    }
  }

}

void Printer::safe_delay(millis_t ms) {
  while (ms > 50) {
    ms -= 50;
    HAL::delayMilliseconds(50);
    check_periodical_actions();
  }
  HAL::delayMilliseconds(ms);
  check_periodical_actions();
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
    if (printer.debugLeveling()) DEBUG_POS("bracket_probe_move", mechanics.current_position);
  #endif
  if (before) {
    saved_feedrate_mm_s = mechanics.feedrate_mm_s;
    saved_feedrate_percentage = mechanics.feedrate_percentage;
    mechanics.feedrate_percentage = 100;
  }
  else {
    mechanics.feedrate_mm_s = saved_feedrate_mm_s;
    mechanics.feedrate_percentage = saved_feedrate_percentage;
  }
}

void Printer::setup_for_endstop_or_probe_move()       { bracket_probe_move(true); }
void Printer::clean_up_after_endstop_or_probe_move()  { bracket_probe_move(false); }

void Printer::quickstop_stepper() {
  planner.quick_stop();
  planner.synchronize();
  mechanics.set_current_from_steppers_for_axis(ALL_AXES);
  mechanics.sync_plan_position_mech_specific();
}

/**
 * Kill all activity and lock the machine.
 * After this the machine will need to be reset.
 */
void Printer::kill(const char* lcd_msg) {
  SERIAL_LM(ER, MSG_ERR_KILLED);

  thermalManager.disable_all_heaters();
  stepper.disable_all();

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

  printer.safe_delay(600);  // Wait a short time (allows messages to get out before shutting down.
  #if DISABLED(CPU_32_BIT)
    cli(); // Stop interrupts
  #endif

  printer.safe_delay(250);  // Wait to ensure all interrupts routines stopped
  thermalManager.disable_all_heaters(); // Turn off heaters again

  #if ENABLED(LASER)
    laser.init();
    #if ENABLED(LASER_PERIPHERALS)
      laser.peripherals_off();
    #endif
  #endif

  #if ENABLED(CNCROUTER)
    cnc.disable_router();
  #endif

  #if HAS_POWER_SWITCH
    powerManager.power_off();
  #endif

  #if HAS_SUICIDE
    suicide();
  #endif

  while(1) { watchdog.reset(); } // Wait for reset

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
      if (fans[f].isIdle()) fans[f].setIdle(false); // put things back the way they were
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

  if (isRunning()) {
    setRunning(false);
    SERIAL_LM(ER, MSG_ERR_STOPPED);
    SERIAL_STR(PAUSE);
    SERIAL_EOL();
    LCD_MESSAGEPGM(MSG_STOPPED);
  }
}

/**
 * Manage several activities:
 *  - Lcd update
 *  - Check periodical actions
 *  - Keep the command buffer full
 *  - Host Keepalive
 *  - Check Flow meter sensor
 *  - Cnc manage
 *  - Check for Filament Runout
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
void Printer::idle(const bool ignore_stepper_queue/*=false*/) {

  lcd_update();

  check_periodical_actions();

  commands.get_available();

  handle_safety_watch();

  if (max_inactivity_watch.stopwatch && max_inactivity_watch.elapsed()) {
    SERIAL_LMT(ER, MSG_KILL_INACTIVE_TIME, parser.command_ptr);
    kill(PSTR(MSG_KILLED));
  }

  #if ENABLED(DHT_SENSOR)
    dhtsensor.spin();
  #endif

  #if ENABLED(CNCROUTER)
    cnc.manage();
  #endif

  #if HAS_FIL_RUNOUT
    filamentrunout.spin();
  #endif

  #if ENABLED(FLOWMETER_SENSOR)

    flowmeter.flowrate_manage();

    #if ENABLED(MINFLOW_PROTECTION)
      if (flowmeter.flow_firstread && print_job_counter.isRunning() && (flowmeter.flowrate < (float)MINFLOW_PROTECTION)) {
        flowmeter.flow_firstread = false;
        kill(PSTR(MSG_KILLED));
      }
    #endif

  #endif // ENABLED(FLOWMETER_SENSOR)

  // Prevent steppers timing-out in the middle of M600
  #if ENABLED(ADVANCED_PAUSE_FEATURE) && ENABLED(PAUSE_PARK_NO_STEPPER_TIMEOUT)
    #define MOVE_AWAY_TEST !did_pause_print
  #else
    #define MOVE_AWAY_TEST true
  #endif

  if (move_watch.stopwatch) {
    if (planner.has_blocks_queued())
      move_watch.start(); // reset stepper move watch to keep steppers powered
    else if (MOVE_AWAY_TEST && !ignore_stepper_queue && move_watch.elapsed()) {
      #if ENABLED(DISABLE_INACTIVE_X)
        stepper.disable_X();
      #endif
      #if ENABLED(DISABLE_INACTIVE_Y)
        stepper.disable_Y();
      #endif
      #if ENABLED(DISABLE_INACTIVE_Z)
        stepper.disable_Z();
      #endif
      #if ENABLED(DISABLE_INACTIVE_E)
        stepper.disable_E();
      #endif
      #if ENABLED(AUTO_BED_LEVELING_UBL) && ENABLED(ULTIPANEL)  // Only needed with an LCD
        if (ubl.lcd_map_control) ubl.lcd_map_control = defer_return_to_status = false;
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
  }

  #if HAS_CHDK // Check if pin should be set to LOW after M240 set it to HIGH
    if (chdkActive && chdk_watch.elapsed()) {
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
        commands.enqueue_and_echo_P(PSTR("G28"));
        LCD_MESSAGEPGM(MSG_AUTO_HOME);
      }
      if (homeDebounceCount < HOME_DEBOUNCE_DELAY)
        homeDebounceCount++;
      else
        homeDebounceCount = 0;
    }
  #endif

  #if ENABLED(EXTRUDER_RUNOUT_PREVENT)

    static watch_t extruder_runout_watch(EXTRUDER_RUNOUT_SECONDS * 1000UL);

    if (heaters[ACTIVE_HOTEND].current_temperature > EXTRUDER_RUNOUT_MINTEMP
      && extruder_runout_watch.elapsed()
      && !planner.has_blocks_queued()
    ) {
      #if ENABLED(DONDOLO_SINGLE_MOTOR)
        const bool oldstatus = E0_ENABLE_READ;
        enable_E0();
      #else // !DONDOLO_SINGLE_MOTOR
        bool oldstatus;
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

      const float olde = mechanics.current_position[E_AXIS];
      mechanics.current_position[E_AXIS] += EXTRUDER_RUNOUT_EXTRUDE;
      planner.buffer_line_kinematic(mechanics.current_position, MMM_TO_MMS(EXTRUDER_RUNOUT_SPEED), tools.active_extruder);
      mechanics.current_position[E_AXIS] = olde;
      planner.set_e_position_mm(olde);
      planner.synchronize();
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

      extruder_runout_watch.start();
    }
  #endif // EXTRUDER_RUNOUT_PREVENT

  #if ENABLED(DUAL_X_CARRIAGE)
    // handle delayed move timeout
    if (mechanics.delayed_move_time && ELAPSED(millis(), mechanics.delayed_move_time + 1000UL) && isRunning()) {
      // travel moves have been received so enact them
      mechanics.delayed_move_time = 0xFFFFFFFFUL; // force moves to be done
      mechanics.set_destination_to_current();
      mechanics.prepare_move_to_destination();
    }
  #endif

  #if ENABLED(IDLE_OOZING_PREVENT)
    if (planner.has_blocks_queued()) axis_last_activity = millis();
    if (heaters[ACTIVE_HOTEND].current_temperature > IDLE_OOZING_MINTEMP && !debugDryrun() && IDLE_OOZING_enabled) {
      #if ENABLED(FILAMENTCHANGEENABLE)
        if (!filament_changing)
      #endif
      {
        if (heaters[ACTIVE_HOTEND].target_temperature < IDLE_OOZING_MINTEMP) {
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
            #if ENABLED(VOLUMETRIC_EXTRUSION)
              tools.calculate_volumetric_multipliers();
            #endif
            tools.refresh_e_factor(e);
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

  #if ENABLED(MONITOR_DRIVER_STATUS)
    monitor_tmc_driver();
  #endif

}

void Printer::setInterruptEvent(const MK4duoInterruptEvent event) {
  if (interruptEvent == INTERRUPT_EVENT_NONE)
    interruptEvent = event;
}

void Printer::handle_interrupt_events() {

  if (interruptEvent == INTERRUPT_EVENT_NONE) return; // Exit if none Event

  const MK4duoInterruptEvent event = interruptEvent;
  interruptEvent = INTERRUPT_EVENT_NONE;

  switch(event) {
    #if HAS_FIL_RUNOUT
      case INTERRUPT_EVENT_FIL_RUNOUT:
        if (!isFilamentOut() && (IS_SD_PRINTING || print_job_counter.isRunning())) {
          setFilamentOut(true);
          commands.enqueue_and_echo_P(PSTR(FILAMENT_RUNOUT_SCRIPT));
          planner.synchronize();
        }
        break;
    #endif

    #if HAS_EXT_ENCODER
      case INTERRUPT_EVENT_ENC_DETECT:
        if (!isFilamentOut() && (IS_SD_PRINTING || print_job_counter.isRunning())) {
          setFilamentOut(true);
          planner.synchronize();

          #if ENABLED(ADVANCED_PAUSE_FEATURE)
            commands.enqueue_and_echo_P(PSTR("M600"));
          #endif
        }
        break;
    #endif

    default:
      break;
  }
}

/**
 * Turn off heating after 30 minutes of inactivity
 */
void Printer::handle_safety_watch() {

  static watch_t safety_watch(30 * 60 * 1000UL); // Set 30 minutes

  if (safety_watch.isRunning() && (IS_SD_PRINTING || print_job_counter.isRunning() || print_job_counter.isPaused() || !thermalManager.heaters_isON()))
    safety_watch.stop();
  else if (!safety_watch.isRunning() && thermalManager.heaters_isON())
    safety_watch.start();
  else if (safety_watch.isRunning() && safety_watch.elapsed()) {
    safety_watch.stop();
    thermalManager.disable_all_heaters();
    SERIAL_EM("Max inactivity time (30 minutes) Heaters switch off!");
  }
}

/**
 * Sensitive pin test for M42, M226
 */
bool Printer::pin_is_protected(const pin_t pin) {
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

/**
 * Private Function
 */

void Printer::setup_pinout() {

  #if HAS_BUZZER
    BUZZ(10,10);
  #endif

  #if PIN_EXISTS(SS)
    OUT_WRITE(SS_PIN, HIGH);
  #endif

  #if PIN_EXISTS(MAX6675_SS)
    OUT_WRITE(MAX6675_SS_PIN, HIGH);
  #endif

  #if PIN_EXISTS(MAX31855_SS0)
    OUT_WRITE(MAX31855_SS0_PIN, HIGH);
  #endif
  #if PIN_EXISTS(MAX31855_SS1)
    OUT_WRITE(MAX31855_SS1_PIN, HIGH);
  #endif
  #if PIN_EXISTS(MAX31855_SS2)
    OUT_WRITE(MAX31855_SS2_PIN, HIGH);
  #endif
  #if PIN_EXISTS(MAX31855_SS3)
    OUT_WRITE(MAX31855_SS3_PIN, HIGH);
  #endif

  #if HAS_SUICIDE
    OUT_WRITE(SUICIDE_PIN, HIGH);
  #endif

  #if HAS_KILL
    SET_INPUT_PULLUP(KILL_PIN);
  #endif

  #if HAS_PHOTOGRAPH
    OUT_WRITE(PHOTOGRAPH_PIN, LOW);
  #endif

  #if HAS_CASE_LIGHT && DISABLED(CASE_LIGHT_USE_NEOPIXEL)
    SET_OUTPUT(CASE_LIGHT_PIN);
  #endif

  #if HAS_Z_PROBE_SLED
    OUT_WRITE(SLED_PIN, LOW); // turn it off
  #endif

  #if HAS_HOME
    SET_INPUT_PULLUP(HOME_PIN);
  #endif

  #if PIN_EXISTS(STAT_LED_RED)
    OUT_WRITE(STAT_LED_RED_PIN, LOW); // turn it off
  #endif

  #if PIN_EXISTS(STAT_LED_BLUE)
    OUT_WRITE(STAT_LED_BLUE_PIN, LOW); // turn it off
  #endif

}

#if ENABLED(IDLE_OOZING_PREVENT)

  void Printer::IDLE_OOZING_retract(bool retracting) {

    if (retracting && !IDLE_OOZING_retracted[tools.active_extruder]) {

      float old_feedrate_mm_s = mechanics.feedrate_mm_s;

      mechanics.set_destination_to_current();
      mechanics.current_position[E_AXIS] += IDLE_OOZING_LENGTH
        #if ENABLED(VOLUMETRIC_EXTRUSION)
          / tools.volumetric_multiplier[tools.active_extruder]
        #endif
      ;
      mechanics.feedrate_mm_s = IDLE_OOZING_FEEDRATE;
      planner.set_e_position_mm(mechanics.current_position[E_AXIS]);
      mechanics.prepare_move_to_destination();
      mechanics.feedrate_mm_s = old_feedrate_mm_s;
      IDLE_OOZING_retracted[tools.active_extruder] = true;
      //SERIAL_EM("-");
    }
    else if (!retracting && IDLE_OOZING_retracted[tools.active_extruder]) {

      float old_feedrate_mm_s = mechanics.feedrate_mm_s;

      mechanics.set_destination_to_current();
      mechanics.current_position[E_AXIS] -= (IDLE_OOZING_LENGTH + IDLE_OOZING_RECOVER_LENGTH)
        #if ENABLED(VOLUMETRIC_EXTRUSION)
          / tools.volumetric_multiplier[tools.active_extruder]
        #endif
      ;

      mechanics.feedrate_mm_s = IDLE_OOZING_RECOVER_FEEDRATE;
      planner.set_e_position_mm(mechanics.current_position[E_AXIS]);
      mechanics.prepare_move_to_destination();
      mechanics.feedrate_mm_s = old_feedrate_mm_s;
      IDLE_OOZING_retracted[tools.active_extruder] = false;
      //SERIAL_EM("+");
    }
  }

#endif

/**
 * Flags Function
 */
void Printer::setDebugLevel(const uint8_t newLevel) {
  if (newLevel != mk_debug_flag) {
    mk_debug_flag = newLevel;
    if (debugDryrun() || debugSimulation()) {
      // Disable all heaters in case they were on
      thermalManager.disable_all_heaters();
    }
  }
  SERIAL_EMV("DebugLevel:", (int)mk_debug_flag);
}

#if ENABLED(HOST_KEEPALIVE_FEATURE)

  /**
   * Output a "busy" message at regular intervals
   * while the machine is not accepting
   */
  void Printer::keepalive(const MK4duoBusyState state) {
    if (!isSuspendAutoreport() && host_keepalive_watch.stopwatch && host_keepalive_watch.elapsed()) {
      switch (state) {
        case InHandler:
        case InProcess:
          SERIAL_LM(BUSY, MSG_BUSY_PROCESSING);
          break;
        case WaitHeater:
          SERIAL_LM(BUSY, MSG_BUSY_WAIT_HEATER);
          break;
        case DoorOpen:
          SERIAL_LM(BUSY, MSG_BUSY_DOOR_OPEN);
          break;
        case PausedforUser:
          SERIAL_LM(BUSY, MSG_BUSY_PAUSED_FOR_USER);
          break;
        case PausedforInput:
          SERIAL_LM(BUSY, MSG_BUSY_PAUSED_FOR_INPUT);
          break;
        default:
          break;
      }
      host_keepalive_watch.start();
    }
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
