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

/**
 * printer.cpp
 *
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"

const char axis_codes[XYZE] = {'X', 'Y', 'Z', 'E'};

Printer printer;

flagdebug_t   Printer::debug_flag;    // For debug
flagVarious_t Printer::various_flag;  // For various

bool Printer::axis_relative_modes[] = AXIS_RELATIVE_MODES;

// Print status related
int16_t Printer::currentLayer   = 0,
        Printer::maxLayer       = -1;   // -1 = unknown
char    Printer::printName[21]  = "";   // max. 20 chars + 0
uint8_t Printer::progress       = 0;

// Inactivity shutdown
uint8_t   Printer::safety_time        = SAFETYTIMER_TIME_MINS,
          Printer::max_inactive_time  = 0,
          Printer::move_time          = DEFAULT_STEPPER_DEACTIVE_TIME;
millis_l  Printer::max_inactivity_ms  = 0,
          Printer::move_ms            = 0;

#if ENABLED(HOST_KEEPALIVE_FEATURE)
  BusyStateEnum Printer::busy_state     = NotBusy;
  uint8_t Printer::host_keepalive_time  = DEFAULT_KEEPALIVE_INTERVAL;
#endif

// Interrupt Event
InterruptEventEnum Printer::interruptEvent = INTERRUPT_EVENT_NONE;

// Printer mode
PrinterModeEnum Printer::mode =
  #if ENABLED(PLOTTER)
    PRINTER_MODE_PLOTTER;
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

#if ENABLED(BARICUDA)
  int Printer::baricuda_valve_pressure  = 0,
      Printer::baricuda_e_to_p_pressure = 0;
#endif

#if ENABLED(IDLE_OOZING_PREVENT)
  bool  Printer::IDLE_OOZING_enabled = true,
        Printer::IDLE_OOZING_retracted[EXTRUDERS] = ARRAY_BY_EXTRUDERS(false);
#endif

#if HAS_CHDK
  millis_s Printer::chdk_ms = 0;
#endif

/** Public Function */

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

  #if HAS_POWER_CHECK || HAS_POWER_SWITCH
    powerManager.init();
  #endif

  #if MB(ALLIGATOR_R2) || MB(ALLIGATOR_R3)
    HAL::spiBegin();
  #endif

  #if HAS_STEPPER_RESET
    stepper.disableStepperDrivers();
  #endif

  // Init Serial for HOST
  Com::setBaudrate();

  // Check startup
  SERIAL_L(START);
  SERIAL_STR(ECHO);

  #if HAS_TRINAMIC
    tmc.init();
  #endif

  #if MECH(MUVE3D) && ENABLED(PROJECTOR_PORT) && ENABLED(PROJECTOR_BAUDRATE)
    DLPSerial.begin(PROJECTOR_BAUDRATE);
  #endif

  // Check startup - does nothing if bootloader sets MCUSR to 0
  HAL::showStartReason();

  SERIAL_LM(ECHO, BUILD_VERSION);

  #if ENABLED(STRING_DISTRIBUTION_DATE) && ENABLED(STRING_CONFIG_H_AUTHOR)
    SERIAL_LM(ECHO, MSG_CONFIGURATION_VER STRING_DISTRIBUTION_DATE MSG_AUTHOR STRING_CONFIG_H_AUTHOR);
    SERIAL_LM(ECHO, MSG_COMPILED __DATE__);
  #endif // STRING_DISTRIBUTION_DATE

  SERIAL_SMV(ECHO, MSG_FREE_MEMORY, HAL::getFreeRam());
  SERIAL_EMV(MSG_PLANNER_BUFFER_BYTES, (int)sizeof(block_t)*BLOCK_BUFFER_SIZE);

  #if HAS_SD_SUPPORT
    if (!card.isDetected()) card.mount();
  #endif

  // Init endstops
  endstops.init();

  // Init Filament runout
  #if HAS_FILAMENT_SENSOR
    filamentrunout.init();
  #endif

  // Initial setup of print job counter
  print_job_counter.init();

  // Load data from EEPROM if available (or use defaults)
  // This also updates variables in the planner, elsewhere
  bool eeprom_loaded = eeprom.load();

  #if ENABLED(WORKSPACE_OFFSETS)
    // Initialize current position based on data.home_offset
    COPY_ARRAY(mechanics.current_position, mechanics.data.home_offset);
  #else
    ZERO(mechanics.current_position);
  #endif

  // Vital to init stepper/planner equivalent for current_position
  mechanics.sync_plan_position();

  // Initialize temperature loop
  thermalManager.init();

  // Initialize stepper. This enables interrupts!
  stepper.init();

  #if ENABLED(CNCROUTER)
    cnc.init();
  #endif

  // Initialize all Servo
  #if HAS_SERVOS
    servo_init();
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
    flowmeter.init();
  #endif

  #if ENABLED(PCF8574_EXPANSION_IO)
    pcf8574.begin();
  #endif

  #if ENABLED(RFID_MODULE)
    setRfid(rfid522.init());
    if (IsRfid()) SERIAL_EM("RFID CONNECT");
  #endif

  lcdui.init();
  lcdui.reset_status();

  // Show MK4duo boot screen
  #if HAS_SPI_LCD && ENABLED(SHOW_BOOTSCREEN)
    lcdui.show_bootscreen();
  #endif

  #if ENABLED(COLOR_MIXING_EXTRUDER) && MIXING_VIRTUAL_TOOLS > 1
    mixer.init();
  #endif

  #if ENABLED(BLTOUCH)
    bltouch.init();
  #endif

  // All Initialized set Running to true.
  setRunning(true);

  #if ENABLED(DELTA_HOME_ON_POWER)
    mechanics.home();
  #endif

  zero_fan_speed();

  #if HAS_LCD_MENU && HAS_EEPROM
    if (!eeprom_loaded) lcdui.goto_screen(menu_eeprom);
  #endif

  #if HAS_SD_RESTART
    restart.check();
  #endif

  // Init Watchdog
  watchdog.init();

  #if HAS_TRINAMIC && !PS_DEFAULT_OFF
    tmc.test_connection(true, true, true, true);
  #endif

  #if HAS_MMU2
    mmu2.init();
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

  for (;;) {

    #if HAS_SD_SUPPORT

      card.checkautostart();

      if (card.isAbortSDprinting()) {
        card.setAbortSDprinting(false);

        #if HAS_SD_RESTART
          // Save Job for restart
          if (IS_SD_PRINTING()) restart.save_job(true);
        #endif

        // Stop SD printing
        card.stopSDPrint();

        // Clear all command in quee
        commands.clear_queue();

        // Stop printer job timer
        print_job_counter.stop();

        // Auto home
        #if Z_HOME_DIR > 0
          commands.enqueue_and_echo_P(PSTR("G28"));
        #else
          commands.enqueue_and_echo_P(PSTR("G28 X Y"));
        #endif

        // Disabled Heaters and Fan
        thermalManager.disable_all_heaters();
        zero_fan_speed();
        setWaitForHeatUp(false);

      }

    #endif // HAS_SD_SUPPORT

    commands.get_available();
    commands.advance_queue();
    endstops.report_state();
    idle();

  }
}

void Printer::check_periodical_actions() {

  static millis_s cycle_1s_ms = 0;

  // Event 1.0 Second
  if (expired(&cycle_1s_ms, 1000U)) {

    planner.check_axes_activity();

    if (!isSuspendAutoreport() && isAutoreportTemp()) {
      thermalManager.report_temperatures();
      SERIAL_EOL();
    }

    #if HAS_SD_SUPPORT
      if (card.isAutoreportSD()) card.printStatus();
    #endif

    if (planner.cleaning_buffer_flag) {
      planner.cleaning_buffer_flag = false;
      #if ENABLED(SD_FINISHED_STEPPERRELEASE) && ENABLED(SD_FINISHED_RELEASECOMMAND)
        commands.enqueue_and_echo_P(PSTR(SD_FINISHED_RELEASECOMMAND));
      #endif
    }

    #if FAN_COUNT > 0
      LOOP_FAN() fans[f].spin();
    #endif

    #if HAS_POWER_SWITCH
      powerManager.spin();
    #endif

    #if ENABLED(FLOWMETER_SENSOR)
      flowmeter.spin();
    #endif
  }

}

void Printer::safe_delay(millis_l ms) {
  while (ms > 50) {
    ms -= 50;
    HAL::delayMilliseconds(50);
    check_periodical_actions();
  }
  HAL::delayMilliseconds(ms);
  check_periodical_actions();
}

void Printer::quickstop_stepper() {
  planner.quick_stop();
  planner.synchronize();
  mechanics.set_current_from_steppers_for_axis(ALL_AXES);
  mechanics.sync_plan_position();
}

/**
 * Kill all activity and lock the machine.
 * After this the machine will need to be reset.
 */
void Printer::kill(PGM_P const lcd_msg/*=nullptr*/) {

  thermalManager.disable_all_heaters();

  SERIAL_LM(ER, MSG_ERR_KILLED);

  #if HAS_LCD
    lcdui.kill_screen(lcd_msg ? lcd_msg : PSTR(MSG_KILLED));
  #else
    UNUSED(lcd_msg);
  #endif

  host_action.power_off();

  minikill();
}

void Printer::minikill() {

  // Wait a short time (allows messages to get out before shutting down.
  for (int i = 1000; i--;) HAL::delayMicroseconds(600);

  DISABLE_ISRS();  // Stop interrupts

  // Wait to ensure all interrupts routines stopped
  for (int i = 1000; i--;) HAL::delayMicroseconds(250);

  // Turn off heaters again
  thermalManager.disable_all_heaters(); 

  #if ENABLED(FLOWMETER_SENSOR) && ENABLED(MINFLOW_PROTECTION)
    flowmeter.flow_firstread = false;
  #endif

  #if HAS_POWER_SWITCH
    powerManager.power_off();
  #endif

  #if HAS_SUICIDE
    suicide();
  #endif

  #if ENABLED(KILL_METHOD) && (KILL_METHOD == 1)
    HAL::resetHardware();
  #endif

  #if ENABLED(LASER)
    laser.init();
    #if ENABLED(LASER_PERIPHERALS)
      laser.peripherals_off();
    #endif
  #endif

  #if ENABLED(CNCROUTER)
    cnc.disable_router();
  #endif

  #if HAS_KILL

    // Wait for kill to be released
    while (!READ(KILL_PIN)) watchdog.reset();

    // Wait for kill to be pressed
    while (READ(KILL_PIN)) watchdog.reset();

    void(*resetFunc)(void) = 0; // Declare resetFunc() at address 0
    resetFunc();                // Jump to address 0

  #else // !HAS_KILL

    // Wait for reset
    for (;;) watchdog.reset();

  #endif // !HAS_KILL

}

/**
 * Turn off heaters and stop the print in progress
 * After a stop the machine may be resumed with M999
 */
void Printer::stop() {

  thermalManager.disable_all_heaters();

  #if ENABLED(PROBING_FANS_OFF)
    LOOP_FAN() {
      if (fans[f].isIdle()) fans[f].setIdle(false); // put things back the way they were
    }
  #endif

  #if ENABLED(FLOWMETER_SENSOR) && ENABLED(MINFLOW_PROTECTION)
    flowmeter.flow_firstread = false;
  #endif

  #if ENABLED(LASER)
    if (laser.diagnostics) SERIAL_EM("Laser set to off, stop() called");
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

  // LCD Update
  lcdui.update();

  #if ENABLED(HOST_KEEPALIVE_FEATURE)
    host_keepalive_tick();
  #endif

  // Control interrupt events
  handle_interrupt_events();

  // Tick timer job counter
  print_job_counter.tick();

  check_periodical_actions();

  commands.get_available();

  handle_safety_watch();

  if (expired(&max_inactivity_ms, millis_l(max_inactive_time * 1000UL))) {
    SERIAL_LMT(ER, MSG_KILL_INACTIVE_TIME, parser.command_ptr);
    kill(PSTR(MSG_KILLED));
  }

  sound.spin();

  #if HAS_MAX31855 || HAS_MAX6675
    thermalManager.getTemperature_SPI();
  #endif

  #if ENABLED(DHT_SENSOR)
    dhtsensor.spin();
  #endif

  #if ENABLED(CNCROUTER)
    cnc.manage();
  #endif

  #if HAS_FILAMENT_SENSOR
    filamentrunout.spin();
  #endif

  #if ENABLED(RFID_MODULE)
    rfid522.spin();
  #endif

  #if ENABLED(BABYSTEPPING)
    babystep.spin();
  #endif

  // Prevent steppers timing-out in the middle of M600
  #if ENABLED(ADVANCED_PAUSE_FEATURE) && ENABLED(PAUSE_PARK_NO_STEPPER_TIMEOUT)
    #define MOVE_AWAY_TEST !advancedpause.did_pause_print
  #else
    #define MOVE_AWAY_TEST true
  #endif

  if (move_time) {
    static bool already_shutdown_steppers; // = false
    if (planner.has_blocks_queued())
      reset_move_ms();  // reset stepper move watch to keep steppers powered
    else if (MOVE_AWAY_TEST && !ignore_stepper_queue && expired(&move_ms, millis_l(move_time * 1000UL))) {
      if (!already_shutdown_steppers) {
        if (printer.debugFeature()) DEBUG_EM("Stepper shutdown");
        already_shutdown_steppers = true; 
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
        #if HAS_LCD_MENU && ENABLED(AUTO_BED_LEVELING_UBL)
          if (ubl.lcd_map_control) {
            ubl.lcd_map_control = false;
            lcdui.defer_status_screen(false);
          }
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
    else
      already_shutdown_steppers = false;
  }

  #if HAS_CHDK // Check if pin should be set to LOW (after M240 set it HIGH)
    if (expired(&chdk_ms, millis_s(PHOTO_SWITCH_MS))) WRITE(CHDK_PIN, LOW);
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
    if (!IS_SD_PRINTING() && !READ(HOME_PIN)) {
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

    static millis_s extruder_runout_ms = 0;

    if (hotends[ACTIVE_HOTEND].current_temperature > EXTRUDER_RUNOUT_MINTEMP
      && expired(&extruder_runout_ms, millis_s(EXTRUDER_RUNOUT_SECONDS * 1000U))
      && !planner.has_blocks_queued()
    ) {
      #if ENABLED(DONDOLO_SINGLE_MOTOR)
        const bool oldstatus = E0_ENABLE_READ();
        enable_E0();
      #else // !DONDOLO_SINGLE_MOTOR
        bool oldstatus;
        switch (tools.active_extruder) {
          case 0: oldstatus = E0_ENABLE_READ(); enable_E0(); break;
          #if DRIVER_EXTRUDERS > 1
            case 1: oldstatus = E1_ENABLE_READ(); enable_E1(); break;
            #if DRIVER_EXTRUDERS > 2
              case 2: oldstatus = E2_ENABLE_READ(); enable_E2(); break;
              #if DRIVER_EXTRUDERS > 3
                case 3: oldstatus = E3_ENABLE_READ(); enable_E3(); break;
                #if DRIVER_EXTRUDERS > 4
                  case 4: oldstatus = E4_ENABLE_READ(); enable_E4(); break;
                  #if DRIVER_EXTRUDERS > 5
                    case 5: oldstatus = E5_ENABLE_READ(); enable_E5(); break;
                  #endif // DRIVER_EXTRUDERS > 5
                #endif // DRIVER_EXTRUDERS > 4
              #endif // DRIVER_EXTRUDERS > 3
            #endif // DRIVER_EXTRUDERS > 2
          #endif // DRIVER_EXTRUDERS > 1
        }
      #endif // !DONDOLO_SINGLE_MOTOR

      const float olde = mechanics.current_position[E_AXIS];
      mechanics.current_position[E_AXIS] += EXTRUDER_RUNOUT_EXTRUDE;
      planner.buffer_line(mechanics.current_position, MMM_TO_MMS(EXTRUDER_RUNOUT_SPEED), tools.active_extruder);
      mechanics.current_position[E_AXIS] = olde;
      planner.set_e_position_mm(olde);
      planner.synchronize();
      #if ENABLED(DONDOLO_SINGLE_MOTOR)
        E0_ENABLE_WRITE(oldstatus);
      #else
        switch (tools.active_extruder) {
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
    if (mechanics.delayed_move_ms && expired(&mechanics.delayed_move_ms, 1000U) && isRunning()) {
      // travel moves have been received so enact them
      mechanics.delayed_move_ms = 0xFFFFU; // force moves to be done
      mechanics.set_destination_to_current();
      mechanics.prepare_move_to_destination();
    }
  #endif

  #if ENABLED(IDLE_OOZING_PREVENT)
    static millis_s axis_last_activity_ms = 0;
    if (planner.has_blocks_queued()) axis_last_activity_ms = millis();
    if (hotends[ACTIVE_HOTEND].current_temperature > IDLE_OOZING_MINTEMP && !debugDryrun() && IDLE_OOZING_enabled) {
      if (hotends[ACTIVE_HOTEND].target_temperature < IDLE_OOZING_MINTEMP)
        IDLE_OOZING_retract(false);
      else if (expired(&axis_last_activity_ms, millis_s(IDLE_OOZING_SECONDS * 1000U)))
        IDLE_OOZING_retract(true);
    }
  #endif

  #if ENABLED(TEMP_STAT_LEDS)
    handle_status_leds();
  #endif

  #if ENABLED(MONITOR_DRIVER_STATUS)
    tmc.monitor_driver();
  #endif

  #if HAS_MMU2
    mmu2.mmu_loop();
  #endif

  // Reset the watchdog
  watchdog.reset();

}

void Printer::setInterruptEvent(const InterruptEventEnum event) {
  if (interruptEvent == INTERRUPT_EVENT_NONE)
    interruptEvent = event;
}

/**
 * isPrinting check
 */
bool Printer::isPrinting()  { return IS_SD_PRINTING() || print_job_counter.isRunning(); }
bool Printer::isPaused()    { return IS_SD_PAUSED()   || print_job_counter.isPaused();  }

/**
 * Sensitive pin test for M42, M226
 */
bool Printer::pin_is_protected(const pin_t pin) {
  static const int8_t sensitive_pins[] PROGMEM = SENSITIVE_PINS;
  for (uint8_t i = 0; i < COUNT(sensitive_pins); i++)
    if (pin == pgm_read_byte(&sensitive_pins[i])) return true;
  return false;
}

#if HAS_SUICIDE
  void Printer::suicide() { OUT_WRITE(SUICIDE_PIN, LOW); }
#endif

/**
 * Debug Flags Function
 */
void Printer::setDebugLevel(const uint8_t newLevel) {
  if (newLevel != debug_flag.all) {
    debug_flag.all = newLevel;
    if (debugDryrun() || debugSimulation()) {
      // Disable all heaters in case they were on
      thermalManager.disable_all_heaters();
    }
  }
  SERIAL_EMV("DebugLevel:", (int)debug_flag.all);
}

/** Private Function */
void Printer::setup_pinout() {

  #if PIN_EXISTS(SS)
    OUT_WRITE(SS_PIN, HIGH);
  #endif

  #if ENABLED(LCD_SDSS) && LCD_SDSS >= 0
    OUT_WRITE(LCD_SDSS, HIGH);
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

void Printer::handle_interrupt_events() {

  if (interruptEvent == INTERRUPT_EVENT_NONE) return; // Exit if none Event

  switch (interruptEvent) {

    #if HAS_FILAMENT_SENSOR

      case INTERRUPT_EVENT_FIL_RUNOUT: {

        #if ENABLED(ADVANCED_PAUSE_FEATURE)
          if (advancedpause.did_pause_print) return;
        #endif

        const char tool = '0' + tools.active_extruder;

        filamentrunout.setFilamentOut(true);
        host_action.prompt_reason = PROMPT_FILAMENT_RUNOUT;
        host_action.prompt_begin(PSTR("Filament Runout T"), false);
        SERIAL_CHR(tool);
        SERIAL_EOL();
        host_action.prompt_show();

        const bool run_runout_script = !filamentrunout.isHostHandling();

        if (run_runout_script
          && ( strstr(FILAMENT_RUNOUT_SCRIPT, "M600")
            || strstr(FILAMENT_RUNOUT_SCRIPT, "M125")
            #if ENABLED(ADVANCED_PAUSE_FEATURE)
              || strstr(FILAMENT_RUNOUT_SCRIPT, "M25")
            #endif
          )
        )
          host_action.paused(false);
        else
          host_action.pause(false);

        SERIAL_SM(ECHO, "filament runout T");
        SERIAL_CHR(tool);
        SERIAL_EOL();

        if (run_runout_script)
          commands.enqueue_and_echo_P(PSTR(FILAMENT_RUNOUT_SCRIPT));

        break;
      }

    #endif // HAS_FILAMENT_SENSOR

    default: break;

  }

  interruptEvent = INTERRUPT_EVENT_NONE;

}

/**
 * Turn off heating after 30 minutes of inactivity
 */
void Printer::handle_safety_watch() {

  static millis_l safety_ms = 0;

  if (isPrinting() || isPaused() || !thermalManager.heaters_isActive())
    safety_ms = 0;
  else if (!safety_ms && thermalManager.heaters_isActive())
    safety_ms = millis();
  else if (safety_ms && expired(&safety_ms, millis_l(safety_time * 60000UL))) {
    thermalManager.disable_all_heaters();
    SERIAL_EM("Heating disabled by safety timer.");
    lcdui.set_status_P(PSTR(MSG_MAX_INACTIVITY_TIME), 99);
  }
}

#if ENABLED(HOST_KEEPALIVE_FEATURE)

  /**
   * Output a "busy" message at regular intervals
   * while the machine is not accepting
   */
  void Printer::host_keepalive_tick() {
    static millis_s host_keepalive_ms = millis();
    if (!isSuspendAutoreport() && expired(&host_keepalive_ms, millis_s(host_keepalive_time * 1000U))) {
      switch (busy_state) {
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
    }
  }

#endif // HOST_KEEPALIVE_FEATURE

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

#if ENABLED(TEMP_STAT_LEDS)

  void Printer::handle_status_leds() {

    static bool red_led = false;
    static millis_s next_status_led_update_ms;

    // Update every 0.5s
    if (expired(&next_status_led_update_ms, 500U)) {
      float max_temp = 0.0;
      #if CHAMBERS > 0
        LOOP_CHAMBER()
          max_temp = MAX(max_temp, chambers[h].target_temperature, chambers[h].current_temperature);
      #endif
      #if BEDS > 0
        LOOP_BED()
          max_temp = MAX(max_temp, beds[h].target_temperature, beds[h].current_temperature);
      #endif
      LOOP_HOTEND()
        max_temp = MAX(max_temp, hotends[h].current_temperature, hotends[h].target_temperature);
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
