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

/**
 * printer.cpp
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"
#include "sanitycheck.h"

Printer printer;

debug_flag_t    Printer::debug_flag;    // For debug
various_flag_t  Printer::various_flag;  // For various

// Print status related
int16_t Printer::currentLayer   = 0,
        Printer::maxLayer       = -1;   // -1 = unknown
char    Printer::printName[21]  = "";   // max. 20 chars + 0
uint8_t Printer::progress       = 0;

// Inactivity shutdown
uint16_t  Printer::safety_time        = SAFETYTIMER_TIME_MINS,
          Printer::max_inactive_time  = 0,
          Printer::move_time          = DEFAULT_STEPPER_DEACTIVE_TIME;

long_timer_t  Printer::max_inactivity_timer,
              Printer::move_timer;

#if ENABLED(HOST_KEEPALIVE_FEATURE)
  BusyStateEnum Printer::busy_state     = NotBusy;
  uint8_t Printer::host_keepalive_time  = DEFAULT_KEEPALIVE_INTERVAL;
#endif

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

#if HAS_CHDK
  short_timer_t Printer::chdk_timer;
#endif

/** Public Function */
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

  // Init CS for TMC SPI
  #if TMC_HAS_SPI
    tmcManager.init_cs_pins();
  #endif

  #if ENABLED(MKR4) // MKR4 System
    #if HAS_E0E1
      OUT_WRITE_RELE(E0E1_CHOICE_PIN, LOW);
    #endif
    #if HAS_E0E2
      OUT_WRITE_RELE(E0E2_CHOICE_PIN, LOW);
    #endif
    #if HAS_E1E3
      OUT_WRITE_RELE(E1E3_CHOICE_PIN, LOW);
    #endif
  #elif ENABLED(MKR6) || ENABLED(MKR12) // MKR6 or MKR12 System
    #if HAS_EX1
      OUT_WRITE_RELE(EX1_CHOICE_PIN, LOW);
    #endif
    #if HAS_EX2
      OUT_WRITE_RELE(EX2_CHOICE_PIN, LOW);
    #endif
  #endif

}

void Printer::factory_parameters() {
  various_flag.all = 0x0000;

  #if HAS_SERVOS
    #if HAS_DONDOLO
      servo[DONDOLO_SERVO_INDEX].angle[0] = DONDOLO_SERVOPOS_E0;
      servo[DONDOLO_SERVO_INDEX].angle[1] = DONDOLO_SERVOPOS_E1;
    #endif
    #if HAS_Z_SERVO_PROBE
      constexpr uint8_t z_probe_angles[2] = PROBE_SERVO_ANGLES;
      servo[PROBE_SERVO_NR].angle[0] = z_probe_angles[0];
      servo[PROBE_SERVO_NR].angle[1] = z_probe_angles[1];
    #endif
  #endif // HAS_SERVOS
}

void Printer::check_periodical_actions() {

  planner.check_axes_activity();

  if (!isSuspendAutoreport() && isAutoreportTemp()) {
    #if HAS_HEATER
      tempManager.report_temperatures();
    #endif
    #if HAS_FAN
      fanManager.report_speed();
    #endif
    SERIAL_EOL();
  }

  #if HAS_SD_SUPPORT
    if (card.isAutoreport()) card.print_status();
  #endif

  if (planner.flag.clean_buffer)
    planner.flag.clean_buffer = false;

  fanManager.spin();

  #if HAS_POWER_SWITCH
    powerManager.spin();
  #endif

  #if ENABLED(FLOWMETER_SENSOR)
    flowmeter.spin();
  #endif

}

void Printer::safe_delay(millis_l time) {
  time += millis();
  while (PENDING(millis(), time)) idle();
}

void Printer::quickstop_stepper() {
  planner.quick_stop();
  planner.synchronize();
  mechanics.set_position_from_steppers_for_axis(ALL_AXES);
  mechanics.sync_plan_position();
}

/**
 * Kill all activity and lock the machine.
 * After this the machine will need to be reset.
 */
void Printer::kill(PGM_P const lcd_msg/*=nullptr*/, const bool steppers_off/*=false*/) {

  tempManager.disable_all_heaters();

  SERIAL_LM(ER, STR_ERR_KILLED);

  #if HAS_LCD
    lcdui.kill_screen(lcd_msg ? lcd_msg : GET_TEXT(MSG_KILLED));
  #else
    UNUSED(lcd_msg);
  #endif

  host_action.power_off();

  minikill(steppers_off);
}

void Printer::minikill(const bool steppers_off/*=false*/) {

  // Wait a short time (allows messages to get out before shutting down.
  for (int i = 1000; i--;) HAL::delayMicroseconds(600);

  DISABLE_ISRS();  // Stop interrupts

  // Wait to ensure all interrupts routines stopped
  for (int i = 1000; i--;) HAL::delayMicroseconds(250);

  // Turn off heaters again
  tempManager.disable_all_heaters(); 

  // Power off all steppers (for M112) or just the E steppers
  steppers_off ? stepper.disable_all() : stepper.disable_E();

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
    while (kill_state()) watchdog.reset();

    // Wait for kill to be pressed
    while (!kill_state()) watchdog.reset();

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

  tempManager.disable_all_heaters();

  #if ENABLED(PROBING_FANS_OFF)
    LOOP_FAN() {
      if (fans[f]->isIdle()) fans[f]->setIdle(false); // put things back the way they were
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
    SERIAL_LM(ER, STR_ERR_STOPPED);
    LCD_MESSAGEPGM(MSG_STOPPED);
  }
}

void Printer::zero_fan_speed() {
  #if HAS_FAN
    LOOP_FAN() fans[f]->speed = 0;
  #endif
}

/**
 * Manage several activities:
 *  - Lcd update
 *  - Keep the command buffer full
 *  - Host Keepalive
 *  - DHT spin
 *  - Cnc manage
 *  - Filament Runout spin
 *  - Read o Write Rfid
 *  - Check for maximum inactive time between commands
 *  - Check for maximum inactive time between stepper commands
 *  - Check if pin CHDK needs to go LOW
 *  - Check for KILL button held down
 *  - Check for HOME button held down
 *  - Check if cooling fan needs to be switched on
 *  - Check if an idle but hot extruder needs filament extruded (EXTRUDER_RUNOUT_PREVENT)
 *  - Check oozing prevent
 */
void Printer::idle(const bool no_stepper_sleep/*=false*/) {

  #if ENABLED(SPI_ENDSTOPS)
    if (endstops.tmc_spi_homing.any
      #if ENABLED(IMPROVE_HOMING_RELIABILITY)
        && ELAPSED(millis(), tmcManager.sg_guard_period)
      #endif
    ) {
      for (uint8_t i = 4; i--;) // Read SGT 4 times per idle loop
        if (endstops.tmc_spi_homing_check()) break;
    }
  #endif

  #if HAS_SD_SUPPORT
    card.manage_sd();
  #endif

  lcdui.update();

  #if HAS_POWER_CHECK
    powerManager.outage();
  #endif

  #if ENABLED(HOST_KEEPALIVE_FEATURE)
    host_keepalive_tick();
  #endif

  // Tick timer job counter
  print_job_counter.tick();

  commands.get_available();

  handle_safety_watch();

  if (max_inactivity_timer.expired(SECOND_TO_MILLIS(max_inactive_time))) {
    SERIAL_LMT(ER, STR_KILL_INACTIVE_TIME, parser.command_ptr);
    kill(GET_TEXT(MSG_KILLED));
  }

  sound.spin();

  #if HAS_MAX31855 || HAS_MAX6675
    tempManager.getTemperature_SPI();
  #endif

  #if HAS_DHT
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
      reset_move_timer();  // reset stepper move watch to keep steppers powered
    else if (MOVE_AWAY_TEST && !no_stepper_sleep && move_timer.expired(SECOND_TO_MILLIS(move_time), false)) {
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
        #if HAS_LCD_MENU && HAS_UBL
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
    if (chdk_timer.expired(PHOTO_SWITCH_MS)) WRITE(CHDK_PIN, LOW);
  #endif

  #if HAS_KILL

    // Check if the kill button was pressed and wait just in case it was an accidental
    // key kill key press
    // -------------------------------------------------------------------------------
    static int killCount = 0;   // make the inactivity button a bit less responsive
    const int KILL_DELAY = 750;
    if (kill_state())
       killCount++;
    else if (killCount > 0)
       killCount--;

    // Exceeded threshold and we can confirm that it was not accidental
    // KILL the machine
    // ----------------------------------------------------------------
    if (killCount >= KILL_DELAY) {
      SERIAL_LM(ER, STR_KILL_BUTTON);
      kill(GET_TEXT(MSG_KILLED));
    }
  #endif

  #if HAS_HOME
    // Handle a standalone HOME button
    static long_timer_t next_home_key_timer(millis());
    if (!IS_SD_PRINTING() && !READ(HOME_PIN)) {
      if (next_home_key_timer.expired(HOME_DEBOUNCE_DELAY)) {
        LCD_MESSAGEPGM(MSG_AUTO_HOME);
        commands.enqueue_now_P(G28_CMD);
      }
    }
  #endif

  #if ENABLED(EXTRUDER_RUNOUT_PREVENT)
    static long_timer_t extruder_runout_timer(millis());
    if (hotends[toolManager.active_hotend()]->deg_current() > EXTRUDER_RUNOUT_MINTEMP
      && extruder_runout_timer.expired(SECOND_TO_MILLIS(EXTRUDER_RUNOUT_SECONDS))
      && !planner.has_blocks_queued()
    ) {
      const float olde = mechanics.position.e;
      mechanics.position.e += EXTRUDER_RUNOUT_EXTRUDE;
      mechanics.line_to_position(MMM_TO_MMS(EXTRUDER_RUNOUT_SPEED));
      mechanics.position.e = olde;
      planner.set_e_position_mm(olde);
      planner.synchronize();
    }
  #endif // EXTRUDER_RUNOUT_PREVENT

  #if ENABLED(DUAL_X_CARRIAGE)
    // handle delayed move timeout
    if (mechanics.delayed_move_timer.expired(SECOND_TO_MILLIS(1), false) && isRunning()) {
      // travel moves have been received so enact them
      mechanics.destination = mechanics.position;
      mechanics.prepare_move_to_destination();
    }
  #endif

  #if ENABLED(IDLE_OOZING_PREVENT)
    static long_timer_t axis_last_activity_timer;
    if (planner.has_blocks_queued()) axis_last_activity_timer.start();
    if (hotends[toolManager.active_hotend()]->deg_current() > IDLE_OOZING_MINTEMP && !debugDryrun() && toolManager.IDLE_OOZING_enabled) {
      if (hotends[toolManager.active_hotend()]->deg_target() < IDLE_OOZING_MINTEMP)
        toolManager.IDLE_OOZING_retract(false);
      else if (axis_last_activity_timer.expired(SECOND_TO_MILLIS(IDLE_OOZING_SECONDS)))
        toolManager.IDLE_OOZING_retract(true);
    }
  #endif

  #if ENABLED(TEMP_STAT_LEDS)
    handle_status_leds();
  #endif

  #if ENABLED(MONITOR_DRIVER_STATUS)
    tmcManager.monitor_drivers();
  #endif

  #if HAS_MMU2
    mmu2.mmu_loop();
  #endif

  watchdog.reset();

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

void Printer::print_M353() {
  SERIAL_LM(CFG, "Total number D<driver extruder> E<Extruder> H<Hotend> B<Bed> C<Chamber> <Fan>");
  SERIAL_SMV(CFG,"  M353 D", stepper.data.drivers_e);
  SERIAL_MV(" E", toolManager.extruder.total);
  SERIAL_MV(" H", tempManager.heater.hotends);
  SERIAL_MV(" B", tempManager.heater.beds);
  SERIAL_MV(" C", tempManager.heater.chambers);
  SERIAL_MV(" F", fanManager.data.fans);
  SERIAL_EOL();
}

#if HAS_SD_SUPPORT

  void Printer::abort_sd_printing() {

    card.setAbortSDprinting(false);

    #if HAS_SD_RESTART
      // Save Job for restart
      if (restart.enabled && IS_SD_PRINTING()) restart.save_job();
    #endif

    // End File print
    card.endFilePrint();

    // Clear all command in queue
    commands.clear_queue();

    // Stop printer job timer
    print_job_counter.stop();

    // Auto home
    #if Z_HOME_DIR > 0
      mechanics.home();
    #else
      mechanics.home(HOME_X | HOME_Y);
    #endif

    // Disabled Heaters and Fan
    tempManager.disable_all_heaters();
    zero_fan_speed();
    setWaitForHeatUp(false);
  }

  void Printer::finish_sd_printing() {
    if (commands.enqueue_one_P(PSTR("M1001"))) card.setComplete(false);
  }

#endif

#if HAS_RESUME_CONTINUE

  void Printer::wait_for_user_response(millis_l ms/*=0*/, const bool no_sleep/*=false*/) {
    PRINTER_KEEPALIVE(PausedforUser);
    printer.setWaitForUser(true);
    if (ms) ms += millis();
    while (printer.isWaitForUser() && !(ms && ELAPSED(millis(), ms))) idle(no_sleep);
    printer.setWaitForUser(false);
  }

#endif

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
      tempManager.disable_all_heaters();
    }
  }
  SERIAL_EMV("DebugLevel:", (int)debug_flag.all);
}

/** Private Function */

/**
 * Turn off heating after 30 minutes of inactivity
 */
void Printer::handle_safety_watch() {

  static long_timer_t safety_timer;

  if (isPrinting() || isPaused() || !tempManager.heaters_isActive())
    safety_timer.stop();
  else if (!safety_timer.isRunning() && tempManager.heaters_isActive())
    safety_timer.start();
  else if (safety_timer.expired(safety_time * 60000)) {
    tempManager.disable_all_heaters();
    SERIAL_EM(STR_MAX_INACTIVITY_TIME);
    lcdui.set_status_P(GET_TEXT(MSG_MAX_INACTIVITY_TIME), 99);
  }
}

#if ENABLED(HOST_KEEPALIVE_FEATURE)

  /**
   * Output a "busy" message at regular intervals
   * while the machine is not accepting
   */
  void Printer::host_keepalive_tick() {
    static short_timer_t host_keepalive_timer(millis());
    if (!isSuspendAutoreport() && host_keepalive_timer.expired(SECOND_TO_MILLIS(host_keepalive_time)) && busy_state != NotBusy) {
      switch (busy_state) {
        case InHandler:
        case InProcess:
          SERIAL_LM(BUSY, STR_BUSY_PROCESSING);
          break;
        case PausedforUser:
          SERIAL_LM(BUSY, STR_BUSY_PAUSED_FOR_USER);
          break;
        case PausedforInput:
          SERIAL_LM(BUSY, STR_BUSY_PAUSED_FOR_INPUT);
          break;
        case DoorOpen:
          SERIAL_LM(BUSY, STR_BUSY_DOOR_OPEN);
          break;
        default:
          break;
      }
    }
  }

#endif // HOST_KEEPALIVE_FEATURE

#if ENABLED(TEMP_STAT_LEDS)

  void Printer::handle_status_leds() {

    static bool red_led = false;
    static short_timer_t next_status_led_update_timer(millis());

    // Update every 0.5s
    if (next_status_led_update_timer.expired(SECOND_TO_MILLIS(0.5))) {
      float max_temp = 0.0;
      #if HAS_CHAMBERS
        LOOP_CHAMBER()
          max_temp = MAX(max_temp, chambers[h]->deg_target(), chambers[h]->deg_current());
      #endif
      #if HAS_BEDS
        LOOP_BED()
          max_temp = MAX(max_temp, beds[h]->deg_target(), beds[h]->deg_current());
      #endif
      #if HAS_HOTENDS
        LOOP_HOTEND()
          max_temp = MAX(max_temp, hotends[h]->deg_current(), hotends[h]->deg_target());
      #endif
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

/**
 * MK4duo setup(): Set up before the program loop
 *  - Set up Hardware Board
 *  - Set up the kill pin, filament runout, power hold
 *  - Start the serial port
 *  - Print startup messages and diagnostics
 *  - Get EEPROM or default settings
 */
void setup() {

  #if ENABLED(MK4DUO_DEV_MODE)
    auto log_current_msg = [&](PGM_P const msg) {
      SERIAL_STR(ECHO);
      SERIAL_CHR('[');
      SERIAL_VAL(millis());
      SERIAL_MSG("] ");
      SERIAL_STR(msg);
      SERIAL_EOL();
    };
    #define SERIAL_LOG(M)   log_current_msg(PSTR(M))
  #else
    #define SERIAL_LOG(...) NOOP
  #endif
  #define   SERIAL_RUN(C)   do{ SERIAL_LOG(STRINGIFY(C)); C; }while(0)

  HAL::hwSetup();

  #if ENABLED(MB_SETUP)
    MB_SETUP;
  #endif

  printer.setup_pinout();

  #if HAS_POWER_CHECK || HAS_POWER_SWITCH
    powerManager.init();
  #endif

  #if MB(ALLIGATOR_R2) || MB(ALLIGATOR_R3)
    externaldac.begin();
  #elif TMC_HAS_SPI && DISABLED(TMC_USE_SW_SPI)
    SPI.begin();
  #endif

  #if HAS_STEPPER_RESET
    stepper.disableStepperDrivers();
  #endif

  // Init Serial for HOST
  Com::setBaudrate();

  // Check startup
  SERIAL_L(START);
  SERIAL_STR(ECHO);

  #if MECH(MUVE3D) && ENABLED(PROJECTOR_PORT) && ENABLED(PROJECTOR_BAUDRATE)
    DLPSerial.begin(PROJECTOR_BAUDRATE);
  #endif

  // Check startup - does nothing if bootloader sets MCUSR to 0
  HAL::showStartReason();

  SERIAL_LM(ECHO, BUILD_VERSION);

  #if ENABLED(STRING_REVISION_DATE) && ENABLED(STRING_CONFIG_AUTHOR)
    SERIAL_LM(ECHO, STR_CONFIGURATION_VER STRING_REVISION_DATE STR_AUTHOR STRING_CONFIG_AUTHOR);
    SERIAL_LM(ECHO, STR_COMPILED __DATE__);
  #endif // STRING_REVISION_DATE

  SERIAL_SMV(ECHO, STR_FREE_MEMORY, freeMemory());
  SERIAL_EMV(STR_PLANNER_BUFFER_BYTES, (int)sizeof(block_t)* (BLOCK_BUFFER_SIZE));

  #if HAS_SD_SUPPORT
    SERIAL_RUN(card.mount());
  #endif

  // Init endstops
  SERIAL_RUN(endstops.init());

  // Init Filament runout
  #if HAS_FILAMENT_SENSOR
    SERIAL_RUN(filamentrunout.init());
  #endif

  // Initial setup of print job counter
  SERIAL_RUN(print_job_counter.init());

  // Load data from EEPROM if available (or use defaults)
  // This also updates variables in the planner, elsewhere
  bool eeprom_loaded = eeprom.load();

  #if ENABLED(WORKSPACE_OFFSETS)
    // Initialize current position based on data.home_offset
    mechanics.position += mechanics.data.home_offset;
  #else
    mechanics.position.reset();
  #endif

  // Vital to init stepper/planner equivalent for position
  SERIAL_RUN(mechanics.sync_plan_position());

  // Initialize stepper. This enables interrupts!
  SERIAL_RUN(stepper.init());

  #if ENABLED(CNCROUTER)
    SERIAL_RUN(cnc.init());
  #endif

  // Initialize all Servo
  #if HAS_SERVOS
    SERIAL_RUN(servo_init());
  #endif

  #if HAS_CASE_LIGHT
    SERIAL_RUN(caselight.update());
  #endif

  #if HAS_SOFTWARE_ENDSTOPS
    SERIAL_RUN(endstops.setSoftEndstop(true));
  #endif

  #if HAS_STEPPER_RESET
    SERIAL_RUN(stepper.enableStepperDrivers());
  #endif

  #if ENABLED(DIGIPOT_I2C)
    SERIAL_RUN(digipot_i2c_init());
  #endif

  #if HAS_COLOR_LEDS
    SERIAL_RUN(leds.setup());
  #endif

  #if ENABLED(LASER)
    SERIAL_RUN(laser.init());
  #endif

  #if ENABLED(FLOWMETER_SENSOR)
    #if ENABLED(MINFLOW_PROTECTION)
      flowmeter.flow_firstread = false;
    #endif
    SERIAL_RUN(flowmeter.init());
  #endif

  #if ENABLED(PCF8574_EXPANSION_IO)
    SERIAL_RUN(pcf8574.begin());
  #endif

  #if ENABLED(RFID_MODULE)
    SERIAL_RUN(setRfid(rfid522.init()));
    if (IsRfid()) SERIAL_EM("RFID CONNECT");
  #endif

  SERIAL_RUN(lcdui.init());
  SERIAL_RUN(lcdui.reset_status());

  // Show MK4duo boot screen
  #if HAS_SPI_LCD && ENABLED(SHOW_BOOTSCREEN)
    SERIAL_RUN(lcdui.show_bootscreen());
  #endif

  #if ENABLED(COLOR_MIXING_EXTRUDER) && MIXING_VIRTUAL_TOOLS > 1
    SERIAL_RUN(mixer.init());
  #endif

  #if HAS_BLTOUCH
    SERIAL_RUN(bltouch.init(true));
  #endif

  // All Initialized set Running to true.
  SERIAL_RUN(printer.setRunning(true));

  #if ENABLED(DELTA_HOME_ON_POWER)
    SERIAL_RUN(mechanics.home());
  #endif

  SERIAL_RUN(printer.zero_fan_speed());

  #if HAS_LCD_MENU && HAS_EEPROM
    if (!eeprom_loaded) {
      SERIAL_RUN(lcdui.goto_screen(lcd_eeprom_allert));
    }
  #endif

  // Reset Watchdog
  SERIAL_RUN(watchdog.reset());

  #if HAS_TRINAMIC && !PS_DEFAULT_OFF
    SERIAL_RUN(tmcManager.test_connection(true, true, true, true));
  #endif

  #if HAS_MMU2
    SERIAL_RUN(mmu2.init());
  #endif

  SERIAL_LOG("setup() completed.");
}

/**
 * The main MK4duo program loop
 *
 *  - Save or log commands to SD
 *  - Process available commands (if not saving)
 *  - Call endstop manager
 *  - Call LCD update
 */
void loop() {

  do {

    printer.idle();

    #if HAS_SD_SUPPORT
      card.checkautostart();
      if (card.isAbortSDprinting()) printer.abort_sd_printing();
      if (card.isComplete()) printer.finish_sd_printing();
    #endif // HAS_SD_SUPPORT

    commands.advance_queue();
    endstops.report_state();

  } while (MK_MAIN_LOOP);
}
