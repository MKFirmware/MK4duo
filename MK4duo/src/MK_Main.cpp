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
 * About Marlin
 *
 * This firmware is a mashup between Sprinter and grbl.
 *  - https://github.com/kliment/Sprinter
 *  - https://github.com/simen/grbl/tree
 *
 * It has preliminary support for Matthew Roberts advance algorithm
 *  - http://reprap.org/pipermail/reprap-dev/2011-May/003323.html
 */

#include "../base.h"

#if ENABLED(PCA9632)
  #include "utility/pca9632.h"
#endif

#if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
  #include "HAL/HAL_endstop_interrupts.h"
#endif

#if ENABLED(RFID_MODULE)
  MFRC522 RFID522;
#endif

#if ENABLED(M100_FREE_MEMORY_WATCHER)
  void gcode_M100();
  #if ENABLED(M100_FREE_MEMORY_DUMPER)
    void M100_dump_routine(const char * const title, const char *start, const char *end);
  #endif
#endif

#if HAS_SDSUPPORT
  CardReader card;
#endif

#if ENABLED(G38_PROBE_TARGET)
  bool G38_move         = false,
       G38_endstop_hit  = false;
#endif

#if ENABLED(FLOWMETER_SENSOR) && ENABLED(MINFLOW_PROTECTION)
  bool flow_firstread = false;
#endif

#if HAS_POWER_SWITCH
  Power powerManager;
#endif

bool Running = true;

// Print status related
long  currentLayer  = 0,
      maxLayer      = -1;   // -1 = unknown
char  printName[21] = "";   // max. 20 chars + 0
float progress      = 0.0;

uint8_t mk_debug_flags = DEBUG_NONE;

// Interrupt Event
MK4duoInterruptEvent interruptEvent = INTERRUPT_EVENT_NONE;

// Printer mode
PrinterMode printer_mode =
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

/**
 * GCode line number handling. Hosts may opt to include line numbers when
 * sending commands to MK4duo, and lines will be checked for sequentiality.
 * M110 N<int> sets the current line number.
 */
static long gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;

/**
 * GCode Command Queue
 * A simple ring buffer of BUFSIZE command strings.
 *
 * Commands are copied into this buffer by the command injectors
 * (immediate, serial, sd card) and they are processed sequentially by
 * the main loop. The process_next_command function parses the next
 * command and hands off execution to individual handler functions.
 */
uint8_t commands_in_queue = 0;          // Count of commands in the queue
static uint8_t  cmd_queue_index_r = 0,  // Ring buffer read position
                cmd_queue_index_w = 0;  // Ring buffer write position

#if ENABLED(M100_FREE_MEMORY_WATCHER)
  char command_queue[BUFSIZE][MAX_CMD_SIZE];
#else
  static char command_queue[BUFSIZE][MAX_CMD_SIZE];
#endif

/**
 * Next Injected Command pointer. NULL if no commands are being injected.
 * Used by MK4duo internally to ensure that commands initiated from within
 * are enqueued ahead of any pending serial or sd card commands.
 */
static const char *injected_commands_P = NULL;

bool pos_saved = false;

#if ENABLED(TEMPERATURE_UNITS_SUPPORT)
  TempUnit input_temp_units = TEMPUNIT_C;
#endif

int16_t flow_percentage[EXTRUDERS] = ARRAY_BY_EXTRUDERS(100),
        density_percentage[EXTRUDERS] = ARRAY_BY_EXTRUDERS(100);

bool axis_relative_modes[] = AXIS_RELATIVE_MODES,
     #if ENABLED(VOLUMETRIC_DEFAULT_ON)
       volumetric_enabled = true;
     #else
       volumetric_enabled = false;
     #endif

float filament_size[EXTRUDERS] = ARRAY_BY_EXTRUDERS(DEFAULT_NOMINAL_FILAMENT_DIA),
      volumetric_multiplier[EXTRUDERS] = ARRAY_BY_EXTRUDERS(1.0);

#if FAN_COUNT > 0
  int16_t fanSpeeds[FAN_COUNT] = { 0 };
#endif
#if HAS_CONTROLLERFAN
  uint8_t controller_fanSpeeds = 0;
#endif
#if HAS_AUTO_FAN
  uint8_t autoFanSpeeds[HOTENDS] = { 0 };
#endif
#if ENABLED(FAN_KICKSTART_TIME)
  uint8_t fanKickstart = 0;
#endif

float hotend_offset[XYZ][HOTENDS];

// The active extruder (tool). Set with T<extruder> command.
uint8_t active_extruder   = 0,
        previous_extruder = 0,
        active_driver     = 0;

static uint8_t target_extruder;

#if ENABLED(CNCROUTER)
  uint8_t active_cnc_tool = 0;
  #define CNC_M6_TOOL_ID 255
#endif

// Relative Mode. Enable with G91, disable with G90.
static bool relative_mode = false;

// For M109 and M190, this flag may be cleared (by M108) to exit the wait loop
volatile bool wait_for_heatup = true;

// For M0/M1, this flag may be cleared (by M108) to exit the wait-for-user loop
#if ENABLED(EMERGENCY_PARSER) || HAS_LCD
  volatile bool wait_for_user = false;
#endif

const char axis_codes[XYZE] = {'X', 'Y', 'Z', 'E'};

// Number of characters read in the current line of serial input
static int serial_count = 0;

// Inactivity shutdown
millis_t previous_cmd_ms              = 0;
static millis_t max_inactive_time     = 0,
                stepper_inactive_time = (DEFAULT_STEPPER_DEACTIVE_TIME) * 1000UL;

// Print Job Timer
PrintCounter print_job_counter = PrintCounter();

#if HEATER_USES_AD595
  float ad595_offset[HOTENDS] = ARRAY_BY_HOTENDS(TEMP_SENSOR_AD595_OFFSET),
        ad595_gain[HOTENDS]   = ARRAY_BY_HOTENDS(TEMP_SENSOR_AD595_GAIN);
#endif

#if ENABLED(NPR2)
  uint8_t old_color = 99;
#endif

#if ENABLED(RFID_MODULE)
  unsigned long Spool_ID[EXTRUDERS] = ARRAY_BY_EXTRUDERS(0);
  bool  RFID_ON = false,
        Spool_must_read[EXTRUDERS]  = ARRAY_BY_EXTRUDERS(false),
        Spool_must_write[EXTRUDERS] = ARRAY_BY_EXTRUDERS(false);
#endif

#if ENABLED(BARICUDA)
  int baricuda_valve_pressure   = 0,
      baricuda_e_to_p_pressure  = 0;
#endif

#if ENABLED(FWRETRACT)

  bool  autoretract_enabled           = false,
        retracted[EXTRUDERS]          = { false },
        retracted_swap[EXTRUDERS]     = { false };

  float retract_length                = RETRACT_LENGTH,
        retract_length_swap           = RETRACT_LENGTH_SWAP,
        retract_feedrate_mm_s         = RETRACT_FEEDRATE,
        retract_zlift                 = RETRACT_ZLIFT,
        retract_recover_length        = RETRACT_RECOVER_LENGTH,
        retract_recover_length_swap   = RETRACT_RECOVER_LENGTH_SWAP,
        retract_recover_feedrate_mm_s = RETRACT_RECOVER_FEEDRATE;

#endif // FWRETRACT

#if MECH(DELTA)

  #if ENABLED(DELTA_AUTO_CALIBRATION_3)

    float ac_prec,
          bed_level_c,
          bed_level_x,
          bed_level_y,
          bed_level_z,
          bed_level_ox,
          bed_level_oy,
          bed_level_oz,
          adj_t1_Radius = 0,
          adj_t2_Radius = 0,
          adj_t3_Radius = 0,
          adj_diagrod_length();

    int fix_tower_errors();
    bool adj_deltaradius();

    void  adj_tower_delta(uint8_t tower);
    void  adj_tower_radius(uint8_t tower);
    void  calibration_report();
    void  bed_probe_all();
    void  adj_endstops();

  #endif // DELTA_AUTO_CALIBRATION_3

#endif

#if IS_SCARA
  // Float constants for SCARA calculations
  const float L1 = SCARA_LINKAGE_1, L2 = SCARA_LINKAGE_2,
              L1_2 = sq(float(L1)), L1_2_2 = 2.0 * L1_2,
              L2_2 = sq(float(L2));

  float scara_segments_per_second = SCARA_SEGMENTS_PER_SECOND,
        delta[ABC];
  static void plan_direct_stepper_move(const float target[XYZE]);
#endif

#if MECH(MAKERARM_SCARA)

  #define LEFT_ARM false
  #define RIGHT_ARM true

  bool arm_orientation = LEFT_ARM;

  bool set_head_index = true;
  float head_offsets[4] = { 0.0 };
  int quadrant_limit = 1;

  float dest_fin[3] = { 0.0 },
        previous_extruder_pos = 0,
        final_extruder_pos,
        layer_height = 0.0;

  bool z_layer_height_check = true;

  float z_offset_for_eq = 0,
        z_for_bed_eq[3] = { 0 },
        z_points_qd1[3] = { 0 },
        z_points_qd2[3] = { 0 },
        read_z = 0.0;

  bool G92_called = false,
       offset_toggle_x = true,
       offset_toggle_y = true;

#endif

#if MECH(MUVE3D)
  static float  peel_distance = 0,
                peel_speed = 0,
                peel_pause = 0,
                retract_speed = 0,
                tilt_distance = 0,
                layer_thickness = 0;
  static bool   tilted = false;
#endif

#if ENABLED(FILAMENT_SENSOR)
  bool    filament_sensor = false;                                // M405 turns on filament_sensor control, M406 turns it off 
  float   filament_width_nominal = DEFAULT_NOMINAL_FILAMENT_DIA,  // Nominal filament width. Change with M404
          filament_width_meas = DEFAULT_MEASURED_FILAMENT_DIA;    // Measured filament diameter
  uint8_t meas_delay_cm = MEASUREMENT_DELAY_CM,                   // Distance delay setting
          measurement_delay[MAX_MEASUREMENT_DELAY + 1];           // Ring buffer to delayed measurement. Store extruder factor after subtracting 100
  int8_t  filwidth_delay_index[2] = { 0, -1 };                    // Indexes into ring buffer
#endif

#if HAS_FIL_RUNOUT || HAS_EXT_ENCODER
  static bool filament_ran_out = false;
#endif

#if HAS_EXT_ENCODER
  int32_t encStepsSinceLastSignal[EXTRUDERS]  = ARRAY_BY_EXTRUDERS(0);
  uint8_t encLastSignal[EXTRUDERS]            = ARRAY_BY_EXTRUDERS(0);
  int8_t  encLastDir[EXTRUDERS]               = ARRAY_BY_EXTRUDERS(1);
  int32_t encLastChangeAt[EXTRUDERS]          = ARRAY_BY_EXTRUDERS(0),
          encErrorSteps[EXTRUDERS]            = ARRAY_BY_EXTRUDERS(ENC_ERROR_STEPS);
#endif

#if ENABLED(ADVANCED_PAUSE_FEATURE)
  AdvancedPauseMenuResponse advanced_pause_menu_response;
#endif

#if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
  float motor_current[XYZ + DRIVER_EXTRUDERS];
#endif

#if ENABLED(COLOR_MIXING_EXTRUDER)
  float mixing_factor[MIXING_STEPPERS]; // Reciprocal of mix proportion. 0.0 = off, otherwise >= 1.0
  #if MIXING_VIRTUAL_TOOLS  > 1
    float mixing_virtual_tool_mix[MIXING_VIRTUAL_TOOLS][MIXING_STEPPERS];
  #endif
#endif

#if ENABLED(IDLE_OOZING_PREVENT)
  unsigned long axis_last_activity = 0;
  bool IDLE_OOZING_enabled = true;
  bool IDLE_OOZING_retracted[EXTRUDERS] = ARRAY_BY_EXTRUDERS(false);
#endif

#if HAS_POWER_CONSUMPTION_SENSOR
  float power_consumption_meas = 0.0;
  unsigned long power_consumption_hour,
                startpower  = 0;
                stoppower   = 0;
#endif

#if ENABLED(NPR2)
  static float  color_position[] = COLOR_STEP,
                color_step_moltiplicator = (DRIVER_MICROSTEP / MOTOR_ANGLE) * CARTER_MOLTIPLICATOR;
#endif // NPR2

#if ENABLED(EASY_LOAD)
  bool allow_lengthy_extrude_once; // for load/unload
#endif

static bool send_ok[BUFSIZE];

#if HAS_CHDK
  millis_t chdkHigh = 0;
  bool chdkActive = false;
#endif

#if ENABLED(PIDTEMP) && ENABLED(PID_ADD_EXTRUSION_RATE)
  int lpq_len = 20;
#endif

#if ENABLED(HOST_KEEPALIVE_FEATURE)
  MK4duoBusyState busy_state = NOT_BUSY;
  static millis_t next_busy_signal_ms = 0;
  uint8_t host_keepalive_interval = DEFAULT_KEEPALIVE_INTERVAL;
#else
  #define host_keepalive() NOOP
#endif

#if ENABLED(CNC_WORKSPACE_PLANES)
  static WorkspacePlane workspace_plane = PLANE_XY;
#endif

/**
 * ***************************************************************************
 * ******************************** FUNCTIONS ********************************
 * ***************************************************************************
 */

void stop();

void get_available_commands();
void process_next_command();

void safe_delay(millis_t ms) {
  while (ms > 50) {
    ms -= 50;
    HAL::delayMilliseconds(50);
    thermalManager.manage_temp_controller();
  }
  HAL::delayMilliseconds(ms);
}

#if ENABLED(ARC_SUPPORT)
  void plan_arc(float target[NUM_AXIS], float* offset, uint8_t clockwise);
#endif

void report_current_position_detail();

/**
 * Sensitive pin test for M42, M226
 */
static bool pin_is_protected(uint8_t pin) {
  static const int8_t sensitive_pins[] PROGMEM = SENSITIVE_PINS;
  for (uint8_t i = 0; i < COUNT(sensitive_pins); i++)
    if (pin == (int8_t)pgm_read_byte(&sensitive_pins[i])) return true;
  return false;
}

/**
 * Inject the next "immediate" command, when possible, onto the front of the queue.
 * Return true if any immediate commands remain to inject.
 */
static bool drain_injected_commands_P() {
  if (injected_commands_P != NULL) {
    size_t i = 0;
    char c, cmd[30];
    strncpy_P(cmd, injected_commands_P, sizeof(cmd) - 1);
    cmd[sizeof(cmd) - 1] = '\0';
    while ((c = cmd[i]) && c != '\n') i++; // find the end of this gcode command
    cmd[i] = '\0';
    if (enqueue_and_echo_command(cmd))     // success?
      injected_commands_P = c ? injected_commands_P + i + 1 : NULL; // next command or done
  }
  return (injected_commands_P != NULL);    // return whether any more remain
}

/**
 * Record one or many commands to run from program memory.
 * Aborts the current queue, if any.
 * Note: drain_injected_commands_P() must be called repeatedly to drain the commands afterwards
 */
void enqueue_and_echo_commands_P(const char * const pgcode) {
  injected_commands_P = pgcode;
  drain_injected_commands_P(); // first command executed asap (when possible)
}

/**
 * Clear the MK4duo command queue
 */
void clear_command_queue() {
  cmd_queue_index_r = cmd_queue_index_w;
  commands_in_queue = 0;
}

/**
 * Once a new command is in the ring buffer, call this to commit it
 */
inline void _commit_command(bool say_ok) {
  send_ok[cmd_queue_index_w] = say_ok;
  if (++cmd_queue_index_w >= BUFSIZE) cmd_queue_index_w = 0;
  commands_in_queue++;
}

/**
 * Copy a command from RAM into the main command buffer.
 * Return true if the command was successfully added.
 * Return false for a full buffer, or if the 'command' is a comment.
 */
inline bool _enqueuecommand(const char* cmd, bool say_ok = false) {
  if (*cmd == ';' || commands_in_queue >= BUFSIZE) return false;
  strcpy(command_queue[cmd_queue_index_w], cmd);
  _commit_command(say_ok);
  return true;
}

/**
 * Enqueue with Serial Echo
 */
bool enqueue_and_echo_command(const char* cmd, bool say_ok/*=false*/) {
  if (_enqueuecommand(cmd, say_ok)) {
    SERIAL_SMT(ECHO, MSG_ENQUEUEING, cmd);
    SERIAL_CHR('"');
    SERIAL_EOL();
    return true;
  }
  return false;
}

#if HAS_FIL_RUNOUT
  void setup_filrunoutpin() {
    #if ENABLED(ENDSTOPPULLUP_FIL_RUNOUT)
      SET_INPUT_PULLUP(FIL_RUNOUT_PIN);
    #else
      SET_INPUT(FIL_RUNOUT_PIN);
    #endif
  }
#endif

void setup_powerhold() {
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

void suicide() {
  #if HAS_SUICIDE
    OUT_WRITE(SUICIDE_PIN, LOW);
  #endif
}

void servo_init() {
  #if NUM_SERVOS >= 1 && HAS_SERVO_0
    servo[0].attach(SERVO0_PIN);
    servo[0].detach(); // Just set up the pin. We don't have a position yet. Don't move to a random position.
  #endif
  #if NUM_SERVOS >= 2 && HAS_SERVO_1
    servo[1].attach(SERVO1_PIN);
    servo[1].detach();
  #endif
  #if NUM_SERVOS >= 3 && HAS_SERVO_2
    servo[2].attach(SERVO2_PIN);
    servo[2].detach();
  #endif
  #if NUM_SERVOS >= 4 && HAS_SERVO_3
    servo[3].attach(SERVO3_PIN);
    servo[3].detach();
  #endif

  #if HAS_DONDOLO
    servo[DONDOLO_SERVO_INDEX].attach(0);
    servo[DONDOLO_SERVO_INDEX].write(DONDOLO_SERVOPOS_E0);
    #if (DONDOLO_SERVO_DELAY > 0)
      safe_delay(DONDOLO_SERVO_DELAY);
      servo[DONDOLO_SERVO_INDEX].detach();
    #endif
  #endif

  #if HAS_Z_SERVO_PROBE
    /**
     * Set position of Z Servo Endstop
     *
     * The servo might be deployed and positioned too low to stow
     * when starting up the machine or rebooting the board.
     * There's no way to know where the nozzle is positioned until
     * homing has been done - no homing with z-probe without init!
     *
     */
    STOW_Z_SERVO();
  #endif
}

/**
 * Stepper Reset (RigidBoard, et.al.)
 */
#if HAS_STEPPER_RESET
  void disableStepperDrivers() {
    OUT_WRITE(STEPPER_RESET_PIN, LOW);  // drive it down to hold in reset motor driver chips
  }
  void enableStepperDrivers() { SET_INPUT(STEPPER_RESET_PIN); }  // set to input, which allows it to be pulled high by pullups
#endif

#if HAS_COLOR_LEDS

  void set_led_color(
    const uint8_t r, const uint8_t g, const uint8_t b
      #if ENABLED(RGBW_LED)
        , const uint8_t w=0
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
  
void gcode_line_error(const char* err, bool doFlush = true) {
  SERIAL_STR(ER);
  SERIAL_PS(err);
  SERIAL_EV(gcode_LastN);
  //Serial.println(gcode_N);
  if (doFlush) FlushSerialRequestResend();
  serial_count = 0;
}

/**
 * Get all commands waiting on the serial port and queue them.
 * Exit when the buffer is full or when no more characters are
 * left on the serial port.
 */
inline void get_serial_commands() {
  static char serial_line_buffer[MAX_CMD_SIZE];
  static bool serial_comment_mode = false;

  #if HAS_DOOR
    if (READ(DOOR_PIN) != DOOR_PIN_INVERTING) {
      KEEPALIVE_STATE(DOOR_OPEN);
      return;  // do nothing while door is open
    }
  #endif

  // If the command buffer is empty for too long,
  // send "wait" to indicate MK4duo is still waiting.
  #if ENABLED(NO_TIMEOUTS) && NO_TIMEOUTS > 0
    static millis_t last_command_time = 0;
    millis_t ms = millis();
    if (commands_in_queue == 0 && !HAL::serialByteAvailable() && ELAPSED(ms, last_command_time + NO_TIMEOUTS)) {
      SERIAL_L(WT);
      last_command_time = ms;
    }
  #endif

  /**
   * Loop while serial characters are incoming and the queue is not full
   */
  while (commands_in_queue < BUFSIZE && HAL::serialByteAvailable() > 0) {

    char serial_char = HAL::serialReadByte();

    /**
     * If the character ends the line
     */
    if (serial_char == '\n' || serial_char == '\r') {

      serial_comment_mode = false; // end of line == end of comment

      if (!serial_count) continue; // skip empty lines

      serial_line_buffer[serial_count] = 0; // terminate string
      serial_count = 0; // reset buffer

      char* command = serial_line_buffer;

      while (*command == ' ') command++; // skip any leading spaces
      char* npos = (*command == 'N') ? command : NULL; // Require the N parameter to start the line
      char* apos = strchr(command, '*');

      if (npos) {

        const bool M110 = strstr_P(command, PSTR("M110")) != NULL;

        if (M110) {
          char* n2pos = strchr(command + 4, 'N');
          if (n2pos) npos = n2pos;
        }

        gcode_N = strtol(npos + 1, NULL, 10);

        if (gcode_N != gcode_LastN + 1 && !M110) {
          gcode_line_error(PSTR(MSG_ERR_LINE_NO));
          return;
        }

        if (apos) {
          byte checksum = 0, count = 0;
          while (command[count] != '*') checksum ^= command[count++];

          if (strtol(apos + 1, NULL, 10) != checksum) {
            gcode_line_error(PSTR(MSG_ERR_CHECKSUM_MISMATCH));
            return;
          }
          // if no errors, continue parsing
        }
        else {
          gcode_line_error(PSTR(MSG_ERR_NO_CHECKSUM));
          return;
        }

        gcode_LastN = gcode_N;
        // if no errors, continue parsing
      }
      else if (apos) { // No '*' without 'N'
        gcode_line_error(PSTR(MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM), false);
        return;
      }

      // Movement commands alert when stopped
      if (IsStopped()) {
        char* gpos = strchr(command, 'G');
        if (gpos) {
          const int codenum = strtol(gpos + 1, NULL, 10);
          switch (codenum) {
            case 0:
            case 1:
            case 2:
            case 3:
              SERIAL_LM(ER, MSG_ERR_STOPPED);
              LCD_MESSAGEPGM(MSG_STOPPED);
              break;
          }
        }
      }

      #if DISABLED(EMERGENCY_PARSER)
        // If command was e-stop process now
        if (strcmp(command, "M108") == 0) {
          wait_for_heatup = false;
          #if ENABLED(ULTIPANEL)
            wait_for_user = false;
          #endif
        }
        if (strcmp(command, "M112") == 0) kill(PSTR(MSG_KILLED));
        if (strcmp(command, "M410") == 0) { quickstop_stepper(); }
      #endif

      #if ENABLED(NO_TIMEOUTS) && NO_TIMEOUTS > 0
        last_command_time = ms;
      #endif

      // Add the command to the queue
      _enqueuecommand(serial_line_buffer, true);
    }
    else if (serial_count >= MAX_CMD_SIZE - 1) {
      // Keep fetching, but ignore normal characters beyond the max length
      // The command will be injected when EOL is reached
    }
    else if (serial_char == '\\') { // Handle escapes
      if (HAL::serialByteAvailable()) {
        // if we have one more character, copy it over
        serial_char = HAL::serialReadByte();
        if (!serial_comment_mode) serial_line_buffer[serial_count++] = serial_char;
      }
      // otherwise do nothing
    }
    else { // its not a newline, carriage return or escape char
      if (serial_char == ';') serial_comment_mode = true;
      if (!serial_comment_mode) serial_line_buffer[serial_count++] = serial_char;
    }
  } // queue has space, serial has data
}

#if HAS_SDSUPPORT

  /**
   * SD Stop & Store location
   */
  void stopSDPrint(const bool store_location) {
    if (card.cardOK && card.isFileOpen() && IS_SD_PRINTING) {
      if (store_location) SERIAL_EM("Close file and save restart.gcode");
      card.stopSDPrint(store_location);
      clear_command_queue();
      quickstop_stepper();
      print_job_counter.stop();
      wait_for_heatup = false;
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

  /**
   * Get commands from the SD Card until the command buffer is full
   * or until the end of the file is reached. The special character '#'
   * can also interrupt buffering.
   */
  inline void get_sdcard_commands() {
    static bool stop_buffering = false,
                sd_comment_mode = false;

    if (!card.sdprinting) return;

    #if HAS_DOOR
      if (READ(DOOR_PIN) != DOOR_PIN_INVERTING) {
        KEEPALIVE_STATE(DOOR_OPEN);
        return;  // do nothing while door is open
      }
    #endif

    #if HAS_POWER_CHECK
      if (READ(POWER_CHECK_PIN) != POWER_CHECK_PIN_INVERTING) {
        stopSDPrint(true);
        return;
      }
    #endif

    /**
     * '#' stops reading from SD to the buffer prematurely, so procedural
     * macro calls are possible. If it occurs, stop_buffering is triggered
     * and the buffer is run dry; this character _can_ occur in serial com
     * due to checksums, however, no checksums are used in SD printing.
     */

    if (commands_in_queue == 0) stop_buffering = false;

    uint16_t sd_count = 0;
    bool card_eof = card.eof();
    while (commands_in_queue < BUFSIZE && !card_eof && !stop_buffering) {
      const int16_t n = card.get();
      char sd_char = (char)n;
      card_eof = card.eof();
      if (card_eof || n == -1
          || sd_char == '\n' || sd_char == '\r'
          || ((sd_char == '#' || sd_char == ':') && !sd_comment_mode)
      ) {
        if (card_eof) {
          SERIAL_EM(MSG_FILE_PRINTED);
          card.printingHasFinished();
          #if ENABLED(PRINTER_EVENT_LEDS)
            LCD_MESSAGEPGM(MSG_INFO_COMPLETED_PRINTS);
            set_led_color(0, 255, 0); // Green
            #if HAS(RESUME_CONTINUE)
              enqueue_and_echo_commands_P(PSTR("M0")); // end of the queue!
            #else
              safe_delay(1000);
            #endif
            set_led_color(0, 0, 0);   // OFF
          #endif
          card.checkautostart(true);
        }
        else if (n == -1) {
          SERIAL_LM(ER, MSG_SD_ERR_READ);
        }
        if (sd_char == '#') stop_buffering = true;

        sd_comment_mode = false; // for new command

        if (!sd_count) continue; // skip empty lines (and comment lines)

        command_queue[cmd_queue_index_w][sd_count] = '\0'; // terminate string
        sd_count = 0; // clear sd line buffer

        _commit_command(false);
      }
      else if (sd_count >= MAX_CMD_SIZE - 1) {
        /**
         * Keep fetching, but ignore normal characters beyond the max length
         * The command will be injected when EOL is reached
         */
      }
      else {
        if (sd_char == ';') sd_comment_mode = true;
        if (!sd_comment_mode) command_queue[cmd_queue_index_w][sd_count++] = sd_char;
      }
    }
  }

#endif // SDSUPPORT

/**
 * Add to the circular command queue the next command from:
 *  - The command-injection queue (injected_commands_P)
 *  - The active serial input (usually USB)
 *  - The SD card file being actively printed
 */
void get_available_commands() {

  // if any immediate commands remain, don't get other commands yet
  if (drain_injected_commands_P()) return;

  get_serial_commands();

  #if HAS_SDSUPPORT
    get_sdcard_commands();
  #endif
}

/**
 * Set target_extruder from the T parameter or the active_extruder
 *
 * Returns TRUE if the target is invalid
 */
bool get_target_extruder_from_command(const uint16_t code) {
  if (parser.seenval('T')) {
    const int8_t e = parser.value_byte();
    if (e >= EXTRUDERS) {
      SERIAL_SMV(ECHO, "M", code);
      SERIAL_EMV(" " MSG_INVALID_EXTRUDER, e);
      return true;
    }
    target_extruder = e;
  }
  else
    target_extruder = active_extruder;

  return false;
}

/**
 * Set target_Hotend from the H parameter or the active_extruder
 *
 * Returns TRUE if the target is invalid
 */
bool get_target_hotend_from_command(const uint16_t code) {
  if (parser.seenval('H')) {
    const int8_t h = parser.value_byte();
    if (h >= HOTENDS) {
      SERIAL_SMV(ER, "M", code);
      SERIAL_EMV(" " MSG_INVALID_HOTEND, h);
      return true;
    }
    target_extruder = h;
  }
  else
    target_extruder = active_extruder;

  return false;
}

#if ENABLED(WORKSPACE_OFFSETS)

  /**
   * Change the home offset for an axis, update the current
   * position and the software endstops to retain the same
   * relative distance to the new home.
   *
   * Since this changes the current_position, code should
   * call sync_plan_position soon after this.
   */
  static void set_home_offset(AxisEnum axis, float v) {
    mechanics.current_position[axis] += v - mechanics.home_offset[axis];
    mechanics.home_offset[axis] = v;
    endstops.update_software_endstops(axis);
  }

#endif // ENABLED(WORKSPACE_OFFSETS)

/**
 * Prepare to do endstop or probe moves
 * with custom feedrates.
 *
 *  - Save current feedrates
 *  - Reset the rate multiplier
 *  - Reset the command timeout
 *  - Enable the endstops (for endstop moves)
 */
void setup_for_endstop_or_probe_move() {
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) DEBUG_POS("setup_for_endstop_or_probe_move", mechanics.current_position);
  #endif
  mechanics.saved_feedrate_mm_s = mechanics.feedrate_mm_s;
  mechanics.saved_feedrate_percentage = mechanics.feedrate_percentage;
  mechanics.feedrate_percentage = 100;
  refresh_cmd_timeout();
}

void clean_up_after_endstop_or_probe_move() {
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) DEBUG_POS("clean_up_after_endstop_or_probe_move", mechanics.current_position);
  #endif
  mechanics.feedrate_mm_s = mechanics.saved_feedrate_mm_s;
  mechanics.feedrate_percentage = mechanics.saved_feedrate_percentage;
  refresh_cmd_timeout();
}

#if HAS_Z_PROBE_SLED

  #if DISABLED(SLED_DOCKING_OFFSET)
    #define SLED_DOCKING_OFFSET 0
  #endif

  /**
   * Method to dock/undock a sled designed by Charles Bell.
   *
   * stow[in]     If false, move to MAX_ and engage the solenoid
   *              If true, move to MAX_X and release the solenoid
   */
  static void dock_sled(bool stow) {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_MV("dock_sled(", stow);
        SERIAL_CHR(')'); SERIAL_EOL();
      }
    #endif

    // Dock sled a bit closer to ensure proper capturing
    mechanics.do_blocking_move_to_x(X_MAX_POS + SLED_DOCKING_OFFSET - ((stow) ? 1 : 0));
    WRITE(SLED_PIN, !stow); // switch solenoid
  }

#endif // Z_PROBE_SLED
  
#if ENABLED(Z_PROBE_ALLEN_KEY)

  void run_deploy_moves_script() {

    const float z_probe_deploy_start_location[] = Z_PROBE_DEPLOY_START_LOCATION,
                z_probe_deploy_end_location[] = Z_PROBE_DEPLOY_END_LOCATION;

    // Move to the start position to initiate deployment
    mechanics.do_blocking_move_to(z_probe_deploy_start_location, mechanics.homing_feedrate_mm_s[Z_AXIS]);

    // Move to engage deployment
    mechanics.do_blocking_move_to(z_probe_deploy_end_location, mechanics.homing_feedrate_mm_s[Z_AXIS] / 10);

    // Move to trigger deployment
    mechanics.do_blocking_move_to(z_probe_deploy_start_location, mechanics.homing_feedrate_mm_s[Z_AXIS]);
  }
  void run_stow_moves_script() {

    const float z_probe_retract_start_location[] = Z_PROBE_RETRACT_START_LOCATION,
                z_probe_retract_end_location[] = Z_PROBE_RETRACT_END_LOCATION;

    // Move to the start position to initiate retraction
    mechanics.do_blocking_move_to(z_probe_retract_start_location, mechanics.homing_feedrate_mm_s[Z_AXIS]);

    // Move the nozzle down to push the Z probe into retracted position
    mechanics.do_blocking_move_to(z_probe_retract_end_location, mechanics.homing_feedrate_mm_s[Z_AXIS] / 10);

    // Move up for safety
    mechanics.do_blocking_move_to(z_probe_retract_start_location, mechanics.homing_feedrate_mm_s[Z_AXIS]);
  }

#endif

#if HAS_BED_PROBE

  #if MECH(MAKERARM_SCARA)

    /**
     * Get the arm-end position based on the probe position
     * If the position is unreachable return vector_3 0,0,0
     */
    vector_3 probe_point_to_end_point(const float &x, const float &y) {

      // Simply can't reach the given point
      if (HYPOT2(x, y) > sq(L1 + L2 + Y_PROBE_OFFSET_FROM_NOZZLE))
        return vector_3();

      float pos[XYZ] = { x, y, 0 };

      // Get the angles for placing the probe at x, y
      inverse_kinematics(pos, Y_PROBE_OFFSET_FROM_NOZZLE);

      // Get the arm-end XY based on the given angles
      forward_kinematics_SCARA(mechanics.delta[A_AXIS], mechanics.delta[B_AXIS]);
      float tx = LOGICAL_X_POSITION(mechanics.cartesian_position[X_AXIS]),
            ty = LOGICAL_Y_POSITION(mechanics.cartesian_position[Y_AXIS]);

      return vector_3(tx, ty, 0);
    }

    /**
     * Get the probe position based on the arm-end position
     * If the position is unreachable return vector_3 0,0,0
     */
    vector_3 end_point_to_probe_point(const float logical[XYZ]) {

      // Simply can't reach the given point
      if (HYPOT2(logical[X_AXIS], logical[Y_AXIS]) > sq(L1 + L2))
        return vector_3();

      // Get the angles for placing the arm-end at x, y
      inverse_kinematics(logical);

      // Get the probe XY based on the sum of the angles
      float ab = RADIANS(mechanics.delta[A_AXIS] + mechanics.delta[B_AXIS] + 90.0);
      return vector_3(
        logical[X_AXIS] + sin(ab) * X_PROBE_OFFSET_FROM_NOZZLE,
        logical[Y_AXIS] - cos(ab) * Y_PROBE_OFFSET_FROM_NOZZLE,
        0
      );
    }

  #endif // MAKERARM_SCARA

#endif // HAS_BED_PROBE

#if ENABLED(FWRETRACT)

  void retract(const bool retracting, const bool swapping = false) {

    static float hop_height;

    if (retracting == retracted[active_extruder]) return;

    const float old_feedrate_mm_s = mechanics.feedrate_mm_s;

    mechanics.set_destination_to_current();

    if (retracting) {
      mechanics.feedrate_mm_s = retract_feedrate_mm_s;
      mechanics.current_position[E_AXIS] += (swapping ? retract_length_swap : retract_length) / volumetric_multiplier[active_extruder];
      mechanics.sync_plan_position_e();
      mechanics.prepare_move_to_destination();

      if (retract_zlift > 0.01) {
        hop_height = mechanics.current_position[Z_AXIS];
        // Pretend current position is lower
        mechanics.current_position[Z_AXIS] -= retract_zlift;
        mechanics.sync_plan_position();
        // Raise up to the old mechanics.current_position
        mechanics.prepare_move_to_destination();
      }
    }
    else {

      // If the height hasn't been lowered, undo the Z hop
      if (retract_zlift > 0.01 && hop_height == mechanics.current_position[Z_AXIS]) {
        // Pretend current position is higher. Z will lower on the next move
        mechanics.current_position[Z_AXIS] += retract_zlift;
        mechanics.sync_plan_position();
        // Lower Z
        mechanics.prepare_move_to_destination();
      }

      mechanics.feedrate_mm_s = retract_recover_feedrate_mm_s;
      const float move_e = swapping ? retract_length_swap + retract_recover_length_swap : retract_length + retract_recover_length;
      mechanics.current_position[E_AXIS] -= move_e / volumetric_multiplier[active_extruder];
      mechanics.sync_plan_position_e();

      // Lower Z and recover E
      mechanics.prepare_move_to_destination();
    }

    mechanics.feedrate_mm_s = old_feedrate_mm_s;
    retracted[active_extruder] = retracting;

  } // retract

#endif // FWRETRACT

#if ENABLED(COLOR_MIXING_EXTRUDER)

  void normalize_mix() {
    float mix_total = 0.0;
    for (uint8_t i = 0; i < MIXING_STEPPERS; i++) mix_total += RECIPROCAL(mixing_factor[i]);
    // Scale all values if they don't add up to ~1.0
    if (!NEAR(mix_total, 1.0)) {
      SERIAL_EM("Warning: Mix factors must add up to 1.0. Scaling.");
      for (uint8_t i = 0; i < MIXING_STEPPERS; i++) mixing_factor[i] *= mix_total;
    }
  }

  // Get mixing parameters from the GCode
  // The total "must" be 1.0 (but it will be normalized)
  // If no mix factors are given, the old mix is preserved
  void gcode_get_mix() {
    const char* mixing_codes = "ABCDHI";
    byte mix_bits = 0;
    for (uint8_t i = 0; i < MIXING_STEPPERS; i++) {
      if (parser.seenval(mixing_codes[i])) {
        SBI(mix_bits, i);
        float v = parser.value_float();
        NOLESS(v, 0.0);
        mixing_factor[i] = RECIPROCAL(v);
      }
    }
    // If any mixing factors were included, clear the rest
    // If none were included, preserve the last mix
    if (mix_bits) {
      for (uint8_t i = 0; i < MIXING_STEPPERS; i++)
        if (!TEST(mix_bits, i)) mixing_factor[i] = 0.0;
      normalize_mix();
    }
  }

#endif

#if ENABLED(IDLE_OOZING_PREVENT)

  void IDLE_OOZING_retract(bool retracting) {  
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

#if HAS_TEMP_HOTEND || HAS_TEMP_BED

  void print_heater_state(const float &c, const int16_t &t,
    #if ENABLED(SHOW_TEMP_ADC_VALUES)
      const int16_t r,
    #endif
    const int8_t e=-2
  ) {
    SERIAL_CHR(' ');
    SERIAL_CHR(
      #if HAS_TEMP_BED && HAS_TEMP_HOTEND
        e == -1 ? 'B' : 'T'
      #elif HAS_TEMP_HOTEND
        'T'
      #else
        'B'
      #endif
    );
    #if HOTENDS > 1
      if (e >= 0) SERIAL_CHR('0' + e);
    #endif
    SERIAL_CHR(':');
    SERIAL_VAL(c, 1);
    SERIAL_MV(" /" , t);
    #if ENABLED(SHOW_TEMP_ADC_VALUES)
      SERIAL_MV(" (", r);
      SERIAL_CHR(')');
    #endif
  }

  void print_heaterstates() {
    #if HAS_TEMP_HOTEND
      print_heater_state(thermalManager.degHotend(target_extruder), thermalManager.degTargetHotend(target_extruder)
        #if ENABLED(SHOW_TEMP_ADC_VALUES)
          , thermalManager.rawHotendTemp(target_extruder)
        #endif
      );
    #endif
    #if HAS_TEMP_BED
      print_heater_state(thermalManager.degBed(), thermalManager.degTargetBed()
        #if ENABLED(SHOW_TEMP_ADC_VALUES)
          , thermalManager.rawBedTemp()
          SERIAL_CHR(')');
        #endif
        , -1 // BED
      );
    #endif
    #if HOTENDS > 1
      LOOP_HOTEND() print_heater_state(thermalManager.degHotend(h), thermalManager.degTargetHotend(h)
        #if ENABLED(SHOW_TEMP_ADC_VALUES)
          , thermalManager.rawHotendTemp(h)
        #endif
        , h
      );
    #endif
    SERIAL_MV(MSG_AT ":", thermalManager.getHeaterPower(target_extruder));
    #if HAS_TEMP_BED
      SERIAL_MV(MSG_BAT, thermalManager.getBedPower());
    #endif
    #if HOTENDS > 1
      LOOP_HOTEND() {
        SERIAL_MV(MSG_AT, h);
        SERIAL_CHR(':');
        SERIAL_VAL(thermalManager.getHeaterPower(h));
      }
    #endif
  }

#endif

#if HAS_TEMP_CHAMBER

  void print_chamberstate() {
    SERIAL_MSG(" CHAMBER:");
    SERIAL_MV(MSG_C, thermalManager.degChamber(), 1);
    SERIAL_MV(" /", thermalManager.degTargetChamber());
    SERIAL_MSG(MSG_CAT);
    #if ENABLED(CHAMBER_WATTS)
      SERIAL_VAL(((CHAMBER_WATTS) * thermalManager.getChamberPower()) / 127.0);
      SERIAL_MSG("W");
    #else
      SERIAL_VAL(thermalManager.getChamberPower());
    #endif
    #if ENABLED(SHOW_TEMP_ADC_VALUES)
      SERIAL_MV(" ADC C:", thermalManager.degChamber(), 1);
      SERIAL_MV(" C->", thermalManager.rawChamberTemp());
    #endif
  }

#endif // HAS_TEMP_CHAMBER

#if HAS_TEMP_COOLER

  void print_coolerstate() {
    SERIAL_MSG(" COOL:");
    SERIAL_MV(MSG_C, thermalManager.degCooler(), 1);
    SERIAL_MV(" /", thermalManager.degTargetCooler());
    SERIAL_MSG(MSG_CAT);
    #if ENABLED(COOLER_WATTS)
      SERIAL_VAL(((COOLER_WATTS) * thermalManager.getCoolerPower()) / 127.0);
      SERIAL_MSG("W");
    #else
      SERIAL_VAL(thermalManager.getCoolerPower());
    #endif
    #if ENABLED(SHOW_TEMP_ADC_VALUES)
      SERIAL_MV(" ADC C:", thermalManager.degCooler(), 1);
      SERIAL_MV(" C->", thermalManager.rawCoolerTemp());
    #endif
  }

#endif // HAS_TEMP_COOLER

#if ENABLED(FLOWMETER_SENSOR)

  void print_flowratestate() {
    float readval = get_flowrate();

    #if ENABLED(MINFLOW_PROTECTION)
      if(readval > MINFLOW_PROTECTION)
        flow_firstread = true;
    #endif

    SERIAL_MV(" FLOW: ", readval);
    SERIAL_MSG(" l/min ");
  }

#endif

#if ENABLED(ARDUINO_ARCH_SAM)&& !MB(RADDS)

  void print_MCUstate() {
    SERIAL_MSG(" MCU: min");
    SERIAL_MV(MSG_C, thermalManager.lowest_temperature_mcu, 1);
    SERIAL_MSG(", current");
    SERIAL_MV(MSG_C, thermalManager.current_temperature_mcu, 1);
    SERIAL_MSG(", max");
    SERIAL_MV(MSG_C, thermalManager.highest_temperature_mcu, 1);
    #if ENABLED(SHOW_TEMP_ADC_VALUES)
      SERIAL_MV(" C->", thermalManager.rawMCUTemp());
    #endif
  }

#endif

#if ENABLED(CNCROUTER) && ENABLED(FAST_PWM_CNCROUTER)

  void print_cncspeed() {
    SERIAL_MV(" CNC speed: ", getCNCSpeed());
    SERIAL_MSG(" rpm ");
  }

#endif

#if HAS_TEMP_HOTEND

  #if DISABLED(MIN_COOLING_SLOPE_DEG)
    #define MIN_COOLING_SLOPE_DEG 1.50
  #endif
  #if DISABLED(MIN_COOLING_SLOPE_TIME)
    #define MIN_COOLING_SLOPE_TIME 60
  #endif

  inline void wait_heater(bool no_wait_for_cooling = true) {

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
    wait_for_heatup = true;
    millis_t now, next_temp_ms = 0, next_cool_check_ms = 0;

    KEEPALIVE_STATE(WAIT_HEATER);

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
        print_heaterstates();
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
      refresh_cmd_timeout(); // to prevent stepper_inactive_time from running out

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

    } while (wait_for_heatup && TEMP_CONDITIONS);

    if (wait_for_heatup) {
      LCD_MESSAGEPGM(MSG_HEATING_COMPLETE);
      #if ENABLED(PRINTER_EVENT_LEDS)
        #if ENABLED(RGBW_LED)
          set_led_color(0, 0, 0, 255);  // Turn on the WHITE LED
        #else
          set_led_color(255, 255, 255); // Set LEDs All On
        #endif
      #endif
    }

    KEEPALIVE_STATE(IN_HANDLER);
  }

#endif

#if HAS_TEMP_BED

  #if DISABLED(MIN_COOLING_SLOPE_DEG_BED)
    #define MIN_COOLING_SLOPE_DEG_BED 1.50
  #endif
  #if DISABLED(MIN_COOLING_SLOPE_TIME_BED)
    #define MIN_COOLING_SLOPE_TIME_BED 60
  #endif

  inline void wait_bed(bool no_wait_for_cooling = true) {

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
    wait_for_heatup = true;
    millis_t now, next_temp_ms = 0, next_cool_check_ms = 0;

    KEEPALIVE_STATE(WAIT_HEATER);

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
        print_heaterstates();
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
      refresh_cmd_timeout(); // to prevent stepper_inactive_time from running out

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

    } while (wait_for_heatup && TEMP_BED_CONDITIONS);

    if (wait_for_heatup) LCD_MESSAGEPGM(MSG_BED_DONE);

    KEEPALIVE_STATE(IN_HANDLER);
  }

#endif // HAS_TEMP_BED

#if HAS_TEMP_CHAMBER

  inline void wait_chamber(bool no_wait_for_heating = true) {
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
    wait_for_heatup = true;
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
      refresh_cmd_timeout(); // to prevent stepper_inactive_time from running out

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

    } while (wait_for_heatup && TEMP_CHAMBER_CONDITIONS);
    LCD_MESSAGEPGM(MSG_CHAMBER_DONE);
    KEEPALIVE_STATE(IN_HANDLER);
  }

#endif

#if HAS_TEMP_COOLER

  inline void wait_cooler(bool no_wait_for_heating = true) {
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
    wait_for_heatup = true;
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
      refresh_cmd_timeout(); // to prevent stepper_inactive_time from running out

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

    } while (wait_for_heatup && TEMP_COOLER_CONDITIONS);
    LCD_MESSAGEPGM(MSG_COOLER_DONE);
    KEEPALIVE_STATE(IN_HANDLER);
  }

#endif

/**
 * ***************************************************************************
 * ***************************** G-CODE HANDLING *****************************
 * ***************************************************************************
 */

/**
 * Set XYZE mechanics.destination and mechanics.feedrate_mm_s from the current GCode command
 *
 *  - Set mechanics.destination from included axis codes
 *  - Set to current for missing axis codes
 *  - Set the mechanics.feedrate_mm_s, if included
 */
void gcode_get_destination() {

  #if ENABLED(IDLE_OOZING_PREVENT)
    if (parser.seen('E')) IDLE_OOZING_retract(false);
  #endif

  LOOP_XYZE(i) {
    if (parser.seen(axis_codes[i]))
      mechanics.destination[i] = parser.value_axis_units((AxisEnum)i) + (axis_relative_modes[i] || relative_mode ? mechanics.current_position[i] : 0);
    else
      mechanics.destination[i] = mechanics.current_position[i];
  }

  if (parser.seen('F') && parser.value_linear_units() > 0.0)
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

#if ENABLED(HOST_KEEPALIVE_FEATURE)
  /**
   * Output a "busy" message at regular intervals
   * while the machine is not accepting commands.
   */
  void host_keepalive() {
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

#endif //HOST_KEEPALIVE_FEATURE

// Define runplan for move axes
#if IS_KINEMATIC
  #define RUNPLAN(RATE_MM_S) planner.buffer_line_kinematic(mechanics.destination, RATE_MM_S, active_extruder);
#else
  #define RUNPLAN(RATE_MM_S) mechanics.line_to_destination(RATE_MM_S);
#endif

#if ENABLED(ADVANCED_PAUSE_FEATURE)

  static float resume_position[XYZE];
  static bool move_away_flag = false;
  #if HAS_SDSUPPORT
    static bool sd_print_paused = false;
  #endif

  #if HAS_BUZZER

    static void filament_change_beep(const int8_t max_beep_count, const bool init=false) {
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

  static void ensure_safe_temperature() {
    bool heaters_heating = true;

    wait_for_heatup = true;
    while (wait_for_heatup && heaters_heating) {
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
  
  static bool pause_print(const float &retract, const float &retract2, const float &z_lift, const float &x_pos, const float &y_pos,
                          const float &unload_length = 0, const int8_t max_beep_count = 0, const bool show_lcd = false) {

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

  static void wait_for_filament_reload(const int8_t max_beep_count = 0) {
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

  static void resume_print(const float &load_length = 0, const float &initial_extrude_length = 0, const int8_t max_beep_count = 0) {
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

/**************************************************
 ***************** GCode Handlers *****************
 **************************************************/

/**
 * G0, G1: Coordinated movement of X Y Z E axes
 */
inline void gcode_G0_G1(
  #if IS_SCARA
    bool fast_move = false
  #elif ENABLED(LASER)
    bool lfire = false
  #endif
) {
  if (IsRunning()) {
    gcode_get_destination(); // For X Y Z E F

    #if ENABLED(FWRETRACT)
      if (autoretract_enabled && !(parser.seen('X') || parser.seen('Y') || parser.seen('Z')) && parser.seen('E')) {
        const float echange = mechanics.destination[E_AXIS] - mechanics.current_position[E_AXIS];
        // Is this move an attempt to retract or recover?
        if ((echange < -MIN_RETRACT && !retracted[active_extruder]) || (echange > MIN_RETRACT && retracted[active_extruder])) {
          mechanics.current_position[E_AXIS] = mechanics.destination[E_AXIS]; // hide the slicer-generated retract/recover from calculations
          mechanics.sync_plan_position_e();  // AND from the planner
          retract(!retracted[active_extruder]);
          return;
        }
      }
    #endif // FWRETRACT

    #if ENABLED(LASER) && ENABLED(LASER_FIRE_G1)
      if (lfire) {
        #if ENABLED(INTENSITY_IN_BYTE)
          if (parser.seenval('S')) laser.intensity = ((float)parser.value_byte() / 255.0) * 100.0;
        #else
          if (parser.seenval('S')) laser.intensity = parser.value_float();
        #endif
        if (parser.seen('L')) laser.duration = parser.value_ulong();
        if (parser.seen('P')) laser.ppm = parser.value_float();
        if (parser.seen('D')) laser.diagnostics = parser.value_bool();
        if (parser.seen('B')) laser.set_mode(parser.value_int());

        laser.status = LASER_ON;
      }
    #endif

    #if IS_SCARA
      fast_move ? prepare_uninterpolated_move_to_destination() : mechanics.prepare_move_to_destination();
    #else
      mechanics.prepare_move_to_destination();
    #endif

    #if ENABLED(LASER) && ENABLED(LASER_FIRE_G1)
      if (lfire) laser.status = LASER_OFF;
    #endif

  }
}

/**
 * G2: Clockwise Arc
 * G3: Counterclockwise Arc
 *
 * This command has two forms: IJ-form and R-form.
 *
 *  - I specifies an X offset. J specifies a Y offset.
 *    At least one of the IJ parameters is required.
 *    X and Y can be omitted to do a complete circle.
 *    The given XY is not error-checked. The arc ends
 *     based on the angle of the mechanics.destination.
 *    Mixing I or J with R will throw an error.
 *
 *  - R specifies the radius. X or Y is required.
 *    Omitting both X and Y will throw an error.
 *    X or Y must differ from the current XY.
 *    Mixing R with I or J will throw an error.
 *
 *  - P specifies the number of full circles to do
 *    before the specified arc move.
 *
 *  Examples:
 *
 *    G2 I10           ; CW circle centered at X+10
 *    G3 X20 Y12 R14   ; CCW circle with r=14 ending at X20 Y12
 */
#if ENABLED(ARC_SUPPORT)

  inline void gcode_G2_G3(bool clockwise) {

    if (IsRunning()) {

      #if ENABLED(SF_ARC_FIX)
        const bool relative_mode_backup = relative_mode;
        relative_mode = true;
      #endif

      gcode_get_destination();

      #if ENABLED(LASER) && ENABLED(LASER_FIRE_G1)
        #if ENABLED(INTENSITY_IN_BYTE)
          if (parser.seenval('S')) laser.intensity = ((float)parser.value_byte() / 255.0) * 100.0;
        #else
          if (parser.seenval('S')) laser.intensity = parser.value_float();
        #endif
        if (parser.seenval('L')) laser.duration = parser.value_ulong();
        if (parser.seenval('P')) laser.ppm = parser.value_float();
        if (parser.seenval('D')) laser.diagnostics = parser.value_bool();
        if (parser.seenval('B')) laser.set_mode(parser.value_int());

        laser.status = LASER_ON;
      #endif

      #if ENABLED(SF_ARC_FIX)
        relative_mode = relative_mode_backup;
      #endif

      float arc_offset[2] = { 0.0, 0.0 };
      if (parser.seenval('R')) {
        const float r = parser.value_linear_units(),
                    p1 = mechanics.current_position[X_AXIS], q1 = mechanics.current_position[Y_AXIS],
                    p2 = mechanics.destination[X_AXIS],      q2 = mechanics.destination[Y_AXIS];
        if (r && (p2 != p1 || q2 != q1)) {
          const float e = clockwise ^ (r < 0) ? -1 : 1,           // clockwise -1/1, counterclockwise 1/-1
                      dx = p2 - p1, dy = q2 - q1,                 // X and Y differences
                      d = HYPOT(dx, dy),                          // Linear distance between the points
                      h = SQRT(sq(r) - sq(d * 0.5)),              // Distance to the arc pivot-point
                      mx = (p1 + p2) * 0.5, my = (q1 + q2) * 0.5, // Point between the two points
                      sx = -dy / d, sy = dx / d,                  // Slope of the perpendicular bisector
                      cx = mx + e * h * sx, cy = my + e * h * sy; // Pivot-point of the arc
          arc_offset[0] = cx - p1;
          arc_offset[1] = cy - q1;
        }
      }
      else {
        if (parser.seenval('I')) arc_offset[0] = parser.value_linear_units();
        if (parser.seenval('J')) arc_offset[1] = parser.value_linear_units();
      }

      if (arc_offset[0] || arc_offset[1]) {

        #if ENABLED(ARC_P_CIRCLES)
          // P indicates number of circles to do
          int8_t circles_to_do = parser.seen('P') ? parser.value_byte() : 0;
          if (!WITHIN(circles_to_do, 0, 100))
            SERIAL_LM(ER, MSG_ERR_ARC_ARGS);
          while (circles_to_do--)
            plan_arc(current_position, arc_offset, clockwise);
        #endif

        // Send an arc to the planner
        plan_arc(mechanics.destination, arc_offset, clockwise);
        refresh_cmd_timeout();
      }
      else {
        // Bad arguments
        SERIAL_LM(ER, MSG_ERR_ARC_ARGS);
      }

      #if ENABLED(LASER) && ENABLED(LASER_FIRE_G1)
        laser.status = LASER_OFF;
      #endif
    }
  }

#endif // ARC_SUPPORT

/**
 * G4: Dwell S<seconds> or P<milliseconds>
 */
inline void gcode_G4() {
  millis_t dwell_ms = 0;

  if (parser.seenval('P')) dwell_ms = parser.value_millis();              // milliseconds to wait
  if (parser.seenval('S')) dwell_ms = parser.value_millis_from_seconds(); // seconds to wait

  stepper.synchronize();
  refresh_cmd_timeout();
  dwell_ms += previous_cmd_ms;  // keep track of when we started waiting

  if (!lcd_hasstatus()) LCD_MESSAGEPGM(MSG_DWELL);

  while (PENDING(millis(), dwell_ms)) idle();
}

#if ENABLED(G5_BEZIER)

  /**
   * Parameters interpreted according to:
   * http://linuxcnc.org/docs/2.6/html/gcode/gcode.html#sec:G5-Cubic-Spline
   * However I, J omission is not supported at this point; all
   * parameters can be omitted and default to zero.
   */

  /**
   * G5: Cubic B-spline
   */
  inline void gcode_G5() {
    if (IsRunning()) {

      gcode_get_destination();

      const float offset[] = {
        parser.linearval('I'),
        parser.linearval('J'),
        parser.linearval('P'),
        parser.linearval('Q')
      };

      plan_cubic_move(offset);
    }
  }
#endif

#if ENABLED(LASER) && ENABLED(LASER_RASTER)

  inline void gcode_G7() {

    if (parser.seenval('L')) laser.raster_raw_length = parser.value_int();

    if (parser.seenval('$')) {
      laser.raster_direction = parser.value_int();
      mechanics.destination[Y_AXIS] = mechanics.current_position[Y_AXIS] + (laser.raster_mm_per_pulse * laser.raster_aspect_ratio); // increment Y axis
    }

    if (parser.seenval('@')) {
      laser.raster_direction = parser.value_int();
      #if ENABLED(LASER_RASTER_MANUAL_Y_FEED)
        mechanics.destination[X_AXIS] = mechanics.current_position[X_AXIS]; // Dont increment X axis
        mechanics.destination[Y_AXIS] = mechanics.current_position[Y_AXIS]; // Dont increment Y axis
      #else
        switch(laser.raster_direction) {
          case 0:
          case 1:
          case 4:
            mechanics.destination[Y_AXIS] = mechanics.current_position[Y_AXIS] + (laser.raster_mm_per_pulse * laser.raster_aspect_ratio); // increment Y axis
          break;
          case 2:
          case 3:
          case 5:
            mechanics.destination[X_AXIS] = mechanics.current_position[X_AXIS] + (laser.raster_mm_per_pulse * laser.raster_aspect_ratio); // increment X axis
          break;
        }
      #endif
    }

    if (parser.seen('D')) laser.raster_num_pixels = base64_decode(laser.raster_data, parser.string_arg + 1, laser.raster_raw_length);

    switch (laser.raster_direction) {
      case 0: // Negative X
        mechanics.destination[X_AXIS] = mechanics.current_position[X_AXIS] - (laser.raster_mm_per_pulse * laser.raster_num_pixels);
        if (laser.diagnostics) SERIAL_EM("Negative Horizontal Raster Line");
      break;
      case 1: // Positive X
        mechanics.destination[X_AXIS] = mechanics.current_position[X_AXIS] + (laser.raster_mm_per_pulse * laser.raster_num_pixels);
        if (laser.diagnostics) SERIAL_EM("Positive Horizontal Raster Line");
      break;
      case 2: // Negative Vertical
        mechanics.destination[Y_AXIS] = mechanics.current_position[Y_AXIS] - (laser.raster_mm_per_pulse * laser.raster_num_pixels);
        if (laser.diagnostics) SERIAL_EM("Negative Vertical Raster Line");
      break;
      case 3: // Positive Vertical
        mechanics.destination[Y_AXIS] = mechanics.current_position[Y_AXIS] + (laser.raster_mm_per_pulse * laser.raster_num_pixels);
        if (laser.diagnostics) SERIAL_EM("Positive Vertical Raster Line");
      break;
      case 4: // Negative X Positive Y 45deg
        mechanics.destination[X_AXIS] = mechanics.current_position[X_AXIS] - ((laser.raster_mm_per_pulse * laser.raster_num_pixels) * 0.707106);
        mechanics.destination[Y_AXIS] = mechanics.current_position[Y_AXIS] + ((laser.raster_mm_per_pulse * laser.raster_num_pixels) * 0.707106);
        if (laser.diagnostics) SERIAL_EM("Negative X Positive Y 45deg Raster Line");
      break;
      case 5: // Positive X Negarite Y 45deg
        mechanics.destination[X_AXIS] = mechanics.current_position[X_AXIS] + ((laser.raster_mm_per_pulse * laser.raster_num_pixels) * 0.707106);
        mechanics.destination[Y_AXIS] = mechanics.current_position[Y_AXIS] - ((laser.raster_mm_per_pulse * laser.raster_num_pixels) * 0.707106);
        if (laser.diagnostics) SERIAL_EM("Positive X Negarite Y 45deg Raster Line");
      break;
      default:
        if (laser.diagnostics) SERIAL_EM("Unknown direction");
      break;
    }

    laser.ppm = 1 / laser.raster_mm_per_pulse; // number of pulses per millimetre
    laser.duration = (1000000 / mechanics.feedrate_mm_s) / laser.ppm; // (1 second in microseconds / (time to move 1mm in microseconds)) / (pulses per mm) = Duration of pulse, taking into account mechanics.feedrate_mm_s as speed and ppm

    laser.mode = RASTER;
    laser.status = LASER_ON;
    mechanics.prepare_move_to_destination();
  }
#endif

#if ENABLED(FWRETRACT)

  /**
   * G10 - Retract filament according to settings of M207
   * G11 - Recover filament according to settings of M208
   */
  inline void gcode_G10_G11(bool doRetract = false) {
    #if EXTRUDERS > 1
      if (doRetract) {
        retracted_swap[active_extruder] = (parser.seen('S') && parser.value_bool()); // checks for swap retract argument
      }
    #endif
    retract(doRetract
     #if EXTRUDERS > 1
      , retracted_swap[active_extruder]
     #endif
    );
  }

#endif //FWRETRACT

#if ENABLED(NOZZLE_CLEAN_FEATURE)
  /**
   * G12: Clean the nozzle
   */
  inline void gcode_G12() {
    // Don't allow nozzle cleaning without homing first
    if (mechanics.axis_unhomed_error()) { return; }

    const uint8_t pattern = parser.seen('P') ? parser.value_ushort() : 0,
                  strokes = parser.seen('S') ? parser.value_ushort() : NOZZLE_CLEAN_STROKES,
                  objects = parser.seen('T') ? parser.value_ushort() : NOZZLE_CLEAN_TRIANGLES;
    const float   radius  = parser.seen('R') ? parser.value_float()  : NOZZLE_CLEAN_CIRCLE_RADIUS;

    Nozzle::clean(pattern, strokes, radius, objects);
  }
#endif

#if ENABLED(CNC_WORKSPACE_PLANES)

  void report_workspace_plane() {
    SERIAL_SM(ECHO, "Workspace Plane ");
    SERIAL_PS(workspace_plane == PLANE_YZ ? PSTR("YZ\n") : workspace_plane == PLANE_ZX ? PSTR("ZX\n") : PSTR("XY\n"));
  }

  /**
   * G17: Select Plane XY
   * G18: Select Plane ZX
   * G19: Select Plane YZ
   */
  inline void gcode_G17() { workspace_plane = PLANE_XY; }
  inline void gcode_G18() { workspace_plane = PLANE_ZX; }
  inline void gcode_G19() { workspace_plane = PLANE_YZ; }

#endif // CNC_WORKSPACE_PLANES

#if ENABLED(INCH_MODE_SUPPORT)
  /**
   * G20: Set input mode to inches
   */
  inline void gcode_G20() { parser.set_input_linear_units(LINEARUNIT_INCH); }

  /**
   * G21: Set input mode to millimeters
   */
  inline void gcode_G21() { parser.set_input_linear_units(LINEARUNIT_MM); }
#endif

#if ENABLED(NOZZLE_PARK_FEATURE)
  /**
   * G27: Park the nozzle
   */
  inline void gcode_G27() {
    // Don't allow nozzle parking without homing first
    if (mechanics.axis_unhomed_error()) { return; }
    Nozzle::park(parser.seen('P') ? parser.value_ushort() : 0);
  }
#endif // NOZZLE_PARK_FEATURE

#if ENABLED(DEBUG_LEVELING_FEATURE)

  void log_machine_info() {
    SERIAL_MSG("Machine Type: ");
    #if IS_DELTA
      SERIAL_EM("Delta");
    #elif IS_SCARA
      SERIAL_EM("SCARA");
    #elif IS_CORE
      SERIAL_EM("Core");
    #else
      SERIAL_EM("Cartesian");
    #endif

    SERIAL_MSG("Probe: ");
    #if ENABLED(PROBE_MANUALLY)
      SERIAL_EM("PROBE_MANUALLY");
    #elif ENABLED(Z_PROBE_FIX_MOUNTED)
      SERIAL_EM("Z_PROBE_FIX_MOUNTED");
    #elif ENABLED(BLTOUCH)
      SERIAL_EM("BLTOUCH");
    #elif ENABLED(Z_PROBE_SLED)
      SERIAL_EM("Z_PROBE_SLED");
    #elif ENABLED(Z_PROBE_ALLEN_KEY)
      SERIAL_EM("ALLEN KEY");
    #elif HAS_Z_SERVO_PROBE
      SERIAL_EM("SERVO PROBE");
    #else
      SERIAL_EM("NONE");
    #endif

    #if HAS_BED_PROBE
      SERIAL_MV("Probe Offset X:", X_PROBE_OFFSET_FROM_NOZZLE);
      SERIAL_MV(" Y:", Y_PROBE_OFFSET_FROM_NOZZLE);
      SERIAL_MV(" Z:", probe.z_offset);
      #if X_PROBE_OFFSET_FROM_NOZZLE > 0
        SERIAL_MSG(" (Right");
      #elif X_PROBE_OFFSET_FROM_NOZZLE < 0
        SERIAL_MSG(" (Left");
      #elif Y_PROBE_OFFSET_FROM_NOZZLE != 0
        SERIAL_MSG(" (Middle");
      #else
        SERIAL_MSG(" (Aligned With");
      #endif
      #if Y_PROBE_OFFSET_FROM_NOZZLE > 0
        SERIAL_MSG("-Back");
      #elif Y_PROBE_OFFSET_FROM_NOZZLE < 0
        SERIAL_MSG("-Front");
      #elif X_PROBE_OFFSET_FROM_NOZZLE != 0
        SERIAL_MSG("-Center");
      #endif
      if (probe.z_offset < 0)
        SERIAL_MSG(" & Below");
      else if (probe.z_offset > 0)
        SERIAL_MSG(" & Above");
      else
        SERIAL_MSG(" & Same Z as");
      SERIAL_EM(" Nozzle)");
    #endif

    #if HAS_ABL
      SERIAL_MSG("Auto Bed Leveling: ");
      #if ENABLED(AUTO_BED_LEVELING_LINEAR)
        SERIAL_MSG("LINEAR");
      #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
        SERIAL_MSG("BILINEAR");
      #elif ENABLED(AUTO_BED_LEVELING_3POINT)
        SERIAL_MSG("3POINT");
      #endif
      if (bedlevel.leveling_is_active()) {
        SERIAL_EM(" (enabled)");
        #if ABL_PLANAR
          const float diff[XYZ] = {
            mechanics.get_axis_position_mm(X_AXIS) - mechanics.current_position[X_AXIS],
            mechanics.get_axis_position_mm(Y_AXIS) - mechanics.current_position[Y_AXIS],
            mechanics.get_axis_position_mm(Z_AXIS) - mechanics.current_position[Z_AXIS]
          };
          SERIAL_MSG("ABL Adjustment X");
          if (diff[X_AXIS] > 0) SERIAL_CHR('+');
          SERIAL_VAL(diff[X_AXIS]);
          SERIAL_MSG(" Y");
          if (diff[Y_AXIS] > 0) SERIAL_CHR('+');
          SERIAL_VAL(diff[Y_AXIS]);
          SERIAL_MSG(" Z");
          if (diff[Z_AXIS] > 0) SERIAL_CHR('+');
          SERIAL_VAL(diff[Z_AXIS]);
        #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
          SERIAL_MV("ABL Adjustment Z", bedlevel.bilinear_z_offset(mechanics.current_position));
        #endif
      }
      else
        SERIAL_MSG(" (disabled)");

      SERIAL_EOL();

    #elif ENABLED(MESH_BED_LEVELING)

      SERIAL_MSG("Mesh Bed Leveling");
      if (bedlevel.leveling_is_active()) {
        float lz = mechanics.current_position[Z_AXIS];
        bedlevel.apply_leveling(mechanics.current_position[X_AXIS], mechanics.current_position[Y_AXIS], lz);
        SERIAL_EM(" (enabled)");
        SERIAL_MV("MBL Adjustment Z", lz);
      }
      else
        SERIAL_MSG(" (disabled)");

      SERIAL_EOL();

    #endif

  }

#endif // DEBUG_LEVELING_FEATURE

#if ENABLED(PROBE_MANUALLY)
  bool g29_in_progress = false;
#else
  constexpr bool g29_in_progress = false;
#endif

/**
 * G28: Home all axes according to settings
 *
 * Parameters
 *
 *  None  Home to all axes with no parameters.
 *        With QUICK_HOME enabled XY will home together, then Z.
 *
 * Cartesian parameters
 *
 *  X   Home to the X endstop
 *  Y   Home to the Y endstop
 *  Z   Home to the Z endstop
 *  B   Return to back point
 *
 */
inline void gcode_G28(const bool always_home_all) {

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_EM(">>> gcode_G28");
      log_machine_info();
    }
  #endif

  #if HAS_POWER_SWITCH
    if (!powerManager.powersupply_on) powerManager.power_on(); // Power On if power is off
  #endif

  // Wait for planner moves to finish!
  stepper.synchronize();

  // Cancel the active G29 session
  #if ENABLED(PROBE_MANUALLY)
    g29_in_progress = false;
    #if HAS_NEXTION_MANUAL_BED
      LcdBedLevelOff();
    #endif
  #endif

  // Disable the leveling matrix before homing
  #if HAS_LEVELING
    bedlevel.set_bed_leveling_enabled(false);
  #endif

  // Always home with tool 0 active
  #if HOTENDS > 1
    const uint8_t old_tool_index = active_extruder;
    tool_change(0, 0, true);
  #endif

  #if ENABLED(CNC_WORKSPACE_PLANES)
    workspace_plane = PLANE_XY;
  #endif

  #if ENABLED(DUAL_X_CARRIAGE)
    mechanics.hotend_duplication_enabled = false;
  #endif

  setup_for_endstop_or_probe_move();
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) SERIAL_EM("> endstops.enable(true)");
  #endif
  endstops.enable(true); // Enable endstops for next homing move

  bool come_back = parser.seen('B');
  float lastpos[NUM_AXIS];
  float old_feedrate_mm_s;
  if (come_back) {
    old_feedrate_mm_s = mechanics.feedrate_mm_s;
    COPY_ARRAY(lastpos, mechanics.current_position);
  }

  mechanics.Home(always_home_all);

  #if ENABLED(NPR2)
    if ((home_all) || (parser.seen('E'))) {
      mechanics.set_destination_to_current();
      mechanics.destination[E_AXIS] = -200;
      active_driver = active_extruder = 1;
      planner.buffer_line_kinematic(mechanics.destination, COLOR_HOMERATE, active_extruder);
      stepper.synchronize();
      old_color = 99;
      active_driver = active_extruder = 0;
      mechanics.current_position[E_AXIS] = 0;
      mechanics.sync_plan_position_e();
    }
  #endif

  endstops.not_homing();

  #if ENABLED(DELTA_HOME_TO_SAFE_ZONE)
    // move to a height where we can use the full xy-area
    mechanics.do_blocking_move_to_z(mechanics.delta_clip_start_height);
  #endif

  if (come_back) {
    mechanics.feedrate_mm_s = mechanics.homing_feedrate_mm_s[X_AXIS];
    COPY_ARRAY(mechanics.destination, lastpos);
    mechanics.prepare_move_to_destination();
    mechanics.feedrate_mm_s = old_feedrate_mm_s;
  }

  #if ENABLED(NEXTION) && ENABLED(NEXTION_GFX)
    #if MECH(DELTA)
      gfx_clear((X_MAX_POS) * 2, (Y_MAX_POS) * 2, Z_MAX_POS);
      gfx_cursor_to(mechanics.current_position[X_AXIS] + (X_MAX_POS), mechanics.current_position[Y_AXIS] + (Y_MAX_POS), mechanics.current_position[Z_AXIS]);
    #else
      gfx_clear(X_MAX_POS, Y_MAX_POS, Z_MAX_POS);
      gfx_cursor_to(mechanics.current_position[X_AXIS], mechanics.current_position[Y_AXIS], mechanics.current_position[Z_AXIS]);
    #endif
  #endif

  clean_up_after_endstop_or_probe_move();

  stepper.synchronize();

  // Restore the active tool after homing
  #if HOTENDS > 1
    tool_change(old_tool_index, 0, true);
  #endif

  lcd_refresh();

  mechanics.report_current_position();

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) SERIAL_EM("<<< gcode_G28");
  #endif

} // G28

void home_all_axes() { gcode_G28(true); }

#if HAS_PROBING_PROCEDURE
  void out_of_range_error(const char* p_edge) {
    SERIAL_MSG("?Probe ");
    SERIAL_PS(p_edge);
    SERIAL_EM(" position out of range.");
  }
#endif

#if ENABLED(MESH_BED_LEVELING)

  // Save 130 bytes with non-duplication of PSTR
  void say_not_entered() { SERIAL_EM(" not entered."); }

  /**
   * G29: Mesh-based Z probe, probes a grid and produces a
   *      mesh to compensate for variable bed height
   *
   * Parameters With MESH_BED_LEVELING:
   *
   *  S0              Produce a mesh report
   *  S1              Start probing mesh points
   *  S2              Probe the next mesh point
   *  S3 Xn Yn Zn.nn  Manually modify a single point
   *  S4 Zn.nn        Set z offset. Positive away from bed, negative closer to bed.
   *  S5              Reset and disable mesh
   *
   * The S0 report the points as below
   *
   *  +----> X-axis  1-n
   *  |
   *  |
   *  v Y-axis  1-n
   *
   */
  inline void gcode_G29() {

    static int mbl_probe_index = -1;
    #if HAS_SOFTWARE_ENDSTOPS
      static bool enable_soft_endstops;
    #endif

    const MeshLevelingState state = parser.seen('S') ? (MeshLevelingState)parser.value_byte() : MeshReport;
    if (!WITHIN(state, 0, 5)) {
      SERIAL_MSG("S out of range (0-5).");
      return;
    }

    int8_t px, py;

    switch (state) {
      case MeshReport:
        if (bedlevel.leveling_is_valid()) {
          SERIAL_EMT("State: ", bedlevel.leveling_is_active() ? MSG_ON : MSG_OFF);
          bedlevel.mbl_mesh_report();
        }
        else
          SERIAL_EM("Mesh bed leveling has no data.");
        break;

      case MeshStart:
        mbl.reset();
        mbl_probe_index = 0;
        enqueue_and_echo_commands_P(PSTR("G28\nG29 S2"));
        break;

      case MeshNext:
        if (mbl_probe_index < 0) {
          SERIAL_EM("Start mesh probing with \"G29 S1\" first.");
          return;
        }
        // For each G29 S2...
        if (mbl_probe_index == 0) {
          #if HAS_SOFTWARE_ENDSTOPS
            // For the initial G29 S2 save software endstop state
            enable_soft_endstops = endstops.soft_endstops_enabled;
          #endif
        }
        else {
          // For G29 S2 after adjusting Z.
          mbl.set_zigzag_z(mbl_probe_index - 1, mechanics.current_position[Z_AXIS]);
          #if HAS_SOFTWARE_ENDSTOPS
            endstops.soft_endstops_enabled = enable_soft_endstops;
          #endif
        }
        // If there's another point to sample, move there with optional lift.
        if (mbl_probe_index < GRID_MAX_POINTS) {
          mbl.zigzag(mbl_probe_index, px, py);
          mechanics.manual_goto_xy(mbl.index_to_xpos[px], mbl.index_to_ypos[py]);

          #if HAS_SOFTWARE_ENDSTOPS
            // Disable software endstops to allow manual adjustment
            // If G29 is not completed, they will not be re-enabled
            endstops.soft_endstops_enabled = false;
          #endif

          mbl_probe_index++;
        }
        else {
          // One last "return to the bed" (as originally coded) at completion
          mechanics.current_position[Z_AXIS] = LOGICAL_Z_POSITION(Z_MIN_POS) + MANUAL_PROBE_HEIGHT;
          mechanics.line_to_current_position();
          stepper.synchronize();

          // After recording the last point, activate the mbl and home
          mbl_probe_index = -1;
          SERIAL_EM("Mesh probing done.");
          BUZZ(100, 659);
          BUZZ(100, 698);
          bedlevel.mesh_probing_done();
        }
        break;

      case MeshSet:
        if (parser.seenval('X')) {
          px = parser.value_int() - 1;
          if (!WITHIN(px, 0, GRID_MAX_POINTS_X - 1)) {
            SERIAL_EM("X out of range (1-" STRINGIFY(GRID_MAX_POINTS_X) ").");
            return;
          }
        }
        else {
          SERIAL_CHR('X'); say_not_entered();
          return;
        }

        if (parser.seenval('Y')) {
          py = parser.value_int() - 1;
          if (!WITHIN(py, 0, GRID_MAX_POINTS_Y - 1)) {
            SERIAL_EM("Y out of range (1-" STRINGIFY(GRID_MAX_POINTS_Y) ").");
            return;
          }
        }
        else {
          SERIAL_CHR('Y'); say_not_entered();
          return;
        }

        if (parser.seenval('Z')) {
          mbl.z_values[px][py] = parser.value_axis_units(Z_AXIS);
        }
        else {
          SERIAL_CHR('Z'); say_not_entered();
          return;
        }
        break;

      case MeshSetZOffset:
        if (parser.seenval('Z')) {
          mbl.z_offset = parser.value_axis_units(Z_AXIS);
        }
        else {
          SERIAL_CHR('Z'); say_not_entered();
          return;
        }
        break;

      case MeshReset:
        bedlevel.reset_bed_level();
        break;

    } // switch(state)

    mechanics.report_current_position();
  }

#elif HAS_ABL

  #if ABL_GRID
    #if ENABLED(PROBE_Y_FIRST)
      #define PR_OUTER_VAR xCount
      #define PR_OUTER_END abl_grid_points_x
      #define PR_INNER_VAR yCount
      #define PR_INNER_END abl_grid_points_y
    #else
      #define PR_OUTER_VAR yCount
      #define PR_OUTER_END abl_grid_points_y
      #define PR_INNER_VAR xCount
      #define PR_INNER_END abl_grid_points_x
    #endif
  #endif

  /**
   * G29: Detailed Z-Probe, probes the bed at 3 or more points.
   *      Will fail if the printer has not been homed with G28.
   *
   * Enhanced G29 Auto Bed Levelling Probe Routine
   *
   *  D  Dry-Run mode. Just evaluate the bed Topology - Don't apply
   *     or alter the bed level data. Useful to check the topology
   *     after a first run of G29.
   *
   *  J  Jettison current bed leveling data
   *
   *  V  Set the verbose level (0-4). Example: "G29 V3"
   *
   * Parameters With LINEAR leveling only:
   *
   *  P  Set the size of the grid that will be probed (P x P points).
   *     Example: "G29 P4"
   *
   *  X  Set the X size of the grid that will be probed (X x Y points).
   *     Example: "G29 X7 Y5"
   *
   *  Y  Set the Y size of the grid that will be probed (X x Y points).
   *
   *  T  Generate a Bed Topology Report. Example: "G29 P5 T" for a detailed report.
   *     This is useful for manual bed leveling and finding flaws in the bed (to
   *     assist with part placement).
   *     Not supported by non-linear delta printer bed leveling.
   *
   * Parameters With LINEAR and BILINEAR leveling only:
   *
   *  S  Set the XY travel speed between probe points (in units/min)
   *
   *  F  Set the Front limit of the probing grid
   *  B  Set the Back limit of the probing grid
   *  L  Set the Left limit of the probing grid
   *  R  Set the Right limit of the probing grid
   *
   * Parameters with DEBUG_LEVELING_FEATURE only:
   *
   *  C  Make a totally fake grid with no actual probing.
   *     For use in testing when no probing is possible.
   *
   * Parameters with BILINEAR leveling only:
   *
   *  Z  Supply an additional Z probe offset
   *
   * Extra parameters with PROBE_MANUALLY:
   *
   *  To do manual probing simply repeat G29 until the procedure is complete.
   *  The first G29 accepts parameters. 'G29 Q' for status, 'G29 A' to abort.
   *
   *  Q  Query leveling and G29 state
   *
   *  A  Abort current leveling procedure
   *
   *  W  Write a mesh point. (Ignored during leveling.)
   *  X  Required X for mesh point
   *  Y  Required Y for mesh point
   *  Z  Required Z for mesh point
   *
   * Without PROBE_MANUALLY:
   *
   *  E  By default G29 will engage the Z probe, test the bed, then disengage.
   *     Include "E" to engage/disengage the Z probe for each sample.
   *     There's no extra effect if you have a fixed Z probe.
   *
   */
  inline void gcode_G29() {

    // G29 Q is also available if debugging
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      const bool query = parser.seen('Q');
      const uint8_t old_debug_flags = mk_debug_flags;
      if (query) mk_debug_flags |= DEBUG_LEVELING;
      if (DEBUGGING(LEVELING)) {
        DEBUG_POS(">>> gcode_G29", mechanics.current_position);
        log_machine_info();
      }
      mk_debug_flags = old_debug_flags;
      #if DISABLED(PROBE_MANUALLY)
        if (query) return;
      #endif
    #endif

    #if ENABLED(PROBE_MANUALLY)
      const bool seenA = parser.seen('A'), seenQ = parser.seen('Q'), no_action = seenA || seenQ;
    #endif

    #if ENABLED(DEBUG_LEVELING_FEATURE) && DISABLED(PROBE_MANUALLY)
      const bool faux = parser.boolval('C');
    #elif ENABLED(PROBE_MANUALLY)
      const bool faux = no_action;
    #else
      bool constexpr faux = false;
    #endif

    #if MECH(DELTA)
      if (!g29_in_progress) {
        // Homing
        home_all_axes();
        mechanics.do_blocking_move_to_z(_Z_PROBE_DEPLOY_HEIGHT, mechanics.homing_feedrate_mm_s[Z_AXIS]);
      }
    #else
      // Don't allow auto-levelling without homing first
      if (mechanics.axis_unhomed_error()) return;
    #endif

    // Define local vars 'static' for manual probing, 'auto' otherwise
    #if ENABLED(PROBE_MANUALLY)
      #define ABL_VAR static
    #else
      #define ABL_VAR
    #endif

    ABL_VAR int verbose_level;
    ABL_VAR float xProbe, yProbe, measured_z;
    ABL_VAR bool dryrun, abl_should_enable;

    #if ENABLED(PROBE_MANUALLY) || ENABLED(AUTO_BED_LEVELING_LINEAR)
      ABL_VAR int abl_probe_index;
    #endif

    #if HAS_SOFTWARE_ENDSTOPS && ENABLED(PROBE_MANUALLY)
      ABL_VAR bool enable_soft_endstops = true;
    #endif

    #if ABL_GRID

      #if ENABLED(PROBE_MANUALLY)
        ABL_VAR uint8_t PR_OUTER_VAR;
        ABL_VAR  int8_t PR_INNER_VAR;
      #endif

      ABL_VAR int left_probe_bed_position, right_probe_bed_position, front_probe_bed_position, back_probe_bed_position;
      ABL_VAR float xGridSpacing = 0.0,
                    yGridSpacing = 0.0;

      #if ENABLED(AUTO_BED_LEVELING_LINEAR)
        ABL_VAR uint8_t abl_grid_points_x = GRID_MAX_POINTS_X,
                        abl_grid_points_y = GRID_MAX_POINTS_Y;
        ABL_VAR bool    do_topography_map;
      #else // Bilinear
        uint8_t constexpr abl_grid_points_x = GRID_MAX_POINTS_X,
                          abl_grid_points_y = GRID_MAX_POINTS_Y;
      #endif

      #if ENABLED(AUTO_BED_LEVELING_LINEAR) || ENABLED(PROBE_MANUALLY)
        #if ENABLED(AUTO_BED_LEVELING_LINEAR)
          ABL_VAR int abl2;
        #else // Bilinear
          int constexpr abl2 = GRID_MAX_POINTS;
        #endif
      #endif

      #if ENABLED(AUTO_BED_LEVELING_BILINEAR)

        ABL_VAR float zoffset;

      #elif ENABLED(AUTO_BED_LEVELING_LINEAR)

        ABL_VAR int indexIntoAB[GRID_MAX_POINTS_X][GRID_MAX_POINTS_Y];

        ABL_VAR float eqnAMatrix[GRID_MAX_POINTS * 3], // "A" matrix of the linear system of equations
                      eqnBVector[GRID_MAX_POINTS],     // "B" vector of Z points
                      mean;
      #endif

    #elif ENABLED(AUTO_BED_LEVELING_3POINT)

      int constexpr abl2 = 3;

      // Probe at 3 arbitrary points
      ABL_VAR vector_3 points[3] = {
        vector_3(ABL_PROBE_PT_1_X, ABL_PROBE_PT_1_Y, 0),
        vector_3(ABL_PROBE_PT_2_X, ABL_PROBE_PT_2_Y, 0),
        vector_3(ABL_PROBE_PT_3_X, ABL_PROBE_PT_3_Y, 0)
      };

    #endif // AUTO_BED_LEVELING_3POINT

    /**
     * On the initial G29 fetch command parameters.
     */
    if (!g29_in_progress) {

      #if ENABLED(PROBE_MANUALLY) || ENABLED(AUTO_BED_LEVELING_LINEAR)
        abl_probe_index = -1;
      #endif

      abl_should_enable = bedlevel.leveling_is_active();

      #if HAS_NEXTION_MANUAL_BED
        LcdBedLevelOn();
      #endif

      #if ENABLED(AUTO_BED_LEVELING_BILINEAR)

        if (parser.seen('W')) {
          if (!bedlevel.leveling_is_valid()) {
            SERIAL_LM(ER, "No bilinear grid");
            return;
          }

          const float z = parser.floatval('Z', RAW_CURRENT_POSITION(Z));
          if (!WITHIN(z, -10, 10)) {
            SERIAL_LM(ER, "Bad Z value");
            return;
          }

          const float x = parser.floatval('X', NAN),
                      y = parser.floatval('Y', NAN);
          int8_t      i = parser.byteval('I', -1),
                      j = parser.byteval('J', -1);

          if (!isnan(x) && !isnan(y)) {
            // Get nearest i / j from x / y
            i = (x - LOGICAL_X_POSITION(bedlevel.bilinear_start[X_AXIS]) + 0.5 * xGridSpacing) / xGridSpacing;
            j = (y - LOGICAL_Y_POSITION(bedlevel.bilinear_start[Y_AXIS]) + 0.5 * yGridSpacing) / yGridSpacing;
            i = constrain(i, 0, GRID_MAX_POINTS_X - 1);
            j = constrain(j, 0, GRID_MAX_POINTS_Y - 1);
          }
          if (WITHIN(i, 0, GRID_MAX_POINTS_X - 1) && WITHIN(j, 0, GRID_MAX_POINTS_Y)) {
            bedlevel.set_bed_leveling_enabled(false);
            bedlevel.z_values[i][j] = z;
            #if ENABLED(ABL_BILINEAR_SUBDIVISION)
              bedlevel.bed_level_virt_interpolate();
            #endif
            bedlevel.set_bed_leveling_enabled(abl_should_enable);
          }
          return;
        } // parser.seen('W')

      #endif

      #if HAS_LEVELING

        // Jettison bed leveling data
        if (parser.seen('J')) {
          bedlevel.reset_bed_level();
          return;
        }

      #endif

      verbose_level = parser.intval('V');
      if (!WITHIN(verbose_level, 0, 4)) {
        SERIAL_EM("?(V)erbose Level is implausible (0-4).");
        return;
      }

      dryrun = parser.boolval('D')
        #if ENABLED(PROBE_MANUALLY)
          || no_action
        #endif
      ;

      #if ENABLED(AUTO_BED_LEVELING_LINEAR)

        do_topography_map = verbose_level > 2 || parser.boolval('T');

        // X and Y specify points in each direction, overriding the default
        // These values may be saved with the completed mesh
        abl_grid_points_x = parser.intval('X', GRID_MAX_POINTS_X);
        abl_grid_points_y = parser.intval('Y', GRID_MAX_POINTS_Y);
        if (parser.seenval('P')) abl_grid_points_x = abl_grid_points_y = parser.value_int();

        if (abl_grid_points_x < 2 || abl_grid_points_y < 2) {
          SERIAL_EM("?Number of probe points is implausible (2 minimum).");
          return;
        }

        abl2 = abl_grid_points_x * abl_grid_points_y;

      #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)

        zoffset = parser.linearval('Z');

      #endif

      #if ABL_GRID

        bedlevel.xy_probe_feedrate_mm_s = MMM_TO_MMS(parser.seen('S') ? parser.value_linear_units() : XY_PROBE_SPEED);

        left_probe_bed_position   = (int)parser.linearval('L', LOGICAL_X_POSITION(LEFT_PROBE_BED_POSITION));
        right_probe_bed_position  = (int)parser.linearval('R', LOGICAL_X_POSITION(RIGHT_PROBE_BED_POSITION));
        front_probe_bed_position  = (int)parser.linearval('F', LOGICAL_Y_POSITION(FRONT_PROBE_BED_POSITION));
        back_probe_bed_position   = (int)parser.linearval('B', LOGICAL_Y_POSITION(BACK_PROBE_BED_POSITION));

        const bool left_out_l   = left_probe_bed_position < LOGICAL_X_POSITION(MIN_PROBE_X),
                   left_out     = left_out_l || left_probe_bed_position > right_probe_bed_position - (MIN_PROBE_EDGE),
                   right_out_r  = right_probe_bed_position > LOGICAL_X_POSITION(MAX_PROBE_X),
                   right_out    = right_out_r || right_probe_bed_position < left_probe_bed_position + MIN_PROBE_EDGE,
                   front_out_f  = front_probe_bed_position < LOGICAL_Y_POSITION(MIN_PROBE_Y),
                   front_out    = front_out_f || front_probe_bed_position > back_probe_bed_position - (MIN_PROBE_EDGE),
                   back_out_b   = back_probe_bed_position > LOGICAL_Y_POSITION(MAX_PROBE_Y),
                   back_out     = back_out_b || back_probe_bed_position < front_probe_bed_position + MIN_PROBE_EDGE;

        if (left_out || right_out || front_out || back_out) {
          if (left_out) {
            out_of_range_error(PSTR("(L)eft"));
            left_probe_bed_position = left_out_l ? LOGICAL_X_POSITION(MIN_PROBE_X) : right_probe_bed_position - (MIN_PROBE_EDGE);
          }
          if (right_out) {
            out_of_range_error(PSTR("(R)ight"));
            right_probe_bed_position = right_out_r ? LOGICAL_Y_POSITION(MAX_PROBE_X) : left_probe_bed_position + MIN_PROBE_EDGE;
          }
          if (front_out) {
            out_of_range_error(PSTR("(F)ront"));
            front_probe_bed_position = front_out_f ? LOGICAL_Y_POSITION(MIN_PROBE_Y) : back_probe_bed_position - (MIN_PROBE_EDGE);
          }
          if (back_out) {
            out_of_range_error(PSTR("(B)ack"));
            back_probe_bed_position = back_out_b ? LOGICAL_Y_POSITION(MAX_PROBE_Y) : front_probe_bed_position + MIN_PROBE_EDGE;
          }
          return;
        }

        // probe at the points of a lattice grid
        xGridSpacing = (right_probe_bed_position - left_probe_bed_position) / (abl_grid_points_x - 1);
        yGridSpacing = (back_probe_bed_position - front_probe_bed_position) / (abl_grid_points_y - 1);

      #endif // ABL_GRID

      if (verbose_level > 0) {
        SERIAL_EM("G29 Auto Bed Leveling");
        if (dryrun) SERIAL_EM("Running in DRY-RUN mode");
      }

      stepper.synchronize();

      // Disable auto bed leveling during G29
      bedlevel.abl_enabled = false;

      if (!dryrun) {
        // Re-orient the current position without leveling
        // based on where the steppers are positioned.
        mechanics.set_current_from_steppers_for_axis(ALL_AXES);

        // Sync the planner to where the steppers stopped
        mechanics.sync_plan_position();
      }

      if (!faux) setup_for_endstop_or_probe_move();

      #if HAS_BED_PROBE
        // Deploy the probe. Probe will raise if needed.
        if (probe.set_deployed(true)) {
          bedlevel.abl_enabled = abl_should_enable;
          return;
        }
      #endif

      #if ENABLED(AUTO_BED_LEVELING_BILINEAR)

        if ( xGridSpacing != bedlevel.bilinear_grid_spacing[X_AXIS]
          || yGridSpacing != bedlevel.bilinear_grid_spacing[Y_AXIS]
          || left_probe_bed_position != LOGICAL_X_POSITION(bedlevel.bilinear_start[X_AXIS])
          || front_probe_bed_position != LOGICAL_Y_POSITION(bedlevel.bilinear_start[Y_AXIS])
        ) {
          if (dryrun) {
            // Before reset bed level, re-enable to correct the position
            bedlevel.abl_enabled = abl_should_enable;
          }
          // Reset grid to 0.0 or "not probed". (Also disables ABL)
          bedlevel.reset_bed_level();

          // Initialize a grid with the given dimensions
          bedlevel.bilinear_grid_spacing[X_AXIS] = xGridSpacing;
          bedlevel.bilinear_grid_spacing[Y_AXIS] = yGridSpacing;
          bedlevel.bilinear_start[X_AXIS] = RAW_X_POSITION(left_probe_bed_position);
          bedlevel.bilinear_start[Y_AXIS] = RAW_Y_POSITION(front_probe_bed_position);

          // Can't re-enable (on error) until the new grid is written
          abl_should_enable = false;
        }

      #elif ENABLED(AUTO_BED_LEVELING_LINEAR)

        mean = 0.0;

      #endif // AUTO_BED_LEVELING_LINEAR

      #if ENABLED(AUTO_BED_LEVELING_3POINT)

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) SERIAL_EM("> 3-point Leveling");
        #endif

        // Probe at 3 arbitrary points
        points[0].z = points[1].z = points[2].z = 0;

      #endif // AUTO_BED_LEVELING_3POINT

    } // !g29_in_progress

    #if ENABLED(PROBE_MANUALLY)

      // For manual probing, get the next index to probe now.
      // On the first probe this will be incremented to 0.
      if (!no_action) {
        ++abl_probe_index;
        g29_in_progress = true;
      }

      // Abort current G29 procedure, go back to ABLStart
      if (seenA && g29_in_progress) {
        SERIAL_EM("Manual G29 aborted");
        #if HAS_SOFTWARE_ENDSTOPS
          endstops.soft_endstops_enabled = enable_soft_endstops;
        #endif
        bedlevel.abl_enabled = abl_should_enable;
        g29_in_progress = false;
        #if ENABLED(LCD_BED_LEVELING) && ENABLED(ULTRA_LCD)
          lcd_wait_for_move = false;
        #endif
      }

      // Query G29 status
      if (verbose_level || seenQ) {
        SERIAL_MSG("Manual G29 ");
        if (g29_in_progress) {
          SERIAL_MV("point ", min(abl_probe_index + 1, abl2));
          SERIAL_EMV(" of ", abl2);
        }
        else
          SERIAL_EM("idle");
      }

      if (no_action) return;

      if (abl_probe_index == 0) {
        // For the initial G29 save software endstop state
        #if HAS_SOFTWARE_ENDSTOPS
          enable_soft_endstops = endstops.soft_endstops_enabled;
        #endif
      }
      else {
        // For G29 after adjusting Z.
        // Save the previous Z before going to the next point
        measured_z = mechanics.current_position[Z_AXIS];

        #if ENABLED(AUTO_BED_LEVELING_LINEAR)

          mean += measured_z;
          eqnBVector[abl_probe_index] = measured_z;
          eqnAMatrix[abl_probe_index + 0 * abl2] = xProbe;
          eqnAMatrix[abl_probe_index + 1 * abl2] = yProbe;
          eqnAMatrix[abl_probe_index + 2 * abl2] = 1;

        #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)

          bedlevel.z_values[xCount][yCount] = measured_z + zoffset;

          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) {
              SERIAL_MV("Save X", xCount);
              SERIAL_MV(" Y", yCount);
              SERIAL_EMV(" Z", bedlevel.z_values[xCount][yCount]);
            }
          #endif

        #elif ENABLED(AUTO_BED_LEVELING_3POINT)

          points[abl_probe_index].z = measured_z;

        #endif
      }

      //
      // If there's another point to sample, move there with optional lift.
      //

      #if ABL_GRID

        // Skip any unreachable points
        while (abl_probe_index < abl2) {

          // Set xCount, yCount based on abl_probe_index, with zig-zag
          PR_OUTER_VAR = abl_probe_index / PR_INNER_END;
          PR_INNER_VAR = abl_probe_index - (PR_OUTER_VAR * PR_INNER_END);

          // Probe in reverse order for every other row/column
          bool zig = (PR_OUTER_VAR & 1); // != ((PR_OUTER_END) & 1);

          if (zig) PR_INNER_VAR = (PR_INNER_END - 1) - PR_INNER_VAR;

          const float xBase = xCount * xGridSpacing + left_probe_bed_position,
                      yBase = yCount * yGridSpacing + front_probe_bed_position;

          xProbe = FLOOR(xBase + (xBase < 0 ? 0 : 0.5));
          yProbe = FLOOR(yBase + (yBase < 0 ? 0 : 0.5));

          #if ENABLED(AUTO_BED_LEVELING_LINEAR)
            indexIntoAB[xCount][yCount] = abl_probe_index;
          #endif

          // Keep looping till a reachable point is found
          if (mechanics.position_is_reachable_xy(xProbe, yProbe)) break;
          ++abl_probe_index;
        }

        // Is there a next point to move to?
        if (abl_probe_index < abl2) {
          mechanics.manual_goto_xy(xProbe, yProbe); // Can be used here too!
          #if HAS_SOFTWARE_ENDSTOPS
            // Disable software endstops to allow manual adjustment
            // If G29 is not completed, they will not be re-enabled
            endstops.soft_endstops_enabled = false;
          #endif
          return;
        }
        else {

          // Leveling done! Fall through to G29 finishing code below

          SERIAL_EM("Grid probing done.");

          // Re-enable software endstops, if needed
          #if HAS_SOFTWARE_ENDSTOPS
            endstops.soft_endstops_enabled = enable_soft_endstops;
          #endif
        }

      #elif ENABLED(AUTO_BED_LEVELING_3POINT)

        // Probe at 3 arbitrary points
        if (abl_probe_index < 3) {
          xProbe = LOGICAL_X_POSITION(points[abl_probe_index].x);
          yProbe = LOGICAL_Y_POSITION(points[abl_probe_index].y);
          #if HAS_SOFTWARE_ENDSTOPS
            // Disable software endstops to allow manual adjustment
            // If G29 is not completed, they will not be re-enabled
            endstops.soft_endstops_enabled = false;
          #endif
          return;
        }
        else {

          SERIAL_EM("3-point probing done.");
          g29_in_progress = false;

          // Re-enable software endstops, if needed
          #if HAS_SOFTWARE_ENDSTOPS
            endstops.soft_endstops_enabled = enable_soft_endstops;
          #endif

          if (!dryrun) {
            vector_3 planeNormal = vector_3::cross(points[0] - points[1], points[2] - points[1]).get_normal();
            if (planeNormal.z < 0) {
              planeNormal.x *= -1;
              planeNormal.y *= -1;
              planeNormal.z *= -1;
            }
            bedlevel.bed_level_matrix = matrix_3x3::create_look_at(planeNormal);

            // Can't re-enable (on error) until the new grid is written
            abl_should_enable = false;
          }

        }

      #endif // AUTO_BED_LEVELING_3POINT

    #else // !PROBE_MANUALLY

      const bool stow_probe_after_each = parser.seen('E');

      #if ABL_GRID

        bool zig = PR_OUTER_END & 1;  // Always end at RIGHT and BACK_PROBE_BED_POSITION

        for (uint8_t PR_OUTER_VAR = 0; PR_OUTER_VAR < PR_OUTER_END; PR_OUTER_VAR++) {

          int8_t inStart, inStop, inInc;

          if (zig) {
            inStart = 0;
            inStop = PR_INNER_END;
            inInc = 1;
          }
          else {
            inStart = PR_INNER_END - 1;
            inStop = -1;
            inInc = -1;
          }

          zig ^= true; // zag

          // Inner loop is Y with PROBE_Y_FIRST enabled
          for (int8_t PR_INNER_VAR = inStart; PR_INNER_VAR != inStop; PR_INNER_VAR += inInc) {

            float xBase = left_probe_bed_position + xGridSpacing * xCount,
                  yBase = front_probe_bed_position + yGridSpacing * yCount;

            xProbe = FLOOR(xBase + (xBase < 0 ? 0 : 0.5));
            yProbe = FLOOR(yBase + (yBase < 0 ? 0 : 0.5));

            #if ENABLED(AUTO_BED_LEVELING_LINEAR)
              indexIntoAB[xCount][yCount] = ++abl_probe_index; // 0...
            #endif

            #if IS_KINEMATIC
              // Avoid probing outside the round or hexagonal area
              if (!mechanics.position_is_reachable_by_probe_xy(xProbe, yProbe)) continue;
            #endif

            measured_z = faux ? 0.001 * random(-100, 101) : probe.check_pt(xProbe, yProbe, stow_probe_after_each, verbose_level);

            if (isnan(measured_z)) {
              bedlevel.abl_enabled = abl_should_enable;
              return;
            }

            #if ENABLED(AUTO_BED_LEVELING_LINEAR)

              mean += measured_z;
              eqnBVector[abl_probe_index] = measured_z;
              eqnAMatrix[abl_probe_index + 0 * abl2] = xProbe;
              eqnAMatrix[abl_probe_index + 1 * abl2] = yProbe;
              eqnAMatrix[abl_probe_index + 2 * abl2] = 1;

            #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)

              bedlevel.z_values[xCount][yCount] = measured_z + zoffset;

            #endif

            abl_should_enable = false;
            idle();

          } // inner
        } // outer

      #elif ENABLED(AUTO_BED_LEVELING_3POINT)

        // Probe at 3 arbitrary points

        for (uint8_t i = 0; i < 3; ++i) {
          // Retain the last probe position
          xProbe = LOGICAL_X_POSITION(points[i].x);
          yProbe = LOGICAL_Y_POSITION(points[i].y);
          measured_z = faux ? 0.001 * random(-100, 101) : probe.check_pt(xProbe, yProbe, stow_probe_after_each, verbose_level);
          if (isnan(measured_z)) {
            bedlevel.abl_enabled = abl_should_enable;
            return;
          }
          points[i].z = measured_z;
        }

        if (!dryrun) {
          vector_3 planeNormal = vector_3::cross(points[0] - points[1], points[2] - points[1]).get_normal();
          if (planeNormal.z < 0) {
            planeNormal.x *= -1;
            planeNormal.y *= -1;
            planeNormal.z *= -1;
          }
          bedlevel.bed_level_matrix = matrix_3x3::create_look_at(planeNormal);

          // Can't re-enable (on error) until the new grid is written
          abl_should_enable = false;
        }

      #endif // AUTO_BED_LEVELING_3POINT

      // Raise to _Z_PROBE_DEPLOY_HEIGHT. Stow the probe.
      if (probe.set_deployed(false)) {
        bedlevel.abl_enabled = abl_should_enable;
        return;
      }

    #endif // !PROBE_MANUALLY

    //
    // G29 Finishing Code
    //
    // Unless this is a dry run, auto bed leveling will
    // definitely be enabled after this point.
    //
    // If code above wants to continue leveling, it should
    // return or loop before this point.
    //

    // Restore state after probing
    if (!faux) clean_up_after_endstop_or_probe_move();

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("> probing complete", mechanics.current_position);
    #endif

    #if ENABLED(PROBE_MANUALLY)
      g29_in_progress = false;
      #if ENABLED(LCD_BED_LEVELING) && ENABLED(ULTRA_LCD)
        lcd_wait_for_move = false;
      #endif
    #endif

    // Calculate leveling, print reports, correct the position
    #if ENABLED(AUTO_BED_LEVELING_BILINEAR)

      if (!dryrun) bedlevel.extrapolate_unprobed_bed_level();
      bedlevel.print_bilinear_leveling_grid();

      bedlevel.refresh_bed_level();

      #if ENABLED(ABL_BILINEAR_SUBDIVISION)
        bedlevel.bed_level_virt_print();
      #endif

    #elif ENABLED(AUTO_BED_LEVELING_LINEAR)

      // For LINEAR leveling calculate matrix, print reports, correct the position

      /**
       * solve the plane equation ax + by + d = z
       * A is the matrix with rows [x y 1] for all the probed points
       * B is the vector of the Z positions
       * the normal vector to the plane is formed by the coefficients of the
       * plane equation in the standard form, which is Vx*x+Vy*y+Vz*z+d = 0
       * so Vx = -a Vy = -b Vz = 1 (we want the vector facing towards positive Z
       */
      float plane_equation_coefficients[3];
      qr_solve(plane_equation_coefficients, abl2, 3, eqnAMatrix, eqnBVector);

      mean /= abl2;

      if (verbose_level) {
        SERIAL_MV("Eqn coefficients: a: ", plane_equation_coefficients[0], 8);
        SERIAL_MV(" b: ", plane_equation_coefficients[1], 8);
        SERIAL_EMV(" d: ", plane_equation_coefficients[2], 8);
        if (verbose_level > 2)
          SERIAL_EMV("Mean of sampled points: ", mean, 8);
      }

      // Create the matrix but don't correct the position yet
      if (!dryrun) {
        bedlevel.bed_level_matrix = matrix_3x3::create_look_at(
          vector_3(-plane_equation_coefficients[0], -plane_equation_coefficients[1], 1)
        );
      }

      // Show the Topography map if enabled
      if (do_topography_map) {
        SERIAL_EM(" Bed Height Topography:");
        SERIAL_EM("   +--- BACK --+");
        SERIAL_EM("   |           |");
        SERIAL_EM(" L |    (+)    | R");
        SERIAL_EM(" E |           | I");
        SERIAL_EM(" F | (-) N (+) | G");
        SERIAL_EM(" T |           | H");
        SERIAL_EM("   |    (-)    | T");
        SERIAL_EM("   |           |");
        SERIAL_EM("   O-- FRONT --+");
        SERIAL_EM(" (0,0)");

        float min_diff = 999;

        for (int8_t yy = abl_grid_points_y - 1; yy >= 0; yy--) {
          for (uint8_t xx = 0; xx < abl_grid_points_x; xx++) {
            int ind = indexIntoAB[xx][yy];
            float diff = eqnBVector[ind] - mean,
                  x_tmp = eqnAMatrix[ind + 0 * abl2],
                  y_tmp = eqnAMatrix[ind + 1 * abl2],
                  z_tmp = 0;

            apply_rotation_xyz(bedlevel.bed_level_matrix, x_tmp, y_tmp, z_tmp);

            NOMORE(min_diff, eqnBVector[ind] - z_tmp);

            if (diff >= 0.0)
              SERIAL_MSG(" +");   // Include + for column alignment
            else
              SERIAL_CHR(' ');
            SERIAL_VAL(diff, 5);
          } // xx
          SERIAL_EOL();
        } // yy
        SERIAL_EOL();

        if (verbose_level > 3) {
          SERIAL_EM("\nCorrected Bed Height vs. Bed Topology:");

          for (int8_t yy = abl_grid_points_y - 1; yy >= 0; yy--) {
            for (uint8_t xx = 0; xx < abl_grid_points_x; xx++) {
              int ind = indexIntoAB[xx][yy];
              float x_tmp = eqnAMatrix[ind + 0 * abl2],
                    y_tmp = eqnAMatrix[ind + 1 * abl2],
                    z_tmp = 0;

              apply_rotation_xyz(bedlevel.bed_level_matrix, x_tmp, y_tmp, z_tmp);

              float diff = eqnBVector[ind] - z_tmp - min_diff;
              if (diff >= 0.0)
                SERIAL_MSG(" +");   // Include + for column alignment
              else
                SERIAL_CHR(' ');
              SERIAL_VAL(diff, 5);
            } // xx
            SERIAL_EOL();
          } // yy
          SERIAL_EOL();
        }
      } // do_topography_map

    #endif // AUTO_BED_LEVELING_LINEAR_GRID

    #if ABL_PLANAR

      // For LINEAR and 3POINT leveling correct the current position

      if (verbose_level > 0)
        bedlevel.bed_level_matrix.debug("\n\nBed Level Correction Matrix:");

      if (!dryrun) {
        //
        // Correct the current XYZ position based on the tilted plane.
        //

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("G29 uncorrected XYZ", mechanics.current_position);
        #endif

        float converted[XYZ];
        COPY_ARRAY(converted, mechanics.current_position);

        bedlevel.abl_enabled = true;
        bedlevel.unapply_leveling(converted); // use conversion machinery
        bedlevel.abl_enabled = false;
 
        // Use the last measured distance to the bed, if possible
        if ( NEAR(mechanics.current_position[X_AXIS], xProbe - (X_PROBE_OFFSET_FROM_NOZZLE))
          && NEAR(mechanics.current_position[Y_AXIS], yProbe - (Y_PROBE_OFFSET_FROM_NOZZLE))
        ) {
          float simple_z = mechanics.current_position[Z_AXIS] - measured_z;
          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) {
              SERIAL_MV("Z from Probe:", simple_z);
              SERIAL_MV("  Matrix:", converted[Z_AXIS]);
              SERIAL_EMV("  Discrepancy:", simple_z - converted[Z_AXIS]);
            }
          #endif
          converted[Z_AXIS] = simple_z;
        }

        // The rotated XY and corrected Z are now current_position
        COPY_ARRAY(mechanics.current_position, converted);

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("G29 corrected XYZ", mechanics.current_position);
        #endif
      }

    #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)

      if (!dryrun) {
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) SERIAL_EMV("G29 uncorrected Z:", mechanics.current_position[Z_AXIS]);
        #endif

        // Unapply the offset because it is going to be immediately applied
        // and cause compensation movement in Z. (Just like bedlevel.unapply_leveling)
        mechanics.current_position[Z_AXIS] -= bedlevel.bilinear_z_offset(mechanics.current_position);

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) SERIAL_EMV(" corrected Z:", mechanics.current_position[Z_AXIS]);
        #endif
      }

    #endif // ABL_PLANAR

    #if ENABLED(Z_PROBE_END_SCRIPT)
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) {
          SERIAL_MSG("Z Probe End Script: ");
          SERIAL_EM(Z_PROBE_END_SCRIPT);
        }
      #endif
      enqueue_and_echo_commands_P(PSTR(Z_PROBE_END_SCRIPT));
      stepper.synchronize();
    #endif

    #if HAS_NEXTION_MANUAL_BED
      LcdBedLevelOff();
    #endif

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) SERIAL_EM("<<< gcode_G29");
    #endif

    mechanics.report_current_position();

    KEEPALIVE_STATE(IN_HANDLER);

    // Auto Bed Leveling is complete! Enable if possible.
    bedlevel.abl_enabled = dryrun ? abl_should_enable : true;

    if (bedlevel.abl_enabled)
      mechanics.sync_plan_position();

  }

#endif // HAS_ABL

#if HAS_BED_PROBE

  /**
   * G30: Do a single Z probe
   */
  inline void gcode_G30() { probe.single_probe(); }

  #if ENABLED(Z_PROBE_SLED)

    /**
     * G31: Deploy the Z probe
     */
    inline void gcode_G31() { probe.set_deployed(true); }

    /**
     * G32: Stow the Z probe
     */
    inline void gcode_G32() { probe.set_deployed(false); }

  #endif // Z_PROBE_SLED

#endif // HAS_BED_PROBE

#if ENABLED(DELTA_AUTO_CALIBRATION_1) || ENABLED(DELTA_AUTO_CALIBRATION_2)

  inline void gcode_G33() { mechanics.auto_calibration(); }

#elif ENABLED(DELTA_AUTO_CALIBRATION_3)

  /**
   * G33: Delta AutoCalibration Algorithm based on Rich Cattell Marlin
   * Usage:
   *    G33 <An> <En> <Rn> <In> <Dn> <Tn>
   *      A = Autocalibration +- precision
   *      E = Adjust Endstop +- precision
   *      R = Adjust Endstop & Delta Radius +- precision
   *      I = Adjust Tower
   *      D = Adjust Diagonal Rod
   *      T = Adjust Tower Radius
   */
  inline void gcode_G33() {

    // Homing
    home_all_axes();

    mechanics.do_blocking_move_to_z(_Z_PROBE_DEPLOY_HEIGHT, mechanics.homing_feedrate_mm_s[Z_AXIS]);

    stepper.synchronize();  // wait until the machine is idle

    SERIAL_EM("Starting Auto Calibration...");
    LCD_MESSAGEPGM("Auto Calibration...");

    ac_prec = ((parser.seen('A') || parser.seen('E') || parser.seen('R')) && parser.has_value()) ? constrain(parser.value_float(), 0.01, 1) : AUTOCALIBRATION_PRECISION;

    SERIAL_MV("Calibration precision: +/-", ac_prec, 2);
    SERIAL_EM(" mm");

    // Probe all points
    bed_probe_all();

    // Show calibration report      
    calibration_report();

    if (parser.seen('E')) {
      SERIAL_EM("Calibration Endstop.");
      int iteration = 0;
      do {
        iteration ++;
        SERIAL_EMV("Iteration: ", iteration);

        SERIAL_EM("Checking/Adjusting Endstop offsets");
        adj_endstops();

        bed_probe_all();
        calibration_report();
      } while (FABS(bed_level_x) > ac_prec
            or FABS(bed_level_y) > ac_prec
            or FABS(bed_level_z) > ac_prec);

      SERIAL_EM("Endstop adjustment complete");
    }

    if (parser.seen('R')) {
      SERIAL_EM("Calibration Endstop & Delta Radius.");
      int iteration = 0;
      do {
        iteration ++;
        SERIAL_EMV("Iteration: ", iteration);

        SERIAL_EM("Checking/Adjusting Endstop offsets");
        adj_endstops();

        bed_probe_all();
        calibration_report();

        SERIAL_EM("Checking delta radius");
        adj_deltaradius();

      } while (FABS(bed_level_c) > ac_prec
            or FABS(bed_level_x) > ac_prec
            or FABS(bed_level_y) > ac_prec
            or FABS(bed_level_z) > ac_prec);

      SERIAL_EM("Endstop & Delta Radius adjustment complete");
    }

    if (parser.seen('I')) {
      SERIAL_EMV("Adjusting Tower Delta for tower", parser.value_byte());
      adj_tower_delta(parser.value_byte());
      SERIAL_EM("Tower Delta adjustment complete");
    }

    if (parser.seen('D')) {
      SERIAL_EM("Adjusting Diagonal Rod Length");
      adj_diagrod_length();
      SERIAL_EM("Diagonal Rod Length adjustment complete");
    }

    if (parser.seen('T')) {
      SERIAL_EMV("Adjusting Tower Radius for tower", parser.value_byte());
      adj_tower_radius(parser.value_byte());
      SERIAL_EM("Tower Radius adjustment complete");
    }

    if (parser.seen('A')) {
      SERIAL_EM("Calibration All.");
      int iteration = 0;
      bool dr_adjusted;

      do {
        do {
          iteration ++;
          SERIAL_EMV("Iteration: ", iteration);

          SERIAL_EM("Checking/Adjusting endstop offsets");
          adj_endstops();

          bed_probe_all();
          calibration_report();

          if (FABS(bed_level_c) > ac_prec) {
            SERIAL_EM("Checking delta radius");
            dr_adjusted = adj_deltaradius();
          }
          else
            dr_adjusted = false;

          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) {
              SERIAL_EMV("bed_level_c = ", bed_level_c, 4);
              SERIAL_EMV("bed_level_x = ", bed_level_x, 4);
              SERIAL_EMV("bed_level_y = ", bed_level_y, 4);
              SERIAL_EMV("bed_level_z = ", bed_level_z, 4);
            }
          #endif

        } while (FABS(bed_level_c) > ac_prec
              or FABS(bed_level_x) > ac_prec
              or FABS(bed_level_y) > ac_prec
              or FABS(bed_level_z) > ac_prec
              or dr_adjusted);

        if  (FABS(bed_level_ox) > ac_prec
          or FABS(bed_level_oy) > ac_prec
          or FABS(bed_level_oz) > ac_prec) {
          SERIAL_EM("Checking for tower geometry errors..");
          if (fix_tower_errors() != 0 ) {
            // Tower positions have been changed .. home to endstops
            SERIAL_EM("Tower Positions changed .. Homing");
            home_all_axes();
            probe.raise(_Z_PROBE_DEPLOY_HEIGHT);
          }
          else {
            SERIAL_EM("Checking Diagonal Rod Length");
            if (adj_diagrod_length() != 0) { 
              // If diagonal rod length has been changed .. home to endstops
              SERIAL_EM("Diagonal Rod Length changed .. Homing");
              home_all_axes();
              probe.raise(_Z_PROBE_DEPLOY_HEIGHT);
            }
          }
          bed_probe_all();
          calibration_report();
        }

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) {
            SERIAL_EMV("bed_level_c = ", bed_level_c, 4);
            SERIAL_EMV("bed_level_x = ", bed_level_x, 4);
            SERIAL_EMV("bed_level_y = ", bed_level_y, 4);
            SERIAL_EMV("bed_level_z = ", bed_level_z, 4);
            SERIAL_EMV("bed_level_ox = ", bed_level_ox, 4);
            SERIAL_EMV("bed_level_oy = ", bed_level_oy, 4);
            SERIAL_EMV("bed_level_oz = ", bed_level_oz, 4);
          }
        #endif
      } while(FABS(bed_level_c) > ac_prec
           or FABS(bed_level_x) > ac_prec
           or FABS(bed_level_y) > ac_prec
           or FABS(bed_level_z) > ac_prec
           or FABS(bed_level_ox) > ac_prec
           or FABS(bed_level_oy) > ac_prec
           or FABS(bed_level_oz) > ac_prec);

      SERIAL_EM("Autocalibration Complete");
    }

    probe.set_deployed(false);

    // reset LCD alert message
    lcd_reset_alert_level();

    clean_up_after_endstop_or_probe_move();

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) SERIAL_EM("<<< gcode_G33");
    #endif

    mechanics.report_current_position();
    KEEPALIVE_STATE(IN_HANDLER);
  }

#endif // DELTA_AUTO_CALIBRATION_1, DELTA_AUTO_CALIBRATION_2 or DELTA_AUTO_CALIBRATION_3

#if ENABLED(G38_PROBE_TARGET)

  static bool G38_run_probe() {

    bool G38_pass_fail = false;

    // Get direction of move and retract
    float retract_mm[XYZ];
    LOOP_XYZ(i) {
      float dist = mechanics.destination[i] - mechanics.current_position[i];
      retract_mm[i] = FABS(dist) < G38_MINIMUM_MOVE ? 0 : mechanics.home_bump_mm((AxisEnum)i) * (dist > 0 ? -1 : 1);
    }

    stepper.synchronize();  // wait until the machine is idle

    // Move until mechanics.destination reached or target hit
    endstops.enable(true);
    G38_move = true;
    G38_endstop_hit = false;
    mechanics.prepare_move_to_destination();
    stepper.synchronize();
    G38_move = false;

    endstops.hit_on_purpose();
    mechanics.set_current_from_steppers_for_axis(ALL_AXES);
    mechanics.sync_plan_position();

    // Only do remaining moves if target was hit
    if (G38_endstop_hit) {

      G38_pass_fail = true;

      // Move away by the retract distance
      mechanics.set_destination_to_current();
      LOOP_XYZ(i) mechanics.destination[i] += retract_mm[i];
      endstops.enable(false);
      mechanics.prepare_move_to_destination();
      stepper.synchronize();

      mechanics.feedrate_mm_s /= 4;

      // Bump the target more slowly
      LOOP_XYZ(i) mechanics.destination[i] -= retract_mm[i] * 2;

      endstops.enable(true);
      G38_move = true;
      mechanics.prepare_move_to_destination();
      stepper.synchronize();
      G38_move = false;

      mechanics.set_current_from_steppers_for_axis(ALL_AXES);
      mechanics.sync_plan_position();
    }

    endstops.hit_on_purpose();
    endstops.not_homing();
    return G38_pass_fail;
  }

  /**
   * G38.2 - probe toward workpiece, stop on contact, signal error if failure
   * G38.3 - probe toward workpiece, stop on contact
   *
   * Like G28 except uses Z min endstop for all axes
   */
  inline void gcode_G38(bool is_38_2) {
    // Get X Y Z E F
    gcode_get_destination();

    setup_for_endstop_or_probe_move();

    // If any axis has enough movement, do the move
    LOOP_XYZ(i)
      if (FABS(mechanics.destination[i] - mechanics.current_position[i]) >= G38_MINIMUM_MOVE) {
        if (!parser.seenval('F')) mechanics.feedrate_mm_s = mechanics.homing_feedrate_mm_s[i];
        // If G38.2 fails throw an error
        if (!G38_run_probe() && is_38_2) {
          SERIAL_LM(ER, "Failed to reach target");
        }
        break;
      }

    clean_up_after_endstop_or_probe_move();
  }

#endif // G38_PROBE_TARGET

#if ENABLED(AUTO_BED_LEVELING_BILINEAR) || ENABLED(MESH_BED_LEVELING)

  /**
   * G42: Move X & Y axes to mesh coordinates (I & J)
   */
  inline void gcode_G42() {
    if (IsRunning()) {
      const bool hasI = parser.seenval('I');
      const int8_t ix = hasI ? parser.value_int() : 0;
      const bool hasJ = parser.seenval('J');
      const int8_t iy = hasJ ? parser.value_int() : 0;

      if ((hasI && !WITHIN(ix, 0, GRID_MAX_POINTS_X - 1)) || (hasJ && !WITHIN(iy, 0, GRID_MAX_POINTS_Y - 1))) {
        SERIAL_EM(MSG_ERR_MESH_XY);
        return;
      }

      #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
        #define _GET_MESH_X(I) bedlevel.bilinear_start[X_AXIS] + I * bedlevel.bilinear_grid_spacing[X_AXIS]
        #define _GET_MESH_Y(J) bedlevel.bilinear_start[Y_AXIS] + J * bedlevel.bilinear_grid_spacing[Y_AXIS]
      #elif ENABLED(MESH_BED_LEVELING)
        #define _GET_MESH_X(I) mbl.index_to_xpos[I]
        #define _GET_MESH_Y(J) mbl.index_to_ypos[J]
      #endif

      mechanics.set_destination_to_current();
      if (hasI) mechanics.destination[X_AXIS] = LOGICAL_X_POSITION(_GET_MESH_X(ix));
      if (hasJ) mechanics.destination[Y_AXIS] = LOGICAL_Y_POSITION(_GET_MESH_Y(iy));
      if (parser.boolval('P')) {
        if (hasI) mechanics.destination[X_AXIS] -= X_PROBE_OFFSET_FROM_NOZZLE;
        if (hasJ) mechanics.destination[Y_AXIS] -= Y_PROBE_OFFSET_FROM_NOZZLE;
      }

      const float fval = parser.linearval('F');
      if (fval > 0.0) mechanics.feedrate_mm_s = MMM_TO_MMS(fval);

      mechanics.prepare_move_to_destination();

    }
  }

#endif // ENABLED(AUTO_BED_LEVELING_BILINEAR) || ENABLED(MESH_BED_LEVELING)

/**
 * G60:  save current position
 *        S<slot> specifies memory slot # (0-based) to save into (default 0)
 */
inline void gcode_G60() {

  const uint8_t slot = parser.byteval('S');

  if (slot >= NUM_POSITON_SLOTS) {
    SERIAL_LMV(ER, MSG_INVALID_POS_SLOT, (int)NUM_POSITON_SLOTS);
    return;
  } 
  COPY_ARRAY(mechanics.stored_position[slot], mechanics.current_position);
  pos_saved = true;

  SERIAL_MSG(MSG_SAVED_POS);
  SERIAL_MV(" S", slot);
  SERIAL_MV("<-X:", mechanics.stored_position[slot][X_AXIS]);
  SERIAL_MV(" Y:", mechanics.stored_position[slot][Y_AXIS]);
  SERIAL_MV(" Z:", mechanics.stored_position[slot][Z_AXIS]);
  SERIAL_EMV(" E:", mechanics.stored_position[slot][E_AXIS]);
}

/**
 * G61:  Apply/restore saved coordinates to the active extruder.
 *        X Y Z E - Value to add at stored coordinates.
 *        F<speed> - Set Feedrate.
 *        S<slot> specifies memory slot # (0-based) to save into (default 0).
 */
inline void gcode_G61() {

  if (!pos_saved) return;

  const uint8_t slot = parser.byteval('S');

  if (slot >= NUM_POSITON_SLOTS) {
    SERIAL_LMV(ER, MSG_INVALID_POS_SLOT, (int)NUM_POSITON_SLOTS);
    return;
  }

  SERIAL_MSG(MSG_RESTORING_POS);
  SERIAL_MV(" S", slot);
  SERIAL_MSG("->");

  if (parser.seen('F') && parser.value_linear_units() > 0.0)
    mechanics.feedrate_mm_s = MMM_TO_MMS(parser.value_linear_units());

  LOOP_XYZE(i) {
    if (parser.seen(axis_codes[i])) {
      mechanics.destination[i] = parser.value_axis_units((AxisEnum)i) + mechanics.stored_position[slot][i];
    }
    else {
      mechanics.destination[i] = mechanics.current_position[i];
    }
    SERIAL_MV(" ", axis_codes[i]);
    SERIAL_MV(":", mechanics.destination[i]);
  }
  SERIAL_EOL();

  // finish moves
  mechanics.prepare_move_to_destination();
  stepper.synchronize();
}

/**
 * G92: Set current position to given X Y Z E
 */
inline void gcode_G92() {
  bool didXYZ = false,
       didE = parser.seenval('E');

  if (!didE) stepper.synchronize();

  LOOP_XYZE(i) {
    if (parser.seenval(axis_codes[i])) {
      #if IS_SCARA
        mechanics.current_position[i] = parser.value_axis_units((AxisEnum)i);
        if (i != E_AXIS) didXYZ = true;
      #else
        #if ENABLED(WORKSPACE_OFFSETS)
          const float p = mechanics.current_position[i];
        #endif
        float v = parser.value_axis_units((AxisEnum)i);

        mechanics.current_position[i] = v;

        if (i != E_AXIS) {
          didXYZ = true;
          #if ENABLED(WORKSPACE_OFFSETS)
            mechanics.position_shift[i] += v - p; // Offset the coordinate space
            endstops.update_software_endstops((AxisEnum)i);
          #endif
        }
      #endif
    }
  }
  if (didXYZ)
    mechanics.sync_plan_position();
  else if (didE)
    mechanics.sync_plan_position_e();

  mechanics.report_current_position();
}

#if HAS_RESUME_CONTINUE

  /**
   * M0: Unconditional stop - Wait for user button press on LCD
   * M1: Conditional stop   - Wait for user button press on LCD
   */
  inline void gcode_M0_M1() {
    const char * const args = parser.string_arg;

    millis_t ms = 0;
    bool hasP = false, hasS = false;
    if (parser.seenval('P')) {
      ms = parser.value_millis(); // milliseconds to wait
      hasP = ms > 0;
    }
    if (parser.seenval('S')) {
      ms = parser.value_millis_from_seconds(); // seconds to wait
      hasS = ms > 0;
    }

    #if ENABLED(ULTIPANEL)

      if (!hasP && !hasS && *args != '\0')
        lcd_setstatus(args, true);
      else {
        LCD_MESSAGEPGM(MSG_USERWAIT);
        #if ENABLED(LCD_PROGRESS_BAR) && PROGRESS_MSG_EXPIRE > 0
          dontExpireStatus();
        #endif
      }

    #else

      if (!hasP && !hasS && *args != '\0')
        SERIAL_LV(ECHO, args);

    #endif

    wait_for_user = true;
    KEEPALIVE_STATE(PAUSED_FOR_USER);

    stepper.synchronize();
    refresh_cmd_timeout();

    if (ms > 0) {
      ms += previous_cmd_ms;  // wait until this time for a click
      while (PENDING(millis(), ms) && wait_for_user) idle();
    }
    else {
      #if ENABLED(ULTIPANEL)
        if (lcd_detected()) {
          while (wait_for_user) idle();
          IS_SD_PRINTING ? LCD_MESSAGEPGM(MSG_RESUMING) : LCD_MESSAGEPGM(WELCOME_MSG);
        }
      #else
        while (wait_for_user) idle();
      #endif
    }

    wait_for_user = false;
    KEEPALIVE_STATE(IN_HANDLER);
  }
#endif // HAS_RESUME_CONTINUE

#if HAS_MULTI_MODE

  /**
   * M3: Setting laser beam or fire laser - CNC clockwise speed
   * M4: Turn on laser beam or CNC counter clockwise speed
   *      S - Laser intensity or CNC speed
   *      L - Laser Duration
   *      P - PPM
   *      D - Diagnostic
   *      B - Set mode
   */
  inline void gcode_M3_M4(bool clockwise) {
    stepper.synchronize();

    switch (printer_mode) {

      #if ENABLED(LASER) && ENABLED(LASER_FIRE_SPINDLE)
        case PRINTER_MODE_LASER: {
          if (IsRunning()) {
            #if ENABLED(INTENSITY_IN_BYTE)
              if (parser.seenval('S')) laser.intensity = (float)(parser.value_byte() / 255) * 100.0;
            #else
              if (parser.seenval('S')) laser.intensity = parser.value_float();
            #endif
            if (parser.seenval('L')) laser.duration = parser.value_ulong();
            if (parser.seenval('P')) laser.ppm = parser.value_float();
            if (parser.seenval('D')) laser.diagnostics = parser.value_bool();
            if (parser.seenval('B')) laser.set_mode(parser.value_int());
          }
          laser.status = LASER_ON;
        }
        break;
      #endif

      #if ENABLED(CNCROUTER)
        case PRINTER_MODE_CNC:
          if (parser.seenval('S')) setCNCRouterSpeed(parser.value_ulong(), clockwise);
        break;
      #endif

      default: break; // other tools

    } // printer_mode

    mechanics.prepare_move_to_destination();
  }

  /**
   * M5: Turn off laser beam - CNC off
   */
  inline void gcode_M5() {
    stepper.synchronize();

    switch (printer_mode) {
    
      #if ENABLED(LASER)
        case PRINTER_MODE_LASER: {
          if (laser.status != LASER_OFF) {
            laser.status = LASER_OFF;
            laser.mode = CONTINUOUS;
            laser.duration = 0;

            if (laser.diagnostics)
              SERIAL_EM("Laser M5 called and laser OFF");
          }
        }
        break;
      #endif

      #if ENABLED(CNCROUTER)
        case PRINTER_MODE_CNC:
          disable_cncrouter();
        break;
      #endif

      default: break; // other tools

    } // printer_mode

    mechanics.prepare_move_to_destination();

  }

  #if ENABLED(CNCROUTER)
    /*
     * M6: CNC tool change
     */
    inline void gcode_M6() { tool_change_cnc(CNC_M6_TOOL_ID); }
  #endif

#endif // HAS_MULTI_MODE

/**
 * M17: Enable power on all stepper motors
 */
inline void gcode_M17() {
  LCD_MESSAGEPGM(MSG_NO_MOVE);
  stepper.enable_all_steppers();
}

#if HAS_SDSUPPORT

  /**
   * M20: List SD card to serial output
   */
  inline void gcode_M20() {
    SERIAL_EM(MSG_BEGIN_FILE_LIST);
    card.ls();
    SERIAL_EM(MSG_END_FILE_LIST);
  }

  /**
   * M21: Init SD Card
   */
  inline void gcode_M21() {
    card.mount();
  }

  /**
   * M22: Release SD Card
   */
  inline void gcode_M22() {
    card.unmount();
  }

  /**
   * M23: Select a file
   */
  inline void gcode_M23() {
    card.selectFile(parser.string_arg);
  }

  /**
   * M24: Start or Resume SD Print
   */
  inline void gcode_M24() {
    #if ENABLED(PARK_HEAD_ON_PAUSE)
      resume_print();
    #endif

    card.startFileprint();
    print_job_counter.start();
    #if HAS_POWER_CONSUMPTION_SENSOR
      startpower = power_consumption_hour;
    #endif
  }

  /**
   * M25: Pause SD Print
   */
  inline void gcode_M25() {
    card.pauseSDPrint();
    print_job_counter.pause();
    SERIAL_LM(REQUEST_PAUSE, "SD pause");

    #if ENABLED(PARK_HEAD_ON_PAUSE)
      enqueue_and_echo_commands_P(PSTR("M125")); // Must be enqueued with pauseSDPrint set to be last in the buffer
    #endif
  }

  /**
   * M26: Set SD Card file index
   */
  inline void gcode_M26() {
    if (card.cardOK && parser.seen('S'))
      card.setIndex(parser.value_long());
  }

  /**
   * M27: Get SD Card status
   */
  inline void gcode_M27() {
    card.printStatus();
  }

  /**
   * M28: Start SD Write
   */
  inline void gcode_M28() {
    card.startWrite(parser.string_arg, false);
  }

  /**
   * M29: Stop SD Write
   * Processed in write to file routine above
   */
  inline void gcode_M29() {
    // card.saving = false;
  }

  /**
   * M30 <filename>: Delete SD Card file
   */
  inline void gcode_M30() {
    if (card.cardOK) {
      card.closeFile();
      card.deleteFile(parser.string_arg);
    }
  }

#endif // SDSUPPORT

/**
 * M31: Get the time since the start of SD Print (or last M109)
 */
inline void gcode_M31() {
  char buffer[21];
  duration_t elapsed = print_job_counter.duration();
  elapsed.toString(buffer);
  lcd_setstatus(buffer);

  SERIAL_LMT(ECHO, MSG_PRINT_TIME, buffer);
}

#if HAS_SDSUPPORT

  /**
   * M32: Make Directory
   */
  inline void gcode_M32() {
    if (card.cardOK) {
      card.makeDirectory(parser.string_arg);
      card.mount();
    }
  }

  /**
   * M33: Close File and store location in restart.gcode
   */
  inline void gcode_M33() {
    stopSDPrint(true);
  }

  /**
   * M34: Select file and start SD print
   */
  inline void gcode_M34() {
    if (card.sdprinting)
      stepper.synchronize();

    if (card.cardOK) {
      char* namestartpos = (strchr(parser.string_arg, '@'));
      if (namestartpos == NULL) {
        namestartpos = parser.string_arg ; // default name position
      }
      else
        namestartpos++; // to skip the '@'

      SERIAL_MV("Open file: ", namestartpos);
      SERIAL_EM(" and start print.");
      card.selectFile(namestartpos);
      if (parser.seenval('S')) card.setIndex(parser.value_long());

      mechanics.feedrate_mm_s       = 20.0; // 20 units/sec
      mechanics.feedrate_percentage = 100;  // 100% mechanics.feedrate_mm_s
      card.startFileprint();
      print_job_counter.start();
      #if HAS_POWER_CONSUMPTION_SENSOR
        startpower = power_consumption_hour;
      #endif
    }
  }

  #if ENABLED(NEXTION)
    /**
     * M35: Upload Firmware to Nextion from SD
     */
    inline void gcode_M35() {
      UploadNewFirmware();
    }
  #endif

#endif // SDSUPPORT

/**
 * M42: Change pin status via GCode
 *
 *  P<pin>  Pin number (LED if omitted)
 *  S<byte> Pin status from 0 - 255
 */
inline void gcode_M42() {
  if (!parser.seenval('S')) return;
  const byte pin_status = parser.value_byte();

  const Pin pin_number = parser.intval('P', LED_PIN);
  if (pin_number < 0) return;

  if (pin_is_protected(pin_number)) {
    SERIAL_LM(ER, MSG_ERR_PROTECTED_PIN);
    return;
  }

  HAL::pinMode(pin_number, OUTPUT);
  HAL::digitalWrite(pin_number, pin_status);
  HAL::analogWrite(pin_number, pin_status);

  #if FAN_COUNT > 0
    switch (pin_number) {
      #if HAS_FAN0
        case FAN_PIN: fanSpeeds[0] = pin_status; break;
      #endif
      #if HAS_FAN1
        case FAN1_PIN: fanSpeeds[1] = pin_status; break;
      #endif
      #if HAS_FAN2
        case FAN2_PIN: fanSpeeds[2] = pin_status; break;
      #endif
      #if HAS_FAN3
        case FAN3_PIN: fanSpeeds[3] = pin_status; break;
      #endif
    }
  #endif
}

#if ENABLED(PINS_DEBUGGING)

  #include "utility/pinsdebug.h"

  inline void toggle_pins() {
    const bool  I_flag  = parser.boolval('I');
    const int   repeat  = parser.intval('R', 1),
                start   = parser.intval('S'),
                end     = parser.intval('E', NUM_DIGITAL_PINS - 1),
                wait    = parser.intval('W', 500);

    for (Pin pin = start; pin <= end; pin++) {

      if (!I_flag && pin_is_protected(pin)) {
        SERIAL_MV("Sensitive Pin: ", pin);
        SERIAL_EM(" untouched.");
      }
      else {
        SERIAL_MV("Pulsing Pin: ", pin);
        HAL::pinMode(pin, OUTPUT);
        for (int16_t j = 0; j < repeat; j++) {
          HAL::digitalWrite(pin, 0);
          safe_delay(wait);
          HAL::digitalWrite(pin, 1);
          safe_delay(wait);
          HAL::digitalWrite(pin, 0);
          safe_delay(wait);
        }
      }
      SERIAL_EOL();
    }
    SERIAL_EM("Done");
  } // toggle_pins

  inline void servo_probe_test(){
    #if !(NUM_SERVOS >= 1 && HAS_SERVO_0)
      SERIAL_LM(ER, "SERVO not setup");
    #elif !HAS_Z_SERVO_PROBE
      SERIAL_LM(ER, "Z_ENDSTOP_SERVO_NR not setup");
    #else

      const uint8_t probe_index = parser.seen('P') ? parser.value_byte() : Z_ENDSTOP_SERVO_NR;

      SERIAL_EM("Servo probe test");
      SERIAL_EMV(".  Using index:  ", probe_index);
      SERIAL_EMV(".  Deploy angle: ", probe.z_servo_angle[0]);
      SERIAL_EMV(".  Stow angle:   ", probe.z_servo_angle[1]);

      bool probe_inverting;

      #if HAS_Z_PROBE_PIN

        #define PROBE_TEST_PIN Z_PROBE_PIN

        SERIAL_EMV("Probe uses Z_MIN_PROBE_PIN: ", PROBE_TEST_PIN);
        SERIAL_EM(".  Uses Z_PROBE_ENDSTOP_INVERTING (ignores Z_MIN_ENDSTOP_INVERTING)");
        SERIAL_MSG(".  Z_PROBE_ENDSTOP_INVERTING: ");

        #if Z_PROBE_ENDSTOP_INVERTING
          SERIAL_EM("true");
        #else
          SERIAL_EM("false");
        #endif

        probe_inverting = Z_PROBE_ENDSTOP_INVERTING;

      #elif HAS_Z_MIN

        #define PROBE_TEST_PIN Z_MIN_PIN

        SERIAL_EMV("Probe uses Z_MIN pin: ", PROBE_TEST_PIN);
        SERIAL_EM(".  Uses Z_MIN_ENDSTOP_INVERTING (ignores Z_PROBE_ENDSTOP_INVERTING)");
        SERIAL_MSG(".  Z_MIN_ENDSTOP_INVERTING: ");

        #if Z_MIN_ENDSTOP_INVERTING
          SERIAL_EM("true");
        #else
          SERIAL_EM("false");
        #endif

        probe_inverting = Z_MIN_ENDSTOP_INVERTING;

      #endif

      SERIAL_EM("Deploy & stow 4 times");
      SET_INPUT_PULLUP(PROBE_TEST_PIN);
      bool deploy_state, stow_state;
      for (uint8_t i = 0; i < 4; i++) {
        servo[probe_index].move(probe.z_servo_angle[0]); // deploy
        safe_delay(500);
        deploy_state = digitalRead(PROBE_TEST_PIN);
        servo[probe_index].move(probe.z_servo_angle[1]); // stow
        safe_delay(500);
        stow_state = digitalRead(PROBE_TEST_PIN);
      }
      if (probe_inverting != deploy_state) SERIAL_EM("WARNING - INVERTING setting probably backwards");

      refresh_cmd_timeout();

      if (deploy_state != stow_state) {
        SERIAL_EM("BLTouch clone detected");
        if (deploy_state) {
          SERIAL_EM(".  DEPLOYED state: HIGH (logic 1)");
          SERIAL_EM(".  STOWED (triggered) state: LOW (logic 0)");
        }
        else {
          SERIAL_EM(".  DEPLOYED state: LOW (logic 0)");
          SERIAL_EM(".  STOWED (triggered) state: HIGH (logic 1)");
        }
        #if ENABLED(BLTOUCH)
          SERIAL_EM("ERROR: BLTOUCH enabled - set this device up as a Z Servo Probe with inverting as true.");
        #endif

      }
      else {                                              // measure active signal length
        servo[probe_index].move(probe.z_servo_angle[0]);  // deploy
        safe_delay(500);
        SERIAL_EM("please trigger probe");
        uint16_t probe_counter = 0;

        // Allow 30 seconds max for operator to trigger probe
        for (uint16_t j = 0; j < 500 * 30 && probe_counter == 0 ; j++) {

          safe_delay(2);

          if (0 == j % (500 * 1)) // keep cmd_timeout happy
            refresh_cmd_timeout();

          if (deploy_state != digitalRead(PROBE_TEST_PIN)) { // probe triggered

            for (probe_counter = 1; probe_counter < 50 && (deploy_state != digitalRead(PROBE_TEST_PIN)); probe_counter ++)
              safe_delay(2);

            if (probe_counter == 50)
              SERIAL_EM("Z Servo Probe detected");   // >= 100mS active time
            else if (probe_counter >= 2 )
              SERIAL_EMV("BLTouch compatible probe detected - pulse width (+/- 4mS): ", probe_counter * 2 );   // allow 4 - 100mS pulse
            else
              SERIAL_EM("noise detected - please re-run test");   // less than 2mS pulse

            servo[probe_index].move(probe.z_servo_angle[1]); // stow

          } // pulse detected

        } // for loop waiting for trigger

        if (probe_counter == 0) SERIAL_EM("trigger not detected");

      } // measure active signal length

    #endif

  } // servo_probe_test

  /**
   * M43: Pin debug - report pin state, watch pins, toggle pins and servo probe test/report
   *
   *  M43         - report name and state of pin(s)
   *                  P<pin>  Pin to read or watch. If omitted, reads all pins.
   *                  I       Flag to ignore MK4duo's pin protection.
   *
   *  M43 W       - Watch pins -reporting changes- until reset, click, or M108.
   *                  P<pin>  Pin to read or watch. If omitted, read/watch all pins.
   *                  I       Flag to ignore MK4duo's pin protection.
   *
   *  M43 E<bool> - Enable / disable background endstop monitoring
   *                  - Machine continues to operate
   *                  - Reports changes to endstops
   *                  - Toggles LED when an endstop changes
   *                  - Can not reliably catch the 5mS pulse from BLTouch type probes
   *
   *  M43 T       - Toggle pin(s) and report which pin is being toggled
   *                  S<pin>  - Start Pin number.   If not given, will default to 0
   *                  L<pin>  - End Pin number.   If not given, will default to last pin defined for this board
   *                  I       - Flag to ignore MK4duo's pin protection.   Use with caution!!!!
   *                  R       - Repeat pulses on each pin this number of times before continueing to next pin
   *                  W       - Wait time (in miliseconds) between pulses.  If not given will default to 500
   *
   *  M43 S       - Servo probe test
   *                  P<index> - Probe index (optional - defaults to 0)
   */
  inline void gcode_M43() {

    if (parser.seen('T')) {   // must be first ot else it's "S" and "E" parameters will execute endstop or servo test
      toggle_pins();
      return;
    }

    // Enable or disable endstop monitoring
    if (parser.seen('E')) {
      endstop_monitor_flag = parser.value_bool();
      SERIAL_MSG("endstop monitor ");
      SERIAL_TXT(endstop_monitor_flag ? "en" : "dis");
      SERIAL_EM("abled");
      return;
    }

    if (parser.seen('S')) {
      servo_probe_test();
      return;
    }

    // Get the range of pins to test or watch
    const uint8_t first_pin = parser.seen('P') ? parser.value_byte() : 0,
                  last_pin = parser.seen('P') ? first_pin : NUM_DIGITAL_PINS - 1;

    if (first_pin > last_pin) return;

    const bool ignore_protection = parser.seen('I') && parser.value_bool();

    // Watch until click, M108, or reset
    if (parser.seen('W') && parser.value_bool()) { // watch digital pins
      SERIAL_EM("Watching pins");
      byte pin_state[last_pin - first_pin + 1];
      for (int8_t pin = first_pin; pin <= last_pin; pin++) {
        if (pin_is_protected(pin) && !ignore_protection) continue;
        pinMode(pin, INPUT_PULLUP);
        // if (IS_ANALOG(pin))
        //   pin_state[pin - first_pin] = analogRead(pin - analogInputToDigitalPin(0)); // int16_t pin_state[...]
        // else
          pin_state[pin - first_pin] = digitalRead(pin);
      }

      #if HAS(RESUME_CONTINUE)
        wait_for_user = true;
        KEEPALIVE_STATE(PAUSED_FOR_USER);
      #endif

      for(;;) {
        for (int8_t pin = first_pin; pin <= last_pin; pin++) {
          if (pin_is_protected(pin)) continue;
          byte val;
          // if (IS_ANALOG(pin))
          //   val = analogRead(pin - analogInputToDigitalPin(0)); // int16_t val
          // else
            val = digitalRead(pin);
          if (val != pin_state[pin - first_pin]) {
            report_pin_state(pin);
            pin_state[pin - first_pin] = val;
          }
        }

        #if HAS(RESUME_CONTINUE)
          if (!wait_for_user) {
            KEEPALIVE_STATE(IN_HANDLER);
            break;
          }
        #endif

        safe_delay(500);
      }
      return;
    }

    // Report current state of selected pin(s)
    for (uint8_t pin = first_pin; pin <= last_pin; pin++)
      report_pin_state_extended(pin, ignore_protection);
  }

#endif // PINS_DEBUGGING

#if ENABLED(Z_MIN_PROBE_REPEATABILITY_TEST)

  /**
   * M48: Z-Probe repeatability measurement function.
   *
   * Usage:
   *   M48 <P#> <X#> <Y#> <V#> <E> <L#> <S>
   *     P = Number of sampled points (4-50, default 10)
   *     X = Sample X position
   *     Y = Sample Y position
   *     V = Verbose level (0-4, default=1)
   *     E = Engage probe for each reading
   *     L = Number of legs of movement before probe
   *     S = Schizoid (Or Star if you prefer)
   *  
   * This function assumes the bed has been homed.  Specifically, that a G28 command
   * as been issued prior to invoking the M48 Z-Probe repeatability measurement function.
   * Any information generated by a prior G29 Bed leveling command will be lost and need to be
   * regenerated.
   */
  inline void gcode_M48() {

    if (mechanics.axis_unhomed_error()) return;

    int8_t verbose_level = parser.seen('V') ? parser.value_byte() : 1;
    if (!WITHIN(verbose_level, 0, 4)) {
      SERIAL_LM(ER, "?Verbose Level not plausible (0-4).");
      return;
    }

    if (verbose_level > 0)
      SERIAL_EM("M48 Z-Probe Repeatability Test");

    int8_t n_samples = parser.seen('P') ? parser.value_byte() : 10;
    if (!WITHIN(n_samples, 4, 50)) {
      SERIAL_LM(ER, "?Sample size not plausible (4-50).");
      return;
    }

    float X_current = mechanics.current_position[X_AXIS],
          Y_current = mechanics.current_position[Y_AXIS];

    bool stow_probe_after_each = parser.seen('E');

    float X_probe_location = parser.seen('X') ? parser.value_linear_units() : X_current + X_PROBE_OFFSET_FROM_NOZZLE;
    float Y_probe_location = parser.seen('Y') ? parser.value_linear_units() : Y_current + Y_PROBE_OFFSET_FROM_NOZZLE;

    #if NOMECH(DELTA)
      if (!WITHIN(X_probe_location, LOGICAL_X_POSITION(MIN_PROBE_X), LOGICAL_X_POSITION(MAX_PROBE_X))) {
        out_of_range_error(PSTR("X"));
        return;
      }
      if (!WITHIN(Y_probe_location, LOGICAL_Y_POSITION(MIN_PROBE_Y), LOGICAL_Y_POSITION(MAX_PROBE_Y))) {
        out_of_range_error(PSTR("Y"));
        return;
      }
    #else
      if (!mechanics.position_is_reachable_by_probe_xy(X_probe_location, Y_probe_location)) {
        SERIAL_LM(ER, "? (X,Y) location outside of probeable radius.");
        return;
      }
    #endif

    bool seen_L = parser.seen('L');
    uint8_t n_legs = seen_L ? parser.value_byte() : 0;
    if (n_legs > 15) {
      SERIAL_LM(ER, "?Number of legs in movement not plausible (0-15).");
      return;
    }
    if (n_legs == 1) n_legs = 2;

    bool schizoid_flag = parser.seen('S');
    if (schizoid_flag && !seen_L) n_legs = 7;

    /**
     * Now get everything to the specified probe point So we can safely do a
     * probe to get us close to the bed.  If the Z-Axis is far from the bed,
     * we don't want to use that as a starting point for each probe.
     */
    if (verbose_level > 2)
      SERIAL_EM("Positioning the probe...");

    // Disable bed level correction in M48 because we want the raw data when we probe
    #if HAS_LEVELING
      const bool was_enabled = bedlevel.leveling_is_active();
      bedlevel.set_bed_leveling_enabled(false);
    #endif

    setup_for_endstop_or_probe_move();

    // Move to the first point, deploy, and probe
    const float t = probe.check_pt(X_probe_location, Y_probe_location, stow_probe_after_each, verbose_level);
    if (isnan(t)) return;

    randomSeed(millis());

    double mean = 0.0, sigma = 0.0, min = 99999.9, max = -99999.9, sample_set[n_samples];

    for (uint8_t n = 0; n < n_samples; n++) {
      if (n_legs) {
        int dir = (random(0, 10) > 5.0) ? -1 : 1;  // clockwise or counter clockwise
        float angle = random(0.0, 360.0),
              radius = random(
                #if MECH(DELTA)
                  mechanics.delta_probe_radius / 8, mechanics.delta_probe_radius / 3
                #else
                  5, X_MAX_LENGTH / 8
                #endif
              );

        if (verbose_level > 3) {
          SERIAL_MV("Starting radius: ", radius);
          SERIAL_MV("   angle: ", angle);
          SERIAL_MSG(" Direction: ");
          if (dir > 0) SERIAL_MSG("Counter-");
          SERIAL_EM("Clockwise");
        }

        for (uint8_t l = 0; l < n_legs - 1; l++) {
          double delta_angle;

          if (schizoid_flag)
            // The points of a 5 point star are 72 degrees apart.  We need to
            // skip a point and go to the next one on the star.
            delta_angle = dir * 2.0 * 72.0;

          else
            // If we do this line, we are just trying to move further
            // around the circle.
            delta_angle = dir * (float) random(25, 45);

          angle += delta_angle;

          while (angle > 360.0)   // We probably do not need to keep the angle between 0 and 2*PI, but the
            angle -= 360.0;       // Arduino documentation says the trig functions should not be given values
          while (angle < 0.0)     // outside of this range.   It looks like they behave correctly with
            angle += 360.0;       // numbers outside of the range, but just to be safe we clamp them.

          X_current = X_probe_location - (X_PROBE_OFFSET_FROM_NOZZLE) + cos(RADIANS(angle)) * radius;
          Y_current = Y_probe_location - (Y_PROBE_OFFSET_FROM_NOZZLE) + sin(RADIANS(angle)) * radius;

          #if MECH(DELTA)
            // If we have gone out too far, we can do a simple fix and scale the numbers
            // back in closer to the origin.
            while (!mechanics.position_is_reachable_by_probe_xy(X_current, Y_current)) {
              X_current *= 0.8;
              Y_current *= 0.8;
              if (verbose_level > 3) {
                SERIAL_MV("Pulling point towards center:", X_current);
                SERIAL_EMV(", ", Y_current);
              }
            }
          #else
            X_current = constrain(X_current, X_MIN_POS, X_MAX_POS);
            Y_current = constrain(Y_current, Y_MIN_POS, Y_MAX_POS);
          #endif
          if (verbose_level > 3) {
            SERIAL_MSG("Going to:");
            SERIAL_MV(" x: ", X_current);
            SERIAL_MV(" y: ", Y_current);
            SERIAL_EMV("  z: ", mechanics.current_position[Z_AXIS]);
          }
          mechanics.do_blocking_move_to_xy(X_current, Y_current);
        } // n_legs loop
      } // n_legs

      // Probe a single point
      sample_set[n] = probe.check_pt(X_probe_location, Y_probe_location, stow_probe_after_each, 0);

      /**
       * Get the current mean for the data points we have so far
       */
      double sum = 0.0;
      for (uint8_t j = 0; j <= n; j++) sum += sample_set[j];
      mean = sum / (n + 1);

      NOMORE(min, sample_set[n]);
      NOLESS(max, sample_set[n]);

      /**
       * Now, use that mean to calculate the standard deviation for the
       * data points we have so far
       */
      sum = 0.0;
      for (uint8_t j = 0; j <= n; j++)
        sum += sq(sample_set[j] - mean);

      sigma = SQRT(sum / (n + 1));
      if (verbose_level > 0) {
        if (verbose_level > 1) {
          SERIAL_VAL(n + 1);
          SERIAL_MV(" of ", (int)n_samples);
          SERIAL_MV(": z: ", sample_set[n], 3);
          if (verbose_level > 2) {
            SERIAL_MV(" mean: ", mean, 4);
            SERIAL_MV(" sigma: ", sigma, 6);
            SERIAL_MV(" min: ", min, 3);
            SERIAL_MV(" max: ", max, 3);
            SERIAL_MV(" range: ", max - min, 3);
          }
          SERIAL_EOL();
        }
      }

    }  // End of probe loop

    if (probe.set_deployed(false)) return;

    SERIAL_EM("Finished!");

    if (verbose_level > 0) {
      SERIAL_MV("Mean: ", mean, 6);
      SERIAL_MV(" Min: ", min, 3);
      SERIAL_MV(" Max: ", max, 3);
      SERIAL_MV(" Range: ", max - min, 3);
      SERIAL_EOL();
    }

    SERIAL_EMV("Standard Deviation: ", sigma, 6);
    SERIAL_EOL();

    clean_up_after_endstop_or_probe_move();

    // Re-enable bed level correction if it had been on
    #if HAS_LEVELING
      bedlevel.set_bed_leveling_enabled(was_enabled);
    #endif

    mechanics.report_current_position();
  }

#endif // Z_MIN_PROBE_REPEATABILITY_TEST

#if HAS_POWER_CONSUMPTION_SENSOR
  /**
   * M70 - Power consumption sensor calibration
   *
   * Z - Calibrate zero current offset
   * A - Isert readed DC Current value (Ampere)
   * W - Insert readed AC Wattage value (Watt)
   */
  inline void gcode_M70() {
    if(parser.seen('Z')) {
      SERIAL_EMV("Actual POWER_ZERO:", POWER_ZERO, 7);
      SERIAL_EMV("New POWER_ZERO:", raw_analog2voltage(), 7);
      SERIAL_EM("Insert new calculated values into the FW and call \"M70 A\" for the next calibration step.");
    }
    else if(parser.seen('A')) {
      SERIAL_EMV("Actual POWER_ERROR:", POWER_ERROR, 7);
      SERIAL_EMV("New POWER_ERROR:", analog2error(parser.value_float()), 7);
      SERIAL_EM("Insert new calculated values into the FW and call \"M70 W\" for the last calibration step.");
    }
    else if(parser.seen('W')) {
      SERIAL_EMV("Actual POWER_EFFICIENCY:", POWER_EFFICIENCY, 7);
      SERIAL_EMV("New POWER_EFFICIENCY:", analog2efficiency(parser.value_float()), 7);
      SERIAL_EM("Insert new calculated values into the FW and then ACS712 it should be calibrated correctly.");
    }
  }
#endif

/**
 * M75: Start print timer
 */
inline void gcode_M75() { print_job_counter.start(); }

/**
 * M76: Pause print timer
 */
inline void gcode_M76() { print_job_counter.pause(); }

/**
 * M77: Stop print timer
 */
inline void gcode_M77() { print_job_counter.stop(); }

/**
 * M78: Show print statistics
 */
inline void gcode_M78() {
  // "M78 S78" will reset the statistics
  if (parser.seen('S') && parser.value_int() == 78)
    print_job_counter.initStats();
  else print_job_counter.showStats();
}

#if HAS_POWER_SWITCH
  /**
   * M80   : Turn on Power Supply
   * M80 S : Report the current state and exit
   */
  inline void gcode_M80() {

    // S: Report the current power supply state and exit
    if (parser.seen('S')) {
      SERIAL_PS(powerManager.powersupply_on ? PSTR("PS:1\n") : PSTR("PS:0\n"));
      return;
    }

    powerManager.power_on();

    // If you have a switch on suicide pin, this is useful
    // if you want to start another print with suicide feature after
    // a print without suicide...
    #if HAS(SUICIDE)
      OUT_WRITE(SUICIDE_PIN, HIGH);
    #endif

    #if ENABLED(HAVE_TMC2130)
      delay(100);
      tmc2130_init(); // Settings only stick when the driver has power
    #endif

    LCD_MESSAGEPGM(WELCOME_MSG);

    #if ENABLED(LASER) && ENABLED(LASER_PERIPHERALS)
      laser.peripherals_on();
      laser.wait_for_peripherals();
    #endif
  }
#endif // HAS_POWER_SWITCH

/**
 * M81: Turn off Power, including Power Supply, if there is one.
 *
 *      This code should ALWAYS be available for EMERGENCY SHUTDOWN!
 */
inline void gcode_M81() {
  thermalManager.disable_all_heaters();
  thermalManager.disable_all_coolers();
  stepper.finish_and_disable();
  #if FAN_COUNT > 0
    #if FAN_COUNT > 1
      LOOP_FAN() fanSpeeds[f] = 0;
    #else
      fanSpeeds[0] = 0;
    #endif
  #endif

  #if ENABLED(LASER)
    laser.extinguish();
    #if ENABLED(LASER_PERIPHERALS)
      laser.peripherals_off();
    #endif
  #endif

  #if ENABLED(CNCROUTER)
    disable_cncrouter();
  #endif

  safe_delay(1000); // Wait 1 second before switching off

  #if HAS(SUICIDE)
    stepper.synchronize();
    suicide();
  #elif HAS_POWER_SWITCH
    powerManager.power_off();
  #endif

  LCD_MESSAGEPGM(MACHINE_NAME " " MSG_OFF ".");

}

/**
 * M82: Set E codes absolute (default)
 */
inline void gcode_M82() { axis_relative_modes[E_AXIS] = false; }

/**
 * M83: Set E codes relative while in Absolute Coordinates (G90) mode
 */
inline void gcode_M83() { axis_relative_modes[E_AXIS] = true; }

/**
 * M18, M84: Disable stepper motors
 */
inline void gcode_M18_M84() {
  if (parser.seenval('S')) {
    stepper_inactive_time = parser.value_millis_from_seconds();
  }
  else {
    bool all_axis = !(parser.seen_axis());
    if (all_axis) {
      stepper.finish_and_disable();
    }
    else {
      stepper.synchronize();
      if (parser.seen('X')) disable_X();
      if (parser.seen('Y')) disable_Y();
      if (parser.seen('Z')) disable_Z();
      #if E0_ENABLE_PIN != X_ENABLE_PIN && E1_ENABLE_PIN != Y_ENABLE_PIN // Only enable on boards that have seperate ENABLE_PINS
        if (parser.seen('E')) stepper.disable_e_steppers();
      #endif
    }
  }
}

/**
 * M85: Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
 */
inline void gcode_M85() {
  if (parser.seenval('S')) max_inactive_time = parser.value_millis_from_seconds();
}

/**
 * M92: Set axis steps-per-unit for one or more axes, X, Y, Z, and E.
 *      (Follows the same syntax as G92)
 *
 *      With multiple extruders use T to specify which one.
 */
inline void gcode_M92() {

  GET_TARGET_EXTRUDER(92);

  LOOP_XYZE(i) {
    if (parser.seen(axis_codes[i])) {
      if (i == E_AXIS) {
        const float value = parser.value_per_axis_unit((AxisEnum)(E_AXIS + TARGET_EXTRUDER));
        if (value < 20.0) {
          float factor = mechanics.axis_steps_per_mm[E_AXIS + TARGET_EXTRUDER] / value; // increase e constants if M92 E14 is given for netfab.
          mechanics.max_jerk[E_AXIS + TARGET_EXTRUDER] *= factor;
          mechanics.max_feedrate_mm_s[E_AXIS + TARGET_EXTRUDER] *= factor;
          mechanics.max_acceleration_steps_per_s2[E_AXIS + TARGET_EXTRUDER] *= factor;
        }
        mechanics.axis_steps_per_mm[E_AXIS + TARGET_EXTRUDER] = value;
      }
      else {
        mechanics.axis_steps_per_mm[i] = parser.value_per_axis_unit((AxisEnum)i);
      }
    }
  }
  mechanics.refresh_positioning();
}

#if ENABLED(ZWOBBLE)
  /**
   * M96: Print ZWobble value
   */
  inline void gcode_M96() {
    mechanics.report_zwobble();
  }

  /**
   * M97: Set ZWobble value
   */
  inline void gcode_M97() {
    float zVal = -1.0, hVal = -1.0, lVal = -1.0;

    if (parser.seen('A')) mechanics.set_zwobble_amplitude(parser.value_float());
    if (parser.seen('W')) mechanics.set_zwobble_period(parser.value_float());
    if (parser.seen('P')) mechanics.set_zwobble_phase(parser.value_float());
    if (parser.seen('Z')) zVal = parser.value_float();
    if (parser.seen('H')) hVal = parser.value_float();
    if (parser.seen('L')) lVal = parser.value_float();
    if (zVal >= 0 && hVal >= 0) mechanics.set_zwobble_sample(zVal, hVal);
    if (zVal >= 0 && lVal >= 0) mechanics.set_zwobble_scaledsample(zVal, lVal);
    if (lVal >  0 && hVal >  0) mechanics.set_zwobble_scalingfactor(hVal / lVal);
  }
#endif // ZWOBBLE

#if ENABLED(HYSTERESIS)

  /**
   * M98: Print Hysteresis value
   */
  inline void gcode_M98() {
    mechanics.report_hysteresis();
  }

  /**
   * M99: Set Hysteresis value
   */
  inline void gcode_M99() {
    LOOP_XYZE(i) {
      if (parser.seen(axis_codes[i]))
        mechanics.set_hysteresis_axis(i, parser.value_float());
    }
  }
#endif // HYSTERESIS

#if HAS_TEMP_HOTEND

  /**
   * M104: Set hotend temperature
   */
  inline void gcode_M104() {

    GET_TARGET_EXTRUDER(104);

    if (DEBUGGING(DRYRUN)) return;

    #if ENABLED(SINGLENOZZLE)
      if (TARGET_EXTRUDER != active_extruder) return;
    #endif

    if (parser.seenval('S')) {
      const int16_t temp = parser.value_celsius();
      thermalManager.setTargetHotend(temp, TARGET_EXTRUDER);

      #if ENABLED(DUAL_X_CARRIAGE)
        if (mechanics.dual_x_carriage_mode == DXC_DUPLICATION_MODE && TARGET_EXTRUDER == 0)
          thermalManager.setTargetHotend(temp ? temp + mechanics.duplicate_hotend_temp_offset : 0, 1);
      #endif

      if (temp > thermalManager.degHotend(TARGET_EXTRUDER))
        lcd_status_printf_P(0, PSTR("H%i %s"), TARGET_EXTRUDER, MSG_HEATING);
    }

    #if ENABLED(AUTOTEMP)
      planner.autotemp_M104_M109();
    #endif

  }

#endif

/**
 * M105: Read hot end and bed temperature
 */
inline void gcode_M105() {

  GET_TARGET_HOTEND(105);

  #if HAS_TEMP_HOTEND || HAS_TEMP_BED || HAS_TEMP_CHAMBER || HAS_TEMP_COOLER || ENABLED(FLOWMETER_SENSOR) || (ENABLED(CNCROUTER) && ENABLED(FAST_PWM_CNCROUTER))
    SERIAL_STR(OK);
    #if HAS_TEMP_HOTEND || HAS_TEMP_BED
      print_heaterstates();
    #endif
    #if HAS_TEMP_CHAMBER
      print_chamberstate();
    #endif
    #if HAS_TEMP_COOLER
      print_coolerstate();
    #endif
    #if ENABLED(FLOWMETER_SENSOR)
      print_flowratestate();
    #endif
    #if ENABLED(CNCROUTER) && ENABLED(FAST_PWM_CNCROUTER)
      print_cncspeed();
    #endif
    #if ENABLED(ARDUINO_ARCH_SAM) && !MB(RADDS)
      print_MCUstate();
    #endif
  #else // HASNT(TEMP_0) && HASNT(TEMP_BED)
    SERIAL_LM(ER, MSG_ERR_NO_THERMISTORS);
  #endif

  SERIAL_EOL();
}

#if FAN_COUNT > 0

  #if ENABLED(FAN_MIN_PWM)
    #define CALC_FAN_SPEED() (speed ? ( FAN_MIN_PWM + (speed * (255 - FAN_MIN_PWM)) / 255 ) : 0)
  #else
    #define CALC_FAN_SPEED() speed
  #endif

  /**
   * M106: Set Fan Speed
   *
   *  S<int>   Speed between 0-255
   *  P<index> Fan index, if more than one fan
   */
  inline void gcode_M106() {
    uint8_t speed = parser.seen('S') ? parser.value_ushort() : 255,
            fan   = parser.seen('P') ? parser.value_ushort() : 0;

    if (fan >= FAN_COUNT || fanSpeeds[fan] == speed)
      return;

    #if ENABLED(FAN_KICKSTART_TIME)
      if (fanKickstart == 0 && speed > fanSpeeds[fan] && speed < 85) {
        if (fanSpeeds[fan]) fanKickstart = FAN_KICKSTART_TIME / 100;
        else                fanKickstart = FAN_KICKSTART_TIME / 25;
      }
    #endif

    fanSpeeds[fan] = CALC_FAN_SPEED();
  }

  /**
   * M107: Fan Off
   */
  inline void gcode_M107() { 
    uint16_t p = parser.seen('P') ? parser.value_ushort() : 0;
    if (p < FAN_COUNT) fanSpeeds[p] = 0;
  }

#endif // FAN_COUNT > 0

#if DISABLED(EMERGENCY_PARSER)
  /**
   * M108: Cancel heatup and wait for the hotend and bed, this G-code is asynchronously handled in the get_serial_commands() parser
   */
  inline void gcode_M108() { wait_for_heatup = false; }
#endif

#if HAS_TEMP_HOTEND

  /**
   * M109: Sxxx Wait for hotend(s) to reach temperature. Waits only when heating.
   *       Rxxx Wait for hotend(s) to reach temperature. Waits when heating and cooling.
   */
  inline void gcode_M109() {

    GET_TARGET_EXTRUDER(109);
    if (DEBUGGING(DRYRUN)) return;

    #if ENABLED(SINGLENOZZLE)
      if (TARGET_EXTRUDER != active_extruder) return;
    #endif

    const bool no_wait_for_cooling = parser.seenval('S');
    if (no_wait_for_cooling || parser.seenval('R')) {
      const int16_t temp = parser.value_celsius();
      thermalManager.setTargetHotend(temp, TARGET_EXTRUDER);

      #if ENABLED(DUAL_X_CARRIAGE)
        if (mechanics.dual_x_carriage_mode == DXC_DUPLICATION_MODE && TARGET_EXTRUDER == 0)
          thermalManager.setTargetHotend(temp ? temp + mechanics.duplicate_hotend_temp_offset : 0, 1);
      #endif

      if (thermalManager.isHeatingHotend(TARGET_EXTRUDER))
        lcd_status_printf_P(0, PSTR("H%i %s"), TARGET_EXTRUDER, MSG_HEATING);
    }

    #if ENABLED(AUTOTEMP)
      planner.autotemp_M104_M109();
    #endif

    wait_heater(no_wait_for_cooling);
  }

#endif

/**
 * M110: Set Current Line Number
 */
inline void gcode_M110() {
  if (parser.seenval('N')) gcode_LastN = parser.value_long();
}

/**
 * M111: Debug mode Repetier Host compatibile
 */
inline void gcode_M111() {
  mk_debug_flags = parser.byteval('S', (uint8_t)DEBUG_NONE);

  const static char str_debug_1[]   PROGMEM = MSG_DEBUG_ECHO;
  const static char str_debug_2[]   PROGMEM = MSG_DEBUG_INFO;
  const static char str_debug_4[]   PROGMEM = MSG_DEBUG_ERRORS;
  const static char str_debug_8[]   PROGMEM = MSG_DEBUG_DRYRUN;
  const static char str_debug_16[]  PROGMEM = MSG_DEBUG_COMMUNICATION;
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    const static char str_debug_32[] PROGMEM = MSG_DEBUG_LEVELING;
  #endif

  const static char* const debug_strings[] PROGMEM = {
    str_debug_1, str_debug_2, str_debug_4, str_debug_8, str_debug_16,
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      str_debug_32
    #endif
  };

  SERIAL_SM(ECHO, MSG_DEBUG_PREFIX);
  if (mk_debug_flags) {
    uint8_t comma = 0;
    for (uint8_t i = 0; i < COUNT(debug_strings); i++) {
      if (TEST(mk_debug_flags, i)) {
        if (comma++) SERIAL_CHR(',');
        SERIAL_PS((char*)pgm_read_word(&(debug_strings[i])));
      }
    }
  }
  else {
    SERIAL_MSG(MSG_DEBUG_OFF);
  }
  SERIAL_EOL();
}

#if DISABLED(EMERGENCY_PARSER)
  /**
   * M112: Emergency Stop
   */
  inline void gcode_M112() { kill(PSTR(MSG_KILLED)); }
#endif

#if ENABLED(HOST_KEEPALIVE_FEATURE)
  /**
   * M113: Get or set Host Keepalive interval (0 to disable)
   *
   *   S<seconds> Optional. Set the keepalive interval.
   */
  inline void gcode_M113() {
    if (parser.seenval('S')) {
      host_keepalive_interval = parser.value_byte();
      NOMORE(host_keepalive_interval, 60);
    }
    else {
      SERIAL_EMV("M113 S", (unsigned long)host_keepalive_interval);
    }
  }
#endif

/**
 * M114: Report current position to host
 */
inline void gcode_M114() {

  if (parser.seen('D')) {
    report_current_position_detail();
    return;
  }

  stepper.synchronize();
  mechanics.report_current_position();
}

/**
 * M115: Capabilities string
 */
inline void gcode_M115() {
  SERIAL_EM(MSG_M115_REPORT);

  #if ENABLED(EXTENDED_CAPABILITIES_REPORT)

    // EEPROM (M500, M501)
    #if ENABLED(EEPROM_SETTINGS)
      SERIAL_LM(CAP, "EEPROM:1");
    #else
      SERIAL_LM(CAP, "EEPROM:0");
    #endif

    // AUTOREPORT_TEMP (M155)
    #if ENABLED(AUTO_REPORT_TEMPERATURES)
      SERIAL_LM(CAP, "AUTOREPORT_TEMP:1");
    #else
      SERIAL_LM(CAP, "AUTOREPORT_TEMP:0");
    #endif

    // PROGRESS (M530 S L, M531 <file>, M532 X L)
    SERIAL_LM(CAP, "PROGRESS:1");

    // Print Job timer M75, M76, M77
    SERIAL_LM(CAP, "PRINT_JOB:1");

    // AUTOLEVEL (G29)
    #if HAS_ABL
      SERIAL_LM(CAP, "AUTOLEVEL:1");
    #else
      SERIAL_LM(CAP, "AUTOLEVEL:0");
    #endif

    // Z_PROBE (G30)
    #if HAS_BED_PROBE
      SERIAL_LM(CAP, "Z_PROBE:1");
    #else
      SERIAL_LM(CAP, "Z_PROBE:0");
    #endif

    // MESH_REPORT (M320 V, M420 V)
    #if HAS_LEVELING
      SERIAL_LM(CAP, "LEVELING_DATA:1");
    #else
      SERIAL_LM(CAP, "LEVELING_DATA:0");
    #endif

    // SOFTWARE_POWER (M80, M81)
    #if HAS_POWER_SWITCH
      SERIAL_LM(CAP, "SOFTWARE_POWER:1");
    #else
      SERIAL_LM(CAP, "SOFTWARE_POWER:0");
    #endif

    // CASE LIGHTS (M355)
    #if HAS_CASE_LIGHT
      SERIAL_LM(CAP, "TOGGLE_LIGHTS:1");
    #else
      SERIAL_LM(CAP, "TOGGLE_LIGHTS:0");
    #endif

    // EMERGENCY_PARSER (M108, M112, M410)
    #if ENABLED(EMERGENCY_PARSER)
      SERIAL_LM(CAP, "EMERGENCY_PARSER:1");
    #else
      SERIAL_LM(CAP, "EMERGENCY_PARSER:0");
    #endif

  #endif // EXTENDED_CAPABILITIES_REPORT
}

/**
 * M117: Set LCD Status Message
 */
inline void gcode_M117() { lcd_setstatus(parser.string_arg); }

/**
 * M118: Display a message in the host console.
 *
 *  A  Append '// ' for an action command, as in OctoPrint
 *  E  Have the host 'echo:' the text
 */
inline void gcode_M118() {
  if (parser.boolval('E')) SERIAL_STR(ECHO);
  if (parser.boolval('A')) SERIAL_MSG("// ");
  SERIAL_ET(parser.string_arg);
}

/**
 * M119: Output endstop states to serial output
 */
inline void gcode_M119() { endstops.M119(); }

/**
 * M120: Enable endstops and set non-homing endstop state to "enabled"
 */
inline void gcode_M120() { endstops.enable_globally(true); }

/**
 * M121: Disable endstops and set non-homing endstop state to "disabled"
 */
inline void gcode_M121() { endstops.enable_globally(false); }

/**
 * M122: Enable, Disable, and/or Report software endstops
 *
 * Usage: M122 S1 to enable, M122 S0 to disable, M122 alone for report
 */
inline void gcode_M122() {
  #if HAS_SOFTWARE_ENDSTOPS
    if (parser.seen('S')) endstops.soft_endstops_enabled = parser.value_bool();
    SERIAL_SM(ECHO, MSG_SOFT_ENDSTOPS);
    SERIAL_PS(endstops.soft_endstops_enabled ? PSTR(MSG_ON) : PSTR(MSG_OFF));
  #else
    SERIAL_MSG(MSG_SOFT_ENDSTOPS);
    SERIAL_MSG(MSG_OFF);
  #endif
  SERIAL_MSG(MSG_SOFT_MIN);
  SERIAL_MV(    MSG_X, endstops.soft_endstop_min[X_AXIS]);
  SERIAL_MV(" " MSG_Y, endstops.soft_endstop_min[Y_AXIS]);
  SERIAL_MV(" " MSG_Z, endstops.soft_endstop_min[Z_AXIS]);
  SERIAL_MSG(MSG_SOFT_MAX);
  SERIAL_MV(    MSG_X, endstops.soft_endstop_max[X_AXIS]);
  SERIAL_MV(" " MSG_Y, endstops.soft_endstop_max[Y_AXIS]);
  SERIAL_MV(" " MSG_Z, endstops.soft_endstop_max[Z_AXIS]);
  SERIAL_EOL();
}

#if ENABLED(PARK_HEAD_ON_PAUSE)

  /**
   * M125: Store current position and move to pause park position.
   *       Called on pause (by M25) to prevent material leaking onto the
   *       object. On resume (M24) the head will be moved back and the
   *       print will resume.
   *
   *       If MK4duo is compiled without SD Card support, M125 can be
   *       used directly to pause the print and move to park position,
   *       resuming with a button click or M108.
   *
   *    L = override retract length
   *    X = override X
   *    Y = override Y
   *    Z = override Z raise
   */
  inline void gcode_M125() {

    // Initial retract before move to pause park position
    const float retract = parser.seen('L') ? parser.value_axis_units(E_AXIS) : 0
      #if ENABLED(PAUSE_PARK_RETRACT_LENGTH) && PAUSE_PARK_RETRACT_LENGTH > 0
        - (PAUSE_PARK_RETRACT_LENGTH)
      #endif
    ;

    // Lift Z axis
    const float z_lift = parser.seen('Z') ? parser.value_linear_units() :
      #if ENABLED(PAUSE_PARK_Z_ADD) && PAUSE_PARK_Z_ADD > 0
        PAUSE_PARK_Z_ADD
      #else
        0
      #endif
    ;

    // Move XY axes to pause park position or given position
    const float x_pos = parser.seen('X') ? parser.value_linear_units() : 0
      #ifdef PAUSE_PARK_X_POS
        + PAUSE_PARK_X_POS
      #endif
    ;
    const float y_pos = parser.seen('Y') ? parser.value_linear_units() : 0
      #ifdef PAUSE_PARK_Y_POS
        + PAUSE_PARK_Y_POS
      #endif
    ;

    #if HOTENDS > 1 && DISABLED(DUAL_X_CARRIAGE)
      if (active_extruder > 0) {
        if (!parser.seen('X')) x_pos += hotend_offset[X_AXIS][active_extruder];
        if (!parser.seen('Y')) y_pos += hotend_offset[Y_AXIS][active_extruder];
      }
    #endif

    const bool job_running = print_job_counter.isRunning();

    if (pause_print(retract, 0, z_lift, x_pos, y_pos)) {
      if (!IS_SD_PRINTING) {
        // Wait for lcd click or M108
        wait_for_filament_reload();

        // Return to print position and continue
        resume_print();

        if (job_running) print_job_counter.start();
      }
    }
  }

#endif // PARK_HEAD_ON_PAUSE

#if ENABLED(BARICUDA)
  #if HAS_HEATER_1
    /**
     * M126: Heater 1 valve open
     */
    inline void gcode_M126() { baricuda_valve_pressure = parser.byteval('S', 255); }
    /**
     * M127: Heater 1 valve close
     */
    inline void gcode_M127() { baricuda_valve_pressure = 0; }
  #endif

  #if HAS_HEATER_2
    /**
     * M128: Heater 2 valve open
     */
    inline void gcode_M128() { baricuda_e_to_p_pressure = parser.byteval('S', 255); }
    /**
     * M129: Heater 2 valve close
     */
    inline void gcode_M129() { baricuda_e_to_p_pressure = 0; }
  #endif
#endif // BARICUDA

#if HAS_TEMP_BED
  /**
   * M140: Set Bed temperature
   */
  inline void gcode_M140() {
    if (DEBUGGING(DRYRUN)) return;
    if (parser.seenval('S')) thermalManager.setTargetBed(parser.value_celsius());
  }
#endif

#if HAS_TEMP_CHAMBER
  /**
   * M141: Set Chamber temperature
   */
  inline void gcode_M141() {
    if (DEBUGGING(DRYRUN)) return;
    if (parser.seenval('S')) thermalManager.setTargetChamber(parser.value_celsius());
  }
#endif

#if HAS_TEMP_COOLER
  /**
   * M142: Set Cooler temperature
   */
  inline void gcode_M142() {
    if (DEBUGGING(DRYRUN)) return;
    if (parser.seenval('S')) thermalManager.setTargetCooler(parser.value_celsius());
  }
#endif

#if ENABLED(ULTIPANEL) && HAS_TEMP_0

  /**
   * M145: Set the heatup state for a material in the LCD menu
   *   S<material> (0=PLA, 1=ABS, 2=GUM)
   *   H<hotend temp>
   *   B<bed temp>
   *   F<fan speed>
   */
  inline void gcode_M145() {
    uint8_t material = (uint8_t)parser.intval('S');
    if (material >= COUNT(lcd_preheat_hotend_temp)) {
      SERIAL_LM(ER, MSG_ERR_MATERIAL_INDEX);
    }
    else {
      int v;
      if (parser.seenval('H')) {
        v = parser.value_int();
        #if HEATER_0_MAXTEMP
          lcd_preheat_hotend_temp[material] = constrain(v, HEATER_0_MINTEMP, HEATER_0_MAXTEMP - 15);
        #endif
      }
      if (parser.seenval('F')) {
        v = parser.value_int();
        lcd_preheat_fan_speed[material] = constrain(v, 0, 255);
      }
      #if HAS_TEMP_BED
        if (parser.seenval('B')) {
          v = parser.value_int();
          lcd_preheat_bed_temp[material] = constrain(v, BED_MINTEMP, BED_MAXTEMP - 15);
        }
      #endif
    }
  }

#endif

#if ENABLED(TEMPERATURE_UNITS_SUPPORT)
  /**
   * M149: Set temperature units
   */
  inline void gcode_M149() {
         if (parser.seenval('C')) set_input_temp_units(TEMPUNIT_C);
    else if (parser.seenval('K')) set_input_temp_units(TEMPUNIT_K);
    else if (parser.seenval('F')) set_input_temp_units(TEMPUNIT_F);
  }
#endif

#if HAS_COLOR_LEDS

  /**
   * M150: Set Status LED Color - Use R-U-B-W for R-G-B-W
   *
   * Always sets all 3 or 4 components. If a component is left out, set to 0.
   *
   * Examples:
   *
   *   M150 R255       ; Turn LED red
   *   M150 R255 U127  ; Turn LED orange (PWM only)
   *   M150            ; Turn LED off
   *   M150 R U B      ; Turn LED white
   *   M150 W          ; Turn LED white using a white LED
   *
   */
  inline void gcode_M150() {
    set_led_color(
      parser.byteval('R'),
      parser.byteval('U'),
      parser.byteval('B')
      #if ENABLED(RGBW_LED)
        , parser.byteval('W')
      #endif
    );
  }

#endif // HAS_COLOR_LEDS

#if ENABLED(AUTO_REPORT_TEMPERATURES) && (HAS_TEMP_HOTEND || HAS_TEMP_BED)

  static uint8_t auto_report_temp_interval;
  static millis_t next_temp_report_ms;

  /**
   * M155: Set temperature auto-report interval. M155 S<seconds>
   */
  inline void gcode_M155() {
    if (parser.seenval('S')) {
      auto_report_temp_interval = parser.value_byte();
      NOMORE(auto_report_temp_interval, 60);
      next_temp_report_ms = millis() + 1000UL * auto_report_temp_interval;
    }
  }

  inline void auto_report_temperatures() {
    if (auto_report_temp_interval && ELAPSED(millis(), next_temp_report_ms)) {
      next_temp_report_ms = millis() + 1000UL * auto_report_temp_interval;
      print_heaterstates();

      #if ENABLED(ARDUINO_ARCH_SAM) && !MB(RADDS)
        print_MCUstate();
      #endif

      SERIAL_EOL();
    }
  }

#endif // AUTO_REPORT_TEMPERATURES

#if ENABLED(COLOR_MIXING_EXTRUDER)
  /**
   * M163: Set a single mix factor for a mixing extruder
   *       This is called "weight" by some systems.
   *
   *   S[index]   The channel index to set
   *   P[float]   The mix value
   *
   */
  inline void gcode_M163() {
    int mix_index = parser.seen('S') ? parser.value_int() : 0;
    if (mix_index < MIXING_STEPPERS) {
      float mix_value = parser.seen('P') ? parser.value_float() : 0.0;
      NOLESS(mix_value, 0.0);
      mixing_factor[mix_index] = RECIPROCAL(mix_value);
    }
  }

  #if MIXING_VIRTUAL_TOOLS  > 1
    /**
     * M164: Store the current mix factors as a virtual tools.
     *
     *   S[index]   The virtual tools to store
     *
     */
    inline void gcode_M164() {
      int tool_index = parser.seen('S') ? parser.value_int() : 0;
      if (tool_index < MIXING_VIRTUAL_TOOLS) {
        normalize_mix();
        for (uint8_t i = 0; i < MIXING_STEPPERS; i++) {
          mixing_virtual_tool_mix[tool_index][i] = mixing_factor[i];
        }
      }
    }
  #endif

  /**
   * M165: Set multiple mix factors for a mixing extruder.
   *       Factors that are left out will be set to 0.
   *       All factors together must add up to 1.0.
   *
   *   A[factor] Mix factor for extruder stepper 1
   *   B[factor] Mix factor for extruder stepper 2
   *   C[factor] Mix factor for extruder stepper 3
   *   D[factor] Mix factor for extruder stepper 4
   *   H[factor] Mix factor for extruder stepper 5
   *   I[factor] Mix factor for extruder stepper 6
   *
   */
  inline void gcode_M165() { gcode_get_mix(); }
#endif  // COLOR_MIXING_EXTRUDER

#if HAS_TEMP_BED
  /**
   * M190: Sxxx Wait for bed current temp to reach target temp. Waits only when heating
   *       Rxxx Wait for bed current temp to reach target temp. Waits when heating and cooling
   */
  inline void gcode_M190() {
    if (DEBUGGING(DRYRUN)) return;

    LCD_MESSAGEPGM(MSG_BED_HEATING);
    const bool no_wait_for_cooling = parser.seen('S');
    if (no_wait_for_cooling || parser.seen('R'))
      thermalManager.setTargetBed(parser.value_celsius());

    wait_bed(no_wait_for_cooling);
  }
#endif // HAS_TEMP_BED

#if HAS_TEMP_CHAMBER
  /**
   * M191: Sxxx Wait for chamber current temp to reach target temp. Waits only when heating
   *       Rxxx Wait for chamber current temp to reach target temp. Waits when heating and cooling
   */
  inline void gcode_M191() {
    if (DEBUGGING(DRYRUN)) return;

    LCD_MESSAGEPGM(MSG_CHAMBER_HEATING);
    bool no_wait_for_cooling = parser.seen('S');
    if (no_wait_for_cooling || parser.seen('R')) thermalManager.setTargetChamber(parser.value_celsius());

    wait_chamber(no_wait_for_cooling);
  }
#endif // HAS_TEMP_CHAMBER

#if HAS_TEMP_COOLER
  /**
   * M192: Sxxx Wait for cooler current temp to reach target temp. Waits only when heating
   *       Rxxx Wait for cooler current temp to reach target temp. Waits when heating and cooling
   */
  inline void gcode_M192() {
    if (DEBUGGING(DRYRUN)) return;

    LCD_MESSAGEPGM(MSG_COOLER_COOLING);
    bool no_wait_for_heating = parser.seen('S');
    if (no_wait_for_heating || parser.seen('R')) thermalManager.setTargetCooler(parser.value_celsius());

    wait_cooler(no_wait_for_heating);
  }
#endif

/**
 * M200: Set filament diameter and set E axis units to cubic units
 *
 *    T<extruder> - Optional extruder number. Current extruder if omitted.
 *    D<linear> - Diameter of the filament. Use "D0" to switch back to linear units on the E axis.
 */
inline void gcode_M200() {

  GET_TARGET_EXTRUDER(200);

  if (parser.seen('D')) {
    // setting any extruder filament size disables volumetric on the assumption that
    // slicers either generate in extruder values as cubic mm or as as filament feeds
    // for all extruders
    volumetric_enabled = (parser.value_linear_units() != 0.0);
    if (volumetric_enabled) {
      filament_size[TARGET_EXTRUDER] = parser.value_linear_units();
      // make sure all extruders have some sane value for the filament size
      for (int i = 0; i < EXTRUDERS; i++)
        if (!filament_size[i]) filament_size[i] = DEFAULT_NOMINAL_FILAMENT_DIA;
    }
  }
  else {
    // reserved for setting filament diameter via UFID or filament measuring device
    return;
  }

  calculate_volumetric_multipliers();
}

/**
 * M201: Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
 *
 *       With multiple extruders use T to specify which one.
 */
inline void gcode_M201() {

  GET_TARGET_EXTRUDER(201);

  LOOP_XYZE(i) {
    if (parser.seen(axis_codes[i])) {
      const uint8_t a = i + (i == E_AXIS ? TARGET_EXTRUDER : 0);
      mechanics.max_acceleration_mm_per_s2[a] = parser.value_axis_units((AxisEnum)a);
    }
  }
  // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
  mechanics.reset_acceleration_rates();
}

#if 0 // Not used for Sprinter/grbl gen6
  inline void gcode_M202() {
    LOOP_XYZE(i) {
      if(parser.seen(axis_codes[i])) axis_travel_steps_per_sqr_second[i] = parser.value_axis_units((AxisEnum)i) * mechanics.axis_steps_per_mm[i];
    }
  }
#endif

/**
 * M203: Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in units/sec
 *
 *       With multiple extruders use T to specify which one.
 */
inline void gcode_M203() {

  GET_TARGET_EXTRUDER(203);

  LOOP_XYZE(i) {
    if (parser.seen(axis_codes[i])) {
      const uint8_t a = i + (i == E_AXIS ? TARGET_EXTRUDER : 0);
      mechanics.max_feedrate_mm_s[a] = parser.value_axis_units((AxisEnum)a);
    }
  }
}

/**
 * M204: Set planner.accelerations in units/sec^2 (M204 P1200 T0 R3000 V3000)
 *
 *    P     = Printing moves
 *    T* R  = Retract only (no X, Y, Z) moves
 *    V     = Travel (non printing) moves
 *
 *  Also sets minimum segment time in ms (B20000) to prevent buffer under-runs and M20 minimum mechanics.feedrate_mm_s
 */
inline void gcode_M204() {

  GET_TARGET_EXTRUDER(204);

  if (parser.seen('S')) {  // Kept for legacy compatibility. Should NOT BE USED for new developments.
    mechanics.travel_acceleration = mechanics.acceleration = parser.value_linear_units();
    SERIAL_EMV("Setting Print and Travel acceleration: ", mechanics.acceleration );
  }
  if (parser.seen('P')) {
    mechanics.acceleration = parser.value_linear_units();
    SERIAL_EMV("Setting Print acceleration: ", mechanics.acceleration );
  }
  if (parser.seen('R')) {
    mechanics.retract_acceleration[TARGET_EXTRUDER] = parser.value_linear_units();
    SERIAL_EMV("Setting Retract acceleration: ", mechanics.retract_acceleration[TARGET_EXTRUDER]);
  }
  if (parser.seen('V')) {
    mechanics.travel_acceleration = parser.value_linear_units();
    SERIAL_EMV("Setting Travel acceleration: ", mechanics.travel_acceleration );
  }
}

/**
 * M205: Set Advanced Settings
 *
 *    S = Min Feed Rate (units/s)
 *    V = Min Travel Feed Rate (units/s)
 *    B = Min Segment Time (µs)
 *    X = Max X Jerk (units/sec^2)
 *    Y = Max Y Jerk (units/sec^2)
 *    Z = Max Z Jerk (units/sec^2)
 *    E = Max E Jerk (units/sec^2)
 */
inline void gcode_M205() {

  GET_TARGET_EXTRUDER(205);

  if (parser.seen('S')) mechanics.min_feedrate_mm_s = parser.value_linear_units();
  if (parser.seen('V')) mechanics.min_travel_feedrate_mm_s = parser.value_linear_units();
  if (parser.seen('B')) mechanics.min_segment_time = parser.value_millis();
  if (parser.seen('X')) mechanics.max_jerk[X_AXIS] = parser.value_linear_units();
  if (parser.seen('Y')) mechanics.max_jerk[Y_AXIS] = parser.value_linear_units();
  if (parser.seen('Z')) mechanics.max_jerk[Z_AXIS] = parser.value_linear_units();
  if (parser.seen('E')) mechanics.max_jerk[E_AXIS + TARGET_EXTRUDER] = parser.value_linear_units();
}

#if ENABLED(WORKSPACE_OFFSETS)

  /**
   * M206: Set Additional Homing Offset (X Y Z). SCARA aliases T=X, P=Y
   */
  inline void gcode_M206() {
    LOOP_XYZ(i) {
      if (parser.seen(axis_codes[i])) {
        set_home_offset((AxisEnum)i, parser.value_linear_units());
      }
    }
    #if MECH(MORGAN_SCARA)
      if (parser.seen('T')) set_home_offset(X_AXIS, parser.value_linear_units()); // Theta
      if (parser.seen('P')) set_home_offset(Y_AXIS, parser.value_linear_units()); // Psi
    #endif

    mechanics.sync_plan_position();
    mechanics.report_current_position();
  }

#endif // ENABLED(WORKSPACE_OFFSETS)

#if ENABLED(FWRETRACT)
  /**
   * M207: Set firmware retraction values
   *
   *   S[+units]    retract_length
   *   W[+units]    retract_length_swap (multi-extruder)
   *   F[units/min] retract_feedrate_mm_s
   *   Z[units]     retract_zlift
   */
  inline void gcode_M207() {
    if (parser.seenval('S')) retract_length = parser.value_axis_units(E_AXIS);
    if (parser.seenval('F')) retract_feedrate_mm_s = MMM_TO_MMS(parser.value_axis_units(E_AXIS));
    if (parser.seenval('Z')) retract_zlift = parser.value_linear_units();
    #if EXTRUDERS > 1
      if (parser.seenval('W')) retract_length_swap = parser.value_axis_units(E_AXIS);
    #endif
  }

  /**
   * M208: Set firmware un-retraction values
   *
   *   S[+units]    retract_recover_length (in addition to M207 S*)
   *   W[+units]    retract_recover_length_swap (multi-extruder)
   *   F[units/min] retract_recover_feedrate_mm_s
   */
  inline void gcode_M208() {
    if (parser.seenval('S')) retract_recover_length = parser.value_axis_units(E_AXIS);
    if (parser.seenval('F')) retract_recover_feedrate_mm_s = MMM_TO_MMS(parser.value_axis_units(E_AXIS));
    #if EXTRUDERS > 1
      if (parser.seenval('W')) retract_recover_length_swap = parser.value_axis_units(E_AXIS);
    #endif
  }

  /**
   * M209: Enable automatic retract (M209 S1)
   *   For slicers that don't support G10/11, reversed extrude-only
   *   moves will be classified as retraction.
   */
  inline void gcode_M209() {
    if (parser.seenval('S')) {
      autoretract_enabled = parser.value_bool();
      for (int i = 0; i < EXTRUDERS; i++) retracted[i] = false;
    }
  }
#endif // FWRETRACT

/**
 * M218 - set hotend offset (in linear units)
 *
 *   T<tool>
 *   X<xoffset>
 *   Y<yoffset>
 *   Z<zoffset>
 */
inline void gcode_M218() {

  GET_TARGET_HOTEND(218);
  if (TARGET_EXTRUDER == 0) return;

  if (parser.seenval('X')) hotend_offset[X_AXIS][TARGET_EXTRUDER] = parser.value_linear_units();
  if (parser.seenval('Y')) hotend_offset[Y_AXIS][TARGET_EXTRUDER] = parser.value_linear_units();
  if (parser.seenval('Z')) hotend_offset[Z_AXIS][TARGET_EXTRUDER] = parser.value_linear_units();

  SERIAL_SM(ECHO, MSG_HOTEND_OFFSET);
  LOOP_HOTEND() {
    SERIAL_MV(" ", hotend_offset[X_AXIS][h]);
    SERIAL_MV(",", hotend_offset[Y_AXIS][h]);
    SERIAL_MV(",", hotend_offset[Z_AXIS][h]);
  }
  SERIAL_EOL();
}

/**
 * M220: Set speed percentage factor, aka "Feed Rate" (M220 S95)
 */
inline void gcode_M220() {
  if (parser.seenval('S')) mechanics.feedrate_percentage = parser.value_int();
}

/**
 * M221: Set extrusion percentage (M221 T0 S95)
 */
inline void gcode_M221() {

  GET_TARGET_EXTRUDER(221);
  if (parser.seenval('S')) flow_percentage[TARGET_EXTRUDER] = parser.value_int();
}

/**
 * M222: Set density extrusion percentage (M222 T0 S95)
 */
inline void gcode_M222() {

  GET_TARGET_EXTRUDER(222);

  if (parser.seenval('S')) {
    density_percentage[TARGET_EXTRUDER] = parser.value_int();
    #if ENABLED(RFID_MODULE)
      RFID522.RfidData[TARGET_EXTRUDER].data.density = density_percentage[TARGET_EXTRUDER];
    #endif
  }
}

/**
 * M226: Wait until the specified pin reaches the state required (M226 P<pin> S<state>)
 */
inline void gcode_M226() {
  if (parser.seenval('P')) {
    const int pin_number = parser.value_int(),
              pin_state = parser.intval('S', -1); // required pin state - default is inverted

    if (WITHIN(pin_state, -1, 1) && pin_number > -1 && !pin_is_protected(pin_number)) {

      int target = LOW;

      stepper.synchronize();

      HAL::pinMode(pin_number, INPUT);
      switch(pin_state) {
        case 1:
          target = HIGH;
          break;
        case 0:
          target = LOW;
          break;
        case -1:
          target = !HAL::digitalRead(pin_number);
          break;
      }

      while (HAL::digitalRead(pin_number) != target) idle();

    } // pin_state -1 0 1 && pin_number > -1
  } // parser.seen('P')
}

#if HAS_CHDK || HAS_PHOTOGRAPH
  /**
   * M240: Trigger a camera
   */
  inline void gcode_M240() {
    #if HAS_CHDK
       OUT_WRITE(CHDK_PIN, HIGH);
       chdkHigh = millis();
       chdkActive = true;
    #elif HAS_PHOTOGRAPH
      const uint8_t NUM_PULSES = 16;
      const float PULSE_LENGTH = 0.01524;
      for (int i = 0; i < NUM_PULSES; i++) {
        WRITE(PHOTOGRAPH_PIN, HIGH);
        HAL::delayMilliseconds(PULSE_LENGTH);
        WRITE(PHOTOGRAPH_PIN, LOW);
        HAL::delayMilliseconds(PULSE_LENGTH);
      }
      HAL::delayMilliseconds(7.33);
      for (int i = 0; i < NUM_PULSES; i++) {
        WRITE(PHOTOGRAPH_PIN, HIGH);
        HAL::delayMilliseconds(PULSE_LENGTH);
        WRITE(PHOTOGRAPH_PIN, LOW);
        HAL::delayMilliseconds(PULSE_LENGTH);
      }
    #endif // HASNT(CHDK) && HAS_PHOTOGRAPH
  }
#endif // HAS_CHDK || PHOTOGRAPH_PIN

#if HAS(LCD_CONTRAST)
  /**
   * M250: Read and optionally set the LCD contrast
   */
  inline void gcode_M250() {
    if (parser.seenval('C')) set_lcd_contrast(parser.value_int());
    SERIAL_EMV("lcd contrast value: ", lcd_contrast);
  }

#endif // DOGLCD

#if HAS_SERVOS
  /**
   * M280: Get or set servo position. P<index> S<angle>
   */
  inline void gcode_M280() {
    if (!parser.seen('P')) return;
    const int servo_index = parser.value_int();

    #if HAS_DONDOLO
      int servo_position = 0;
      if (parser.seenval('S')) {
        servo_position = parser.value_int();
        if (servo_index >= 0 && servo_index < NUM_SERVOS && servo_index != DONDOLO_SERVO_INDEX) {
          MOVE_SERVO(servo_index, servo_position);
        }
        else if (servo_index == DONDOLO_SERVO_INDEX) {
          Servo *srv = &servo[servo_index];
          srv->attach(0);
          srv->write(servo_position);
          #if (DONDOLO_SERVO_DELAY > 0)
            safe_delay(DONDOLO_SERVO_DELAY);
            srv->detach();
          #endif
        }
        else {
          SERIAL_SMV(ER, "Servo ", servo_index);
          SERIAL_EM(" out of range");
        }
      }
    #else
      if (WITHIN(servo_index, 0, NUM_SERVOS - 1)) {
        if (parser.seenval('S'))
          MOVE_SERVO(servo_index, parser.value_int());
        else {
          SERIAL_SMV(ECHO, " Servo ", servo_index);
          SERIAL_EMV(": ", servo[servo_index].read());
        }
      }
      else {
        SERIAL_SMV(ER, "Servo ", servo_index);
        SERIAL_EM(" out of range");
      }
    #endif
  }
#endif // NUM_SERVOS > 0

#if HAS_BUZZER
  /**
   * M300: Play beep sound S<frequency Hz> P<duration ms>
   */
  inline void gcode_M300() {
    uint16_t const frequency = parser.seen('S') ? parser.value_ushort() : 260;
    uint16_t duration = parser.seen('P') ? parser.value_ushort() : 1000;

    // Limits the tone duration to 0-5 seconds.
    NOMORE(duration, 5000);

    BUZZ(duration, frequency);
  }
#endif // HAS_BUZZER

#if ENABLED(PIDTEMP)
  /**
   * M301: Set PID parameters P I D (and optionally C, L)
   *
   *   P[float] Kp term
   *   I[float] Ki term (unscaled)
   *   D[float] Kd term (unscaled)
   *
   * With PID_ADD_EXTRUSION_RATE:
   *
   *   C[float] Kc term
   *   L[float] LPQ length
   */
  inline void gcode_M301() {

    // multi-hotend PID patch: M301 updates or prints a single hotend's PID values
    // default behaviour (omitting E parameter) is to update for hotend 0 only
    int h = parser.seen('H') ? parser.value_int() : 0; // hotend being updated

    if (h < HOTENDS) { // catch bad input value
      if (parser.seen('P')) PID_PARAM(Kp, h) = parser.value_float();
      if (parser.seen('I')) PID_PARAM(Ki, h) = parser.value_float();
      if (parser.seen('D')) PID_PARAM(Kd, h) = parser.value_float();
      #if ENABLED(PID_ADD_EXTRUSION_RATE)
        if (parser.seen('C')) PID_PARAM(Kc, h) = parser.value_float();
        if (parser.seen('L')) lpq_len = parser.value_float();
        NOMORE(lpq_len, LPQ_MAX_LEN);
      #endif

      thermalManager.updatePID();
      SERIAL_SMV(ECHO, "H", h);
      SERIAL_MV(" P:", PID_PARAM(Kp, h));
      SERIAL_MV(" I:", PID_PARAM(Ki, h));
      SERIAL_MV(" D:", PID_PARAM(Kd, h));
      #if ENABLED(PID_ADD_EXTRUSION_RATE)
        SERIAL_MV(" C:", PID_PARAM(Kc, h));
      #endif
      SERIAL_EOL();
    }
    else {
      SERIAL_LM(ER, MSG_INVALID_EXTRUDER);
    }
  }
#endif // PIDTEMP

#if HAS_EXTRUDERS && ENABLED(PREVENT_COLD_EXTRUSION)
  /**
   * M302: Allow cold extrudes, or set the minimum extrude temperature
   *
   *       S<temperature> sets the minimum extrude temperature
   *       P<bool> enables (1) or disables (0) cold extrusion
   *
   *  Examples:
   *
   *       M302         ; report current cold extrusion state
   *       M302 P0      ; enable cold extrusion checking
   *       M302 P1      ; disables cold extrusion checking
   *       M302 S0      ; always allow extrusion (disables checking)
   *       M302 S170    ; only allow extrusion above 170
   *       M302 S170 P1 ; set min extrude temp to 170 but leave disabled
   */
  inline void gcode_M302() {
    bool seen_S = parser.seen('S');
    if (seen_S) {
      thermalManager.extrude_min_temp = parser.value_celsius();
      thermalManager.allow_cold_extrude = (thermalManager.extrude_min_temp == 0);
    }

    if (parser.seen('P'))
      thermalManager.allow_cold_extrude = (thermalManager.extrude_min_temp == 0) || parser.value_bool();
    else if (!seen_S) {
      // Report current state
      SERIAL_MV("Cold extrudes are ", (thermalManager.allow_cold_extrude ? "en" : "dis"));
      SERIAL_MV("abled (min temp ", thermalManager.extrude_min_temp);
      SERIAL_EM("C)");
    }
  }
#endif // PREVENT_COLD_EXTRUSION

#if ENABLED(PIDTEMP)

  /**
   * M303: PID relay autotune
   *
   *       S<temperature> sets the target temperature. (default target temperature = 150C)
   *       H<hotend> (-1 for the bed, -2 for chamber, -3 for cooler) (default 0)
   *       C<cycles>
   *       U<bool> with a non-zero value will apply the result to current settings
   */
  inline void gcode_M303() {
    #if HAS_PID_HEATING || HAS_PID_COOLING
      const int   h = parser.seen('H') ? parser.value_int() : 0,
                  c = parser.seen('C') ? parser.value_int() : 5;
      const bool  u = parser.seen('U') && parser.value_bool() != 0;

      int16_t temp = parser.seen('S') ? parser.value_celsius() : (h < 0 ? 70 : 200);

      if (WITHIN(h, 0, HOTENDS - 1)) target_extruder = h;

      KEEPALIVE_STATE(NOT_BUSY); // don't send "busy: processing" messages during autotune output

      thermalManager.PID_autotune(temp, h, c, u);

      KEEPALIVE_STATE(IN_HANDLER);
    #else
      SERIAL_LM(ER, MSG_ERR_M303_DISABLED);
    #endif
  }

#endif

#if ENABLED(PIDTEMPBED)

  // M304: Set bed PID parameters P I and D
  inline void gcode_M304() {
    if (parser.seen('P')) thermalManager.bedKp = parser.value_float();
    if (parser.seen('I')) thermalManager.bedKi = parser.value_float();
    if (parser.seen('D')) thermalManager.bedKd = parser.value_float();

    thermalManager.updatePID();
    SERIAL_SMV(ECHO, " p:", thermalManager.bedKp);
    SERIAL_MV(" i:", thermalManager.bedKi);
    SERIAL_EMV(" d:", thermalManager.bedKd);
  }

#endif // PIDTEMPBED

#if ENABLED(PIDTEMPCHAMBER)

  // M305: Set chamber PID parameters P I and D
  inline void gcode_M305() {
    if (parser.seen('P')) thermalManager.chamberKp = parser.value_float();
    if (parser.seen('I')) thermalManager.chamberKi = parser.value_float();
    if (parser.seen('D')) thermalManager.chamberKd = parser.value_float();

    thermalManager.updatePID();
    SERIAL_SMV(OK, " p:", thermalManager.chamberKp);
    SERIAL_MV(" i:", thermalManager.chamberKi);
    SERIAL_EMV(" d:", thermalManager.chamberKd);
  }

#endif // PIDTEMPCHAMBER

#if ENABLED(PIDTEMPCOOLER)

  // M306: Set cooler PID parameters P I and D
  inline void gcode_M306() {
    if (parser.seen('P')) thermalManager.coolerKp = parser.value_float();
    if (parser.seen('I')) thermalManager.coolerKi = parser.value_float();
    if (parser.seen('D')) thermalManager.coolerKd = parser.value_float();

    thermalManager.updatePID();
    SERIAL_SMV(OK, " p:", thermalManager.coolerKp);
    SERIAL_MV(" i:", thermalManager.coolerKi);
    SERIAL_EMV(" d:", thermalManager.coolerKd);
  }

#endif // PIDTEMPCOOLER

#if HAS_ABL

  /**
   * M320: Enable/Disable Bed Leveling and/or set the Z fade height.
   *
   *       S[bool]   Turns leveling on or off
   *       Z[height] Sets the Z fade height (0 or none to disable)
   *       V[bool]   Verbose - Print the leveling grid
   */
  inline void gcode_M320() {

    // V to print the matrix
    if (parser.seen('V')) {
      #if ABL_PLANAR
        bedlevel.bed_level_matrix.debug("Bed Level Correction Matrix:");
      #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
        if (bedlevel.leveling_is_valid()) {
          bedlevel.print_bilinear_leveling_grid();
          #if ENABLED(ABL_BILINEAR_SUBDIVISION)
            bedlevel.bed_level_virt_print();
          #endif
        }
      #endif
    }

    bool to_enable = false;
    if (parser.seen('S')) {
      to_enable = parser.value_bool();
      bedlevel.set_bed_leveling_enabled(to_enable);
    }

    #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
      if (parser.seen('Z')) {
        bedlevel.set_z_fade_height(parser.value_linear_units());
        SERIAL_LMV(ECHO, "ABL Fade Height = ", parser.value_linear_units(), 2);
      }
    #endif

    const bool new_status = bedlevel.leveling_is_active();
    if (to_enable && !new_status)
      SERIAL_LM(ER, MSG_ERR_M320_M420_FAILED);

    SERIAL_LMV(ECHO, "ABL: ", new_status ? MSG_ON : MSG_OFF);
  }

  #if ENABLED(AUTO_BED_LEVELING_BILINEAR)

    /**
     * M321: Set Level bilinear manual
     *
     * Usage:
     *   M321 I<xindex> J<yindex> Z<linear>
     *   M321 I<xindex> J<yindex> Q<offset>
     */
    inline void gcode_M321() {
      const bool hasI = parser.seen('I');
      const int8_t ix = hasI ? parser.value_int() : -1;
      const bool hasJ = parser.seen('J');
      const int8_t iy = hasJ ? parser.value_int() : -1;
      const bool hasZ = parser.seen('Z'), hasQ = !hasZ && parser.seen('Q');

      if (!hasI || !hasJ || !(hasZ || hasQ)) {
        SERIAL_LM(ER, MSG_ERR_M321_PARAMETERS);
      }
        else if (!WITHIN(ix, 0, GRID_MAX_POINTS_X - 1) || !WITHIN(iy, 0, GRID_MAX_POINTS_Y - 1)) {
        SERIAL_LM(ER, MSG_ERR_MESH_XY);
      }

      if (hasI && hasJ && !(hasZ || hasQ)) {
        SERIAL_MV("Level value in ix", ix);
        SERIAL_MV(" iy", iy);
        SERIAL_EMV(" Z", bedlevel.z_values[ix][iy]);
        return;
      }
      else {
        bedlevel.z_values[ix][iy] = parser.value_linear_units() + (hasQ ? bedlevel.z_values[ix][iy] : 0);
        #if ENABLED(ABL_BILINEAR_SUBDIVISION)
          bedlevel.bed_level_virt_interpolate();
        #endif
      }
    }

  #endif

  // M322: Reset auto leveling matrix
  inline void gcode_M322() {
    bedlevel.reset_bed_level();
    if (parser.seen('S') && parser.value_bool())
      eeprom.Store_Settings();
  }

#endif

#if HAS_MICROSTEPS

  // M350 Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
  inline void gcode_M350() {
    if (parser.seen('S')) for (int i = 0; i <= 4; i++) stepper.microstep_mode(i, parser.value_byte());
    LOOP_XYZE(i) if (parser.seen(axis_codes[i])) stepper.microstep_mode(i, parser.value_byte());
    if (parser.seen('B')) stepper.microstep_mode(4, parser.value_byte());
    stepper.microstep_readings();
  }

  /**
   * M351: Toggle MS1 MS2 pins directly with axis codes X Y Z E B
   *       S# determines MS1 or MS2, X# sets the pin high/low.
   */
  inline void gcode_M351() {
    if (parser.seen('S')) switch(parser.value_byte()) {
      case 1:
        LOOP_XYZE(i) if (parser.seen(axis_codes[i])) stepper.microstep_ms(i, parser.value_byte(), -1);
        if (parser.seen('B')) stepper.microstep_ms(4, parser.value_byte(), -1);
        break;
      case 2:
        LOOP_XYZE(i) if (parser.seen(axis_codes[i])) stepper.microstep_ms(i, -1, parser.value_byte());
        if (parser.seen('B')) stepper.microstep_ms(4, -1, parser.value_byte());
        break;
    }
    stepper.microstep_readings();
  }

#endif // HAS_MICROSTEPS

#if HAS_CASE_LIGHT

  int case_light_brightness;
  bool case_light_on;

  void update_case_light() {
    pinMode(CASE_LIGHT_PIN, OUTPUT);
    uint8_t case_light_bright = (uint8_t)case_light_brightness;
    if (case_light_on) {
      HAL::analogWrite(CASE_LIGHT_PIN, INVERT_CASE_LIGHT ? 255 - case_light_brightness : case_light_brightness );
      WRITE(CASE_LIGHT_PIN, INVERT_CASE_LIGHT ? LOW : HIGH);
    }
    else WRITE(CASE_LIGHT_PIN, INVERT_CASE_LIGHT ? HIGH : LOW);
  }

  /**
   * M355: Turn case light on/off and set brightness
   *
   *   P<byte>  Set case light brightness (PWM pin required - ignored otherwise)
   *
   *   S<bool>  Set case light on/off
   *
   *   When S turns on the light on a PWM pin then the current brightness level is used/restored
   *
   *   M355 P200 S0 turns off the light & sets the brightness level
   *   M355 S1 turns on the light with a brightness of 200 (assuming a PWM pin)
   */
  inline void gcode_M355() {
    uint8_t args = 0;
    if (parser.seen('P')) ++args, case_light_brightness = parser.value_byte();
    if (parser.seen('S')) ++args, case_light_on = parser.value_bool(); 
    if (args) update_case_light();

    // always report case light status
    SERIAL_STR(ECHO);
    if (!case_light_on)
      SERIAL_EM("Case light: off");
    else
      SERIAL_MV("Case light: ", case_light_brightness);
  }

#endif // HAS_CASE_LIGHT

#if MECH(MORGAN_SCARA)

  bool SCARA_move_to_cal(uint8_t delta_a, uint8_t delta_b) {
    if (IsRunning()) {
      forward_kinematics_SCARA(delta_a, delta_b);
      mechanics.destination[X_AXIS] = LOGICAL_X_POSITION(mechanics.cartesian_position[X_AXIS]);
      mechanics.destination[Y_AXIS] = LOGICAL_Y_POSITION(mechanics.cartesian_position[Y_AXIS]);
      mechanics.destination[Z_AXIS] = mechanics.current_position[Z_AXIS];
      mechanics.prepare_move_to_destination();
      return true;
    }
    return false;
  }

  /**
   * M360: SCARA calibration: Move to cal-position ThetaA (0 deg calibration)
   */
  inline bool gcode_M360() {
    SERIAL_LM(ECHO, " Cal: Theta 0");
    return SCARA_move_to_cal(0, 120);
  }

  /**
   * M361: SCARA calibration: Move to cal-position ThetaB (90 deg calibration - steps per degree)
   */
  inline bool gcode_M361() {
    SERIAL_LM(ECHO, " Cal: Theta 90");
    return SCARA_move_to_cal(90, 130);
  }

  /**
   * M362: SCARA calibration: Move to cal-position PsiA (0 deg calibration)
   */
  inline bool gcode_M362() {
    SERIAL_LM(ECHO, " Cal: Psi 0");
    return SCARA_move_to_cal(60, 180);
  }

  /**
   * M363: SCARA calibration: Move to cal-position PsiB (90 deg calibration - steps per degree)
   */
  inline bool gcode_M363() {
    SERIAL_LM(ECHO, " Cal: Psi 90");
    return SCARA_move_to_cal(50, 90);
  }

  /**
   * M364: SCARA calibration: Move to cal-position PsiC (90 deg to Theta calibration position)
   */
  inline bool gcode_M364() {
    SERIAL_LM(ECHO, " Cal: Theta-Psi 90");
    return SCARA_move_to_cal(45, 135);
  }

#endif // MORGAN_SCARA

#if ENABLED(EXT_SOLENOID)

  void enable_solenoid(uint8_t num) {
    switch(num) {
      case 0:
        OUT_WRITE(SOL0_PIN, HIGH);
        break;
        #if HAS(SOLENOID_1) && EXTRUDERS > 1
          case 1:
            OUT_WRITE(SOL1_PIN, HIGH);
            break;
        #endif
        #if HAS(SOLENOID_2) && EXTRUDERS > 2
          case 2:
            OUT_WRITE(SOL2_PIN, HIGH);
            break;
        #endif
        #if HAS(SOLENOID_3) && EXTRUDERS > 3
          case 3:
            OUT_WRITE(SOL3_PIN, HIGH);
            break;
        #endif
        #if HAS(SOLENOID_4) && EXTRUDERS > 4
          case 4:
            OUT_WRITE(SOL4_PIN, HIGH);
            break;
        #endif
      default:
        SERIAL_LM(ER, MSG_INVALID_SOLENOID);
        break;
    }
  }

  void enable_solenoid_on_active_extruder() { enable_solenoid(active_extruder); }

  void disable_all_solenoids() {
    OUT_WRITE(SOL0_PIN, LOW);
    #if HAS(SOLENOID_1) && EXTRUDERS > 1
      OUT_WRITE(SOL1_PIN, LOW);
    #endif
    #if HAS(SOLENOID_2) && EXTRUDERS > 2
      OUT_WRITE(SOL2_PIN, LOW);
    #endif
    #if HAS(SOLENOID_3) && EXTRUDERS > 3
      OUT_WRITE(SOL3_PIN, LOW);
    #endif
    #if HAS(SOLENOID_4) && EXTRUDERS > 4
      OUT_WRITE(SOL4_PIN, LOW);
    #endif
  }

  /**
   * M380: Enable solenoid on the active extruder
   */
  inline void gcode_M380() { enable_solenoid_on_active_extruder(); }

  /**
   * M381: Disable all solenoids
   */
  inline void gcode_M381() { disable_all_solenoids(); }

#endif // EXT_SOLENOID

/**
 * M400: Finish all moves
 */
inline void gcode_M400() { stepper.synchronize(); }

#if HAS_BED_PROBE

  /**
   * M401: Engage Z Servo endstop if available
   */
  inline void gcode_M401() { probe.set_deployed(true); }

  /**
   * M402: Retract Z Servo endstop if enabled
   */
  inline void gcode_M402() { probe.set_deployed(false); }

#endif // (ENABLED(AUTO_BED_LEVELING_FEATURE) && DISABLED(Z_PROBE_SLED) && HAS_Z_SERVO_PROBE)

#if ENABLED(FILAMENT_SENSOR)

  /**
   * M404: Display or set (in current units) the nominal filament width (3mm, 1.75mm ) W<3.0>
   */
  inline void gcode_M404() {
    if (parser.seen('W')) {
      filament_width_nominal = parser.value_linear_units();
    }
    else {
      SERIAL_EMV("Filament dia (nominal mm):", filament_width_nominal);
    }
  }
    
  /**
   * M405: Turn on filament sensor for control
   */
  inline void gcode_M405() {
    // This is technically a linear measurement, but since it's quantized to centimeters and is a different unit than
    // everything else, it uses parser.value_int() instead of parser.value_linear_units().
    if (parser.seen('D')) meas_delay_cm = parser.value_byte();
    NOMORE(meas_delay_cm, MAX_MEASUREMENT_DELAY);

    if (filwidth_delay_index[1] == -1) { // Initialize the ring buffer if not done since startup
      const uint8_t temp_ratio = thermalManager.widthFil_to_size_ratio() - 100; // -100 to scale within a signed byte

      for (uint8_t i = 0; i < COUNT(measurement_delay); ++i)
        measurement_delay[i] = temp_ratio;

      filwidth_delay_index[0] = filwidth_delay_index[1] = 0;
    }

    filament_sensor = true;

    //SERIAL_MV("Filament dia (measured mm):", filament_width_meas);
    //SERIAL_EMV("Extrusion ratio(%):", flow_percentage[active_extruder]);
  }

  /**
   * M406: Turn off filament sensor for control
   */
  inline void gcode_M406() { filament_sensor = false; }
  
  /**
   * M407: Get measured filament diameter on serial output
   */
  inline void gcode_M407() {
    SERIAL_EMV("Filament dia (measured mm):", filament_width_meas);
  }

#endif // FILAMENT_SENSOR

#if ENABLED(JSON_OUTPUT)
  /**
   * M408: JSON STATUS OUTPUT
   */
  inline void gcode_M408() {
    bool firstOccurrence;
    uint8_t type = 0;

    if (parser.seen('S')) type = parser.value_byte();

    SERIAL_MSG("{\"status\":\"");
    #if HAS_SDSUPPORT
      if (!print_job_counter.isRunning() && !card.sdprinting) SERIAL_CHR('I'); // IDLING
      else if (card.sdprinting) SERIAL_CHR('P');          // SD PRINTING
      else SERIAL_MSG("B");                               // SOMETHING ELSE, BUT SOMETHIG
    #else
      if (!print_job_counter.isRunning()) SERIAL_CHR('I');                     // IDLING
      else SERIAL_CHR('B');                               // SOMETHING ELSE, BUT SOMETHIG
    #endif

    SERIAL_MSG("\",\"coords\": {");
    SERIAL_MSG("\"axesHomed\":[");
    if (mechanics.axis_homed[X_AXIS] && mechanics.axis_homed[Y_AXIS] && mechanics.axis_homed[Z_AXIS])
      SERIAL_MSG("1, 1, 1");
    else
      SERIAL_MSG("0, 0, 0");

    SERIAL_MV("],\"extr\":[", mechanics.current_position[E_AXIS]);
    SERIAL_MV("],\"xyz\":[", mechanics.current_position[X_AXIS]); // X AXIS
    SERIAL_MV(",", mechanics.current_position[Y_AXIS]);           // Y AXIS
    SERIAL_MV(",", mechanics.current_position[Z_AXIS]);           // Z AXIS

    SERIAL_MV("]},\"currentTool\":", active_extruder);

    #if HAS_POWER_SWITCH
      SERIAL_MSG(",\"params\": {\"atxPower\":");
      SERIAL_CHR(powerManager.powersupply_on ? '1' : '0');
    #else
      SERIAL_MSG(",\"params\": {\"NormPower\":");
    #endif

    SERIAL_MSG(",\"fanPercent\":[");
    SERIAL_VAL(fanSpeeds[0]);

    SERIAL_MV("],\"speedFactor\":", mechanics.feedrate_percentage);

    SERIAL_MSG(",\"extrFactors\":[");
    firstOccurrence = true;
    for (uint8_t i = 0; i < EXTRUDERS; i++) {
      if (!firstOccurrence) SERIAL_CHR(',');
      SERIAL_VAL(flow_percentage[i]); // Really *100? 100 is normal
      firstOccurrence = false;
    }
    SERIAL_EM("]},");

    SERIAL_MSG("\"temps\": {");
    #if HAS_TEMP_BED
      SERIAL_MV("\"bed\": {\"current\":", thermalManager.degBed(), 1);
      SERIAL_MV(",\"active\":", thermalManager.degTargetBed());
      SERIAL_MSG(",\"state\":");
      SERIAL_CHR(thermalManager.degTargetBed() > 0 ? '2' : '1');
      SERIAL_MSG("},");
    #endif
    SERIAL_MSG("\"heads\": {\"current\":[");
    firstOccurrence = true;
    for (int8_t h = 0; h < HOTENDS; h++) {
      if (!firstOccurrence) SERIAL_CHR(',');
      SERIAL_VAL(thermalManager.degHotend(h), 1);
      firstOccurrence = false;
    }
    SERIAL_MSG("],\"active\":[");
    firstOccurrence = true;
    for (int8_t h = 0; h < HOTENDS; h++) {
      if (!firstOccurrence) SERIAL_CHR(',');
      SERIAL_VAL(thermalManager.degTargetHotend(h));
      firstOccurrence = false;
    }
    SERIAL_MSG("],\"state\":[");
    firstOccurrence = true;
    for (int8_t h = 0; h < HOTENDS; h++) {
      if (!firstOccurrence) SERIAL_CHR(',');
      SERIAL_CHR(thermalManager.degTargetHotend(h) > HOTEND_AUTO_FAN_TEMPERATURE ? '2' : '1');
      firstOccurrence = false;
    }

    SERIAL_MV("]}},\"time\":", HAL::timeInMilliseconds());

    switch (type) {
      case 0:
      case 1:
        break;
      case 2:
        SERIAL_EM(",");
        SERIAL_MSG("\"coldExtrudeTemp\":0,\"coldRetractTemp\":0.0,\"geometry\":\"");
        #if MECH(CARTESIAN)
          SERIAL_MSG("cartesian");
        #elif MECH(COREXY)
          SERIAL_MSG("corexy");
        #elif MECH(COREYX)
          SERIAL_MSG("coreyx");
        #elif MECH(COREXZ)
          SERIAL_MSG("corexz");
        #elif MECH(COREZX)
          SERIAL_MSG("corezx");
        #elif MECH(DELTA)
          SERIAL_MSG("delta");
        #endif
        SERIAL_MSG("\",\"name\":\"");
        SERIAL_MSG(CUSTOM_MACHINE_NAME);
        SERIAL_MSG("\",\"tools\":[");
        firstOccurrence = true;
        for (uint8_t i = 0; i < EXTRUDERS; i++) {
          if (!firstOccurrence) SERIAL_CHR(',');
          SERIAL_MV("{\"number\":", i + 1);
          #if HOTENDS > 1
            SERIAL_MV(",\"heaters\":[", i + 1);
            SERIAL_MSG("],");
          #else
            SERIAL_MSG(",\"heaters\":[1],");
          #endif
          #if DRIVER_EXTRUDERS > 1
            SERIAL_MV("\"drives\":[", i);
            SERIAL_MSG("]");
          #else
            SERIAL_MSG("\"drives\":[0]");
          #endif
          SERIAL_MSG("}");
          firstOccurrence = false;
        }
        break;
      case 3:
        SERIAL_EM(",");
        SERIAL_MSG("\"currentLayer\":");
        #if HAS_SDSUPPORT
          if (card.sdprinting && card.layerHeight > 0) { // ONLY CAN TELL WHEN SD IS PRINTING
            SERIAL_VAL((int) (mechanics.current_position[Z_AXIS] / card.layerHeight));
          }
          else SERIAL_VAL(0);
        #else
          SERIAL_VAL(-1);
        #endif
        SERIAL_MSG(",\"extrRaw\":[");
        firstOccurrence = true;
        for (uint8_t i = 0; i < EXTRUDERS; i++) {
          if (!firstOccurrence) SERIAL_CHR(',');
          SERIAL_VAL(mechanics.current_position[E_AXIS] * flow_percentage[i]);
          firstOccurrence = false;
        }
        SERIAL_MSG("],");
        #if HAS_SDSUPPORT
          if (card.sdprinting) {
            SERIAL_MSG("\"fractionPrinted\":");
            float fractionprinted;
            if (card.fileSize < 2000000) {
              fractionprinted = (float)card.sdpos / (float)card.fileSize;
            }
            else fractionprinted = (float)(card.sdpos >> 8) / (float)(card.fileSize >> 8);
            SERIAL_VAL((float) floorf(fractionprinted * 1000) / 1000);
            SERIAL_CHR(',');
          }
        #endif
        SERIAL_MSG("\"firstLayerHeight\":");
        #if HAS_SDSUPPORT
          if (card.sdprinting) SERIAL_VAL(card.firstlayerHeight);
          else SERIAL_MSG("0");
        #else
          SERIAL_MSG("0");
        #endif
        break;
      case 4:
      case 5:
        SERIAL_EM(",");
        SERIAL_MSG("\"axisMins\":[");
        SERIAL_VAL((int) X_MIN_POS);
        SERIAL_CHR(',');
        SERIAL_VAL((int) Y_MIN_POS);
        SERIAL_CHR(',');
        SERIAL_VAL((int) Z_MIN_POS);
        SERIAL_MSG("],\"axisMaxes\":[");
        SERIAL_VAL((int) X_MAX_POS);
        SERIAL_CHR(',');
        SERIAL_VAL((int) Y_MAX_POS);
        SERIAL_CHR(',');
        SERIAL_VAL((int) Z_MAX_POS);
        SERIAL_MSG("],\"planner.accelerations\":[");
        SERIAL_VAL(mechanics.max_acceleration_mm_per_s2[X_AXIS]);
        SERIAL_CHR(',');
        SERIAL_VAL(mechanics.max_acceleration_mm_per_s2[Y_AXIS]);
        SERIAL_CHR(',');
        SERIAL_VAL(mechanics.max_acceleration_mm_per_s2[Z_AXIS]);
        for (uint8_t i = 0; i < EXTRUDERS; i++) {
          SERIAL_CHR(',');
          SERIAL_VAL(mechanics.max_acceleration_mm_per_s2[E_AXIS + i]);
        }
        SERIAL_MSG("],");

        #if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
          SERIAL_MSG("\"currents\":[");
          SERIAL_VAL(motor_current[X_AXIS]);
          SERIAL_CHR(',');
          SERIAL_VAL(motor_current[Y_AXIS]);
          SERIAL_CHR(',');
          SERIAL_VAL(motor_current[Z_AXIS]);
          for (uint8_t i = 0; i < DRIVER_EXTRUDERS; i++) {
            SERIAL_CHR(',');
            SERIAL_VAL(motor_current[E_AXIS + i]);
          }
          SERIAL_EM("],");
        #endif

        SERIAL_MSG("\"firmwareElectronics\":\"");
        #if MB(RAMPS_13_HFB) || MB(RAMPS_13_HHB) || MB(RAMPS_13_HFF) || MB(RAMPS_13_HHF) || MB(RAMPS_13_HHH)
          SERIAL_MSG("RAMPS");
        #elif MB(ALLIGATOR)
          SERIAL_MSG("ALLIGATOR");
        #elif MB(ALLIGATOR_V3)
          SERIAL_MSG("ALLIGATOR_V3");
        #elif MB(RADDS) || MB(RAMPS_FD_V1) || MB(RAMPS_FD_V2) || MB(SMART_RAMPS) || MB(RAMPS4DUE)
          SERIAL_MSG("Arduino due");
        #elif MB(ULTRATRONICS)
          SERIAL_MSG("ULTRATRONICS");
        #else
          SERIAL_MSG("AVR");
        #endif
        SERIAL_MSG("\",\"firmwareName\":\"");
        SERIAL_MSG(FIRMWARE_NAME);
        SERIAL_MSG(",\"firmwareVersion\":\"");
        SERIAL_MSG(SHORT_BUILD_VERSION);
        SERIAL_MSG("\",\"firmwareDate\":\"");
        SERIAL_MSG(STRING_DISTRIBUTION_DATE);

        SERIAL_MSG("\",\"minFeedrates\":[0,0,0");
        for (uint8_t i = 0; i < EXTRUDERS; i++) {
          SERIAL_MSG(",0");
        }
        SERIAL_MSG("],\"maxFeedrates\":[");
        SERIAL_VAL(mechanics.max_feedrate_mm_s[X_AXIS]);
        SERIAL_CHR(',');
        SERIAL_VAL(mechanics.max_feedrate_mm_s[Y_AXIS]);
        SERIAL_CHR(',');
        SERIAL_VAL(mechanics.max_feedrate_mm_s[Z_AXIS]);
        for (uint8_t i = 0; i < EXTRUDERS; i++) {
          SERIAL_CHR(',');
          SERIAL_VAL(mechanics.max_feedrate_mm_s[E_AXIS + i]);
        }
        SERIAL_CHR(']');
        break;
    }
    SERIAL_CHR('}');
    SERIAL_EOL();
  }
#endif // JSON_OUTPUT

#if DISABLED(EMERGENCY_PARSER)
  /**
   * M410: Quickstop - Abort all planned moves
   *
   * This will stop the carriages mid-move, so most likely they
   * will be out of sync with the stepper position after this.
   */
  inline void gcode_M410() { quickstop_stepper(); }
#endif

#if ENABLED(MESH_BED_LEVELING)
  /**
   * M420: Enable/Disable Bed Leveling and/or set the Z fade height.
   *
   *    S[bool]   Turns leveling on or off
   *    Z[height] Sets the Z fade height (0 or none to disable)
   *    V[bool]   Verbose - Print the leveling grid
   */
  inline void gcode_M420() {
    bool to_enable = false;

    // V to print the matrix or mesh
    if (parser.seen('V') && bedlevel.leveling_is_valid()) {
      SERIAL_EM("Mesh Bed Level data:");
      bedlevel.mbl_mesh_report();
    }

    if (parser.seen('S')) {
      to_enable = parser.value_bool();
      bedlevel.set_bed_leveling_enabled(to_enable);
    }

    #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT)
      if (parser.seen('Z')) bedlevel.set_z_fade_height(parser.value_linear_units());
    #endif

    const bool new_status = bedlevel.leveling_is_active();

    if (to_enable && !new_status)
      SERIAL_LM(ER, MSG_ERR_M320_M420_FAILED);

    SERIAL_LMV(ECHO, "MBL: ", new_status ? MSG_ON : MSG_OFF);
  }

  /**
   * M421: Set a single Mesh Bed Leveling Z coordinate
   *
   * Usage:
   *   M421 X<linear> Y<linear> Z<linear>
   *   M421 X<linear> Y<linear> Q<offset>
   *   M421 I<xindex> J<yindex> Z<linear>
   *   M421 I<xindex> J<yindex> Q<offset>
   */
  inline void gcode_M421() {
    const bool hasX = parser.seen('X'), hasI = parser.seen('I');
    const int8_t ix = hasI ? parser.value_int() : hasX ? mbl.probe_index_x(RAW_X_POSITION(parser.value_linear_units())) : -1;
    const bool hasY = parser.seen('Y'), hasJ = parser.seen('J');
    const int8_t iy = hasJ ? parser.value_int() : hasY ? mbl.probe_index_y(RAW_Y_POSITION(parser.value_linear_units())) : -1;
    const bool hasZ = parser.seen('Z'), hasQ = !hasZ && parser.seen('Q');

    if (int(hasI && hasJ) + int(hasX && hasY) != 1 || !(hasZ || hasQ)) {
      SERIAL_LM(ER, MSG_ERR_M421_PARAMETERS);
    }
    else if (ix < 0 || iy < 0) {
      SERIAL_LM(ER, MSG_ERR_MESH_XY);
    }
    else
      mbl.set_z(ix, iy, parser.value_linear_units() + (hasQ ? mbl.z_values[ix][iy] : 0));
  }
#endif // MESH_BED_LEVELING

#if ENABLED(WORKSPACE_OFFSETS)

  /**
   * M428: Set home_offset based on the distance between the
   *       current_position and the nearest "reference point."
   *       If an axis is past center its Endstop position
   *       is the reference-point. Otherwise it uses 0. This allows
   *       the Z offset to be set near the bed when using a max Endstop.
   *
   *       M428 can't be used more than 2cm away from 0 or an Endstop.
   *
   *       Use M206 to set these values directly.
   */
  inline void gcode_M428() {
    bool err = false;
    LOOP_XYZ(i) {
      if (mechanics.axis_homed[i]) {
        const float base = (mechanics.current_position[i] > (endstops.soft_endstop_min[i] + endstops.soft_endstop_max[i]) * 0.5) ? mechanics.base_home_pos[(AxisEnum)i] : 0,
                    diff = base - RAW_POSITION(mechanics.current_position[i], i);
        if (WITHIN(diff, -20, 20)) {
          set_home_offset((AxisEnum)i, diff);
        }
        else {
          SERIAL_LM(ER, MSG_ERR_M428_TOO_FAR);
          LCD_ALERTMESSAGEPGM("Err: Too far!");
          BUZZ(200, 40);
          err = true;
          break;
        }
      }
    }

    if (!err) {
      mechanics.sync_plan_position();
      mechanics.report_current_position();
      LCD_MESSAGEPGM(MSG_HOME_OFFSETS_APPLIED);
      BUZZ(100, 659);
      BUZZ(100, 698);
    }
  }

#endif // ENABLED(WORKSPACE_OFFSETS)

#if HAS_MULTI_MODE

  /**
   * Shared function for Printer Mode GCodes
   */
  static void gcode_printer_mode(const int8_t new_mode) {
    const static char str_tooltype_0[] PROGMEM = "FFF";
    const static char str_tooltype_1[] PROGMEM = "Laser";
    const static char str_tooltype_2[] PROGMEM = "CNC";
    const static char* const tool_strings[] PROGMEM = { str_tooltype_0, str_tooltype_1, str_tooltype_2 };
    if (new_mode >= 0 && (PrinterMode)new_mode < PRINTER_MODE_COUNT) printer_mode = (PrinterMode)new_mode;
    SERIAL_SM(ECHO, "Printer-Mode: ");
    SERIAL_PS((char*)pgm_read_word(&(tool_strings[printer_mode])));
    SERIAL_CHR(' ');
    SERIAL_EV((int)(printer_mode == PRINTER_MODE_FFF ? active_extruder : 0));
  }

  /**
   * M450: Set and/or report current tool type
   *
   *  S<type> - The new tool type
   */
  inline void gcode_M450() {
    gcode_printer_mode(parser.seen('S') ? parser.value_byte() : -1);
  }

  /**
   * M451: Select FFF printer mode
   */
  inline void gcode_M451() { gcode_printer_mode(PRINTER_MODE_FFF); }

  #if ENABLED(LASER)
    /**
     * M452: Select Laser printer mode
     */
    inline void gcode_M452() { gcode_printer_mode(PRINTER_MODE_LASER); }
  #endif

  #if HAS_CNCROUTER
    /**
     * M453: Select CNC printer mode
     */
    inline void gcode_M453() { gcode_printer_mode(PRINTER_MODE_CNC); }
  #endif

#endif // HAS_MULTI_MODE

/**
 * M500: Store settings in EEPROM
 */
inline void gcode_M500() {
  (void)eeprom.Store_Settings();
}

/**
 * M501: Read settings from EEPROM
 */
inline void gcode_M501() {
  (void)eeprom.Load_Settings();
}

/**
 * M502: Revert to default settings
 */
inline void gcode_M502() {
  (void)eeprom.Factory_Settings();
}

/**
 * M503: print settings currently in memory
 */
inline void gcode_M503() {
  (void)eeprom.Print_Settings(parser.seen('S') && !parser.value_bool());
}

#if ENABLED(RFID_MODULE)
  /**
   * M522: Read or Write on card. M522 T<extruders> R<read> or W<write> L<list>
   */
  inline void gcode_M522() {

    GET_TARGET_EXTRUDER(522);
    if (!RFID_ON) return;

    if (parser.seen('R')) {
      SERIAL_EM("Put RFID on tag!");
      #if ENABLED(NEXTION)
        rfid_setText("Put RFID on tag!");
      #endif
      Spool_must_read[TARGET_EXTRUDER] = true;
    }
    if (parser.seen('W')) {
      if (Spool_ID[TARGET_EXTRUDER] != 0) {
        SERIAL_EM("Put RFID on tag!");
        #if ENABLED(NEXTION)
          rfid_setText("Put RFID on tag!");
        #endif
        Spool_must_write[TARGET_EXTRUDER] = true;
      }
      else {
        SERIAL_LM(ER, "You have not read this Spool!");
        #if ENABLED(NEXTION)
          rfid_setText("You have not read this Spool!", 64488);
        #endif
      }
    }

    if (parser.seen('L')) RFID522.printInfo(TARGET_EXTRUDER);
  }
#endif // RFID_MODULE

/**
 * M530: S<printing> L<layer> - Enables explicit printing mode (S1) or disables it (S0). L can set layer count
 */
inline void gcode_M530() {

  if (parser.seen('L')) maxLayer = parser.value_long();
  
  if (parser.seen('S') && parser.value_bool()) {
    print_job_counter.start();

    SERIAL_MSG("Start Printing");
    if (maxLayer > 0) SERIAL_EMV(" - MaxLayer:", maxLayer);
    else SERIAL_EOL();

    #if ENABLED(START_GCODE)
      enqueue_and_echo_commands_P(PSTR(START_PRINTING_SCRIPT));
    #endif
    #if HAS_FIL_RUNOUT
      filament_ran_out = false;
      SERIAL_EM("Filament runout activated.");
      SERIAL_STR(RESUME);
      SERIAL_EOL();
    #endif
    #if HAS_POWER_CONSUMPTION_SENSOR
      startpower = power_consumption_hour;
    #endif
  }
  else {
    print_job_counter.stop();
    SERIAL_EM("Stop Printing");
    #if ENABLED(STOP_GCODE)
      enqueue_and_echo_commands_P(PSTR(STOP_PRINTING_SCRIPT));
    #endif
    #if HAS_FIL_RUNOUT
      filament_ran_out = false;
      SERIAL_EM("Filament runout deactivated.");
    #endif
  }
}

/**
 * M531: filename - Define filename being printed
 */
inline void gcode_M531() {
  strncpy(printName, parser.string_arg, 20);
  printName[20] = 0;
}

/**
 * M532: X<percent> L<curLayer> - update current print state progress (X=0..100) and layer L
 */
inline void gcode_M532() {
  if (parser.seen('X'))
    progress = parser.value_float();
  if (progress > 100.0)
    progress = 100.0;
  else if (progress < 0)
    progress = 0;

  if (parser.seen('L'))
    currentLayer = parser.value_long();
}

#if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)

  /**
   * M540: Set whether SD card print should abort on endstop hit (M540 S<0|1>)
   */
  inline void gcode_M540() {
    if (parser.seen('S')) stepper.abort_on_endstop_hit = parser.value_bool();
  }

#endif // ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED

#if HEATER_USES_AD595
  /**
   * M595 - set Hotend AD595 offset & Gain H<hotend_number> O<offset> S<gain>
   */
  inline void gcode_M595() {

    GET_TARGET_HOTEND(595);

    if (parser.seen('O')) ad595_offset[TARGET_EXTRUDER] = parser.value_float();
    if (parser.seen('S')) ad595_gain[TARGET_EXTRUDER] = parser.value_float();

    for (int8_t h = 0; h < HOTENDS; h++) {
      // if gain == 0 you get MINTEMP!
      if (ad595_gain[h] == 0) ad595_gain[h]= 1;
    }

    SERIAL_EM(MSG_AD595);
    for (int8_t h = 0; h < HOTENDS; h++) {
      SERIAL_MV(" T", h);
      SERIAL_MV(" Offset: ", ad595_offset[h]);
      SERIAL_EMV(", Gain: ", ad595_gain[h]);
    }
  }
#endif // HEATER_USES_AD595

#if ENABLED(ADVANCED_PAUSE_FEATURE)

  /**
   * M600: Pause Park and filament change
   *
   *  E[distance] - Retract the filament this far (negative value)
   *  Z[distance] - Move the Z axis by this distance
   *  X[position] - Move to this X position, with Y
   *  Y[position] - Move to this Y position, with X
   *  U[distance] - Retract distance for removal (negative value) (manual reload)
   *  L[distance] - Extrude distance for insertion (positive value) (manual reload)
   *  B[count]    - Number of times to beep, -1 for indefinite (if equipped with a buzzer)
   *
   *  Default values are used for omitted arguments.
   *
   */
  inline void gcode_M600() {

    // Homing first
    if (mechanics.axis_unhomed_error()) home_all_axes();

    // Initial retract before move to pause park position
    const float retract = parser.seen('E') ? parser.value_axis_units(E_AXIS) : 0
      #if ENABLED(PAUSE_PARK_RETRACT_LENGTH) && PAUSE_PARK_RETRACT_LENGTH > 0
        - (PAUSE_PARK_RETRACT_LENGTH)
      #endif
    ;

    // Second retract after cooldown hotend
    const float retract2 = 0
      #if ENABLED(PAUSE_PARK_RETRACT_2_LENGTH) && PAUSE_PARK_RETRACT_2_LENGTH > 0
        - (PAUSE_PARK_RETRACT_2_LENGTH)
      #endif
    ;

    // Lift Z axis
    const float z_lift = parser.seen('Z') ? parser.value_linear_units() :
      #if ENABLED(PAUSE_PARK_Z_ADD) && PAUSE_PARK_Z_ADD > 0
        PAUSE_PARK_Z_ADD
      #else
        0
      #endif
    ;

    // Move XY axes to filament exchange position
    const float x_pos = parser.seen('X') ? parser.value_linear_units() : 0
      #if ENABLED(PAUSE_PARK_X_POS)
        + PAUSE_PARK_X_POS
      #endif
    ;
    const float y_pos = parser.seen('Y') ? parser.value_linear_units() : 0
      #if ENABLED(PAUSE_PARK_Y_POS)
        + PAUSE_PARK_Y_POS
      #endif
    ;

    // Unload filament
    const float unload_length = parser.seen('U') ? parser.value_axis_units(E_AXIS) : 0
      #if ENABLED(PAUSE_PARK_UNLOAD_LENGTH) && PAUSE_PARK_UNLOAD_LENGTH > 0
        - (PAUSE_PARK_UNLOAD_LENGTH)
      #endif
    ;

    // Load filament
    const float load_length = parser.seen('L') ? parser.value_axis_units(E_AXIS) : 0
      #if ENABLED(PAUSE_PARK_LOAD_LENGTH)
        + PAUSE_PARK_LOAD_LENGTH
      #endif
    ;

    const int beep_count = parser.seen('B') ? parser.value_int() :
      #if ENABLED(PAUSE_PARK_NUMBER_OF_ALERT_BEEPS)
        PAUSE_PARK_NUMBER_OF_ALERT_BEEPS
      #else
        -1
      #endif
    ;

    const bool job_running = print_job_counter.isRunning();

    if (pause_print(retract, retract2, z_lift, x_pos, y_pos, unload_length, beep_count, true)) {
      wait_for_filament_reload(beep_count);
      resume_print(load_length, PAUSE_PARK_EXTRUDE_LENGTH, beep_count);
    }

    // Resume the print job timer if it was running
    if (job_running) print_job_counter.start();

  }

#endif // ADVANCED_PAUSE_FEATURE

#if ENABLED(DUAL_X_CARRIAGE)

  /**
   * M605: Set dual x-carriage movement mode
   *
   *    M605 S0: Full control mode. The slicer has full control over x-carriage movement
   *    M605 S1: Auto-park mode. The inactive head will auto park/unpark without slicer involvement
   *    M605 S2 [Xnnn] [Rmmm]: Duplication mode. The second extruder will duplicate the first with nnn
   *                         units x-offset and an optional differential hotend temperature of
   *                         mmm degrees. E.g., with "M605 S2 X100 R2" the second extruder will duplicate
   *                         the first with a spacing of 100mm in the x direction and 2 degrees hotter.
   *
   *    Note: the X axis should be homed after changing dual x-carriage mode.
   */
  inline void gcode_M605() {
    stepper.synchronize();
    if (parser.seen('S')) mechanics.dual_x_carriage_mode = (DualXMode)parser.value_byte();
    switch(mechanics.dual_x_carriage_mode) {
      case DXC_FULL_CONTROL_MODE:
      case DXC_AUTO_PARK_MODE:
        break;
      case DXC_DUPLICATION_MODE:
        if (parser.seen('X')) mechanics.duplicate_hotend_x_offset = max(parser.value_linear_units(), X2_MIN_POS - mechanics.x_home_pos(0));
        if (parser.seen('R')) mechanics.duplicate_hotend_temp_offset = parser.value_celsius_diff();
        SERIAL_SM(ECHO, MSG_HOTEND_OFFSET);
        SERIAL_CHR(' ');
        SERIAL_VAL(hotend_offset[X_AXIS][0]);
        SERIAL_CHR(',');
        SERIAL_VAL(hotend_offset[Y_AXIS][0]);
        SERIAL_CHR(' ');
        SERIAL_VAL(mechanics.duplicate_hotend_x_offset);
        SERIAL_CHR(',');
        SERIAL_EV(hotend_offset[Y_AXIS][1]);
        break;
      default:
        mechanics.dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;
        break;
    }
    mechanics.active_hotend_parked = false;
    mechanics.hotend_duplication_enabled = false;
    mechanics.delayed_move_time = 0;
  }

#endif // DUAL_X_CARRIAGE

#if ENABLED(LASER)

  // M649 set laser options
  inline void gcode_M649() {
    // do this at the start so we can debug if needed!
    if (parser.seen('D') && IsRunning()) laser.diagnostics = parser.value_bool();

    // Wait for the rest 
    // stepper.synchronize();
    if (parser.seen('S') && IsRunning()) {
      laser.intensity = parser.value_float();
      laser.rasterlaserpower =  laser.intensity;
    }

    if (IsRunning()) {
      if (parser.seen('L')) laser.duration = parser.value_ulong();
      if (parser.seen('P')) laser.ppm = parser.value_float();
      if (parser.seen('B')) laser.set_mode(parser.value_int());
      if (parser.seen('R')) laser.raster_mm_per_pulse = (parser.value_float());
    }

    if (parser.seen('F')) {
      float next_feedrate = parser.value_linear_units();
      if (next_feedrate > 0.0) mechanics.feedrate_mm_s = next_feedrate;
    }
  }

#endif // LASER

#if MECH(MUVE3D)
  
  // M650: Set peel distance
  inline void gcode_M650() {

    stepper.synchronize();

    peel_distance   = (parser.seen('D') ? parser.value_float() : 2.0);
    peel_speed      = (parser.seen('S') ? parser.value_float() : 2.0);
    retract_speed   = (parser.seen('R') ? parser.value_float() : 2.0);
    peel_pause      = (parser.seen('P') ? parser.value_float() : 0.0);
    tilt_distance   = (parser.seen('T') ? parser.value_float() : 20.0);
    layer_thickness = (parser.seen('H') ? parser.value_float() : 0.0);

    // Initialize tilted to false. The intent here is that you would send this command at the start of a print job, and
    // the platform would be level when you do. As such, we assume that you either hand-cranked it to level, or executed 
    // an M654 command via manual GCode before running a new print job. If not, then the platform is currently tilted, and
    // your print job is going to go poorly.
    tilted = false;
  }

  // M651: Run peel move and return back to start.
  inline void gcode_M651() {

    if (peel_distance > 0) {
      planner.buffer_line(mechanics.destination[X_AXIS], mechanics.destination[Y_AXIS], mechanics.destination[Z_AXIS] + peel_distance, mechanics.destination[Z_AXIS], peel_speed, active_extruder);
      planner.buffer_line(mechanics.destination[X_AXIS], mechanics.destination[Y_AXIS], mechanics.destination[Z_AXIS] + peel_distance, mechanics.destination[Z_AXIS] + peel_distance, peel_speed, active_extruder);
      stepper.synchronize();
      if (peel_pause > 0) safe_delay(peel_pause);
    }

    planner.buffer_line(mechanics.destination[X_AXIS], mechanics.destination[Y_AXIS], mechanics.destination[Z_AXIS], mechanics.destination[Z_AXIS], retract_speed, active_extruder);
    stepper.synchronize();
  }

  // M653: Execute tilt move
  inline void gcode_M653() {
    // Double tilts are not allowed.
    if (!tilted) {
      planner.buffer_line(mechanics.destination[X_AXIS], mechanics.destination[Y_AXIS], mechanics.destination[Z_AXIS] + tilt_distance, mechanics.destination[Z_AXIS], retract_speed, active_extruder);
      stepper.synchronize();
    }
  }

  // M654 - execute untilt move
  inline void gcode_M654() {
    // Can only untilt if tilted
    if (tilted) {
       // To prevent subsequent commands from not knowing our
       // actual position, update the Z axis, then move to it.
       mechanics.destination[Z_AXIS] += tilt_distance;
       planner.buffer_line(mechanics.destination[X_AXIS], mechanics.destination[Y_AXIS], mechanics.destination[Z_AXIS], mechanics.destination[Z_AXIS], retract_speed, active_extruder);
       // And save it away as our current position, because we're there.
       mechanics.set_current_to_destination();
       stepper.synchronize();
       tilted = false;
    }
  }

  // M655: Send projector control commands via serial
  inline void gcode_M655() {

    // Viewsonic commands
    if (parser.seen('V')) {
      int tempVal = parser.value_int();

      switch(tempVal) {
        // Power Off
        case 0: {
          // 0614000400341101005E
          const byte off[] = {0x06, 0x14, 0x00, 0x04, 0x00, 
                              0x34, 0x11, 0x01, 0x00, 0x5E};
          DLPSerial.write(off, sizeof(off));
        }
        break;
        // Power On
        case 1: {
          // 0614000400341100005D
          const byte on[] = {0x06, 0x14, 0x00, 0x04, 0x00,
                             0x34, 0x11, 0x00, 0x00, 0x5D};
          DLPSerial.write(on, sizeof(on));
        }
        break;
        // Factory Reset
        case 2: {
          // 0614000400341102005F
          const byte reset[] = {0x06, 0x14, 0x00, 0x04, 0x00,
                                0x34, 0x11, 0x02, 0x00, 0x5F};
          DLPSerial.write(reset, sizeof(reset));
        }
        break;
        // Splash Screen Black
        case 3: {
          // 061400040034110A0067
          const byte blackScreen[] = {0x06, 0x14, 0x00, 0x04, 0x00,
                                      0x34, 0x11, 0x0A, 0x00, 0x67};
          DLPSerial.write(blackScreen, sizeof(blackScreen));
        }
        break;
        // High Altitude On
        case 4: {
          // 061400040034110C016A
          const byte HAOn[] = {0x06, 0x14, 0x00, 0x04, 0x00,
                               0x34, 0x11, 0x0C, 0x01, 0x6A};
          DLPSerial.write(HAOn, sizeof(HAOn));
        }
        break;
        // High Altitude Off
        case 5: {
          // 061400040034110C0069
          const byte HAOff[] = {0x06, 0x14, 0x00, 0x04, 0x00,
                                0x34, 0x11, 0x0C, 0x00, 0x69};
          DLPSerial.write(HAOff, sizeof(HAOff));
        }
        break;
        // Lamp Mode Normal
        case 6: {
          // 0614000400341110006D
          const byte lampNormal[] = {0x06, 0x14, 0x00, 0x04, 0x00,
                                     0x34, 0x11, 0x10, 0x00, 0x6D};
          DLPSerial.write(lampNormal, sizeof(lampNormal));
        }
        break;
        // Contrast Decrease
        case 7: {
          // 06140004003412020060
          const byte contDec[] = {0x06, 0x14, 0x00, 0x04, 0x00,
                                  0x34, 0x12, 0x02, 0x00, 0x60};
          DLPSerial.write(contDec, sizeof(contDec));
        }
        break;
        // Contrast Increase
        case 8: {
          // 06140004003412020161
          const byte contInc[] = {0x06, 0x14, 0x00, 0x04, 0x00,
                                  0x34, 0x12, 0x02, 0x01, 0x61};
          DLPSerial.write(contInc, sizeof(contInc));
        }
        break;
        // Brightness Decrease
        case 9: {
          // 06140004003412030061
          const byte brightDec[] = {0x06, 0x14, 0x00, 0x04, 0x00,
                                    0x34, 0x12, 0x03, 0x00, 0x61};
          DLPSerial.write(brightDec, sizeof(brightDec));
        }
        break;
        // Brightness Increase
        case 10: {
          // 06140004003412030162
          const byte brightInc[] = {0x06, 0x14, 0x00, 0x04, 0x00,
                                    0x34, 0x12, 0x03, 0x01, 0x62};
          DLPSerial.write(brightInc, sizeof(brightInc));
        }
        break;
      }
    }
  }

#endif // MECH(MUVE3D)

#if HAS_BED_PROBE && NOMECH(DELTA)

  // M666: Set Z probe offset
  inline void gcode_M666() {

    SERIAL_SM(ECHO, MSG_ZPROBE_ZOFFSET);
    SERIAL_CHR(' ');

    if (parser.seen('P')) {
      float p_val = parser.value_linear_units();
      if (Z_PROBE_OFFSET_RANGE_MIN <= p_val && p_val <= Z_PROBE_OFFSET_RANGE_MAX) {

        #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
          // Correct bilinear grid for new probe offset
          const float diff = p_val - probe.z_offset;
          if (diff) {
            for (uint8_t x = 0; x < GRID_MAX_POINTS_X; x++)
              for (uint8_t y = 0; y < GRID_MAX_POINTS_Y; y++)
                bedlevel.z_values[x][y] += diff;
          }
          #if ENABLED(ABL_BILINEAR_SUBDIVISION)
            bedlevel.bed_level_virt_interpolate();
          #endif
        #endif

        probe.z_offset = p_val;
        SERIAL_VAL(probe.z_offset);
      }
      else {
        SERIAL_MT(MSG_Z_MIN, Z_PROBE_OFFSET_RANGE_MIN);
        SERIAL_CHR(' ');
        SERIAL_MT(MSG_Z_MAX, Z_PROBE_OFFSET_RANGE_MAX);
      }
    }
    else {
      SERIAL_MV(": ", probe.z_offset, 3);
    }

    SERIAL_EOL();
  }

#elif MECH(DELTA)

  /**
   * M666: Set delta endstop and geometry adjustment
   *
   *    D = Diagonal Rod
   *    R = Delta Radius
   *    S = Segments per Second
   *    A = Alpha (Tower 1) Diagonal Rod Adjust
   *    B = Beta  (Tower 2) Diagonal Rod Adjust
   *    C = Gamma (Tower 3) Diagonal Rod Adjust
   *    I = Alpha (Tower 1) Tower Radius Adjust
   *    J = Beta  (Tower 2) Tower Radius Adjust
   *    K = Gamma (Tower 3) Tower Radius Adjust
   *    U = Alpha (Tower 1) Tower Position Adjust
   *    V = Beta  (Tower 2) Tower Position Adjust
   *    W = Gamma (Tower 3) Tower Position Adjust
   *    X = Alpha (Tower 1) Endstop Adjust
   *    Y = Beta  (Tower 2) Endstop Adjust
   *    Z = Gamma (Tower 3) Endstop Adjust
   *    O = Print radius
   *    Q = Probe radius
   *    P = Z probe offset
   *    H = Z Height
   */
  inline void gcode_M666() {

    if (parser.seen('H')) {
      const float old_delta_height = mechanics.delta_height;
      mechanics.delta_height = parser.value_linear_units();
      mechanics.current_position[Z_AXIS] += mechanics.delta_height - old_delta_height;
    }

    if (parser.seen('D')) mechanics.delta_diagonal_rod              = parser.value_linear_units();
    if (parser.seen('R')) mechanics.delta_radius                    = parser.value_linear_units();
    if (parser.seen('S')) mechanics.delta_segments_per_second       = parser.value_float();
    if (parser.seen('A')) mechanics.delta_diagonal_rod_adj[A_AXIS]  = parser.value_linear_units();
    if (parser.seen('B')) mechanics.delta_diagonal_rod_adj[B_AXIS]  = parser.value_linear_units();
    if (parser.seen('C')) mechanics.delta_diagonal_rod_adj[C_AXIS]  = parser.value_linear_units();
    if (parser.seen('I')) mechanics.delta_tower_radius_adj[A_AXIS]  = parser.value_linear_units();
    if (parser.seen('J')) mechanics.delta_tower_radius_adj[B_AXIS]  = parser.value_linear_units();
    if (parser.seen('K')) mechanics.delta_tower_radius_adj[C_AXIS]  = parser.value_linear_units();
    if (parser.seen('U')) mechanics.delta_tower_pos_adj[A_AXIS]     = parser.value_linear_units();
    if (parser.seen('V')) mechanics.delta_tower_pos_adj[B_AXIS]     = parser.value_linear_units();
    if (parser.seen('W')) mechanics.delta_tower_pos_adj[C_AXIS]     = parser.value_linear_units();
    if (parser.seen('O')) mechanics.delta_print_radius              = parser.value_linear_units();
    if (parser.seen('Q')) mechanics.delta_probe_radius              = parser.value_linear_units();

    mechanics.recalc_delta_settings();

    #if HAS_BED_PROBE

      if (parser.seen('P')) {

        SERIAL_SM(ECHO, MSG_ZPROBE_ZOFFSET);
        SERIAL_CHR(' ');

        float p_val = parser.value_linear_units();
        if (Z_PROBE_OFFSET_RANGE_MIN <= p_val && p_val <= Z_PROBE_OFFSET_RANGE_MAX) {

          #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
            // Correct bilinear grid for new probe offset
            const float diff = p_val - probe.z_offset;
            if (diff) {
              for (uint8_t x = 0; x < GRID_MAX_POINTS_X; x++)
                for (uint8_t y = 0; y < GRID_MAX_POINTS_Y; y++)
                  bedlevel.z_values[x][y] += diff;
            }
            #if ENABLED(ABL_BILINEAR_SUBDIVISION)
              bedlevel.bed_level_virt_interpolate();
            #endif
          #endif

          probe.z_offset = p_val;
          SERIAL_VAL(probe.z_offset);
        }
        else {
          SERIAL_MT(MSG_Z_MIN, Z_PROBE_OFFSET_RANGE_MIN);
          SERIAL_CHR(' ');
          SERIAL_MT(MSG_Z_MAX, Z_PROBE_OFFSET_RANGE_MAX);
        }

        SERIAL_EOL();
      }

    #endif // HAS_BED_PROBE

    LOOP_XYZ(i) {
      if (parser.seen(axis_codes[i])) mechanics.delta_endstop_adj[i] = parser.value_linear_units();
    }

    if (parser.seen('L')) {
      SERIAL_LM(CFG, "Current Delta geometry values:");
      LOOP_XYZ(i) {
        SERIAL_SV(CFG, axis_codes[i]);
        SERIAL_EMV(" (Endstop Adj): ", mechanics.delta_endstop_adj[i], 3);
      }

      #if HAS_BED_PROBE
        SERIAL_LMV(CFG, "P (ZProbe ZOffset): ", probe.z_offset, 3);
      #endif

      SERIAL_LMV(CFG, "A (Tower A Diagonal Rod Correction): ",  mechanics.delta_diagonal_rod_adj[0], 3);
      SERIAL_LMV(CFG, "B (Tower B Diagonal Rod Correction): ",  mechanics.delta_diagonal_rod_adj[1], 3);
      SERIAL_LMV(CFG, "C (Tower C Diagonal Rod Correction): ",  mechanics.delta_diagonal_rod_adj[2], 3);
      SERIAL_LMV(CFG, "I (Tower A Radius Correction): ",        mechanics.delta_tower_radius_adj[0], 3);
      SERIAL_LMV(CFG, "J (Tower B Radius Correction): ",        mechanics.delta_tower_radius_adj[1], 3);
      SERIAL_LMV(CFG, "K (Tower C Radius Correction): ",        mechanics.delta_tower_radius_adj[2], 3);
      SERIAL_LMV(CFG, "U (Tower A Position Correction): ",      mechanics.delta_tower_pos_adj[0], 3);
      SERIAL_LMV(CFG, "V (Tower B Position Correction): ",      mechanics.delta_tower_pos_adj[1], 3);
      SERIAL_LMV(CFG, "W (Tower C Position Correction): ",      mechanics.delta_tower_pos_adj[2], 3);
      SERIAL_LMV(CFG, "R (Delta Radius): ",                     mechanics.delta_radius, 4);
      SERIAL_LMV(CFG, "D (Diagonal Rod Length): ",              mechanics.delta_diagonal_rod, 4);
      SERIAL_LMV(CFG, "S (Delta Segments per second): ",        mechanics.delta_segments_per_second);
      SERIAL_LMV(CFG, "O (Delta Print Radius): ",               mechanics.delta_print_radius);
      SERIAL_LMV(CFG, "Q (Delta Probe Radius): ",               mechanics.delta_probe_radius);
      SERIAL_LMV(CFG, "H (Z-Height): ",                         mechanics.delta_height, 3);
    }
  }

#endif // MECH DELTA

#if ENABLED(LIN_ADVANCE)

  /**
   * M900: Set and/or Get advance K factor and WH/D ratio
   *
   *  K<factor>                  Set advance K factor
   *  R<ratio>                   Set ratio directly (overrides WH/D)
   *  W<width> H<height> D<diam> Set ratio from WH/D
   */
  inline void gcode_M900() {
    stepper.synchronize();

    const float newK = parser.seen('K') ? parser.value_float() : -1;
    if (newK >= 0) planner.extruder_advance_k = newK;

    float newR = parser.seen('R') ? parser.value_float() : -1;
    if (newR < 0) {
      const float newD = parser.seen('D') ? parser.value_float() : -1,
                  newW = parser.seen('W') ? parser.value_float() : -1,
                  newH = parser.seen('H') ? parser.value_float() : -1;
      if (newD >= 0 && newW >= 0 && newH >= 0)
        newR = newD ? (newW * newH) / (sq(newD * 0.5) * M_PI) : 0;
    }
    if (newR >= 0) planner.advance_ed_ratio = newR;

    SERIAL_SMV(ECHO, "Advance K=", planner.extruder_advance_k);
    SERIAL_MSG(" E/D=");
    if (planner.advance_ed_ratio) SERIAL_VAL(planner.advance_ed_ratio);
    else SERIAL_MSG("Auto");
    SERIAL_EOL();
  }

#endif // LIN_ADVANCE

#if MB(ALLIGATOR) || MB(ALLIGATOR_V3)

  /**
   * M906: Set motor currents
   */
  inline void gcode_M906() {

    GET_TARGET_EXTRUDER(906);

    LOOP_XYZE(i) {
      if (parser.seen(axis_codes[i])) {
        const uint8_t a = i + (i == E_AXIS ? TARGET_EXTRUDER : 0);
        motor_current[a] = parser.value_float();
      }
    }
    stepper.set_driver_current();
  }

#elif ENABLED(HAVE_TMC2130)

  static void tmc2130_get_current(TMC2130Stepper &st, const char name) {
    SERIAL_CHR(name);
    SERIAL_MSG(" axis driver current: ");
    SERIAL_EV(st.getCurrent());
  }
  static void tmc2130_set_current(TMC2130Stepper &st, const char name, const int mA) {
    st.setCurrent(mA, R_SENSE, HOLD_MULTIPLIER);
    tmc2130_get_current(st, name);
  }

  static void tmc2130_report_otpw(TMC2130Stepper &st, const char name) {
    SERIAL_CHR(name);
    SERIAL_MSG(" axis temperature prewarn triggered: ");
    SERIAL_PS(st.getOTPW() ? PSTR("true") : PSTR("false"));
    SERIAL_EOL();
  }
  static void tmc2130_clear_otpw(TMC2130Stepper &st, const char name) {
    st.clear_otpw();
    SERIAL_CHR(name);
    SERIAL_EM(" prewarn flag cleared");
  }

  static void tmc2130_get_pwmthrs(TMC2130Stepper &st, const char name, const uint16_t spmm) {
    SERIAL_CHR(name);
    SERIAL_MSG(" stealthChop max speed set to ");
    SERIAL_EV(12650000UL * st.microsteps() / (256 * st.stealth_max_speed() * spmm));
  }
  static void tmc2130_set_pwmthrs(TMC2130Stepper &st, const char name, const int32_t thrs, const uint32_t spmm) {
    st.stealth_max_speed(12650000UL * st.microsteps() / (256 * thrs * spmm));
    tmc2130_get_pwmthrs(st, name, spmm);
  }

  static void tmc2130_get_sgt(TMC2130Stepper &st, const char name) {
    SERIAL_CHR(name);
    SERIAL_MSG(" driver homing sensitivity set to ");
    SERIAL_EV(st.sgt());
  }
  static void tmc2130_set_sgt(TMC2130Stepper &st, const char name, const int8_t sgt_val) {
    st.sgt(sgt_val);
    tmc2130_get_sgt(st, name);
  }

  /**
   * M906: Set motor current in milliamps using axis codes X, Y, Z, E
   * Report driver currents when no axis specified
   *
   * S1: Enable automatic current control
   * S0: Disable
   */
  inline void gcode_M906() {
    uint16_t values[NUM_AXIS];
    LOOP_XYZE(i)
      values[i] = parser.seen(axis_codes[i]) ? parser.value_int() : 0;

    #if ENABLED(X_IS_TMC2130)
      if (values[X_AXIS]) tmc2130_set_current(values[X_AXIS], stepperX, 'X');
      else tmc2130_get_current(stepperX, 'X');
    #endif
    #if ENABLED(Y_IS_TMC2130)
      if (values[Y_AXIS]) tmc2130_set_current(values[Y_AXIS], stepperY, 'Y');
      else tmc2130_get_current(stepperY, 'Y');
    #endif
    #if ENABLED(Z_IS_TMC2130)
      if (values[Z_AXIS]) tmc2130_set_current(values[Z_AXIS], stepperZ, 'Z');
      else tmc2130_get_current(stepperZ, 'Z');
    #endif
    #if ENABLED(E0_IS_TMC2130)
      if (values[E_AXIS]) tmc2130_set_current(values[E_AXIS], stepperE0, 'E');
      else tmc2130_get_current(stepperE0, 'E');
    #endif

    #if ENABLED(AUTOMATIC_CURRENT_CONTROL)
      if (parser.seen('S')) auto_current_control = parser.value_bool();
    #endif
  }

  /**
   * M911: Report TMC2130 stepper driver overtemperature pre-warn flag
   * The flag is held by the library and persist until manually cleared by M912
   */
  inline void gcode_M911() {
    const bool reportX = parser.seen('X'), reportY = parser.seen('Y'), reportZ = parser.seen('Z'), reportE = parser.seen('E'),
             reportAll = (!reportX && !reportY && !reportZ && !reportE) || (reportX && reportY && reportZ && reportE);
    #if ENABLED(X_IS_TMC2130)
      if (reportX || reportAll) tmc2130_report_otpw(stepperX, 'X');
    #endif
    #if ENABLED(Y_IS_TMC2130)
      if (reportY || reportAll) tmc2130_report_otpw(stepperY, 'Y');
    #endif
    #if ENABLED(Z_IS_TMC2130)
      if (reportZ || reportAll) tmc2130_report_otpw(stepperZ, 'Z');
    #endif
    #if ENABLED(E0_IS_TMC2130)
      if (reportE || reportAll) tmc2130_report_otpw(stepperE0, 'E');
    #endif
  }

  /**
   * M912: Clear TMC2130 stepper driver overtemperature pre-warn flag held by the library
   */
  inline void gcode_M912() {
    const bool clearX = parser.seen('X'), clearY = parser.seen('Y'), clearZ = parser.seen('Z'), clearE = parser.seen('E'),
             clearAll = (!clearX && !clearY && !clearZ && !clearE) || (clearX && clearY && clearZ && clearE);
    #if ENABLED(X_IS_TMC2130)
      if (clearX || clearAll) tmc2130_clear_otpw(stepperX, 'X');
    #endif
    #if ENABLED(Y_IS_TMC2130)
      if (clearY || clearAll) tmc2130_clear_otpw(stepperY, 'Y');
    #endif
    #if ENABLED(Z_IS_TMC2130)
      if (clearZ || clearAll) tmc2130_clear_otpw(stepperZ, 'Z');
    #endif
    #if ENABLED(E0_IS_TMC2130)
      if (clearE || clearAll) tmc2130_clear_otpw(stepperE0, 'E');
    #endif
  }

  /**
   * M913: Set HYBRID_THRESHOLD speed.
   */
  #if ENABLED(HYBRID_THRESHOLD)
    inline void gcode_M913() {
      uint16_t values[XYZE];
      LOOP_XYZE(i)
        values[i] = parser.seen(axis_codes[i]) ? parser.value_int() : 0;

      #if ENABLED(X_IS_TMC2130)
        if (values[X_AXIS]) tmc2130_set_pwmthrs(stepperX, 'X', values[X_AXIS], mechanics.axis_steps_per_mm[X_AXIS]);
        else tmc2130_get_pwmthrs(stepperX, 'X', mechanics.axis_steps_per_mm[X_AXIS]);
      #endif
      #if ENABLED(Y_IS_TMC2130)
        if (values[Y_AXIS]) tmc2130_set_pwmthrs(stepperY, 'Y', values[Y_AXIS], mechanics.axis_steps_per_mm[Y_AXIS]);
        else tmc2130_get_pwmthrs(stepperY, 'Y', mechanics.axis_steps_per_mm[Y_AXIS]);
      #endif
      #if ENABLED(Z_IS_TMC2130)
        if (values[Z_AXIS]) tmc2130_set_pwmthrs(stepperZ, 'Z', values[Z_AXIS], mechanics.axis_steps_per_mm[Z_AXIS]);
        else tmc2130_get_pwmthrs(stepperZ, 'Z', mechanics.axis_steps_per_mm[Z_AXIS]);
      #endif
      #if ENABLED(E0_IS_TMC2130)
        if (values[E_AXIS]) tmc2130_set_pwmthrs(stepperE0, 'E', values[E_AXIS], mechanics.axis_steps_per_mm[E_AXIS]);
        else tmc2130_get_pwmthrs(stepperE0, 'E', mechanics.axis_steps_per_mm[E_AXIS]);
      #endif
    }
  #endif // HYBRID_THRESHOLD

  /**
   * M914: Set SENSORLESS_HOMING sensitivity.
   */
  #if ENABLED(SENSORLESS_HOMING)
    inline void gcode_M914() {
      #if ENABLED(X_IS_TMC2130)
        if (parser.seen(axis_codes[X_AXIS])) tmc2130_set_sgt(stepperX, 'X', parser.value_int());
        else tmc2130_get_sgt(stepperX, 'X');
      #endif
      #if ENABLED(Y_IS_TMC2130)
        if (parser.seen(axis_codes[Y_AXIS])) tmc2130_set_sgt(stepperY, 'Y', parser.value_int());
        else tmc2130_get_sgt(stepperY, 'Y');
      #endif
    }
  #endif // SENSORLESS_HOMING

#endif // HAVE_TMC2130

/**
 * M907: Set digital trimpot motor current using axis codes X, Y, Z, E, B, S
 */
inline void gcode_M907() {
  #if HAS_DIGIPOTSS
    LOOP_XYZE(i)
      if (parser.seen(axis_codes[i])) stepper.digipot_current(i, parser.value_int());
    if (parser.seen('B')) stepper.digipot_current(4, parser.value_int());
    if (parser.seen('S')) for (uint8_t i = 0; i <= 4; i++) stepper.digipot_current(i, parser.value_int());
  #elif HAS_MOTOR_CURRENT_PWM
    #if PIN_EXISTS(MOTOR_CURRENT_PWM_XY)
      if (parser.seen('X')) stepper.digipot_current(0, parser.value_int());
    #endif
    #if PIN_EXISTS(MOTOR_CURRENT_PWM_Z)
      if (parser.seen('Z')) stepper.digipot_current(1, parser.value_int());
    #endif
    #if PIN_EXISTS(MOTOR_CURRENT_PWM_E)
      if (parser.seen('E')) stepper.digipot_current(2, parser.value_int());
    #endif
  #endif
  #if ENABLED(DIGIPOT_I2C)
    // this one uses actual amps in floating point
    LOOP_XYZE(i) if (parser.seen(axis_codes[i])) digipot_i2c_set_current(i, parser.value_float());
    // for each additional extruder (named B,C,D,E..., channels 4,5,6,7...)
    for (uint8_t i = NUM_AXIS; i < DIGIPOT_I2C_NUM_CHANNELS; i++) if(parser.seen('B' + i - (NUM_AXIS))) digipot_i2c_set_current(i, parser.value_float());
  #endif
}

#if HAS_DIGIPOTSS
  /**
   * M908: Control digital trimpot directly (M908 P<pin> S<current>)
   */
  inline void gcode_M908() {
    digitalPotWrite(
      parser.intval('P'),
      parser.intval('S')
    );
  }
#endif // HAS_DIGIPOTSS

#if ENABLED(NEXTION) && ENABLED(NEXTION_GFX)

  /**
   * M995: Nextion Origin
   */
  inline void gcode_M995() { gfx_origin(parser.linearval('X'), parser.linearval('Y'), parser.linearval('Z')); }

  /**
   * M996: Nextion Scale
   */
  inline void gcode_M996() {
    if (parser.seenval('S')) gfx_scale(parser.value_float());
  }

#endif

#if ENABLED(NPR2)

  /**
   * M997: Cxx Move Carter xx gradi
   */
  inline void gcode_M997() {
    long csteps;
    if (parser.seenval('C')) {
      csteps = parser.value_ulong() * color_step_moltiplicator;
      SERIAL_EMV("csteps: ", csteps);
      if (csteps < 0) stepper.colorstep(-csteps, false);
      if (csteps > 0) stepper.colorstep(csteps, true);
    }
  }

#endif

/**
 * M999: Restart after being stopped
 *
 * Default behaviour is to flush the serial buffer and request
 * a resend to the host starting on the last N line received.
 *
 * Sending "M999 S1" will resume printing without flushing the
 * existing command buffer.
 *
 */
inline void gcode_M999() {
  Running = true;
  lcd_reset_alert_level();

  if (parser.boolval('S')) return;

  FlushSerialRequestResend();
}

/**
 * T0-TN: Switch tool, usually switching extruders or CNC tools
 *
 * For Extruders:
 *   F[units/min] Set the movement feedrate
 *   S1           Don't move the tool in XY after change
 *
 * For CNC no other parameters are expected
 *
 */
inline void gcode_T(uint8_t tool_id) {

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_MV(">>> gcode_T(", tool_id);
      SERIAL_CHR(')'); SERIAL_EOL();
      DEBUG_POS("BEFORE", mechanics.current_position);
    }
  #endif

  #if ENABLED(CNCROUTER)
    
    bool wait = true;
    bool raise_z = false;

    if (printer_mode == PRINTER_MODE_CNC) {
      // Host manage wait on change, don't block
      if (parser.seen('W')) wait = false;
      // Host manage position, don't raise Z
      if (parser.seen('Z')) raise_z = false;

      tool_change_cnc(tool_id, wait, raise_z);
    }

  #endif

  #if EXTRUDERS == 1 && ENABLED(ADVANCED_PAUSE_FEATURE)

    if (printer_mode == PRINTER_MODE_FFF && (IS_SD_PRINTING || print_job_counter.isRunning()) && previous_extruder != tool_id) {
      gcode_M600();
      previous_extruder = tool_id;
    }

  #elif EXTRUDERS > 1 && (HOTENDS == 1 || (ENABLED(COLOR_MIXING_EXTRUDER) && MIXING_VIRTUAL_TOOLS > 1))

    if (printer_mode == PRINTER_MODE_FFF) tool_change(tool_id);

  #elif EXTRUDERS > 1 && HOTENDS > 1

    if (printer_mode == PRINTER_MODE_FFF) tool_change(
      tool_id,
      MMM_TO_MMS(parser.linearval('F')),
      (tool_id == active_extruder) || parser.boolval('S')
    );

  #endif

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      DEBUG_POS("AFTER", mechanics.current_position);
      SERIAL_EM("<<< gcode_T");
    }
  #endif
}


#if ENABLED(NPR2)

  void MK_multi_tool_change(const uint8_t &e) {

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

  void MK_multi_tool_change(const uint8_t e) {

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

  void MK_multi_tool_change(const uint8_t &e) {

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

  void MK_multi_tool_change(const uint8_t &e) {

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

#if HAS_DONDOLO

  inline void move_extruder_servo(const uint8_t e) {
    const int angles[2] = { DONDOLO_SERVOPOS_E0, DONDOLO_SERVOPOS_E1 };
    MOVE_SERVO(DONDOLO_SERVO_INDEX, angles[e]);

    #if (DONDOLO_SERVO_DELAY > 0)
      safe_delay(DONDOLO_SERVO_DELAY);
    #endif
  }

#endif

inline void invalid_extruder_error(const uint8_t e) {
  SERIAL_SMV(ER, "T", (int)e);
  SERIAL_EM(" " MSG_INVALID_EXTRUDER);
}

#if EXTRUDERS > 1 || ENABLED(COLOR_MIXING_EXTRUDER)

  void tool_change(const uint8_t tmp_extruder, const float fr_mm_s/*=0.0*/, bool no_move/*=false*/) {

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
              && IsRunning()
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
        if (!no_move && IsRunning()) {
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

#if ENABLED(CNCROUTER)

  // TODO: manage auto tool change 
  void tool_change_cnc(uint8_t tool_id, bool wait/*=true*/, bool raise_z/*=true*/) {

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

/**
 * Process a single command and dispatch it to its handler
 * This is called from the main loop()
 */
void process_next_command() {
  char * const current_command = command_queue[cmd_queue_index_r];

  if (DEBUGGING(ECHO)) {
    SERIAL_LV(ECHO, current_command);
    #if ENABLED(M100_FREE_MEMORY_WATCHER)
      SERIAL_SMV(ECHO, "slot:", cmd_queue_index_r);
      #if ENABLED(M100_FREE_MEMORY_DUMPER)
        M100_dump_routine("   Command Queue:", (const char*)command_queue, (const char*)(command_queue + sizeof(command_queue)));
      #endif
    #endif
  }

  KEEPALIVE_STATE(IN_HANDLER);

  // Parse the next command in the queue
  parser.parse(current_command);

  // Handle a known G, M, or T
  switch(parser.command_letter) {
    case 'G': switch (parser.codenum) {

      // G0, G1
      case 0:
      case 1:
        #if IS_SCARA
          gcode_G0_G1(parser.codenum == 0); break;
        #elif ENABLED(LASER)
          gcode_G0_G1(parser.codenum == 1); break;
        #else
          gcode_G0_G1(); break;
        #endif

      // G2, G3
      #if ENABLED(ARC_SUPPORT)
        case 2: // G2  - CW ARC
        case 3: // G3  - CCW ARC
          gcode_G2_G3(parser.codenum == 2); break;
      #endif

      // G4 Dwell
      case 4:
        gcode_G4(); break;

      #if ENABLED(LASER)
        #if ENABLED(G5_BEZIER)
          case 5: // G5: Bezier curve - from http://forums.reprap.org/read.php?147,93577
            gcode_G5(); break;
        #endif // G5_BEZIER

        #if ENABLED(LASER_RASTER)
          case 7: // G7: Execute laser raster line
            gcode_G7(); break;
        #endif // LASER_RASTER
      #endif // LASER

      #if ENABLED(FWRETRACT)
        case 10: // G10: retract
        case 11: // G11: retract_recover
          gcode_G10_G11(parser.codenum == 10); break;
      #endif // FWRETRACT

      // G17 - G19: XXX CNC plane selection
      // G17 -> XY (default)
      // G18 -> ZX 
      // G19 -> YZ

      #if ENABLED(NOZZLE_CLEAN_FEATURE)
        case 12: // G12: Nozzle Clean
          gcode_G12(); break;
      #endif // NOZZLE_CLEAN_FEATURE

      #if ENABLED(CNC_WORKSPACE_PLANES)
        case 17: // G17: Select Plane XY
          gcode_G17(); break;
        case 18: // G18: Select Plane ZX
          gcode_G18(); break;
        case 19: // G19: Select Plane YZ
          gcode_G19(); break;
      #endif // CNC_WORKSPACE_PLANES

      #if ENABLED(INCH_MODE_SUPPORT)
        case 20: //G20: Inch Mode
          gcode_G20(); break;

        case 21: //G21: MM Mode
          gcode_G21(); break;
      #endif // INCH_MODE_SUPPORT

      #if ENABLED(NOZZLE_PARK_FEATURE)
        case 27: // G27: Nozzle Park
          gcode_G27(); break;
      #endif // NOZZLE_PARK_FEATURE

      case 28: //G28: Home all axes, one at a time
        gcode_G28(false); break;

      #if HAS_LEVELING
        case 29: // G29 Detailed Z probe, probes the bed at 3 or more points.
          gcode_G29(); break;
      #endif // HAS_LEVELING

      #if HAS_BED_PROBE
        case 30: // G30 Single Z Probe
          gcode_G30(); break;
        
        #if ENABLED(Z_PROBE_SLED)
          case 31: // G31: dock the sled
            gcode_G31(); break;
          case 32: // G32: undock the sled
            gcode_G32(); break;
        #endif // Z_PROBE_SLED
      #endif // HAS_BED_PROBE

      #if HAS_DELTA_AUTO_CALIBRATION
        case 33: // G33 Delta AutoCalibration
          gcode_G33(); break;
      #endif // HAS_DELTA_AUTO_CALIBRATION

      #if ENABLED(G38_PROBE_TARGET)
        case 38: // G38.2 & G38.3
          if (subcode == 2 || subcode == 3)
            gcode_G38(subcode == 2);
          break;
      #endif

      #if ENABLED(AUTO_BED_LEVELING_BILINEAR) || ENABLED(MESH_BED_LEVELING)
        case 42: // G42: Move X & Y axes to mesh coordinates (I & J)
          gcode_G42(); break;
      #endif

      // G40 Compensation Off XXX CNC
      // G54-G59 Coordinate system selection (CNC XXX)      

      case 60: // G60 Saved Coordinates
        gcode_G60(); break;
      case 61: // G61 Restore Coordinates
        gcode_G61(); break;
   
      // G80: Cancel Canned Cycle (XXX CNC)

      case 90: // G90 - Use Absolute Coordinates
        relative_mode = false; break;
      case 91: // G91 - Use Relative Coordinates
        relative_mode = true; break;

      case 92: // G92
        gcode_G92(); break;

      // G92.x Reset Coordinate System Offset (CNC XXX)
      // G93: Feed Rate Mode (Inverse Time Mode) (CNC XXX)
      // G94: Feed Rate Mode (Units per Minute) (CNC XXX)

      #if ENABLED(DEBUG_GCODE_PARSER)
        case 800: // GCode Parser Test
          parser.debug(); break;
      #endif
    }
    break;

    case 'M': switch (parser.codenum) {
      #if ENABLED(ULTIPANEL) || ENABLED(EMERGENCY_PARSER)
        case 0: // M0: Unconditional stop - Wait for user button press on LCD
        case 1: // M1: Conditional stop - Wait for user button press on LCD
          gcode_M0_M1(); break;
      #endif // ULTIPANEL || EMERGENCY_PARSER

      #if ENABLED(LASER) || ENABLED(CNCROUTER)
        case 3: // M03: Setting laser beam or CNC clockwise speed
        case 4: // M04: Turn on laser beam or CNC counter clockwise speed
          gcode_M3_M4(parser.codenum == 3); break;
        case 5: // M05: Turn off laser beam or CNC stop
          gcode_M5(); break;
      #endif // LASER || CNCROUTER

      #if ENABLED(CNCROUTER)
        case 6: // M06: Tool change CNC
          gcode_M6(); break;
        // case 7: // M07 - Mist coolant CNC XXX
        // case 8: // M08 - Flood coolant CNC XXX
        // case 9: // M09 - Coolant off CNC XXX
        // case 10: // M10 - Vacuum on CNC XXX
        // case 11: // M11 - Vacuum off CNC XXX
      #endif // CNCROUTER

      case 17: // M17: Enable/Power all stepper motors
        gcode_M17(); break;

      #if HAS_SDSUPPORT
        case 20: // M20: list SD card
          gcode_M20(); break;
        case 21: // M21: init SD card
          gcode_M21(); break;
        case 22: // M22: release SD card
          gcode_M22(); break;
        case 23: // M23: Select file
          gcode_M23(); break;
        case 24: // M24: Start SD print
          gcode_M24(); break;
        case 25: // M25: Pause SD print
          gcode_M25(); break;
        case 26: // M26: Set SD index
          gcode_M26(); break;
        case 27: // M27: Get SD status
          gcode_M27(); break;
        case 28: // M28: Start SD write
          gcode_M28(); break;
        case 29: // M29: Stop SD write
          gcode_M29(); break;
        case 30: // M30 <filename> Delete File
          gcode_M30(); break;
      #endif // SDSUPPORT

      case 31: // M31: Report time since the start of SD print or last M109
        gcode_M31(); break;

      #if HAS_SDSUPPORT
        case 32: // M32: Make directory
          gcode_M32(); break;
        case 33: // M33: Stop printing, close file and save restart.gcode
          gcode_M33(); break;
        case 34: // M34: Select file and start SD print
          gcode_M34(); break;
        #if ENABLED(NEXTION)
          case 35: // M35: Upload Firmware to Nextion from SD
            gcode_M35(); break;
        #endif
      #endif // SDSUPPORT

      case 42: // M42: Change pin state
        gcode_M42(); break;

      #if ENABLED(PINS_DEBUGGING)
        case 43: // M43: Read pin state
          gcode_M43(); break;
      #endif

      #if ENABLED(Z_MIN_PROBE_REPEATABILITY_TEST)
        case 48: // M48: Z probe repeatability test
          gcode_M48(); break;
      #endif

      #if HAS_POWER_CONSUMPTION_SENSOR
        case 70: // M70: Power consumption sensor calibration
          gcode_M70(); break;
      #endif

      case 75: // M75: Start print timer
        gcode_M75(); break;
      case 76: // M76: Pause print timer
        gcode_M76(); break;
      case 77: // M77: Stop print timer
        gcode_M77(); break;
      case 78: // M78: Show print statistics
        gcode_M78(); break;

      #if HAS_POWER_SWITCH
        case 80: // M80: Turn on Power Supply
          gcode_M80(); break;
      #endif

      case 81: // M81: Turn off Power, including Power Supply, if possible
        gcode_M81(); break;
      case 82: // M82: Set E axis normal mode (same as other axes)
        gcode_M82(); break;
      case 83: // M83: Set E axis relative mode
        gcode_M83(); break;
      case 18: // M18 => M84
      case 84: // M84: Disable all steppers or set timeout
        gcode_M18_M84(); break;
      case 85: // M85: Set inactivity stepper shutdown timeout
        gcode_M85(); break;
      case 92: // M92: Set the steps-per-unit for one or more axes
        gcode_M92(); break;

      #if ENABLED(ZWOBBLE)
        case 96: // M96: Print ZWobble value
          gcode_M96(); break;
        case 97: // M97: Set ZWobble parameter
          gcode_M97(); break;
      #endif

      #if ENABLED(HYSTERESIS)
        case 98: // M98: Print Hysteresis value
          gcode_M98(); break;
        case 99: // M99: Set Hysteresis parameter
          gcode_M99(); break;
      #endif

      #if ENABLED(M100_FREE_MEMORY_WATCHER)
        case 100: // M100: Free Memory Report
          gcode_M100(); break;
      #endif

      #if HAS_TEMP_HOTEND
        case 104: // M104: Set hot end temperature
          gcode_M104(); break;
      #endif

      case 105: // M105: Report current temperature
        gcode_M105();
        KEEPALIVE_STATE(NOT_BUSY);
        return; // "ok" already printed

      #if FAN_COUNT > 0
        case 106: // M106: Fan On
          gcode_M106(); break;
        case 107: // M107: Fan Off
          gcode_M107(); break;
      #endif // FAN_COUNT > 0

      #if DISABLED(EMERGENCY_PARSER)
        case 108: // M108: Cancel heatup
          gcode_M108(); break;
      #endif

      #if HAS_TEMP_HOTEND
        case 109: // M109: Wait for hotend temperature to reach target
          gcode_M109(); break;
      #endif

      case 110: // M110: Set Current Line Number
        gcode_M110(); break;

      case 111: // M111: Set debug level
        gcode_M111(); break;

      #if DISABLED(EMERGENCY_PARSER)
        case 112: //  M112: Emergency Stop
          gcode_M112(); break;
      #endif

      #if ENABLED(HOST_KEEPALIVE_FEATURE)
        case 113: // M113: Set Host Keepalive interval
          gcode_M113(); break;
      #endif

      case 114: // M114: Report current position
        gcode_M114(); break;

      case 115: // M115: Report capabilities
        gcode_M115(); break;

      case 117: // M117: Set LCD message text, if possible
        gcode_M117(); break;

      case 118: // M118: Print to Host the message text
        gcode_M118(); break;

      case 119: // M119: Report endstop states
        gcode_M119(); break;
      case 120: // M120: Enable endstops
        gcode_M120(); break;
      case 121: // M121: Disable endstops
        gcode_M121(); break;
      case 122: // M122: Disable or enable software endstops
        gcode_M122(); break;

      #if ENABLED(HAVE_TMC2130DRIVER)
        case 123: // M123: Diagnose, used to debug TMC2130
          gcode_M123(); break;
      #endif

      #if ENABLED(PARK_HEAD_ON_PAUSE)
        case 125: // M125: Store current position and move to pause park position
          gcode_M125(); break;
      #endif

      #if ENABLED(BARICUDA)
        // PWM for HEATER_1_PIN
        #if HAS_HEATER_1
          case 126: // M126 valve open
            gcode_M126(); break;
          case 127: // M127 valve closed
            gcode_M127(); break;
        #endif // HAS_HEATER_1

        // PWM for HEATER_2_PIN
        #if HAS_HEATER_2
          case 128: // M128 valve open
            gcode_M128(); break;
          case 129: // M129 valve closed
            gcode_M129(); break;
        #endif // HAS_HEATER_2
      #endif // BARICUDA

      #if HAS_TEMP_BED
        case 140: // M140 - Set bed temp
          gcode_M140(); break;
      #endif

      #if HAS_TEMP_CHAMBER
        case 141: // M141 - Set chamber temp
          gcode_M141(); break;
      #endif

      #if HAS_TEMP_COOLER
        case 142: // M142 - Set cooler temp
          gcode_M142(); break;
      #endif

      #if ENABLED(ULTIPANEL) && HAS_TEMP_0
        case 145: // M145: Set material heatup parameters
          gcode_M145(); break;
      #endif

      #if ENABLED(TEMPERATURE_UNITS_SUPPORT)
        case 149: // M149: Set temperature units
          gcode_M149(); break;
      #endif

      #if HAS_COLOR_LEDS
        case 150: // M150
          gcode_M150(); break;
      #endif //BLINKM

      #if ENABLED(AUTO_REPORT_TEMPERATURES) && (HAS_TEMP_HOTEND || HAS_TEMP_BED)
        case 155: // M155: Set temperature auto-report interval
          gcode_M155(); break;
      #endif

      #if ENABLED(COLOR_MIXING_EXTRUDER)
        case 163: // M163 S<int> P<float> set weight for a mixing extruder
          gcode_M163(); break;
        #if MIXING_VIRTUAL_TOOLS > 1
          case 164: // M164 S<int> save current mix as a virtual tools
            gcode_M164(); break;
        #endif
        case 165: // M165 [ABCDHI]<float> set multiple mix weights
          gcode_M165(); break;
      #endif

      #if HAS_TEMP_BED
        case 190: // M190 - Wait for bed heater to reach target.
          gcode_M190(); break;
      #endif // TEMP_BED

      #if HAS_TEMP_CHAMBER
        case 191: // M191 - Wait for chamber heater to reach target.
          gcode_M191(); break;
      #endif

      #if HAS_TEMP_COOLER
        case 192: // M192 - Wait for chamber heater to reach target.
          gcode_M192(); break;
      #endif

      case 200: // // M200 D<diameter> Set filament diameter and set E axis units to cubic. (Use S0 to revert to linear units.)
        gcode_M200(); break;
      case 201: // M201
        gcode_M201(); break;
      #if 0 // Not used for Sprinter/grbl gen6
      case 202: // M202
        gcode_M202();
        break;
      #endif
      case 203: // M203 max mechanics.feedrate_mm_s units/sec
        gcode_M203(); break;
      case 204: // M204 mechanics.acceleration S normal moves T filament only moves
        gcode_M204(); break;
      case 205: //M205 advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk
        gcode_M205(); break;

      #if ENABLED(WORKSPACE_OFFSETS)
        case 206: // M206: Set home offsets
          gcode_M206(); break;
      #endif

      #if ENABLED(FWRETRACT)
        case 207: //M207 - M207 - Set Retract Length: S<length>, Feedrate: F<units/min>, and Z lift: Z<distance>1
          gcode_M207(); break;
        case 208: // M208 - Set Recover (unretract) Additional (!) Length: S<length> and Feedrate: F<units/min>
          gcode_M208(); break;
        case 209: // M209 - Turn Automatic Retract Detection on/off: S<bool> (For slicers that don't support G10/11). Every normal extrude-only move will be classified as retract depending on the direction.
          gcode_M209(); break;
      #endif

      case 218: // M218: Set a tool offset: T<index> X<offset> Y<offset> Z<offset>
        gcode_M218(); break;
      case 220: // M220: Set Feedrate Percentage: S<percent> ("FR" on your LCD)
        gcode_M220(); break;
      case 221: // M221: Set Flow Percentage: T<extruder> S<percent>
        gcode_M221(); break;
      case 222: // M222: Set Purge Percentage: T<extruder> S<percent>
        gcode_M222(); break;
      case 226: // M226: P<pin number> S<pin state>- Wait until the specified pin reaches the state required
        gcode_M226(); break;

      #if HAS_CHDK || HAS_PHOTOGRAPH
        case 240: // M240: Triggers a camera by emulating a Canon RC-1 : http://www.doc-diy.net/photo/rc-1_hacked/
          gcode_M240(); break;
      #endif

      #if HAS(LCD_CONTRAST)
        case 250: // M250: Set LCD contrast value: C<value> (value 0..63)
          gcode_M250(); break;
      #endif

      #if HAS_SERVOS
        case 280: // M280: Set servo position absolute
          gcode_M280(); break;
      #endif

      #if HAS_BUZZER
        case 300: // M300: Play beep tone
          gcode_M300(); break;
      #endif

      #if ENABLED(PIDTEMP)
        case 301: // M301: Set hotend PID parameters
          gcode_M301(); break;
      #endif

      #if HAS_EXTRUDERS && ENABLED(PREVENT_COLD_EXTRUSION)
        case 302: // M302: Allow cold extrudes (set the minimum extrude temperature)
          gcode_M302(); break;
      #endif

      #if ENABLED(PIDTEMP)
        case 303: // M303: PID autotune
          gcode_M303(); break;
      #endif

      #if ENABLED(PIDTEMPBED)
        case 304: // M304: Set Bed PID
          gcode_M304(); break;
      #endif

      #if ENABLED(PIDTEMPCHAMBER)
        case 305: // M305: Set Chamber PID
          gcode_M305(); break;
      #endif

      #if ENABLED(PIDTEMPCOOLER)
        case 306: // M306: Set Cooler PID
          gcode_M306(); break;
      #endif

      #if HAS_ABL
        case 320: // M320: Activate ABL
          gcode_M320(); break;
        #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
          case 321: // M321: Set a Auto Bed Leveling Z coordinate
            gcode_M321(); break;
        #endif
        case 322: // M322: Reset auto leveling matrix
          gcode_M322(); break;
      #endif

      #if HAS_MICROSTEPS
        case 350: // M350: Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
          gcode_M350(); break;
        case 351: // M351: Toggle MS1 MS2 pins directly, S# determines MS1 or MS2, X# sets the pin high/low.
          gcode_M351(); break;
      #endif

      #if HAS_CASE_LIGHT
        case 355: // M355: Turn case lights on/off
          gcode_M355(); break;
      #endif

      #if MECH(MORGAN_SCARA)
        case 360:  // M360: SCARA Theta pos1
          if (gcode_M360()) return; break;
        case 361:  // M361: SCARA Theta pos2
          if (gcode_M361()) return; break;
        case 362:  // M362: SCARA Psi pos1
          if (gcode_M362()) return; break;
        case 363:  // M363: SCARA Psi pos2
          if (gcode_M363()) return; break;
        case 364:  // M364: SCARA Psi pos3 (90 deg to Theta)
          if (gcode_M364()) return; break;
      #endif

      case 400: // M400 finish all moves
        gcode_M400(); break;

      #if HAS_BED_PROBE
        case 401: // M401: Engage Z Servo endstop if available
          gcode_M401(); break;
        case 402: // M402: Retract Z Servo endstop if enabled
          gcode_M402(); break;
      #endif

      #if ENABLED(FILAMENT_SENSOR)
        case 404:  //M404 Enter the nominal filament width (3mm, 1.75mm ) N<3.0> or display nominal filament width
          gcode_M404(); break;
        case 405:  //M405 Turn on filament sensor for control
          gcode_M405(); break;
        case 406:  //M406 Turn off filament sensor for control
          gcode_M406(); break;
        case 407:   //M407 Display measured filament diameter
          gcode_M407(); break;
      #endif

      #if ENABLED(JSON_OUTPUT)
        case 408: // M408 JSON STATUS OUTPUT
          gcode_M408(); break;
      #endif // JSON_OUTPUT

      #if DISABLED(EMERGENCY_PARSER)
        case 410: // M410 quickstop - Abort all the planned moves.
          gcode_M410(); break;
      #endif

      #if ENABLED(MESH_BED_LEVELING)
        case 420: // M420 Enable/Disable Mesh Bed Leveling
          gcode_M420(); break;
        case 421: // M421 Set a Mesh Bed Leveling Z coordinate
          gcode_M421(); break;
      #endif

      #if ENABLED(WORKSPACE_OFFSETS)
        case 428: // M428 Apply current_position to home_offset
          gcode_M428(); break;
      #endif

      #if HAS_MULTI_MODE
        case 450:
          gcode_M450(); break; // report printer mode
        case 451:
          gcode_M451(); break;    // set printer mode printer
        #if ENABLED(LASER)
          case 452:
            gcode_M452(); break;  // set printer mode laser
        #endif
        #if ENABLED(CNCROUTER)
          case 453:
            gcode_M453(); break;  // set printer mode router
        #endif
      #endif

      case 500: // M500: Store settings in EEPROM
        gcode_M500(); break;
      case 501: // M501: Read settings from EEPROM
        gcode_M501(); break;
      case 502: // M502: Revert to default settings
        gcode_M502(); break;
      case 503: // M503: print settings currently in memory
        gcode_M503(); break;

      #if ENABLED(RFID_MODULE)
        case 522: // M422: Read or Write on card. M522 T<extruders> R<read> or W<write>
          gcode_M522(); break;
      #endif

      case 530: // M530: S<printing> L<layer> - Enables explicit printing mode (S1) or disables it (S0). L can set layer count
        gcode_M530(); break;
      case 531: // M531: filename - Define filename being printed
        gcode_M531(); break;
      case 532: // M532: X<percent> L<curLayer> - update current print state progress (X=0..100) and layer L
        gcode_M532(); break;

      #if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
        case 540: // M540: Set abort on endstop hit for SD printing
          gcode_M540(); break;
      #endif

      #if HEATER_USES_AD595
        case 595: // M595 set Hotends AD595 offset & gain
          gcode_M595(); break;
      #endif

      #if ENABLED(ADVANCED_PAUSE_FEATURE)
        case 600: // Pause Park X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
          gcode_M600(); break;
      #endif

      #if ENABLED(DUAL_X_CARRIAGE)
        case 605:
          gcode_M605(); break;
      #endif

      #if ENABLED(LASER)
        case 649: // M649 set laser options
          gcode_M649(); break;
      #endif 

      #if MECH(MUVE3D)
        case 650: // M650: Set peel distance
          gcode_M650(); break;
        case 651: // M651: Run peel move and return back to start.
          gcode_M651(); break;
        case 653: // M653: Execute tilt move
          gcode_M653(); break;
        case 654: // M654 - execute untilt move
          gcode_M654(); break;
        case 655: // M655: Send projector control commands via serial
          gcode_M655(); break;
      #endif

      #if HAS_BED_PROBE || MECH(DELTA)
        case 666: // M666 Set Z probe offset or set delta endstop and geometry adjustment
          gcode_M666(); break;
      #endif

      #if ENABLED(LIN_ADVANCE)
        case 900: // M900 Set advance factor.
          gcode_M900(); break;
      #endif

      #if MB(ALLIGATOR) || MB(ALLIGATOR_V3) || ENABLED(HAVE_TMC2130)
        case 906: // M906 Set motor currents XYZ T0-4 E
          gcode_M906(); break;
      #endif

      case 907: // M907 Set digital trimpot motor current using axis codes.
        gcode_M907(); break;

      #if HAS_DIGIPOTSS
        case 908: // M908 Control digital trimpot directly.
          gcode_M908(); break;
      #endif // HAS_DIGIPOTSS

      #if ENABLED(HAVE_TMC2130)
        case 911: // M911: Report TMC2130 prewarn triggered flags
          gcode_M911(); break;
        case 912: // M912: Clear TMC2130 prewarn triggered flags
          gcode_M912(); break;

        #if ENABLED(HYBRID_THRESHOLD)
          case 913: // M913: Set HYBRID_THRESHOLD speed.
            gcode_M913(); break;
        #endif

        #if ENABLED(SENSORLESS_HOMING)
          case 914: // M914: Set SENSORLESS_HOMING sensitivity.
            gcode_M914(); break;
        #endif
      #endif

      #if ENABLED(NEXTION) && ENABLED(NEXTION_GFX)
        case 995: // M995 Nextion origin
          gcode_M995(); break;
        case 996: // M996 Nextion scale
          gcode_M996(); break;
      #endif

      #if ENABLED(NPR2)
        case 997: // M997 Cxx Move Carter xx gradi
          gcode_M997(); break;
      #endif // NPR2

      case 999: // M999: Restart after being Stopped
        gcode_M999(); break;

      #if ENABLED(DEBUG_GCODE_PARSER)
        case 800: // GCode Parser Test
          parser.debug(); break;
      #endif
    }
    break;

    case 'T':
      gcode_T(parser.codenum);
    break;

    default: parser.unknown_command_error();
  }

  KEEPALIVE_STATE(NOT_BUSY);

  ok_to_send();
}

/**
 * Send a "Resend: nnn" message to the host to
 * indicate that a command needs to be re-sent.
 */
void FlushSerialRequestResend() {
  //char command_queue[cmd_queue_index_r][100]="Resend:";
  HAL::serialFlush();
  SERIAL_LV(RESEND, gcode_LastN + 1);
  ok_to_send();
}

/**
 * Send an "ok" message to the host, indicating
 * that a command was successfully processed.
 *
 * If ADVANCED_OK is enabled also include:
 *   N<int>  Line number of the command, if any
 *   P<int>  Planner space remaining
 *   B<int>  Block queue space remaining
 */
void ok_to_send() {
  refresh_cmd_timeout();
  if (!send_ok[cmd_queue_index_r]) return;
  SERIAL_STR(OK);
  #if ENABLED(ADVANCED_OK)
    char* p = command_queue[cmd_queue_index_r];
    if (*p == 'N') {
      SERIAL_CHR(' ');
      SERIAL_CHR(*p++);
      while (NUMERIC_SIGNED(*p))
        SERIAL_CHR(*p++);
    }
    SERIAL_MV(" P", (int)(BLOCK_BUFFER_SIZE - planner.movesplanned() - 1));
    SERIAL_MV(" B", BUFSIZE - commands_in_queue);
  #endif
  SERIAL_EOL();
}

/**
 * Function for DELTA
 */
#if MECH(DELTA)

  #if ENABLED(DELTA_AUTO_CALIBRATION_3)

    void bed_probe_all() {
      // Initial throwaway probe.. used to stabilize probe
      bed_level_c = probe.check_pt(0.0, 0.0);

      // Probe all bed positions & store carriage positions
      bed_level_z = probe.check_pt(0.0, mechanics.delta_probe_radius);
      bed_level_oy = probe.check_pt(-SIN_60 * mechanics.delta_probe_radius, COS_60 * mechanics.delta_probe_radius);
      bed_level_x = probe.check_pt(-SIN_60 * mechanics.delta_probe_radius, -COS_60 * mechanics.delta_probe_radius);
      bed_level_oz = probe.check_pt(0.0, -mechanics.delta_probe_radius);
      bed_level_y = probe.check_pt(SIN_60 * mechanics.delta_probe_radius, -COS_60 * mechanics.delta_probe_radius);
      bed_level_ox = probe.check_pt(SIN_60 * mechanics.delta_probe_radius, COS_60 * mechanics.delta_probe_radius);
      bed_level_c = probe.check_pt(0.0, 0.0);
    }

    void apply_endstop_adjustment(const float x_endstop, const float y_endstop, const float z_endstop) {
      mechanics.delta_endstop_adj[X_AXIS] += x_endstop;
      mechanics.delta_endstop_adj[Y_AXIS] += y_endstop;
      mechanics.delta_endstop_adj[Z_AXIS] += z_endstop;

      mechanics.Transform(mechanics.current_position);
      mechanics.set_position_mm(mechanics.delta[A_AXIS] - x_endstop , mechanics.delta[B_AXIS] - y_endstop, mechanics.delta[C_AXIS] - z_endstop, mechanics.current_position[E_AXIS]);  
      stepper.synchronize();
    }

    void adj_endstops() {
      bool x_done = false;
      bool y_done = false;
      bool z_done = false;

      do {
        bed_level_z = probe.check_pt(0.0, mechanics.delta_probe_radius);
        bed_level_x = probe.check_pt(-SIN_60 * mechanics.delta_probe_radius, -COS_60 * mechanics.delta_probe_radius);
        bed_level_y = probe.check_pt(SIN_60 * mechanics.delta_probe_radius, -COS_60 * mechanics.delta_probe_radius);

        apply_endstop_adjustment(bed_level_x, bed_level_y, bed_level_z);

        SERIAL_MV("x:", bed_level_x, 4);
        SERIAL_MV(" (adj:", mechanics.delta_endstop_adj[0], 4);
        SERIAL_MV(") y:", bed_level_y, 4);
        SERIAL_MV(" (adj:", mechanics.delta_endstop_adj[1], 4);
        SERIAL_MV(") z:", bed_level_z, 4);
        SERIAL_MV(" (adj:", mechanics.delta_endstop_adj[2], 4);
        SERIAL_CHR(')'); SERIAL_EOL();

        if (FABS(bed_level_x) <= ac_prec) {
          x_done = true;
          SERIAL_MSG("X=OK ");
        }
        else {
          x_done = false;
          SERIAL_MSG("X=ERROR ");
        }

        if (FABS(bed_level_y) <= ac_prec) {
          y_done = true;
          SERIAL_MSG("Y=OK ");
        }
        else {
          y_done = false;
          SERIAL_MSG("Y=ERROR ");
        }

        if (FABS(bed_level_z) <= ac_prec) {
          z_done = true;
          SERIAL_EM("Z=OK");
        }
        else {
          z_done = false;
          SERIAL_EM("Z=ERROR");
        }
      } while (((x_done == false) or (y_done == false) or (z_done == false)));

      const float high_endstop = MAX3(mechanics.delta_endstop_adj[A_AXIS], mechanics.delta_endstop_adj[B_AXIS], mechanics.delta_endstop_adj[C_AXIS]);

      SERIAL_EMV("High endstop:", high_endstop, 4);

      if (high_endstop > 0) {
        SERIAL_EMV("Reducing Build height by ", high_endstop);
        LOOP_XYZ(i) mechanics.delta_endstop_adj[i] -= high_endstop;
        mechanics.delta_height -= high_endstop;
      }

      mechanics.recalc_delta_settings();
    }

    int fix_tower_errors() {
      bool t1_err, t2_err, t3_err,
              xy_equal, xz_equal, yz_equal;
      float saved_tower_radius_adj[ABC],
            high_diff,
            x_diff, y_diff, z_diff,
            low_opp, high_opp;
      uint8_t err_tower = 0;

      COPY_ARRAY(saved_tower_radius_adj, mechanics.delta_tower_radius_adj);

      x_diff = FABS(bed_level_x - bed_level_ox);
      y_diff = FABS(bed_level_y - bed_level_oy);
      z_diff = FABS(bed_level_z - bed_level_oz);
      high_diff = MAX3(x_diff, y_diff, z_diff);

      if (x_diff <= ac_prec) t1_err = false; else t1_err = true;
      if (y_diff <= ac_prec) t2_err = false; else t2_err = true;
      if (z_diff <= ac_prec) t3_err = false; else t3_err = true;

      SERIAL_MV("x_diff:", x_diff, 5);
      SERIAL_MV(" y_diff:", y_diff, 5);
      SERIAL_MV(" z_diff:", z_diff, 5);
      SERIAL_EMV(" high_diff:", high_diff, 5);

      // Are all errors equal? (within defined precision)
      xy_equal = false;
      xz_equal = false;
      yz_equal = false;
      if (FABS(x_diff - y_diff) <= ac_prec) xy_equal = true;
      if (FABS(x_diff - z_diff) <= ac_prec) xz_equal = true;
      if (FABS(y_diff - z_diff) <= ac_prec) yz_equal = true;

      SERIAL_MSG("xy_equal = ");
      if (xy_equal == true) SERIAL_EM("true"); else SERIAL_EM("false");
      SERIAL_MSG("xz_equal = ");
      if (xz_equal == true) SERIAL_EM("true"); else SERIAL_EM("false");
      SERIAL_MSG("yz_equal = ");
      if (yz_equal == true) SERIAL_EM("true"); else SERIAL_EM("false");

      low_opp   = MIN3(bed_level_ox, bed_level_oy, bed_level_oz);
      high_opp  = MAX3(bed_level_ox, bed_level_oy, bed_level_oz);

      SERIAL_EMV("Opp Range = ", high_opp - low_opp, 5);

      if (high_opp - low_opp  < ac_prec) {
        SERIAL_EM("Opposite Points within Limits - Adjustment not required");
        t1_err = false;
        t2_err = false;
        t3_err = false;
      }

      // All Towers have errors
      if ((t1_err == true) and (t2_err == true) and (t3_err == true)) {
        if ((xy_equal == false) or (xz_equal == false) or (yz_equal == false)) {
          // Errors not equal .. select the tower that needs to be adjusted
          if (high_diff == x_diff) err_tower = 1;
          if (high_diff == y_diff) err_tower = 2;
          if (high_diff == z_diff) err_tower = 3;
          SERIAL_MV("Tower ", err_tower);
          SERIAL_EM(" has largest error");
        }
        if ((xy_equal == true) and (xz_equal == true) and (yz_equal == true)) {
          SERIAL_EM("All Towers Errors Equal");
          t1_err = false;
          t2_err = false;
          t3_err = false;
        }
      }

      /*
      // Two tower errors
      if ((t1_err == true) and (t2_err == true) and (t3_err == false)) {
        if (high_diff == x_diff) err_tower = 1;
        else err_tower = 2;
      }
      else if ((t1_err == true) and (t2_err == false) and (t3_err == true)) {
        if (high_diff == x_diff) err_tower = 1;
        else err_tower = 3;
      }
      else if ((t1_err == false) and (t2_err == true) and (t3_err == true)) {
        if (high_diff == y_diff) err_tower = 2;
        else err_tower = 3;
      }
      */

      // Single tower error
      if ((t1_err == true) and (t2_err == false) and (t3_err == false)) err_tower = 1;
      if ((t1_err == false) and (t2_err == true) and (t3_err == false)) err_tower = 2;
      if ((t1_err == false) and (t2_err == false) and (t3_err == true)) err_tower = 3;

      SERIAL_MSG("t1:");
      if (t1_err == true) SERIAL_MSG("Err"); else SERIAL_MSG("OK");
      SERIAL_MSG(" t2:");
      if (t2_err == true) SERIAL_MSG("Err"); else SERIAL_MSG("OK");
      SERIAL_MSG(" t3:");
      if (t3_err == true) SERIAL_MSG("Err"); else SERIAL_MSG("OK");
      SERIAL_EOL();

      if (err_tower == 0)
        SERIAL_EM("Tower geometry OK");
      else {
        SERIAL_MV("Tower", int(err_tower));
        SERIAL_EM(" Error: Adjusting");
        adj_tower_radius(err_tower);
      }

      // Set return value to indicate if anything has been changed (0 = no change)
      int retval = 0;
      LOOP_XYZ(i) if (saved_tower_radius_adj[i] != mechanics.delta_tower_radius_adj[i]) retval++;
      return retval;
    }

    bool adj_deltaradius() {
      bool adj_done;
      int adj_attempts;
      float adj_dRadius, adjdone_vector;

      bed_level_c = probe.check_pt(0.0, 0.0);

      if (FABS(bed_level_c) <= ac_prec) {
        SERIAL_EM("Delta Radius OK");
        return false;
      }
      else {
        SERIAL_EM("Adjusting Delta Radius");
        SERIAL_EMV("Bed level center = ", bed_level_c);

        // set initial direction and magnitude for delta radius adjustment
        adj_attempts = 0; 
        adj_dRadius = 0; 
        adjdone_vector = 0.01; 

        do {
          mechanics.delta_radius += adj_dRadius;
          mechanics.recalc_delta_settings();
          adj_done = false;

          adj_endstops();
          bed_level_c = probe.check_pt(0.0, 0.0);

          // Set inital adjustment value if it is currently 0
          if (adj_dRadius == 0) {
            if (bed_level_c > 0) adj_dRadius = -0.2;
            if (bed_level_c < 0) adj_dRadius = 0.2;
          }

          // Adjustment complete?
          if (FABS(bed_level_c) <= ac_prec) {
            //Done to within acprec .. but done within adjdone_vector? 
            if (FABS(bed_level_c) <= adjdone_vector)
              adj_done = true;
            else {
              adj_attempts ++;
              if (adj_attempts > 3) {
                adjdone_vector += 0.01;
                adj_attempts = 0;
              }
            }
          }

          // Show progress
          SERIAL_MV(" c:", bed_level_c, 4);
          SERIAL_MV(" delta radius:", mechanics.delta_radius, 4);
          SERIAL_MV(" prec:", adjdone_vector, 3);
          SERIAL_MV(" tries:", adj_attempts);
          SERIAL_MSG(" done:");
          if (adj_done == true) SERIAL_EM("true");
          else SERIAL_EM("false");

          // Overshot target? .. reverse and scale down adjustment
          if (((bed_level_c < 0) and (adj_dRadius < 0)) or ((bed_level_c > 0) and (adj_dRadius > 0))) adj_dRadius = -(adj_dRadius / 2);
  
        } while (adj_done == false);

        return true;
      }
    }

    void adj_tower_radius(uint8_t tower) {
      bool adj_done;
      float adj_tRadius = 0.0,
            bed_level   = 0.0,
            bed_level_o = 0.0;

      do {
        mechanics.delta_tower_radius_adj[tower - 1] += adj_tRadius;
        mechanics.recalc_delta_settings();
        adj_done = false;

        if (tower == 1) {
          // Bedlevel_x
          bed_level = probe.check_pt(-SIN_60 * mechanics.delta_probe_radius, -COS_60 * mechanics.delta_probe_radius);
          // Bedlevel_ox
          bed_level_o = probe.check_pt(SIN_60 * mechanics.delta_probe_radius, COS_60 * mechanics.delta_probe_radius);
        }
        if (tower == 2) {
          // Bedlevel_y
          bed_level = probe.check_pt(SIN_60 * mechanics.delta_probe_radius, -COS_60 * mechanics.delta_probe_radius);
          // Bedlevel_oy
          bed_level_o = probe.check_pt(-SIN_60 * mechanics.delta_probe_radius, COS_60 * mechanics.delta_probe_radius);
        }
        if (tower == 3) {
          // Bedlevel_z
          bed_level = probe.check_pt(0.0, mechanics.delta_probe_radius);
          // Bedlevel_oz
          bed_level_o = probe.check_pt(0.0, -mechanics.delta_probe_radius);
        }

        // Set inital adjustment value if it is currently 0
        if (adj_tRadius == 0) {
          if (bed_level_o < bed_level) adj_tRadius = -1;
          if (bed_level_o > bed_level) adj_tRadius = 1;
        }

        // Overshot target? .. reverse and scale down adjustment
        if (((bed_level_o < bed_level) and (adj_tRadius > 0)) or ((bed_level_o > bed_level) and (adj_tRadius < 0))) adj_tRadius = -(adj_tRadius / 2);

        // Adjustment complete?
        if (FABS(bed_level_o) < bed_level + 0.015) adj_done = true;

        // Show progress
        SERIAL_MV("tower:", bed_level, 4);
        SERIAL_MV(" opptower:", bed_level_o, 4);
        SERIAL_MV(" tower radius adj:", mechanics.delta_tower_radius_adj[tower - 1], 4);
        SERIAL_MSG(" done:");
        if (adj_done == true) SERIAL_EM("true");
        else SERIAL_EM("false");

        if (adj_done == false) adj_endstops();

      } while (adj_done == false);
    }

    void adj_tower_delta(uint8_t tower) {
      float adj_val = 0;
      float adj_mag = 0.2;
      float adj_prv;

      do {
        mechanics.delta_tower_pos_adj[tower - 1] += adj_val;
        mechanics.recalc_delta_settings();

        if ((tower == 1) or (tower == 3)) bed_level_oy = probe.check_pt(-SIN_60 * mechanics.delta_probe_radius, COS_60 * mechanics.delta_probe_radius);
        if ((tower == 1) or (tower == 2)) bed_level_oz = probe.check_pt(0.0, -mechanics.delta_probe_radius);
        if ((tower == 2) or (tower == 3)) bed_level_ox = probe.check_pt(SIN_60 * mechanics.delta_probe_radius, COS_60 * mechanics.delta_probe_radius);

        adj_prv = adj_val;
        adj_val = 0;

        if (tower == 1) {
          if (bed_level_oy < bed_level_oz) adj_val = adj_mag;
          if (bed_level_oy > bed_level_oz) adj_val = -adj_mag;
        }

        if (tower == 2) {
          if (bed_level_oz < bed_level_ox) adj_val = adj_mag;
          if (bed_level_oz > bed_level_ox) adj_val = -adj_mag;
        }

        if (tower == 3) {
          if (bed_level_ox < bed_level_oy) adj_val = adj_mag;
          if (bed_level_ox > bed_level_oy) adj_val = -adj_mag;
        }
           
        if ((adj_val > 0) and (adj_prv < 0)) {
          adj_mag = adj_mag / 2;
          adj_val = adj_mag;
        }

        if ((adj_val < 0) and (adj_prv > 0)) {
          adj_mag = adj_mag / 2;
          adj_val = -adj_mag;
        }

        // Show Adjustments made
        if (tower == 1) {
          SERIAL_MV("oy:", bed_level_oy, 4);
          SERIAL_MV(" oz:", bed_level_oz, 4);
        }

        if (tower == 2) {
          SERIAL_MV("ox:", bed_level_ox, 4);
          SERIAL_MV(" oz:", bed_level_oz, 4);
        }

        if (tower == 3) {
          SERIAL_MV("ox:", bed_level_ox, 4);
          SERIAL_MV(" oy:", bed_level_oy, 4);
        }

        SERIAL_EMV(" tower delta adj:", adj_val, 5);
      } while(adj_val != 0);
    }

    float adj_diagrod_length() {
      float adj_val = 0;
      float adj_mag = 0.2;
      float adj_prv, target;
      float prev_diag_rod = mechanics.delta_diagonal_rod;

      do {
        mechanics.delta_diagonal_rod += adj_val;
        mechanics.recalc_delta_settings();

        bed_level_oy = probe.check_pt(-SIN_60 * mechanics.delta_probe_radius, COS_60 * mechanics.delta_probe_radius);
        bed_level_oz = probe.check_pt(0.0, -mechanics.delta_probe_radius);
        bed_level_ox = probe.check_pt(SIN_60 * mechanics.delta_probe_radius, COS_60 * mechanics.delta_probe_radius);
        bed_level_c = probe.check_pt(0.0, 0.0);

        target = (bed_level_ox + bed_level_oy + bed_level_oz) / 3;
        adj_prv = adj_val;
        adj_val = 0;

        if (bed_level_c - 0.01 < target) adj_val = -adj_mag;
        if (bed_level_c + 0.01 > target) adj_val = adj_mag;

        if (((adj_val > 0) and (adj_prv < 0)) or ((adj_val < 0) and (adj_prv > 0))) {
          adj_val = adj_val / 2;
          adj_mag = adj_mag / 2;
        }

        if ((bed_level_c - 0.01 < target) and (bed_level_c + 0.01 > target)) adj_val = 0;

        // If adj magnatude is very small.. quit adjusting
        if ((abs(adj_val) < 0.001) and (adj_val != 0)) adj_val = 0;

        SERIAL_MV("target:", target, 4);
        SERIAL_MV(" c:", bed_level_c, 4);
        SERIAL_EMV(" adj:", adj_val, 5);
      } while(adj_val != 0);

      return (mechanics.delta_diagonal_rod - prev_diag_rod);
    }

    void calibration_report() {
      // Display Report
      SERIAL_EM("| \tZ-Tower\t\t\tEndstop Offsets");

      SERIAL_MSG("| \t");
      if (bed_level_z >= 0) SERIAL_MSG(" ");
      SERIAL_MV("", bed_level_z, 4);
      SERIAL_MV("\t\t\tX:", mechanics.delta_endstop_adj[0], 4);
      SERIAL_MV(" Y:", mechanics.delta_endstop_adj[1], 4);
      SERIAL_EMV(" Z:", mechanics.delta_endstop_adj[2], 4);

      SERIAL_MSG("| ");
      if (bed_level_ox >= 0) SERIAL_MSG(" ");
      SERIAL_MV("", bed_level_ox, 4);
      SERIAL_MSG("\t");
      if (bed_level_oy >= 0) SERIAL_MSG(" ");
      SERIAL_MV("", bed_level_oy, 4);
      SERIAL_EM("\t\tTower Offsets");

      SERIAL_MSG("| \t");
      if (bed_level_c >= 0) SERIAL_MSG(" ");
      SERIAL_MV("", bed_level_c, 4);
      SERIAL_MV("\t\t\tA:", mechanics.delta_tower_radius_adj[0]);
      SERIAL_MV(" B:", mechanics.delta_tower_radius_adj[1]);
      SERIAL_EMV(" C:", mechanics.delta_tower_radius_adj[2]);

      SERIAL_MSG("| ");
      if (bed_level_x >= 0) SERIAL_MSG(" ");
      SERIAL_MV("", bed_level_x, 4);
      SERIAL_MSG("\t");
      if (bed_level_y >= 0) SERIAL_MSG(" ");
      SERIAL_MV("", bed_level_y, 4);
      SERIAL_MV("\t\tI:", mechanics.delta_tower_pos_adj[0]);
      SERIAL_MV(" J:", mechanics.delta_tower_pos_adj[1]);
      SERIAL_EMV(" K:", mechanics.delta_tower_pos_adj[2]);

      SERIAL_MSG("| \t");
      if (bed_level_oz >= 0) SERIAL_MSG(" ");
      SERIAL_MV("", bed_level_oz, 4);
      SERIAL_EMV("\t\t\tDelta Radius: ", mechanics.delta_radius, 4);

      SERIAL_EMV("| X-Tower\tY-Tower\t\tDiagonal Rod: ", mechanics.delta_diagonal_rod, 4);
      SERIAL_EOL();
    }

  #endif

#endif // DELTA

void report_xyze(const float pos[XYZE], const uint8_t n = 4, const uint8_t precision = 3) {
  for (uint8_t i = 0; i < n; i++) {
    SERIAL_CHR(' ');
    SERIAL_CHR(axis_codes[i]);
    SERIAL_CHR(':');
    SERIAL_VAL(pos[i], precision);
  }
  SERIAL_EOL();
}

inline void report_xyz(const float pos[XYZ]) { report_xyze(pos, 3); }

void report_current_position_detail() {

  stepper.synchronize();

  SERIAL_MSG("\nLogical:");
  report_xyze(mechanics.current_position);

  SERIAL_MSG("Raw:    ");
  const float raw[XYZ] = { RAW_X_POSITION(mechanics.current_position[X_AXIS]), RAW_Y_POSITION(mechanics.current_position[Y_AXIS]), RAW_Z_POSITION(mechanics.current_position[Z_AXIS]) };
  report_xyz(raw);

  float leveled[XYZ] = { mechanics.current_position[X_AXIS], mechanics.current_position[Y_AXIS], mechanics.current_position[Z_AXIS] };

  #if HAS_LEVELING

    SERIAL_MSG("Leveled:");
    bedlevel.apply_leveling(leveled);
    report_xyz(leveled);

    SERIAL_MSG("UnLevel:");
    float unleveled[XYZ] = { leveled[X_AXIS], leveled[Y_AXIS], leveled[Z_AXIS] };
    bedlevel.unapply_leveling(unleveled);
    report_xyz(unleveled);

  #endif

  #if IS_KINEMATIC
    #if IS_SCARA
      SERIAL_MSG("ScaraK: ");
    #else
      SERIAL_MSG("DeltaK: ");
    #endif
    mechanics.Transform(leveled);  // writes delta[]
    report_xyz(mechanics.delta);
  #endif

  SERIAL_MSG("Stepper:");
  const long step_count[XYZE] = { stepper.position(X_AXIS), stepper.position(Y_AXIS), stepper.position(Z_AXIS), stepper.position(E_AXIS) };
  report_xyze((float*)step_count, 4, 0);

  #if IS_SCARA
    const float deg[XYZ] = {
      stepper.get_axis_position_degrees(A_AXIS),
      stepper.get_axis_position_degrees(B_AXIS)
    };
    SERIAL_MSG("Degrees:");
    report_xyze(deg, 2);
  #endif

  SERIAL_MSG("FromStp:");
  mechanics.get_cartesian_from_steppers();  // writes cartesian_position[XYZ] (with forward kinematics)
  const float from_steppers[XYZE] = { mechanics.cartesian_position[X_AXIS], mechanics.cartesian_position[Y_AXIS], mechanics.cartesian_position[Z_AXIS], mechanics.get_axis_position_mm(E_AXIS) };
  report_xyze(from_steppers);

  const float diff[XYZE] = {
    from_steppers[X_AXIS] - leveled[X_AXIS],
    from_steppers[Y_AXIS] - leveled[Y_AXIS],
    from_steppers[Z_AXIS] - leveled[Z_AXIS],
    from_steppers[E_AXIS] - mechanics.current_position[E_AXIS]
  };
  SERIAL_MSG("Differ: ");
  report_xyze(diff);
}

#if ENABLED(ARC_SUPPORT)

  #if N_ARC_CORRECTION < 1
    #undef N_ARC_CORRECTION
    #define N_ARC_CORRECTION 1
  #endif

  /**
   * Plan an arc in 2 dimensions
   *
   * The arc is approximated by generating many small linear segments.
   * The length of each segment is configured in MM_PER_ARC_SEGMENT (Default 1mm)
   * Arcs should only be made relatively large (over 5mm), as larger arcs with
   * larger segments will tend to be more efficient. Your slicer should have
   * options for G2/G3 arc generation. In future these options may be GCode tunable.
   */
  void plan_arc(
    float logical[XYZE],  // Destination position
    float *offset,        // Center of rotation relative to current_position
    uint8_t clockwise     // Clockwise?
  ) {

    #if ENABLED(CNC_WORKSPACE_PLANES)
      AxisEnum p_axis, q_axis, l_axis;
      switch (workspace_plane) {
        case PLANE_XY: p_axis = X_AXIS; q_axis = Y_AXIS; l_axis = Z_AXIS; break;
        case PLANE_ZX: p_axis = Z_AXIS; q_axis = X_AXIS; l_axis = Y_AXIS; break;
        case PLANE_YZ: p_axis = Y_AXIS; q_axis = Z_AXIS; l_axis = X_AXIS; break;
      }
    #else
      constexpr AxisEnum p_axis = X_AXIS, q_axis = Y_AXIS, l_axis = Z_AXIS;
    #endif

    // Radius vector from center to current location
    float r_P = -offset[0], r_Q = -offset[1];

    const float radius = HYPOT(r_P, r_Q),
                center_P = mechanics.current_position[p_axis] - r_P,
                center_Q = mechanics.current_position[q_axis] - r_Q,
                rt_X = logical[p_axis] - center_P,
                rt_Y = logical[q_axis] - center_Q,
                linear_travel = logical[l_axis] - mechanics.current_position[l_axis],
                extruder_travel = logical[E_AXIS] - mechanics.current_position[E_AXIS];

    // CCW angle of rotation between position and target from the circle center. Only one atan2() trig computation required.
    float angular_travel = ATAN2(r_P * rt_Y - r_Q * rt_X, r_P * rt_X + r_Q * rt_Y);
    if (angular_travel < 0) angular_travel += RADIANS(360);
    if (clockwise) angular_travel -= RADIANS(360);

    // Make a circle if the angular rotation is 0
    if (angular_travel == 0 && mechanics.current_position[p_axis] == logical[p_axis] && mechanics.current_position[q_axis] == logical[q_axis])
      angular_travel += RADIANS(360);

    float mm_of_travel = HYPOT(angular_travel * radius, FABS(linear_travel));
    if (mm_of_travel < 0.001) return;

    uint16_t segments = FLOOR(mm_of_travel / (MM_PER_ARC_SEGMENT));
    if (segments == 0) segments = 1;
    
    /**
     * Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
     * and phi is the angle of rotation. Based on the solution approach by Jens Geisler.
     *     r_T = [cos(phi) -sin(phi);
     *            sin(phi)  cos(phi] * r ;
     *
     * For arc generation, the center of the circle is the axis of rotation and the radius vector is
     * defined from the circle center to the initial position. Each line segment is formed by successive
     * vector rotations. This requires only two cos() and sin() computations to form the rotation
     * matrix for the duration of the entire arc. Error may accumulate from numerical round-off, since
     * all double numbers are single precision on the Arduino. (True double precision will not have
     * round off issues for CNC applications.) Single precision error can accumulate to be greater than
     * tool precision in some cases. Therefore, arc path correction is implemented.
     *
     * Small angle approximation may be used to reduce computation overhead further. This approximation
     * holds for everything, but very small circles and large MM_PER_ARC_SEGMENT values. In other words,
     * theta_per_segment would need to be greater than 0.1 rad and N_ARC_CORRECTION would need to be large
     * to cause an appreciable drift error. N_ARC_CORRECTION~=25 is more than small enough to correct for
     * numerical drift error. N_ARC_CORRECTION may be on the order a hundred(s) before error becomes an
     * issue for CNC machines with the single precision Arduino calculations.
     *
     * This approximation also allows plan_arc to immediately insert a line segment into the planner
     * without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
     * a correction, the planner should have caught up to the lag caused by the initial plan_arc overhead.
     * This is important when there are successive arc motions.
     */
    // Vector rotation matrix values
    float arc_target[XYZE];
    const float theta_per_segment = angular_travel / segments,
                linear_per_segment = linear_travel / segments,
                extruder_per_segment = extruder_travel / segments,
                sin_T = theta_per_segment,
                cos_T = 1 - 0.5 * sq(theta_per_segment); // Small angle approximation

    // Initialize the linear axis
    arc_target[l_axis] = mechanics.current_position[l_axis];

    // Initialize the extruder axis
    arc_target[E_AXIS] = mechanics.current_position[E_AXIS];

    const float fr_mm_s = MMS_SCALED(mechanics.feedrate_mm_s);

    millis_t next_idle_ms = millis() + 200UL;

    #if N_ARC_CORRECTION > 1
      int8_t count = N_ARC_CORRECTION;
    #endif

    for (uint16_t i = 1; i < segments; i++) { // Iterate (segments-1) times

      thermalManager.manage_temp_controller();
      if (ELAPSED(millis(), next_idle_ms)) {
        next_idle_ms = millis() + 200UL;
        idle();
      }

      #if N_ARC_CORRECTION > 1
        if (--count) {
          // Apply vector rotation matrix to previous r_P / 1
          const float r_new_Y = r_P * sin_T + r_Q * cos_T;
          r_P = r_P * cos_T - r_Q * sin_T;
          r_Q = r_new_Y;
        }
        else
      #endif
      {
        #if N_ARC_CORRECTION > 1
          count = N_ARC_CORRECTION;
        #endif

        // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
        // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
        // To reduce stuttering, the sin and cos could be computed at different times.
        // For now, compute both at the same time.
        const float cos_Ti = cos(i * theta_per_segment),
                    sin_Ti = sin(i * theta_per_segment);
        r_P = -offset[0] * cos_Ti + offset[1] * sin_Ti;
        r_Q = -offset[0] * sin_Ti - offset[1] * cos_Ti;
      }

      // Update arc_target location
      arc_target[p_axis] = center_P + r_P;
      arc_target[q_axis] = center_Q + r_Q;
      arc_target[l_axis] += linear_per_segment;
      arc_target[E_AXIS] += extruder_per_segment;

      endstops.clamp_to_software_endstops(arc_target);

      planner.buffer_line_kinematic(arc_target, fr_mm_s, active_extruder);
    }

    // Ensure last segment arrives at target location.
    planner.buffer_line_kinematic(logical, fr_mm_s, active_extruder);

    // As far as the parser is concerned, the position is now == target. In reality the
    // motion control system might still be processing the action and the real tool position
    // in any intermediate location.
    mechanics.set_current_to_destination();
  }

#endif

#if HAS_CONTROLLERFAN

  void controllerFan() {
    static millis_t lastMotorOn = 0,    // Last time a motor was turned on
                    nextMotorCheck = 0; // Last time the state was checked
    millis_t ms = millis();
    if (ELAPSED(ms, nextMotorCheck)) {
      nextMotorCheck = ms + 2500UL; // Not a time critical function, so only check every 2.5s
      if (X_ENABLE_READ == X_ENABLE_ON || Y_ENABLE_READ == Y_ENABLE_ON || Z_ENABLE_READ == Z_ENABLE_ON || thermalManager.soft_pwm_bed > 0
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

#if MECH(MORGAN_SCARA)

  /**
   * Morgan SCARA Forward mechanics. Results in cartesian_position[].
   * Maths and first version by QHARLEY.
   * Integrated and slightly restructured by Joachim Cerny.
   */
  void forward_kinematics_SCARA(const float &a, const float &b) {

    const float a_sin = sin(RADIANS(a)) * L1,
                a_cos = cos(RADIANS(a)) * L1,
                b_sin = sin(RADIANS(b)) * L2,
                b_cos = cos(RADIANS(b)) * L2;

    cartesian_position[X_AXIS] = a_cos + b_cos + SCARA_OFFSET_X;  //theta
    cartesian_position[Y_AXIS] = a_sin + b_sin + SCARA_OFFSET_Y;  //theta+phi

      //SERIAL_MV(" cartesian_position[X_AXIS]=", cartesian_position[X_AXIS]);
      //SERIAL_EMV(" cartesian_position[Y_AXIS]=", cartesian_position[Y_AXIS]);
  }

  /**
   * Morgan SCARA Inverse mechanics. Results in delta[].
   *
   * See http://forums.reprap.org/read.php?185,283327
   * 
   * Maths and first version by QHARLEY.
   * Integrated and slightly restructured by Joachim Cerny.
   */
  void inverse_kinematics(const float logical[XYZ]) {

    static float C2, S2, SK1, SK2, THETA, PSI; 

    const float sx = RAW_X_POSITION(logical[X_AXIS]) - SCARA_offset_x,  // Translate SCARA to standard X Y
                sy = RAW_Y_POSITION(logical[Y_AXIS]) - SCARA_offset_y;  // With scaling factor.

    if (L1 == L2)
      C2 = HYPOT2(sx, sy) / L1_2_2 - 1;
    else
      C2 = (HYPOT2(sx, sy) - (L1_2 + L2_2)) / (2.0 * L1 * L2);

    S2 = SQRT(1 - sq(C2));

    // Unrotated Arm1 plus rotated Arm2 gives the distance from Center to End
    SK1 = L1 + L2 * C2;

    // Rotated Arm2 gives the distance from Arm1 to Arm2
    SK2 = L2 * S2;

    // Angle of Arm1 is the difference between Center-to-End angle and the Center-to-Elbow
    THETA = ATAN2(SK1, SK2) - ATAN2(sx, sy);

    // Angle of Arm2
    PSI = ATAN2(S2, C2);

    delta[A_AXIS] = DEGREES(THETA);        // theta is support arm angle
    delta[B_AXIS] = DEGREES(THETA + PSI);  // equal to sub arm angle (inverted motor)
    delta[C_AXIS] = logical[Z_AXIS];

    /*
    DEBUG_POS("SCARA IK", logical);
    DEBUG_POS("SCARA IK", delta);
    SERIAL_MV("  SCARA (x,y) ", sx);
    SERIAL_MV(",", sy);
    SERIAL_MV(" C2=", C2);
    SERIAL_MV(" S2=", S2);
    SERIAL_MV(" Theta=", THETA);
    SERIAL_EMV(" Psi=", PSI);
    */
  }

#endif // MORGAN_SCARA

#if ENABLED(TEMP_STAT_LEDS)

  static bool red_led = false;
  static millis_t next_status_led_update_ms = 0;

  void handle_status_leds(void) {
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

void setInterruptEvent(const MK4duoInterruptEvent event) {
  if (interruptEvent == INTERRUPT_EVENT_NONE)
    interruptEvent = event;
}

void handle_Interrupt_Event() {

  if (interruptEvent == INTERRUPT_EVENT_NONE) return; // Exit if none Event

  const MK4duoInterruptEvent event = interruptEvent;
  interruptEvent = INTERRUPT_EVENT_NONE;

  switch(event) {
    #if HAS_FIL_RUNOUT || HAS_DAV_SYSTEM
      //case INTERRUPT_EVENT_FIL_RUNOUT:
      case INTERRUPT_EVENT_DAV_SYSTEM:
        if (!filament_ran_out) {
          filament_ran_out = true;
          enqueue_and_echo_commands_P(PSTR(FILAMENT_RUNOUT_SCRIPT));
          SERIAL_LM(REQUEST_PAUSE, "End Filament detect");
          stepper.synchronize();
        }
        break;
    #endif

    #if HAS_EXT_ENCODER
      case INTERRUPT_EVENT_ENC_DETECT:
        if (!filament_ran_out) {
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

void quickstop_stepper() {
  stepper.quick_stop();
  stepper.synchronize();
  mechanics.set_current_from_steppers_for_axis(ALL_AXES);
  mechanics.sync_plan_position();
}

float calculate_volumetric_multiplier(float diameter) {
  if (!volumetric_enabled || diameter == 0) return 1.0;
  float d2 = diameter * 0.5;
  return 1.0 / (M_PI * d2 * d2);
}

void calculate_volumetric_multipliers() {
  for (uint8_t e = 0; e < EXTRUDERS; e++)
    volumetric_multiplier[e] = calculate_volumetric_multiplier(filament_size[e]);
}

#if ENABLED(HAVE_TMC2130)

  void automatic_current_control(TMC2130Stepper &st, String axisID) {
    // Check otpw even if we don't use automatic control. Allows for flag inspection.
    const bool is_otpw = st.checkOT();

    // Report if a warning was triggered
    static bool previous_otpw = false;
    if (is_otpw && !previous_otpw) {
      char timestamp[10];
      duration_t elapsed = print_job_counter.duration();
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

  void checkOverTemp() {
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
void manage_inactivity(bool ignore_stepper_queue/*=false*/) {

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

  if (commands_in_queue < BUFSIZE) get_available_commands();

  const millis_t ms = millis();

  if (max_inactive_time && ELAPSED(ms, previous_cmd_ms + max_inactive_time)) {
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

  if (MOVE_AWAY_TEST && stepper_inactive_time && ELAPSED(ms, previous_cmd_ms + stepper_inactive_time)
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
        enqueue_and_echo_commands_P(PSTR("G28"));
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
    if (ELAPSED(ms, previous_cmd_ms + (EXTRUDER_RUNOUT_SECONDS) * 1000UL)
      && thermalManager.degHotend(active_extruder) > EXTRUDER_RUNOUT_MINTEMP) {
      bool oldstatus;
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

      previous_cmd_ms = ms; // refresh_cmd_timeout()

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
    if (mechanics.delayed_move_time && ELAPSED(ms, mechanics.delayed_move_time + 1000UL) && IsRunning()) {
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

/**
 * Standard idle routine keeps the machine alive
 */
void idle(
  #if ENABLED(ADVANCED_PAUSE_FEATURE) || ENABLED(CNCROUTER)
    bool no_stepper_sleep/*=false*/
  #endif
) {

  static uint8_t cycle_1500ms = 15;

  /**
   * Start event periodical
   */

  #if ENABLED(NEXTION)
    lcd_key_touch_update();
  #else
    lcd_update();
  #endif

  host_keepalive();

  #if ENABLED(AUTO_REPORT_TEMPERATURES) && (HAS_TEMP_HOTEND || HAS_TEMP_BED)
    auto_report_temperatures();
  #endif

  #if ENABLED(FLOWMETER_SENSOR)
    flowrate_manage();
  #endif

  #if ENABLED(CNCROUTER)
    cnc_manage();
  #endif

  manage_inactivity(
    #if ENABLED(ADVANCED_PAUSE_FEATURE) || ENABLED(CNCROUTER)
      no_stepper_sleep
    #endif
  );

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
 * Kill all activity and lock the machine.
 * After this the machine will need to be reset.
 */
void kill(const char* lcd_msg) {
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
void Stop() {
  #if ENABLED(FLOWMETER_SENSOR) && ENABLED(MINFLOW_PROTECTION)
    flow_firstread = false;
  #endif

  thermalManager.disable_all_heaters();
  thermalManager.disable_all_coolers();

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

  if (IsRunning()) {
    Running = false;
    Stopped_gcode_LastN = gcode_LastN; // Save last g_code for restart
    SERIAL_LM(ER, MSG_ERR_STOPPED);
    SERIAL_STR(PAUSE);
    SERIAL_EOL();
    LCD_MESSAGEPGM(MSG_STOPPED);
  }
}

/**
 * MK4duo entry-point: Set up before the program loop
 *  - Set up Alligator Board
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
void setup() {

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
  for (int8_t i = 0; i < BUFSIZE; i++) send_ok[i] = true;

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
    (void)eeprom.Load_Settings();
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
  servo_init();

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

/**
 * The main MK4duo program loop
 *
 *  - Save or log commands to SD
 *  - Process available commands (if not saving)
 *  - Call heater manager
 *  - Call inactivity manager
 *  - Call endstop manager
 *  - Call LCD update
 */
void loop() {

  if (commands_in_queue < BUFSIZE) get_available_commands();

  #if HAS_EEPROM_SD
    static uint8_t wait_for_host_init_string_to_finish = 1;
    if (wait_for_host_init_string_to_finish) {
      if (commands_in_queue != 0 && wait_for_host_init_string_to_finish == 1) wait_for_host_init_string_to_finish = 2;
      if (commands_in_queue == 0 && wait_for_host_init_string_to_finish >= 2) wait_for_host_init_string_to_finish++;
      if (wait_for_host_init_string_to_finish >= 250) {
        wait_for_host_init_string_to_finish = 0;
        // loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
        eeprom.Load_Settings();
      }
    }
  #endif

  #if HAS_SDSUPPORT
    card.checkautostart(false);
  #endif

  if (commands_in_queue) {

    #if HAS_SDSUPPORT

      if (card.saving) {
        char* command = command_queue[cmd_queue_index_r];
        if (strstr_P(command, PSTR("M29"))) {
          // M29 closes the file
          card.finishWrite();
          ok_to_send();
        }
        else {
          // Write the string from the read buffer to SD
          card.write_command(command);
          ok_to_send();
        }
      }
      else
        process_next_command();

    #else

      process_next_command();

    #endif // SDSUPPORT

    // The queue may be reset by a command handler or by code invoked by idle() within a handler
    if (commands_in_queue) {
      --commands_in_queue;
      if (++cmd_queue_index_r >= BUFSIZE) cmd_queue_index_r = 0;
    }
  }
  endstops.report_state();
  idle();
}
