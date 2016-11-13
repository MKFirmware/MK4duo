/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2016 Alberto Cotronei @MagoKimbra
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

#if ENABLED(RFID_MODULE)
  MFRC522 RFID522;
#endif

#if ENABLED(M100_FREE_MEMORY_WATCHER)
  void gcode_M100();
#endif

#if ENABLED(SDSUPPORT)
  CardReader card;
#endif

#if ENABLED(FLOWMETER_SENSOR) && ENABLED(MINFLOW_PROTECTION)
  bool flow_firstread = false;
#endif

#if ENABLED(PINS_DEBUGGING)
  bool endstop_monitor_flag = false;
#endif

bool Running = true;

// Print status related
long  currentLayer = 0,
      maxLayer = -1; // -1 = unknown
char  printName[21] = ""; // max. 20 chars + 0
float progress = 0.0;

uint8_t mk_debug_flags = DEBUG_NONE;

/**
 * Cartesian Current Position
 *   Used to track the logical position as moves are queued.
 *   Used by 'line_to_current_position' to do a move after changing it.
 *   Used by 'SYNC_PLAN_POSITION_KINEMATIC' to update 'planner.position'.
 */
float current_position[XYZE] = { 0.0 };

/**
 * Cartesian Destination
 *   A temporary position, usually applied to 'current_position'.
 *   Set with 'gcode_get_destination' or 'set_destination_to_current'.
 *   'line_to_destination' sets 'current_position' to 'destination'.
 */
float destination[XYZE] = { 0.0 };

/**
 * axis_homed
 *   Flags that each linear axis was homed.
 *   XYZ on cartesian, ABC on delta, ABZ on SCARA.
 *
 * axis_known_position
 *   Flags that the position is known in each linear axis. Set when homed.
 *   Cleared whenever a stepper powers off, potentially losing its position.
 */
bool axis_homed[XYZ] = { false }, axis_known_position[XYZ] = { false };

/**
 * GCode line number handling. Hosts may opt to include line numbers when
 * sending commands to Marlin, and lines will be checked for sequentiality.
 * M110 S<int> sets the current line number.
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
static char command_queue[BUFSIZE][MAX_CMD_SIZE];
static uint8_t  cmd_queue_index_r = 0,  // Ring buffer read position
                cmd_queue_index_w = 0,  // Ring buffer write position
                commands_in_queue = 0;  // Count of commands in the queue

/**
 * Current GCode Command
 * When a GCode handler is running, these will be set
 */
static char *current_command,      // The command currently being executed
            *current_command_args, // The address where arguments begin
            *seen_pointer;         // Set by code_seen(), used by the code_value functions

/**
 * Next Injected Command pointer. NULL if no commands are being injected.
 * Used by Marlin internally to ensure that commands initiated from within
 * are enqueued ahead of any pending serial or sd card commands.
 */
static const char *injected_commands_P = NULL;

bool pos_saved = false;
float stored_position[NUM_POSITON_SLOTS][NUM_AXIS];

#if ENABLED(INCH_MODE_SUPPORT)
  float linear_unit_factor = 1.0, volumetric_unit_factor = 1.0;
#endif

#if ENABLED(TEMPERATURE_UNITS_SUPPORT)
  TempUnit input_temp_units = TEMPUNIT_C;
#endif

/**
 * Feed rates are often configured with mm/m
 * but the planner and stepper like mm/s units.
 */
float constexpr homing_feedrate_mm_s[] = {
  #if MECH(DELTA)
    MMM_TO_MMS(HOMING_FEEDRATE_XYZ), MMM_TO_MMS(HOMING_FEEDRATE_XYZ), MMM_TO_MMS(HOMING_FEEDRATE_XYZ)
  #else
    MMM_TO_MMS(HOMING_FEEDRATE_X), MMM_TO_MMS(HOMING_FEEDRATE_Y), MMM_TO_MMS(HOMING_FEEDRATE_Z)
  #endif
};
static float feedrate_mm_s = MMM_TO_MMS(1500.0), saved_feedrate_mm_s;
int feedrate_percentage = 100, saved_feedrate_percentage,
    flow_percentage[EXTRUDERS] = ARRAY_BY_EXTRUDERS(100),
    density_percentage[EXTRUDERS] = ARRAY_BY_EXTRUDERS(100);

bool axis_relative_modes[] = AXIS_RELATIVE_MODES,
     volumetric_enabled = false;

float filament_size[EXTRUDERS] = ARRAY_BY_EXTRUDERS(DEFAULT_NOMINAL_FILAMENT_DIA),
      volumetric_multiplier[EXTRUDERS] = ARRAY_BY_EXTRUDERS(1.0);

// The distance that XYZ has been offset by G92. Reset by G28.
float position_shift[XYZ] = { 0 };

// This offset is added to the configured home position.
// Set by M206, M428, or menu item. Saved to EEPROM.
float home_offset[XYZ] = { 0 };

// Software Endstops. Default to configured limits.
#if ENABLED(SOFTWARE_MIN_ENDSTOPS) || ENABLED(SOFTWARE_MAX_ENDSTOPS)
  bool soft_endstops_enabled = true;
#endif
float soft_endstop_min[XYZ] = { X_MIN_POS, Y_MIN_POS, Z_MIN_POS },
      soft_endstop_max[XYZ] = { X_MAX_POS, Y_MAX_POS, Z_MAX_POS };

int fanSpeed = 0;

float hotend_offset[XYZ][HOTENDS];

// The active extruder (tool). Set with T<extruder> command.
uint8_t active_extruder = 0;
uint8_t previous_extruder = 0;
uint8_t active_driver = 0;

static uint8_t target_extruder;

// Relative Mode. Enable with G91, disable with G90.
static bool relative_mode = false;

// For M109 and M190, this flag may be cleared (by M108) to exit the wait loop
volatile bool wait_for_heatup = true;

// For M0/M1, this flag may be cleared (by M108) to exit the wait-for-user loop
#if HAS(LCD)
  volatile bool wait_for_user = false;
#endif

const char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};

static bool home_all_axis = true;

// Number of characters read in the current line of serial input
static int serial_count = 0;

// Inactivity shutdown
millis_t previous_cmd_ms = 0;
static millis_t max_inactive_time = 0;
static millis_t stepper_inactive_time = (DEFAULT_STEPPER_DEACTIVE_TIME) * 1000UL;

// Print Job Timer
PrintCounter print_job_counter = PrintCounter();

#if HAS(BED_PROBE)
  float zprobe_zoffset = Z_PROBE_OFFSET_FROM_NOZZLE;
#endif

#define PLANNER_XY_FEEDRATE() (min(planner.max_feedrate_mm_s[X_AXIS], planner.max_feedrate_mm_s[Y_AXIS]))

#if HAS(ABL)
  int xy_probe_feedrate_mm_s = MMM_TO_MMS(XY_PROBE_SPEED);
  #define XY_PROBE_FEEDRATE_MM_S xy_probe_feedrate_mm_s
#elif defined(XY_PROBE_SPEED)
  #define XY_PROBE_FEEDRATE_MM_S MMM_TO_MMS(XY_PROBE_SPEED)
#else
  #define XY_PROBE_FEEDRATE_MM_S PLANNER_XY_FEEDRATE()
#endif

#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
  #if MECH(DELTA)
    #define ADJUST_DELTA(V) \
      if (planner.abl_enabled) { \
        const float zadj = bilinear_z_offset(V); \
        delta[A_AXIS] += zadj; \
        delta[B_AXIS] += zadj; \
        delta[C_AXIS] += zadj; \
      }
  #else
    #define ADJUST_DELTA(V) if (planner.abl_enabled) { delta[Z_AXIS] += bilinear_z_offset(V); }
  #endif
#endif

#if ENABLED(Z_DUAL_ENDSTOPS)
  float z_endstop_adj = 0;
#endif

#if HEATER_USES_AD595
  float ad595_offset[HOTENDS] = ARRAY_BY_HOTENDS(TEMP_SENSOR_AD595_OFFSET);
  float ad595_gain[HOTENDS] = ARRAY_BY_HOTENDS(TEMP_SENSOR_AD595_GAIN);
#endif

#if ENABLED(NPR2)
  uint8_t old_color = 99;
#endif

#if ENABLED(RFID_MODULE)
  bool RFID_ON = false;
  unsigned long Spool_ID[EXTRUDERS] = ARRAY_BY_EXTRUDERS(0);
  bool Spool_must_read[EXTRUDERS]   = ARRAY_BY_EXTRUDERS(false);
  bool Spool_must_write[EXTRUDERS]  = ARRAY_BY_EXTRUDERS(false);
#endif

#if HAS(Z_SERVO_ENDSTOP)
  const int z_servo_angle[2] = Z_ENDSTOP_SERVO_ANGLES;
#endif

#if ENABLED(BARICUDA)
  int baricuda_valve_pressure = 0;
  int baricuda_e_to_p_pressure = 0;
#endif

#if ENABLED(FWRETRACT)

  bool autoretract_enabled = false;
  bool retracted[EXTRUDERS] = { false };
  bool retracted_swap[EXTRUDERS] = { false };

  float retract_length = RETRACT_LENGTH;
  float retract_length_swap = RETRACT_LENGTH_SWAP;
  float retract_feedrate_mm_s = RETRACT_FEEDRATE;
  float retract_zlift = RETRACT_ZLIFT;
  float retract_recover_length = RETRACT_RECOVER_LENGTH;
  float retract_recover_length_swap = RETRACT_RECOVER_LENGTH_SWAP;
  float retract_recover_feedrate_mm_s = RETRACT_RECOVER_FEEDRATE;

#endif // FWRETRACT

#if HAS(POWER_SWITCH)
  bool powersupply = 
    #if ENABLED(PS_DEFAULT_OFF)
      false
    #else
      true
    #endif
  ;
#endif

#if MECH(DELTA)
  float delta[ABC],
        endstop_adj[ABC] = { 0 };

  // these are the default values, can be overriden with M666
  float delta_radius,
        diagrod_adj[ABC] = { 0 },
        tower_adj[6] = { 0 },
        delta_diagonal_rod,
        delta_tmp[ABC] = { 0.0 },
        delta_tower1_x,
        delta_tower1_y,
        delta_tower2_x,
        delta_tower2_y,
        delta_tower3_x,
        delta_tower3_y,
        base_max_pos[ABC] = {X_MAX_POS, Y_MAX_POS, Z_MAX_POS},
        base_home_pos[ABC] = {X_HOME_POS, Y_HOME_POS, Z_HOME_POS},
        max_length[ABC] = {X_MAX_LENGTH, Y_MAX_LENGTH, Z_MAX_LENGTH},
        delta_diagonal_rod_2_tower_1,
        delta_diagonal_rod_2_tower_2,
        delta_diagonal_rod_2_tower_3,
        delta_clip_start_height = Z_MAX_POS,
        delta_segments_per_second;

  const float z_probe_deploy_start_location[] = Z_PROBE_DEPLOY_START_LOCATION,
              z_probe_deploy_end_location[] = Z_PROBE_DEPLOY_END_LOCATION,
              z_probe_retract_start_location[] = Z_PROBE_RETRACT_START_LOCATION,
              z_probe_retract_end_location[] = Z_PROBE_RETRACT_END_LOCATION;

  float delta_safe_distance_from_top();

  #if ENABLED(AUTO_CALIBRATION_FEATURE)
    const float bed_radius = DELTA_PROBEABLE_RADIUS;
    float ac_prec = AUTOCALIBRATION_PRECISION,
          bed_level_c,
          bed_level_x,
          bed_level_y,
          bed_level_z,
          bed_level_ox,
          bed_level_oy,
          bed_level_oz,
          bed_safe_z,
          adj_t1_Radius = 0,
          adj_t2_Radius = 0,
          adj_t3_Radius = 0,
          probe_bed(float x, float y),
          adj_diagrod_length();

    int fix_tower_errors();
    bool adj_deltaradius();

    void  adj_tower_delta(uint8_t tower);
    void  adj_tower_radius(uint8_t tower);
    void  home_delta();
    void  calibration_report();
    void  bed_probe_all();
    void  adj_endstops();
  #endif
#endif

#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
  int bilinear_grid_spacing[2] = { 0 }, bilinear_start[2] = { 0 };
  float bilinear_level_grid[ABL_GRID_POINTS_X][ABL_GRID_POINTS_Y];
#endif

#if IS_SCARA
  // Float constants for SCARA calculations
  const float L1 = SCARA_LINKAGE_1, L2 = SCARA_LINKAGE_2,
              L1_2 = sq(float(L1)), L1_2_2 = 2.0 * L1_2,
              L2_2 = sq(float(L2));

  float delta_segments_per_second = SCARA_SEGMENTS_PER_SECOND,
        delta[ABC];
  static void plan_direct_stepper_move(const float target[XYZE]);
#endif

float cartes[XYZ] = { 0 };

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

#if ENABLED(FILAMENT_SENSOR)
  bool filament_sensor = false;                                 // M405 turns on filament_sensor control, M406 turns it off 
  float filament_width_nominal = DEFAULT_NOMINAL_FILAMENT_DIA,  // Nominal filament width. Change with M404
        filament_width_meas = DEFAULT_MEASURED_FILAMENT_DIA;    // Measured filament diameter
  int8_t measurement_delay[MAX_MEASUREMENT_DELAY + 1];          // Ring buffer to delayed measurement. Store extruder factor after subtracting 100
  int filwidth_delay_index[2] = { 0, -1 };                      // Indexes into ring buffer
  int meas_delay_cm = MEASUREMENT_DELAY_CM;                     // Distance delay setting
#endif

#if HAS(FIL_RUNOUT)
  static bool filament_ran_out = false;
#endif

#if ENABLED(FILAMENT_CHANGE_FEATURE)
  FilamentChangeMenuResponse filament_change_menu_response;
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

#if HAS(POWER_CONSUMPTION_SENSOR)
  float power_consumption_meas = 0.0;
  unsigned long power_consumption_hour;
  unsigned long startpower = 0;
  unsigned long stoppower = 0;
#endif

#if ENABLED(NPR2)
  static float color_position[] = COLOR_STEP;
  static float color_step_moltiplicator = (DRIVER_MICROSTEP / MOTOR_ANGLE) * CARTER_MOLTIPLICATOR;
#endif // NPR2

#if ENABLED(EASY_LOAD)
  bool allow_lengthy_extrude_once; // for load/unload
#endif

static bool send_ok[BUFSIZE];

#if HAS(SERVOS)
  Servo servo[NUM_SERVOS];
  #define MOVE_SERVO(I, P) servo[I].move(P)
  #define DEPLOY_Z_SERVO() MOVE_SERVO(Z_ENDSTOP_SERVO_NR, z_servo_angle[0])
  #define STOW_Z_SERVO()   MOVE_SERVO(Z_ENDSTOP_SERVO_NR, z_servo_angle[1])
#endif

#if HAS(CHDK)
  millis_t chdkHigh = 0;
  boolean chdkActive = false;
#endif

#if ENABLED(PIDTEMP) && ENABLED(PID_ADD_EXTRUSION_RATE)
  int lpq_len = 20;
#endif

#if ENABLED(HOST_KEEPALIVE_FEATURE)
  // States for managing MK and host communication
  // MK sends messages if blocked or busy
  static MKBusyState busy_state = NOT_BUSY;
  static millis_t next_busy_signal_ms = 0;
  uint8_t host_keepalive_interval = DEFAULT_KEEPALIVE_INTERVAL;
  #define KEEPALIVE_STATE(n) do{ busy_state = n; }while(0)
#else
  #define host_keepalive() ;
  #define KEEPALIVE_STATE(n) ;
#endif // HOST_KEEPALIVE_FEATURE

#if ENABLED(M100_FREE_MEMORY_WATCHER)
  // top_of_stack() returns the location of a variable on its stack frame.  The value returned is above
  // the stack once the function returns to the caller.

  unsigned char* top_of_stack() {
    unsigned char x;
    return &x + 1; // x is pulled on return;
  }

  //
  // 3 support routines to print hex numbers.  We can print a nibble, byte and word
  //
  void prt_hex_nibble( unsigned int n ) {
    if ( n <= 9 )
      SERIAL_V(n);
    else
      SERIAL_V((char)('A' + n - 10));
    HAL::delayMilliseconds(2);
  }

  void prt_hex_byte(unsigned int b) {
    prt_hex_nibble(( b & 0xf0) >> 4);
    prt_hex_nibble(b & 0x0f);
  }

  void prt_hex_word(unsigned int w) {
    prt_hex_byte((w & 0xff00) >> 8);
    prt_hex_byte(w & 0x0ff);
  }

  // how_many_E5s_are_here() is a utility function to easily find out how many 0xE5's are
  // at the specified location. Having this logic as a function simplifies the search code.
  //
  int how_many_E5s_are_here( unsigned char* p) {
    int n;
    for (n = 0; n < 32000; n++) {
      if (*(p + n) != (unsigned char) 0xe5)
        return n - 1;
    }
    return -1;
  }
#endif

#define DEFINE_PGM_READ_ANY(type, reader)       \
  static inline type pgm_read_any(const type *p)  \
  { return pgm_read_##reader##_near(p); }

DEFINE_PGM_READ_ANY(float,       float)
DEFINE_PGM_READ_ANY(signed char, byte)

#define XYZ_CONSTS_FROM_CONFIG(type, array, CONFIG) \
  static const PROGMEM type array##_P[XYZ] =        \
      { X_##CONFIG, Y_##CONFIG, Z_##CONFIG };     \
  static inline type array(int axis)          \
  { return pgm_read_any(&array##_P[axis]); }

#if NOMECH(DELTA)
  XYZ_CONSTS_FROM_CONFIG(float, base_max_pos,  MAX_POS)
  XYZ_CONSTS_FROM_CONFIG(float, base_home_pos, HOME_POS)
  XYZ_CONSTS_FROM_CONFIG(float, max_length,    MAX_LENGTH)
#endif
XYZ_CONSTS_FROM_CONFIG(float, base_min_pos,    MIN_POS)
XYZ_CONSTS_FROM_CONFIG(float, home_bump_mm,    HOME_BUMP_MM)
XYZ_CONSTS_FROM_CONFIG(signed char, home_dir,  HOME_DIR)

/**
 * ***************************************************************************
 * ******************************** FUNCTIONS ********************************
 * ***************************************************************************
 */

void stop();

void get_available_commands();
void process_next_command();
void prepare_move_to_destination();

void get_cartesian_from_steppers();
void set_current_from_steppers_for_axis(AxisEnum axis);

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

void tool_change(const uint8_t tmp_extruder, const float fr_mm_s = 0.0, bool no_move = false);
static void report_current_position();

/**
 * Sensitive pin test for M42, M226
 */
static bool pin_is_protected(uint8_t pin) {
  static const int sensitive_pins[] = SENSITIVE_PINS;
  for (uint8_t i = 0; i < COUNT(sensitive_pins); i++)
    if (sensitive_pins[i] == pin) return true;
  return false;
}

#if ENABLED(DEBUG_LEVELING_FEATURE)
  void print_xyz(const char* prefix, const char* suffix, const float x, const float y, const float z) {
    SERIAL_PS(prefix);
    SERIAL_MV("(", x);
    SERIAL_MV(", ", y);
    SERIAL_MV(", ", z);
    SERIAL_M(")");

    if (suffix) SERIAL_PS(suffix);
    else SERIAL_E;
  }

  void print_xyz(const char* prefix, const char* suffix, const float xyz[]) {
    print_xyz(prefix, suffix, xyz[X_AXIS], xyz[Y_AXIS], xyz[Z_AXIS]);
  }

  #if HAS(ABL)
    void print_xyz(const char* prefix, const char* suffix, const vector_3 &xyz) {
      print_xyz(prefix, suffix, xyz.x, xyz.y, xyz.z);
    }
  #endif

  #define DEBUG_POS(SUFFIX,VAR)       do{ \
    print_xyz(PSTR("  " STRINGIFY(VAR) "="), PSTR(" : " SUFFIX "\n"), VAR); } while(0)
#endif

/**
 * sync_plan_position
 *
 * Set the planner/stepper positions directly from current_position with
 * no kinematic translation. Used for homing axes and cartesian/core syncing.
 */
inline void sync_plan_position() {
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) DEBUG_POS("sync_plan_position", current_position);
  #endif
  planner.set_position_mm(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
}
inline void sync_plan_position_e() { planner.set_e_position_mm(current_position[E_AXIS]); }

#if IS_KINEMATIC

  inline void sync_plan_position_kinematic() {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("sync_plan_position_kinematic", current_position);
    #endif
    planner.set_position_mm_kinematic(current_position);
  }

  #define SYNC_PLAN_POSITION_KINEMATIC() sync_plan_position_kinematic()

#else

  #define SYNC_PLAN_POSITION_KINEMATIC() sync_plan_position()

#endif

/**
 * Inject the next "immediate" command, when possible.
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
    if (enqueue_and_echo_command(cmd)) {   // success?
      if (c)                               // newline char?
        injected_commands_P += i + 1;        // advance to the next command
      else
        injected_commands_P = NULL;          // nul char? no more commands
    }
  }
  return (injected_commands_P != NULL);      // return whether any more remain
}

/**
 * Record one or many commands to run from program memory.
 * Aborts the current queue, if any.
 * Note: drain_injected_commands_P() must be called repeatedly to drain the commands afterwards
 */
void enqueue_and_echo_commands_P(const char* pgcode) {
  injected_commands_P = pgcode;
  drain_injected_commands_P(); // first command executed asap (when possible)
}

void clear_command_queue() {
  cmd_queue_index_r = cmd_queue_index_w;
  commands_in_queue = 0;
}

/**
 * Once a new command is in the ring buffer, call this to commit it
 */
inline void _commit_command(bool say_ok) {
  send_ok[cmd_queue_index_w] = say_ok;
  cmd_queue_index_w = (cmd_queue_index_w + 1) % BUFSIZE;
  commands_in_queue++;
}

/**
 * Copy a command directly into the main command buffer, from RAM.
 * Returns true if successfully adds the command
 */
inline bool _enqueuecommand(const char* cmd, bool say_ok = false) {
  if (*cmd == ';' || commands_in_queue >= BUFSIZE) return false;
  strcpy(command_queue[cmd_queue_index_w], cmd);
  _commit_command(say_ok);
  return true;
}

void enqueue_and_echo_command_now(const char* cmd) {
  while (!enqueue_and_echo_command(cmd)) idle();
}

/**
 * Enqueue with Serial Echo
 */
bool enqueue_and_echo_command(const char* cmd, bool say_ok/*=false*/) {
  if (_enqueuecommand(cmd, say_ok)) {
    SERIAL_SMT(ECHO, MSG_ENQUEUEING, cmd);
    SERIAL_EM("\"");
    return true;
  }
  return false;
}

#if MB(ALLIGATOR) || MB(ALLIGATOR_V3)

  void setup_alligator_board() {
    ExternalDac::begin();
    SET_INPUT(MOTOR_FAULT_PIN);
    #if MB(ALLIGATOR_V3)
      SET_INPUT(MOTOR_FAULT_PIGGY_PIN);
      SET_INPUT(FTDI_COM_RESET_PIN);
      SET_INPUT(ESP_WIFI_MODULE_RESET_PIN);
      SET_OUTPUT(EXP1_VOLTAGE_SELECT);
      OUT_WRITE(EXP1_OUT_ENABLE_PIN, HIGH);
    #elif MB(ALLIGATOR)
      // Init Expansion Port Voltage logic Selector
      OUT_WRITE(EXP_VOLTAGE_LEVEL_PIN, UI_VOLTAGE_LEVEL);
    #endif

    #if HAS(BUZZER)
      buzz(10,10);
    #endif
  }

#endif

#if MB(ULTRATRONICS)
  void setup_ultratronics_board() {
    /* avoid floating pins */
    OUT_WRITE(ORIG_FAN_PIN, LOW);
    OUT_WRITE(ORIG_FAN2_PIN, LOW);
    OUT_WRITE(ORIG_TEMP_BED_PIN, LOW);
    OUT_WRITE(ORIG_HEATER_0_PIN, LOW);
    OUT_WRITE(ORIG_HEATER_1_PIN, LOW);
    OUT_WRITE(ORIG_HEATER_2_PIN, LOW);
    OUT_WRITE(ORIG_HEATER_3_PIN, LOW);

    /* setup CS pins */
    OUT_WRITE(MAX31855_SS0, HIGH);
    OUT_WRITE(MAX31855_SS1, HIGH);
    OUT_WRITE(MAX31855_SS2, HIGH);
    OUT_WRITE(MAX31855_SS3, HIGH);

    OUT_WRITE(ENC424_SS, HIGH);
    OUT_WRITE(SDSS, HIGH);

    SET_INPUT(MISO);
    SET_OUTPUT(MOSI);
  }
#endif // ULTRATRONICS

void setup_killpin() {
  #if HAS(KILL)
    SET_INPUT(KILL_PIN);
    PULLUP(KILL_PIN);
  #endif
  }

#if HAS(FIL_RUNOUT)
  void setup_filrunoutpin() {
    SET_INPUT(FIL_RUNOUT_PIN);
    #if ENABLED(ENDSTOPPULLUP_FIL_RUNOUT)
      PULLUP(FIL_RUNOUT_PIN);
    #endif
  }
#endif

// Set home pin
void setup_homepin(void) {
  #if HAS(HOME)
    SET_INPUT(HOME_PIN);
    PULLUP(HOME_PIN);
  #endif
}

void setup_photpin() {
  #if HAS(PHOTOGRAPH)
    OUT_WRITE(PHOTOGRAPH_PIN, LOW);
  #endif
}

#if HAS(CASE_LIGHT)
  void setup_case_light() {
    digitalWrite(CASE_LIGHT_PIN,
      #if ENABLED(CASE_LIGHT_DEFAULT_ON)
        255
      #else
        0
      #endif
    );
    analogWrite(CASE_LIGHT_PIN,
      #if ENABLED(CASE_LIGHT_DEFAULT_ON)
        255
      #else
        0
      #endif
    );
  }
#endif

void setup_powerhold() {
  #if HAS(SUICIDE)
    OUT_WRITE(SUICIDE_PIN, HIGH);
  #endif
  #if HAS(POWER_SWITCH)
    #if ENABLED(PS_DEFAULT_OFF)
      OUT_WRITE(PS_ON_PIN, PS_ON_ASLEEP);
    #else
      OUT_WRITE(PS_ON_PIN, PS_ON_AWAKE);
    #endif
  #endif
}

void suicide() {
  #if HAS(SUICIDE)
    OUT_WRITE(SUICIDE_PIN, LOW);
  #endif
}

void servo_init() {
  #if NUM_SERVOS >= 1 && HAS(SERVO_0)
    servo[0].attach(SERVO0_PIN);
    servo[0].detach(); // Just set up the pin. We don't have a position yet. Don't move to a random position.
  #endif
  #if NUM_SERVOS >= 2 && HAS(SERVO_1)
    servo[1].attach(SERVO1_PIN);
    servo[1].detach();
  #endif
  #if NUM_SERVOS >= 3 && HAS(SERVO_2)
    servo[2].attach(SERVO2_PIN);
    servo[2].detach();
  #endif
  #if NUM_SERVOS >= 4 && HAS(SERVO_3)
    servo[3].attach(SERVO3_PIN);
    servo[3].detach();
  #endif

  #if HAS(DONDOLO)
    servo[DONDOLO_SERVO_INDEX].attach(0);
    servo[DONDOLO_SERVO_INDEX].write(DONDOLO_SERVOPOS_E0);
    #if (DONDOLO_SERVO_DELAY > 0)
      safe_delay(DONDOLO_SERVO_DELAY);
      servo[DONDOLO_SERVO_INDEX].detach();
    #endif
  #endif

  #if HAS(Z_SERVO_ENDSTOP)
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
#if HAS(STEPPER_RESET)
  void disableStepperDrivers() {
    OUT_WRITE(STEPPER_RESET_PIN, LOW);  // drive it down to hold in reset motor driver chips
  }
  void enableStepperDrivers() { SET_INPUT(STEPPER_RESET_PIN); }  // set to input, which allows it to be pulled high by pullups
#endif

void gcode_line_error(const char* err, bool doFlush = true) {
  SERIAL_S(ER);
  SERIAL_PS(err);
  SERIAL_EV(gcode_LastN);
  //Serial.println(gcode_N);
  if (doFlush) FlushSerialRequestResend();
  serial_count = 0;
}

inline void get_serial_commands() {
  static char serial_line_buffer[MAX_CMD_SIZE];
  static boolean serial_comment_mode = false;

  // If the command buffer is empty for too long,
  // send "wait" to indicate Marlin is still waiting.
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
      serial_count = 0; //reset buffer

      char* command = serial_line_buffer;

      while (*command == ' ') command++; // skip any leading spaces
      char* npos = (*command == 'N') ? command : NULL; // Require the N parameter to start the line
      char* apos = strchr(command, '*');

      if (npos) {

        boolean M110 = strstr_P(command, PSTR("M110")) != NULL;

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
          int codenum = strtol(gpos + 1, NULL, 10);
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

      // If command was e-stop process now
      if (strcmp(command, "M108") == 0) {
        wait_for_heatup = false;
        #if ENABLED(ULTIPANEL)
          wait_for_user = false;
        #endif
      }
      if (strcmp(command, "M112") == 0) kill(PSTR(MSG_KILLED));
      if (strcmp(command, "M410") == 0) { quickstop_stepper(); }

      #if defined(NO_TIMEOUTS) && NO_TIMEOUTS > 0
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

#if ENABLED(SDSUPPORT)

  inline void get_sdcard_commands() {
    static bool stop_buffering = false,
                sd_comment_mode = false;

    if (!card.sdprinting) return;

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
      int16_t n = card.get();
      char sd_char = (char)n;
      card_eof = card.eof();
      if (card_eof || n == -1
          || sd_char == '\n' || sd_char == '\r'
          || ((sd_char == '#' || sd_char == ':') && !sd_comment_mode)
      ) {
        if (card_eof) {
          SERIAL_EM(MSG_FILE_PRINTED);
          card.printingHasFinished();
          card.checkautostart(true);
        }
        else if (n == -1) {
          SERIAL_LM(ER, MSG_SD_ERR_READ);
        }
        if (sd_char == '#') stop_buffering = true;

        sd_comment_mode = false; // for new command

        if (!sd_count) continue; // skip empty lines

        command_queue[cmd_queue_index_w][sd_count] = '\0'; // terminate string
        sd_count = 0; // clear buffer

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

  #if ENABLED(SDSUPPORT)
    get_sdcard_commands();
  #endif
}

inline bool code_has_value() {
  int i = 1;
  char c = seen_pointer[i];
  while (c == ' ') c = seen_pointer[++i];
  if (c == '-' || c == '+') c = seen_pointer[++i];
  if (c == '.') c = seen_pointer[++i];
  return NUMERIC(c);
}

inline float code_value_float() {
  float ret;
  char* e = strchr(seen_pointer, 'E');
  if (e) {
    *e = 0;
    ret = strtod(seen_pointer + 1, NULL);
    *e = 'E';
  }
  else
    ret = strtod(seen_pointer + 1, NULL);
  return ret;
}

inline unsigned long code_value_ulong() { return strtoul(seen_pointer + 1, NULL, 10); }

inline long code_value_long() { return strtol(seen_pointer + 1, NULL, 10); }

inline int code_value_int() { return (int)strtol(seen_pointer + 1, NULL, 10); }

inline uint16_t code_value_ushort() { return (uint16_t)strtoul(seen_pointer + 1, NULL, 10); }

inline uint8_t code_value_byte() { return (uint8_t)(constrain(strtol(seen_pointer + 1, NULL, 10), 0, 255)); }

inline bool code_value_bool() { return !code_has_value() || code_value_byte() > 0; }

#if ENABLED(INCH_MODE_SUPPORT)
  inline void set_input_linear_units(LinearUnit units) {
    switch (units) {
      case LINEARUNIT_INCH:
        linear_unit_factor = 25.4;
        break;
      case LINEARUNIT_MM:
      default:
        linear_unit_factor = 1.0;
        break;
    }
    volumetric_unit_factor = pow(linear_unit_factor, 3.0);
  }

  inline float axis_unit_factor(int axis) {
    return (axis >= E_AXIS && volumetric_enabled ? volumetric_unit_factor : linear_unit_factor);
  }

  inline float code_value_linear_units() { return code_value_float() * linear_unit_factor; }
  inline float code_value_axis_units(int axis) { return code_value_float() * axis_unit_factor(axis); }
  inline float code_value_per_axis_unit(int axis) { return code_value_float() / axis_unit_factor(axis); }

#else

  inline float code_value_linear_units() { return code_value_float(); }
  inline float code_value_axis_units(int axis) { UNUSED(axis); return code_value_float(); }
  inline float code_value_per_axis_unit(int axis) { UNUSED(axis); return code_value_float(); }

#endif

#if ENABLED(TEMPERATURE_UNITS_SUPPORT)
  inline void set_input_temp_units(TempUnit units) { input_temp_units = units; }

  float code_value_temp_abs() {
    switch (input_temp_units) {
      case TEMPUNIT_C:
        return code_value_float();
      case TEMPUNIT_F:
        return (code_value_float() - 32) * 0.5555555556;
      case TEMPUNIT_K:
        return code_value_float() - 272.15;
      default:
        return code_value_float();
    }
  }

  float code_value_temp_diff() {
    switch (input_temp_units) {
      case TEMPUNIT_C:
      case TEMPUNIT_K:
        return code_value_float();
      case TEMPUNIT_F:
        return code_value_float() * 0.5555555556;
      default:
        return code_value_float();
    }
  }
#else
  float code_value_temp_abs() { return code_value_float(); }
  float code_value_temp_diff() { return code_value_float(); }
#endif

FORCE_INLINE millis_t code_value_millis() { return code_value_ulong(); }
inline millis_t code_value_millis_from_seconds() { return code_value_float() * 1000; }

bool code_seen(char code) {
  seen_pointer = strchr(current_command_args, code);
  return (seen_pointer != NULL); // Return TRUE if the code-letter was found
}

/**
 * Set target_extruder from the T parameter or the active_extruder
 *
 * Returns TRUE if the target is invalid
 */
bool get_target_extruder_from_command(int code) {
  if (code_seen('T')) {
    if (code_value_byte() >= EXTRUDERS) {
      SERIAL_SMV(ER, "M", code);
      SERIAL_EMV(" " MSG_INVALID_EXTRUDER, code_value_byte());
      return true;
    }
    target_extruder = code_value_byte();
  }
  else
    target_extruder = active_extruder;

  return false;
}

/**
 * Set target_Hotend from the T parameter or the active_extruder
 *
 * Returns TRUE if the target is invalid
 */
bool get_target_hotend_from_command(int code) {
  if (code_seen('H')) {
    if (code_value_byte() >= HOTENDS) {
      SERIAL_SMV(ER, "M", code);
      SERIAL_EMV(" " MSG_INVALID_HOTEND, code_value_byte());
      return true;
    }
    target_extruder = code_value_byte();
  }
  else
    target_extruder = active_extruder;

  return false;
}

#if ENABLED(DUAL_X_CARRIAGE)

  #define DXC_FULL_CONTROL_MODE 0
  #define DXC_AUTO_PARK_MODE    1
  #define DXC_DUPLICATION_MODE  2

  static int dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;

  static float x_home_pos(int extruder) {
    if (extruder == 0)
      return LOGICAL_X_POSITION(base_home_pos(X_AXIS));
    else
      // In dual carriage mode the extruder offset provides an override of the
      // second X-carriage offset when homed - otherwise X2_HOME_POS is used.
      // This allow soft recalibration of the second extruder offset position without firmware reflash
      // (through the M218 command).
      return (hotend_offset[X_AXIS][1] > 0) ? hotend_offset[X_AXIS][1] : X2_HOME_POS;
  }

  static int x_home_dir(int extruder) {
    return (extruder == 0) ? X_HOME_DIR : X2_HOME_DIR;
  }

  static float inactive_hotend_x_pos = X2_MAX_POS;  // used in mode 0 & 1
  static bool active_hotend_parked = false;         // used in mode 1 & 2
  static float raised_parked_position[NUM_AXIS];    // used in mode 1
  static millis_t delayed_move_time = 0;            // used in mode 1
  static float duplicate_hotend_x_offset = DEFAULT_DUPLICATION_X_OFFSET; // used in mode 2
  static float duplicate_hotend_temp_offset = 0;    // used in mode 2
  bool hotend_duplication_enabled = false;          // used in mode 2

#endif // DUAL_X_CARRIAGE

/**
 * Software endstops can be used to monitor the open end of
 * an axis that has a hardware endstop on the other end. Or
 * they can prevent axes from moving past endstops and grinding.
 *
 * To keep doing their job as the coordinate system changes,
 * the software endstop positions must be refreshed to remain
 * at the same positions relative to the machine.
 */
void update_software_endstops(AxisEnum axis) {
  float offs = LOGICAL_POSITION(0, axis);

  #if ENABLED(DUAL_X_CARRIAGE)
    if (axis == X_AXIS) {
      float dual_max_x = max(hotend_offset[X_AXIS][1], X2_MAX_POS);
      if (active_extruder != 0) {
        soft_endstop_min[X_AXIS] = X2_MIN_POS + offs;
        soft_endstop_max[X_AXIS] = dual_max_x + offs;
        return;
      }
      else if (dual_x_carriage_mode == DXC_DUPLICATION_MODE) {
        soft_endstop_min[X_AXIS] = base_min_pos(X_AXIS) + offs;
        soft_endstop_max[X_AXIS] = min(base_max_pos(X_AXIS), dual_max_x - duplicate_hotend_x_offset) + offs;
        return;
      }
    }
    else
  #endif
  {
    #if MECH(DELTA)
      soft_endstop_min[axis] = base_min_pos(axis) + offs;
      soft_endstop_max[axis] = base_max_pos[axis] + offs;
    #else
      soft_endstop_min[axis] = base_min_pos(axis) + offs;
      soft_endstop_max[axis] = base_max_pos(axis) + offs;
    #endif
  }

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_MV("For ", axis_codes[axis]);
      SERIAL_MV(" axis:\n home_offset = ", home_offset[axis]);
      SERIAL_MV("\n position_shift = ", position_shift[axis]);
      SERIAL_MV("\n soft_endstop_min = ", soft_endstop_min[axis]);
      SERIAL_EMV("\n soft_endstop_max = ", soft_endstop_max[axis]);
    }
  #endif

  #if MECH(DELTA)
    if (axis == Z_AXIS) {
      delta_clip_start_height = soft_endstop_max[axis] - delta_safe_distance_from_top();
    }
  #endif
}

/**
 * Change the home offset for an axis, update the current
 * position and the software endstops to retain the same
 * relative distance to the new home.
 *
 * Since this changes the current_position, code should
 * call sync_plan_position soon after this.
 */
static void set_home_offset(AxisEnum axis, float v) {
  current_position[axis] += v - home_offset[axis];
  home_offset[axis] = v;
  update_software_endstops(axis);
}

/**
 * Set an axis' current position to its home position (after homing).
 *
 * For Core and Cartesian robots this applies one-to-one when an
 * individual axis has been homed.
 *
 * DELTA should wait until all homing is done before setting the XYZ
 * current_position to home, because homing is a single operation.
 * In the case where the axis positions are already known and previously
 * homed, DELTA could home to X or Y individually by moving either one
 * to the center. However, homing Z always homes XY and Z.
 *
 * SCARA should wait until all XY homing is done before setting the XY
 * current_position to home, because neither X nor Y is at home until
 * both are at home. Z can however be homed individually.
 * 
 */
static void set_axis_is_at_home(AxisEnum axis) {
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_MV(">>> set_axis_is_at_home(", axis_codes[axis]);
      SERIAL_C(')'); SERIAL_E;
    }
  #endif

  axis_known_position[axis] = axis_homed[axis] = true;

  position_shift[axis] = 0;
  update_software_endstops(axis);

  #if ENABLED(DUAL_X_CARRIAGE)
    if (axis == X_AXIS && (active_extruder != 0 || dual_x_carriage_mode == DXC_DUPLICATION_MODE)) {
      if (active_extruder != 0)
        current_position[X_AXIS] = x_home_pos(active_extruder);
      else
        current_position[X_AXIS] = LOGICAL_X_POSITION(base_home_pos(X_AXIS));
      update_software_endstops(X_AXIS);
      return;
    }
  #endif

  #if MECH(MORGAN_SCARA)

    /**
     * Morgan SCARA homes XY at the same time
     */
    if (axis == X_AXIS || axis == Y_AXIS) {

      float homeposition[XYZ];
      LOOP_XYZ(i) homeposition[i] = LOGICAL_POSITION(base_home_pos((AxisEnum)i), i);

      // SERIAL_MV("homeposition X:", homeposition[X_AXIS]);
      // SERIAL_EMV(" Y:", homeposition[Y_AXIS]);

      /**
       * Works out real Homeposition angles using inverse kinematics,
       * and calculates homing offset using forward kinematics
       */
      inverse_kinematics(homeposition);
      forward_kinematics_SCARA(delta[A_AXIS], delta[B_AXIS]);

      // SERIAL_MV("base Theta= ", delta[X_AXIS]);
      // SERIAL_EMV(" base Psi+Theta=", delta[Y_AXIS]);

      current_position[axis] = LOGICAL_POSITION(cartes[axis], axis);

      /**
       * SCARA home positions are based on configuration since the actual
       * limits are determined by the inverse kinematic transform.
       */
      soft_endstop_min[axis] = base_min_pos(axis); // + (cartes[axis] - base_home_pos(axis));
      soft_endstop_max[axis] = base_max_pos(axis); // + (cartes[axis] - base_home_pos(axis));
    }
    else
  #endif
  {
    #if MECH(DELTA)
      current_position[axis] = LOGICAL_POSITION(base_home_pos[axis], axis);
    #else
      current_position[axis] = LOGICAL_POSITION(base_home_pos(axis), axis);
    #endif
  }

  /**
   * Z Probe Z Homing? Account for the probe's Z offset.
   */
  #if HAS(BED_PROBE) && (Z_HOME_DIR < 0)
    if (axis == Z_AXIS) {
      #if HOMING_Z_WITH_PROBE

        current_position[Z_AXIS] -= zprobe_zoffset;

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) {
            SERIAL_EM("*** Z HOMED WITH PROBE ***");
            SERIAL_EMV("zprobe_zoffset = ", zprobe_zoffset);
          }
        #endif

      #elif ENABLED(DEBUG_LEVELING_FEATURE)

        if (DEBUGGING(LEVELING)) SERIAL_EM("*** Z HOMED TO ENDSTOP ***");

      #endif
    }
  #endif

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_MV("> home_offset[", axis_codes[axis]);
      SERIAL_EMV("] = ", home_offset[axis]);
      DEBUG_POS("", current_position);
      SERIAL_MV("<<< set_axis_is_at_home(", axis_codes[axis]);
      SERIAL_C(')'); SERIAL_E;
    }
  #endif
}

/**
 * Some planner shorthand inline functions
 */
inline float get_homing_bump_feedrate(AxisEnum axis) {
  int constexpr homing_bump_divisor[] = HOMING_BUMP_DIVISOR;
  int hbd = homing_bump_divisor[axis];
  if (hbd < 1) {
    hbd = 10;
    SERIAL_LM(ER, "Warning: Homing Bump Divisor < 1");
  }
  return homing_feedrate_mm_s[axis] / hbd;
}

/**
 * line_to_current_position
 * Move the planner to the current position from wherever it last moved
 * (or from wherever it has been told it is located).
 */
inline void line_to_current_position() {
  planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate_mm_s, active_extruder, active_driver);
}

/**
 * line_to_destination
 * Move the planner, not necessarily synced with current_position
 */
inline void line_to_destination(float fr_mm_s) {
  planner.buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], fr_mm_s, active_extruder, active_driver);
}
inline void line_to_destination() { line_to_destination(feedrate_mm_s); }

inline void set_current_to_destination() { memcpy(current_position, destination, sizeof(current_position)); }
inline void set_destination_to_current() { memcpy(destination, current_position, sizeof(destination)); }

#if IS_KINEMATIC
  /**
   * Calculate delta, start a line, and set current_position to destination
   */
  void prepare_uninterpolated_move_to_destination(const float fr_mm_s=0.0) {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("prepare_uninterpolated_move_to_destination", destination);
    #endif

    if ( current_position[X_AXIS] == destination[X_AXIS]
      && current_position[Y_AXIS] == destination[Y_AXIS]
      && current_position[Z_AXIS] == destination[Z_AXIS]
      && current_position[E_AXIS] == destination[E_AXIS]
    ) return;

    refresh_cmd_timeout();
    planner.buffer_line_kinematic(destination, MMS_SCALED(fr_mm_s ? fr_mm_s : feedrate_mm_s), active_extruder, active_driver);
    set_current_to_destination();
  }
#endif

/**
 *  Plan a move to (X, Y, Z) and set the current_position
 *  The final current_position may not be the one that was requested
 */
void do_blocking_move_to(const float &x, const float &y, const float &z, const float &fr_mm_s /*=0.0*/) {
  float old_feedrate_mm_s = feedrate_mm_s;

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) print_xyz(PSTR(">>> do_blocking_move_to"), NULL, x, y, z);
  #endif

  #if MECH(DELTA)

    feedrate_mm_s = fr_mm_s ? fr_mm_s : XY_PROBE_FEEDRATE_MM_S;

    set_destination_to_current();          // sync destination at the start

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("set_destination_to_current", destination);
    #endif

    // when in the danger zone
    if (current_position[Z_AXIS] > delta_clip_start_height) {
      if (z > delta_clip_start_height) {   // staying in the danger zone
        destination[X_AXIS] = x;           // move directly (uninterpolated)
        destination[Y_AXIS] = y;
        destination[Z_AXIS] = z;
        prepare_uninterpolated_move_to_destination(); // set_current_to_destination
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("danger zone move", current_position);
        #endif
        return;
      }
      else {
        destination[Z_AXIS] = delta_clip_start_height;
        prepare_uninterpolated_move_to_destination(); // set_current_to_destination
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("zone border move", current_position);
        #endif
      }
    }

    if (z > current_position[Z_AXIS]) {    // raising?
      destination[Z_AXIS] = z;
      prepare_uninterpolated_move_to_destination();   // set_current_to_destination
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) DEBUG_POS("z raise move", current_position);
      #endif
    }

    destination[X_AXIS] = x;
    destination[Y_AXIS] = y;
    prepare_move_to_destination();         // set_current_to_destination
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("xy move", current_position);
    #endif

    if (z < current_position[Z_AXIS]) {    // lowering?
      destination[Z_AXIS] = z;
      prepare_uninterpolated_move_to_destination();   // set_current_to_destination
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) DEBUG_POS("z lower move", current_position);
      #endif
    }

  #elif IS_SCARA

    set_destination_to_current();

    // If Z needs to raise, do it before moving XY
    if (destination[Z_AXIS] < z) {
      destination[Z_AXIS] = z;
      prepare_uninterpolated_move_to_destination(fr_mm_s ? fr_mm_s : homing_feedrate_mm_s[Z_AXIS]);
    }

    destination[X_AXIS] = x;
    destination[Y_AXIS] = y;
    prepare_uninterpolated_move_to_destination(fr_mm_s ? fr_mm_s : XY_PROBE_FEEDRATE_MM_S);

    // If Z needs to lower, do it after moving XY
    if (destination[Z_AXIS] > z) {
      destination[Z_AXIS] = z;
      prepare_uninterpolated_move_to_destination(fr_mm_s ? fr_mm_s : homing_feedrate_mm_s[Z_AXIS]);
    }

  #else

    // If Z needs to raise, do it before moving XY
    if (current_position[Z_AXIS] < z) {
      feedrate_mm_s = fr_mm_s ? fr_mm_s : homing_feedrate_mm_s[Z_AXIS];
      current_position[Z_AXIS] = z;
      line_to_current_position();
    }

    feedrate_mm_s = fr_mm_s ? fr_mm_s : XY_PROBE_FEEDRATE_MM_S;
    current_position[X_AXIS] = x;
    current_position[Y_AXIS] = y;
    line_to_current_position();

    // If Z needs to lower, do it after moving XY
    if (current_position[Z_AXIS] > z) {
      feedrate_mm_s = fr_mm_s ? fr_mm_s : homing_feedrate_mm_s[Z_AXIS];
      current_position[Z_AXIS] = z;
      line_to_current_position();
    }

  #endif

  stepper.synchronize();

  feedrate_mm_s = old_feedrate_mm_s;

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) SERIAL_EM("<<< do_blocking_move_to");
  #endif
}
void do_blocking_move_to_x(const float &x, const float &fr_mm_s/*=0.0*/) {
  do_blocking_move_to(x, current_position[Y_AXIS], current_position[Z_AXIS], fr_mm_s);
}
void do_blocking_move_to_z(const float &z, const float &fr_mm_s/*=0.0*/) {
  do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], z, fr_mm_s);
}
void do_blocking_move_to_xy(const float &x, const float &y, const float &fr_mm_s/*=0.0*/) {
  do_blocking_move_to(x, y, current_position[Z_AXIS], fr_mm_s);
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
static void setup_for_endstop_or_probe_move() {
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) DEBUG_POS("setup_for_endstop_or_probe_move", current_position);
  #endif
  saved_feedrate_mm_s = feedrate_mm_s;
  saved_feedrate_percentage = feedrate_percentage;
  feedrate_percentage = 100;
  refresh_cmd_timeout();
}

static void clean_up_after_endstop_or_probe_move() {
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) DEBUG_POS("clean_up_after_endstop_or_probe_move", current_position);
  #endif
  feedrate_mm_s = saved_feedrate_mm_s;
  feedrate_percentage = saved_feedrate_percentage;
  refresh_cmd_timeout();
}

#if HAS(BED_PROBE)
  /**
   * Raise Z to a minimum height to make room for a servo to move
   */
  void do_probe_raise(float z_raise) {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_MV("do_probe_raise(", z_raise);
        SERIAL_C(')'); SERIAL_E;
      }
    #endif

    float z_dest = LOGICAL_Z_POSITION(z_raise);
    if (zprobe_zoffset < 0) z_dest -= zprobe_zoffset;

    if (z_dest > current_position[Z_AXIS])
      do_blocking_move_to_z(z_dest);
  }
#endif // HAS_BED_PROBE

#if ENABLED(Z_PROBE_ALLEN_KEY) || HAS(Z_PROBE_SLED) || HAS(PROBING_PROCEDURE) || HOTENDS > 1 || ENABLED(NOZZLE_CLEAN_FEATURE) || ENABLED(NOZZLE_PARK_FEATURE)
  static bool axis_unhomed_error(const bool x, const bool y, const bool z) {
    const bool  xx = x && !axis_homed[X_AXIS],
                yy = y && !axis_homed[Y_AXIS],
                zz = z && !axis_homed[Z_AXIS];
    if (xx || yy || zz) {
      SERIAL_SM(ER, MSG_HOME " ");
      if (xx) SERIAL_M(MSG_X);
      if (yy) SERIAL_M(MSG_Y);
      if (zz) SERIAL_M(MSG_Z);
      SERIAL_EM(" " MSG_FIRST);

      #if ENABLED(ULTRA_LCD)
        char message[3 * (LCD_WIDTH) + 1] = ""; // worst case is kana.utf with up to 3*LCD_WIDTH+1
        strcat_P(message, PSTR(MSG_HOME " "));
        if (xx) strcat_P(message, PSTR(MSG_X));
        if (yy) strcat_P(message, PSTR(MSG_Y));
        if (zz) strcat_P(message, PSTR(MSG_Z));
        strcat_P(message, PSTR(" " MSG_FIRST));
        lcd_setstatus(message);
      #endif
      return true;
    }
    return false;
  }
#endif

#if HAS(Z_PROBE_SLED)

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
        SERIAL_C(')'); SERIAL_E;
      }
    #endif

    // Dock sled a bit closer to ensure proper capturing
    do_blocking_move_to_x(X_MAX_POS + SLED_DOCKING_OFFSET - ((stow) ? 1 : 0));
    digitalWrite(SLED_PIN, !stow); // switch solenoid
  }

#endif // Z_PROBE_SLED
  
#if ENABLED(Z_PROBE_ALLEN_KEY)
  void run_deploy_moves_script() {
    // Move to the start position to initiate deployment
    do_blocking_move_to(z_probe_deploy_start_location[X_AXIS], z_probe_deploy_start_location[Y_AXIS], z_probe_deploy_start_location[Z_AXIS], homing_feedrate_mm_s[Z_AXIS]);

    // Move to engage deployment
    do_blocking_move_to(z_probe_deploy_end_location[X_AXIS], z_probe_deploy_end_location[Y_AXIS], z_probe_deploy_end_location[Z_AXIS], homing_feedrate_mm_s[Z_AXIS] / 10);

    // Move to trigger deployment
    do_blocking_move_to(z_probe_deploy_start_location[X_AXIS], z_probe_deploy_start_location[Y_AXIS], z_probe_deploy_start_location[Z_AXIS], homing_feedrate_mm_s[Z_AXIS]);
  }
  void run_stow_moves_script() {
    // Move to the start position to initiate retraction
    do_blocking_move_to(z_probe_retract_start_location[X_AXIS], z_probe_retract_start_location[Y_AXIS], z_probe_retract_start_location[Z_AXIS], homing_feedrate_mm_s[Z_AXIS]);

    // Move the nozzle down to push the Z probe into retracted position
    do_blocking_move_to(z_probe_retract_end_location[X_AXIS], z_probe_retract_end_location[Y_AXIS], z_probe_retract_end_location[Z_AXIS], homing_feedrate_mm_s[Z_AXIS] / 10);

    // Move up for safety
    do_blocking_move_to(z_probe_retract_start_location[X_AXIS], z_probe_retract_start_location[Y_AXIS], z_probe_retract_start_location[Z_AXIS], homing_feedrate_mm_s[Z_AXIS]);
  }
#endif

#if HAS(BED_PROBE)

  // TRIGGERED_WHEN_STOWED_TEST can easily be extended to servo probes, ... if needed.
  #if ENABLED(PROBE_IS_TRIGGERED_WHEN_STOWED_TEST)
    #if HAS(Z_PROBE_PIN)
      #define _TRIGGERED_WHEN_STOWED_TEST (READ(Z_PROBE_PIN) != Z_PROBE_ENDSTOP_INVERTING)
    #else
      #define _TRIGGERED_WHEN_STOWED_TEST (READ(Z_MIN_PIN) != Z_MIN_ENDSTOP_INVERTING)
    #endif
  #endif

  #define DEPLOY_PROBE() set_probe_deployed(true)
  #define STOW_PROBE() set_probe_deployed(false)

  #if ENABLED(BLTOUCH)
    FORCE_INLINE void set_bltouch_deployed(const bool &deploy) {
      servo[Z_ENDSTOP_SERVO_NR].move(deploy ? BLTOUCH_DEPLOY : BLTOUCH_STOW);
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) {
          SERIAL_MV("set_bltouch_deployed(", deploy);
          SERIAL_C(')'); SERIAL_E;
        }
      #endif
    }
  #endif

  // returns false for ok and true for failure
  static bool set_probe_deployed(bool deploy) {

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        DEBUG_POS("set_probe_deployed", current_position);
        SERIAL_EMV("deploy: ", deploy);
      }
    #endif

    if (endstops.z_probe_enabled == deploy) return false;

    // Make room for probe
    do_probe_raise(_Z_PROBE_DEPLOY_HEIGHT);

    // When deploying make sure BLTOUCH is not already triggered
    #if ENABLED(BLTOUCH)
      if (deploy && TEST_BLTOUCH()) { stop(); return true; }
    #endif

    #if ENABLED(Z_PROBE_SLED)
      if (axis_unhomed_error(true, false, false)) { stop(); return true; }
    #elif ENABLED(Z_PROBE_ALLEN_KEY)
      if (axis_unhomed_error(true, true,  true )) { stop(); return true; }
    #endif

    float oldXpos = current_position[X_AXIS],
          oldYpos = current_position[Y_AXIS];

    #if ENABLED(_TRIGGERED_WHEN_STOWED_TEST)
      // If endstop is already false, the Z probe is deployed
      if (_TRIGGERED_WHEN_STOWED_TEST == deploy) {        // closed after the probe specific actions.
                                                          // Would a goto be less ugly?
        //while (!_TRIGGERED_WHEN_STOWED_TEST) { idle();  // would offer the opportunity
                                                          // for a triggered when stowed manual probe.
        if (!deploy) endstops.enable_z_probe(false); // Switch off triggered when stowed probes early
                                                     // otherwise an Allen-Key probe can't be stowed.
    #endif

    #if ENABLED(Z_PROBE_SLED)
      dock_sled(!deploy);
    #elif ENABLED(BLTOUCH)
      set_bltouch_deployed(deploy);
    #elif HAS_Z_SERVO_ENDSTOP && DISABLED(BLTOUCH)
      servo[Z_ENDSTOP_SERVO_NR].move(z_servo_angle[deploy ? 0 : 1]);
    #elif ENABLED(Z_PROBE_ALLEN_KEY)
      deploy ? run_deploy_moves_script() : run_stow_moves_script();
    #endif

    #if ENABLED(_TRIGGERED_WHEN_STOWED_TEST)
      } // opened before the probe specific actions

      if (_TRIGGERED_WHEN_STOWED_TEST == deploy) {
        if (IsRunning()) {
          SERIAL_LM(ER, "Z-Probe failed");
          LCD_ALERTMESSAGEPGM("Err: ZPROBE");
        }
        stop();
        return true;
      }
    #endif

    do_blocking_move_to(oldXpos, oldYpos, current_position[Z_AXIS]); // return to position before deploy
    endstops.enable_z_probe(deploy);
    return false;
  }

  static void do_probe_move(float z, float fr_mm_m) {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS(">>> do_probe_move", current_position);
    #endif

    // Move down until probe triggered
    do_blocking_move_to_z(LOGICAL_Z_POSITION(z), MMM_TO_MMS(fr_mm_m));

    // Clear endstop flags
    endstops.hit_on_purpose();

    // Get Z where the steppers were interrupted
    set_current_from_steppers_for_axis(Z_AXIS);

    // Tell the planner where we actually are
    SYNC_PLAN_POSITION_KINEMATIC();

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("<<< do_probe_move", current_position);
    #endif
  }

  // Do a single Z probe and return with current_position[Z_AXIS]
  // at the height where the probe triggered.
  static float run_z_probe() {

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS(">>> run_z_probe", current_position);
    #endif

    // Prevent stepper_inactive_time from running out and EXTRUDER_RUNOUT_PREVENT from extruding
    refresh_cmd_timeout();

    #if ENABLED(PROBE_DOUBLE_TOUCH)

      // Do a first probe at the fast speed
      do_probe_move(-(Z_MAX_LENGTH) - 10, Z_PROBE_SPEED_FAST);

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        float first_probe_z = current_position[Z_AXIS];
        if (DEBUGGING(LEVELING)) SERIAL_MV("1st Probe Z:", first_probe_z);
      #endif

      // move up by the bump distance
      do_blocking_move_to_z(current_position[Z_AXIS] + home_bump_mm(Z_AXIS), MMM_TO_MMS(Z_PROBE_SPEED_FAST));

    #else

      // If the nozzle is above the travel height then
      // move down quickly before doing the slow probe
      float z = LOGICAL_Z_POSITION(Z_PROBE_BETWEEN_HEIGHT);
      if (zprobe_zoffset < 0) z -= zprobe_zoffset;
      if (z < current_position[Z_AXIS])
        do_blocking_move_to_z(z, MMM_TO_MMS(Z_PROBE_SPEED_FAST));

    #endif

    // move down slowly to find bed
    do_probe_move(-(Z_MAX_LENGTH) - 10, Z_PROBE_SPEED_SLOW);

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("<<< run_z_probe", current_position);
    #endif

    // Debug: compare probe heights
    #if ENABLED(PROBE_DOUBLE_TOUCH) && ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_MV("2nd Probe Z:", current_position[Z_AXIS]);
        SERIAL_EMV(" Discrepancy:", first_probe_z - current_position[Z_AXIS]);
      }
    #endif

    return current_position[Z_AXIS];
  }

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
      forward_kinematics_SCARA(delta[A_AXIS], delta[B_AXIS]);
      float tx = LOGICAL_X_POSITION(cartes[X_AXIS]),
            ty = LOGICAL_Y_POSITION(cartes[Y_AXIS]);

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
      float ab = RADIANS(delta[A_AXIS] + delta[B_AXIS] + 90.0);
      return vector_3(
        logical[X_AXIS] + sin(ab) * X_PROBE_OFFSET_FROM_NOZZLE,
        logical[Y_AXIS] - cos(ab) * Y_PROBE_OFFSET_FROM_NOZZLE,
        0
      );
    }

  #endif // MAKERARM_SCARA

  //
  // - Move to the given XY
  // - Deploy the probe, if not already deployed
  // - Probe the bed, get the Z position
  // - Depending on the 'stow' flag
  //   - Stow the probe, or
  //   - Raise to the BETWEEN height
  // - Return the probed Z position
  //
  static float probe_pt(const float &x, const float &y, bool stow = true, int verbose_level = 1) {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_MV(">>> probe_pt(", x);
        SERIAL_MV(", ", y);
        SERIAL_MV(", ", stow ? "stow" : "no stow");
        SERIAL_C(')'); SERIAL_E;
        DEBUG_POS("", current_position);
      }
    #endif

    float old_feedrate_mm_s = feedrate_mm_s;

    #if MECH(MAKERARM_SCARA)
      vector_3 point = probe_point_to_end_point(x, y);
      float dx = point.x, dy = point.y;
      if (dx == 0.0 && dy == 0.0) { BUZZ(100, 220); return 0.0; }
    #else
      float dx = x - X_PROBE_OFFSET_FROM_NOZZLE,
            dy = y - Y_PROBE_OFFSET_FROM_NOZZLE;
    #endif

    // Ensure a minimum height before moving the probe
    do_probe_raise(Z_PROBE_BETWEEN_HEIGHT);

    // Move the probe to the given XY
    do_blocking_move_to_xy(dx, dy, XY_PROBE_FEEDRATE_MM_S);

    if (DEPLOY_PROBE()) return NAN;

    float measured_z = run_z_probe();

    if (!stow)
      do_probe_raise(Z_PROBE_BETWEEN_HEIGHT);
    else
      if (STOW_PROBE()) return NAN;

    if (verbose_level > 2) {
      SERIAL_M(MSG_BED_LEVELLING_BED);
      SERIAL_MV(MSG_BED_LEVELLING_X, x, 3);
      SERIAL_MV(MSG_BED_LEVELLING_Y, y, 3);
      SERIAL_EMV(MSG_BED_LEVELLING_Z, measured_z - -zprobe_zoffset + 0.0001, 3);
    }

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) SERIAL_EM("<<< probe_pt");
    #endif

    feedrate_mm_s = old_feedrate_mm_s;

    return measured_z;
  }

#endif // HAS(BED_PROBE)

#if PLANNER_LEVELING
  /**
   * Turn bed leveling on or off, fixing the current
   * position as-needed.
   *
   * Disable: Current position = physical position
   *  Enable: Current position = "unleveled" physical position
   */
  void set_bed_leveling_enabled(bool enable=true) {
    #if ENABLED(MESH_BED_LEVELING)

      if (!enable && mbl.active())
        current_position[Z_AXIS] +=
          mbl.get_z(RAW_CURRENT_POSITION(X_AXIS), RAW_CURRENT_POSITION(Y_AXIS)) - (MESH_HOME_SEARCH_Z);

      mbl.set_active(enable && mbl.has_mesh()); // was set_has_mesh(). Is this not correct?

    #elif HAS(ABL)

      if (enable != planner.abl_enabled) {
        planner.abl_enabled = enable;
        if (!enable)
          set_current_from_steppers_for_axis(
            #if ABL_PLANAR
              ALL_AXES
            #else
              Z_AXIS
            #endif
          );
        else
          planner.unapply_leveling(current_position);
      }

    #endif
  }

  /**
   * Reset calibration results to zero.
   */
  void reset_bed_level() {
    #if ENABLED(MESH_BED_LEVELING)
      if (mbl.has_mesh()) {
        set_bed_leveling_enabled(false);
        mbl.reset();
        mbl.set_has_mesh(false);
      }
    #else
      planner.abl_enabled = false;
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) SERIAL_EM("reset_bed_level");
      #endif
      #if ABL_PLANAR
        planner.bed_level_matrix.set_to_identity();
      #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
        for (uint8_t x = 0; x < ABL_GRID_POINTS_X; x++)
          for (uint8_t y = 0; y < ABL_GRID_POINTS_Y; y++)
            bilinear_level_grid[x][y] = 1000.0;
      #endif
    #endif
  }

#endif // PLANNER_LEVELING

#if ENABLED(AUTO_BED_LEVELING_BILINEAR)

  /**
   * Extrapolate a single point from its neighbors
   */
  static void extrapolate_one_point(uint8_t x, uint8_t y, int8_t xdir, int8_t ydir) {
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_M("Extrapolate [");
        if (x < 10) SERIAL_C(' ');
        SERIAL_V((int)x);
        SERIAL_C(xdir ? (xdir > 0 ? '+' : '-') : ' ');
        SERIAL_C(' ');
        if (y < 10) SERIAL_C(' ');
        SERIAL_V((int)y);
        SERIAL_C(ydir ? (ydir > 0 ? '+' : '-') : ' ');
        SERIAL_C(']');
      }
    #endif
    if (bilinear_level_grid[x][y] < 999.0) {
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) SERIAL_EM(" (done)");
      #endif
      return;  // Don't overwrite good values.
    }
    SERIAL_E;

    // Get X neighbors, Y neighbors, and XY neighbors
    float a1 = bilinear_level_grid[x + xdir][y], a2 = bilinear_level_grid[x + xdir * 2][y],
          b1 = bilinear_level_grid[x][y + ydir], b2 = bilinear_level_grid[x][y + ydir * 2],
          c1 = bilinear_level_grid[x + xdir][y + ydir], c2 = bilinear_level_grid[x + xdir * 2][y + ydir * 2];

    // Treat far unprobed points as zero, near as equal to far
    if (a2 > 999.0) a2 = 0.0; if (a1 > 999.0) a1 = a2;
    if (b2 > 999.0) b2 = 0.0; if (b1 > 999.0) b1 = b2;
    if (c2 > 999.0) c2 = 0.0; if (c1 > 999.0) c1 = c2;

    float a = 2 * a1 - a2, b = 2 * b1 - b2, c = 2 * c1 - c2;

    // Take the average intstead of the median
    bilinear_level_grid[x][y] = (a + b + c) / 3.0;

    // Median is robust (ignores outliers).
    // bilinear_level_grid[x][y] = (a < b) ? ((b < c) ? b : (c < a) ? a : c)
    //                                : ((c < b) ? b : (a < c) ? a : c);
  }

  //#define EXTRAPOLATE_FROM_EDGE

  #if ENABLED(EXTRAPOLATE_FROM_EDGE)
    #if ABL_GRID_POINTS_X < ABL_GRID_POINTS_Y
      #define HALF_IN_X
    #elif ABL_GRID_POINTS_Y < ABL_GRID_POINTS_X
      #define HALF_IN_Y
    #endif
  #endif

  /**
   * Fill in the unprobed points (corners of circular print surface)
   * using linear extrapolation, away from the center.
   */
  static void extrapolate_unprobed_bed_level() {
    #if ENABLED(HALF_IN_X)
      const uint8_t ctrx2 = 0, xlen = ABL_GRID_POINTS_X - 1;
    #else
      const uint8_t ctrx1 = (ABL_GRID_POINTS_X - 1) / 2, // left-of-center
                    ctrx2 = ABL_GRID_POINTS_X / 2,       // right-of-center
                    xlen = ctrx1;
    #endif

    #if ENABLED(HALF_IN_Y)
      const uint8_t ctry2 = 0, ylen = ABL_GRID_POINTS_Y - 1;
    #else
      const uint8_t ctry1 = (ABL_GRID_POINTS_Y - 1) / 2, // top-of-center
                    ctry2 = ABL_GRID_POINTS_Y / 2,       // bottom-of-center
                    ylen = ctry1;
    #endif

    for (uint8_t xo = 0; xo <= xlen; xo++)
      for (uint8_t yo = 0; yo <= ylen; yo++) {
        uint8_t x2 = ctrx2 + xo, y2 = ctry2 + yo;
        #if DISABLED(HALF_IN_X)
          uint8_t x1 = ctrx1 - xo;
        #endif
        #if DISABLED(HALF_IN_Y)
          uint8_t y1 = ctry1 - yo;
          #if DISABLED(HALF_IN_X)
            extrapolate_one_point(x1, y1, +1, +1);   //  left-below + +
          #endif
          extrapolate_one_point(x2, y1, -1, +1);     // right-below - +
        #endif
        #if DISABLED(HALF_IN_X)
          extrapolate_one_point(x1, y2, +1, -1);     //  left-above + -
        #endif
        extrapolate_one_point(x2, y2, -1, -1);       // right-above - -
      }

  }

  /**
   * Print calibration results for plotting or manual frame adjustment.
   */
  void print_bed_level() {
    SERIAL_LM(ECHO, "Bilinear Leveling Grid:");
    SERIAL_S(ECHO);
    for (uint8_t x = 0; x < ABL_GRID_POINTS_X; x++) {
      SERIAL_M("    ");
      if (x < 10) SERIAL_C(' ');
      SERIAL_V((int)x);
    }
    SERIAL_E;
    for (uint8_t y = 0; y < ABL_GRID_POINTS_Y; y++) {
      SERIAL_S(ECHO);
      if (y < 10) SERIAL_C(' ');
      SERIAL_V((int)y);
      for (uint8_t x = 0; x < ABL_GRID_POINTS_X; x++) {
        SERIAL_C(' ');
        float offset = bilinear_level_grid[x][y];
        if (offset < 999.0) {
          if (offset >= 0) SERIAL_C('+');
          SERIAL_V(offset, 2);
        }
        else
          SERIAL_M(" ====");
      }
      SERIAL_E;
    }
    SERIAL_E;
  }

#endif // AUTO_BED_LEVELING_BILINEAR

/**
 * Home an individual linear axis
 */
static void do_homing_move(AxisEnum axis, float distance, float fr_mm_s=0.0) {

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_MV(">>> do_homing_move(", axis_codes[axis]);
      SERIAL_MV(", ", distance);
      SERIAL_MV(", ", fr_mm_s);
      SERIAL_C(')'); SERIAL_E;
    }
  #endif

  #if HOMING_Z_WITH_PROBE && ENABLED(BLTOUCH)
    bool deploy_bltouch = (axis == Z_AXIS && distance < 0);
    if (deploy_bltouch) set_bltouch_deployed(true);
  #endif

  // Tell the planner we're at Z=0
  current_position[axis] = 0;

  #if IS_SCARA
    SYNC_PLAN_POSITION_KINEMATIC();
    current_position[axis] = distance;
    inverse_kinematics(current_position);
    planner.buffer_line(delta[A_AXIS], delta[B_AXIS], delta[C_AXIS], current_position[E_AXIS], fr_mm_s ? fr_mm_s : homing_feedrate_mm_s[axis], active_extruder, active_driver);
  #else
    sync_plan_position();
    current_position[axis] = distance;
    planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], fr_mm_s ? fr_mm_s : homing_feedrate_mm_s[axis], active_extruder, active_driver);
  #endif

  stepper.synchronize();

  #if HOMING_Z_WITH_PROBE && ENABLED(BLTOUCH)
    if (deploy_bltouch) set_bltouch_deployed(false);
  #endif

  endstops.hit_on_purpose();

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_MV("<<< do_homing_move(", axis_codes[axis]);
      SERIAL_C(')'); SERIAL_E;
    }
  #endif
}

/**
 * Home an individual "raw axis" to its endstop.
 * This applies to XYZ on Cartesian and Core robots, and
 * to the individual ABC steppers on DELTA and SCARA.
 *
 * At the end of the procedure the axis is marked as
 * homed and the current position of that axis is updated.
 * Kinematic robots should wait till all axes are homed
 * before updating the current position.
 */
#define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)

static void homeaxis(AxisEnum axis) {

  #if IS_SCARA
    // Only Z homing (with probe) is permitted
    if (axis != Z_AXIS) { BUZZ(100, 880); return; }
  #else
    #define CAN_HOME(A) \
      (axis == A##_AXIS && ((A##_MIN_PIN > -1 && A##_HOME_DIR < 0) || (A##_MAX_PIN > -1 && A##_HOME_DIR > 0)))
    if (!CAN_HOME(X) && !CAN_HOME(Y) && !CAN_HOME(Z)) return;
  #endif

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_MV(">>> homeaxis(", axis_codes[axis]);
      SERIAL_C(')'); SERIAL_E;
    }
  #endif

  int axis_home_dir =
    #if ENABLED(DUAL_X_CARRIAGE)
      (axis == X_AXIS) ? x_home_dir(active_extruder) :
    #endif
    home_dir(axis);

  #if ENABLED(LASERBEAM) && (LASER_HAS_FOCUS == false)
    if (axis == Z_AXIS) goto AvoidLaserFocus;
  #endif

  // Homing Z towards the bed? Deploy the Z probe or endstop.
  #if HOMING_Z_WITH_PROBE
    if (axis == Z_AXIS && DEPLOY_PROBE()) return;
  #endif

  // Set a flag for Z motor locking
  #if ENABLED(Z_DUAL_ENDSTOPS)
    if (axis == Z_AXIS) stepper.set_homing_flag(true);
  #endif

  // Fast move towards endstop until triggered
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) SERIAL_EM("Home 1 Fast:");
  #endif
  #if MECH(DELTA)
    do_homing_move(axis, 1.5 * max_length[axis] * axis_home_dir);
  #else
    do_homing_move(axis, 1.5 * max_length(axis) * axis_home_dir);
  #endif

  // When homing Z with probe respect probe clearance
  const float bump = axis_home_dir * (
    #if HOMING_Z_WITH_PROBE
      (axis == Z_AXIS) ? max(Z_PROBE_BETWEEN_HEIGHT, home_bump_mm(Z_AXIS)) :
    #endif
    home_bump_mm(axis)
  );

  // If a second homing move is configured...
  if (bump) {
    // Move away from the endstop by the axis HOME_BUMP_MM
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) SERIAL_EM("Move Away:");
    #endif
    do_homing_move(axis, -bump);

    // Slow move towards endstop until triggered
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) SERIAL_EM("Home 2 Slow:");
    #endif
    do_homing_move(axis, 2 * bump, get_homing_bump_feedrate(axis));
  }

  #if ENABLED(Z_DUAL_ENDSTOPS)
    if (axis == Z_AXIS) {
      float adj = fabs(z_endstop_adj);
      bool lockZ1;
      if (axis_home_dir > 0) {
        adj = -adj;
        lockZ1 = (z_endstop_adj > 0);
      }
      else
        lockZ1 = (z_endstop_adj < 0);

      if (lockZ1) stepper.set_z_lock(true); else stepper.set_z2_lock(true);

      // Move to the adjusted endstop height
      do_homing_move(axis, adj);

      if (lockZ1) stepper.set_z_lock(false); else stepper.set_z2_lock(false);
      stepper.set_homing_flag(false);
    } // Z_AXIS
  #endif

  #if IS_SCARA

    set_axis_is_at_home(axis);
    SYNC_PLAN_POSITION_KINEMATIC();

  #elif MECH(DELTA)
    // Delta has already moved all three towers up in G28
    // so here it re-homes each tower in turn.
    // Delta homing treats the axes as normal linear axes.

    // retrace by the amount specified in endstop_adj
    if (endstop_adj[axis] * Z_HOME_DIR < 0) {
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) SERIAL_EM("endstop_adj:");
      #endif
      do_homing_move(axis, endstop_adj[axis]);
    }

  #else

    // For cartesian/core machines,
    // set the axis to its home position
    set_axis_is_at_home(axis);
    sync_plan_position();

    destination[axis] = current_position[axis];

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("> AFTER set_axis_is_at_home", current_position);
    #endif

  #endif

  // Put away the Z probe
  #if HOMING_Z_WITH_PROBE
    if (axis == Z_AXIS && STOW_PROBE()) return;
  #endif

  #if ENABLED(LASERBEAM) && (LASER_HAS_FOCUS == false)
    AvoidLaserFocus:
  #endif

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_MV("<<< homeaxis(", axis_codes[axis]);
      SERIAL_C(')'); SERIAL_E;
    }
  #endif
} // homeaxis()

#if ENABLED(FWRETRACT)

  void retract(bool retracting, bool swapping = false) {

    if (retracting == retracted[active_extruder]) return;

    float old_feedrate_mm_s = feedrate_mm_s;

    set_destination_to_current();

    if (retracting) {
      feedrate_mm_s = retract_feedrate_mm_s;
      current_position[E_AXIS] += (swapping ? retract_length_swap : retract_length) / volumetric_multiplier[active_extruder];
      sync_plan_position_e();
      prepare_move_to_destination();

      if (retract_zlift > 0.01) {
        current_position[Z_AXIS] -= retract_zlift;
        SYNC_PLAN_POSITION_KINEMATIC();
        prepare_move_to_destination();
      }
    }
    else {
      if (retract_zlift > 0.01) {
        current_position[Z_AXIS] += retract_zlift;
        SYNC_PLAN_POSITION_KINEMATIC();
      }

      feedrate_mm_s = MMM_TO_MMS(retract_recover_feedrate_mm_s);
      float move_e = swapping ? retract_length_swap + retract_recover_length_swap : retract_length + retract_recover_length;
      current_position[E_AXIS] -= move_e / volumetric_multiplier[active_extruder];
      sync_plan_position_e();
      prepare_move_to_destination();
    }

    feedrate_mm_s = old_feedrate_mm_s;
    retracted[active_extruder] = retracting;

  }

#endif // FWRETRACT

#if ENABLED(COLOR_MIXING_EXTRUDER)

  void normalize_mix() {
    float mix_total = 0.0;
    for (int i = 0; i < MIXING_STEPPERS; i++) mix_total += RECIPROCAL(mixing_factor[i]);
    // Scale all values if they don't add up to ~1.0
    if (!NEAR(mix_total, 1.0)) {
      SERIAL_EM("Warning: Mix factors must add up to 1.0. Scaling.");
      for (int i = 0; i < MIXING_STEPPERS; i++) mixing_factor[i] *= mix_total;
    }
  }

  // Get mixing parameters from the GCode
  // Factors that are left out are set to 0
  // The total "must" be 1.0 (but it will be normalized)
  void gcode_get_mix() {
    const char* mixing_codes = "ABCDHI";
    for (int i = 0; i < MIXING_STEPPERS; i++) {
      float v = code_seen(mixing_codes[i]) ? code_value_float() : 0.0;
      NOLESS(v, 0.0);
      mixing_factor[i] = RECIPROCAL(v);
    }
    normalize_mix();
  }

#endif

#if ENABLED(IDLE_OOZING_PREVENT)

  void IDLE_OOZING_retract(bool retracting) {  
    if (retracting && !IDLE_OOZING_retracted[active_extruder]) {
      float old_feedrate_mm_s = feedrate_mm_s;
      set_destination_to_current();
      current_position[E_AXIS] += IDLE_OOZING_LENGTH / volumetric_multiplier[active_extruder];
      feedrate_mm_s = IDLE_OOZING_FEEDRATE;
      planner.set_e_position_mm(current_position[E_AXIS]);
      prepare_move_to_destination();
      feedrate_mm_s = old_feedrate_mm_s;
      IDLE_OOZING_retracted[active_extruder] = true;
      //SERIAL_EM("-");
    }
    else if (!retracting && IDLE_OOZING_retracted[active_extruder]) {
      float old_feedrate_mm_s = feedrate_mm_s;
      set_destination_to_current();
      current_position[E_AXIS] -= (IDLE_OOZING_LENGTH+IDLE_OOZING_RECOVER_LENGTH) / volumetric_multiplier[active_extruder];
      feedrate_mm_s = IDLE_OOZING_RECOVER_FEEDRATE;
      planner.set_e_position_mm(current_position[E_AXIS]);
      prepare_move_to_destination();
      feedrate_mm_s = old_feedrate_mm_s;
      IDLE_OOZING_retracted[active_extruder] = false;
      //SERIAL_EM("+");
    }
  }

#endif

#if HAS(TEMP_0) || HAS(TEMP_BED) || ENABLED(HEATER_0_USES_MAX6675)
  void print_heaterstates() {
    #if HAS(TEMP_0) || ENABLED(HEATER_0_USES_MAX6675)
      SERIAL_MV(MSG_T, thermalManager.degHotend(target_extruder), 1);
      SERIAL_MV(" /", thermalManager.degTargetHotend(target_extruder), 1);
      #if ENABLED(SHOW_TEMP_ADC_VALUES)
        SERIAL_MV(" (", thermalManager.rawHotendTemp(target_extruder) / OVERSAMPLENR);
        SERIAL_C(')');
      #endif
    #endif
    #if HAS(TEMP_BED)
      SERIAL_MV(MSG_B, thermalManager.degBed(), 1);
      SERIAL_MV(" /", thermalManager.degTargetBed(), 1);
      #if ENABLED(SHOW_TEMP_ADC_VALUES)
        SERIAL_MV(" (", thermalManager.rawBedTemp() / OVERSAMPLENR);
        SERIAL_C(')');
      #endif
    #endif
    #if HOTENDS > 1
      for (uint8_t h = 0; h < HOTENDS; h++) {
        SERIAL_MV(" T", h);
        SERIAL_C(':');
        SERIAL_V(thermalManager.degHotend(h), 1);
        SERIAL_MV(" /", thermalManager.degTargetHotend(h), 1);
        #if ENABLED(SHOW_TEMP_ADC_VALUES)
          SERIAL_MV(" (", thermalManager.rawHotendTemp(h) / OVERSAMPLENR);
          SERIAL_C(')');
        #endif
      }
    #endif
    SERIAL_MV(MSG_AT ":", thermalManager.getHeaterPower(target_extruder));
    #if HAS(TEMP_BED)
      SERIAL_MV(MSG_BAT, thermalManager.getBedPower());
    #endif
    #if HOTENDS > 1
      for (uint8_t h = 0; h < HOTENDS; h++) {
        SERIAL_MV(MSG_AT, h);
        SERIAL_C(':');
        SERIAL_V(thermalManager.getHeaterPower(h));
      }
    #endif
  }
#endif

#if HAS(TEMP_CHAMBER)
  void print_chamberstate() {
    SERIAL_M(" CHAMBER: ");
    SERIAL_MV(MSG_C, degChamber(), 1);
    SERIAL_MV(" /", degTargetChamber(), 1);
    SERIAL_M(MSG_CAT);
    #if ENABLED(CHAMBER_WATTS)
      SERIAL_V(((CHAMBER_WATTS) * thermalManager.getChamberPower()) / 127.0);
      SERIAL_M("W");
    #else
      SERIAL_V(thermalManager.getChamberPower());
    #endif
    #if ENABLED(SHOW_TEMP_ADC_VALUES)
      SERIAL_MV("    ADC C:", degChamber(), 1);
      SERIAL_MV("C->", thermalManager.rawChamberTemp() / OVERSAMPLENR, 1);
    #endif
  }
#endif // HAS(TEMP_CHAMBER)

#if HAS(TEMP_COOLER)
  void print_coolerstate() {
    SERIAL_M(" COOL: ");
    SERIAL_MV(MSG_C, degCooler(), 1);
    SERIAL_MV(" /", degTargetCooler(), 1);
    SERIAL_M(MSG_CAT);
    #if ENABLED(COOLER_WATTS)
      SERIAL_V(((COOLER_WATTS) * thermalManager.getCoolerPower()) / 127.0);
      SERIAL_M("W");
    #else
      SERIAL_V(thermalManager.getCoolerPower());
    #endif
    #if ENABLED(SHOW_TEMP_ADC_VALUES)
      SERIAL_MV("    ADC C:", degCooler(), 1);
      SERIAL_MV("C->", thermalManager.rawCoolerTemp() / OVERSAMPLENR, 0);
    #endif
  }
#endif // HAS(TEMP_COOLER)

#if ENABLED(FLOWMETER_SENSOR)
  void print_flowratestate() {
    float readval = get_flowrate();

    #if ENABLED(MINFLOW_PROTECTION)
      if(readval > MINFLOW_PROTECTION)
        flow_firstread = true;
    #endif

    SERIAL_MV(" FLOW: ", readval);
    SERIAL_M(" l/min ");
  }
#endif

#ifndef MIN_COOLING_SLOPE_DEG
  #define MIN_COOLING_SLOPE_DEG 1.50
#endif
#ifndef MIN_COOLING_SLOPE_TIME
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
  #endif // TEMP_RESIDENCY_TIME

  float theTarget = -1.0, old_temp = 9999.0;
  bool wants_to_cool = false;
  wait_for_heatup = true;
  millis_t now, next_temp_ms = 0, next_cool_check_ms = 0;

  KEEPALIVE_STATE(NOT_BUSY);

  do {
    // Target temperature might be changed during the loop
    if (theTarget != thermalManager.degTargetHotend(target_extruder)) {
      wants_to_cool = thermalManager.isCoolingHotend(target_extruder);
      theTarget = thermalManager.degTargetHotend(target_extruder);

      // Exit if S<lower>, continue if S<higher>, R<lower>, or R<higher>
      if (no_wait_for_cooling && wants_to_cool) break;
    }

    now = millis();
    if (ELAPSED(now, next_temp_ms)) { // Print temp & remaining time every 1s while waiting
      next_temp_ms = now + 1000UL;
      print_heaterstates();
      #if TEMP_RESIDENCY_TIME > 0
        SERIAL_M(MSG_W);
        if (residency_start_ms) {
          long rem = ((TEMP_RESIDENCY_TIME * 1000UL) - (now - residency_start_ms)) / 1000UL;
          SERIAL_EV(rem);
        }
        else {
          SERIAL_EM("?");
        }
      #else
        SERIAL_E;
      #endif
    }

    idle();
    refresh_cmd_timeout(); // to prevent stepper_inactive_time from running out

    float temp = thermalManager.degHotend(target_extruder);

    #if TEMP_RESIDENCY_TIME > 0

      float temp_diff = fabs(theTarget - temp);

      if (!residency_start_ms) {
        // Start the TEMP_RESIDENCY_TIME timer when we reach target temp for the first time.
        if (temp_diff < TEMP_WINDOW) residency_start_ms = now;
      }
      else if (temp_diff > TEMP_HYSTERESIS) {
        // Restart the timer whenever the temperature falls outside the hysteresis.
        residency_start_ms = now;
      }

    #endif // TEMP_RESIDENCY_TIME > 0

    // Prevent a wait-forever situation if R is misused i.e. M109 R0
    if (wants_to_cool) {
      // break after MIN_COOLING_SLOPE_TIME seconds
      // if the temperature did not drop at least MIN_COOLING_SLOPE_DEG
      if (!next_cool_check_ms || ELAPSED(now, next_cool_check_ms)) {
        if (old_temp - temp < MIN_COOLING_SLOPE_DEG) break;
        next_cool_check_ms = now + 1000UL * MIN_COOLING_SLOPE_TIME;
        old_temp = temp;
      }
    }

  } while (wait_for_heatup && TEMP_CONDITIONS);

  if (wait_for_heatup) LCD_MESSAGEPGM(MSG_HEATING_COMPLETE);

  KEEPALIVE_STATE(IN_HANDLER);
}

#if HAS(TEMP_BED)

  #ifndef MIN_COOLING_SLOPE_DEG_BED
    #define MIN_COOLING_SLOPE_DEG_BED 1.50
  #endif
  #ifndef MIN_COOLING_SLOPE_TIME_BED
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
    #endif // TEMP_BED_RESIDENCY_TIME > 0

    float theTarget = -1.0, old_temp = 9999.0;
    bool wants_to_cool = false;
    wait_for_heatup = true;
    millis_t now, next_temp_ms = 0, next_cool_check_ms = 0;

    KEEPALIVE_STATE(NOT_BUSY);

    target_extruder = active_extruder; // for print_heaterstates

    // Wait for temperature to come close enough
    do {
      // Target temperature might be changed during the loop
      if (theTarget != thermalManager.degTargetBed())
        theTarget = thermalManager.degTargetBed();

      wants_to_cool = thermalManager.isCoolingBed();

      // Exit if S<lower>, continue if S<higher>, R<lower>, or R<higher>
      if (no_wait_for_cooling && wants_to_cool) break;

      now = millis();
      if (ELAPSED(now, next_temp_ms)) { // Print Temp Reading every 1 second while heating up.
        next_temp_ms = now + 1000UL;
        print_heaterstates();
        #if TEMP_BED_RESIDENCY_TIME > 0
          SERIAL_M(MSG_W);
          if (residency_start_ms) {
            long rem = (((TEMP_BED_RESIDENCY_TIME) * 1000UL) - (now - residency_start_ms)) / 1000UL;
            SERIAL_EV(rem);
          }
          else {
            SERIAL_EM("?");
          }
        #else
          SERIAL_E;
        #endif
      }

      idle();
      refresh_cmd_timeout(); // to prevent stepper_inactive_time from running out

      float temp = thermalManager.degBed();

      #if TEMP_BED_RESIDENCY_TIME > 0

        float temp_diff = fabs(theTarget - temp);

        if (!residency_start_ms) {
          // Start the TEMP_BED_RESIDENCY_TIME timer when we reach target temp for the first time.
          if (temp_diff < TEMP_BED_WINDOW) residency_start_ms = now;
        }
        else if (temp_diff > TEMP_BED_HYSTERESIS) {
          // Restart the timer whenever the temperature falls outside the hysteresis.
          residency_start_ms = now;
        }

      #endif //TEMP_BED_RESIDENCY_TIME > 0

      // Prevent a wait-forever situation if R is misused i.e. M190 R0
      if (wants_to_cool) {
        // break after MIN_COOLING_SLOPE_TIME_BED seconds
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
#endif // HAS(TEMP_BED)

#if HAS(TEMP_CHAMBER)
  inline void wait_chamber(bool no_wait_for_heating = true) {
    #if TEMP_CHAMBER_RESIDENCY_TIME > 0
      millis_t residency_start_ms = 0;
      // Loop until the temperature has stabilized
      #define TEMP_CHAMBER_CONDITIONS (!residency_start_ms || PENDING(now, residency_start_ms + (TEMP_CHAMBER_RESIDENCY_TIME) * 1000UL))
    #else
      // Loop until the temperature is very close target
      #define TEMP_CHAMBER_CONDITIONS (wants_to_heat ? thermalManager.isHeatingChamber() : thermalManager.isCoolingChamber())
    #endif

    float theTarget = -1;
    bool wants_to_heat;
    wait_for_heatup = true;
    millis_t now, next_temp_ms = 0;

    KEEPALIVE_STATE(NOT_BUSY);

    // Wait for temperature to come close enough
    do {
      // Target temperature might be changed during the loop
      if (theTarget != degTargetChamber())
        theTarget = degTargetChamber();

      wants_to_heat = thermalManager.isHeatingChamber();

      // Exit if S<higher>, continue if S<lower>, R<higher>, or R<lower>
      if (no_wait_for_heating && wants_to_heat) break;

      now = millis();
      if (ELAPSED(now, next_temp_ms)) { // Print Temp Reading every 1 second while heating up.
        next_temp_ms = now + 1000UL;
        print_chamberstate();
        #if TEMP_CHAMBER_RESIDENCY_TIME > 0
          SERIAL_M(MSG_W);
          if (residency_start_ms) {
            long rem = (((TEMP_CHAMBER_RESIDENCY_TIME) * 1000UL) - (now - residency_start_ms)) / 1000UL;
            SERIAL_EV(rem);
          }
          else {
            SERIAL_EM("?");
          }
        #else
          SERIAL_E;
        #endif
      }

		idle();
		refresh_cmd_timeout(); // to prevent stepper_inactive_time from running out
	
      #if TEMP_CHAMBER_RESIDENCY_TIME > 0

        float temp_diff = fabs(theTarget - degTargetChamber());

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

#if HAS(TEMP_COOLER)
  inline void wait_cooler(bool no_wait_for_heating = true) {
    #if TEMP_COOLER_RESIDENCY_TIME > 0
      millis_t residency_start_ms = 0;
      // Loop until the temperature has stabilized
      #define TEMP_COOLER_CONDITIONS (!residency_start_ms || PENDING(now, residency_start_ms + (TEMP_COOLER_RESIDENCY_TIME) * 1000UL))
    #else
      // Loop until the temperature is very close target
      #define TEMP_COOLER_CONDITIONS (wants_to_heat ? thermalManager.isHeatingCooler() : thermalManager.isCoolingCooler())
    #endif

    float theTarget = -1;
    bool wants_to_heat;
    wait_for_heatup = true;
    millis_t now, next_temp_ms = 0;

    KEEPALIVE_STATE(NOT_BUSY);

    // Wait for temperature to come close enough
    do {
      // Target temperature might be changed during the loop
      if (theTarget != degTargetCooler())
        theTarget = degTargetCooler();

      wants_to_heat = thermalManager.isHeatingCooler();

      // Exit if S<higher>, continue if S<lower>, R<higher>, or R<lower>
      if (no_wait_for_heating && wants_to_heat) break;

      // Prevent a wait-forever situation if R is misused i.e. M190 C R50
      // Simply don't wait to heat a cooler over 25C
      if (wants_to_heat && theTarget > 25) break;

      now = millis();
      if (ELAPSED(now, next_temp_ms)) { //Print Temp Reading every 1 second while heating up.
        next_temp_ms = now + 1000UL;
        print_coolerstate();
        #if TEMP_COOLER_RESIDENCY_TIME > 0
          SERIAL_M(MSG_W);
          if (residency_start_ms) {
            long rem = (((TEMP_COOLER_RESIDENCY_TIME) * 1000UL) - (now - residency_start_ms)) / 1000UL;
            SERIAL_EV(rem);
          }
          else {
            SERIAL_EM("?");
          }
        #else
          SERIAL_E;
        #endif
      }

		idle();
		refresh_cmd_timeout(); // to prevent stepper_inactive_time from running out
	
      #if TEMP_COOLER_RESIDENCY_TIME > 0

        float temp_diff = fabs(theTarget - degTargetCooler());

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
 * Set XYZE destination and feedrate_mm_s from the current GCode command
 *
 *  - Set destination from included axis codes
 *  - Set to current for missing axis codes
 *  - Set the feedrate_mm_s, if included
 */
void gcode_get_destination() {
  #if ENABLED(IDLE_OOZING_PREVENT)
    if(code_seen('E') IDLE_OOZING_retract(false);
  #endif

  LOOP_XYZE(i) {
    if (code_seen(axis_codes[i]))
      destination[i] = code_value_axis_units(i) + (axis_relative_modes[i] || relative_mode ? current_position[i] : 0);
    else
      destination[i] = current_position[i];
  }

  if (code_seen('F') && code_value_linear_units() > 0.0)
    feedrate_mm_s = MMM_TO_MMS(code_value_linear_units());

  if (code_seen('P'))
    destination[E_AXIS] = (code_value_axis_units(E_AXIS) * density_percentage[previous_extruder] / 100) + current_position[E_AXIS];

  if(!DEBUGGING(DRYRUN))
    print_job_counter.data.filamentUsed += (destination[E_AXIS] - current_position[E_AXIS]);

  #if ENABLED(COLOR_MIXING_EXTRUDER)
    gcode_get_mix();
  #endif

  #if ENABLED(RFID_MODULE)
    RFID522.RfidData[active_extruder].data.lenght -= (destination[E_AXIS] - current_position[E_AXIS]);
  #endif

  #if ENABLED(NEXTION) && ENABLED(NEXTION_GFX)
    #if MECH(DELTA)
      if((code_seen('X') || code_seen('Y')) && code_seen('E'))
        gfx_line_to(destination[X_AXIS] + (X_MAX_POS), destination[Y_AXIS] + (Y_MAX_POS), destination[Z_AXIS]);
      else
        gfx_cursor_to(destination[X_AXIS] + (X_MAX_POS), destination[Y_AXIS] + (Y_MAX_POS), destination[Z_AXIS]);
    #else
      if((code_seen('X') || code_seen('Y')) && code_seen('E'))
        gfx_line_to(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS]);
      else
        gfx_cursor_to(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS]);
    #endif
  #endif
}

void unknown_command_error() {
  SERIAL_SMV(ER, MSG_UNKNOWN_COMMAND, current_command);
  SERIAL_EM("\"");
}

#if ENABLED(HOST_KEEPALIVE_FEATURE)
  /**
   * Output a "busy" message at regular intervals
   * while the machine is not accepting commands.
   */
  void host_keepalive() {
    millis_t ms = millis();
    if (host_keepalive_interval && busy_state != NOT_BUSY) {
      if (PENDING(ms, next_busy_signal_ms)) return;
      switch (busy_state) {
        case IN_HANDLER:
        case IN_PROCESS:
          SERIAL_LM(BUSY, MSG_BUSY_PROCESSING);
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
    next_busy_signal_ms = ms + host_keepalive_interval * 1000UL;
  }

#endif //HOST_KEEPALIVE_FEATURE

bool position_is_reachable(float target[XYZ]
  #if HAS(BED_PROBE)
    , bool by_probe=false
  #endif
) {
  float dx = target[X_AXIS], dy = target[Y_AXIS];

  #define WITHINXY(x,y) ((x) >= X_MIN_POS - 0.0001 && (x) <= X_MAX_POS + 0.0001 && (y) >= Y_MIN_POS - 0.0001 && (y) <= Y_MAX_POS + 0.0001)
  #define WITHINZ(z) ((z) >= Z_MIN_POS - 0.0001 && (z) <= Z_MAX_POS + 0.0001)

  #if MECH(MAKERARM_SCARA)
    if (by_probe) {
      // If the returned point is 0,0,0 the radius test will fail
      vector_3 point = probe_point_to_end_point(dx, dy);
      // Is the tool point outside the rectangular bounds?
      if (!WITHINXY(point.x, point.y)) {
        // Try the opposite arm orientation
        arm_orientation = !arm_orientation;
        point = probe_point_to_end_point(dx, dy);
        // If still unreachable keep the old arm orientation
        if (!WITHINXY(point.x, point.y)) arm_orientation = !arm_orientation;
      }
      dx = point.x;
      dy = point.y;
    }
  #elif HAS(BED_PROBE)
    if (by_probe) {
      dx -= X_PROBE_OFFSET_FROM_NOZZLE;
      dy -= Y_PROBE_OFFSET_FROM_NOZZLE;
    }
  #endif

  dx -= LOGICAL_X_POSITION(0);
  dy -= LOGICAL_Y_POSITION(0);

  const float dz = RAW_Z_POSITION(target[Z_AXIS]);

  #if IS_SCARA
    #if MIDDLE_DEAD_ZONE_R > 0
      const float R2 = HYPOT2(dx - SCARA_OFFSET_X, dy - SCARA_OFFSET_Y);
      bool good = WITHINZ(dz) && (R2 >= sq(float(MIDDLE_DEAD_ZONE_R))) && (R2 <= sq(L1 + L2));
      return good;
    #else
      return WITHINZ(dz) && HYPOT2(dx - (SCARA_OFFSET_X), dy - (SCARA_OFFSET_Y)) <= sq(L1 + L2);
    #endif
  #elif MECH(DELTA)
    return WITHINZ(dz) && HYPOT2(dx, dy) <= sq(DELTA_PRINTABLE_RADIUS);
  #else
    return WITHINZ(dz) && WITHINXY(dx, dy);
  #endif
}

/**************************************************
 ***************** GCode Handlers *****************
 **************************************************/

/**
 * G0, G1: Coordinated movement of X Y Z E axes
 */
inline void gcode_G0_G1(
  #if IS_SCARA
    bool fast_move = false
  #elif ENABLED(LASERBEAM)
    bool lfire = false
  #endif
) {
  if (IsRunning()) {
    gcode_get_destination(); // For X Y Z E F

    #if ENABLED(FWRETRACT)
      if (autoretract_enabled && !(code_seen('X') || code_seen('Y') || code_seen('Z')) && code_seen('E')) {
        float echange = destination[E_AXIS] - current_position[E_AXIS];
        // Is this move an attempt to retract or recover?
        if ((echange < -MIN_RETRACT && !retracted[active_extruder]) || (echange > MIN_RETRACT && retracted[active_extruder])) {
          current_position[E_AXIS] = destination[E_AXIS]; // hide the slicer-generated retract/recover from calculations
          sync_plan_position_e();  // AND from the planner
          retract(!retracted[active_extruder]);
          return;
        }
      }
    #endif // FWRETRACT

    #if ENABLED(LASERBEAM) && ENABLED(LASER_FIRE_G1)
      if (lfire) {
        if (code_seen('S') && IsRunning()) laser.intensity = code_value_float();
        if (code_seen('L') && IsRunning()) laser.duration = code_value_ulong();
        if (code_seen('P') && IsRunning()) laser.ppm = code_value_float();
        if (code_seen('D') && IsRunning()) laser.diagnostics = code_value_bool();
        if (code_seen('B') && IsRunning()) laser_set_mode(code_value_int());

        laser.status = LASER_ON;
        laser.fired = LASER_FIRE_G1;
      }
    #endif

    #if IS_SCARA
      fast_move ? prepare_uninterpolated_move_to_destination() : prepare_move_to_destination();
    #else
      prepare_move_to_destination();
    #endif

    #if ENABLED(LASERBEAM) && ENABLED(LASER_FIRE_G1)
      if (lfire) laser.status = LASER_OFF;
    #endif

  }
}

/**
 * G2: Clockwise Arc
 * G3: Counterclockwise Arc
 */
#if ENABLED(ARC_SUPPORT)
  inline void gcode_G2_G3(bool clockwise) {
    if (IsRunning()) {

      #if ENABLED(SF_ARC_FIX)
        bool relative_mode_backup = relative_mode;
        relative_mode = true;
      #endif

      gcode_get_destination();

      #if ENABLED(LASERBEAM) && ENABLED(LASER_FIRE_G1)
        if (code_seen('S') && IsRunning()) laser.intensity = code_value_float();
        if (code_seen('L') && IsRunning()) laser.duration = code_value_ulong();
        if (code_seen('P') && IsRunning()) laser.ppm = code_value_float();
        if (code_seen('D') && IsRunning()) laser.diagnostics = code_value_bool();
        if (code_seen('B') && IsRunning()) laser_set_mode(code_value_int());

        laser.status = LASER_ON;
        laser.fired = LASER_FIRE_G1;
      #endif

      #if ENABLED(SF_ARC_FIX)
        relative_mode = relative_mode_backup;
      #endif

      float arc_offset[2] = { 0.0, 0.0 };
      if (code_seen('R')) {
        const float r = code_value_axis_units(X_AXIS),
                    x1 = current_position[X_AXIS], y1 = current_position[Y_AXIS],
                    x2 = destination[X_AXIS], y2 = destination[Y_AXIS];
        if (r && (x2 != x1 || y2 != y1)) {
          const float e = clockwise ^ (r < 0) ? -1 : 1,           // clockwise -1/1, counterclockwise 1/-1
                      dx = x2 - x1, dy = y2 - y1,                 // X and Y differences
                      d = HYPOT(dx, dy),                          // Linear distance between the points
                      h = sqrt(sq(r) - sq(d * 0.5)),              // Distance to the arc pivot-point
                      mx = (x1 + x2) * 0.5, my = (y1 + y2) * 0.5, // Point between the two points
                      sx = -dy / d, sy = dx / d,                  // Slope of the perpendicular bisector
                      cx = mx + e * h * sx, cy = my + e * h * sy; // Pivot-point of the arc
          arc_offset[X_AXIS] = cx - x1;
          arc_offset[Y_AXIS] = cy - y1;
        }
      }
      else {
        if (code_seen('I')) arc_offset[X_AXIS] = code_value_axis_units(X_AXIS);
        if (code_seen('J')) arc_offset[Y_AXIS] = code_value_axis_units(Y_AXIS);
      }

      if (arc_offset[0] || arc_offset[1]) {
        // Send an arc to the planner
        plan_arc(destination, arc_offset, clockwise);
        refresh_cmd_timeout();
      }
      else {
        // Bad arguments
        SERIAL_LM(ER, MSG_ERR_ARC_ARGS);
      }
    }
  }
#endif // ARC_SUPPORT

/**
 * G4: Dwell S<seconds> or P<milliseconds>
 */
inline void gcode_G4() {
  millis_t codenum = 0;

  if (code_seen('P')) codenum = code_value_millis(); // milliseconds to wait
  if (code_seen('S')) codenum = code_value_millis_from_seconds(); // seconds to wait

  stepper.synchronize();
  refresh_cmd_timeout();
  codenum += previous_cmd_ms;  // keep track of when we started waiting

  if (!lcd_hasstatus()) LCD_MESSAGEPGM(MSG_DWELL);

  while (PENDING(millis(), codenum)) idle();
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

      float offset[] = {
        code_seen('I') ? code_value_axis_units(X_AXIS) : 0.0,
        code_seen('J') ? code_value_axis_units(Y_AXIS) : 0.0,
        code_seen('P') ? code_value_axis_units(X_AXIS) : 0.0,
        code_seen('Q') ? code_value_axis_units(Y_AXIS) : 0.0
      };

      plan_cubic_move(offset);
    }
  }
#endif

#if ENABLED(LASERBEAM) && ENABLED(LASER_RASTER)
  inline void gcode_G7() {

    if (code_seen('L')) laser.raster_raw_length = code_value_int();

    if (code_seen('$')) {
      laser.raster_direction = code_value_bool();
      destination[Y_AXIS] = current_position[Y_AXIS] + (laser.raster_mm_per_pulse * laser.raster_aspect_ratio); // increment Y axis
    }

    if (code_seen('D')) laser.raster_num_pixels = base64_decode(laser.raster_data, seen_pointer + 1, laser.raster_raw_length);

    if (!laser.raster_direction) {
      destination[X_AXIS] = current_position[X_AXIS] - (laser.raster_mm_per_pulse * laser.raster_num_pixels);
      if (laser.diagnostics)
        SERIAL_EM("Negative Raster Line");
    }
    else {
      destination[X_AXIS] = current_position[X_AXIS] + (laser.raster_mm_per_pulse * laser.raster_num_pixels);
      if (laser.diagnostics)
        SERIAL_EM("Positive Raster Line");
    }

    laser.ppm = 1 / laser.raster_mm_per_pulse; // number of pulses per millimetre
    laser.duration = (1000000 / feedrate_mm_s) / laser.ppm; // (1 second in microseconds / (time to move 1mm in microseconds)) / (pulses per mm) = Duration of pulse, taking into account feedrate_mm_s as speed and ppm

    laser.mode = RASTER;
    laser.status = LASER_ON;
    laser.fired = RASTER;
    prepare_move_to_destination();
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
        retracted_swap[active_extruder] = (code_seen('S') && code_value_bool()); // checks for swap retract argument
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
    if (axis_unhomed_error(true, true, true)) { return; }

    uint8_t const pattern = code_seen('P') ? code_value_ushort() : 0;
    uint8_t const strokes = code_seen('S') ? code_value_ushort() : NOZZLE_CLEAN_STROKES;
    uint8_t const objects = code_seen('T') ? code_value_ushort() : 3;

    Nozzle::clean(pattern, strokes, objects);
  }
#endif

#if ENABLED(INCH_MODE_SUPPORT)
  /**
   * G20: Set input mode to inches
   */
  inline void gcode_G20() {
    set_input_linear_units(LINEARUNIT_INCH);
  }

  /**
   * G21: Set input mode to millimeters
   */
  inline void gcode_G21() {
    set_input_linear_units(LINEARUNIT_MM);
  }
#endif

#if ENABLED(NOZZLE_PARK_FEATURE)
  /**
   * G27: Park the nozzle
   */
  inline void gcode_G27() {
    // Don't allow nozzle parking without homing first
    if (axis_unhomed_error(true, true, true)) { return; }
    uint8_t const z_action = code_seen('P') ? code_value_ushort() : 0;
    Nozzle::park(z_action);
  }
#endif // NOZZLE_PARK_FEATURE

#if ENABLED(QUICK_HOME)
  static void quick_home_xy() {

    // Pretend the current position is 0,0
    current_position[X_AXIS] = current_position[Y_AXIS] = 0;
    sync_plan_position();

    #if ENABLED(DUAL_X_CARRIAGE)
      int x_axis_home_dir = x_home_dir(active_extruder);
    #else
      int x_axis_home_dir = home_dir(X_AXIS);
    #endif

    float mlx = max_length(X_AXIS),
          mly = max_length(Y_AXIS),
          mlratio = mlx > mly ? mly / mlx : mlx / mly,
          fr_mm_s = min(homing_feedrate_mm_s[X_AXIS], homing_feedrate_mm_s[Y_AXIS]) * sqrt(sq(mlratio) + 1.0);

    do_blocking_move_to_xy(1.5 * mlx * x_axis_home_dir, 1.5 * mly * home_dir(Y_AXIS), fr_mm_s);
    endstops.hit_on_purpose(); // clear endstop hit flags
    current_position[X_AXIS] = current_position[Y_AXIS] = 0.0;
  }
#endif // QUICK_HOME

#if ENABLED(DEBUG_LEVELING_FEATURE)

  void log_machine_info() {
    SERIAL_M("Machine Type: ");
    #if MECH(DELTA)
      SERIAL_EM("Delta");
    #elif IS_SCARA
      SERIAL_EM("SCARA");
    #elif IS_CORE
      SERIAL_EM("Core");
    #else
      SERIAL_EM("Cartesian");
    #endif

    SERIAL_M("Probe: ");
    #if ENABLED(Z_PROBE_FIX_MOUNTED)
      SERIAL_EM("Z_PROBE_FIX_MOUNTED");
    #elif ENABLED(BLTOUCH)
      SERIAL_EM("BLTOUCH");
    #elif ENABLED(Z_PROBE_SLED)
      SERIAL_EM("Z_PROBE_SLED");
    #elif ENABLED(Z_PROBE_ALLEN_KEY)
      SERIAL_EM("ALLEN KEY");
    #elif HAS(Z_SERVO_ENDSTOP)
      SERIAL_EM("SERVO PROBE");
    #else
      SERIAL_EM("NONE");
    #endif

    #if HAS(BED_PROBE)
      SERIAL_MV("Probe Offset X:", X_PROBE_OFFSET_FROM_NOZZLE);
      SERIAL_MV(" Y:", Y_PROBE_OFFSET_FROM_NOZZLE);
      SERIAL_MV(" Z:", zprobe_zoffset);
      #if (X_PROBE_OFFSET_FROM_NOZZLE > 0)
        SERIAL_M(" (Right");
      #elif (X_PROBE_OFFSET_FROM_NOZZLE < 0)
        SERIAL_M(" (Left");
      #elif (Y_PROBE_OFFSET_FROM_NOZZLE != 0)
        SERIAL_M(" (Middle");
      #else
        SERIAL_M(" (Aligned With");
      #endif
      #if (Y_PROBE_OFFSET_FROM_NOZZLE > 0)
        SERIAL_M("-Back");
      #elif (Y_PROBE_OFFSET_FROM_NOZZLE < 0)
        SERIAL_M("-Front");
      #elif (X_PROBE_OFFSET_FROM_NOZZLE != 0)
        SERIAL_M("-Center");
      #endif
      if (zprobe_zoffset < 0)
        SERIAL_M(" & Below");
      else if (zprobe_zoffset > 0)
        SERIAL_M(" & Above");
      else
        SERIAL_M(" & Same Z as");
      SERIAL_EM(" Nozzle)");
    #endif

    #if HAS(ABL)
      SERIAL_M("Auto Bed Leveling: ");
      #if ENABLED(AUTO_BED_LEVELING_LINEAR)
        SERIAL_M("LINEAR");
      #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
        SERIAL_M("BILINEAR");
      #elif ENABLED(AUTO_BED_LEVELING_3POINT)
        SERIAL_M("3POINT");
      #endif
      if (planner.abl_enabled) {
        SERIAL_EM(" (enabled)");
        #if ENABLED(AUTO_BED_LEVELING_LINEAR) || ENABLED(AUTO_BED_LEVELING_3POINT)
          float diff[XYZ] = {
            stepper.get_axis_position_mm(X_AXIS) - current_position[X_AXIS],
            stepper.get_axis_position_mm(Y_AXIS) - current_position[Y_AXIS],
            stepper.get_axis_position_mm(Z_AXIS) - current_position[Z_AXIS]
          };
          SERIAL_M("ABL Adjustment X");
          if (diff[X_AXIS] > 0) SERIAL_C('+');
          SERIAL_V(diff[X_AXIS]);
          SERIAL_M(" Y");
          if (diff[Y_AXIS] > 0) SERIAL_C('+');
          SERIAL_V(diff[Y_AXIS]);
          SERIAL_M(" Z");
          if (diff[Z_AXIS] > 0) SERIAL_C('+');
          SERIAL_V(diff[Z_AXIS]);
        #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
          SERIAL_MV("ABL Adjustment Z", bilinear_z_offset(current_position));
        #endif
      }
      SERIAL_E;
    #elif ENABLED(MESH_BED_LEVELING)
      SERIAL_M("Mesh Bed Leveling");
      if (mbl.active()) {
        SERIAL_EM(" (enabled)");
        SERIAL_MV("MBL Adjustment Z", mbl.get_z(RAW_CURRENT_POSITION(X_AXIS), RAW_CURRENT_POSITION(Y_AXIS)));
      }
      SERIAL_E;
    #endif

  }

#endif // DEBUG_LEVELING_FEATURE

#if MECH(DELTA)

  /**
   * A delta can only safely home all axes at the same time
   * This is like quick_home_xy() but for 3 towers.
   */
  inline void home_delta() {

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS(">>> home_delta", current_position);
    #endif

    // Init the current position of all carriages to 0,0,0
    ZERO(current_position);
    sync_plan_position();

    // Move all carriages together linearly until an endstop is hit.
    current_position[X_AXIS] = current_position[Y_AXIS] = current_position[Z_AXIS] = max_length[Z_AXIS] + 10;
    feedrate_mm_s = homing_feedrate_mm_s[X_AXIS];
    line_to_current_position();
    stepper.synchronize();
    endstops.hit_on_purpose(); // clear endstop hit flags

    // At least one carriage has reached the top.
    // Now re-home each carriage separately.
    HOMEAXIS(A);
    HOMEAXIS(B);
    HOMEAXIS(C);

    // Set all carriages to their home positions
    // Do this here all at once for Delta, because
    // XYZ isn't ABC. Applying this per-tower would
    // give the impression that they are the same.
    LOOP_XYZ(i) set_axis_is_at_home((AxisEnum)i);

    SYNC_PLAN_POSITION_KINEMATIC();

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("<<< home_delta", current_position);
    #endif

    // move to a height where we can use the full xy-area
    do_blocking_move_to_z(delta_clip_start_height);

  }
#endif // DELTA

#if ENABLED(Z_SAFE_HOMING)

  inline void home_z_safely() {

    // Disallow Z homing if X or Y are unknown
    if (!axis_known_position[X_AXIS] || !axis_known_position[Y_AXIS]) {
      LCD_MESSAGEPGM(MSG_ERR_Z_HOMING);
      SERIAL_LM(ECHO, MSG_ERR_Z_HOMING);
      return;
    }

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) SERIAL_EM("Z_SAFE_HOMING >>>");
    #endif

    SYNC_PLAN_POSITION_KINEMATIC();

    /**
     * Move the Z probe (or just the nozzle) to the safe homing point
     */
    destination[X_AXIS] = LOGICAL_X_POSITION(Z_SAFE_HOMING_X_POINT);
    destination[Y_AXIS] = LOGICAL_Y_POSITION(Z_SAFE_HOMING_Y_POINT);
    destination[Z_AXIS] = current_position[Z_AXIS]; // Z is already at the right height

    if (position_is_reachable(
          destination
          #if HOMING_Z_WITH_PROBE
            , true
          #endif
        )
    ) {
      #if HOMING_Z_WITH_PROBE
        destination[X_AXIS] -= X_PROBE_OFFSET_FROM_NOZZLE;
        destination[Y_AXIS] -= Y_PROBE_OFFSET_FROM_NOZZLE;
      #endif
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) DEBUG_POS("Z_SAFE_HOMING", destination);
      #endif
      do_blocking_move_to_xy(destination[X_AXIS], destination[Y_AXIS]);
      HOMEAXIS(Z);
    }
    else {
      LCD_MESSAGEPGM(MSG_ZPROBE_OUT);
      SERIAL_LM(ECHO, MSG_ZPROBE_OUT);
    }

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) SERIAL_EM("<<< Z_SAFE_HOMING");
    #endif
  }
#endif // Z_SAFE_HOMING

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
inline void gcode_G28() {

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_EM(">>> gcode_G28");
      log_machine_info();
    }
  #endif

  // Wait for planner moves to finish!
  stepper.synchronize();

  #if HAS(ABL)
    // For auto bed leveling, disable abl
    set_bed_leveling_enabled(false);
  #endif

  // Always home with tool 0 active
  #if HOTENDS > 1
    uint8_t old_tool_index = active_extruder;
    tool_change(0, 0, true);
  #endif

  #if ENABLED(DUAL_X_CARRIAGE)
    hotend_duplication_enabled = false;
  #endif

  /**
   * For mesh bed leveling deactivate the mesh calculations, will be turned
   * on again when homing all axis
   */
  #if ENABLED(MESH_BED_LEVELING)
    float pre_home_z = MESH_HOME_SEARCH_Z;
    if (mbl.active()) {
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) SERIAL_EM("MBL was active");
      #endif
      // Save known Z position if already homed
      if (axis_homed[X_AXIS] && axis_homed[Y_AXIS] && axis_homed[Z_AXIS]) {
        pre_home_z = current_position[Z_AXIS];
        pre_home_z += mbl.get_z(RAW_CURRENT_POSITION(X_AXIS), RAW_CURRENT_POSITION(Y_AXIS));
      }
      mbl.set_active(false);
      current_position[Z_AXIS] = pre_home_z;
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) DEBUG_POS("Set Z to pre_home_z", current_position);
      #endif
    }
  #endif

  setup_for_endstop_or_probe_move();
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) SERIAL_EM("> endstops.enable(true)");
  #endif
  endstops.enable(true); // Enable endstops for next homing move

  bool come_back = code_seen('B');
  float lastpos[NUM_AXIS];
  float old_feedrate_mm_s;
  if (come_back) {
    old_feedrate_mm_s = feedrate_mm_s;
    memcpy(lastpos, current_position, sizeof(lastpos));
  }

  bool  homeX = code_seen('X'),
        homeY = code_seen('Y'),
        homeZ = code_seen('Z'),
        homeE = code_seen('E');

  #if ENABLED(FORCE_HOME_XY_BEFORE_Z)
    if (homeZ) homeX = homeY = true;
  #endif

  home_all_axis = (!homeX && !homeY && !homeZ && !homeE) || (homeX && homeY && homeZ);

  #if ENABLED(NPR2)
    if ((home_all_axis) || (code_seen('E'))) {
      active_driver = active_extruder = 1;
      planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], -200, COLOR_HOMERATE, active_extruder, active_driver);
      stepper.synchronize();
      old_color = 99;
      active_driver = active_extruder = 0;
    }
  #endif

  #if MECH(DELTA)

    /**
     * A delta can only safely home all axis at the same time
     */

    home_delta();

  #else // NOT DELTA

    set_destination_to_current();

    #if Z_HOME_DIR > 0  // If homing away from BED do Z first

      if (home_all_axis || homeZ) {
        HOMEAXIS(Z);
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("> HOMEAXIS(Z)", current_position);
        #endif
      }

    #else

      if (home_all_axis || homeX || homeY) {
        // Raise Z before homing any other axes and z is not already high enough (never lower z)
        destination[Z_AXIS] = LOGICAL_Z_POSITION(MIN_Z_HEIGHT_FOR_HOMING);
        if (destination[Z_AXIS] > current_position[Z_AXIS]) {
          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING))
              SERIAL_EMV("Raise Z (before homing) to ", destination[Z_AXIS]);
          #endif
          do_blocking_move_to_z(destination[Z_AXIS]);
        }
      }

    #endif

    #if ENABLED(QUICK_HOME)

      if (home_all_axis || (homeX && homeY)) quick_home_xy();

    #endif

    #if ENABLED(HOME_Y_BEFORE_X)
      // Home Y
      if (home_all_axis || homeY) {
        HOMEAXIS(Y);
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("> homeY", current_position);
        #endif
      }
    #endif

    // Home X
    if (home_all_axis || homeX) {
      #if ENABLED(DUAL_X_CARRIAGE)
        int tmp_extruder = active_extruder;
        active_extruder = !active_extruder;
        HOMEAXIS(X);
        inactive_hotend_x_pos = RAW_X_POSITION(current_position[X_AXIS]);
        active_extruder = tmp_extruder;
        HOMEAXIS(X);
        // reset state used by the different modes
        memcpy(raised_parked_position, current_position, sizeof(raised_parked_position));
        delayed_move_time = 0;
        active_hotend_parked = true;
      #else
        HOMEAXIS(X);
      #endif
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) DEBUG_POS("> homeX", current_position);
      #endif
    }

    #if DISABLED(HOME_Y_BEFORE_X)
      // Home Y
      if (home_all_axis || homeY) {
        HOMEAXIS(Y);
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("> homeY", current_position);
        #endif
      }
    #endif

    // Home Z last if homing towards the bed
    #if Z_HOME_DIR < 0
      if (home_all_axis || homeZ) {
        #if ENABLED(Z_SAFE_HOMING)
          home_z_safely();
        #else
          HOMEAXIS(Z);
        #endif // !Z_SAFE_HOMING
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("> (home_all_axis || homeZ) > final", current_position);
        #endif
      }
    #endif // Z_HOME_DIR < 0

    SYNC_PLAN_POSITION_KINEMATIC();

  #endif // !DELTA (gcode_G28)

  endstops.not_homing();

  // Enable mesh leveling again
  #if ENABLED(MESH_BED_LEVELING)
    if (mbl.has_mesh()) {
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) SERIAL_EM("MBL has mesh");
      #endif
      if (home_all_axis || (axis_homed[X_AXIS] && axis_homed[Y_AXIS] && homeZ)) {
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) SERIAL_EM("MBL Z homing");
        #endif
        current_position[Z_AXIS] = MESH_HOME_SEARCH_Z
          #if Z_HOME_DIR > 0
            + Z_MAX_POS
          #endif
        ;
        SYNC_PLAN_POSITION_KINEMATIC();
        mbl.set_active(true);
        #if ENABLED(MESH_G28_REST_ORIGIN)
          current_position[Z_AXIS] = 0.0;
          set_destination_to_current();
          line_to_destination(homing_feedrate_mm_s[Z_AXIS]);
          stepper.synchronize();
          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) DEBUG_POS("MBL Rest Origin", current_position);
          #endif
        #else
          planner.unapply_leveling(current_position);
          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) DEBUG_POS("MBL adjusted MESH_HOME_SEARCH_Z", current_position);
          #endif
        #endif
      }
      else if ((axis_homed[X_AXIS] && axis_homed[Y_AXIS] && axis_homed[Z_AXIS]) && (homeX || homeY)) {
        current_position[Z_AXIS] = pre_home_z;
        SYNC_PLAN_POSITION_KINEMATIC();
        mbl.set_active(true);
        planner.unapply_leveling(current_position);
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("MBL Home X or Y", current_position);
        #endif
      }
    }
  #endif

  if (come_back) {
    #if MECH(DELTA)
      feedrate_mm_s = homing_feedrate_mm_s[X_AXIS];
      memcpy(destination, lastpos, sizeof(destination));
      prepare_move_to_destination();
      feedrate_mm_s = old_feedrate_mm_s;
    #else
      if (homeX) {
        feedrate_mm_s = homing_feedrate_mm_s[X_AXIS];
        destination[X_AXIS] = lastpos[X_AXIS];
        prepare_move_to_destination();
      }
      if (homeY) {
        feedrate_mm_s = homing_feedrate_mm_s[Y_AXIS];
        destination[Y_AXIS] = lastpos[Y_AXIS];
        prepare_move_to_destination();
      }
      if (homeZ) {
        feedrate_mm_s = homing_feedrate_mm_s[Z_AXIS];
        destination[Z_AXIS] = lastpos[Z_AXIS];
        prepare_move_to_destination();
      }
      feedrate_mm_s = old_feedrate_mm_s;
    #endif
  }

  #if ENABLED(NEXTION) && ENABLED(NEXTION_GFX)
    #if MECH(DELTA)
      gfx_clear((X_MAX_POS) * 2, (Y_MAX_POS) * 2, Z_MAX_POS);
      gfx_cursor_to(current_position[X_AXIS] + (X_MAX_POS), current_position[Y_AXIS] + (Y_MAX_POS), current_position[Z_AXIS]);
    #else
      gfx_clear(X_MAX_POS, Y_MAX_POS, Z_MAX_POS);
      gfx_cursor_to(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS]);
    #endif
  #endif

  clean_up_after_endstop_or_probe_move();

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) SERIAL_EM("<<< gcode_G28");
  #endif

  // Restore the active tool after homing
  #if HOTENDS > 1
    tool_change(old_tool_index, 0, true);
  #endif

  report_current_position();
}

#if HAS(PROBING_PROCEDURE)
  void out_of_range_error(const char* p_edge) {
    SERIAL_M("?Probe ");
    SERIAL_PS(p_edge);
    SERIAL_EM(" position out of range.");
  }
#endif

#if ENABLED(MESH_BED_LEVELING)

  inline void _mbl_goto_xy(float x, float y) {
    float old_feedrate_mm_s = feedrate_mm_s;
    feedrate_mm_s = homing_feedrate_mm_s[X_AXIS];

    current_position[Z_AXIS] = MESH_HOME_SEARCH_Z
      #if Z_PROBE_BETWEEN_HEIGHT > MIN_Z_HEIGHT_FOR_HOMING
        + Z_PROBE_BETWEEN_HEIGHT
      #elif MIN_Z_HEIGHT_FOR_HOMING > 0
        + MIN_Z_HEIGHT_FOR_HOMING
      #endif
    ;
    line_to_current_position();

    current_position[X_AXIS] = LOGICAL_X_POSITION(x);
    current_position[Y_AXIS] = LOGICAL_Y_POSITION(y);
    line_to_current_position();

    #if Z_PROBE_BETWEEN_HEIGHT > 0 || MIN_Z_HEIGHT_FOR_HOMING > 0
      current_position[Z_AXIS] = LOGICAL_Z_POSITION(MESH_HOME_SEARCH_Z);
      line_to_current_position();
    #endif

    feedrate_mm_s = old_feedrate_mm_s;
    stepper.synchronize();
  }

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

    static int probe_point = -1;
    MeshLevelingState state = code_seen('S') ? (MeshLevelingState)code_value_byte() : MeshReport;
    if (state < 0 || state > 5) {
      SERIAL_M("S out of range (0-5).");
      return;
    }

    int8_t px, py;

    switch (state) {
      case MeshReport:
        if (mbl.has_mesh()) {
          SERIAL_EMV("State: ", mbl.active() ? "On" : "Off");
          SERIAL_MV("Num X,Y: ", MESH_NUM_X_POINTS);
          SERIAL_EMV(",", MESH_NUM_Y_POINTS);
          SERIAL_EMV("Z search height: ", MESH_HOME_SEARCH_Z);
          SERIAL_EMV("Z offset: ", mbl.z_offset, 5);
          SERIAL_EM("Measured points:");
          for (py = 0; py < MESH_NUM_Y_POINTS; py++) {
            for (px = 0; px < MESH_NUM_X_POINTS; px++) {
              SERIAL_MV("  ", mbl.z_values[py][px], 5);
            }
            SERIAL_E;
          }
        }
        else
          SERIAL_EM("Mesh bed leveling not active.");
        break;

      case MeshStart:
        mbl.reset();
        probe_point = 0;
        enqueue_and_echo_commands_P(PSTR("G28\nG29 S2"));
        break;

      case MeshNext:
        if (probe_point < 0) {
          SERIAL_EM("Start mesh probing with \"G29 S1\" first.");
          return;
        }
        // For each G29 S2...
        if (probe_point == 0) {
          // For the intial G29 S2 make Z a positive value (e.g., 4.0)
          current_position[Z_AXIS] = MESH_HOME_SEARCH_Z
            #if Z_HOME_DIR > 0
              + Z_MAX_POS
            #endif
          ;
          SYNC_PLAN_POSITION_KINEMATIC();
        }
        else {
          // For G29 S2 after adjusting Z.
          mbl.set_zigzag_z(probe_point - 1, current_position[Z_AXIS]);
        }
        // If there's another point to sample, move there with optional lift.
        if (probe_point < (MESH_NUM_X_POINTS) * (MESH_NUM_Y_POINTS)) {
          mbl.zigzag(probe_point, px, py);
          _mbl_goto_xy(mbl.get_probe_x(px), mbl.get_probe_y(py));
          probe_point++;
        }
        else {
          // One last "return to the bed" (as originally coded) at completion
          current_position[Z_AXIS] = MESH_HOME_SEARCH_Z
            #if Z_PROBE_BETWEEN_HEIGHT > MIN_Z_HEIGHT_FOR_HOMING
              + Z_PROBE_BETWEEN_HEIGHT
            #elif MIN_Z_HEIGHT_FOR_HOMING > 0
              + MIN_Z_HEIGHT_FOR_HOMING
            #endif
          ;
          line_to_current_position();
          stepper.synchronize();

          // After recording the last point, activate the mbl and home
          SERIAL_EM("Mesh probing done.");
          probe_point = -1;
          mbl.set_has_mesh(true);
          enqueue_and_echo_commands_P(PSTR("G28"));
        }
        break;

      case MeshSet:
        if (code_seen('X')) {
          px = code_value_int() - 1;
          if (px < 0 || px >= MESH_NUM_X_POINTS) {
            SERIAL_EM("X out of range (1-" STRINGIFY(MESH_NUM_X_POINTS) ").");
            return;
          }
        }
        else {
          SERIAL_C('X'); SERIAL_EM(" not entered.");
          return;
        }
        if (code_seen('Y')) {
          py = code_value_int() - 1;
          if (py < 0 || py >= MESH_NUM_Y_POINTS) {
            SERIAL_EM("Y out of range (1-" STRINGIFY(MESH_NUM_Y_POINTS) ").");
            return;
          }
        }
        else {
          SERIAL_C('Y'); SERIAL_EM(" not entered.");
          return;
        }
        if (code_seen('Z')) {
          mbl.z_values[py][px] = code_value_axis_units(Z_AXIS);
        }
        else {
          SERIAL_C('Z'); SERIAL_EM(" not entered.");
          return;
        }
        break;

      case MeshSetZOffset:
        if (code_seen('Z')) {
          mbl.z_offset = code_value_axis_units(Z_AXIS);
        }
        else {
          SERIAL_C('Z'); SERIAL_EM(" not entered.");
          return;
        }
        break;

      case MeshReset:
        if (mbl.active()) {
          current_position[Z_AXIS] +=
            mbl.get_z(RAW_CURRENT_POSITION(X_AXIS), RAW_CURRENT_POSITION(Y_AXIS)) - MESH_HOME_SEARCH_Z;
          mbl.reset();
          SYNC_PLAN_POSITION_KINEMATIC();
        }
        else
          mbl.reset();

    } // switch(state)

    report_current_position();
  }

#elif HAS(ABL)

  /**
   * G29: Detailed Z-Probe, probes the bed at 3 or more points.
   *      Will fail if the printer has not been homed with G28.
   *
   * Enhanced G29 Auto Bed Levelling Probe Routine
   * 
   * Parameters With AUTO_BED_LEVELING_GRID:
   *
   *  P  Set the size of the grid that will be probed (P x P points).
   *     Example: "G29 P4"
   *
   *  S  Set the XY travel speed between probe points (in units/min)
   *
   *  D  Dry-Run mode. Just evaluate the bed Topology - Don't apply
   *     or clean the rotation Matrix. Useful to check the topology
   *     after a first run of G29.
   *
   *  V  Set the verbose level (0-4). Example: "G29 V3"
   *
   *  T  Generate a Bed Topology Report. Example: "G29 P5 T" for a detailed report.
   *     This is useful for manual bed levelling and finding flaws in the bed (to
   *     assist with part placement).
   *
   *  F  Set the Front limit of the probing grid
   *  B  Set the Back limit of the probing grid
   *  L  Set the Left limit of the probing grid
   *  R  Set the Right limit of the probing grid
   *
   * Global Parameters:
   *
   * E/e By default G29 will engage the probe, test the bed, then disengage.
   *     Include "E" to engage/disengage the probe for each sample.
   *     There's no extra effect if you have a fixed probe.
   *     Usage: "G29 E" or "G29 e"
   *
   */
  inline void gcode_G29() {

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      bool query = code_seen('Q');
      uint8_t old_debug_flags = mk_debug_flags;
      if (query) mk_debug_flags |= DEBUG_LEVELING;
      if (DEBUGGING(LEVELING)) {
        DEBUG_POS(">>> gcode_G29", current_position);
        log_machine_info();
      }
      mk_debug_flags = old_debug_flags;
      if (query) return;
    #endif

    #if MECH(DELTA)
      // Homing and deploy z probe
      if (!axis_homed[X_AXIS] || !axis_homed[Y_AXIS] || !axis_homed[Z_AXIS])
        home_delta();
    #else
      // Don't allow auto-levelling without homing first
      if (axis_unhomed_error(true, true, true)) return;
    #endif

    int verbose_level = code_seen('V') ? code_value_int() : 1;
    if (verbose_level < 0 || verbose_level > 4) {
      SERIAL_LM(ER, "?(V)erbose Level is implausible (0-4).");
      return;
    }

    bool dryrun = code_seen('D'),
         stow_probe_after_each = code_seen('E');

    #if ABL_GRID

      if (verbose_level > 0) {
        SERIAL_EM("G29 Auto Bed Leveling");
        if (dryrun) SERIAL_EM("Running in DRY-RUN mode");
      }

      #if ABL_PLANAR

        bool do_topography_map = verbose_level > 2 || code_seen('T');

        // X and Y specify points in each direction, overriding the default
        // These values may be saved with the completed mesh
        int abl_grid_points_x = code_seen('X') ? code_value_int() : ABL_GRID_POINTS_X,
            abl_grid_points_y = code_seen('Y') ? code_value_int() : ABL_GRID_POINTS_Y;

        if (code_seen('P')) abl_grid_points_x = abl_grid_points_y = code_value_int();

        if (abl_grid_points_x < 2 || abl_grid_points_y < 2) {
          SERIAL_LM(ER, "?Number of probed (P)oints is implausible (2 minimum).\n");
          return;
        }

      #else

        const int abl_grid_points_x = ABL_GRID_POINTS_X, abl_grid_points_y = ABL_GRID_POINTS_Y;

      #endif

      xy_probe_feedrate_mm_s = MMM_TO_MMS(code_seen('S') ? code_value_linear_units() : XY_PROBE_SPEED);

      int left_probe_bed_position = code_seen('L') ? (int)code_value_axis_units(X_AXIS) : LOGICAL_X_POSITION(LEFT_PROBE_BED_POSITION),
          right_probe_bed_position = code_seen('R') ? (int)code_value_axis_units(X_AXIS) : LOGICAL_X_POSITION(RIGHT_PROBE_BED_POSITION),
          front_probe_bed_position = code_seen('F') ? (int)code_value_axis_units(Y_AXIS) : LOGICAL_Y_POSITION(FRONT_PROBE_BED_POSITION),
          back_probe_bed_position = code_seen('B') ? (int)code_value_axis_units(Y_AXIS) : LOGICAL_Y_POSITION(BACK_PROBE_BED_POSITION);

      bool left_out_l = left_probe_bed_position < LOGICAL_X_POSITION(MIN_PROBE_X),
           left_out = left_out_l || left_probe_bed_position > right_probe_bed_position - (MIN_PROBE_EDGE),
           right_out_r = right_probe_bed_position > LOGICAL_X_POSITION(MAX_PROBE_X),
           right_out = right_out_r || right_probe_bed_position < left_probe_bed_position + MIN_PROBE_EDGE,
           front_out_f = front_probe_bed_position < LOGICAL_Y_POSITION(MIN_PROBE_Y),
           front_out = front_out_f || front_probe_bed_position > back_probe_bed_position - (MIN_PROBE_EDGE),
           back_out_b = back_probe_bed_position > LOGICAL_Y_POSITION(MAX_PROBE_Y),
           back_out = back_out_b || back_probe_bed_position < front_probe_bed_position + MIN_PROBE_EDGE;

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

    #endif // ABL_GRID

    stepper.synchronize();

    // Disable auto bed leveling during G29
    bool abl_should_enable = planner.abl_enabled;

    planner.abl_enabled = false;

    if (!dryrun) {
      // Re-orient the current position without leveling
      // based on where the steppers are positioned.
      set_current_from_steppers_for_axis(ALL_AXES);

      // Sync the planner to where the steppers stopped
      SYNC_PLAN_POSITION_KINEMATIC();
    }

    setup_for_endstop_or_probe_move();

    // Deploy the probe. Probe will raise if needed.
    if (DEPLOY_PROBE()) {
      planner.abl_enabled = abl_should_enable;
      return;
    }

    float xProbe = 0, yProbe = 0, measured_z = 0;

    #if ABL_GRID

      // probe at the points of a lattice grid
      const float xGridSpacing = (right_probe_bed_position - left_probe_bed_position) / (abl_grid_points_x - 1),
                  yGridSpacing = (back_probe_bed_position - front_probe_bed_position) / (abl_grid_points_y - 1);

      #if ENABLED(AUTO_BED_LEVELING_BILINEAR)

        float zoffset = zprobe_zoffset;
        if (code_seen('Z')) zoffset += code_value_axis_units(Z_AXIS);

        reset_bed_level();
        bilinear_grid_spacing[X_AXIS] = xGridSpacing;
        bilinear_grid_spacing[Y_AXIS] = yGridSpacing;
        bilinear_start[X_AXIS] = RAW_X_POSITION(left_probe_bed_position);
        bilinear_start[Y_AXIS] = RAW_Y_POSITION(front_probe_bed_position);
        // Can't re-enable (on error) until the new grid is written
        abl_should_enable = false;

      #elif ENABLED(AUTO_BED_LEVELING_LINEAR)

        /**
         * solve the plane equation ax + by + d = z
         * A is the matrix with rows [x y 1] for all the probed points
         * B is the vector of the Z positions
         * the normal vector to the plane is formed by the coefficients of the
         * plane equation in the standard form, which is Vx*x+Vy*y+Vz*z+d = 0
         * so Vx = -a Vy = -b Vz = 1 (we want the vector facing towards positive Z
         */

        int abl2 = abl_grid_points_x * abl_grid_points_y,
            indexIntoAB[abl_grid_points_x][abl_grid_points_y],
            probePointCounter = -1;

        float eqnAMatrix[abl2 * 3], // "A" matrix of the linear system of equations
              eqnBVector[abl2],     // "B" vector of Z points
              mean = 0.0;

      #endif // AUTO_BED_LEVELING_LINEAR

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

      bool zig = PR_OUTER_END & 1; // always end at [RIGHT_PROBE_BED_POSITION, BACK_PROBE_BED_POSITION]

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

        zig = !zig;

        for (int8_t PR_INNER_VAR = inStart; PR_INNER_VAR != inStop; PR_INNER_VAR += inInc) {

          float xBase = left_probe_bed_position + xGridSpacing * xCount,
                yBase = front_probe_bed_position + yGridSpacing * yCount;

          xProbe = floor(xBase + (xBase < 0 ? 0 : 0.5));
          yProbe = floor(yBase + (yBase < 0 ? 0 : 0.5));

          #if ENABLED(AUTO_BED_LEVELING_LINEAR)
            indexIntoAB[xCount][yCount] = ++probePointCounter;
          #endif

          #if IS_KINEMATIC
            // Avoid probing outside the round or hexagonal area
            float pos[XYZ] = { xProbe, yProbe, 0 };
            if (!position_is_reachable(pos, true)) continue;
          #endif

          measured_z = probe_pt(xProbe, yProbe, stow_probe_after_each, verbose_level);

          if (measured_z == NAN) {
            planner.abl_enabled = abl_should_enable;
            return;
          }

          #if ENABLED(AUTO_BED_LEVELING_LINEAR)

            mean += measured_z;
            eqnBVector[probePointCounter] = measured_z;
            eqnAMatrix[probePointCounter + 0 * abl2] = xProbe;
            eqnAMatrix[probePointCounter + 1 * abl2] = yProbe;
            eqnAMatrix[probePointCounter + 2 * abl2] = 1;

          #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)

            bilinear_level_grid[xCount][yCount] = measured_z + zoffset;

          #endif

          idle();

        } // xProbe
      } // yProbe

    #elif ENABLED(AUTO_BED_LEVELING_3POINT)

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) SERIAL_EM("> 3-point Leveling");
      #endif

      // Probe at 3 arbitrary points
      vector_3 points[3] = {
        vector_3(ABL_PROBE_PT_1_X, ABL_PROBE_PT_1_Y, 0),
        vector_3(ABL_PROBE_PT_2_X, ABL_PROBE_PT_2_Y, 0),
        vector_3(ABL_PROBE_PT_3_X, ABL_PROBE_PT_3_Y, 0)
      };

      for (uint8_t i = 0; i < 3; ++i) {
        // Retain the last probe position
        xProbe = LOGICAL_X_POSITION(points[i].x);
        yProbe = LOGICAL_Y_POSITION(points[i].y);
        measured_z = points[i].z = probe_pt(xProbe, yProbe, stow_probe_after_each, verbose_level);
      }

      if (measured_z == NAN) {
        planner.abl_enabled = abl_should_enable;
        return;
      }

      if (!dryrun) {
        vector_3 planeNormal = vector_3::cross(points[0] - points[1], points[2] - points[1]).get_normal();
        if (planeNormal.z < 0) {
          planeNormal.x *= -1;
          planeNormal.y *= -1;
          planeNormal.z *= -1;
        }
        planner.bed_level_matrix = matrix_3x3::create_look_at(planeNormal);

        // Can't re-enable (on error) until the new grid is written
        abl_should_enable = false;
      }

    #endif // !AUTO_BED_LEVELING_3POINT

    // Raise to _Z_PROBE_DEPLOY_HEIGHT. Stow the probe.
    if (STOW_PROBE()) {
      planner.abl_enabled = abl_should_enable;
      return;
    }

    // Restore state after probing
    clean_up_after_endstop_or_probe_move();

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("> probing complete", current_position);
    #endif

    // Calculate leveling, print reports, correct the position
    #if ENABLED(AUTO_BED_LEVELING_BILINEAR)

      if (!dryrun) extrapolate_unprobed_bed_level();
      print_bed_level();

    #elif ENABLED(AUTO_BED_LEVELING_LINEAR)

      // For LINEAR leveling calculate matrix, print reports, correct the position

      // solve lsq problem
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
        planner.bed_level_matrix = matrix_3x3::create_look_at(
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

            apply_rotation_xyz(planner.bed_level_matrix, x_tmp, y_tmp, z_tmp);

            NOMORE(min_diff, eqnBVector[ind] - z_tmp);

            if (diff >= 0.0)
              SERIAL_M(" +");   // Include + for column alignment
            else
              SERIAL_C(' ');
            SERIAL_V(diff, 5);
          } // xx
          SERIAL_E;
        } // yy
        SERIAL_E;

        if (verbose_level > 3) {
          SERIAL_EM("\nCorrected Bed Height vs. Bed Topology:");

          for (int8_t yy = abl_grid_points_y - 1; yy >= 0; yy--) {
            for (uint8_t xx = 0; xx < abl_grid_points_x; xx++) {
              int ind = indexIntoAB[xx][yy];
              float x_tmp = eqnAMatrix[ind + 0 * abl2],
                    y_tmp = eqnAMatrix[ind + 1 * abl2],
                    z_tmp = 0;

              apply_rotation_xyz(planner.bed_level_matrix, x_tmp, y_tmp, z_tmp);

              float diff = eqnBVector[ind] - z_tmp - min_diff;
              if (diff >= 0.0)
                SERIAL_M(" +");   // Include + for column alignment
              else
                SERIAL_C(' ');
              SERIAL_V(diff, 5);
            } // xx
            SERIAL_E;
          } // yy
          SERIAL_E;
        }
      } // do_topography_map

    #endif // AUTO_BED_LEVELING_LINEAR_GRID

    #if ABL_PLANAR

      // For LINEAR and 3POINT leveling correct the current position

      if (verbose_level > 0)
        planner.bed_level_matrix.debug("\n\nBed Level Correction Matrix:");

      if (!dryrun) {
        //
        // Correct the current XYZ position based on the tilted plane.
        //

        // 1. Get the distance from the current position to the reference point.
        float x_dist = RAW_CURRENT_POSITION(X_AXIS) - X_TILT_FULCRUM,
              y_dist = RAW_CURRENT_POSITION(Y_AXIS) - Y_TILT_FULCRUM,
              z_real = RAW_CURRENT_POSITION(Z_AXIS),
              z_zero = 0;

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("G29 uncorrected XYZ", current_position);
        #endif

        matrix_3x3 inverse = matrix_3x3::transpose(planner.bed_level_matrix);

        // 2. Apply the inverse matrix to the distance
        //    from the reference point to X, Y, and zero.
        apply_rotation_xyz(inverse, x_dist, y_dist, z_zero);

        // 3. Get the matrix-based corrected Z.
        //    (Even if not used, get it for comparison.)
        float new_z = z_real + z_zero;

        // 4. Use the last measured distance to the bed, if possible
        if ( NEAR(current_position[X_AXIS], xProbe - (X_PROBE_OFFSET_FROM_NOZZLE))
          && NEAR(current_position[Y_AXIS], yProbe - (Y_PROBE_OFFSET_FROM_NOZZLE))
        ) {
          float simple_z = z_real - (measured_z - (-zprobe_zoffset));
          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) {
              SERIAL_MV("Z from Probe:", simple_z);
              SERIAL_MV("  Matrix:", new_z);
              SERIAL_EMV("  Discrepancy:", simple_z - new_z);
            }
          #endif
          new_z = simple_z;
        }

        // 5. The rotated XY and corrected Z are now current_position
        current_position[X_AXIS] = LOGICAL_X_POSITION(x_dist) + X_TILT_FULCRUM;
        current_position[Y_AXIS] = LOGICAL_Y_POSITION(y_dist) + Y_TILT_FULCRUM;
        current_position[Z_AXIS] = new_z;

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("G29 corrected XYZ", current_position);
        #endif
      }

    #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)

      if (!dryrun) {
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) SERIAL_EMV("G29 uncorrected Z:", current_position[Z_AXIS]);
        #endif

        current_position[Z_AXIS] -= bilinear_z_offset(current_position);

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) SERIAL_EMV(" corrected Z:", current_position[Z_AXIS]);
        #endif
      }

    #endif // ABL_PLANAR

    #if ENABLED(Z_PROBE_END_SCRIPT)
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) {
          SERIAL_M("Z Probe End Script: ");
          SERIAL_EM(Z_PROBE_END_SCRIPT);
        }
      #endif
      enqueue_and_echo_commands_P(PSTR(Z_PROBE_END_SCRIPT));
      stepper.synchronize();
    #endif

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) SERIAL_EM("<<< gcode_G29");
    #endif

    report_current_position();

    KEEPALIVE_STATE(IN_HANDLER);

    // Auto Bed Leveling is complete! Enable if possible.
    planner.abl_enabled = dryrun ? abl_should_enable : true;

    if (planner.abl_enabled) SYNC_PLAN_POSITION_KINEMATIC();
  }

#endif // HAS_ABL

#if HAS(BED_PROBE) && NOMECH(DELTA)

  /**
   * G30: Do a single Z probe at the current XY
   * Usage:
   *   G30 <X#> <Y#> <S#>
   *     X = Probe X position (default=current probe position)
   *     Y = Probe Y position (default=current probe position)
   *     S = Stows the probe if 1 (default=1)
   */
  inline void gcode_G30() {

    float X_probe_location = code_seen('X') ? code_value_axis_units(X_AXIS) : current_position[X_AXIS] + X_PROBE_OFFSET_FROM_NOZZLE;
    float Y_probe_location = code_seen('Y') ? code_value_axis_units(Y_AXIS) : current_position[Y_AXIS] + Y_PROBE_OFFSET_FROM_NOZZLE;
    bool stow = code_seen('S') ? code_value_bool() : true;

    // Disable leveling so the planner won't mess with us
    #if PLANNER_LEVELING
      set_bed_leveling_enabled(false);
    #endif

    setup_for_endstop_or_probe_move();

    // TODO: clear the leveling matrix or the planner will be set incorrectly
    float measured_z = probe_pt(X_probe_location, Y_probe_location, stow, 1);

    SERIAL_MV(" Bed X: ", current_position[X_AXIS] + X_PROBE_OFFSET_FROM_NOZZLE + 0.0001);
    SERIAL_MV(" Y: ", current_position[Y_AXIS] + Y_PROBE_OFFSET_FROM_NOZZLE + 0.0001);
    SERIAL_MV(" Z: ", measured_z + 0.0001);
    SERIAL_E;

    clean_up_after_endstop_or_probe_move();

    report_current_position();
  }

  #if ENABLED(Z_PROBE_SLED)

    /**
     * G31: Deploy the Z probe
     */
    inline void gcode_G31() { DEPLOY_PROBE(); }

    /**
     * G32: Stow the Z probe
     */
    inline void gcode_G32() { STOW_PROBE(); }

  #endif // Z_PROBE_SLED

#elif ENABLED(AUTO_CALIBRATION_FEATURE)

  /**
   * G30: Delta AutoCalibration
   * Usage:
   *    G30 <X#> <Y#> <A#> <E> <R> <I> <D> <T> <S#> <U#>
   *      X = Probe X position (default=current probe position)
   *      Y = Probe Y position (default=current probe position)
   *      A = Autocalibration +/- precision
   *      E = Adjust Endstop
   *      R = Adjust Endstop & Delta Radius
   *      I = Adjust Tower
   *      D = Adjust Diagonal Rod
   *      T = Adjust Tower Radius
   *      S = Stows the probe if 1 (default=1)
   *      U = <bool> with a non-zero value will apply the result to current zprobe_zoffset
   */
  inline void gcode_G30() {

    #if HAS(ABL)
      set_bed_leveling_enabled(false);
    #endif

    // Homing and deploy z probe
    if (!axis_homed[X_AXIS] || !axis_homed[Y_AXIS] || !axis_homed[Z_AXIS])
      home_delta();

    setup_for_endstop_or_probe_move();

    bed_safe_z = current_position[Z_AXIS];

    if (code_seen('X') || code_seen('Y')) {
      // Probe specified X, Y point
      float X_probe_location = code_seen('X') ? code_value_axis_units(X_AXIS) : current_position[X_AXIS] + X_PROBE_OFFSET_FROM_NOZZLE,
            Y_probe_location = code_seen('Y') ? code_value_axis_units(Y_AXIS) : current_position[Y_AXIS] + Y_PROBE_OFFSET_FROM_NOZZLE;
      bool stow = code_seen('S') ? code_value_bool() : true;
      
      float measured_z = probe_pt(X_probe_location, Y_probe_location, stow, 1),
            new_zprobe_zoffset = soft_endstop_min[Z_AXIS] - measured_z;

      SERIAL_MV(" Bed X:", current_position[X_AXIS] + X_PROBE_OFFSET_FROM_NOZZLE + 0.0001);
      SERIAL_MV(" Y: ", current_position[Y_AXIS] + Y_PROBE_OFFSET_FROM_NOZZLE + 0.0001);
      SERIAL_MV(" Z: ", measured_z + zprobe_zoffset, 4);
      SERIAL_EMV("  New Z probe offset = ", new_zprobe_zoffset, 4);

      if (code_seen('U') && code_value_bool() != 0)
        zprobe_zoffset = new_zprobe_zoffset;

      clean_up_after_endstop_or_probe_move();

      report_current_position();

      return;
    }

    if (code_seen('A')) {
      SERIAL_EM("Starting Auto Calibration...");
      LCD_MESSAGEPGM("Auto Calibration...");
      if (code_has_value()) ac_prec = code_value_float();
      SERIAL_MV("Calibration precision: +/-", ac_prec, 2);
      SERIAL_EM(" mm");
    }

    // Probe all points
    bed_probe_all();

    // Show calibration report      
    calibration_report();

    if (code_seen('E')) {
      int iteration = 0;
      do {
        iteration ++;
        SERIAL_EMV("Iteration: ", iteration);

        SERIAL_EM("Checking/Adjusting Endstop offsets");
        adj_endstops();

        bed_probe_all();
        calibration_report();
      } while ((bed_level_x < -ac_prec) or (bed_level_x > ac_prec)
            or (bed_level_y < -ac_prec) or (bed_level_y > ac_prec)
            or (bed_level_z < -ac_prec) or (bed_level_z > ac_prec));

      SERIAL_EM("Endstop adjustment complete");
    }

    if (code_seen('R')) {
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

      } while ((bed_level_c < -ac_prec) or (bed_level_c > ac_prec)
            or (bed_level_x < -ac_prec) or (bed_level_x > ac_prec)
            or (bed_level_y < -ac_prec) or (bed_level_y > ac_prec)
            or (bed_level_z < -ac_prec) or (bed_level_z > ac_prec));
    }

    if (code_seen('I')) {
      SERIAL_EMV("Adjusting Tower Delta for tower", code_value_byte());
      adj_tower_delta(code_value_byte());
      SERIAL_EM("Tower Delta adjustment complete");
    }

    if (code_seen('D')) {
      SERIAL_EM("Adjusting Diagonal Rod Length");
      adj_diagrod_length();
      SERIAL_EM("Diagonal Rod Length adjustment complete");
    }

    if (code_seen('T')) {
      SERIAL_EMV("Adjusting Tower Radius for tower", code_value_byte());
      adj_tower_radius(code_value_byte());
      SERIAL_EM("Tower Radius adjustment complete");
    }

    if (code_seen('A')) {
      int iteration = 0;
      boolean dr_adjusted;

      do {
        do {
          iteration ++;
          SERIAL_EMV("Iteration: ", iteration);

          SERIAL_EM("Checking/Adjusting endstop offsets");
          adj_endstops();

          bed_probe_all();
          calibration_report();

          if ((bed_level_c < -ac_prec) or (bed_level_c > ac_prec)) {
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

          //idle();
        } while ((bed_level_c < -ac_prec) or (bed_level_c > ac_prec)
              or (bed_level_x < -ac_prec) or (bed_level_x > ac_prec)
              or (bed_level_y < -ac_prec) or (bed_level_y > ac_prec)
              or (bed_level_z < -ac_prec) or (bed_level_z > ac_prec)
              or (dr_adjusted));

        if ((bed_level_ox < -ac_prec) or (bed_level_ox > ac_prec) or
            (bed_level_oy < -ac_prec) or (bed_level_oy > ac_prec) or
            (bed_level_oz < -ac_prec) or (bed_level_oz > ac_prec)) {
          SERIAL_EM("Checking for tower geometry errors..");
          if (fix_tower_errors() != 0 ) {
            // Tower positions have been changed .. home to endstops
            SERIAL_EM("Tower Positions changed .. Homing");
            home_delta();
            do_probe_raise(_Z_PROBE_DEPLOY_HEIGHT);
          }
          else {
            SERIAL_EM("Checking Diagonal Rod Length");
            if (adj_diagrod_length() != 0) { 
              // If diagonal rod length has been changed .. home to endstops
              SERIAL_EM("Diagonal Rod Length changed .. Homing");
              home_delta();
              do_probe_raise(_Z_PROBE_DEPLOY_HEIGHT);
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
      } while((bed_level_c < -ac_prec) or (bed_level_c > ac_prec)
           or (bed_level_x < -ac_prec) or (bed_level_x > ac_prec)
           or (bed_level_y < -ac_prec) or (bed_level_y > ac_prec)
           or (bed_level_z < -ac_prec) or (bed_level_z > ac_prec)
           or (bed_level_ox < -ac_prec) or (bed_level_ox > ac_prec)
           or (bed_level_oy < -ac_prec) or (bed_level_oy > ac_prec)
           or (bed_level_oz < -ac_prec) or (bed_level_oz > ac_prec));

      SERIAL_EM("Autocalibration Complete");
    }

    STOW_PROBE();

    // reset LCD alert message
    lcd_reset_alert_level();

    clean_up_after_endstop_or_probe_move();

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) SERIAL_EM("<<< gcode_G30");
    #endif

    report_current_position();
    KEEPALIVE_STATE(IN_HANDLER);
  }

#endif // AUTO_BED_LEVELING_FEATURE && DELTA

/**
 * G60:  save current position
 *        S<slot> specifies memory slot # (0-based) to save into (default 0)
 */
inline void gcode_G60() {
  uint8_t slot = 0;
  if (code_seen('S')) slot = code_value_byte();

  if (slot >= NUM_POSITON_SLOTS) {
    SERIAL_LMV(ER, MSG_INVALID_POS_SLOT, (int)NUM_POSITON_SLOTS);
    return;
  } 
  memcpy(stored_position[slot], current_position, sizeof(*stored_position));
  pos_saved = true;

  SERIAL_M(MSG_SAVED_POS);
  SERIAL_MV(" S", slot);
  SERIAL_MV("<-X:", stored_position[slot][X_AXIS]);
  SERIAL_MV(" Y:", stored_position[slot][Y_AXIS]);
  SERIAL_MV(" Z:", stored_position[slot][Z_AXIS]);
  SERIAL_EMV(" E:", stored_position[slot][E_AXIS]);
}

/**
 * G61:  Apply/restore saved coordinates to the active extruder.
 *        X Y Z E - Value to add at stored coordinates.
 *        F<speed> - Set Feedrate.
 *        S<slot> specifies memory slot # (0-based) to save into (default 0).
 */
inline void gcode_G61() {
  if (!pos_saved) return;

  uint8_t slot = 0;
  if (code_seen('S')) slot = code_value_byte();

  if (slot >= NUM_POSITON_SLOTS) {
    SERIAL_LMV(ER, MSG_INVALID_POS_SLOT, (int)NUM_POSITON_SLOTS);
    return;
  }

  SERIAL_M(MSG_RESTORING_POS);
  SERIAL_MV(" S", slot);
  SERIAL_M("->");

  if (code_seen('F')) {
    float next_feedrate = code_value_linear_units();
    if (next_feedrate > 0.0) feedrate_mm_s = next_feedrate;
  }

  LOOP_XYZE(i) {
    if (code_seen(axis_codes[i])) {
      destination[i] = code_value_axis_units(i) + stored_position[slot][i];
    }
    else {
      destination[i] = current_position[i];
    }
    SERIAL_MV(" ", axis_codes[i]);
    SERIAL_MV(":", destination[i]);
  }
  SERIAL_E;

  // finish moves
  prepare_move_to_destination();
  stepper.synchronize();
}

/**
 * G92: Set current position to given X Y Z E
 */
inline void gcode_G92() {
  bool didXYZ = false,
       didE = code_seen('E');

  if (!didE) stepper.synchronize();

  LOOP_XYZE(i) {
    if (code_seen(axis_codes[i])) {
      #if IS_SCARA
        current_position[i] = code_value_axis_units(i);
        if (i != E_AXIS) didXYZ = true;
      #else
        float p = current_position[i],
              v = code_value_axis_units(i);

        current_position[i] = v;

        if (i != E_AXIS) {
          position_shift[i] += v - p; // Offset the coordinate space
          update_software_endstops((AxisEnum)i);
          didXYZ = true;
        }
      #endif
    }
  }
  if (didXYZ)
    SYNC_PLAN_POSITION_KINEMATIC();
  else if (didE)
    sync_plan_position_e();

  report_current_position();
}

#if ENABLED(ULTIPANEL)

  /**
   * M0: Unconditional stop - Wait for user button press on LCD
   * M1: Conditional stop   - Wait for user button press on LCD
   */
  inline void gcode_M0_M1() {
    char* args = current_command_args;

    millis_t codenum = 0;
    bool hasP = false, hasS = false;
    if (code_seen('P')) {
      codenum = code_value_millis(); // milliseconds to wait
      hasP = codenum > 0;
    }
    if (code_seen('S')) {
      codenum = code_value_millis_from_seconds(); // seconds to wait
      hasS = codenum > 0;
    }

    if (!hasP && !hasS && *args != '\0')
      lcd_setstatus(args, true);
    else {
      LCD_MESSAGEPGM(MSG_USERWAIT);
      #if ENABLED(LCD_PROGRESS_BAR) && PROGRESS_MSG_EXPIRE > 0
        dontExpireStatus();
      #endif
    }

    wait_for_user = true;
    KEEPALIVE_STATE(PAUSED_FOR_USER);

    stepper.synchronize();
    refresh_cmd_timeout();

    if (codenum > 0) {
      codenum += previous_cmd_ms;  // wait until this time for a click
      while (PENDING(millis(), codenum) && wait_for_user) idle();
    }
    else {
      if (lcd_detected()) {
        while (wait_for_user) idle();
        IS_SD_PRINTING ? LCD_MESSAGEPGM(MSG_RESUMING) : LCD_MESSAGEPGM(WELCOME_MSG);
      }
    }

    wait_for_user = false;
    KEEPALIVE_STATE(IN_HANDLER);
  }
#endif // ULTIPANEL

#if (ENABLED(LASERBEAM) && ENABLED(LASER_FIRE_SPINDLE))
  /**
   * M3: S - Setting laser beam or fire laser 
   */
  inline void gcode_M3_M4() {
    if (code_seen('S') && IsRunning()) laser.intensity = code_value_float();
    if (code_seen('L') && IsRunning()) laser.duration = code_value_ulong();
    if (code_seen('P') && IsRunning()) laser.ppm = code_value_float();
    if (code_seen('D') && IsRunning()) laser.diagnostics = code_value_bool();
    if (code_seen('B') && IsRunning()) laser_set_mode(code_value_int());

    laser.status = LASER_ON;
    laser.fired = LASER_FIRE_SPINDLE;

    lcd_update();

    prepare_move_to_destination();
  }

  /**
   * M5: Turn off laser beam
   */
  inline void gcode_M5() {
    if (laser.status != LASER_OFF) {
      laser.status = LASER_OFF;
      laser.mode = CONTINUOUS;
      laser.duration = 0;

      lcd_update();

      prepare_move_to_destination();

      if (laser.diagnostics)
        SERIAL_EM("Laser M5 called and laser OFF");
    }
  }
#endif // LASERBEAM

/**
 * M17: Enable power on all stepper motors
 */
inline void gcode_M17() {
  LCD_MESSAGEPGM(MSG_NO_MOVE);
  stepper.enable_all_steppers();
}

#if ENABLED(SDSUPPORT)

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
    card.selectFile(current_command_args);
  }

  /**
   * M24: Start SD Print
   */
  inline void gcode_M24() {
    card.startFileprint();
    print_job_counter.start();
    #if HAS(POWER_CONSUMPTION_SENSOR)
      startpower = power_consumption_hour;
    #endif
  }

  /**
   * M25: Pause SD Print
   */
  inline void gcode_M25() {
    card.pauseSDPrint();
  }

  /**
   * M26: Set SD Card file index
   */
  inline void gcode_M26() {
    if (card.cardOK && code_seen('S'))
      card.setIndex(code_value_long());
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
    card.startWrite(current_command_args, false);
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
      card.deleteFile(current_command_args);
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

  #if ENABLED(AUTOTEMP)
    thermalManager.autotempShutdown();
  #endif
}

#if ENABLED(SDSUPPORT)

  /**
   * M32: Make Directory
   */
  inline void gcode_M32() {
    if (card.cardOK) {
      card.makeDirectory(current_command_args);
      card.mount();
    }
  }

  /**
   * M33: Close File and save restart.gcode
   */
  inline void gcode_M33() {
    if (card.cardOK && card.isFileOpen() && IS_SD_PRINTING) {
      SERIAL_EM("Close file and save restart.gcode");
      card.stopSDPrint(true);
      clear_command_queue();
      quickstop_stepper();
      print_job_counter.stop();
      #if ENABLED(AUTOTEMP)
        thermalManager.autotempShutdown();
      #endif
      wait_for_heatup = false;
      lcd_setstatus(MSG_PRINT_ABORTED, true);
    }
  }

  /**
   * M34: Select file and start SD print
   */
  inline void gcode_M34() {
    if (card.sdprinting)
      stepper.synchronize();

    if (card.cardOK) {
      char* namestartpos = (strchr(current_command_args, '@'));
      if (namestartpos == NULL) {
        namestartpos = current_command_args ; // default name position
      }
      else
        namestartpos++; // to skip the '@'

      SERIAL_MV("Open file: ", namestartpos);
      SERIAL_EM(" and start print.");
      card.selectFile(namestartpos);
      if (code_seen('S')) card.setIndex(code_value_long());

      feedrate_mm_s       = 20.0; // 20 units/sec
      feedrate_percentage = 100;  // 100% feedrate_mm_s
      card.startFileprint();
      print_job_counter.start();
      #if HAS(POWER_CONSUMPTION_SENSOR)
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
  if (!code_seen('S')) return;

  int pin_status = code_value_int();
  if (pin_status < 0 || pin_status > 255) return;

  int pin_number = code_seen('P') ? code_value_int() : LED_PIN;
  if (pin_number < 0) return;

  if (pin_is_protected(pin_number)) {
    SERIAL_LM(ER, MSG_ERR_PROTECTED_PIN);
    return;
  }

  pinMode(pin_number, OUTPUT);
  digitalWrite(pin_number, pin_status);
  analogWrite(pin_number, pin_status);

  #if HAS(FAN)
    if (pin_number == FAN_PIN) fanSpeed = pin_status;
  #endif
}

#if ENABLED(PINS_DEBUGGING)

  #include "utility/pinsdebug.h"

  /**
   * M43: Pin report and debug
   *
   *      E<bool> Enable / disable background endstop monitoring
   *               - Machine continues to operate
   *               - Reports changes to endstops
   *               - Toggles LED when an endstop changes
   *
   *   or
   *
   *      P<pin>  Pin to read or watch. If omitted, read/watch all pins.
   *      W<bool> Watch pins -reporting changes- until reset, click, or M108.
   *      I<bool> Flag to ignore MK4duo pin protection.
   *
   */
  inline void gcode_M43() {

    // Enable or disable endstop monitoring
    if (code_seen('E')) {
      endstop_monitor_flag = code_value_bool();
      SERIAL_M("endstop monitor ");
      SERIAL_T(endstop_monitor_flag ? "en" : "dis");
      SERIAL_EM("abled");
      return;
    }

    // Get the range of pins to test or watch
    int first_pin = 0, last_pin = NUM_DIGITAL_PINS - 1;
    if (code_seen('P')) {
      first_pin = last_pin = code_value_byte();
      if (first_pin > NUM_DIGITAL_PINS - 1) return;
    }

    bool ignore_protection = code_seen('I') ? code_value_bool() : false;

    // Watch until click, M108, or reset
    if (code_seen('W') && code_value_bool()) { // watch digital pins
      byte pin_state[last_pin - first_pin + 1];
      for (int8_t pin = first_pin; pin <= last_pin; pin++) {
        if (pin_is_protected(pin) && !ignore_protection) continue;
        pinMode(pin, INPUT_PULLUP);
        // if (IS_ANALOG(pin))
        //   pin_state[pin - first_pin] = analogRead(pin - analogInputToDigitalPin(0)); // int16_t pin_state[...]
        // else
          pin_state[pin - first_pin] = digitalRead(pin);
      }

      #if ENABLED(ULTIPANEL)
        wait_for_user = true;
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

        #if ENABLED(ULTIPANEL)
          if (!wait_for_user) break;
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

    if (axis_unhomed_error(true, true, true)) return;

    int8_t verbose_level = code_seen('V') ? code_value_byte() : 1;
    if (verbose_level < 0 || verbose_level > 4 ) {
      SERIAL_LM(ER, "?Verbose Level not plausible (0-4).");
      return;
    }

    if (verbose_level > 0)
      SERIAL_EM("M48 Z-Probe Repeatability Test");

    int8_t n_samples = code_seen('P') ? code_value_byte() : 10;
    if (n_samples < 4 || n_samples > 50) {
      SERIAL_LM(ER, "?Sample size not plausible (4-50).");
      return;
    }

    float  X_current = current_position[X_AXIS],
           Y_current = current_position[Y_AXIS];

    bool stow_probe_after_each = code_seen('E');

    float X_probe_location = code_seen('X') ? code_value_axis_units(X_AXIS) : X_current + X_PROBE_OFFSET_FROM_NOZZLE;
    #if NOMECH(DELTA)
      if (X_probe_location < LOGICAL_X_POSITION(MIN_PROBE_X) || X_probe_location > LOGICAL_X_POSITION(MAX_PROBE_X)) {
        out_of_range_error(PSTR("X"));
        return;
      }
    #endif

    float Y_probe_location = code_seen('Y') ? code_value_axis_units(Y_AXIS) : Y_current + Y_PROBE_OFFSET_FROM_NOZZLE;
    #if NOMECH(DELTA)
      if (Y_probe_location < LOGICAL_Y_POSITION(MIN_PROBE_Y) || Y_probe_location > LOGICAL_Y_POSITION(MAX_PROBE_Y)) {
        out_of_range_error(PSTR("Y"));
        return;
      }
    #else
      float pos[XYZ] = { X_probe_location, Y_probe_location, 0 };
      if (!position_is_reachable(pos, true)) {
        SERIAL_LM(ER, "? (X,Y) location outside of probeable radius.");
        return;
      }
    #endif

    bool seen_L = code_seen('L');
    uint8_t n_legs = seen_L ? code_value_byte() : 0;
    if (n_legs > 15) {
      SERIAL_LM(ER, "?Number of legs in movement not plausible (0-15).");
      return;
    }
    if (n_legs == 1) n_legs = 2;

    bool schizoid_flag = code_seen('S');
    if (schizoid_flag && !seen_L) n_legs = 7;

    /**
     * Now get everything to the specified probe point So we can safely do a
     * probe to get us close to the bed.  If the Z-Axis is far from the bed,
     * we don't want to use that as a starting point for each probe.
     */
    if (verbose_level > 2)
      SERIAL_EM("Positioning the probe...");

    // we don't do bed level correction in M48 because we want the raw data when we probe
    #if HAS(ABL)
      reset_bed_level();
    #endif

    setup_for_endstop_or_probe_move();

    // Move to the first point, deploy, and probe
    probe_pt(X_probe_location, Y_probe_location, stow_probe_after_each, verbose_level);

    randomSeed(millis());

    double mean = 0.0, sigma = 0.0, min = 99999.9, max = -99999.9, sample_set[n_samples];

    for (uint8_t n = 0; n < n_samples; n++) {
      if (n_legs) {
        int dir = (random(0, 10) > 5.0) ? -1 : 1;  // clockwise or counter clockwise
        float angle = random(0.0, 360.0),
              radius = random(
                #if MECH(DELTA)
                  DELTA_PROBEABLE_RADIUS / 8, DELTA_PROBEABLE_RADIUS / 3
                #else
                  5, X_MAX_LENGTH / 8
                #endif
              );

        if (verbose_level > 3) {
          SERIAL_MV("Starting radius: ", radius);
          SERIAL_MV("   angle: ", angle);
          SERIAL_M(" Direction: ");
          if (dir > 0) SERIAL_M("Counter-");
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
            while (HYPOT(X_current, Y_current) > DELTA_PROBEABLE_RADIUS) {
              X_current /= 1.25;
              Y_current /= 1.25;
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
            SERIAL_M("Going to:");
            SERIAL_MV(" x: ", X_current);
            SERIAL_MV(" y: ", Y_current);
            SERIAL_EMV("  z: ", current_position[Z_AXIS]);
          }
          do_blocking_move_to_xy(X_current, Y_current);
        } // n_legs loop
      } // n_legs

      // Probe a single point
      sample_set[n] = probe_pt(X_probe_location, Y_probe_location, stow_probe_after_each, 0);

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

      sigma = sqrt(sum / (n + 1));
      if (verbose_level > 0) {
        if (verbose_level > 1) {
          SERIAL_V(n + 1);
          SERIAL_MV(" of ", (int)n_samples);
          SERIAL_MV(": z: ", sample_set[n], 3);
          if (verbose_level > 2) {
            SERIAL_MV(" mean: ", mean, 4);
            SERIAL_MV(" sigma: ", sigma, 6);
            SERIAL_MV(" min: ", min, 3);
            SERIAL_MV(" max: ", max, 3);
            SERIAL_MV(" range: ", max - min, 3);
          }
        }
        SERIAL_E;
      }

    }  // End of probe loop

    if (STOW_PROBE()) return;

    SERIAL_EM("Finished!");

    if (verbose_level > 0) {
      SERIAL_MV("Mean: ", mean, 6);
      SERIAL_MV(" Min: ", min, 3);
      SERIAL_MV(" Max: ", max, 3);
      SERIAL_MV(" Range: ", max - min, 3);
      SERIAL_E;
    }

    SERIAL_EMV("Standard Deviation: ", sigma, 6);
    SERIAL_E;

    clean_up_after_endstop_or_probe_move();

    report_current_position();
  }

#endif // Z_MIN_PROBE_REPEATABILITY_TEST

#if HAS(POWER_CONSUMPTION_SENSOR)
  /**
   * M70 - Power consumption sensor calibration
   *
   * Z - Calibrate zero current offset
   * A - Isert readed DC Current value (Ampere)
   * W - Insert readed AC Wattage value (Watt)
   */
  inline void gcode_M70() {
    if(code_seen('Z')) {
      SERIAL_EMV("Actual POWER_ZERO:", POWER_ZERO, 7);
      SERIAL_EMV("New POWER_ZERO:", raw_analog2voltage(), 7);
      SERIAL_EM("Insert new calculated values into the FW and call \"M70 A\" for the next calibration step.");
    }
    else if(code_seen('A')) {
      SERIAL_EMV("Actual POWER_ERROR:", POWER_ERROR, 7);
      SERIAL_EMV("New POWER_ERROR:", analog2error(code_value_float()), 7);
      SERIAL_EM("Insert new calculated values into the FW and call \"M70 W\" for the last calibration step.");
    }
    else if(code_seen('W')) {
      SERIAL_EMV("Actual POWER_EFFICIENCY:", POWER_EFFICIENCY, 7);
      SERIAL_EMV("New POWER_EFFICIENCY:", analog2efficiency(code_value_float()), 7);
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
  if (code_seen('S') && code_value_int() == 78)
    print_job_counter.initStats();
  else print_job_counter.showStats();
}

#if HAS(POWER_SWITCH)
  /**
   * M80: Turn on Power Supply
   */
  inline void gcode_M80() {
    OUT_WRITE(PS_ON_PIN, PS_ON_AWAKE); // GND

    // If you have a switch on suicide pin, this is useful
    // if you want to start another print with suicide feature after
    // a print without suicide...
    #if HAS(SUICIDE)
      OUT_WRITE(SUICIDE_PIN, HIGH);
    #endif

    powersupply = true;

    #if HAS(LCD)
      LCD_MESSAGEPGM(WELCOME_MSG);
      lcd_update();
    #endif

    #if ENABLED(LASERBEAM) && ENABLED(LASER_PERIPHERALS)
      laser_peripherals_on();
      laser_wait_for_peripherals();
    #endif
  }
#endif // HAS(POWER_SWITCH)

/**
 * M81: Turn off Power, including Power Supply, if there is one.
 *
 *      This code should ALWAYS be available for EMERGENCY SHUTDOWN!
 */
inline void gcode_M81() {
  thermalManager.disable_all_heaters();
  thermalManager.disable_all_coolers();
  stepper.synchronize();
  disable_e();
  stepper.finish_and_disable();
  fanSpeed = 0;

  #if ENABLED(LASERBEAM)
    laser_extinguish();
    #if ENABLED(LASER_PERIPHERALS)
      laser_peripherals_off();
    #endif
  #endif

  safe_delay(1000); // Wait 1 second before switching off

  #if HAS(SUICIDE)
    stepper.synchronize();
    suicide();
  #elif HAS(POWER_SWITCH)
    OUT_WRITE(PS_ON_PIN, PS_ON_ASLEEP);
    powersupply = false;
  #endif

  #if ENABLED(ULTIPANEL)
    LCD_MESSAGEPGM(MACHINE_NAME " " MSG_OFF ".");
    lcd_update();
  #endif
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
 * M18, M84: Disable all stepper motors
 */
inline void gcode_M18_M84() {
  if (code_seen('S')) {
    stepper_inactive_time = code_value_millis_from_seconds();
  }
  else {
    bool all_axis = !((code_seen('X')) || (code_seen('Y')) || (code_seen('Z')) || (code_seen('E')));
    if (all_axis) {
      stepper.finish_and_disable();
    }
    else {
      stepper.synchronize();
      if (code_seen('X')) disable_x();
      if (code_seen('Y')) disable_y();
      if (code_seen('Z')) disable_z();
      #if ((E0_ENABLE_PIN != X_ENABLE_PIN) && (E1_ENABLE_PIN != Y_ENABLE_PIN)) // Only enable on boards that have seperate ENABLE_PINS
        if (code_seen('E')) {
          disable_e();
        }
      #endif
    }
  }
}

/**
 * M85: Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
 */
inline void gcode_M85() {
  if (code_seen('S')) max_inactive_time = code_value_millis_from_seconds();
}

/**
 * M92: Set planner.axis_steps_per_mm
 */
inline void gcode_M92() {
  if (get_target_extruder_from_command(92)) return;

  LOOP_XYZE(i) {
    if (code_seen(axis_codes[i])) {
      if (i == E_AXIS)
        planner.axis_steps_per_mm[i + target_extruder] = code_value_per_axis_unit(i + target_extruder);
      else
        planner.axis_steps_per_mm[i] = code_value_per_axis_unit(i);
    }
  }
  planner.refresh_positioning();
}

#if ENABLED(ZWOBBLE)
  /**
   * M96: Print ZWobble value
   */
  inline void gcode_M96() {
    zwobble.ReportToSerial();
  }

  /**
   * M97: Set ZWobble value
   */
  inline void gcode_M97() {
    float zVal = -1.0, hVal = -1.0, lVal = -1.0;

    if (code_seen('A')) zwobble.setAmplitude(code_value_float());
    if (code_seen('W')) zwobble.setPeriod(code_value_float());
    if (code_seen('P')) zwobble.setPhase(code_value_float());
    if (code_seen('Z')) zVal = code_value_float();
    if (code_seen('H')) hVal = code_value_float();
    if (code_seen('L')) lVal = code_value_float();
    if (zVal >= 0 && hVal >= 0) zwobble.setSample(zVal, hVal);
    if (zVal >= 0 && lVal >= 0) zwobble.setScaledSample(zVal, lVal);
    if (lVal >  0 && hVal >  0) zwobble.setScalingFactor(hVal/lVal);
  }
#endif // ZWOBBLE

#if ENABLED(HYSTERESIS)
  /**
   * M98: Print Hysteresis value
   */
  inline void gcode_M98() {
    hysteresis.ReportToSerial();
  }

  /**
   * M99: Set Hysteresis value
   */
  inline void gcode_M99() {
    LOOP_XYZE(i) {
      if (code_seen(axis_codes[i]))
        hysteresis.SetAxis(i, code_value_float());
    }
  }
#endif // HYSTERESIS

/**
 * M100 Free Memory Watcher
 *
 * This code watches the free memory block between the bottom of the heap and the top of the stack.
 * This memory block is initialized and watched via the M100 command.
 *
 * M100 I Initializes the free memory block and prints vitals statistics about the area
 * M100 F Identifies how much of the free memory block remains free and unused. It also
 *        detects and reports any corruption within the free memory block that may have
 *        happened due to errant firmware.
 * M100 D Does a hex display of the free memory block along with a flag for any errant
 *        data that does not match the expected value.
 * M100 C x Corrupts x locations within the free memory block. This is useful to check the
 *        correctness of the M100 F and M100 D commands.
 *
 * Initial version by Roxy-3DPrintBoard
 *
 */
#if ENABLED(M100_FREE_MEMORY_WATCHER)
  inline void gcode_M100() {
    static int m100_not_initialized = 1;
    unsigned char* sp, *ptr;
    int i, j, n;

    // M100 D dumps the free memory block from __brkval to the stack pointer.
    // malloc() eats memory from the start of the block and the stack grows
    // up from the bottom of the block.    Solid 0xE5's indicate nothing has
    // used that memory yet.   There should not be anything but 0xE5's within
    // the block of 0xE5's.  If there is, that would indicate memory corruption
    // probably caused by bad pointers.  Any unexpected values will be flagged in
    // the right hand column to help spotting them.
    #if ENABLED(M100_FREE_MEMORY_DUMPER) // Disable to remove Dump sub-command
      if (code_seen('D')) {
        ptr = (unsigned char*) __brkval;

        // We want to start and end the dump on a nice 16 byte boundry even though
        // the values we are using are not 16 byte aligned.
        //
        SERIAL_M("\n__brkval : ");
        prt_hex_word((unsigned int) ptr);
        ptr = (unsigned char*)((unsigned long) ptr & 0xfff0);
        sp = top_of_stack();
        SERIAL_M("\nStack Pointer : ");
        prt_hex_word((unsigned int) sp);
        SERIAL_E;
        sp = (unsigned char*)((unsigned long) sp | 0x000f);
        n = sp - ptr;
        //
        // This is the main loop of the Dump command.
        //
        while (ptr < sp) {
          prt_hex_word((unsigned int) ptr);  // Print the address
          SERIAL_C(':');
          for (i = 0; i < 16; i++) {     // and 16 data bytes
            prt_hex_byte(*(ptr+i));
            SERIAL_C(' ');
            HAL::delayMilliseconds(2);
          }
          SERIAL_C('|');        // now show where non 0xE5's are
          for (i = 0; i < 16; i++) {
            HAL::delayMilliseconds(2);
            if (*(ptr + i) == 0xe5)
              SERIAL_C(' ');
            else
              SERIAL_C('?');
          }
          SERIAL_E;
          ptr += 16;
          HAL::delayMilliseconds(2);
        }
        SERIAL_EM("Done.");
        return;
      }
    #endif

    //
    // M100 F   requests the code to return the number of free bytes in the memory pool along with
    // other vital statistics that define the memory pool.
    //
    if (code_seen('F')) {
      #if 0
        int max_addr = (int) __brkval;
        int max_cnt = 0;
      #endif
      int block_cnt = 0;
      ptr = (unsigned char*) __brkval;
      sp = top_of_stack();
      n = sp - ptr;
      // Scan through the range looking for the biggest block of 0xE5's we can find
      for (i = 0; i < n; i++) {
        if (*(ptr + i) == (unsigned char) 0xe5) {
          j = how_many_E5s_are_here((unsigned char*) ptr + i);
          if ( j > 8) {
            SERIAL_MV("Found ", j);
            SERIAL_M(" bytes free at 0x");
            prt_hex_word((int) ptr + i);
            SERIAL_E;
            i += j;
            block_cnt++;
          }
          #if 0
            if (j > max_cnt) {  // We don't do anything with this information yet
              max_cnt  = j;     // but we do know where the biggest free memory block is.
              max_addr = (int) ptr + i;
            }
          #endif
        }
      }
      if (block_cnt > 1)
          SERIAL_EM("\nMemory Corruption detected in free memory area.\n");

      SERIAL_EM("\nDone.");
      return;
    }

    //
    // M100 C x  Corrupts x locations in the free memory pool and reports the locations of the corruption.
    // This is useful to check the correctness of the M100 D and the M100 F commands.
    //
    #if ENABLED(M100_FREE_MEMORY_CORRUPTOR)
      if (code_seen('C')) {
        int x = code_value_int(); // x gets the # of locations to corrupt within the memory pool
        SERIAL_EM("Corrupting free memory block.\n");
        ptr = (unsigned char*) __brkval;
        SERIAL_MV("\n__brkval : ", ptr);
        ptr += 8;
        sp = top_of_stack();
        SERIAL_MV("\nStack Pointer : ", sp);
        SERIAL_EM("\n");
        n = sp - ptr - 64;  // -64 just to keep us from finding interrupt activity that
        // has altered the stack.
        j = n / (x + 1);
        for (i = 1; i <= x; i++) {
          *(ptr + (i * j)) = i;
          SERIAL_M("\nCorrupting address: 0x");
          prt_hex_word((unsigned int)(ptr + (i * j)));
        }
        SERIAL_EM("\n");
        return;
      }
    #endif

    //
    // M100 I    Initializes the free memory pool so it can be watched and prints vital
    // statistics that define the free memory pool.
    //
    if (m100_not_initialized || code_seen('I')) {     // If no sub-command is specified, the first time
      SERIAL_EM("Initializing free memory block.\n");   // this happens, it will Initialize.
      ptr = (unsigned char*) __brkval;        // Repeated M100 with no sub-command will not destroy the
      SERIAL_MV("\n__brkval : ",(long) ptr);    // state of the initialized free memory pool.
      ptr += 8;
      sp = top_of_stack();
      SERIAL_MV("\nStack Pointer : ", sp);
      SERIAL_EM("\n");
      n = sp - ptr - 64;    // -64 just to keep us from finding interrupt activity that
      // has altered the stack.
      SERIAL_V(n);
      SERIAL_EM(" bytes of memory initialized.\n");

      for (i = 0; i < n; i++)
        *(ptr+i) = (unsigned char) 0xe5;

      for (i = 0; i < n; i++) {
        if ( *(ptr + i) != (unsigned char) 0xe5) {
          SERIAL_MV("? address : ", ptr + i);
          SERIAL_MV("=", *(ptr + i));
          SERIAL_EM("\n");
        }
      }
      m100_not_initialized = 0;
      SERIAL_EM("Done.\n");
      return;
    }
    return;
  }
#endif

/**
 * M104: Set hotend temperature
 */
inline void gcode_M104() {
  if (get_target_extruder_from_command(104)) return;
  if (DEBUGGING(DRYRUN)) return;

  #if ENABLED(SINGLENOZZLE)
    if (target_extruder != active_extruder) return;
  #endif

  if (code_seen('S')) {
    thermalManager.setTargetHotend(code_value_temp_abs(), target_extruder);
    #if ENABLED(DUAL_X_CARRIAGE)
      if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && target_extruder == 0)
        thermalManager.setTargetHotend(code_value_temp_abs() == 0.0 ? 0.0 : code_value_temp_abs() + duplicate_hotend_temp_offset, 1);
    #endif

    if (code_value_temp_abs() > thermalManager.degHotend(target_extruder)) LCD_MESSAGEPGM(MSG_HEATING);
  }
}

/**
 * M105: Read hot end and bed temperature
 */
inline void gcode_M105() {
  if (get_target_extruder_from_command(105)) return;

  #if HAS(TEMP_0) || HAS(TEMP_BED) || ENABLED(HEATER_0_USES_MAX6675) || HAS(TEMP_COOLER) || ENABLED(FLOWMETER_SENSOR)
    SERIAL_S(OK);
    #if HAS(TEMP_0) || HAS(TEMP_BED) || ENABLED(HEATER_0_USES_MAX6675)
      print_heaterstates();
    #endif
    #if HAS(TEMP_CHAMBER)
      print_chamberstate();
    #endif
    #if HAS(TEMP_COOLER)
      print_coolerstate();
    #endif
    #if ENABLED(FLOWMETER_SENSOR)
      print_flowratestate();
    #endif
  #else // HASNT(TEMP_0) && HASNT(TEMP_BED)
    SERIAL_LM(ER, MSG_ERR_NO_THERMISTORS);
  #endif

  SERIAL_E;
}

#if HAS(FAN)
  /**
   * M106: Set Fan Speed
   *
   *  S<int>   Speed between 0-255
   */
  inline void gcode_M106() {
    uint16_t s = code_seen('S') ? code_value_ushort() : 255;
    NOMORE(s, 255);
    fanSpeed = s;
  }

  /**
   * M107: Fan Off
   */
  inline void gcode_M107() { fanSpeed = 0; }

#endif // HAS(FAN)

/**
 * M108: Cancel heatup and wait for the hotend and bed, this G-code is asynchronously handled in the get_serial_commands() parser
 */
inline void gcode_M108() { wait_for_heatup = false; }

/**
 * M109: Sxxx Wait for hotend(s) to reach temperature. Waits only when heating.
 *       Rxxx Wait for hotend(s) to reach temperature. Waits when heating and cooling.
 */
inline void gcode_M109() {
  if (get_target_extruder_from_command(109)) return;
  if (DEBUGGING(DRYRUN)) return;

  #if ENABLED(SINGLENOZZLE)
    if (target_extruder != active_extruder) return;
  #endif

  bool no_wait_for_cooling = code_seen('S');
  if (no_wait_for_cooling || code_seen('R')) {
    thermalManager.setTargetHotend(code_value_temp_abs(), target_extruder);
    #if ENABLED(DUAL_X_CARRIAGE)
      if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && target_extruder == 0)
        thermalManager.setTargetHotend(code_value_temp_abs() == 0.0 ? 0.0 : code_value_temp_abs() + duplicate_hotend_temp_offset, 1);
    #endif

    if (thermalManager.isHeatingHotend(target_extruder)) LCD_MESSAGEPGM(MSG_HEATING);
  }

  #if ENABLED(AUTOTEMP)
    planner.autotemp_M109();
  #endif

  wait_heater(no_wait_for_cooling);
}

/**
 * M110: Set Current Line Number
 */
inline void gcode_M110() {
  if (code_seen('N')) gcode_N = code_value_long();
}

/**
 * M111: Debug mode Repetier Host compatibile
 */
inline void gcode_M111() {
  mk_debug_flags = code_seen('S') ? code_value_byte() : (uint8_t) DEBUG_NONE;

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
        if (comma++) SERIAL_C(',');
        SERIAL_PS((char*)pgm_read_word(&(debug_strings[i])));
      }
    }
  }
  else {
    SERIAL_M(MSG_DEBUG_OFF);
  }
  SERIAL_E;
}

/**
 * M112: Emergency Stop
 */
inline void gcode_M112() { kill(PSTR(MSG_KILLED)); }

#if ENABLED(HOST_KEEPALIVE_FEATURE)
  /**
   * M113: Get or set Host Keepalive interval (0 to disable)
   *
   *   S<seconds> Optional. Set the keepalive interval.
   */
  inline void gcode_M113() {
    if (code_seen('S')) {
      host_keepalive_interval = code_value_byte();
      NOMORE(host_keepalive_interval, 60);
    }
    else {
      SERIAL_EMV("M113 S", (unsigned long)host_keepalive_interval);
    }
  }
#endif

/**
 * M114: Output current position to serial port
 */
inline void gcode_M114() { report_current_position(); }

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

    // AUTOLEVEL (G29)
    #if HAS(ABL)
      SERIAL_LM(CAP, "AUTOLEVEL:1");
    #else
      SERIAL_LM(CAP, "AUTOLEVEL:0");
    #endif

    // Z_PROBE (G30)
    #if HAS(BED_PROBE)
      SERIAL_LM(CAP, "Z_PROBE:1");
    #else
      SERIAL_LM(CAP, "Z_PROBE:0");
    #endif

    // SOFTWARE_POWER (G30)
    #if HAS(POWER_SWITCH)
      SERIAL_LM(CAP, "SOFTWARE_POWER:1");
    #else
      SERIAL_LM(CAP, "SOFTWARE_POWER:0");
    #endif

    // TOGGLE_LIGHTS (M355)
    #if HAS(CASE_LIGHT)
      SERIAL_LM(CAP, "TOGGLE_LIGHTS:1");
    #else
      SERIAL_LM(CAP, "TOGGLE_LIGHTS:0");
    #endif

  #endif // EXTENDED_CAPABILITIES_REPORT
}

/**
 * M117: Set LCD Status Message
 */
inline void gcode_M117() {
  lcd_setstatus(current_command_args);
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
  #if ENABLED(SOFTWARE_MIN_ENDSTOPS) || ENABLED(SOFTWARE_MAX_ENDSTOPS)
    if (code_seen('S')) soft_endstops_enabled = code_value_bool();
    SERIAL_SM(ECHO, MSG_SOFT_ENDSTOPS);
    SERIAL_PS(soft_endstops_enabled ? PSTR(MSG_ON) : PSTR(MSG_OFF));
  #else
    SERIAL_M(MSG_SOFT_ENDSTOPS);
    SERIAL_M(MSG_OFF);
  #endif
  SERIAL_M(MSG_SOFT_MIN);
  SERIAL_MV(    MSG_X, soft_endstop_min[X_AXIS]);
  SERIAL_MV(" " MSG_Y, soft_endstop_min[Y_AXIS]);
  SERIAL_MV(" " MSG_Z, soft_endstop_min[Z_AXIS]);
  SERIAL_M(MSG_SOFT_MAX);
  SERIAL_MV(    MSG_X, soft_endstop_max[X_AXIS]);
  SERIAL_MV(" " MSG_Y, soft_endstop_max[Y_AXIS]);
  SERIAL_MV(" " MSG_Z, soft_endstop_max[Z_AXIS]);
  SERIAL_E;
}

#if ENABLED(BARICUDA)
  #if HAS(HEATER_1)
    /**
     * M126: Heater 1 valve open
     */
    inline void gcode_M126() { baricuda_valve_pressure = code_seen('S') ? code_value_byte() : 255; }
    /**
     * M127: Heater 1 valve close
     */
    inline void gcode_M127() { baricuda_valve_pressure = 0; }
  #endif

  #if HAS(HEATER_2)
    /**
     * M128: Heater 2 valve open
     */
    inline void gcode_M128() { baricuda_e_to_p_pressure = code_seen('S') ? code_value_byte() : 255; }
    /**
     * M129: Heater 2 valve close
     */
    inline void gcode_M129() { baricuda_e_to_p_pressure = 0; }
  #endif
#endif //BARICUDA

#if HAS(TEMP_BED)
  /**
   * M140: Set Bed temperature
   */
  inline void gcode_M140() {
    if (DEBUGGING(DRYRUN)) return;
    if (code_seen('S')) thermalManager.setTargetBed(code_value_temp_abs());
  }
#endif

#if HAS(TEMP_CHAMBER)
  /**
   * M141: Set Chamber temperature
   */
  inline void gcode_M141() {
    if (DEBUGGING(DRYRUN)) return;
    if (code_seen('S')) setTargetChamber(code_value_temp_abs());
  }
#endif

#if HAS(TEMP_COOLER)
  /**
   * M142: Set Cooler temperature
   */
  inline void gcode_M142() {
    if (DEBUGGING(DRYRUN)) return;
    if (code_seen('S')) setTargetCooler(code_value_temp_abs());
  }
#endif

#if ENABLED(ULTIPANEL) && TEMP_SENSOR_0 != 0

  /**
   * M145: Set the heatup state for a material in the LCD menu
   *   S<material> (0=PLA, 1=ABS, 2=GUM)
   *   H<hotend temp>
   *   B<bed temp>
   *   F<fan speed>
   */
  inline void gcode_M145() {
    uint8_t material = code_seen('S') ? (uint8_t)code_value_int() : 0;
    if (material >= COUNT(lcd_preheat_hotend_temp)) {
      SERIAL_LM(ER, MSG_ERR_MATERIAL_INDEX);
    }
    else {
      int v;
      if (code_seen('H')) {
        v = code_value_int();
        lcd_preheat_hotend_temp[material] = constrain(v, HEATER_0_MINTEMP, HEATER_0_MAXTEMP - 15);
      }
      if (code_seen('F')) {
        v = code_value_int();
        lcd_preheat_fan_speed[material] = constrain(v, 0, 255);
      }
      #if TEMP_SENSOR_BED != 0
        if (code_seen('B')) {
          v = code_value_int();
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
         if (code_seen('C')) set_input_temp_units(TEMPUNIT_C);
    else if (code_seen('K')) set_input_temp_units(TEMPUNIT_K);
    else if (code_seen('F')) set_input_temp_units(TEMPUNIT_F);
  }
#endif

#if ENABLED(BLINKM)
  /**
   * M150: Set Status LED Color - Use R-U-B for R-G-B
   */
  inline void gcode_M150() {
    SendColors(
      code_seen('R') ? code_value_byte() : 0,
      code_seen('U') ? code_value_byte() : 0,
      code_seen('B') ? code_value_byte() : 0
    );
  }

#endif // BLINKM

#if ENABLED(AUTO_REPORT_TEMPERATURES) && (HAS_TEMP_HOTEND || HAS_TEMP_BED)

  static uint8_t auto_report_temp_interval;
  static millis_t next_temp_report_ms;

  /**
   * M155: Set temperature auto-report interval. M155 S<seconds>
   */
  inline void gcode_M155() {
    if (code_seen('S')) {
      auto_report_temp_interval = code_value_byte();
      NOMORE(auto_report_temp_interval, 60);
      next_temp_report_ms = millis() + 1000UL * auto_report_temp_interval;
    }
  }

  inline void auto_report_temperatures() {
    if (auto_report_temp_interval && ELAPSED(millis(), next_temp_report_ms)) {
      next_temp_report_ms = millis() + 1000UL * auto_report_temp_interval;
      print_heaterstates();
      SERIAL_E;
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
    int mix_index = code_seen('S') ? code_value_int() : 0;
    if (mix_index < MIXING_STEPPERS) {
      float mix_value = code_seen('P') ? code_value_float() : 0.0;
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
      int tool_index = code_seen('S') ? code_value_int() : 0;
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

#if HAS(TEMP_BED)
  /**
   * M190: Sxxx Wait for bed current temp to reach target temp. Waits only when heating
   *       Rxxx Wait for bed current temp to reach target temp. Waits when heating and cooling
   */
  inline void gcode_M190() {
    if (DEBUGGING(DRYRUN)) return;

    LCD_MESSAGEPGM(MSG_BED_HEATING);
    bool no_wait_for_cooling = code_seen('S');
    if (no_wait_for_cooling || code_seen('R'))
      thermalManager.setTargetBed(code_value_temp_abs());

    wait_bed(no_wait_for_cooling);
  }
#endif // HAS(TEMP_BED)

#if HAS(TEMP_CHAMBER)
  /**
   * M191: Sxxx Wait for chamber current temp to reach target temp. Waits only when heating
   *       Rxxx Wait for chamber current temp to reach target temp. Waits when heating and cooling
   */
  inline void gcode_M191() {
    if (DEBUGGING(DRYRUN)) return;

    LCD_MESSAGEPGM(MSG_CHAMBER_HEATING);
    bool no_wait_for_cooling = code_seen('S');
    if (no_wait_for_cooling || code_seen('R')) setTargetChamber(code_value_temp_abs());

    wait_chamber(no_wait_for_cooling);
  }
#endif // HAS(TEMP_CHAMBER)

#if HAS(TEMP_COOLER)
  /**
   * M192: Sxxx Wait for cooler current temp to reach target temp. Waits only when heating
   *       Rxxx Wait for cooler current temp to reach target temp. Waits when heating and cooling
   */
  inline void gcode_M192() {
    if (DEBUGGING(DRYRUN)) return;

    LCD_MESSAGEPGM(MSG_COOLER_COOLING);
    bool no_wait_for_heating = code_seen('S');
    if (no_wait_for_heating || code_seen('R')) setTargetCooler(code_value_temp_abs());

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

  if (get_target_extruder_from_command(200)) return;

  if (code_seen('D')) {
    // setting any extruder filament size disables volumetric on the assumption that
    // slicers either generate in extruder values as cubic mm or as as filament feeds
    // for all extruders
    volumetric_enabled = (code_value_linear_units() != 0.0);
    if (volumetric_enabled) {
      filament_size[target_extruder] = code_value_linear_units();
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
 */
inline void gcode_M201() {
  if (get_target_extruder_from_command(201)) return;

  LOOP_XYZE(i) {
    if (code_seen(axis_codes[i])) {
      if (i == E_AXIS)
        planner.max_acceleration_mm_per_s2[i + target_extruder] = code_value_axis_units(i + target_extruder);
      else
        planner.max_acceleration_mm_per_s2[i] = code_value_axis_units(i);
    }
  }
  // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
  planner.reset_acceleration_rates();
}

#if 0 // Not used for Sprinter/grbl gen6
  inline void gcode_M202() {
    LOOP_XYZE(i) {
      if(code_seen(axis_codes[i])) axis_travel_steps_per_sqr_second[i] = code_value_axis_units(i) * planner.axis_steps_per_mm[i];
    }
  }
#endif

/**
 * M203: Set maximum feedrate_mm_s that your machine can sustain in units/sec
 *
 *    X,Y,Z   = AXIS
 *    T* E    = E_AXIS
 *
 */
inline void gcode_M203() {
  if (get_target_extruder_from_command(203)) return;

  LOOP_XYZE(i) {
    if (code_seen(axis_codes[i])) {
      if (i == E_AXIS)
        planner.max_feedrate_mm_s[i + target_extruder] = code_value_axis_units(i + target_extruder);
      else
        planner.max_feedrate_mm_s[i] = code_value_axis_units(i);
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
 *  Also sets minimum segment time in ms (B20000) to prevent buffer under-runs and M20 minimum feedrate_mm_s
 */
inline void gcode_M204() {
  if (get_target_extruder_from_command(204)) return;

  if (code_seen('S')) {  // Kept for legacy compatibility. Should NOT BE USED for new developments.
    planner.travel_acceleration = planner.acceleration = code_value_linear_units();
    SERIAL_EMV("Setting Print and Travel acceleration: ", planner.acceleration );
  }
  if (code_seen('P')) {
    planner.acceleration = code_value_linear_units();
    SERIAL_EMV("Setting Print acceleration: ", planner.acceleration );
  }
  if (code_seen('R')) {
    planner.retract_acceleration[target_extruder] = code_value_linear_units();
    SERIAL_EMV("Setting Retract acceleration: ", planner.retract_acceleration[target_extruder]);
  }
  if (code_seen('V')) {
    planner.travel_acceleration = code_value_linear_units();
    SERIAL_EMV("Setting Travel acceleration: ", planner.travel_acceleration );
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
  if (get_target_extruder_from_command(205)) return;

  if (code_seen('S')) planner.min_feedrate_mm_s = code_value_linear_units();
  if (code_seen('V')) planner.min_travel_feedrate_mm_s = code_value_linear_units();
  if (code_seen('B')) planner.min_segment_time = code_value_millis();
  if (code_seen('X')) planner.max_jerk[X_AXIS] = code_value_axis_units(X_AXIS);
  if (code_seen('Y')) planner.max_jerk[Y_AXIS] = code_value_axis_units(Y_AXIS);
  if (code_seen('Z')) planner.max_jerk[Z_AXIS] = code_value_axis_units(Z_AXIS);
  if (code_seen('E')) planner.max_jerk[E_AXIS + target_extruder] = code_value_axis_units(E_AXIS + target_extruder);
}

/**
 * M206: Set Additional Homing Offset (X Y Z). SCARA aliases T=X, P=Y
 */
inline void gcode_M206() {
  LOOP_XYZ(i) {
    if (code_seen(axis_codes[i])) {
      set_home_offset((AxisEnum)i, code_value_axis_units(i));
    }
  }
  #if MECH(MORGAN_SCARA)
    if (code_seen('T')) set_home_offset(X_AXIS, code_value_axis_units(X_AXIS)); // Theta
    if (code_seen('P')) set_home_offset(Y_AXIS, code_value_axis_units(Y_AXIS)); // Psi
  #endif

  sync_plan_position();
  report_current_position();
}

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
    if (code_seen('S')) retract_length = code_value_axis_units(E_AXIS);
    if (code_seen('F')) retract_feedrate_mm_s = MMM_TO_MMS(code_value_axis_units(E_AXIS));
    if (code_seen('Z')) retract_zlift = code_value_axis_units(Z_AXIS);
    #if EXTRUDERS > 1
      if (code_seen('W')) retract_length_swap = code_value_axis_units(E_AXIS);
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
    if (code_seen('S')) retract_recover_length = code_value_axis_units(E_AXIS);
    if (code_seen('F')) retract_recover_feedrate_mm_s = code_value_axis_units(E_AXIS) / 60;
    #if EXTRUDERS > 1
      if (code_seen('W')) retract_recover_length_swap = code_value_axis_units(E_AXIS);
    #endif
  }

  /**
   * M209: Enable automatic retract (M209 S1)
   *   For slicers that don't support G10/11, reversed extrude-only
   *   moves will be classified as retraction.
   */
  inline void gcode_M209() {
    if (code_seen('S')) {
      autoretract_enabled = code_value_bool();
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
  if (get_target_hotend_from_command(218) || target_extruder == 0) return;

  if (code_seen('X')) hotend_offset[X_AXIS][target_extruder] = code_value_axis_units(X_AXIS);
  if (code_seen('Y')) hotend_offset[Y_AXIS][target_extruder] = code_value_axis_units(Y_AXIS);
  if (code_seen('Z')) hotend_offset[Z_AXIS][target_extruder] = code_value_axis_units(Z_AXIS);

  SERIAL_SM(ECHO, MSG_HOTEND_OFFSET);
  HOTEND_LOOP() {
    SERIAL_MV(" ", hotend_offset[X_AXIS][h]);
    SERIAL_MV(",", hotend_offset[Y_AXIS][h]);
    SERIAL_MV(",", hotend_offset[Z_AXIS][h]);
  }
  SERIAL_E;
}

/**
 * M220: Set speed percentage factor, aka "Feed Rate" (M220 S95)
 */
inline void gcode_M220() {
  if (code_seen('S')) feedrate_percentage = code_value_int();
}

/**
 * M221: Set extrusion percentage (M221 T0 S95)
 */
inline void gcode_M221() {
  if (get_target_extruder_from_command(221)) return;

  if (code_seen('S')) flow_percentage[target_extruder] = code_value_int();
}

/**
 * M222: Set density extrusion percentage (M222 T0 S95)
 */
inline void gcode_M222() {
  if (get_target_extruder_from_command(222)) return;

  if (code_seen('S')) {
    density_percentage[target_extruder] = code_value_int();
    #if ENABLED(RFID_MODULE)
      RFID522.RfidData[target_extruder].data.density = density_percentage[target_extruder];
    #endif
  }
}

/**
 * M226: Wait until the specified pin reaches the state required (M226 P<pin> S<state>)
 */
inline void gcode_M226() {
  if (code_seen('P')) {
    int pin_number = code_value_int(),
        pin_state = code_seen('S') ? code_value_int() : -1; // required pin state - default is inverted

    if (pin_state >= -1 && pin_state <= 1 && pin_number > -1 && !pin_is_protected(pin_number)) {

      int target = LOW;

      stepper.synchronize();

      pinMode(pin_number, INPUT);
      switch(pin_state) {
        case 1:
          target = HIGH;
          break;
        case 0:
          target = LOW;
          break;
        case -1:
          target = !digitalRead(pin_number);
          break;
      }

      while(digitalRead(pin_number) != target) idle();

    } // pin_state -1 0 1 && pin_number > -1
  } // code_seen('P')
}

#if HAS(CHDK) || HAS(PHOTOGRAPH)
  /**
   * M240: Trigger a camera
   */
  inline void gcode_M240() {
    #if HAS(CHDK)
       OUT_WRITE(CHDK_PIN, HIGH);
       chdkHigh = millis();
       chdkActive = true;
    #elif HAS(PHOTOGRAPH)
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
    #endif // HASNT(CHDK) && HAS(PHOTOGRAPH)
  }
#endif // HAS(CHDK) || PHOTOGRAPH_PIN

#if HAS(LCD_CONTRAST)
  /**
   * M250: Read and optionally set the LCD contrast
   */
  inline void gcode_M250() {
    if (code_seen('C')) lcd_setcontrast(code_value_int() & 0x3F);
    SERIAL_EMV("lcd contrast value: ", lcd_contrast);
  }

#endif // DOGLCD

#if HAS(SERVOS)
  /**
   * M280: Get or set servo position. P<index> S<angle>
   */
  inline void gcode_M280() {
    if (!code_seen('P')) return;
    int servo_index = code_value_int();

    #if HAS(DONDOLO)
      int servo_position = 0;
      if (code_seen('S')) {
        servo_position = code_value_int();
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
      if (servo_index >= 0 && servo_index < NUM_SERVOS) {
        if (code_seen('S'))
          MOVE_SERVO(servo_index, code_value_int());
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

#if HAS(BUZZER)
  /**
   * M300: Play beep sound S<frequency Hz> P<duration ms>
   */
  inline void gcode_M300() {
    uint16_t const frequency = code_seen('S') ? code_value_ushort() : 260;
    uint16_t duration = code_seen('P') ? code_value_ushort() : 1000;

    // Limits the tone duration to 0-5 seconds.
    NOMORE(duration, 5000);

    buzz(duration, frequency);
  }
#endif // HAS(BUZZER)

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
    int h = code_seen('H') ? code_value_int() : 0; // hotend being updated

    if (h < HOTENDS) { // catch bad input value
      if (code_seen('P')) PID_PARAM(Kp, h) = code_value_float();
      if (code_seen('I')) PID_PARAM(Ki, h) = scalePID_i(code_value_float());
      if (code_seen('D')) PID_PARAM(Kd, h) = scalePID_d(code_value_float());
      #if ENABLED(PID_ADD_EXTRUSION_RATE)
        if (code_seen('C')) PID_PARAM(Kc, h) = code_value_float();
        if (code_seen('L')) lpq_len = code_value_float();
        NOMORE(lpq_len, LPQ_MAX_LEN);
      #endif

      thermalManager.updatePID();
      SERIAL_SMV(ECHO, "H", h);
      SERIAL_MV(" P:", PID_PARAM(Kp, h));
      SERIAL_MV(" I:", unscalePID_i(PID_PARAM(Ki, h)));
      SERIAL_MV(" D:", unscalePID_d(PID_PARAM(Kd, h)));
      #if ENABLED(PID_ADD_EXTRUSION_RATE)
        SERIAL_MV(" C:", PID_PARAM(Kc, h));
      #endif
      SERIAL_E;
    }
    else {
      SERIAL_LM(ER, MSG_INVALID_EXTRUDER);
    }
  }
#endif // PIDTEMP

#if ENABLED(PREVENT_COLD_EXTRUSION)
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
    bool seen_S = code_seen('S');
    if (seen_S) {
      thermalManager.extrude_min_temp = code_value_temp_abs();
      thermalManager.allow_cold_extrude = (thermalManager.extrude_min_temp == 0);
    }

    if (code_seen('P'))
      thermalManager.allow_cold_extrude = (thermalManager.extrude_min_temp == 0) || code_value_bool();
    else if (!seen_S) {
      // Report current state
      SERIAL_MV("Cold extrudes are ", (thermalManager.allow_cold_extrude ? "en" : "dis"));
      SERIAL_MV("abled (min temp ", int(thermalManager.extrude_min_temp + 0.5));
      SERIAL_EM("C)");
    }
  }
#endif // PREVENT_COLD_EXTRUSION

#if HAS(PID_HEATING) || HAS(PID_COOLING)
  /**
   * M303: PID relay autotune
   *       S<temperature> sets the target temperature. (default target temperature = 150C)
   *       H<hotend> (-1 for the bed, -2 for chamber, -3 for cooler) (default 0)
   *       C<cycles>
   *       U<bool> with a non-zero value will apply the result to current settings
   */
  inline void gcode_M303() {
    int h = code_seen('H') ? code_value_int() : 0;
    int c = code_seen('C') ? code_value_int() : 5;
    bool u = code_seen('U') && code_value_bool() != 0;

    float temp = code_seen('S') ? code_value_temp_abs() : (h < 0 ? 70.0 : 150.0);

    if (h >= 0 && h < HOTENDS) target_extruder = h;

    KEEPALIVE_STATE(NOT_BUSY); // don't send "busy: processing" messages during autotune output

    thermalManager.PID_autotune(temp, h, c, u);

    KEEPALIVE_STATE(IN_HANDLER);
  }
#endif

#if ENABLED(PIDTEMPBED)
  // M304: Set bed PID parameters P I and D
  inline void gcode_M304() {
    if (code_seen('P')) thermalManager.bedKp = code_value_float();
    if (code_seen('I')) thermalManager.bedKi = scalePID_i(code_value_float());
    if (code_seen('D')) thermalManager.bedKd = scalePID_d(code_value_float());

    thermalManager.updatePID();
    SERIAL_SMV(ECHO, " p:", thermalManager.bedKp);
    SERIAL_MV(" i:", unscalePID_i(thermalManager.bedKi));
    SERIAL_EMV(" d:", unscalePID_d(thermalManager.bedKd));
  }
#endif // PIDTEMPBED

#if ENABLED(PIDTEMPCHAMBER)
  // M305: Set chamber PID parameters P I and D
  inline void gcode_M305() {
    if (code_seen('P')) thermalManager.chamberKp = code_value_float();
    if (code_seen('I')) thermalManager.chamberKi = scalePID_i(code_value_float());
    if (code_seen('D')) thermalManager.chamberKd = scalePID_d(code_value_float());

    thermalManager.updatePID();
    SERIAL_SMV(OK, " p:", thermalManager.chamberKp);
    SERIAL_MV(" i:", unscalePID_i(thermalManager.chamberKi));
    SERIAL_EMV(" d:", unscalePID_d(thermalManager.chamberKd));
  }
#endif // PIDTEMPCHAMBER

#if ENABLED(PIDTEMPCOOLER)
  // M306: Set cooler PID parameters P I and D
  inline void gcode_M306() {
    if (code_seen('P')) thermalManager.coolerKp = code_value_float();
    if (code_seen('I')) thermalManager.coolerKi = scalePID_i(code_value_float());
    if (code_seen('D')) thermalManager.coolerKd = scalePID_d(code_value_float());

    thermalManager.updatePID();
    SERIAL_SMV(OK, " p:", thermalManager.coolerKp);
    SERIAL_MV(" i:", unscalePID_i(thermalManager.coolerKi));
    SERIAL_EMV(" d:", unscalePID_d(thermalManager.coolerKd));
  }
#endif // PIDTEMPCOOLER

#if HAS(ABL)

  // M320 Activate autolevel
  inline void gcode_M320() { set_bed_leveling_enabled(true); }

  // M321 Deactivate autoleveling
  inline void gcode_M321() { set_bed_leveling_enabled(false); }

  // M322 Reset auto leveling matrix
  inline void gcode_M322() {
    reset_bed_level();
    if (code_seen('S') && code_value_bool())
      eeprom.StoreSettings();
  }

#endif

#if HAS(MICROSTEPS)

  // M350 Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
  inline void gcode_M350() {
    if (code_seen('S')) for (int i = 0; i <= 4; i++) stepper.microstep_mode(i, code_value_byte());
    LOOP_XYZE(i) if (code_seen(axis_codes[i])) stepper.microstep_mode(i, code_value_byte());
    if (code_seen('B')) stepper.microstep_mode(4, code_value_byte());
    stepper.microstep_readings();
  }

  /**
   * M351: Toggle MS1 MS2 pins directly with axis codes X Y Z E B
   *       S# determines MS1 or MS2, X# sets the pin high/low.
   */
  inline void gcode_M351() {
    if (code_seen('S')) switch(code_value_byte()) {
      case 1:
        LOOP_XYZE(i) if (code_seen(axis_codes[i])) stepper.microstep_ms(i, code_value_byte(), -1);
        if (code_seen('B')) stepper.microstep_ms(4, code_value_byte(), -1);
        break;
      case 2:
        LOOP_XYZE(i) if (code_seen(axis_codes[i])) stepper.microstep_ms(i, -1, code_value_byte());
        if (code_seen('B')) stepper.microstep_ms(4, -1, code_value_byte());
        break;
    }
    stepper.microstep_readings();
  }

#endif // HAS(MICROSTEPS)

#if HAS(CASE_LIGHT)

  /**
   * M355: Turn case lights on/off
   *
   *   S<bool>  Turn case light on or off
   *   P<byte>  Set case light brightness (PWM pin required)
   *
   */
  inline void gcode_M355() {
    static bool case_light_on
      #if ENABLED(CASE_LIGHT_DEFAULT_ON)
        = true;
      #else
        ;
      #endif
    static uint8_t case_light_brightness = 255;
    if (code_seen('P')) case_light_brightness = code_value_byte();
    if (code_seen('S')) {
      case_light_on = code_value_bool();
      digitalWrite(CASE_LIGHT_PIN, case_light_on ? HIGH : LOW);
      analogWrite(CASE_LIGHT_PIN, case_light_on ? case_light_brightness : 0);
    }
    SERIAL_SM(ECHO, "Case lights ");
    case_light_on ? SERIAL_EM("on") : SERIAL_EM("off");
  }

#endif // HAS_CASE_LIGHT

#if MECH(MORGAN_SCARA)

  bool SCARA_move_to_cal(uint8_t delta_a, uint8_t delta_b) {
    if (IsRunning()) {
      forward_kinematics_SCARA(delta_a, delta_b);
      destination[X_AXIS] = LOGICAL_X_POSITION(cartes[X_AXIS]);
      destination[Y_AXIS] = LOGICAL_Y_POSITION(cartes[Y_AXIS]);
      destination[Z_AXIS] = current_position[Z_AXIS];
      prepare_move_to_destination();
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
   * M364: SCARA calibration: Move to cal-position PSIC (90 deg to Theta calibration position)
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
        #if HAS(SOLENOID_1)
          case 1:
            OUT_WRITE(SOL1_PIN, HIGH);
            break;
        #endif
        #if HAS(SOLENOID_2)
          case 2:
            OUT_WRITE(SOL2_PIN, HIGH);
            break;
        #endif
        #if HAS(SOLENOID_3)
          case 3:
            OUT_WRITE(SOL3_PIN, HIGH);
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
    OUT_WRITE(SOL1_PIN, LOW);
    OUT_WRITE(SOL2_PIN, LOW);
    OUT_WRITE(SOL3_PIN, LOW);
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

#if HAS(BED_PROBE)

  /**
   * M401: Engage Z Servo endstop if available
   */
  inline void gcode_M401() { DEPLOY_PROBE(); }

  /**
   * M402: Retract Z Servo endstop if enabled
   */
  inline void gcode_M402() { STOW_PROBE(); }

#endif // (ENABLED(AUTO_BED_LEVELING_FEATURE) && DISABLED(Z_PROBE_SLED) && HAS(Z_SERVO_ENDSTOP))

#if ENABLED(FILAMENT_SENSOR)

  /**
   * M404: Display or set (in current units) the nominal filament width (3mm, 1.75mm ) W<3.0>
   */
  inline void gcode_M404() {
    if (code_seen('W')) {
      filament_width_nominal = code_value_linear_units();
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
    // everything else, it uses code_value_int() instead of code_value_linear_units().
    if (code_seen('D')) meas_delay_cm = code_value_int();
    NOMORE(meas_delay_cm, MAX_MEASUREMENT_DELAY);

    if (filwidth_delay_index[1] == -1) { // Initialize the ring buffer if not done since startup
      int temp_ratio = widthFil_to_size_ratio();

      for (uint8_t i = 0; i < COUNT(measurement_delay); ++i)
        measurement_delay[i] = temp_ratio - 100;  // Subtract 100 to scale within a signed byte

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

    if (code_seen('S')) type = code_value_byte();

    SERIAL_M("{\"status\":\"");
    #if ENABLED(SDSUPPORT)
      if (!print_job_counter.isRunning() && !card.sdprinting) SERIAL_M("I"); // IDLING
      else if (card.sdprinting) SERIAL_M("P");          // SD PRINTING
      else SERIAL_M("B");                               // SOMETHING ELSE, BUT SOMETHIG
    #else
      if (!print_job_counter.isRunning()) SERIAL_M("I");                     // IDLING
      else SERIAL_M("B");                               // SOMETHING ELSE, BUT SOMETHIG
    #endif

    SERIAL_M("\",\"coords\": {");
    SERIAL_M("\"axesHomed\":[");
    if (axis_was_homed & (_BV(X_AXIS)|_BV(Y_AXIS)|_BV(Z_AXIS)) == (_BV(X_AXIS)|_BV(Y_AXIS)|_BV(Z_AXIS)))
      SERIAL_M("1, 1, 1");
    else
      SERIAL_M("0, 0, 0");

    SERIAL_MV("],\"extr\":[", current_position[E_AXIS]);
    SERIAL_MV("],\"xyz\":[", current_position[X_AXIS]); // X
    SERIAL_MV(",", current_position[Y_AXIS]); // Y
    SERIAL_MV(",", current_position[Z_AXIS]); // Z

    SERIAL_MV("]},\"currentTool\":", active_extruder);

    #if HAS(POWER_SWITCH)
      SERIAL_M(",\"params\": {\"atxPower\":");
      SERIAL_M(powersupply ? "1" : "0");
    #else
      SERIAL_M(",\"params\": {\"NormPower\":");
    #endif

    SERIAL_M(",\"fanPercent\":[");
    SERIAL_V(fanSpeed);

    SERIAL_MV("],\"speedFactor\":", feedrate_percentage);

    SERIAL_M(",\"extrFactors\":[");
    firstOccurrence = true;
    for (uint8_t i = 0; i < EXTRUDERS; i++) {
      if (!firstOccurrence) SERIAL_M(",");
      SERIAL_V(flow_percentage[i]); // Really *100? 100 is normal
      firstOccurrence = false;
    }
    SERIAL_EM("]},");

    SERIAL_M("\"temps\": {");
    #if HAS(TEMP_BED)
      SERIAL_MV("\"bed\": {\"current\":", thermalManager.degBed(), 1);
      SERIAL_MV(",\"active\":", thermalManager.degTargetBed(), 1);
      SERIAL_M(",\"state\":");
      SERIAL_M(thermalManager.degTargetBed() > 0 ? "2" : "1");
      SERIAL_M("},");
    #endif
    SERIAL_M("\"heads\": {\"current\":[");
    firstOccurrence = true;
    for (int8_t h = 0; h < HOTENDS; h++) {
      if (!firstOccurrence) SERIAL_M(",");
      SERIAL_V(thermalManager.degHotend(h), 1);
      firstOccurrence = false;
    }
    SERIAL_M("],\"active\":[");
    firstOccurrence = true;
    for (int8_t h = 0; h < HOTENDS; h++) {
      if (!firstOccurrence) SERIAL_M(",");
      SERIAL_V(thermalManager.degTargetHotend(h), 1);
      firstOccurrence = false;
    }
    SERIAL_M("],\"state\":[");
    firstOccurrence = true;
    for (int8_t h = 0; h < HOTENDS; h++) {
      if (!firstOccurrence) SERIAL_M(",");
      SERIAL_M(thermalManager.degTargetHotend(h) > HOTEND_AUTO_FAN_TEMPERATURE ? "2" : "1");
      firstOccurrence = false;
    }

    SERIAL_MV("]}},\"time\":", HAL::timeInMilliseconds());

    switch (type) {
      case 0:
      case 1:
        break;
      case 2:
        SERIAL_EM(",");
        SERIAL_M("\"coldExtrudeTemp\":0,\"coldRetractTemp\":0.0,\"geometry\":\"");
        #if MECH(CARTESIAN)
          SERIAL_M("cartesian");
        #elif MECH(COREXY)
          SERIAL_M("corexy");
        #elif MECH(COREYX)
          SERIAL_M("coreyx");
        #elif MECH(COREXZ)
          SERIAL_M("corexz");
        #elif MECH(COREZX)
          SERIAL_M("corezx");
        #elif MECH(DELTA)
          SERIAL_M("delta");
        #endif
        SERIAL_M("\",\"name\":\"");
        SERIAL_M(CUSTOM_MACHINE_NAME);
        SERIAL_M("\",\"tools\":[");
        firstOccurrence = true;
        for (uint8_t i = 0; i < EXTRUDERS; i++) {
          if (!firstOccurrence) SERIAL_M(",");
          SERIAL_MV("{\"number\":", i + 1);
          #if HOTENDS > 1
            SERIAL_MV(",\"heaters\":[", i + 1);
            SERIAL_M("],");
          #else
            SERIAL_M(",\"heaters\":[1],");
          #endif
          #if DRIVER_EXTRUDERS > 1
            SERIAL_MV("\"drives\":[", i);
            SERIAL_M("]");
          #else
            SERIAL_M("\"drives\":[0]");
          #endif
          SERIAL_M("}");
          firstOccurrence = false;
        }
        break;
      case 3:
        SERIAL_EM(",");
        SERIAL_M("\"currentLayer\":");
        #if ENABLED(SDSUPPORT)
          if (card.sdprinting && card.layerHeight > 0) { // ONLY CAN TELL WHEN SD IS PRINTING
            SERIAL_V((int) (current_position[Z_AXIS] / card.layerHeight));
          }
          else SERIAL_V(0);
        #else
          SERIAL_V(-1);
        #endif
        SERIAL_M(",\"extrRaw\":[");
        firstOccurrence = true;
        for (uint8_t i = 0; i < EXTRUDERS; i++) {
          if (!firstOccurrence) SERIAL_M(",");
          SERIAL_V(current_position[E_AXIS] * flow_percentage[i]);
          firstOccurrence = false;
        }
        SERIAL_M("],");
        #if ENABLED(SDSUPPORT)
          if (card.sdprinting) {
            SERIAL_M("\"fractionPrinted\":");
            float fractionprinted;
            if (card.fileSize < 2000000) {
              fractionprinted = (float)card.sdpos / (float)card.fileSize;
            }
            else fractionprinted = (float)(card.sdpos >> 8) / (float)(card.fileSize >> 8);
            SERIAL_V((float) floorf(fractionprinted * 1000) / 1000);
            SERIAL_M(",");
          }
        #endif
        SERIAL_M("\"firstLayerHeight\":");
        #if ENABLED(SDSUPPORT)
          if (card.sdprinting) SERIAL_V(card.firstlayerHeight);
          else SERIAL_M("0");
        #else
          SERIAL_M("0");
        #endif
        break;
      case 4:
      case 5:
        SERIAL_EM(",");
        SERIAL_M("\"axisMins\":[");
        SERIAL_V((int) X_MIN_POS);
        SERIAL_M(",");
        SERIAL_V((int) Y_MIN_POS);
        SERIAL_M(",");
        SERIAL_V((int) Z_MIN_POS);
        SERIAL_M("],\"axisMaxes\":[");
        SERIAL_V((int) X_MAX_POS);
        SERIAL_M(",");
        SERIAL_V((int) Y_MAX_POS);
        SERIAL_M(",");
        SERIAL_V((int) Z_MAX_POS);
        SERIAL_M("],\"planner.accelerations\":[");
        SERIAL_V(planner.acceleration_units_per_sq_second[X_AXIS]);
        SERIAL_M(",");
        SERIAL_V(planner.acceleration_units_per_sq_second[Y_AXIS]);
        SERIAL_M(",");
        SERIAL_V(planner.acceleration_units_per_sq_second[Z_AXIS]);
        for (uint8_t i = 0; i < EXTRUDERS; i++) {
          SERIAL_M(",");
          SERIAL_V(planner.acceleration_units_per_sq_second[E_AXIS + i]);
        }
        SERIAL_M("],");

        #if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
          SERIAL_M("\"currents\":[");
          SERIAL_V(motor_current[X_AXIS]);
          SERIAL_M(",");
          SERIAL_V(motor_current[Y_AXIS]);
          SERIAL_M(",");
          SERIAL_V(motor_current[Z_AXIS]);
          for (uint8_t i = 0; i < DRIVER_EXTRUDERS; i++) {
            SERIAL_M(",");
            SERIAL_V(motor_current[E_AXIS + i]);
          }
          SERIAL_EM("],");
        #endif

        SERIAL_M("\"firmwareElectronics\":\"");
        #if MB(RAMPS_13_HFB) || MB(RAMPS_13_HHB) || MB(RAMPS_13_HFF) || MB(RAMPS_13_HHF) || MB(RAMPS_13_HHH)
          SERIAL_M("RAMPS");
        #elif MB(ALLIGATOR)
          SERIAL_M("ALLIGATOR");
        #elif MB(ALLIGATOR_V3)
          SERIAL_M("ALLIGATOR_V3");
        #elif MB(RADDS) || MB(RAMPS_FD_V1) || MB(RAMPS_FD_V2) || MB(SMART_RAMPS) || MB(RAMPS4DUE)
          SERIAL_M("Arduino due");
        #elif MB(ULTRATRONICS)
          SERIAL_M("ULTRATRONICS");
        #else
          SERIAL_M("AVR");
        #endif
        SERIAL_M("\",\"firmwareName\":\"");
        SERIAL_M(FIRMWARE_NAME);
        SERIAL_M(",\"firmwareVersion\":\"");
        SERIAL_M(SHORT_BUILD_VERSION);
        SERIAL_M("\",\"firmwareDate\":\"");
        SERIAL_M(STRING_DISTRIBUTION_DATE);

        SERIAL_M("\",\"minFeedrates\":[0,0,0");
        for (uint8_t i = 0; i < EXTRUDERS; i++) {
          SERIAL_M(",0");
        }
        SERIAL_M("],\"maxFeedrates\":[");
        SERIAL_V(planner.max_feedrate_mm_s[X_AXIS]);
        SERIAL_M(",");
        SERIAL_V(planner.max_feedrate_mm_s[Y_AXIS]);
        SERIAL_M(",");
        SERIAL_V(planner.max_feedrate_mm_s[Z_AXIS]);
        for (uint8_t i = 0; i < EXTRUDERS; i++) {
          SERIAL_M(",");
          SERIAL_V(planner.max_feedrate_mm_s[E_AXIS + i]);
        }
        SERIAL_M("]");
        break;
    }
    SERIAL_EM("}");
  }
#endif // JSON_OUTPUT

/**
 * M410: Quickstop - Abort all planned moves
 *
 * This will stop the carriages mid-move, so most likely they
 * will be out of sync with the stepper position after this.
 */
inline void gcode_M410() { quickstop_stepper(); }

#if ENABLED(MESH_BED_LEVELING)
  /**
   * M420: Enable/Disable Mesh Bed Leveling
   */
  inline void gcode_M420() { if (code_seen('S')) set_bed_leveling_enabled(code_value_bool()); }

  /**
   * M421: Set a single Mesh Bed Leveling Z coordinate
   * Use either 'M421 X<mm> Y<mm> Z<mm>' or 'M421 I<xindex> J<yindex> Z<mm>'
   */
  inline void gcode_M421() {
    int8_t px, py;
    float z = 0;
    bool hasX, hasY, hasZ, hasI, hasJ;
    if ((hasX = code_seen('X'))) px = mbl.probe_index_x(code_value_axis_units(X_AXIS));
    if ((hasY = code_seen('Y'))) py = mbl.probe_index_y(code_value_axis_units(Y_AXIS));
    if ((hasI = code_seen('I'))) px = code_value_axis_units(X_AXIS);
    if ((hasJ = code_seen('J'))) py = code_value_axis_units(Y_AXIS);
    if ((hasZ = code_seen('Z'))) z = code_value_axis_units(Z_AXIS);

    if (hasX && hasY && hasZ) {

      if (px >= 0 && py >= 0)
        mbl.set_z(px, py, z);
      else {
        SERIAL_LM(ER, MSG_ERR_MESH_XY);
      }
    }
    else if (hasI && hasJ && hasZ) {
      if (px >= 0 && px < MESH_NUM_X_POINTS && py >= 0 && py < MESH_NUM_Y_POINTS)
        mbl.set_z(px, py, z);
      else {
        SERIAL_LM(ER, MSG_ERR_MESH_XY);
      }
    }
    else {
      SERIAL_LM(ER, MSG_ERR_M421_PARAMETERS);
    }
  }
#endif // MESH_BED_LEVELING

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
    if (axis_homed[i]) {
      #if MECH(DELTA)
        float base = (current_position[i] > (soft_endstop_min[i] + soft_endstop_max[i]) / 2) ? base_home_pos[i] : 0,
              diff = current_position[i] - LOGICAL_POSITION(base, i);
      #else
        float base = (current_position[i] > (soft_endstop_min[i] + soft_endstop_max[i]) / 2) ? base_home_pos(i) : 0,
              diff = current_position[i] - LOGICAL_POSITION(base, i);
      #endif
      if (diff > -20 && diff < 20) {
        set_home_offset((AxisEnum)i, home_offset[i] - diff);
      }
      else {
        SERIAL_LM(ER, MSG_ERR_M428_TOO_FAR);
        LCD_ALERTMESSAGEPGM("Err: Too far!");
        #if HAS(BUZZER)
          buzz(200, 40);
        #endif
        err = true;
        break;
      }
    }
  }

  if (!err) {
    SYNC_PLAN_POSITION_KINEMATIC();
    report_current_position();
    SERIAL_EM("Offset applied.");
    LCD_MESSAGEPGM("Offset applied.");
    #if HAS(BUZZER)
      buzz(200, 659);
      buzz(200, 698);
    #endif
  }
}

/**
 * M500: Store settings in EEPROM
 */
inline void gcode_M500() {
  eeprom.StoreSettings();
}

/**
 * M501: Read settings from EEPROM
 */
inline void gcode_M501() {
  eeprom.RetrieveSettings();
}

/**
 * M502: Revert to default settings
 */
inline void gcode_M502() {
  eeprom.ResetDefault();
}

/**
 * M503: print settings currently in memory
 */
inline void gcode_M503() {
  eeprom.PrintSettings(code_seen('S') && !code_value_bool());
}

#if ENABLED(RFID_MODULE)
  /**
   * M522: Read or Write on card. M522 T<extruders> R<read> or W<write> L<list>
   */
  inline void gcode_M522() {
    if (get_target_extruder_from_command(522)) return;
    if (!RFID_ON) return;

    if (code_seen('R')) {
      SERIAL_EM("Put RFID on tag!");
      #if ENABLED(NEXTION)
        rfid_setText("Put RFID on tag!");
      #endif
      Spool_must_read[target_extruder] = true;
    }
    if (code_seen('W')) {
      if (Spool_ID[target_extruder] != 0) {
        SERIAL_EM("Put RFID on tag!");
        #if ENABLED(NEXTION)
          rfid_setText("Put RFID on tag!");
        #endif
        Spool_must_write[target_extruder] = true;
      }
      else {
        SERIAL_LM(ER, "You have not read this Spool!");
        #if ENABLED(NEXTION)
          rfid_setText("You have not read this Spool!", 64488);
        #endif
      }
    }

    if (code_seen('L')) RFID522.printInfo(target_extruder);
  }
#endif // RFID_MODULE

/**
 * M530: S<printing> L<layer> - Enables explicit printing mode (S1) or disables it (S0). L can set layer count
 */
inline void gcode_M530() {

  if (code_seen('L')) maxLayer = code_value_long();
  
  if (code_seen('S') && code_value_bool()) {
    print_job_counter.start();

    SERIAL_M("Start Printing");
    if (maxLayer > 0) SERIAL_EMV(" - MaxLayer:", maxLayer);
    else SERIAL_E;

    #if ENABLED(START_GCODE)
      enqueue_and_echo_commands_P(PSTR(START_PRINTING_SCRIPT));
    #endif
    #if HAS(FIL_RUNOUT)
      filament_ran_out = false;
      SERIAL_EM("Filament runout activated.");
      SERIAL_S(RESUME);
      SERIAL_E;
    #endif
    #if HAS(POWER_CONSUMPTION_SENSOR)
      startpower = power_consumption_hour;
    #endif
  }
  else {
    print_job_counter.stop();
    SERIAL_EM("Stop Printing");
    #if ENABLED(STOP_GCODE)
      enqueue_and_echo_commands_P(PSTR(STOP_PRINTING_SCRIPT));
    #endif
    #if HAS(FIL_RUNOUT)
      filament_ran_out = false;
      SERIAL_EM("Filament runout deactivated.");
    #endif
  }
}

/**
 * M531: filename - Define filename being printed
 */
inline void gcode_M531() {
  strncpy(printName, current_command_args, 20);
  printName[20] = 0;
}

/**
 * M532: X<percent> L<curLayer> - update current print state progress (X=0..100) and layer L
 */
inline void gcode_M532() {
  if (code_seen('X'))
    progress = code_value_float();
  if (progress > 100.0)
    progress = 100.0;
  else if (progress < 0)
    progress = 0;

  if (code_seen('L'))
    currentLayer = code_value_long();
}

#if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)

  /**
   * M540: Set whether SD card print should abort on endstop hit (M540 S<0|1>)
   */
  inline void gcode_M540() {
    if (code_seen('S')) stepper.abort_on_endstop_hit = code_value_bool();
  }

#endif // ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED

#if HEATER_USES_AD595
  /**
   * M595 - set Hotend AD595 offset & Gain H<hotend_number> O<offset> S<gain>
   */
  inline void gcode_M595() {
    if (get_target_hotend_from_command(595)) return;

    if (code_seen('O')) ad595_offset[target_extruder] = code_value_float();
    if (code_seen('S')) ad595_gain[target_extruder] = code_value_float();

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

#if ENABLED(FILAMENT_CHANGE_FEATURE)

  /**
   * M600: Pause for filament change
   *
   *  E[distance] - Retract the filament this far (negative value)
   *  Z[distance] - Move the Z axis by this distance
   *  X[position] - Move to this X position, with Y
   *  Y[position] - Move to this Y position, with X
   *  L[distance] - Retract distance for removal (manual reload)
   *
   *  Default values are used for omitted arguments.
   *
   */
  inline void gcode_M600() {

    if (thermalManager.tooColdToExtrude(active_extruder)) {
      SERIAL_LM(ER, MSG_TOO_COLD_FOR_FILAMENTCHANGE);
      return;
    }

    // Show initial message and wait for synchronize steppers
    lcd_filament_change_show_message(FILAMENT_CHANGE_MESSAGE_INIT);
    stepper.synchronize();

    float lastpos[NUM_AXIS];

    // Save current position of all axes
    LOOP_XYZE(i)
      lastpos[i] = destination[i] = current_position[i];

    // Define runplan for move axes
    #if IS_KINEMATIC
			#define RUNPLAN(RATE_MM_S) planner.buffer_line_kinematic(destination, RATE_MM_S, active_extruder, active_driver);
    #else
      #define RUNPLAN(RATE_MM_S) line_to_destination(RATE_MM_S);
    #endif

    KEEPALIVE_STATE(IN_HANDLER);

    // Initial retract before move to filament change position
    if (code_seen('E')) destination[E_AXIS] += code_value_axis_units(E_AXIS);
    #if ENABLED(FILAMENT_CHANGE_RETRACT_LENGTH) && FILAMENT_CHANGE_RETRACT_LENGTH > 0
      else destination[E_AXIS] -= FILAMENT_CHANGE_RETRACT_LENGTH;
    #endif

    RUNPLAN(FILAMENT_CHANGE_RETRACT_FEEDRATE);

    // Lift Z axis
    float z_lift = code_seen('Z') ? code_value_axis_units(Z_AXIS) :
      #if ENABLED(FILAMENT_CHANGE_Z_ADD) && FILAMENT_CHANGE_Z_ADD > 0
        FILAMENT_CHANGE_Z_ADD
      #else
        0
      #endif
    ;

    if (z_lift > 0) {
      destination[Z_AXIS] += z_lift;
      NOMORE(destination[Z_AXIS], Z_MAX_POS);
      RUNPLAN(FILAMENT_CHANGE_Z_FEEDRATE);
    }

    // Move XY axes to filament exchange position
    if (code_seen('X')) destination[X_AXIS] = code_value_axis_units(X_AXIS);
    #if ENABLED(FILAMENT_CHANGE_X_POS)
      else destination[X_AXIS] = FILAMENT_CHANGE_X_POS;
    #endif

    if (code_seen('Y')) destination[Y_AXIS] = code_value_axis_units(Y_AXIS);
    #if ENABLED(FILAMENT_CHANGE_Y_POS)
      else destination[Y_AXIS] = FILAMENT_CHANGE_Y_POS;
    #endif

    RUNPLAN(FILAMENT_CHANGE_XY_FEEDRATE);

    stepper.synchronize();
    lcd_filament_change_show_message(FILAMENT_CHANGE_MESSAGE_UNLOAD);

    // Unload filament
    if (code_seen('L')) destination[E_AXIS] += code_value_axis_units(E_AXIS);
    #if ENABLED(FILAMENT_CHANGE_UNLOAD_LENGTH) && FILAMENT_CHANGE_UNLOAD_LENGTH > 0
      else destination[E_AXIS] -= FILAMENT_CHANGE_UNLOAD_LENGTH;
    #endif

    RUNPLAN(FILAMENT_CHANGE_UNLOAD_FEEDRATE);

    // Synchronize steppers and then disable extruders steppers for manual filament changing
    stepper.synchronize();
    //disable extruder steppers so filament can be removed
    disable_e();
    safe_delay(100);
    boolean beep = true;
    boolean sleep = false;
    uint8_t cnt = 0;

    int old_target_temperature[HOTENDS] = { 0 };
    for (int8_t h = 0; h < HOTENDS; h++) {
      old_target_temperature[h] = thermalManager.target_temperature[h];
    }
    int old_target_temperature_bed = thermalManager.target_temperature_bed;
    millis_t last_set = millis();

    // Wait for filament insert by user and press button
    lcd_filament_change_show_message(FILAMENT_CHANGE_MESSAGE_INSERT);

    // LCD click or M108 will clear this
    wait_for_user = true;

    while (wait_for_user) {
      if ((millis() - last_set > 60000) && cnt <= FILAMENT_CHANGE_PRINTER_OFF) beep = true;
      if (cnt >= FILAMENT_CHANGE_PRINTER_OFF && !sleep) {
        thermalManager.disable_all_heaters();
        thermalManager.disable_all_coolers();
        sleep = true;
        lcd_reset_alert_level();
        LCD_ALERTMESSAGEPGM("Zzzz Zzzz Zzzz");
      }
      if (beep) {
        #if HAS(BUZZER)
          for(uint8_t i = 0; i < 3; i++) buzz(300, 1000);
        #endif
        last_set = millis();
        beep = false;
        ++cnt;
      }
      idle(true);
    } // while(wait_for_user)

    // Reset LCD alert message
    lcd_reset_alert_level();

    if (sleep) {
      stepper.enable_all_steppers(); // Enable all stepper
      HOTEND_LOOP() {
        thermalManager.setTargetHotend(old_target_temperature[h], h);
        wait_heater();
      }
      #if HAS(TEMP_BED)
        thermalManager.setTargetBed(old_target_temperature_bed);
        wait_bed();
      #endif
    }

    // Show load message
    lcd_filament_change_show_message(FILAMENT_CHANGE_MESSAGE_LOAD);

    // Load filament
    if (code_seen('L')) destination[E_AXIS] -= code_value_axis_units(E_AXIS);
    #if ENABLED(FILAMENT_CHANGE_LOAD_LENGTH) && FILAMENT_CHANGE_LOAD_LENGTH > 0
      else destination[E_AXIS] += FILAMENT_CHANGE_LOAD_LENGTH;
    #endif

    RUNPLAN(FILAMENT_CHANGE_LOAD_FEEDRATE);
    stepper.synchronize();

    #if ENABLED(FILAMENT_CHANGE_EXTRUDE_LENGTH) && FILAMENT_CHANGE_EXTRUDE_LENGTH > 0
      do {
        // Extrude filament to get into hotend
        lcd_filament_change_show_message(FILAMENT_CHANGE_MESSAGE_EXTRUDE);
        destination[E_AXIS] += FILAMENT_CHANGE_EXTRUDE_LENGTH;
        RUNPLAN(FILAMENT_CHANGE_EXTRUDE_FEEDRATE);
        stepper.synchronize();
        // Ask user if more filament should be extruded
        KEEPALIVE_STATE(PAUSED_FOR_USER);
        lcd_filament_change_show_message(FILAMENT_CHANGE_MESSAGE_OPTION);
        while (filament_change_menu_response == FILAMENT_CHANGE_RESPONSE_WAIT_FOR) idle(true);
        KEEPALIVE_STATE(IN_HANDLER);
      } while (filament_change_menu_response != FILAMENT_CHANGE_RESPONSE_RESUME_PRINT);
    #endif

    lcd_filament_change_show_message(FILAMENT_CHANGE_MESSAGE_RESUME);

    KEEPALIVE_STATE(IN_HANDLER);

    // Set extruder to saved position
    destination[E_AXIS] = current_position[E_AXIS] = lastpos[E_AXIS];
    planner.set_e_position_mm(current_position[E_AXIS]);

    #if IS_KINEMATIC
      // Move XYZ to starting position
      planner.buffer_line_kinematic(lastpos, FILAMENT_CHANGE_XY_FEEDRATE, active_extruder, active_driver);
    #else
      // Move XY to starting position, then Z
      destination[X_AXIS] = lastpos[X_AXIS];
      destination[Y_AXIS] = lastpos[Y_AXIS];
      RUNPLAN(FILAMENT_CHANGE_XY_FEEDRATE);
      destination[Z_AXIS] = lastpos[Z_AXIS];
      RUNPLAN(FILAMENT_CHANGE_Z_FEEDRATE);
    #endif
    stepper.synchronize();

    #if HAS(FIL_RUNOUT)
      filament_ran_out = false;
    #endif

    // Show status screen
    lcd_filament_change_show_message(FILAMENT_CHANGE_MESSAGE_STATUS);
  }

#endif // FILAMENT_CHANGE_FEATURE

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
    if (code_seen('S')) dual_x_carriage_mode = code_value_byte();
    switch(dual_x_carriage_mode) {
      case DXC_DUPLICATION_MODE:
        if (code_seen('X')) duplicate_hotend_x_offset = max(code_value_axis_units(X_AXIS), X2_MIN_POS - x_home_pos(0));
        if (code_seen('R')) duplicate_hotend_temp_offset = code_value_temp_diff();
        SERIAL_M(MSG_HOTEND_OFFSET);
        SERIAL_MV(" ", hotend_offset[X_AXIS][0]);
        SERIAL_MV(",", hotend_offset[Y_AXIS][0]);
        SERIAL_MV(" ", duplicate_hotend_x_offset);
        SERIAL_EMV(",", hotend_offset[Y_AXIS][1]);
        break;
      case DXC_FULL_CONTROL_MODE:
      case DXC_AUTO_PARK_MODE:
        break;
      default:
        dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;
        break;
    }
    active_hotend_parked = false;
    hotend_duplication_enabled = false;
    delayed_move_time = 0;
  }
#endif // DUAL_X_CARRIAGE

#if ENABLED(LASERBEAM)

  // M649 set laser options
  inline void gcode_M649() {
    // do this at the start so we can debug if needed!
    if (code_seen('D') && IsRunning()) laser.diagnostics = code_value_bool();

    // Wait for the rest 
    // stepper.synchronize();
    if (code_seen('S') && IsRunning()) {
      laser.intensity = code_value_float();
      laser.rasterlaserpower =  laser.intensity;
    }

    if (code_seen('L') && IsRunning()) laser.duration = code_value_ulong();
    if (code_seen('P') && IsRunning()) laser.ppm = code_value_float();
    if (code_seen('B') && IsRunning()) laser_set_mode(code_value_int());
    if (code_seen('R') && IsRunning()) laser.raster_mm_per_pulse = (code_value_float());

    if (code_seen('F')) {
      float next_feedrate = code_value_linear_units();
      if(next_feedrate > 0.0) feedrate_mm_s = next_feedrate;
    }
  }
#endif // LASERBEAM

#if HAS(BED_PROBE) && NOMECH(DELTA)
  // M666: Set Z probe offset
  inline void gcode_M666() {
    if (code_seen('P')) {
      float p_val = code_value_axis_units(Z_AXIS);
      if (Z_PROBE_OFFSET_RANGE_MIN <= p_val && p_val <= Z_PROBE_OFFSET_RANGE_MAX) {
        zprobe_zoffset = p_val;
      }
    }
    if (code_seen('L')) {
      SERIAL_EMV("P (ZProbe ZOffset): ", zprobe_zoffset, 3);
    }
  }

#elif MECH(DELTA)

  // M666: Set delta endstop and geometry adjustment
  inline void gcode_M666() {
    if (code_seen('A')) {
      tower_adj[0] = code_value_linear_units();
      set_delta_constants();
    }
    if (code_seen('B')) {
      tower_adj[1] = code_value_linear_units();
      set_delta_constants();
    }
    if (code_seen('C')) {
      tower_adj[2] = code_value_linear_units();
      set_delta_constants();
    }
    if (code_seen('I')) {
      tower_adj[3] = code_value_linear_units();
      set_delta_constants();
    }
    if (code_seen('J')) {
      tower_adj[4] = code_value_linear_units();
      set_delta_constants();
    }
    if (code_seen('K')) {
      tower_adj[5] = code_value_linear_units();
      set_delta_constants();
    }
    if (code_seen('U')) {
      diagrod_adj[0] = code_value_linear_units();
      set_delta_constants();
    }
    if (code_seen('V')) {
      diagrod_adj[1] = code_value_linear_units();
      set_delta_constants();
    }
    if (code_seen('W')) {
      diagrod_adj[2] = code_value_linear_units();
      set_delta_constants();
    }
    if (code_seen('R')) {
      delta_radius = code_value_linear_units();
      set_delta_constants();
    }
    if (code_seen('D')) {
      delta_diagonal_rod = code_value_linear_units();
      set_delta_constants();
    }
    if (code_seen('H')) {
      soft_endstop_max[Z_AXIS] = code_value_axis_units(Z_AXIS);
      set_delta_constants();
    }
    if (code_seen('S')) {
      delta_segments_per_second = code_value_float();
    }

    #if HAS(BED_PROBE)
      if (code_seen('P')) {
        float p_val = code_value_axis_units(Z_AXIS);
        if (Z_PROBE_OFFSET_RANGE_MIN <= p_val && p_val <= Z_PROBE_OFFSET_RANGE_MAX) {
          zprobe_zoffset = p_val;
        }
      }
    #endif

    LOOP_XYZ(i) {
      if (code_seen(axis_codes[i])) endstop_adj[i] = code_value_axis_units(i);
    }

    if (code_seen('L')) {
      SERIAL_LM(CFG, "Current Delta geometry values:");
      LOOP_XYZ(i) {
        SERIAL_SV(CFG, axis_codes[i]);
        SERIAL_EMV(" (Endstop Adj): ", endstop_adj[i], 3);
      }

      #if HAS(BED_PROBE)
        SERIAL_LMV(CFG, "P (ZProbe ZOffset): ", zprobe_zoffset, 3);
      #endif

      SERIAL_LMV(CFG, "A (Tower A Position Correction): ", tower_adj[0], 3);
      SERIAL_LMV(CFG, "B (Tower B Position Correction): ", tower_adj[1], 3);
      SERIAL_LMV(CFG, "C (Tower C Position Correction): ", tower_adj[2], 3);
      SERIAL_LMV(CFG, "I (Tower A Radius Correction): ", tower_adj[3], 3);
      SERIAL_LMV(CFG, "J (Tower B Radius Correction): ", tower_adj[4], 3);
      SERIAL_LMV(CFG, "K (Tower C Radius Correction): ", tower_adj[5], 3);
      SERIAL_LMV(CFG, "U (Tower A Diagonal Rod Correction): ", diagrod_adj[0], 3);
      SERIAL_LMV(CFG, "V (Tower B Diagonal Rod Correction): ", diagrod_adj[1], 3);
      SERIAL_LMV(CFG, "W (Tower C Diagonal Rod Correction): ", diagrod_adj[2], 3);
      SERIAL_LMV(CFG, "R (Delta Radius): ", delta_radius, 4);
      SERIAL_LMV(CFG, "D (Diagonal Rod Length): ", delta_diagonal_rod, 4);
      SERIAL_LMV(CFG, "S (Delta Segments per second): ", delta_segments_per_second, 1);
      SERIAL_LMV(CFG, "H (Z-Height): ", soft_endstop_max[Z_AXIS], 3);
    }
  }
#endif // MECH DELTA

#if ENABLED(LIN_ADVANCE)
  /**
   * M905: Set advance factor
   */
  inline void gcode_M905() {
    stepper.synchronize();
    planner.advance_M905(code_seen('K') ? code_value_float() : -1.0);
  }
#endif

#if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
  /**
   * M906: Set motor currents
   */
  inline void gcode_M906() {
    if (get_target_extruder_from_command(906)) return;
    LOOP_XYZE(i) {
      if (code_seen(axis_codes[i])) {
        if (i == E_AXIS)
          motor_current[i + target_extruder] = code_value_float();
        else
          motor_current[i] = code_value_float();
      }
    }
    stepper.set_driver_current();
  }
#endif // ALLIGATOR

/**
 * M907: Set digital trimpot motor current using axis codes X, Y, Z, E, B, S
 */
inline void gcode_M907() {
  #if HAS(DIGIPOTSS)
    LOOP_XYZE(i)
      if (code_seen(axis_codes[i])) stepper.digipot_current(i, code_value_int());
    if (code_seen('B')) stepper.digipot_current(4, code_value_int());
    if (code_seen('S')) for (uint8_t i = 0; i <= 4; i++) stepper.digipot_current(i, code_value_int());
  #elif HAS_MOTOR_CURRENT_PWM
    #if PIN_EXISTS(MOTOR_CURRENT_PWM_XY)
      if (code_seen('X')) stepper.digipot_current(0, code_value_int());
    #endif
    #if PIN_EXISTS(MOTOR_CURRENT_PWM_Z)
      if (code_seen('Z')) stepper.digipot_current(1, code_value_int());
    #endif
    #if PIN_EXISTS(MOTOR_CURRENT_PWM_E)
      if (code_seen('E')) stepper.digipot_current(2, code_value_int());
    #endif
  #endif
  #if ENABLED(DIGIPOT_I2C)
    // this one uses actual amps in floating point
    LOOP_XYZE(i) if (code_seen(axis_codes[i])) digipot_i2c_set_current(i, code_value_float());
    // for each additional extruder (named B,C,D,E..., channels 4,5,6,7...)
    for (uint8_t i = NUM_AXIS; i < DIGIPOT_I2C_NUM_CHANNELS; i++) if(code_seen('B' + i - (NUM_AXIS))) digipot_i2c_set_current(i, code_value_float());
  #endif
}

#if HAS(DIGIPOTSS)
  /**
   * M908: Control digital trimpot directly (M908 P<pin> S<current>)
   */
  inline void gcode_M908() {
    digitalPotWrite(
      code_seen('P') ? code_value_int() : 0,
      code_seen('S') ? code_value_int() : 0
    );
  }
#endif // HAS(DIGIPOTSS)

#if ENABLED(NPR2)
  /**
   * M997: Cxx Move Carter xx gradi
   */
  inline void gcode_M997() {
    long csteps;
    if (code_seen('C')) {
      csteps = code_value_ulong() * color_step_moltiplicator;
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

  if (code_seen('S') && code_value_bool()) return;

  FlushSerialRequestResend();
}

/**
 * T0-T5: Switch tool, usually switching extruders
 *
 *   F[units/min] Set the movement feedrate
 *   S1           Don't move the tool in XY after change
 */
inline void gcode_T(uint8_t tmp_extruder) {

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_MV(">>> gcode_T(", tmp_extruder);
      SERIAL_C(')'); SERIAL_E;
      DEBUG_POS("BEFORE", current_position);
    }
  #endif

  #if HOTENDS == 1 || (ENABLED(COLOR_MIXING_EXTRUDER) && MIXING_VIRTUAL_TOOLS > 1)

    tool_change(tmp_extruder);

  #elif HOTENDS > 1

    tool_change(
      tmp_extruder,
      code_seen('F') ? MMM_TO_MMS(code_value_axis_units(X_AXIS)) : 0.0,
      (tmp_extruder == active_extruder) || (code_seen('S') && code_value_bool())
    );

  #endif

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      DEBUG_POS("AFTER", current_position);
      SERIAL_EM("<<< gcode_T");
    }
  #endif
}

#if ENABLED(MKSE6)
  inline void move_extruder_servo(uint8_t e) {
    const int angles[EXTRUDERS] = ARRAY_BY_EXTRUDERS_N (
      MKSE6_SERVOPOS_E0, MKSE6_SERVOPOS_E1,
      MKSE6_SERVOPOS_E2, MKSE6_SERVOPOS_E3,
      MKSE6_SERVOPOS_E4, MKSE6_SERVOPOS_E5
    );
    MOVE_SERVO(MKSE6_SERVO_INDEX, angles[e]);

    #if (MKSE6_SERVO_DELAY > 0)
      HAL::delayMilliseconds(MKSE6_SERVO_DELAY);
    #endif
  }
#endif

#if HAS(DONDOLO)
  inline void move_extruder_servo(uint8_t e) {
    const int angles[2] = { DONDOLO_SERVOPOS_E0, DONDOLO_SERVOPOS_E1 };
    MOVE_SERVO(DONDOLO_SERVO_INDEX, angles[e]);

    #if (DONDOLO_SERVO_DELAY > 0)
      HAL::delayMilliseconds(DONDOLO_SERVO_DELAY);
    #endif
  }
#endif

inline void invalid_extruder_error(const uint8_t &e) {
  SERIAL_SMV(ER, "T", (int)e);
  SERIAL_EM(" " MSG_INVALID_EXTRUDER);
}

void tool_change(const uint8_t tmp_extruder, const float fr_mm_s/*=0.0*/, bool no_move/*=false*/) {

  #if HOTENDS <= 1

    #if ENABLED(COLOR_MIXING_EXTRUDER) && MIXING_VIRTUAL_TOOLS > 1

      // T0-T15: Switch virtual tool by changing the mix
      if (tmp_extruder >= MIXING_VIRTUAL_TOOLS) {
        invalid_extruder_error(tmp_extruder);
        return;
      }

      // T0-Tnnn: Switch virtual tool by changing the mix
      for (uint8_t j = 0; j < MIXING_STEPPERS; j++) {
        mixing_factor[j] = mixing_virtual_tool_mix[tmp_extruder][j];
      }

      SERIAL_EMV(MSG_ACTIVE_COLOR, (int)tmp_extruder);

    #elif ENABLED(MKSE6)

      if (tmp_extruder >= EXTRUDERS) {
        invalid_extruder_error(tmp_extruder);
        return;
      }

      stepper.synchronize(); // Finish all movement

      move_extruder_servo(tmp_extruder);

      // Set the new active extruder
      previous_extruder = active_extruder;
      active_extruder = tmp_extruder;
      active_driver = 0;

      UNUSED(fr_mm_s);
      UNUSED(no_move);

    #elif ENABLED(NPR2)

      if (tmp_extruder >= EXTRUDERS) {
        invalid_extruder_error(tmp_extruder);
        return;
      }

      if (tmp_extruder != old_color) {
        long csteps;
        stepper.synchronize(); // Finish all movement

        if (old_color == 99)
          csteps = (color_position[tmp_extruder]) * color_step_moltiplicator;
        else
          csteps = (color_position[tmp_extruder] - color_position[old_color]) * color_step_moltiplicator;

        if (csteps < 0) stepper.colorstep(-csteps, false);
        if (csteps > 0) stepper.colorstep(csteps, true);

        // Set the new active extruder
        previous_extruder = active_extruder;
        old_color = active_extruder = tmp_extruder;
        active_driver = 0;
        SERIAL_EMV(MSG_ACTIVE_COLOR, (int)active_extruder);

        UNUSED(fr_mm_s);
        UNUSED(no_move);
      }

    #elif ENABLED(MKR4)

      if (tmp_extruder >= EXTRUDERS) {
        invalid_extruder_error(tmp_extruder);
        return;
      }

      #if (EXTRUDERS == 4) && HAS(E0E2) && HAS(E1E3) && (DRIVER_EXTRUDERS == 2)

        stepper.synchronize(); // Finish all movement
        disable_e();
        switch(tmp_extruder) {
          case 0:
            WRITE_RELE(E0E2_CHOICE_PIN, LOW);
            WRITE_RELE(E1E3_CHOICE_PIN, LOW);
            active_driver = 0;
            safe_delay(500); // 500 microseconds delay for relay
            enable_e0();
            break;
          case 1:
            WRITE_RELE(E0E2_CHOICE_PIN, LOW);
            WRITE_RELE(E1E3_CHOICE_PIN, LOW);
            active_driver = 1;
            safe_delay(500); // 500 microseconds delay for relay
            enable_e1();
            break;
          case 2:
            WRITE_RELE(E0E2_CHOICE_PIN, HIGH);
            WRITE_RELE(E1E3_CHOICE_PIN, LOW);
            active_driver = 0;
            safe_delay(500); // 500 microseconds delay for relay
            enable_e2();
            break;
          case 3:
            WRITE_RELE(E0E2_CHOICE_PIN, LOW);
            WRITE_RELE(E1E3_CHOICE_PIN, HIGH);
            active_driver = 1;
            safe_delay(500); // 500 microseconds delay for relay
            enable_e3();
            break;
        }

      #elif (EXTRUDERS == 3) && HAS(E0E2) && (DRIVER_EXTRUDERS == 2)

        stepper.synchronize(); // Finish all movement
        disable_e();
        switch(tmp_extruder) {
          case 0:
            WRITE_RELE(E0E2_CHOICE_PIN, LOW);
            active_driver = 0;
            safe_delay(500); // 500 microseconds delay for relay
            enable_e0();
            break;
          case 1:
            WRITE_RELE(E0E2_CHOICE_PIN, LOW);
            active_driver = 1;
            safe_delay(500); // 500 microseconds delay for relay
            enable_e1();
            break;
          case 2:
            WRITE_RELE(E0E2_CHOICE_PIN, HIGH);
            active_driver = 0;
            safe_delay(500); // 500 microseconds delay for relay
            enable_e0();
            break;
        }

      #elif (EXTRUDERS == 2) && HAS(E0E1) && (DRIVER_EXTRUDERS == 1)

        stepper.synchronize(); // Finish all movement
        disable_e();
        switch(tmp_extruder) {
          case 0:
            WRITE_RELE(E0E1_CHOICE_PIN, LOW);
            active_driver = 0;
            safe_delay(500); // 500 microseconds delay for relay
            enable_e0();
            break;
          case 1:
            WRITE_RELE(E0E1_CHOICE_PIN, HIGH);
            active_driver = 0;
            safe_delay(500); // 500 microseconds delay for relay
            enable_e0();
            break;
        }

      #endif // E0E1_CHOICE_PIN E0E2_CHOICE_PIN E1E3_CHOICE_PIN

      // Set the new active extruder
      previous_extruder = active_extruder;
      active_extruder = tmp_extruder;

      UNUSED(fr_mm_s);
      UNUSED(no_move);

    #elif ENABLED(MKR6)

      if (tmp_extruder >= EXTRUDERS) {
        invalid_extruder_error(tmp_extruder);
        return;
      }

      #if (EXTRUDERS == 2) && HAS(EX1) && (DRIVER_EXTRUDERS == 1)

        stepper.synchronize(); // Finish all movement
        disable_e();
        switch(tmp_extruder) {
          case 0:
            WRITE_RELE(EX1_CHOICE_PIN, LOW);
            active_driver = 0;
            safe_delay(500); // 500 microseconds delay for relay
            enable_e0();
            break;
          case 1:
            WRITE_RELE(EX1_CHOICE_PIN, HIGH);
            active_driver = 0;
            safe_delay(500); // 500 microseconds delay for relay
            enable_e0();
            break;
        }

      #elif (EXTRUDERS == 3) && HAS(EX1) && HAS(EX2) && (DRIVER_EXTRUDERS == 1)

        stepper.synchronize(); // Finish all movement
        disable_e();
        switch(tmp_extruder) {
          case 0:
            WRITE_RELE(EX1_CHOICE_PIN, LOW);
            WRITE_RELE(EX2_CHOICE_PIN, LOW);
            active_driver = 0;
            safe_delay(500); // 500 microseconds delay for relay
            enable_e0();
            break;
          case 1:
            WRITE_RELE(EX1_CHOICE_PIN, HIGH);
            WRITE_RELE(EX2_CHOICE_PIN, LOW);
            active_driver = 0;
            safe_delay(500); // 500 microseconds delay for relay
            enable_e0();
            break;
          case 2:
            WRITE_RELE(EX1_CHOICE_PIN, HIGH);
            WRITE_RELE(EX2_CHOICE_PIN, HIGH);
            active_driver = 0;
            safe_delay(500); // 500 microseconds delay for relay
            enable_e0();
            break;
        }

      #elif (EXTRUDERS > 3) && HAS(EX1) && HAS(EX2) && (DRIVER_EXTRUDERS == 2)

        stepper.synchronize(); // Finish all movement
        disable_e();
        switch(tmp_extruder) {
          case 0:
            WRITE_RELE(EX1_CHOICE_PIN, LOW);
            WRITE_RELE(EX2_CHOICE_PIN, LOW);
            active_driver = 0;
            safe_delay(500); // 500 microseconds delay for relay
            enable_e0();
            break;
          case 1:
            WRITE_RELE(EX1_CHOICE_PIN, HIGH);
            WRITE_RELE(EX2_CHOICE_PIN, LOW);
            active_driver = 0;
            safe_delay(500); // 500 microseconds delay for relay
            enable_e0();
            break;
          case 2:
            WRITE_RELE(EX1_CHOICE_PIN, HIGH);
            WRITE_RELE(EX2_CHOICE_PIN, HIGH);
            active_driver = 0;
            safe_delay(500); // 500 microseconds delay for relay
            enable_e0();
            break;
          case 3:
            WRITE_RELE(EX1_CHOICE_PIN, LOW);
            WRITE_RELE(EX2_CHOICE_PIN, LOW);
            active_driver = 1;
            safe_delay(500); // 500 microseconds delay for relay
            enable_e1();
            break;
          case 4:
            WRITE_RELE(EX1_CHOICE_PIN, HIGH);
            WRITE_RELE(EX2_CHOICE_PIN, LOW);
            active_driver = 1;
            safe_delay(500); // 500 microseconds delay for relay
            enable_e1();
            break;
          case 5:
            WRITE_RELE(EX1_CHOICE_PIN, HIGH);
            WRITE_RELE(EX2_CHOICE_PIN, HIGH);
            active_driver = 1;
            safe_delay(500); // 500 microseconds delay for relay
            enable_e1();
            break;
        }

      #endif

      // Set the new active extruder
      previous_extruder = active_extruder;
      active_extruder = tmp_extruder;
      
      UNUSED(fr_mm_s);
      UNUSED(no_move);

    #else

      if (tmp_extruder >= EXTRUDERS) {
        invalid_extruder_error(tmp_extruder);
        return;
      }

      // Set the new active extruder
      previous_extruder = active_extruder;
      active_driver = active_extruder = tmp_extruder;

      UNUSED(fr_mm_s);
      UNUSED(no_move);

    #endif

  #else // HOTENDS > 1

    if (tmp_extruder >= EXTRUDERS) {
      invalid_extruder_error(tmp_extruder);
      return;
    }

    float old_feedrate_mm_s = feedrate_mm_s;

    feedrate_mm_s = fr_mm_s > 0.0 ? (old_feedrate_mm_s = fr_mm_s) : XY_PROBE_FEEDRATE_MM_S;

    if (tmp_extruder != active_extruder) {
      if (!no_move && axis_unhomed_error(true, true, true)) {
        SERIAL_EM("No move on toolchange");
        no_move = true;
      }

      // Save current position to destination, for use later
      set_destination_to_current();
      
      #if ENABLED(DUAL_X_CARRIAGE)

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) {
            SERIAL_M("Dual X Carriage Mode ");
            switch (dual_x_carriage_mode) {
              case DXC_DUPLICATION_MODE: SERIAL_EM("DXC_DUPLICATION_MODE"); break;
              case DXC_AUTO_PARK_MODE: SERIAL_EM("DXC_AUTO_PARK_MODE"); break;
              case DXC_FULL_CONTROL_MODE: SERIAL_EM("DXC_FULL_CONTROL_MODE"); break;
            }
          }
        #endif

        if (dual_x_carriage_mode == DXC_AUTO_PARK_MODE && IsRunning() &&
            (delayed_move_time || current_position[X_AXIS] != x_home_pos(active_extruder))
        ) {
          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) {
              SERIAL_EMV("Raise to ", current_position[Z_AXIS] + TOOLCHANGE_PARK_ZLIFT);
              SERIAL_EMV("MoveX to ", x_home_pos(active_extruder));
              SERIAL_EMV("Lower to ", current_position[Z_AXIS]);
            }
          #endif

          // Park old head: 1) raise 2) move to park position 3) lower
          for (uint8_t i = 0; i < 3; i++)
            planner.buffer_line(
              i == 0 ? current_position[X_AXIS] : x_home_pos(active_extruder),
              current_position[Y_AXIS],
              current_position[Z_AXIS] + (i == 2 ? 0 : TOOLCHANGE_PARK_ZLIFT),
              current_position[E_AXIS],
              planner.max_feedrate_mm_s[i == 1 ? X_AXIS : Z_AXIS],
              active_extruder,
              active_driver
            );
          stepper.synchronize();
        }

        // apply Y & Z extruder offset (x offset is already used in determining home pos)
        current_position[Y_AXIS] -= hotend_offset[Y_AXIS][active_extruder] - hotend_offset[Y_AXIS][tmp_extruder];
        current_position[Z_AXIS] -= hotend_offset[Z_AXIS][active_extruder] - hotend_offset[Z_AXIS][tmp_extruder];
        active_extruder = active_driver = tmp_extruder;

        // This function resets the max/min values - the current position may be overwritten below.
        set_axis_is_at_home(X_AXIS);

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("New Extruder", current_position);
        #endif

        switch (dual_x_carriage_mode) {
          case DXC_FULL_CONTROL_MODE:
            current_position[X_AXIS] = LOGICAL_X_POSITION(inactive_hotend_x_pos);
            inactive_hotend_x_pos = RAW_X_POSITION(destination[X_AXIS]);
            break;
          case DXC_DUPLICATION_MODE:
            active_hotend_parked = (active_extruder == 0); // this triggers the second extruder to move into the duplication position
            if (active_hotend_parked)
              current_position[X_AXIS] = LOGICAL_X_POSITION(inactive_hotend_x_pos);
            else
              current_position[X_AXIS] = destination[X_AXIS] + duplicate_hotend_x_offset;
            inactive_hotend_x_pos = RAW_X_POSITION(destination[X_AXIS]);
            hotend_duplication_enabled = false;
            break;
          default:
            // record raised toolhead position for use by unpark
            memcpy(raised_parked_position, current_position, sizeof(raised_parked_position));
            raised_parked_position[Z_AXIS] += TOOLCHANGE_UNPARK_ZLIFT;
            active_hotend_parked = true;
            delayed_move_time = 0;
            break;
        }

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) {
            SERIAL_EMV("Active hotend parked: ", active_hotend_parked ? "yes" : "no");
            DEBUG_POS("New hotend (parked)", current_position);
          }
        #endif

        // No extra case for HAS_ABL in DUAL_X_CARRIAGE. Does that mean they don't work together?
      #else // !DUAL_X_CARRIAGE

        #if HAS(DONDOLO)
          // <0 if the new nozzle is higher, >0 if lower. A bigger raise when lower.
          float z_diff = hotend_offset[Z_AXIS][active_extruder] - hotend_offset[Z_AXIS][tmp_extruder],
                z_raise = 0.3 + (z_diff > 0.0 ? z_diff : 0.0);

          set_destination_to_current();

          // Always raise by some amount
          destination[Z_AXIS] += z_raise;
          planner.buffer_line_kinematic(destination, planner.max_feedrate_mm_s[Z_AXIS], active_extruder, active_driver);
          stepper.synchronize();

          move_extruder_servo(tmp_extruder);
          HAL::delayMilliseconds(500);

          // Move back down, if needed
          if (z_raise != z_diff) {
            destination[Z_AXIS] = current_position[Z_AXIS] + z_diff;
            planner.buffer_line_kinematic(destination, planner.max_feedrate_mm_s[Z_AXIS], active_extruder, active_driver);
            stepper.synchronize();
          }
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

          offset_vec.apply_rotation(planner.bed_level_matrix.transpose(planner.bed_level_matrix));

          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) offset_vec.debug("offset_vec (AFTER)");
          #endif

          // Adjustments to the current position
          float xydiff[2] = { offset_vec.x, offset_vec.y };
          current_position[Z_AXIS] += offset_vec.z;

        #else // !ABL_PLANAR

          float xydiff[2] = {
            hotend_offset[X_AXIS][tmp_extruder] - hotend_offset[X_AXIS][active_extruder],
            hotend_offset[Y_AXIS][tmp_extruder] - hotend_offset[Y_AXIS][active_extruder]
          };

          #if ENABLED(MESH_BED_LEVELING)

            if (mbl.active()) {
              #if ENABLED(DEBUG_LEVELING_FEATURE)
                if (DEBUGGING(LEVELING)) SERIAL_MV("Z before MBL: ", current_position[Z_AXIS]);
              #endif
              float xpos = RAW_CURRENT_POSITION(X_AXIS),
                    ypos = RAW_CURRENT_POSITION(Y_AXIS);
              current_position[Z_AXIS] += mbl.get_z(xpos + xydiff[X_AXIS], ypos + xydiff[Y_AXIS]) - mbl.get_z(xpos, ypos);
              #if ENABLED(DEBUG_LEVELING_FEATURE)
                if (DEBUGGING(LEVELING))
                  SERIAL_EMV(" after: ", current_position[Z_AXIS]);
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
        current_position[X_AXIS] += xydiff[X_AXIS];
        current_position[Y_AXIS] += xydiff[Y_AXIS];
        for (uint8_t i = X_AXIS; i <= Y_AXIS; i++) {
          position_shift[i] += xydiff[i];
          update_software_endstops((AxisEnum)i);
        }

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
        if (DEBUGGING(LEVELING)) DEBUG_POS("Sync After Toolchange", current_position);
      #endif

      // Tell the planner the new "current position"
      SYNC_PLAN_POSITION_KINEMATIC();

      // Move to the "old position" (move the extruder into place)
      if (!no_move && IsRunning()) {
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("Move back", destination);
        #endif
        prepare_move_to_destination();
      }

    } // (tmp_extruder != active_extruder)

    stepper.synchronize();

    #if ENABLED(EXT_SOLENOID)
      disable_all_solenoids();
      enable_solenoid_on_active_extruder();
    #endif // EXT_SOLENOID

    feedrate_mm_s = old_feedrate_mm_s;

  #endif // HOTENDS > 1

  SERIAL_LMV(ECHO, MSG_ACTIVE_DRIVER, (int)active_driver);
  SERIAL_LMV(ECHO, MSG_ACTIVE_EXTRUDER, (int)active_extruder);
}
  
/**
 * Process a single command and dispatch it to its handler
 * This is called from the main loop()
 */
void process_next_command() {
  current_command = command_queue[cmd_queue_index_r];

  if (DEBUGGING(ECHO)) {
    SERIAL_LV(ECHO, current_command);
  }

  // Sanitize the current command:
  //  - Skip leading spaces
  //  - Bypass N[-0-9][0-9]*[ ]*
  //  - Overwrite * with nul to mark the end
  while (*current_command == ' ') ++current_command;
  if (*current_command == 'N' && NUMERIC_SIGNED(current_command[1])) {
    current_command += 2; // skip N[-0-9]
    while (NUMERIC(*current_command)) ++current_command; // skip [0-9]*
    while (*current_command == ' ') ++current_command; // skip [ ]*
  }
  char* starpos = strchr(current_command, '*');  // * should always be the last parameter
  if (starpos) while (*starpos == ' ' || *starpos == '*') *starpos-- = '\0'; // nullify '*' and ' '

  char *cmd_ptr = current_command;

  // Get the command code, which must be G, M, or T
  char command_code = *cmd_ptr++;

  // Skip spaces to get the numeric part
  while (*cmd_ptr == ' ') cmd_ptr++;

  uint16_t codenum = 0; // define ahead of goto

  // Bail early if there's no code
  bool code_is_good = NUMERIC(*cmd_ptr);
  if (!code_is_good) goto ExitUnknownCommand;

  // Get and skip the code number
  do {
    codenum = (codenum * 10) + (*cmd_ptr - '0');
    cmd_ptr++;
  } while (NUMERIC(*cmd_ptr));

  // Skip all spaces to get to the first argument, or nul
  while (*cmd_ptr == ' ') cmd_ptr++;

  // The command's arguments (if any) start here, for sure!
  current_command_args = cmd_ptr;

  KEEPALIVE_STATE(IN_HANDLER);

  // Handle a known G, M, or T
  switch(command_code) {
    case 'G': switch (codenum) {

      // G0, G1
      case 0:
      case 1:
        #if IS_SCARA
          gcode_G0_G1(codenum == 0); break;
        #elif ENABLED(LASERBEAM)
          gcode_G0_G1(codenum == 1); break;
        #else
          gcode_G0_G1(); break;
        #endif

      // G2, G3
      #if ENABLED(ARC_SUPPORT) && NOMECH(SCARA)
        case 2: // G2  - CW ARC
        case 3: // G3  - CCW ARC
          gcode_G2_G3(codenum == 2); break;
      #endif

      // G4 Dwell
      case 4:
        gcode_G4(); break;

      #if ENABLED(LASERBEAM)
        #if ENABLED(G5_BEZIER)
          case 5: // G5: Bezier curve - from http://forums.reprap.org/read.php?147,93577
            gcode_G5(); break;
        #endif // G5_BEZIER

        #if ENABLED(LASER_RASTER)
          case 7: // G7: Execute laser raster line
            gcode_G7(); break;
        #endif // LASER_RASTER
      #endif // LASERBEAM

      #if ENABLED(FWRETRACT)
        case 10: // G10: retract
        case 11: // G11: retract_recover
          gcode_G10_G11(codenum == 10); break;
      #endif // FWRETRACT

      #if ENABLED(NOZZLE_CLEAN_FEATURE)
        case 12: // G12: Nozzle Clean
          gcode_G12(); break;
      #endif // NOZZLE_CLEAN_FEATURE

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
        gcode_G28(); break;

      #if PLANNER_LEVELING
        case 29: // G29 Detailed Z probe, probes the bed at 3 or more points.
          gcode_G29(); break;
      #endif // PLANNER_LEVELING

      #if HAS(BED_PROBE) && NOMECH(DELTA)
        case 30: // G30 Single Z Probe
          gcode_G30(); break;
        
        #if ENABLED(Z_PROBE_SLED)
          case 31: // G31: dock the sled
            gcode_G31(); break;
          case 32: // G32: undock the sled
            gcode_G32(); break;
        #endif // Z_PROBE_SLED

      #elif ENABLED(AUTO_CALIBRATION_FEATURE)

        case 30:  // G30 Delta AutoCalibration
          gcode_G30(); break;

      #endif // AUTO_BED_LEVELING_FEATURE & DELTA

      case 60: // G60 Saved Coordinates
        gcode_G60(); break;
      case 61: // G61 Restore Coordinates
        gcode_G61(); break;
      case 90: // G90
        relative_mode = false; break;
      case 91: // G91
        relative_mode = true; break;
      case 92: // G92
        gcode_G92(); break;
    }
    break;

    case 'M': switch (codenum) {
      #if ENABLED(ULTIPANEL)
        case 0: // M0: Unconditional stop - Wait for user button press on LCD
        case 1: // M1: Conditional stop - Wait for user button press on LCD
          gcode_M0_M1(); break;
      #endif // ULTIPANEL

      #if ENABLED(LASERBEAM) && ENABLED(LASER_FIRE_SPINDLE)
        case 3: // M03: Setting laser beam
        case 4: // M04: Turn on laser beam
          gcode_M3_M4(); break;
        case 5: // M05: Turn off laser beam
          gcode_M5(); break;
      #endif // LASERBEAM

      case 17: // M17: Enable/Power all stepper motors
        gcode_M17(); break;

      #if ENABLED(SDSUPPORT)
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

      #if ENABLED(SDSUPPORT)
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

      #if HAS(POWER_CONSUMPTION_SENSOR)
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

      #if HAS(POWER_SWITCH)
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

      case 104: // M104: Set hot end temperature
        gcode_M104(); break;

      case 105: // M105: Report current temperature
        gcode_M105();
        KEEPALIVE_STATE(NOT_BUSY);
        return; // "ok" already printed

      #if HAS(FAN)
        case 106: // M106: Fan On
          gcode_M106(); break;
        case 107: // M107: Fan Off
          gcode_M107(); break;
      #endif // HAS(FAN)

      case 108: // M108: Cancel heatup
        gcode_M108(); break;

      case 109: // M109: Wait for hotend temperature to reach target
        gcode_M109(); break;

      case 110: // M110: Set Current Line Number
        gcode_M110(); break;

      case 111: // M111: Set debug level
        gcode_M111(); break;

      case 112: //  M112: Emergency Stop
        gcode_M112(); break;

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

      case 119: // M119: Report endstop states
        gcode_M119(); break;
      case 120: // M120: Enable endstops
        gcode_M120(); break;
      case 121: // M121: Disable endstops
        gcode_M121(); break;
      case 122: // M122: Disable or enable software endstops
        gcode_M122(); break;

      #if ENABLED(BARICUDA)
        // PWM for HEATER_1_PIN
        #if HAS(HEATER_1)
          case 126: // M126 valve open
            gcode_M126(); break;
          case 127: // M127 valve closed
            gcode_M127(); break;
        #endif // HAS(HEATER_1)

        // PWM for HEATER_2_PIN
        #if HAS(HEATER_2)
          case 128: // M128 valve open
            gcode_M128(); break;
          case 129: // M129 valve closed
            gcode_M129(); break;
        #endif // HAS(HEATER_2)
      #endif // BARICUDA

      #if HAS(TEMP_BED)
        case 140: // M140 - Set bed temp
          gcode_M140(); break;
      #endif

      #if HAS(TEMP_CHAMBER)
        case 141: // M141 - Set chamber temp
          gcode_M141(); break;
      #endif

      #if HAS(TEMP_COOLER)
        case 142: // M142 - Set cooler temp
          gcode_M142(); break;
      #endif

      #if ENABLED(BLINKM)
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

      #if HAS(TEMP_BED)
        case 190: // M190 - Wait for bed heater to reach target.
          gcode_M190(); break;
      #endif // TEMP_BED

      #if HAS(TEMP_CHAMBER)
        case 191: // M191 - Wait for chamber heater to reach target.
          gcode_M191(); break;
      #endif

      #if HAS(TEMP_COOLER)
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
      case 203: // M203 max feedrate_mm_s units/sec
        gcode_M203(); break;
      case 204: // M204 planner.acceleration S normal moves T filament only moves
        gcode_M204(); break;
      case 205: //M205 advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk
        gcode_M205(); break;
      case 206: // M206 additional homing offset
        gcode_M206(); break;

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

      #if HAS(CHDK) || HAS(PHOTOGRAPH)
        case 240: // M240: Triggers a camera by emulating a Canon RC-1 : http://www.doc-diy.net/photo/rc-1_hacked/
          gcode_M240(); break;
      #endif

      #if ENABLED(DOGLCD) && LCD_CONTRAST >= 0
        case 250: // M250: Set LCD contrast value: C<value> (value 0..63)
          gcode_M250(); break;
      #endif

      #if HAS(SERVOS)
        case 280: // M280: Set servo position absolute
          gcode_M280(); break;
      #endif

      #if HAS(BUZZER)
        case 300: // M300: Play beep tone
          gcode_M300(); break;
      #endif

      #if ENABLED(PIDTEMP)
        case 301: // M301: Set hotend PID parameters
          gcode_M301(); break;
      #endif

      #if ENABLED(PREVENT_COLD_EXTRUSION)
        case 302: // M302: Allow cold extrudes (set the minimum extrude temperature)
          gcode_M302(); break;
      #endif

      #if HAS(PID_HEATING)
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

      #if HAS(ABL)
        case 320: // M320: Activate autolevel
          gcode_M320(); break;
        case 321: // M321: Deactivate autoleveling
          gcode_M321(); break;
        case 322: // M322: Reset auto leveling matrix
          gcode_M322(); break;
      #endif

      #if HAS(MICROSTEPS)
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

      #if HAS(BED_PROBE)
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

      case 410: // M410 quickstop - Abort all the planned moves.
        gcode_M410(); break;

      #if ENABLED(MESH_BED_LEVELING)
        case 420: // M420 Enable/Disable Mesh Bed Leveling
          gcode_M420(); break;
        case 421: // M421 Set a Mesh Bed Leveling Z coordinate
          gcode_M421(); break;
      #endif

      case 428: // M428 Apply current_position to home_offset
        gcode_M428(); break;

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
        case 540:
          gcode_M540(); break;
      #endif

      #if HEATER_USES_AD595
        case 595: // M595 set Hotends AD595 offset & gain
          gcode_M595(); break;
      #endif

      #if ENABLED(FILAMENT_CHANGE_FEATURE)
        case 600: // Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
          gcode_M600(); break;
      #endif

      #if ENABLED(DUAL_X_CARRIAGE)
        case 605:
          gcode_M605(); break;
      #endif

      #if ENABLED(LASERBEAM)
        case 649: // M649 set laser options
          gcode_M649(); break;
      #endif 

      #if HAS(ABL) || MECH(DELTA)
        case 666: // M666 Set Z probe offset or set delta endstop and geometry adjustment
          gcode_M666(); break;
      #endif

      #if ENABLED(LIN_ADVANCE)
        case 905: // M905 Set advance factor.
          gcode_M905(); break;
      #endif

      #if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
        case 906: // M906 Set motor currents XYZ T0-4 E
          gcode_M906(); break;
      #endif

      case 907: // M907 Set digital trimpot motor current using axis codes.
        gcode_M907(); break;

      #if HAS(DIGIPOTSS)
        case 908: // M908 Control digital trimpot directly.
          gcode_M908(); break;
      #endif // HAS(DIGIPOTSS)

      #if ENABLED(NPR2)
        case 997: // M997 Cxx Move Carter xx gradi
          gcode_M997(); break;
      #endif // NPR2

      case 999: // M999: Restart after being Stopped
        gcode_M999(); break;
    }
    break;

    case 'T':
      gcode_T(codenum);
    break;

    default: code_is_good = false;
  }

  KEEPALIVE_STATE(NOT_BUSY);

ExitUnknownCommand:

  // Still unknown command? Throw an error
  if (!code_is_good) unknown_command_error();

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
  SERIAL_S(OK);
  #if ENABLED(ADVANCED_OK)
    char* p = command_queue[cmd_queue_index_r];
    if (*p == 'N') {
      SERIAL_C(' ');
      SERIAL_C(*p++);
      while (NUMERIC_SIGNED(*p))
        SERIAL_C(*p++);
    }
    SERIAL_MV(" P", (int)(BLOCK_BUFFER_SIZE - planner.movesplanned() - 1));
    SERIAL_MV(" B", BUFSIZE - commands_in_queue);
  #endif
  SERIAL_E;
}

#if ENABLED(SOFTWARE_MIN_ENDSTOPS) || ENABLED(SOFTWARE_MAX_ENDSTOPS)
  void clamp_to_software_endstops(float target[XYZ]) {
    if (SOFTWARE_MIN_ENDSTOPS && soft_endstops_enabled) {
      NOLESS(target[X_AXIS], soft_endstop_min[X_AXIS]);
      NOLESS(target[Y_AXIS], soft_endstop_min[Y_AXIS]);
      #if !ENABLED(LASERBEAM)
        NOLESS(target[Z_AXIS], soft_endstop_min[Z_AXIS]);
      #endif
    }

    if (SOFTWARE_MAX_ENDSTOPS && soft_endstops_enabled) {
      NOMORE(target[X_AXIS], soft_endstop_max[X_AXIS]);
      NOMORE(target[Y_AXIS], soft_endstop_max[Y_AXIS]);
      #if !ENABLED(LASERBEAM)
        NOMORE(target[Z_AXIS], soft_endstop_max[Z_AXIS]);
      #endif
    }
  }
#endif

#if ENABLED(AUTO_BED_LEVELING_BILINEAR)

  // Get the Z adjustment for non-linear bed leveling
  float bilinear_z_offset(float cartesian[XYZ]) {

    // XY relative to the probed area
    const float x = RAW_X_POSITION(cartesian[X_AXIS]) - bilinear_start[X_AXIS],
                y = RAW_Y_POSITION(cartesian[Y_AXIS]) - bilinear_start[Y_AXIS];

    // Convert to grid box units
    float ratio_x = x / bilinear_grid_spacing[X_AXIS],
          ratio_y = y / bilinear_grid_spacing[Y_AXIS];

    // Whole units for the grid line indices. Constrained within bounds.
    const int gridx = constrain(floor(ratio_x), 0, ABL_GRID_POINTS_X - 1),
              gridy = constrain(floor(ratio_y), 0, ABL_GRID_POINTS_Y - 1),
              nextx = min(gridx + 1, ABL_GRID_POINTS_X - 1),
              nexty = min(gridy + 1, ABL_GRID_POINTS_Y - 1);

    // Subtract whole to get the ratio within the grid box
    ratio_x -= gridx; ratio_y -= gridy;

    // Never less than 0.0. (Over 1.0 is fine due to previous contraints.)
    NOLESS(ratio_x, 0); NOLESS(ratio_y, 0);

    // Z at the box corners
    const float z1 = bilinear_level_grid[gridx][gridy],  // left-front
                z2 = bilinear_level_grid[gridx][nexty],  // left-back
                z3 = bilinear_level_grid[nextx][gridy],  // right-front
                z4 = bilinear_level_grid[nextx][nexty],  // right-back

                // Bilinear interpolate
                L = z1 + (z2 - z1) * ratio_y,   // Linear interp. LF -> LB
                R = z3 + (z4 - z3) * ratio_y,   // Linear interp. RF -> RB
                offset = L + ratio_x * (R - L);

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        static float last_offset = 0;
        if (fabs(last_offset - offset) > 0.2) {
          SERIAL_M("Sudden Shift at ");
          SERIAL_MV("x=", x);
          SERIAL_MV(" / ", bilinear_grid_spacing[X_AXIS]);
          SERIAL_EMV(" -> gridx=", gridx);
          SERIAL_MV(" y=", y);
          SERIAL_MV(" / ", bilinear_grid_spacing[Y_AXIS]);
          SERIAL_EMV(" -> gridy=", gridy);
          SERIAL_MV(" ratio_x=", ratio_x);
          SERIAL_EMV(" ratio_y=", ratio_y);
          SERIAL_MV(" z1=", z1);
          SERIAL_MV(" z2=", z2);
          SERIAL_MV(" z3=", z3);
          SERIAL_EMV(" z4=", z4);
          SERIAL_MV(" L=", L);
          SERIAL_MV(" R=", R);
          SERIAL_EMV(" offset=", offset);
        }
        last_offset = offset;
      }
    #endif

    return offset;
  }

#endif // AUTO_BED_LEVELING_BILINEAR

/**
 * Function for DELTA
 */
#if MECH(DELTA)

  void set_delta_constants() {
    max_length[Z_AXIS]    = soft_endstop_max[Z_AXIS] - Z_MIN_POS;
    base_max_pos[Z_AXIS]  = soft_endstop_max[Z_AXIS];
    base_home_pos[Z_AXIS] = soft_endstop_max[Z_AXIS];

    delta_diagonal_rod_2_tower_1 = sq(delta_diagonal_rod + diagrod_adj[0]);
    delta_diagonal_rod_2_tower_2 = sq(delta_diagonal_rod + diagrod_adj[1]);
    delta_diagonal_rod_2_tower_3 = sq(delta_diagonal_rod + diagrod_adj[2]);

    // Effective X/Y positions of the three vertical towers.
    delta_tower1_x = (delta_radius + tower_adj[3]) * cos((210 + tower_adj[0]) * M_PI/180); // front left tower
    delta_tower1_y = (delta_radius + tower_adj[3]) * sin((210 + tower_adj[0]) * M_PI/180); 
    delta_tower2_x = (delta_radius + tower_adj[4]) * cos((330 + tower_adj[1]) * M_PI/180); // front right tower
    delta_tower2_y = (delta_radius + tower_adj[4]) * sin((330 + tower_adj[1]) * M_PI/180); 
    delta_tower3_x = (delta_radius + tower_adj[5]) * cos((90 + tower_adj[2]) * M_PI/180);  // back middle tower
    delta_tower3_y = (delta_radius + tower_adj[5]) * sin((90 + tower_adj[2]) * M_PI/180); 
  }

  #if ENABLED(DELTA_FAST_SQRT)
    /**
     * Fast inverse sqrt from Quake III Arena
     * See: https://en.wikipedia.org/wiki/Fast_inverse_square_root
     */
    float Q_rsqrt(float number) {
      long i;
      float x2, y;
      const float threehalfs = 1.5f;
      x2 = number * 0.5f;
      y  = number;
      i  = * ( long * ) &y;                       // evil floating point bit level hacking
      i  = 0x5f3759df - ( i >> 1 );               // what the f***?
      y  = * ( float * ) &i;
      y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
      // y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed
      return y;
    }

    #define _SQRT(n) (1.0f / Q_rsqrt(n))

  #else

    #define _SQRT(n) sqrt(n)

  #endif

  /**
   * Delta Inverse Kinematics
   *
   * Calculate the tower positions for a given logical
   * position, storing the result in the delta[] array.
   *
   * This is an expensive calculation, requiring 3 square
   * roots per segmented linear move, and strains the limits
   * of a Mega2560 with a Graphical Display.
   */

  /*
  // Macro to obtain the Z position of an individual tower
  #define DELTA_Z(T) raw[Z_AXIS] + _SQRT(     \
        delta_diagonal_rod_2_tower_##T        \
        -sq(delta_tower##T##_x - raw[X_AXIS]) \
        -sq(delta_tower##T##_y - raw[Y_AXIS]) \
      )
  */

  // Macro to obtain the Z position of an individual tower
  #define DELTA_Z(T) raw[Z_AXIS] + _SQRT(    \
    delta_diagonal_rod_2_tower_##T - HYPOT2( \
        delta_tower##T##_x - raw[X_AXIS],    \
        delta_tower##T##_y - raw[Y_AXIS]     \
      )                                      \
    )

  #define DELTA_RAW_IK() do {   \
    delta[A_AXIS] = DELTA_Z(1); \
    delta[B_AXIS] = DELTA_Z(2); \
    delta[C_AXIS] = DELTA_Z(3); \
  } while(0)

  #define DELTA_LOGICAL_IK() do {      \
    const float raw[XYZ] = {           \
      RAW_X_POSITION(logical[X_AXIS]), \
      RAW_Y_POSITION(logical[Y_AXIS]), \
      RAW_Z_POSITION(logical[Z_AXIS])  \
    };                                 \
    DELTA_RAW_IK();                    \
  } while(0)

  #define DELTA_DEBUG() do { \
      SERIAL_MV("cartesian X:", raw[X_AXIS]); \
      SERIAL_MV(" Y:", raw[Y_AXIS]);          \
      SERIAL_EMV(" Z:", raw[Z_AXIS]);        \
      SERIAL_MV("delta A:", delta[A_AXIS]);   \
      SERIAL_MV(" B:", delta[B_AXIS]);        \
      SERIAL_EMV(" C:", delta[C_AXIS]);      \
    } while(0)

  void inverse_kinematics(const float logical[XYZ]) {
    DELTA_LOGICAL_IK();
    // DELTA_DEBUG();
  }

  float delta_safe_distance_from_top() {
    float cartesian[XYZ] = {
      LOGICAL_X_POSITION(0),
      LOGICAL_Y_POSITION(0),
      LOGICAL_Z_POSITION(0)
    };
    inverse_kinematics(cartesian);
    float distance = delta[A_AXIS];
    cartesian[Y_AXIS] = LOGICAL_Y_POSITION(DELTA_PRINTABLE_RADIUS);
    inverse_kinematics(cartesian);
    return abs(distance - delta[A_AXIS]);
  }

  /**
   * Delta Forward Kinematics
   *
   * See the Wikipedia article "Trilateration"
   * https://en.wikipedia.org/wiki/Trilateration
   *
   * Establish a new coordinate system in the plane of the
   * three carriage points. This system has its origin at
   * tower1, with tower2 on the X axis. Tower3 is in the X-Y
   * plane with a Z component of zero.
   * We will define unit vectors in this coordinate system
   * in our original coordinate system. Then when we calculate
   * the Xnew, Ynew and Znew values, we can translate back into
   * the original system by moving along those unit vectors
   * by the corresponding values.
   *
   * Variable names matched to Marlin, c-version, and avoid the
   * use of any vector library.
   *
   * by Andreas Hardtung 2016-06-07
   * based on a Java function from "Delta Robot Kinematics V3"
   * by Steve Graves
   *
   * The result is stored in the cartes[] array.
   */
  void forward_kinematics_DELTA(float z1, float z2, float z3) {
    // Create a vector in old coordinates along x axis of new coordinate
    float p12[3] = { delta_tower2_x - delta_tower1_x, delta_tower2_y - delta_tower1_y, z2 - z1 };

    // Get the Magnitude of vector.
    float d = sqrt( sq(p12[0]) + sq(p12[1]) + sq(p12[2]) );

    // Create unit vector by dividing by magnitude.
    float ex[3] = { p12[0] / d, p12[1] / d, p12[2] / d };

    // Get the vector from the origin of the new system to the third point.
    float p13[3] = { delta_tower3_x - delta_tower1_x, delta_tower3_y - delta_tower1_y, z3 - z1 };

    // Use dot product to find the component of this vector on the X axis.
    float i = ex[0] * p13[0] + ex[1] * p13[1] + ex[2] * p13[2];

    // Create a vector along the x axis that represents the x component of p13.
    float iex[3] = { ex[0] * i, ex[1] * i, ex[2] * i };

    // Subtract the X component away from the original vector leaving only the Y component. We use the
    // variable that will be the unit vector after we scale it.
    float ey[3] = { p13[0] - iex[0], p13[1] - iex[1], p13[2] - iex[2]};

    // The magnitude of Y component
    float j = sqrt( sq(ey[0]) + sq(ey[1]) + sq(ey[2]) );

    // Convert to a unit vector
    ey[0] /= j; ey[1] /= j;  ey[2] /= j;

    // The cross product of the unit x and y is the unit z
    // float[] ez = vectorCrossProd(ex, ey);
    float ez[3] = {
      ex[1] * ey[2] - ex[2] * ey[1],
      ex[2] * ey[0] - ex[0] * ey[2],
      ex[0] * ey[1] - ex[1] * ey[0]
    };

    // We now have the d, i and j values defined in Wikipedia.
    // Plug them into the equations defined in Wikipedia for Xnew, Ynew and Znew
    float Xnew = (delta_diagonal_rod_2_tower_1 - delta_diagonal_rod_2_tower_2 + sq(d)) / (d * 2),
          Ynew = ((delta_diagonal_rod_2_tower_1 - delta_diagonal_rod_2_tower_3 + HYPOT2(i, j)) / 2 - i * Xnew) / j,
          Znew = sqrt(delta_diagonal_rod_2_tower_1 - HYPOT2(Xnew, Ynew));

    // Start from the origin of the old coordinates and add vectors in the
    // old coords that represent the Xnew, Ynew and Znew to find the point
    // in the old system.
    cartes[X_AXIS] = delta_tower1_x + ex[0] * Xnew + ey[0] * Ynew - ez[0] * Znew;
    cartes[Y_AXIS] = delta_tower1_y + ex[1] * Xnew + ey[1] * Ynew - ez[1] * Znew;
    cartes[Z_AXIS] =             z1 + ex[2] * Xnew + ey[2] * Ynew - ez[2] * Znew;
  }

  void forward_kinematics_DELTA(float point[ABC]) {
    forward_kinematics_DELTA(point[A_AXIS], point[B_AXIS], point[C_AXIS]);
  }

  #if ENABLED(AUTO_CALIBRATION_FEATURE)
    /**
     * Probe bed height at position (x,y), returns the measured z value
     */
    float probe_bed(float x, float y) {
      return probe_pt(x, y, false, 1) + zprobe_zoffset;
    }

    void bed_probe_all() {
      // Initial throwaway probe.. used to stabilize probe
      bed_level_c = probe_bed(0.0, 0.0);

      // Probe all bed positions & store carriage positions
      bed_level_z = probe_bed(0.0, bed_radius);
      bed_level_oy = probe_bed(-SIN_60 * bed_radius, COS_60 * bed_radius);
      bed_level_x = probe_bed(-SIN_60 * bed_radius, -COS_60 * bed_radius);
      bed_level_oz = probe_bed(0.0, -bed_radius);
      bed_level_y = probe_bed(SIN_60 * bed_radius, -COS_60 * bed_radius);
      bed_level_ox = probe_bed(SIN_60 * bed_radius, COS_60 * bed_radius);
      bed_level_c = probe_bed(0.0, 0.0);
    }

    void apply_endstop_adjustment(float x_endstop, float y_endstop, float z_endstop) {
      float saved_endstop_adj[ABC] = { 0 };
      memcpy(saved_endstop_adj, endstop_adj, sizeof(saved_endstop_adj));
      endstop_adj[X_AXIS] += x_endstop;
      endstop_adj[Y_AXIS] += y_endstop;
      endstop_adj[Z_AXIS] += z_endstop;

      inverse_kinematics(current_position);
      planner.set_position_mm(delta[A_AXIS] - (endstop_adj[A_AXIS] - saved_endstop_adj[A_AXIS]) , delta[B_AXIS] - (endstop_adj[B_AXIS] - saved_endstop_adj[B_AXIS]), delta[C_AXIS] - (endstop_adj[C_AXIS] - saved_endstop_adj[C_AXIS]), current_position[E_AXIS]);  
      stepper.synchronize();
    }

    void adj_endstops() {
      boolean x_done = false;
      boolean y_done = false;
      boolean z_done = false;

      do {
        bed_level_z = probe_bed(0.0, bed_radius);
        bed_level_x = probe_bed(-SIN_60 * bed_radius, -COS_60 * bed_radius);
        bed_level_y = probe_bed(SIN_60 * bed_radius, -COS_60 * bed_radius);

        apply_endstop_adjustment(bed_level_x, bed_level_y, bed_level_z);

        SERIAL_MV("x:", bed_level_x, 4);
        SERIAL_MV(" (adj:", endstop_adj[0], 4);
        SERIAL_MV(") y:", bed_level_y, 4);
        SERIAL_MV(" (adj:", endstop_adj[1], 4);
        SERIAL_MV(") z:", bed_level_z, 4);
        SERIAL_MV(" (adj:", endstop_adj[2], 4);
        SERIAL_C(')'); SERIAL_E;

        if ((bed_level_x >= -ac_prec) and (bed_level_x <= ac_prec)) {
          x_done = true;
          SERIAL_M("X=OK ");
        }
        else {
          x_done = false;
          SERIAL_M("X=ERROR ");
        }

        if ((bed_level_y >= -ac_prec) and (bed_level_y <= ac_prec)) {
          y_done = true;
          SERIAL_M("Y=OK ");
        }
        else {
          y_done = false;
          SERIAL_M("Y=ERROR ");
        }

        if ((bed_level_z >= -ac_prec) and (bed_level_z <= ac_prec)) {
          z_done = true;
          SERIAL_EM("Z=OK");
        }
        else {
          z_done = false;
          SERIAL_EM("Z=ERROR");
        }
      } while (((x_done == false) or (y_done == false) or (z_done == false)));

      float high_endstop = MAX3(endstop_adj[A_AXIS], endstop_adj[B_AXIS], endstop_adj[C_AXIS]);

      SERIAL_EMV("High endstop:", high_endstop, 4);

      if (high_endstop > 0) {
        SERIAL_EMV("Reducing Build height by ", high_endstop);
        for (uint8_t i = 0; i < ABC; i++) {
          endstop_adj[i] -= high_endstop;
        }
        soft_endstop_max[Z_AXIS] -= high_endstop;
      }

      /*
      else if (high_endstop < 0) {
        SERIAL_EMV("Increment Build height by ", abs(high_endstop));
        for(uint8_t i = 0; i < 3; i++) {
          endstop_adj[i] -= high_endstop;
        }
        soft_endstop_max[Z_AXIS] -= high_endstop;
      }
      */

      set_delta_constants();
    }

    int fix_tower_errors() {
      boolean t1_err, t2_err, t3_err,
              xy_equal, xz_equal, yz_equal;
      float saved_tower_adj[6];
      uint8_t err_tower = 0;
      float high_diff,
            x_diff, y_diff, z_diff,
            low_opp, high_opp;

      for (uint8_t i = 0; i < 6; i++) saved_tower_adj[i] = tower_adj[i];

      x_diff = abs(bed_level_x - bed_level_ox);
      y_diff = abs(bed_level_y - bed_level_oy);
      z_diff = abs(bed_level_z - bed_level_oz);
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
      if (abs(x_diff - y_diff) <= ac_prec) xy_equal = true;
      if (abs(x_diff - z_diff) <= ac_prec) xz_equal = true;
      if (abs(y_diff - z_diff) <= ac_prec) yz_equal = true;

      SERIAL_M("xy_equal = ");
      if (xy_equal == true) SERIAL_EM("true"); else SERIAL_EM("false");
      SERIAL_M("xz_equal = ");
      if (xz_equal == true) SERIAL_EM("true"); else SERIAL_EM("false");
      SERIAL_M("yz_equal = ");
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

      SERIAL_M("t1:");
      if (t1_err == true) SERIAL_M("Err"); else SERIAL_M("OK");
      SERIAL_M(" t2:");
      if (t2_err == true) SERIAL_M("Err"); else SERIAL_M("OK");
      SERIAL_M(" t3:");
      if (t3_err == true) SERIAL_M("Err"); else SERIAL_M("OK");
      SERIAL_E;

      if (err_tower == 0) {
        SERIAL_EM("Tower geometry OK");
      }
      else {
        SERIAL_MV("Tower", int(err_tower));
        SERIAL_EM(" Error: Adjusting");
        adj_tower_radius(err_tower);
      }

      // Set return value to indicate if anything has been changed (0 = no change)
      int retval = 0;
      for (uint8_t i = 0; i < 6; i++) if (saved_tower_adj[i] != tower_adj[i]) retval++;
      return retval;
    }

    bool adj_deltaradius() {
      boolean adj_done;
      int adj_attempts;
      float adj_dRadius, adjdone_vector;

      bed_level_c = probe_bed(0.0, 0.0);

      if ((bed_level_c >= -ac_prec) and (bed_level_c <= ac_prec)) {
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
          delta_radius += adj_dRadius;
          set_delta_constants();
          adj_done = false;

          adj_endstops();
          bed_level_c = probe_bed(0.0, 0.0);

          // Set inital adjustment value if it is currently 0
          if (adj_dRadius == 0) {
            if (bed_level_c > 0) adj_dRadius = -0.2;
            if (bed_level_c < 0) adj_dRadius = 0.2;
          }

          // Adjustment complete?
          if ((bed_level_c >= -ac_prec) and (bed_level_c <= ac_prec)) {
            //Done to within acprec .. but done within adjdone_vector? 
            if ((bed_level_c >= -adjdone_vector) and (bed_level_c <= adjdone_vector))
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
          SERIAL_MV(" delta radius:", delta_radius, 4);
          SERIAL_MV(" prec:", adjdone_vector, 3);
          SERIAL_MV(" tries:", adj_attempts);
          SERIAL_M(" done:");
          if (adj_done == true) SERIAL_EM("true");
          else SERIAL_EM("false");

          // Overshot target? .. reverse and scale down adjustment
          if (((bed_level_c < 0) and (adj_dRadius < 0)) or ((bed_level_c > 0) and (adj_dRadius > 0))) adj_dRadius = -(adj_dRadius / 2);
  
        } while (adj_done == false);

        return true;
      }
    }

    void adj_tower_radius(uint8_t tower) {
      boolean adj_done;
      float adj_tRadius = 0.0,
            bed_level   = 0.0,
            bed_level_o = 0.0;

      do {
        tower_adj[tower + 2] += adj_tRadius;
        set_delta_constants();
        adj_done = false;

        if (tower == 1) {
          // Bedlevel_x
          bed_level = probe_bed(-SIN_60 * bed_radius, -COS_60 * bed_radius);
          // Bedlevel_ox
          bed_level_o = probe_bed(SIN_60 * bed_radius, COS_60 * bed_radius);
        }
        if (tower == 2) {
          // Bedlevel_y
          bed_level = probe_bed(SIN_60 * bed_radius, -COS_60 * bed_radius);
          // Bedlevel_oy
          bed_level_o = probe_bed(-SIN_60 * bed_radius, COS_60 * bed_radius);
        }
        if (tower == 3) {
          // Bedlevel_z
          bed_level = probe_bed(0.0, bed_radius);
          // Bedlevel_oz
          bed_level_o = probe_bed(0.0, -bed_radius);
        }

        // Set inital adjustment value if it is currently 0
        if (adj_tRadius == 0) {
          if (bed_level_o < bed_level) adj_tRadius = -1;
          if (bed_level_o > bed_level) adj_tRadius = 1;
        }

        // Overshot target? .. reverse and scale down adjustment
        if (((bed_level_o < bed_level) and (adj_tRadius > 0)) or ((bed_level_o > bed_level) and (adj_tRadius < 0))) adj_tRadius = -(adj_tRadius / 2);

        // Adjustment complete?
        if ((bed_level_o > bed_level - 0.015) and (bed_level_o < bed_level + 0.015)) adj_done = true;

        // Show progress
        SERIAL_MV("tower:", bed_level, 4);
        SERIAL_MV(" opptower:", bed_level_o, 4);
        SERIAL_MV(" tower radius adj:", tower_adj[tower + 2], 4);
        SERIAL_M(" done:");
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
        tower_adj[tower - 1] += adj_val;
        set_delta_constants();

        if ((tower == 1) or (tower == 3)) bed_level_oy = probe_bed(-SIN_60 * bed_radius, COS_60 * bed_radius);
        if ((tower == 1) or (tower == 2)) bed_level_oz = probe_bed(0.0, -bed_radius);
        if ((tower == 2) or (tower == 3)) bed_level_ox = probe_bed(SIN_60 * bed_radius, COS_60 * bed_radius);

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
      float prev_diag_rod = delta_diagonal_rod;

      do {
        delta_diagonal_rod += adj_val;
        set_delta_constants();

        bed_level_oy = probe_bed(-SIN_60 * bed_radius, COS_60 * bed_radius);
        bed_level_oz = probe_bed(0.0, -bed_radius);
        bed_level_ox = probe_bed(SIN_60 * bed_radius, COS_60 * bed_radius);
        bed_level_c = probe_bed(0.0, 0.0);

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

      return (delta_diagonal_rod - prev_diag_rod);
    }

    void calibration_report() {
      // Display Report
      SERIAL_EM("| \tZ-Tower\t\t\tEndstop Offsets");

      SERIAL_M("| \t");
      if (bed_level_z >= 0) SERIAL_M(" ");
      SERIAL_MV("", bed_level_z, 4);
      SERIAL_MV("\t\t\tX:", endstop_adj[0], 4);
      SERIAL_MV(" Y:", endstop_adj[1], 4);
      SERIAL_EMV(" Z:", endstop_adj[2], 4);

      SERIAL_M("| ");
      if (bed_level_ox >= 0) SERIAL_M(" ");
      SERIAL_MV("", bed_level_ox, 4);
      SERIAL_M("\t");
      if (bed_level_oy >= 0) SERIAL_M(" ");
      SERIAL_MV("", bed_level_oy, 4);
      SERIAL_EM("\t\tTower Offsets");

      SERIAL_M("| \t");
      if (bed_level_c >= 0) SERIAL_M(" ");
      SERIAL_MV("", bed_level_c, 4);
      SERIAL_MV("\t\t\tA:",tower_adj[0]);
      SERIAL_MV(" B:",tower_adj[1]);
      SERIAL_EMV(" C:",tower_adj[2]);

      SERIAL_M("| ");
      if (bed_level_x >= 0) SERIAL_M(" ");
      SERIAL_MV("", bed_level_x, 4);
      SERIAL_M("\t");
      if (bed_level_y >= 0) SERIAL_M(" ");
      SERIAL_MV("", bed_level_y, 4);
      SERIAL_MV("\t\tI:",tower_adj[3]);
      SERIAL_MV(" J:",tower_adj[4]);
      SERIAL_EMV(" K:",tower_adj[5]);

      SERIAL_M("| \t");
      if (bed_level_oz >= 0) SERIAL_M(" ");
      SERIAL_MV("", bed_level_oz, 4);
      SERIAL_EMV("\t\t\tDelta Radius: ", delta_radius, 4);

      SERIAL_EMV("| X-Tower\tY-Tower\t\tDiagonal Rod: ", delta_diagonal_rod, 4);
      SERIAL_E;
    }

  #endif // AUTO_CALIBRATION_FEATURE

#endif // DELTA

/**
 * Get the stepper positions in the cartes[] array.
 * Forward kinematics are applied for DELTA and SCARA.
 *
 * The result is in the current coordinate space with
 * leveling applied. The coordinates need to be run through
 * unapply_leveling to obtain the "ideal" coordinates
 * suitable for current_position, etc.
 */
void get_cartesian_from_steppers() {
  #if MECH(DELTA)
    forward_kinematics_DELTA(
      stepper.get_axis_position_mm(A_AXIS),
      stepper.get_axis_position_mm(B_AXIS),
      stepper.get_axis_position_mm(C_AXIS)
    );
    cartes[X_AXIS] += LOGICAL_X_POSITION(0);
    cartes[Y_AXIS] += LOGICAL_Y_POSITION(0);
    cartes[Z_AXIS] += LOGICAL_Z_POSITION(0);
  #elif IS_SCARA
    forward_kinematics_SCARA(
      stepper.get_axis_position_degrees(A_AXIS),
      stepper.get_axis_position_degrees(B_AXIS)
    );
    cartes[X_AXIS] += LOGICAL_X_POSITION(0);
    cartes[Y_AXIS] += LOGICAL_Y_POSITION(0);
    cartes[Z_AXIS] = stepper.get_axis_position_mm(Z_AXIS);
  #else
    cartes[X_AXIS] = stepper.get_axis_position_mm(X_AXIS);
    cartes[Y_AXIS] = stepper.get_axis_position_mm(Y_AXIS);
    cartes[Z_AXIS] = stepper.get_axis_position_mm(Z_AXIS);
  #endif
}

/**
 * Set the current_position for an axis based on
 * the stepper positions, removing any leveling that
 * may have been applied.
 */
void set_current_from_steppers_for_axis(const AxisEnum axis) {
  get_cartesian_from_steppers();
  #if PLANNER_LEVELING
    planner.unapply_leveling(cartes);
  #endif
  if (axis == ALL_AXES)
    memcpy(current_position, cartes, sizeof(cartes));
  else
    current_position[axis] = cartes[axis];
}

#if ENABLED(MESH_BED_LEVELING)

  /**
   * Prepare a mesh-leveled linear move in a Cartesian setup,
   * splitting the move where it crosses mesh borders.
   */
  void mesh_line_to_destination(float fr_mm_s, uint8_t x_splits = 0xff, uint8_t y_splits = 0xff) {
    int cx1 = mbl.cell_index_x(RAW_CURRENT_POSITION(X_AXIS)),
        cy1 = mbl.cell_index_y(RAW_CURRENT_POSITION(Y_AXIS)),
        cx2 = mbl.cell_index_x(RAW_X_POSITION(destination[X_AXIS])),
        cy2 = mbl.cell_index_y(RAW_Y_POSITION(destination[Y_AXIS]));
    NOMORE(cx1, MESH_NUM_X_POINTS - 2);
    NOMORE(cy1, MESH_NUM_Y_POINTS - 2);
    NOMORE(cx2, MESH_NUM_X_POINTS - 2);
    NOMORE(cy2, MESH_NUM_Y_POINTS - 2);

    if (cx1 == cx2 && cy1 == cy2) {
      // Start and end on same mesh square
      line_to_destination(fr_mm_s);
      set_current_to_destination();
      return;
    }

    #define MBL_SEGMENT_END(A) (current_position[A ##_AXIS] + (destination[A ##_AXIS] - current_position[A ##_AXIS]) * normalized_dist)

    float normalized_dist, end[XYZE];

    // Split at the left/front border of the right/top square
    int8_t gcx = max(cx1, cx2), gcy = max(cy1, cy2);
    if (cx2 != cx1 && TEST(x_splits, gcx)) {
      memcpy(end, destination, sizeof(end));
      destination[X_AXIS] = LOGICAL_X_POSITION(mbl.get_probe_x(gcx));
      normalized_dist = (destination[X_AXIS] - current_position[X_AXIS]) / (end[X_AXIS] - current_position[X_AXIS]);
      destination[Y_AXIS] = MBL_SEGMENT_END(Y);
      CBI(x_splits, gcx);
    }
    else if (cy2 != cy1 && TEST(y_splits, gcy)) {
      memcpy(end, destination, sizeof(end));
      destination[Y_AXIS] = LOGICAL_Y_POSITION(mbl.get_probe_y(gcy));
      normalized_dist = (destination[Y_AXIS] - current_position[Y_AXIS]) / (end[Y_AXIS] - current_position[Y_AXIS]);
      destination[X_AXIS] = MBL_SEGMENT_END(X);
      CBI(y_splits, gcy);
    }
    else {
      // Already split on a border
      line_to_destination(fr_mm_s);
      set_current_to_destination();
      return;
    }

    destination[Z_AXIS] = MBL_SEGMENT_END(Z);
    destination[E_AXIS] = MBL_SEGMENT_END(E);

    // Do the split and look for more borders
    mesh_line_to_destination(fr_mm_s, x_splits, y_splits);

    // Restore destination from stack
    memcpy(destination, end, sizeof(end));
    mesh_line_to_destination(fr_mm_s, x_splits, y_splits);  
  }

#elif ENABLED(AUTO_BED_LEVELING_BILINEAR) && !IS_KINEMATIC

  #define CELL_INDEX(A,V) ((RAW_##A##_POSITION(V) - bilinear_start[A##_AXIS]) / bilinear_grid_spacing[A##_AXIS])

  /**
   * Prepare a bilinear-leveled linear move on Cartesian,
   * splitting the move where it crosses mesh borders.
   */
  void bilinear_line_to_destination(float fr_mm_s, uint8_t x_splits = 0xff, uint8_t y_splits = 0xff) {
    int cx1 = CELL_INDEX(X, current_position[X_AXIS]),
        cy1 = CELL_INDEX(Y, current_position[Y_AXIS]),
        cx2 = CELL_INDEX(X, destination[X_AXIS]),
        cy2 = CELL_INDEX(Y, destination[Y_AXIS]);
    cx1 = constrain(cx1, 0, ABL_GRID_POINTS_X - 2);
    cy1 = constrain(cy1, 0, ABL_GRID_POINTS_Y - 2);
    cx2 = constrain(cx2, 0, ABL_GRID_POINTS_X - 2);
    cy2 = constrain(cy2, 0, ABL_GRID_POINTS_Y - 2);

    if (cx1 == cx2 && cy1 == cy2) {
      // Start and end on same mesh square
      line_to_destination(fr_mm_s);
      set_current_to_destination();
      return;
    }

    #define LINE_SEGMENT_END(A) (current_position[A ##_AXIS] + (destination[A ##_AXIS] - current_position[A ##_AXIS]) * normalized_dist)

    float normalized_dist, end[XYZE];

    // Split at the left/front border of the right/top square
    int8_t gcx = max(cx1, cx2), gcy = max(cy1, cy2);
    if (cx2 != cx1 && TEST(x_splits, gcx)) {
      memcpy(end, destination, sizeof(end));
      destination[X_AXIS] = LOGICAL_X_POSITION(bilinear_start[X_AXIS] + bilinear_grid_spacing[X_AXIS] * gcx);
      normalized_dist = (destination[X_AXIS] - current_position[X_AXIS]) / (end[X_AXIS] - current_position[X_AXIS]);
      destination[Y_AXIS] = LINE_SEGMENT_END(Y);
      CBI(x_splits, gcx);
    }
    else if (cy2 != cy1 && TEST(y_splits, gcy)) {
      memcpy(end, destination, sizeof(end));
      destination[Y_AXIS] = LOGICAL_Y_POSITION(bilinear_start[Y_AXIS] + bilinear_grid_spacing[Y_AXIS] * gcy);
      normalized_dist = (destination[Y_AXIS] - current_position[Y_AXIS]) / (end[Y_AXIS] - current_position[Y_AXIS]);
      destination[X_AXIS] = LINE_SEGMENT_END(X);
      CBI(y_splits, gcy);
    }
    else {
      // Already split on a border
      line_to_destination(fr_mm_s);
      set_current_to_destination();
      return;
    }

    destination[Z_AXIS] = LINE_SEGMENT_END(Z);
    destination[E_AXIS] = LINE_SEGMENT_END(E);

    // Do the split and look for more borders
    bilinear_line_to_destination(fr_mm_s, x_splits, y_splits);

    // Restore destination from stack
    memcpy(destination, end, sizeof(end));
    bilinear_line_to_destination(fr_mm_s, x_splits, y_splits);
  }

#endif // AUTO_BED_LEVELING_BILINEAR

#if IS_KINEMATIC

  /**
   * Prepare a linear move in a DELTA or SCARA setup.
   *
   * This calls planner.buffer_line several times, adding
   * small incremental moves for DELTA or SCARA.
   */
  inline bool prepare_kinematic_move_to(float ltarget[NUM_AXIS]) {

    // Get the top feedrate of the move in the XY plane
    float _feedrate_mm_s = MMS_SCALED(feedrate_mm_s);

    // If the move is only in Z don't split up the move.
    // This shortcut cannot be used if planar bed leveling
    // is in use, but is fine with mesh-based bed leveling
    if (ltarget[X_AXIS] == current_position[X_AXIS] && ltarget[Y_AXIS] == current_position[Y_AXIS]) {
      planner.buffer_line_kinematic(ltarget, _feedrate_mm_s, active_extruder, active_driver);
      return true;
    }

    // Get the cartesian distances moved in XYZE
    float difference[NUM_AXIS];
    LOOP_XYZE(i) difference[i] = ltarget[i] - current_position[i];

    // Get the linear distance in XYZ
    float cartesian_mm = sqrt(sq(difference[X_AXIS]) + sq(difference[Y_AXIS]) + sq(difference[Z_AXIS]));

    // If the move is very short, check the E move distance
    if (UNEAR_ZERO(cartesian_mm)) cartesian_mm = abs(difference[E_AXIS]);

    // No E move either? Game over.
    if (UNEAR_ZERO(cartesian_mm)) return false;

    // Minimum number of seconds to move the given distance
    float seconds = cartesian_mm / _feedrate_mm_s;

    // The number of segments-per-second times the duration
    // gives the number of segments we should produce
    uint16_t segments = delta_segments_per_second * seconds;

    #if IS_SCARA
      NOMORE(segments, cartesian_mm * 2);
    #endif

    NOLESS(segments, 1);

    // The approximate length of each segment
    float segment_distance[XYZE] = {
      difference[X_AXIS] / segments,
      difference[Y_AXIS] / segments,
      difference[Z_AXIS] / segments,
      difference[E_AXIS] / segments
    };

    //SERIAL_MV("mm=", cartesian_mm);
    //SERIAL_MV(" seconds=", seconds);
    //SERIAL_EMV(" segments=", segments);

    // Drop one segment so the last move is to the exact target.
    // If there's only 1 segment, loops will be skipped entirely.
    --segments;

    // Get the logical current position as starting point
    float logical[XYZE];
    memcpy(logical, current_position, sizeof(logical));

    // Delta can inline its kinematics
    #if MECH(DELTA)
      #define DELTA_IK() DELTA_LOGICAL_IK()
    #else
      #define DELTA_IK() inverse_kinematics(logical)
    #endif

    // For non-interpolated delta calculate every segment
    for (uint16_t s = segments + 1; --s;) {
      LOOP_XYZE(i) logical[i] += segment_distance[i];
      planner.buffer_line_kinematic(logical, _feedrate_mm_s, active_extruder, active_driver);
    }

    // Since segment_distance is only approximate,
    // the final move must be to the exact destination.
    planner.buffer_line_kinematic(ltarget, _feedrate_mm_s, active_extruder, active_driver);
    return true;
  }

#else // !IS_KINEMATIC

  /**
   * Prepare a linear move in a Cartesian setup.
   * If Mesh Bed Leveling is enabled, perform a mesh move.
   */
  inline bool prepare_move_to_destination_cartesian() {
    #if ENABLED(LASERBEAM) && ENABLED(LASER_FIRE_E)
      if (current_position[E_AXIS] != destination[E_AXIS] && ((current_position[X_AXIS] != destination [X_AXIS]) || (current_position[Y_AXIS] != destination [Y_AXIS]))){
        laser.status = LASER_ON;
        laser.fired = LASER_FIRE_E;
      }
      if (current_position[E_AXIS] == destination[E_AXIS] && laser.fired == LASER_FIRE_E)
        laser.status = LASER_OFF;
    #endif

    // Do not use feedrate_percentage for E or Z only moves
    if (current_position[X_AXIS] == destination[X_AXIS] && current_position[Y_AXIS] == destination[Y_AXIS]) {
      line_to_destination();
    }
    else {
      #if ENABLED(MESH_BED_LEVELING)
        if (mbl.active()) {
          mesh_line_to_destination(MMS_SCALED(feedrate_mm_s));
          return false;
        }
        else
      #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
        if (planner.abl_enabled) {
          bilinear_line_to_destination(MMS_SCALED(feedrate_mm_s));
          return false;
        }
        else
      #endif
          line_to_destination(MMS_SCALED(feedrate_mm_s));
    }
    return true;
  }

#endif // !IS_KINEMATIC

#if ENABLED(DUAL_X_CARRIAGE)

  inline bool prepare_move_to_destination_dualx() {
    if (active_hotend_parked) {
      if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && active_extruder == 0) {
        // move duplicate extruder into correct duplication position.
        planner.set_position_mm(
          LOGICAL_X_POSITION(inactive_hotend_x_pos),
          current_position[Y_AXIS],
          current_position[Z_AXIS],
          current_position[E_AXIS]
        );
        planner.buffer_line(current_position[X_AXIS] + duplicate_hotend_x_offset,
                            current_position[Y_AXIS],
                            current_position[Z_AXIS],
                            current_position[E_AXIS],
                            planner.max_feedrate_mm_s[X_AXIS], 1, 1);
        SYNC_PLAN_POSITION_KINEMATIC();
        stepper.synchronize();
        hotend_duplication_enabled = true;
        active_hotend_parked = false;
      }
      else if (dual_x_carriage_mode == DXC_AUTO_PARK_MODE) { // handle unparking of head
        if (current_position[E_AXIS] == destination[E_AXIS]) {
          // This is a travel move (with no extrusion)
          // Skip it, but keep track of the current position
          // (so it can be used as the start of the next non-travel move)
          if (delayed_move_time != 0xFFFFFFFFUL) {
            set_current_to_destination();
            NOLESS(raised_parked_position[Z_AXIS], destination[Z_AXIS]);
            delayed_move_time = millis();
            return false;
          }
        }
        delayed_move_time = 0;
        // unpark extruder: 1) raise, 2) move into starting XY position, 3) lower
        planner.buffer_line(raised_parked_position[X_AXIS], raised_parked_position[Y_AXIS], raised_parked_position[Z_AXIS], current_position[E_AXIS], planner.max_feedrate_mm_s[Z_AXIS], active_extruder, active_driver);
        planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], raised_parked_position[Z_AXIS], current_position[E_AXIS], min(planner.max_feedrate_mm_s[X_AXIS], planner.max_feedrate_mm_s[Y_AXIS]), active_extruder, active_driver);
        planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], planner.max_feedrate_mm_s[Z_AXIS], active_extruder, active_driver);
        active_hotend_parked = false;
      }
    }
    return true;
  }

#endif // DUAL_X_CARRIAGE

/**
 * Prepare a single move and get ready for the next one
 *
 * (This may call planner.buffer_line several times to put
 *  smaller moves into the planner for DELTA or SCARA.)
 */
void prepare_move_to_destination() {
  clamp_to_software_endstops(destination);
  refresh_cmd_timeout();

  #if ENABLED(PREVENT_COLD_EXTRUSION)
    if (!DEBUGGING(DRYRUN)) {
      if (destination[E_AXIS] != current_position[E_AXIS]) {
        if (thermalManager.tooColdToExtrude(active_extruder)) {
          current_position[E_AXIS] = destination[E_AXIS]; // Behave as if the move really took place, but ignore E part
          SERIAL_LM(ER, MSG_ERR_COLD_EXTRUDE_STOP);
        }
        #if ENABLED(PREVENT_LENGTHY_EXTRUDE)
          if (labs(destination[E_AXIS] - current_position[E_AXIS]) > EXTRUDE_MAXLENGTH) {
            current_position[E_AXIS] = destination[E_AXIS]; // Behave as if the move really took place, but ignore E part
            SERIAL_LM(ER, MSG_ERR_LONG_EXTRUDE_STOP);
          }
        #endif
      }
    }
  #endif

  #if IS_KINEMATIC
    if (!prepare_kinematic_move_to(destination)) return;
  #else
    #if ENABLED(DUAL_X_CARRIAGE)
      if (!prepare_move_to_destination_dualx()) return;
    #endif
    if (!prepare_move_to_destination_cartesian()) return;
  #endif

  set_current_to_destination();
}

/**
 * Output the current position to serial
 */
static void report_current_position() {
  SERIAL_MV( "X:", current_position[X_AXIS]);
  SERIAL_MV(" Y:", current_position[Y_AXIS]);
  SERIAL_MV(" Z:", current_position[Z_AXIS]);
  SERIAL_MV(" E:", current_position[E_AXIS]);

  stepper.report_positions();

  #if IS_SCARA
    SERIAL_MV("SCARA Theta:", stepper.get_axis_position_degrees(A_AXIS));
    SERIAL_EMV("   Psi+Theta:", stepper.get_axis_position_degrees(B_AXIS));
  #endif
}

#if ENABLED(ARC_SUPPORT)
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
    float logical[NUM_AXIS], // Destination position
    float *offset,          // Center of rotation relative to current_position
    uint8_t clockwise       // Clockwise?
  ) {

    float radius = HYPOT(offset[X_AXIS], offset[Y_AXIS]),
          center_X = current_position[X_AXIS] + offset[X_AXIS],
          center_Y = current_position[Y_AXIS] + offset[Y_AXIS],
          linear_travel = logical[Z_AXIS] - current_position[Z_AXIS],
          extruder_travel = logical[E_AXIS] - current_position[E_AXIS],
          r_X = -offset[X_AXIS],  // Radius vector from center to current location
          r_Y = -offset[Y_AXIS],
          rt_X = logical[X_AXIS] - center_X,
          rt_Y = logical[Y_AXIS] - center_Y;

    // CCW angle of rotation between position and logical from the circle center. Only one atan2() trig computation required.
    float angular_travel = atan2(r_X * rt_Y - r_Y * rt_X, r_X * rt_X + r_Y * rt_Y);
    if (angular_travel < 0) angular_travel += RADIANS(360);
    if (clockwise) angular_travel -= RADIANS(360);

    // Make a circle if the angular rotation is 0
    if (angular_travel == 0 && current_position[X_AXIS] == logical[X_AXIS] && current_position[Y_AXIS] == logical[Y_AXIS])
      angular_travel += RADIANS(360);

    float mm_of_travel = HYPOT(angular_travel * radius, fabs(linear_travel));
    if (mm_of_travel < 0.001) return;

    uint16_t segments = floor(mm_of_travel / (MM_PER_ARC_SEGMENT));
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
    float arc_target[XYZE],
          theta_per_segment = angular_travel / segments,
          linear_per_segment = linear_travel / segments,
          extruder_per_segment = extruder_travel / segments,
          sin_T = theta_per_segment,
          cos_T = 1 - 0.5 * sq(theta_per_segment); // Small angle approximation

    // Initialize the linear axis
    arc_target[Z_AXIS] = current_position[Z_AXIS];

    // Initialize the extruder axis
    arc_target[E_AXIS] = current_position[E_AXIS];

    float fr_mm_s = MMS_SCALED(feedrate_mm_s);

    millis_t next_idle_ms = millis() + 200UL;

    int8_t count = 0;
    for (uint16_t i = 1; i < segments; i++) { // Iterate (segments-1) times

      thermalManager.manage_temp_controller();
      if (ELAPSED(millis(), next_idle_ms)) {
        next_idle_ms = millis() + 200UL;
        idle();
      }

      if (++count < N_ARC_CORRECTION) {
        // Apply vector rotation matrix to previous r_X / 1
        float r_new_Y = r_X * sin_T + r_Y * cos_T;
        r_X = r_X * cos_T - r_Y * sin_T;
        r_Y = r_new_Y;
      }
      else {
        // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
        // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
        // To reduce stuttering, the sin and cos could be computed at different times.
        // For now, compute both at the same time.
        float cos_Ti = cos(i * theta_per_segment),
              sin_Ti = sin(i * theta_per_segment);
        r_X = -offset[X_AXIS] * cos_Ti + offset[Y_AXIS] * sin_Ti;
        r_Y = -offset[X_AXIS] * sin_Ti - offset[Y_AXIS] * cos_Ti;
        count = 0;
      }

      // Update arc_target location
      arc_target[X_AXIS] = center_X + r_X;
      arc_target[Y_AXIS] = center_Y + r_Y;
      arc_target[Z_AXIS] += linear_per_segment;
      arc_target[E_AXIS] += extruder_per_segment;

      clamp_to_software_endstops(arc_target);

      planner.buffer_line_kinematic(arc_target, fr_mm_s, active_extruder, active_driver);
    }

    // Ensure last segment arrives at target location.
    planner.buffer_line_kinematic(logical, fr_mm_s, active_extruder, active_driver);

    // As far as the parser is concerned, the position is now == target. In reality the
    // motion control system might still be processing the action and the real tool position
    // in any intermediate location.
    set_current_to_destination();
  }
#endif

#if HAS(CONTROLLERFAN)

  void controllerFan() {
    static millis_t lastMotor = 0;      // Last time a motor was turned on
    static millis_t lastMotorCheck = 0; // Last time the state was checked
    millis_t ms = millis();
    if (ms >= lastMotorCheck + 2500) { // Not a time critical function, so we only check every 2500ms
      lastMotorCheck = ms;
      if (X_ENABLE_READ == X_ENABLE_ON || Y_ENABLE_READ == Y_ENABLE_ON || Z_ENABLE_READ == Z_ENABLE_ON || soft_pwm_bed > 0
        || E0_ENABLE_READ == E_ENABLE_ON // If any of the drivers are enabled...
        #if EXTRUDERS > 1
          || E1_ENABLE_READ == E_ENABLE_ON
          #if HAS(X2_ENABLE)
            || X2_ENABLE_READ == X_ENABLE_ON
          #endif
          #if EXTRUDERS > 2
            || E2_ENABLE_READ == E_ENABLE_ON
            #if EXTRUDERS > 3
              || E3_ENABLE_READ == E_ENABLE_ON
            #endif
          #endif
        #endif
      ) {
        lastMotor = ms; //... set time to NOW so the fan will turn on
      }
      
  #if ENABLED(INVERTED_HEATER_PINS)
      uint8_t speed = (lastMotor == 0 || ms >= lastMotor + (CONTROLLERFAN_SECS * 1000UL)) ? 255 - CONTROLLERFAN_MIN_SPEED : (255 - CONTROLLERFAN_SPEED);
  #else
      uint8_t speed = (lastMotor == 0 || ms >= lastMotor + (CONTROLLERFAN_SECS * 1000UL)) ? CONTROLLERFAN_MIN_SPEED : CONTROLLERFAN_SPEED;
  #endif

      // allows digital or PWM fan output to be used (see M42 handling)
      #if ENABLED(FAN_SOFT_PWM)
        fanSpeedSoftPwm_controller = speed;
      #else
        digitalWrite(CONTROLLERFAN_PIN, speed);
        analogWrite(CONTROLLERFAN_PIN, speed);
      #endif
    }
  }

#endif // HAS(CONTROLLERFAN)

#if MECH(MORGAN_SCARA)

  /**
   * Morgan SCARA Forward Kinematics. Results in cartes[].
   * Maths and first version by QHARLEY.
   * Integrated and slightly restructured by Joachim Cerny.
   */
  void forward_kinematics_SCARA(const float &a, const float &b) {

    float a_sin = sin(RADIANS(a)) * L1,
          a_cos = cos(RADIANS(a)) * L1,
          b_sin = sin(RADIANS(b)) * L2,
          b_cos = cos(RADIANS(b)) * L2;

    cartes[X_AXIS] = a_cos + b_cos + SCARA_OFFSET_X;  //theta
    cartes[Y_AXIS] = a_sin + b_sin + SCARA_OFFSET_Y;  //theta+phi

      //SERIAL_MV(" cartes[X_AXIS]=", cartes[X_AXIS]);
      //SERIAL_EMV(" cartes[Y_AXIS]=", cartes[Y_AXIS]);
  }

  /**
   * Morgan SCARA Inverse Kinematics. Results in delta[].
   *
   * See http://forums.reprap.org/read.php?185,283327
   * 
   * Maths and first version by QHARLEY.
   * Integrated and slightly restructured by Joachim Cerny.
   */
  void inverse_kinematics(const float logical[XYZ]) {

    static float C2, S2, SK1, SK2, THETA, PSI; 

    float sx = RAW_X_POSITION(logical[X_AXIS]) - SCARA_offset_x,  // Translate SCARA to standard X Y
          sy = RAW_Y_POSITION(logical[Y_AXIS]) - SCARA_offset_y;  // With scaling factor.

    if (L1 == L2)
      C2 = HYPOT2(sx, sy) / L1_2_2 - 1;
    else
      C2 = (HYPOT2(sx, sy) - (L1_2 + L2_2)) / (2.0 * L1 * L2);

    S2 = _SQRT(sq(C2) - 1);

    // Unrotated Arm1 plus rotated Arm2 gives the distance from Center to End
    SK1 = L1 + L2 * C2;

    // Rotated Arm2 gives the distance from Arm1 to Arm2
    SK2 = L2 * S2;

    // Angle of Arm1 is the difference between Center-to-End angle and the Center-to-Elbow
    THETA = atan2(SK1, SK2) - atan2(sx, sy);

    // Angle of Arm2
    PSI = atan2(S2, C2);

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
        #if HAS(TEMP_BED)
          max_temp = MAX3(max_temp, thermalManager.degTargetBed(), thermalManager.degBed());
        #endif
      for (int8_t h = 0; h < HOTENDS; ++h)
        max_temp = MAX3(max_temp, thermalManager.degHotend(h), thermalManager.degTargetHotend(h));
      bool new_led = (max_temp > 55.0) ? true : (max_temp < 54.0) ? false : red_led;
      if (new_led != red_led) {
        red_led = new_led;
        WRITE(STAT_LED_RED, new_led ? HIGH : LOW);
        WRITE(STAT_LED_BLUE, new_led ? LOW : HIGH);
      }
    }
  }

#endif

#if HAS(FIL_RUNOUT)
  void handle_filament_runout() {
    if (!filament_ran_out) {
      filament_ran_out = true;
      enqueue_and_echo_commands_P(PSTR(FILAMENT_RUNOUT_SCRIPT));
      stepper.synchronize();
    }
  }
#endif

#if ENABLED(FAST_PWM_FAN) || ENABLED(FAST_PWM_COOLER)

  void setPwmFrequency(uint8_t pin, uint8_t val) {
    val &= 0x07;
    switch(digitalPinToTimer(pin)) {

      #if defined(TCCR0A)
        case TIMER0A:
        case TIMER0B:
             // TCCR0B &= ~(_BV(CS00) | _BV(CS01) | _BV(CS02));
             // TCCR0B |= val;
             break;
      #endif

      #if defined(TCCR1A)
        case TIMER1A:
        case TIMER1B:
             // TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
             // TCCR1B |= val;
             break;
      #endif

      #if defined(TCCR2)
        case TIMER2:
        case TIMER2:
             TCCR2 &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
             TCCR2 |= val;
             break;
      #endif

      #if defined(TCCR2A)
        case TIMER2A:
        case TIMER2B:
             TCCR2B &= ~(_BV(CS20) | _BV(CS21) | _BV(CS22));
             TCCR2B |= val;
             break;
      #endif

      #if defined(TCCR3A)
        case TIMER3A:
        case TIMER3B:
        case TIMER3C:
             TCCR3B &= ~(_BV(CS30) | _BV(CS31) | _BV(CS32));
             TCCR3B |= val;
             break;
      #endif

      #if defined(TCCR4A)
        case TIMER4A:
        case TIMER4B:
        case TIMER4C:
             TCCR4B &= ~(_BV(CS40) | _BV(CS41) | _BV(CS42));
             TCCR4B |= val;
             break;
      #endif

      #if defined(TCCR5A)
        case TIMER5A:
        case TIMER5B:
        case TIMER5C:
             TCCR5B &= ~(_BV(CS50) | _BV(CS51) | _BV(CS52));
             TCCR5B |= val;
             break;
      #endif
    }
  }
#endif // FAST_PWM_FAN

void quickstop_stepper() {
  stepper.quick_stop();
  stepper.synchronize();
  set_current_from_steppers_for_axis(ALL_AXES);
  SYNC_PLAN_POSITION_KINEMATIC();
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

  #if HAS(FIL_RUNOUT)
    if ((IS_SD_PRINTING || print_job_counter.isRunning()) && READ(FIL_RUNOUT_PIN) == FIL_RUNOUT_PIN_INVERTING)
      handle_filament_runout();
  #endif

  if (commands_in_queue < BUFSIZE) get_available_commands();

  millis_t ms = millis();

  if (max_inactive_time && ELAPSED(ms, previous_cmd_ms + max_inactive_time)) kill(PSTR(MSG_KILLED));

  #if ENABLED(FLOWMETER_SENSOR) && ENABLED(MINFLOW_PROTECTION)
    if (flow_firstread && print_job_counter.isRunning() && (get_flowrate() < (float)MINFLOW_PROTECTION)) {
      flow_firstread = false;
      kill(PSTR(MSG_KILLED));
    }
  #endif

  if (stepper_inactive_time && ELAPSED(ms, previous_cmd_ms + stepper_inactive_time)
      && !ignore_stepper_queue && !planner.blocks_queued()) {
    #if DISABLE_X == true
      disable_x();
    #endif
    #if DISABLE_Y == true
      disable_y();
    #endif
    #if DISABLE_Z == true
      disable_z();
    #endif
    #if DISABLE_E == true
      disable_e();
    #endif
    #if ENABLED(LASERBEAM)
      if (laser.time / 60000 > 0) {
        laser.lifetime += laser.time / 60000; // convert to minutes
        laser.time = 0;
        eeprom.StoreSettings();
      }
      laser_init();
      #if ENABLED(LASER_PERIPHERALS)
        laser_peripherals_off();
      #endif
    #endif
  }

  #if HAS(CHDK) // Check if pin should be set to LOW after M240 set it to HIGH
    if (chdkActive && PENDING(ms, chdkHigh + CHDK_DELAY)) {
      chdkActive = false;
      WRITE(CHDK_PIN, LOW);
    }
  #endif

  #if HAS(KILL)
    
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
    if (killCount >= KILL_DELAY) kill(PSTR(MSG_KILLED));
  #endif

  #if HAS(HOME)
    // Check to see if we have to home, use poor man's debouncer
    // ---------------------------------------------------------
    static int homeDebounceCount = 0;   // poor man's debouncing count
    const int HOME_DEBOUNCE_DELAY = 750;
    if (!READ(HOME_PIN)) {
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
    
  #if HAS(CONTROLLERFAN)
    controllerFan(); // Check if fan should be turned on to cool stepper drivers down
  #endif

  #if ENABLED(EXTRUDER_RUNOUT_PREVENT)
    if (ELAPSED(ms, previous_cmd_ms + (EXTRUDER_RUNOUT_SECONDS) * 1000UL)
      && thermalManager.degHotend(active_extruder) > EXTRUDER_RUNOUT_MINTEMP) {
      bool oldstatus;
      #if ENABLED(DONDOLO_SINGLE_MOTOR)
        oldstatus = E0_ENABLE_READ;
        enable_e0();
      #else // !DONDOLO_SINGLE_MOTOR
        switch(active_extruder) {
          case 0:
            oldstatus = E0_ENABLE_READ;
            enable_e0();
            break;
          #if DRIVER_EXTRUDERS > 1
            case 1:
              oldstatus = E1_ENABLE_READ;
              enable_e1();
              break;
            #if DRIVER_EXTRUDERS > 2
              case 2:
                oldstatus = E2_ENABLE_READ;
                enable_e2();
                break;
              #if DRIVER_EXTRUDERS > 3
                case 3:
                  oldstatus = E3_ENABLE_READ;
                  enable_e3();
                  break;
                #if DRIVER_EXTRUDERS > 4
                  case 4:
                    oldstatus = E4_ENABLE_READ;
                    enable_e4();
                    break;
                  #if DRIVER_EXTRUDERS > 5
                    case 5:
                      oldstatus = E5_ENABLE_READ;
                      enable_e5();
                      break;
                  #endif
                #endif
              #endif
            #endif
          #endif
        }
      #endif // !DONDOLO_SINGLE_MOTOR

      previous_cmd_ms = ms; // refresh_cmd_timeout()

      #if IS_KINEMATIC
        inverse_kinematics(current_position);
        ADJUST_DELTA(current_position);
        planner.buffer_line(
          delta[A_AXIS], delta[B_AXIS], delta[C_AXIS],
          current_position[E_AXIS] + EXTRUDER_RUNOUT_EXTRUDE,
          MMM_TO_MMS(EXTRUDER_RUNOUT_SPEED), active_extruder,
          active_driver
        );
      #else
        planner.buffer_line(
          current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS],
          current_position[E_AXIS] + EXTRUDER_RUNOUT_EXTRUDE,
          MMM_TO_MMS(EXTRUDER_RUNOUT_SPEED), active_extruder,
          active_driver
        );
      #endif
      stepper.synchronize();
      planner.set_e_position_mm(current_position[E_AXIS]);
      #if ENABLED(DONDOLO_SINGLE_MOTOR)
        E0_ENABLE_WRITE(oldstatus);
      #else
        switch(active_extruder) {
          case 0:
            E0_ENABLE_WRITE(oldstatus);
            break;
          #if DRIVER_EXTRUDERS > 1
            case 1:
              E1_ENABLE_WRITE(oldstatus);
              break;
            #if DRIVER_EXTRUDERS > 2
              case 2:
                E2_ENABLE_WRITE(oldstatus);
                break;
              #if DRIVER_EXTRUDERS > 3
                case 3:
                  E3_ENABLE_WRITE(oldstatus);
                  break;
                #if DRIVER_EXTRUDERS > 4
                  case 4:
                    E4_ENABLE_WRITE(oldstatus);
                    break;
                  #if DRIVER_EXTRUDERS > 5
                    case 5:
                      E5_ENABLE_WRITE(oldstatus);
                      break;
                  #endif
                #endif
              #endif
            #endif
          #endif
        }
      #endif // !DONDOLO_SINGLE_MOTOR
    }
  #endif

  #if ENABLED(DUAL_X_CARRIAGE)
    // handle delayed move timeout
    if (delayed_move_time && ELAPSED(ms, delayed_move_time + 1000UL) && IsRunning()) {
      // travel moves have been received so enact them
      delayed_move_time = 0xFFFFFFFFUL; // force moves to be done
      set_destination_to_current();
      prepare_move_to_destination();
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

  planner.check_axes_activity();
}

/**
 * Standard idle routine keeps the machine alive
 */
void idle(
  #if ENABLED(FILAMENT_CHANGE_FEATURE)
    bool no_stepper_sleep/*=false*/
  #endif
) {
  lcd_update();

  host_keepalive();

  #if ENABLED(AUTO_REPORT_TEMPERATURES) && (HAS_TEMP_HOTEND || HAS_TEMP_BED)
    auto_report_temperatures();
  #endif

  #if ENABLED(FLOWMETER_SENSOR)
    flowrate_manage();
  #endif

  manage_inactivity(
    #if ENABLED(FILAMENT_CHANGE_FEATURE)
      no_stepper_sleep
    #endif
  );

  thermalManager.manage_temp_controller();

  print_job_counter.tick();
}

/**
 * Kill all activity and lock the machine.
 * After this the machine will need to be reset.
 */
void kill(const char* lcd_msg) {
  SERIAL_LM(ER, MSG_ERR_KILLED);

  #if ENABLED(KILL_METHOD) && KILL_METHOD == 1
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

  HAL::delayMilliseconds(500); // Wait a short time

  cli(); // Stop interrupts
  thermalManager.disable_all_heaters();
  thermalManager.disable_all_coolers();
  stepper.disable_all_steppers();

  #if ENABLED(LASERBEAM)
    laser_init();
    #if ENABLED(LASER_PERIPHERALS)
      laser_peripherals_off();
    #endif
  #endif

  #if HAS(POWER_SWITCH)
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
void stop() {
  #if ENABLED(FLOWMETER_SENSOR) && ENABLED(MINFLOW_PROTECTION)
    flow_firstread = false;
  #endif

  thermalManager.disable_all_heaters();
  thermalManager.disable_all_coolers();

  #if ENABLED(LASERBEAM)
    if (laser.diagnostics) SERIAL_EM("Laser set to off, stop() called");
    laser_extinguish();
    #if ENABLED(LASER_PERIPHERALS)
      laser_peripherals_off();
    #endif
  #endif

  if (IsRunning()) {
    Running = false;
    Stopped_gcode_LastN = gcode_LastN; // Save last g_code for restart
    SERIAL_LM(ER, MSG_ERR_STOPPED);
    SERIAL_S(PAUSE);
    SERIAL_E;
    LCD_MESSAGEPGM(MSG_STOPPED);
  }
}

/**
 * Marlin entry-point: Set up before the program loop
 *  - Set up Alligator Board
 *  - Set up the kill pin, filament runout, power hold
 *  - Start the serial port
 *  - Print startup messages and diagnostics
 *  - Get EEPROM or default settings
 *  - Initialize managers for:
 *    • temperature
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

  #if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
    setup_alligator_board();    // Initialize Alligator Board
  #elif MB(ULTRATRONICS)
    setup_ultratronics_board(); // Initialize Ultratronics Board
  #endif

  #if ENABLED(FILAMENT_RUNOUT_SENSOR)
    setup_filrunoutpin();
  #endif

  setup_killpin();

  setup_powerhold();

  #if HAS(STEPPER_RESET)
    disableStepperDrivers();
  #endif

  SERIAL_INIT(BAUDRATE);
  SERIAL_L(START);
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

  #if ENABLED(SDSUPPORT)
    // loads custom configuration from SDCARD if available else uses defaults
    card.RetrieveSettings();
  #endif

  // loads data from EEPROM if available else uses defaults (and resets step acceleration rate)
  eeprom.RetrieveSettings();

  // Initialize current position based on home_offset
  memcpy(current_position, home_offset, sizeof(home_offset));

  // Vital to init stepper/planner equivalent for current_position
  SYNC_PLAN_POSITION_KINEMATIC();

  thermalManager.init();    // Initialize temperature loop

  #if ENABLED(USE_WATCHDOG)
    watchdog_init();
  #endif

  stepper.init();    // Initialize stepper, this enables interrupts!
  servo_init();

  #if HAS_PHOTOGRAPH
    OUT_WRITE(PHOTOGRAPH_PIN, LOW);
  #endif

  #if HAS_CASE_LIGHT
    setup_case_light();
  #endif

  #if HAS(BED_PROBE)
    endstops.enable_z_probe(false);
  #endif

  #if HAS(STEPPER_RESET)
    enableStepperDrivers();
  #endif

  #if ENABLED(DIGIPOT_I2C)
    digipot_i2c_init();
  #endif

  #if HAS(Z_PROBE_SLED)
    OUT_WRITE(SLED_PIN, LOW); // turn it off
  #endif

  setup_homepin();

  #if PIN_EXISTS(STAT_LED_RED)
    OUT_WRITE(STAT_LED_RED_PIN, LOW); // turn it off
  #endif

  #if PIN_EXISTS(STAT_LED_BLUE)
    OUT_WRITE(STAT_LED_BLUE_PIN, LOW); // turn it off
  #endif

  #if ENABLED(LASERBEAM)
    laser_init();
  #endif

  #if ENABLED(FLOWMETER_SENSOR)
    #if ENABLED(MINFLOW_PROTECTION)
      flow_firstread = false;
    #endif
    flow_init();
  #endif

  #if ENABLED(COLOR_MIXING_EXTRUDER) && MIXING_VIRTUAL_TOOLS > 1
    // Initialize mixing to 100% color 1
    for (uint8_t i = 0; i < MIXING_STEPPERS; i++)
      mixing_factor[i] = (i == 0) ? 1.0 : 0.0;
    for (uint8_t t = 0; t < MIXING_VIRTUAL_TOOLS; t++)
      for (uint8_t i = 0; i < MIXING_STEPPERS; i++)
        mixing_virtual_tool_mix[t][i] = mixing_factor[i];
  #endif

  #if ENABLED(RFID_MODULE)
    RFID_ON = RFID522.init();
    if (RFID_ON)
      SERIAL_EM("RFID CONNECT");
  #endif

  #if ENABLED(FIRMWARE_TEST)
    FirmwareTest();
  #endif

  lcd_init();
  #if ENABLED(SHOW_BOOTSCREEN)
    #if ENABLED(DOGLCD)
      safe_delay(SPLASH_SCREEN_DURATION);
    #elif ENABLED(ULTRA_LCD)
      bootscreen();
      #if DISABLED(SDSUPPORT)
        lcd_init();
      #endif
    #endif
  #endif
}

/**
 * The main Marlin program loop
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

  #if ENABLED(SDSUPPORT)
    card.checkautostart(false);
  #endif

  if (commands_in_queue) {

    #if ENABLED(SDSUPPORT)

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
      cmd_queue_index_r = (cmd_queue_index_r + 1) % BUFSIZE;
    }
  }
  endstops.report_state();
  idle();
}
