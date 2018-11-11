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

#include "../../MK4duo.h"

#if ENABLED(ULTRA_LCD)

#include <stdarg.h>

#include "lcdprint.h"

// Buttons
volatile uint8_t buttons;

#if ENABLED(SDSUPPORT) && PIN_EXISTS(SD_DETECT)
  uint8_t lcd_sd_status;
#endif

#if ENABLED(STATUS_MESSAGE_SCROLLING)
  uint8_t status_scroll_offset = 0;
  #if LONG_FILENAME_LENGTH > CHARSIZE * 2 * (LCD_WIDTH)
    #define MAX_MESSAGE_LENGTH LONG_FILENAME_LENGTH
  #else
    #define MAX_MESSAGE_LENGTH CHARSIZE * 2 * (LCD_WIDTH)
  #endif
#else
  #define MAX_MESSAGE_LENGTH CHARSIZE * (LCD_WIDTH)
#endif

char lcd_status_message[MAX_MESSAGE_LENGTH + 1];
uint8_t lcd_status_update_delay = 1, // First update one loop delayed
        lcd_status_message_level;    // Higher level blocks lower level

#if (HAS_LCD_FILAMENT_SENSOR && HAS_SD_SUPPORT) || HAS_LCD_POWER_SENSOR
  millis_t previous_lcd_status_ms = 0;
#endif

#if ENABLED(ULTIPANEL) && HAS_SD_SUPPORT && ENABLED(SCROLL_LONG_FILENAMES)
  uint8_t filename_scroll_pos, filename_scroll_max;
#endif

millis_t next_button_update_ms;

#if HAS_GRAPHICAL_LCD
  bool drawing_screen, first_page; // = false
#endif

#if HAS_LCD_POWER_SENSOR
  millis_t print_millis = 0;
#endif

// Encoder Handling
#if HAS_ENCODER_ACTION
  uint32_t encoderPosition;
  volatile int8_t encoderDiff; // Updated in lcd_buttons_update, added to encoderPosition every LCD update
  #if ENABLED(ENCODER_RATE_MULTIPLIER)
    bool encoderRateMultiplierEnabled;
  #endif
  #if ENABLED(REVERSE_MENU_DIRECTION)
    int8_t encoderDirection = 1;
  #endif
#endif

#if ENABLED(ULTIPANEL)
  #include "menu/menu.h"

  screenFunc_t currentScreen = lcd_status_screen;

  #if ENABLED(ENCODER_RATE_MULTIPLIER)
    millis_t lastEncoderMovementMillis = 0;
  #endif

  bool lcd_clicked, wait_for_unclick;
  float move_menu_scale;

  bool use_click() {
    const bool click = lcd_clicked;
    lcd_clicked = false;
    return click;
  }

#else

  constexpr bool lcd_clicked = false;

#endif

void lcd_init() {

  lcd_implementation_init();

  #if ENABLED(NEWPANEL)
    #if BUTTON_EXISTS(EN1)
      SET_INPUT_PULLUP(BTN_EN1);
    #endif
    #if BUTTON_EXISTS(EN2)
      SET_INPUT_PULLUP(BTN_EN2);
    #endif
    #if BUTTON_EXISTS(ENC)
      SET_INPUT_PULLUP(BTN_ENC);
    #endif

    #if ENABLED(REPRAPWORLD_KEYPAD) && DISABLED(ADC_KEYPAD)
      SET_OUTPUT(SHIFT_CLK);
      OUT_WRITE(SHIFT_LD, HIGH);
      SET_INPUT_PULLUP(SHIFT_OUT);
    #endif

    #if BUTTON_EXISTS(UP)
      SET_INPUT(BTN_UP);
    #endif
    #if BUTTON_EXISTS(DWN)
      SET_INPUT(BTN_DWN);
    #endif
    #if BUTTON_EXISTS(LFT)
      SET_INPUT(BTN_LFT);
    #endif
    #if BUTTON_EXISTS(RT)
      SET_INPUT(BTN_RT);
    #endif

  #else // !NEWPANEL

    #if ENABLED(SR_LCD_2W_NL) // Non latching 2 wire shift register
      SET_OUTPUT(SR_DATA_PIN);
      SET_OUTPUT(SR_CLK_PIN);
    #elif ENABLED(SHIFT_CLK)
      SET_OUTPUT(SHIFT_CLK);
      OUT_WRITE(SHIFT_LD, HIGH);
      OUT_WRITE(SHIFT_EN, LOW);
      SET_INPUT_PULLUP(SHIFT_OUT);
    #endif // SR_LCD_2W_NL

  #endif // !NEWPANEL

  #if HAS_SD_SUPPORT && PIN_EXISTS(SD_DETECT)
    SET_INPUT_PULLUP(SD_DETECT_PIN);
    lcd_sd_status = 2; // UNKNOWN
  #endif

  #if ENABLED(LCD_HAS_SLOW_BUTTONS)
    slow_buttons = 0;
  #endif

  lcd_buttons_update();

  #if HAS_ENCODER_ACTION
    encoderDiff = 0;
  #endif
}

bool lcd_blink() {
  static uint8_t blink = 0;
  static millis_t next_blink_ms = 0;
  millis_t ms = millis();
  if (ELAPSED(ms, next_blink_ms)) {
    blink ^= 0xFF;
    next_blink_ms = ms + 1000 - (LCD_UPDATE_INTERVAL) / 2;
  }
  return blink != 0;
}

////////////////////////////////////////////
///////////// Keypad Handling //////////////
////////////////////////////////////////////

#if ENABLED(REPRAPWORLD_KEYPAD)
  volatile uint8_t buttons_reprapworld_keypad;
#endif

#if ENABLED(ADC_KEYPAD)

  inline bool handle_adc_keypad() {
    #define ADC_MIN_KEY_DELAY 100
    if (buttons_reprapworld_keypad) {
      #if HAS_ENCODER_ACTION
        lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
        if (encoderDirection == -1) { // side effect which signals we are inside a menu
          #if ENABLED(ULTIPANEL)
            if      (RRK(EN_REPRAPWORLD_KEYPAD_DOWN))   encoderPosition -= ENCODER_STEPS_PER_MENU_ITEM;
            else if (RRK(EN_REPRAPWORLD_KEYPAD_UP))     encoderPosition += ENCODER_STEPS_PER_MENU_ITEM;
            else if (RRK(EN_REPRAPWORLD_KEYPAD_LEFT))   { menu_item_back::action(); lcd_quick_feedback(true); }
            else if (RRK(EN_REPRAPWORLD_KEYPAD_RIGHT))  { lcd_return_to_status(); lcd_quick_feedback(true); }
          #endif
        }
        else if (RRK(EN_REPRAPWORLD_KEYPAD_DOWN))     encoderPosition += ENCODER_PULSES_PER_STEP;
        else if (RRK(EN_REPRAPWORLD_KEYPAD_UP))       encoderPosition -= ENCODER_PULSES_PER_STEP;
        else if (RRK(EN_REPRAPWORLD_KEYPAD_RIGHT))    encoderPosition = 0;
      #endif
      next_button_update_ms = millis() + ADC_MIN_KEY_DELAY;
      return true;
    }

    return false;
  }

#elif ENABLED(REPRAPWORLD_KEYPAD)

  #if ENABLED(ULTIPANEL)

    void lcd_move_x();
    void lcd_move_y();
    void lcd_move_z();

    void _reprapworld_keypad_move(const AxisEnum axis, const int16_t dir) {
      move_menu_scale = REPRAPWORLD_KEYPAD_MOVE_STEP;
      encoderPosition = dir;
      switch (axis) {
        case X_AXIS: lcd_move_x(); break;
        case Y_AXIS: lcd_move_y(); break;
        case Z_AXIS: lcd_move_z();
        default: break;
      }
    }

  #endif

  inline void handle_reprapworld_keypad() {

    static uint8_t keypad_debounce = 0;

    if (!RRK( EN_REPRAPWORLD_KEYPAD_F1    | EN_REPRAPWORLD_KEYPAD_F2
            | EN_REPRAPWORLD_KEYPAD_F3    | EN_REPRAPWORLD_KEYPAD_DOWN
            | EN_REPRAPWORLD_KEYPAD_RIGHT | EN_REPRAPWORLD_KEYPAD_MIDDLE
            | EN_REPRAPWORLD_KEYPAD_UP    | EN_REPRAPWORLD_KEYPAD_LEFT )
    ) {
      if (keypad_debounce > 0) keypad_debounce--;
    }
    else if (!keypad_debounce) {
      keypad_debounce = 2;

      #if ENABLED(ULTIPANEL)

        if (RRK(EN_REPRAPWORLD_KEYPAD_MIDDLE))  lcd_goto_screen(menu_move);

        #if NOMECH(DELTA) && Z_HOME_DIR == -1
          if (RRK(EN_REPRAPWORLD_KEYPAD_F2))    _reprapworld_keypad_move(Z_AXIS,  1);
        #endif

        if (printer.isHomedAll()) {
          #if MECH(DELTA) || Z_HOME_DIR != -1
            if (RRK(EN_REPRAPWORLD_KEYPAD_F2))  _reprapworld_keypad_move(Z_AXIS,  1);
          #endif
          if (RRK(EN_REPRAPWORLD_KEYPAD_F3))    _reprapworld_keypad_move(Z_AXIS, -1);
          if (RRK(EN_REPRAPWORLD_KEYPAD_LEFT))  _reprapworld_keypad_move(X_AXIS, -1);
          if (RRK(EN_REPRAPWORLD_KEYPAD_RIGHT)) _reprapworld_keypad_move(X_AXIS,  1);
          if (RRK(EN_REPRAPWORLD_KEYPAD_DOWN))  _reprapworld_keypad_move(Y_AXIS,  1);
          if (RRK(EN_REPRAPWORLD_KEYPAD_UP))    _reprapworld_keypad_move(Y_AXIS, -1);
        }

      #endif // ENABLED(ULTIPANEL)

      if (!printer.isHomedAll() && RRK(EN_REPRAPWORLD_KEYPAD_F1)) commands.enqueue_and_echo_P(PSTR("G28"));
    }
  }

#endif // REPRAPWORLD_KEYPAD

/**
 * Status Screen
 *
 * This is very display-dependent, so the lcd implementation draws this.
 */

#if ENABLED(LCD_PROGRESS_BAR)
  millis_t progress_bar_ms = 0;     // Start millis of the current progress bar cycle
  #if PROGRESS_MSG_EXPIRE > 0
    static millis_t expire_status_ms = 0;
    void dontExpireStatus() { expire_status_ms = 0; }
  #endif
#endif

#if LCD_INFO_SCREEN_STYLE == 0
  void lcd_impl_status_screen_0();
#elif LCD_INFO_SCREEN_STYLE == 1
  void lcd_impl_status_screen_1();
#endif

void lcd_status_screen() {

  #if ENABLED(ULTIPANEL)
    ENCODER_DIRECTION_NORMAL();
    ENCODER_RATE_MULTIPLY(false);
  #endif

  #if ENABLED(LCD_PROGRESS_BAR)

    //
    // HD44780 implements the following message blinking and
    // message expiration because Status Line and Progress Bar
    // share the same line on the display.
    //

    millis_t ms = millis();

    // If the message will blink rather than expire...
    #if DISABLED(PROGRESS_MSG_ONCE)
      if (ELAPSED(ms, progress_bar_ms + PROGRESS_BAR_MSG_TIME + PROGRESS_BAR_BAR_TIME))
        progress_bar_ms = ms;
    #endif

    #if PROGRESS_MSG_EXPIRE > 0

      // Handle message expire
      if (expire_status_ms > 0) {

        // Expire the message if a job is active and the bar has ticks
        if (printer.progress > 2 && print_job_counter.isPaused()) {
          if (ELAPSED(ms, expire_status_ms)) {
            lcd_status_message[0] = '\0';
            expire_status_ms = 0;
          }
        }
        else {
          // Defer message expiration before bar appears
          // and during any pause (not just SD)
          expire_status_ms += LCD_UPDATE_INTERVAL;
        }
      }

    #endif // PROGRESS_MSG_EXPIRE

  #endif // LCD_PROGRESS_BAR

  #if ENABLED(ULTIPANEL)

    if (use_click()) {
      #if (HAS_LCD_FILAMENT_SENSOR && ENABLED(SDSUPPORT)) || HAS_LCD_POWER_SENSOR
        previous_lcd_status_ms = millis();  // get status message to show up for a while
      #endif
      lcd_goto_screen(menu_main);
      lcd_implementation_init(); // May revive the LCD if static electricity killed it
      return;
    }

  #endif // ENABLED(ULTIPANEL)

  #if ENABLED(ULTIPANEL_FEEDMULTIPLY)

    const int16_t new_frm = mechanics.feedrate_percentage + (int32_t)encoderPosition;
    // Dead zone at 100% feedrate
    if ((mechanics.feedrate_percentage < 100 && new_frm > 100) || (mechanics.feedrate_percentage > 100 && new_frm < 100)) {
      mechanics.feedrate_percentage = 100;
      encoderPosition = 0;
    }
    else if (mechanics.feedrate_percentage == 100) {
      if ((int32_t)encoderPosition > ENCODER_FEEDRATE_DEADZONE) {
        mechanics.feedrate_percentage += (int32_t)encoderPosition - (ENCODER_FEEDRATE_DEADZONE);
        encoderPosition = 0;
      }
      else if ((int32_t)encoderPosition < -(ENCODER_FEEDRATE_DEADZONE)) {
        mechanics.feedrate_percentage += (int32_t)encoderPosition + ENCODER_FEEDRATE_DEADZONE;
        encoderPosition = 0;
      }
    }
    else {
      mechanics.feedrate_percentage = new_frm;
      encoderPosition = 0;
    }

    mechanics.feedrate_percentage = constrain(mechanics.feedrate_percentage, 10, 999);

  #endif // ULTIPANEL_FEEDMULTIPLY

  #if LCD_INFO_SCREEN_STYLE == 0
    lcd_impl_status_screen_0();
  #elif LCD_INFO_SCREEN_STYLE == 1
    lcd_impl_status_screen_1();
  #endif
}

/**
 * Reset the status message
 */
void lcd_reset_status() {
  static const char paused[] PROGMEM = MSG_PRINT_PAUSED;
  static const char printing[] PROGMEM = MSG_PRINTING;
  static const char welcome[] PROGMEM = WELCOME_MSG;
  PGM_P msg;
  if (print_job_counter.isPaused())
    msg = paused;
  #if HAS_SD_SUPPORT
    else if (IS_SD_PRINTING())
      return lcd_setstatus(card.fileName, true);
  #endif
  else if (print_job_counter.isRunning())
    msg = printing;
  else
    msg = welcome;

  lcd_setstatusPGM(msg, -1);
}

void lcd_kill_screen(PGM_P lcd_msg) {
  lcd_init();
  lcd_setalertstatusPGM(lcd_msg);
  lcd_kill_screen();
}

void lcd_quick_feedback(const bool clear_buttons) {

  #if ENABLED(ULTIPANEL)
    lcd_refresh();
    if (clear_buttons) buttons = 0;
    next_button_update_ms = millis() + 500;
  #else
    UNUSED(clear_buttons);
  #endif

  // Buzz and wait. The delay is needed for buttons to settle!
  sound.playTone(LCD_FEEDBACK_FREQUENCY_DURATION_MS, LCD_FEEDBACK_FREQUENCY_HZ);

  #if ENABLED(ULTIPANEL)
    #if ENABLED(LCD_USE_I2C_BUZZER)
      HAL::delayMilliseconds(10);
    #endif
  #endif
}

#if ENABLED(ULTIPANEL)

  extern bool no_reentry; // Flag to prevent recursion into menu handlers

  int8_t manual_move_axis = (int8_t)NO_AXIS;
  millis_t manual_move_start_time = 0;

  #if IS_KINEMATIC
    bool processing_manual_move = false;
    float manual_move_offset = 0;
  #endif

  #if !IS_KINEMATIC || (IS_KINEMATIC && EXTRUDERS > 1)
    int8_t manual_move_e_index = 0;
  #else
    constexpr int8_t manual_move_e_index = 0;
  #endif

  /**
   * If the most recent manual move hasn't been fed to the planner yet,
   * and the planner can accept one, send a move immediately.
   */
  void manage_manual_move() {

    if (processing_manual_move) return;

    if (manual_move_axis != (int8_t)NO_AXIS && ELAPSED(millis(), manual_move_start_time) && !planner.is_full()) {

      #if IS_KINEMATIC

        const float old_feedrate = mechanics.feedrate_mm_s;
        mechanics.feedrate_mm_s = MMM_TO_MMS(manual_feedrate_mm_m[manual_move_axis]);

        #if EXTRUDERS > 1
          const int8_t old_extruder = tools.active_extruder;
          tools.active_extruder = manual_move_e_index;
        #endif

        // Set movement on a single axis
        mechanics.set_destination_to_current();
        mechanics.destination[manual_move_axis] += manual_move_offset;

        // Reset for the next move
        manual_move_offset = 0;
        manual_move_axis = (int8_t)NO_AXIS;

        // DELTA and SCARA machines use segmented moves, which could fill the planner during the call to
        // move_to_destination. This will cause idle() to be called, which can then call this function while the
        // previous invocation is being blocked. Modifications to manual_move_offset shouldn't be made while
        // processing_manual_move is true or the planner will get out of sync.
        processing_manual_move = true;
        mechanics.prepare_move_to_destination(); // will call set_current_to_destination
        processing_manual_move = false;

        mechanics.feedrate_mm_s = old_feedrate;
        #if EXTRUDERS > 1
          tools.active_extruder = old_extruder;
        #endif

      #else

        planner.buffer_line(mechanics.current_position, MMM_TO_MMS(manual_feedrate_mm_m[manual_move_axis]), tools.active_extruder);
        manual_move_axis = (int8_t)NO_AXIS;

      #endif
    }
  }

#endif // ENABLED(ULTIPANEL)

/**
 * Update the LCD, read encoder buttons, etc.
 *   - Read button states
 *   - Check the SD Card slot state
 *   - Act on RepRap World keypad input
 *   - Update the encoder position
 *   - Apply acceleration to the encoder position
 *   - Set lcdDrawUpdate = LCDVIEW_CALL_REDRAW_NOW on controller events
 *   - Reset the Info Screen timeout if there's any input
 *   - Update status indicators, if any
 *
 *   Run the current LCD menu handler callback function:
 *   - Call the handler only if lcdDrawUpdate != LCDVIEW_NONE
 *   - Before calling the handler, LCDVIEW_CALL_NO_REDRAW => LCDVIEW_NONE
 *   - Call the menu handler. Menu handlers should do the following:
 *     - If a value changes, set lcdDrawUpdate to LCDVIEW_REDRAW_NOW and draw the value
 *       (Encoder events automatically set lcdDrawUpdate for you.)
 *     - if (lcdDrawUpdate) { redraw }
 *     - Before exiting the handler set lcdDrawUpdate to:
 *       - LCDVIEW_CLEAR_CALL_REDRAW to clear screen and set LCDVIEW_CALL_REDRAW_NEXT.
 *       - LCDVIEW_REDRAW_NOW to draw now (including remaining stripes).
 *       - LCDVIEW_CALL_REDRAW_NEXT to draw now and get LCDVIEW_REDRAW_NOW on the next loop.
 *       - LCDVIEW_CALL_NO_REDRAW to draw now and get LCDVIEW_NONE on the next loop.
 *     - NOTE: For graphical displays menu handlers may be called 2 or more times per loop,
 *             so don't change lcdDrawUpdate without considering this.
 *
 *   After the menu handler callback runs (or not):
 *   - Clear the LCD if lcdDrawUpdate == LCDVIEW_CLEAR_CALL_REDRAW
 *   - Update lcdDrawUpdate for the next loop (i.e., move one state down, usually)
 *
 * This function is only called from the main thread.
 */

LCDViewActionEnum lcdDrawUpdate = LCDVIEW_CLEAR_CALL_REDRAW;

#if ENABLED(AUTO_BED_LEVELING_UBL) || ENABLED(G26_MESH_VALIDATION)
  bool lcd_external_control = false;
#endif

#if ENABLED(LCD_HAS_SLOW_BUTTONS)
  volatile uint8_t slow_buttons;
#endif

bool lcd_detected() {
  return
    #if (ENABLED(LCD_I2C_TYPE_MCP23017) || ENABLED(LCD_I2C_TYPE_MCP23008)) && defined(DETECT_DEVICE)
      lcd.LcdDetected() == 1
    #else
      true
    #endif
  ;
}

void lcd_update() {

  static uint16_t max_display_update_time = 0;
  static millis_t next_lcd_update_ms;

  #if ENABLED(ULTIPANEL)

    #if LCD_TIMEOUT_TO_STATUS
      static millis_t return_to_status_ms = 0;
    #endif

    // Handle any queued Move Axis motion
    manage_manual_move();

    // Update button states for LCD_CLICKED(), etc.
    // After state changes the next button update
    // may be delayed 300-500ms.
    lcd_buttons_update();

    #if ENABLED(AUTO_BED_LEVELING_UBL)
      // Don't run the debouncer if UBL owns the display
      #define UBL_CONDITION !lcd_external_control
    #else
      #define UBL_CONDITION true
    #endif

    // If the action button is pressed...
    if (UBL_CONDITION && LCD_CLICKED()) {
      if (!wait_for_unclick) {                                  // If not waiting for a debounce release:
        wait_for_unclick = true;                                // Set debounce flag to ignore continous clicks
        lcd_clicked = !printer.isWaitForUser() && !no_reentry;  // Keep the click if not waiting for a user-click
        printer.setWaitForUser(false);                          // Any click clears wait for user
        lcd_quick_feedback(true);                               // Always make a click sound
      }
    }
    else wait_for_unclick = false;

    #if BUTTON_EXISTS(BACK)
      if (LCD_BACK_CLICKED) {
        lcd_quick_feedback(true);
        lcd_goto_previous_menu();
      }
    #endif

  #endif // ENABLED(ULTIPANEL)

  #if HAS_SD_SUPPORT && PIN_EXISTS(SD_DETECT)

    const uint8_t sd_status = (uint8_t)IS_SD_INSERTED();
    if (sd_status != lcd_sd_status && lcd_detected()) {

      uint8_t old_sd_status = lcd_sd_status; // prevent re-entry to this block!
      lcd_sd_status = sd_status;

      if (sd_status) {
        printer.safe_delay(500);  // Some boards need a delay to get settled
        card.mount();
        if (old_sd_status == 2)
          card.beginautostart();  // Initial boot
        else
          LCD_MESSAGEPGM(MSG_SD_INSERTED);
      }
      else {
        card.unmount();
        if (old_sd_status != 2) LCD_MESSAGEPGM(MSG_SD_REMOVED);
      }

      lcd_refresh();
      lcd_implementation_init(); // May revive the LCD if static electricity killed it
    }

  #endif // HAS_SD_SUPPORT && SD_DETECT_PIN

  #if HAS_SD_RESTART
    if (restart.count && restart.job_phase == RESTART_IDLE) {
      lcd_goto_screen(menu_sdcard_restart);
      restart.job_phase = RESTART_MAYBE; // Waiting for a response
    }
  #endif

  const millis_t ms = millis();
  if (ELAPSED(ms, next_lcd_update_ms)
    #if HAS_GRAPHICAL_LCD
      || drawing_screen
    #endif
  ) {

    next_lcd_update_ms = ms + LCD_UPDATE_INTERVAL;

    #if ENABLED(LCD_HAS_STATUS_INDICATORS)
      lcd_implementation_update_indicators();
    #endif

    #if ENABLED(ULTIPANEL)

      #if ENABLED(LCD_HAS_SLOW_BUTTONS)
        slow_buttons = lcd_implementation_read_slow_buttons(); // buttons which take too long to read in interrupt context
      #endif

    #endif // ENABLED(ULTIPANEL)

    #if HAS_ENCODER_ACTION

      #if ENABLED(ADC_KEYPAD)

        if (handle_adc_keypad()) {
          #if ENABLED(ULTIPANEL) && LCD_TIMEOUT_TO_STATUS
            return_to_status_ms = ms + LCD_TIMEOUT_TO_STATUS;
          #endif
        }

      #elif ENABLED(REPRAPWORLD_KEYPAD)

        handle_reprapworld_keypad();

      #endif

      const bool encoderPastThreshold = (ABS(encoderDiff) >= ENCODER_PULSES_PER_STEP);
      if (encoderPastThreshold || lcd_clicked) {
        if (encoderPastThreshold) {

          #if ENABLED(ULTIPANEL) && ENABLED(ENCODER_RATE_MULTIPLIER)

            int32_t encoderMultiplier = 1;

            if (encoderRateMultiplierEnabled) {
              int32_t encoderMovementSteps = ABS(encoderDiff) / ENCODER_PULSES_PER_STEP;

              if (lastEncoderMovementMillis) {
                // Note that the rate is always calculated between two passes through the
                // loop and that the abs of the encoderDiff value is tracked.
                float encoderStepRate = float(encoderMovementSteps) / float(ms - lastEncoderMovementMillis) * 1000;

                if (encoderStepRate >= ENCODER_100X_STEPS_PER_SEC)     encoderMultiplier = 100;
                else if (encoderStepRate >= ENCODER_10X_STEPS_PER_SEC) encoderMultiplier = 10;

                #if ENABLED(ENCODER_RATE_MULTIPLIER_DEBUG)
                  SERIAL_SMV(DEB, "Enc Step Rate: ", encoderStepRate);
                  SERIAL_MV("  Multiplier: ", encoderMultiplier);
                  SERIAL_MV("  ENCODER_10X_STEPS_PER_SEC: ", ENCODER_10X_STEPS_PER_SEC);
                  SERIAL_EMV("  ENCODER_100X_STEPS_PER_SEC: ", ENCODER_100X_STEPS_PER_SEC);
                #endif
              }

              lastEncoderMovementMillis = ms;
            } // encoderRateMultiplierEnabled

          #else

            constexpr int32_t encoderMultiplier = 1;

          #endif // ENCODER_RATE_MULTIPLIER

          encoderPosition += (encoderDiff * encoderMultiplier) / ENCODER_PULSES_PER_STEP;
          encoderDiff = 0;
        }
        #if ENABLED(ULTIPANEL) && LCD_TIMEOUT_TO_STATUS
          return_to_status_ms = ms + LCD_TIMEOUT_TO_STATUS;
        #endif
        lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
      }

    #endif

    // This runs every ~100ms when idling often enough.
    // Instead of tracking changes just redraw the Status Screen once per second.
    if (
      #if ENABLED(ULTIPANEL)
        currentScreen == lcd_status_screen &&
      #endif
      !lcd_status_update_delay--
    ) {
      lcd_status_update_delay = 9
        #if HAS_GRAPHICAL_LCD
          + 3
        #endif
      ;
      max_display_update_time--;
      lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
    }

    #if ENABLED(ULTIPANEL) && ENABLED(SCROLL_LONG_FILENAMES)
      // If scrolling of long file names is enabled and we are in the sd card menu,
      // cause a refresh to occur until all the text has scrolled into view.
      if (currentScreen == menu_sdcard && filename_scroll_pos < filename_scroll_max && !lcd_status_update_delay--) {
        lcd_status_update_delay = 6;
        lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
        filename_scroll_pos++;
        #if LCD_TIMEOUT_TO_STATUS
          return_to_status_ms = ms + LCD_TIMEOUT_TO_STATUS;
        #endif
      }
    #endif

    // then we want to use 1/2 of the time only.
    uint16_t bbr2 = planner.block_buffer_runtime() >> 1;

    #if HAS_GRAPHICAL_LCD
      const bool &is_drawing = drawing_screen;
    #else
      constexpr bool is_drawing = false;
    #endif

    if ((lcdDrawUpdate || is_drawing) && (!bbr2 || bbr2 > max_display_update_time)) {

      // Change state of drawing flag between screen updates
      if (!is_drawing) switch (lcdDrawUpdate) {
        case LCDVIEW_CALL_NO_REDRAW:
          lcdDrawUpdate = LCDVIEW_NONE;
          break;
        case LCDVIEW_CLEAR_CALL_REDRAW:
        case LCDVIEW_CALL_REDRAW_NEXT:
          lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
        case LCDVIEW_REDRAW_NOW:        // set above, or by a handler through LCDVIEW_CALL_REDRAW_NEXT
        case LCDVIEW_NONE:
          break;
      } // switch

      #if ENABLED(ADC_KEYPAD)
        buttons_reprapworld_keypad = 0;
      #endif

      #if ENABLED(ULTIPANEL)
        #define CURRENTSCREEN() (*currentScreen)()
      #else
        #define CURRENTSCREEN() lcd_status_screen()
      #endif

      #if HAS_GRAPHICAL_LCD
        if (!drawing_screen) {                        // If not already drawing pages
          u8g.firstPage();                            // Start the first page
          drawing_screen = first_page = true;         // Flag as drawing pages
        }
        lcd_setFont(FONT_MENU);                       // Setup font for every page draw
        u8g.setColorIndex(1);                         // And reset the color
        CURRENTSCREEN();                              // Draw and process the current screen
        first_page = false;

        // The screen handler can clear drawing_screen for an action that changes the screen.
        // If still drawing and there's another page, update max-time and return now.
        // The nextPage will already be set up on the next call.
        if (drawing_screen && (drawing_screen = u8g.nextPage())) {
          NOLESS(max_display_update_time, millis() - ms);
          return;
        }
      #else
        CURRENTSCREEN();
      #endif

      #if ENABLED(ULTIPANEL)
        lcd_clicked = false;
      #endif

      // Keeping track of the longest time for an individual LCD update.
      // Used to do screen throttling when the planner starts to fill up.
      NOLESS(max_display_update_time, millis() - ms);
    }

    #if ENABLED(ULTIPANEL) && LCD_TIMEOUT_TO_STATUS
      // Return to Status Screen after a timeout
      if (currentScreen == lcd_status_screen || defer_return_to_status)
        return_to_status_ms = ms + LCD_TIMEOUT_TO_STATUS;
      else if (ELAPSED(ms, return_to_status_ms))
        lcd_return_to_status();
    #endif

    // Change state of drawing flag between screen updates
    if (!is_drawing) switch (lcdDrawUpdate) {
      case LCDVIEW_CLEAR_CALL_REDRAW:
        lcd_implementation_clear(); break;
      case LCDVIEW_REDRAW_NOW:
        lcdDrawUpdate = LCDVIEW_NONE;
      case LCDVIEW_NONE:
      case LCDVIEW_CALL_REDRAW_NEXT:
      case LCDVIEW_CALL_NO_REDRAW:
      default: break;
    } // switch

  } // ELAPSED(ms, next_lcd_update_ms)
}

void lcd_finishstatus(const bool persist=false) {

  #if !(ENABLED(LCD_PROGRESS_BAR) && (PROGRESS_MSG_EXPIRE > 0))
    UNUSED(persist);
  #endif

  #if ENABLED(LCD_PROGRESS_BAR)
    progress_bar_ms = millis();
    #if PROGRESS_MSG_EXPIRE > 0
      expire_status_ms = persist ? 0 : progress_bar_ms + PROGRESS_MSG_EXPIRE;
    #endif
  #endif

  lcd_refresh();

  #if (HAS_LCD_FILAMENT_SENSOR && ENABLED(SDSUPPORT)) || HAS_LCD_POWER_SENSOR
    previous_lcd_status_ms = millis();  // get status message to show up for a while
  #endif

  #if ENABLED(STATUS_MESSAGE_SCROLLING)
    status_scroll_offset = 0;
  #endif
}

bool lcd_hasstatus() { return (lcd_status_message[0] != '\0'); }

void lcd_setstatus(PGM_P const message, const bool persist) {
  if (lcd_status_message_level > 0) return;

  // Here we have a problem. The message is encoded in UTF8, so
  // arbitrarily cutting it will be a problem. We MUST be sure
  // that there is no cutting in the middle of a multibyte character!

  // Get a pointer to the null terminator
  PGM_P pend = message + strlen(message);

  //  If length of supplied UTF8 string is greater than
  // our buffer size, start cutting whole UTF8 chars
  while ((pend - message) > MAX_MESSAGE_LENGTH) {
    --pend;
    while (!START_OF_UTF8_CHAR(*pend)) --pend;
  };

  // At this point, we have the proper cut point. Use it
  uint8_t maxLen = pend - message;
  strncpy(lcd_status_message, message, maxLen);
  lcd_status_message[maxLen] = '\0';

  lcd_finishstatus(persist);
}

void lcd_setstatusPGM(PGM_P const message, int8_t level) {
  if (level < 0) level = lcd_status_message_level = 0;
  if (level < lcd_status_message_level) return;
  lcd_status_message_level = level;

  // Here we have a problem. The message is encoded in UTF8, so
  // arbitrarily cutting it will be a problem. We MUST be sure
  // that there is no cutting in the middle of a multibyte character!

  // Get a pointer to the null terminator
  PGM_P pend = message + strlen_P(message);

  //  If length of supplied UTF8 string is greater than
  // our buffer size, start cutting whole UTF8 chars
  while ((pend - message) > MAX_MESSAGE_LENGTH) {
    --pend;
    while (!START_OF_UTF8_CHAR(pgm_read_byte(pend))) --pend;
  };

  // At this point, we have the proper cut point. Use it
  uint8_t maxLen = pend - message;
  strncpy_P(lcd_status_message, message, maxLen);
  lcd_status_message[maxLen] = '\0';

  lcd_finishstatus(level > 0);
}

void lcd_status_printf_P(const uint8_t level, PGM_P const fmt, ...) {
  if (level < lcd_status_message_level) return;
  lcd_status_message_level = level;
  va_list args;
  va_start(args, fmt);
  vsnprintf_P(lcd_status_message, MAX_MESSAGE_LENGTH, fmt, args);
  va_end(args);
  lcd_finishstatus(level > 0);
}

void lcd_setalertstatusPGM(PGM_P const message) {
  lcd_setstatusPGM(message, 1);
  #if ENABLED(ULTIPANEL)
    lcd_return_to_status();
  #endif
}

void lcd_reset_alert_level() { lcd_status_message_level = 0; }

#if ENABLED(ADC_KEYPAD)

  typedef struct {
    uint16_t ADCKeyValueMin, ADCKeyValueMax;
    uint8_t  ADCKeyNo;
  } _stADCKeypadTable_;

  static const _stADCKeypadTable_ stADCKeyTable[] PROGMEM = {
    // VALUE_MIN, VALUE_MAX, KEY
    { 250, 256, BLEN_REPRAPWORLD_KEYPAD_F1 + 1 },     // F1
    { 250, 256, BLEN_REPRAPWORLD_KEYPAD_F2 + 1 },     // F2
    { 250, 256, BLEN_REPRAPWORLD_KEYPAD_F3 + 1 },     // F3
    {  18,  32, BLEN_REPRAPWORLD_KEYPAD_LEFT + 1 },   // LEFT
    { 118, 138, BLEN_REPRAPWORLD_KEYPAD_RIGHT + 1 },  // RIGHT
    {  34,  54, BLEN_REPRAPWORLD_KEYPAD_UP + 1 },     // UP
    { 166, 180, BLEN_REPRAPWORLD_KEYPAD_DOWN + 1 },   // DOWN
    {  70,  90, BLEN_REPRAPWORLD_KEYPAD_MIDDLE + 1 }, // ENTER
  };

  uint8_t get_ADC_keyValue(void) {

    static uint8_t ADCKey_count = 0;
    const uint16_t currentkpADCValue = (HAL::AnalogInputValues[ADC_KEYPAD_PIN] >> 2);

    #if ENABLED(ADC_KEYPAD_DEBUG)
      SERIAL_EV(currentkpADCValue);
    #endif

    if (ADCKey_count < 3) {
      if (currentkpADCValue > 250)
        // ADC Key release
        ADCKey_count = 0;
      else
        ADCKey_count++;
    }
    else {
      ADCKey_count = 0;
      if (currentkpADCValue < 250) {
        for (uint8_t i = 0; i < ADC_KEY_NUM; i++) {
          const uint16_t lo = pgm_read_word(&stADCKeyTable[i].ADCKeyValueMin),
                         hi = pgm_read_word(&stADCKeyTable[i].ADCKeyValueMax);
          if (WITHIN(currentkpADCValue, lo, hi)) return pgm_read_byte(&stADCKeyTable[i].ADCKeyNo);
        }
      }
    }

    return 0;
  }
#endif

#if ENABLED(ULTIPANEL)

  /**
   * Setup Rotary Encoder Bit Values (for two pin encoders to indicate movement)
   * These values are independent of which pins are used for EN_A and EN_B indications
   * The rotary encoder part is also independent to the chipset used for the LCD
   */
  #if ENABLED(EN_A) && ENABLED(EN_B)
    #define encrot0 0
    #define encrot1 2
    #define encrot2 3
    #define encrot3 1
  #endif

  #define GET_SHIFT_BUTTON_STATES(DST) \
    uint8_t new_##DST = 0; \
    WRITE(SHIFT_LD, LOW); \
    WRITE(SHIFT_LD, HIGH); \
    for (int8_t i = 0; i < 8; i++) { \
      new_##DST >>= 1; \
      if (READ(SHIFT_OUT)) SBI(new_##DST, 7); \
      WRITE(SHIFT_CLK, HIGH); \
      WRITE(SHIFT_CLK, LOW); \
    } \
    DST = ~new_##DST; //invert it, because a pressed switch produces a logical 0

  #if ENABLED(G26_MESH_VALIDATION)
    void lcd_chirp() {
      sound.playTone(LCD_FEEDBACK_FREQUENCY_DURATION_MS, LCD_FEEDBACK_FREQUENCY_HZ);
    }
  #endif

  #if ENABLED(AUTO_BED_LEVELING_UBL) || ENABLED(G26_MESH_VALIDATION)
    bool is_lcd_clicked() { return LCD_CLICKED(); }
    void wait_for_release() {
      while (is_lcd_clicked()) printer.safe_delay(50);
      printer.safe_delay(50);
    }
  #endif

  /**
   * Read encoder buttons from the hardware registers
   * Warning: This function is called from interrupt context!
   */
  void lcd_buttons_update() {
    static uint8_t lastEncoderBits;
    const millis_t now = millis();
    if (ELAPSED(now, next_button_update_ms)) {

      #if ENABLED(NEWPANEL)
        uint8_t newbutton = 0;

        #if BUTTON_EXISTS(EN1)
          if (BUTTON_PRESSED(EN1)) newbutton |= EN_A;
        #endif
        #if BUTTON_EXISTS(EN2)
          if (BUTTON_PRESSED(EN2)) newbutton |= EN_B;
        #endif
        #if BUTTON_EXISTS(ENC)
          if (BUTTON_PRESSED(ENC)) newbutton |= EN_C;
        #endif
        #if BUTTON_EXISTS(BACK)
          if (BUTTON_PRESSED(BACK)) newbutton |= EN_D;
        #endif

        //
        // Directional buttons
        //
        #if LCD_HAS_DIRECTIONAL_BUTTONS

          #if ENABLED(REVERSE_MENU_DIRECTION)
            #define _ENCODER_UD_STEPS (ENCODER_STEPS_PER_MENU_ITEM * encoderDirection)
          #else
            #define _ENCODER_UD_STEPS ENCODER_STEPS_PER_MENU_ITEM
          #endif
          #if ENABLED(REVERSE_ENCODER_DIRECTION)
            #define ENCODER_UD_STEPS _ENCODER_UD_STEPS
            #define ENCODER_LR_PULSES ENCODER_PULSES_PER_STEP
          #else
            #define ENCODER_UD_STEPS -(_ENCODER_UD_STEPS)
            #define ENCODER_LR_PULSES -(ENCODER_PULSES_PER_STEP)
          #endif

          if (false) {
            // for the else-ifs below
          }
          #if BUTTON_EXISTS(UP)
            else if (BUTTON_PRESSED(UP)) {
              encoderDiff = -(ENCODER_UD_STEPS);
              next_button_update_ms = now + 300;
            }
          #endif
          #if BUTTON_EXISTS(DWN)
            else if (BUTTON_PRESSED(DWN)) {
              encoderDiff = ENCODER_UD_STEPS;
              next_button_update_ms = now + 300;
            }
          #endif
          #if BUTTON_EXISTS(LFT)
            else if (BUTTON_PRESSED(LFT)) {
              encoderDiff = -(ENCODER_LR_PULSES);
              next_button_update_ms = now + 300;
            }
          #endif
          #if BUTTON_EXISTS(RT)
            else if (BUTTON_PRESSED(RT)) {
              encoderDiff = ENCODER_LR_PULSES;
              next_button_update_ms = now + 300;
            }
          #endif

        #endif // LCD_HAS_DIRECTIONAL_BUTTONS

        buttons = newbutton;
        #if ENABLED(LCD_HAS_SLOW_BUTTONS)
          buttons |= slow_buttons;
        #endif

        #if ENABLED(ADC_KEYPAD)

          uint8_t newbutton_reprapworld_keypad = 0;
          buttons = 0;
          if (buttons_reprapworld_keypad == 0) {
            newbutton_reprapworld_keypad = get_ADC_keyValue();
            if (WITHIN(newbutton_reprapworld_keypad, 1, 8))
              buttons_reprapworld_keypad = _BV(newbutton_reprapworld_keypad - 1);
          }

        #elif ENABLED(REPRAPWORLD_KEYPAD)

          GET_SHIFT_BUTTON_STATES(buttons_reprapworld_keypad);

        #endif

      #else // !NEWPANEL

        GET_SHIFT_BUTTON_STATES(buttons);

      #endif

    } // next_button_update_ms

    // Manage encoder rotation
    #if ENABLED(REVERSE_MENU_DIRECTION) && ENABLED(REVERSE_ENCODER_DIRECTION)
      #define ENCODER_DIFF_CW  (encoderDiff -= encoderDirection)
      #define ENCODER_DIFF_CCW (encoderDiff += encoderDirection)
    #elif ENABLED(REVERSE_MENU_DIRECTION)
      #define ENCODER_DIFF_CW  (encoderDiff += encoderDirection)
      #define ENCODER_DIFF_CCW (encoderDiff -= encoderDirection)
    #elif ENABLED(REVERSE_ENCODER_DIRECTION)
      #define ENCODER_DIFF_CW  (encoderDiff--)
      #define ENCODER_DIFF_CCW (encoderDiff++)
    #else
      #define ENCODER_DIFF_CW  (encoderDiff++)
      #define ENCODER_DIFF_CCW (encoderDiff--)
    #endif
    #define ENCODER_SPIN(_E1, _E2) switch (lastEncoderBits) { case _E1: ENCODER_DIFF_CW; break; case _E2: ENCODER_DIFF_CCW; }

    uint8_t enc = 0;
    if (buttons & EN_A) enc |= B01;
    if (buttons & EN_B) enc |= B10;
    if (enc != lastEncoderBits) {
      switch (enc) {
        case encrot0: ENCODER_SPIN(encrot3, encrot1); break;
        case encrot1: ENCODER_SPIN(encrot0, encrot2); break;
        case encrot2: ENCODER_SPIN(encrot1, encrot3); break;
        case encrot3: ENCODER_SPIN(encrot2, encrot0); break;
      }
      #if ENABLED(AUTO_BED_LEVELING_UBL)
        if (lcd_external_control) {
          ubl.encoder_diff = encoderDiff;   // Make encoder rotation available to UBL G29 mesh editing.
          encoderDiff = 0;                  // Hide the encoder event from the current screen handler.
        }
      #endif
      lastEncoderBits = enc;
    }
  }

  void lcd_eeprom_allert() {
    START_SCREEN();
    STATIC_ITEM(MSG_EEPROM_CHANGED_ALLERT_1);
    STATIC_ITEM(MSG_EEPROM_CHANGED_ALLERT_2);
    STATIC_ITEM(MSG_EEPROM_CHANGED_ALLERT_3);
    STATIC_ITEM(MSG_EEPROM_CHANGED_ALLERT_4);
    END_SCREEN();
  }

#endif // ENABLED(ULTIPANEL)

#endif // ULTRA_LCD
