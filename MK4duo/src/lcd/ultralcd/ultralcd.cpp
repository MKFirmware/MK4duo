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

#include "../../../MK4duo.h"

#if HAS_SPI_LCD

#include <stdarg.h>

LcdUI lcdui;

#if ENABLED(STATUS_MESSAGE_SCROLLING)
  uint8_t LcdUI::status_scroll_offset; // = 0
  #if LONG_FILENAME_LENGTH > CHARSIZE * 2 * (LCD_WIDTH)
    #define MAX_MESSAGE_LENGTH LONG_FILENAME_LENGTH
  #else
    #define MAX_MESSAGE_LENGTH CHARSIZE * 2 * (LCD_WIDTH)
  #endif
#else
  #define MAX_MESSAGE_LENGTH CHARSIZE * (LCD_WIDTH)
#endif

#ifdef MAX_MESSAGE_LENGTH
  uint8_t LcdUI::status_message_level; // = 0
  char LcdUI::status_message[MAX_MESSAGE_LENGTH + 1];
#endif

#if HAS_GRAPHICAL_LCD
  #include "dogm/ultralcd_dogm.h"
#endif

#include "lcdprint.h"

#if HAS_ENCODER_ACTION
  volatile uint8_t LcdUI::buttons;
  #if HAS_SLOW_BUTTONS
    volatile uint8_t LcdUI::slow_buttons;
  #endif
#endif

#if HAS_SD_SUPPORT && PIN_EXISTS(SD_DETECT)
  uint8_t lcd_sd_status;
#endif

#if HAS_LCD_MENU && LCD_TIMEOUT_TO_STATUS
  bool LcdUI::defer_return_to_status;
#endif

uint8_t LcdUI::status_update_delay = 1; // First update one loop delayed

#if (HAS_LCD_FILAMENT_SENSOR && HAS_SD_SUPPORT) || HAS_LCD_POWER_SENSOR
  millis_t LcdUI::previous_status_ms = 0;
#endif

millis_t next_button_update_ms;

#if HAS_GRAPHICAL_LCD
  bool LcdUI::drawing_screen, LcdUI::first_page; // = false
#endif

#if HAS_LCD_POWER_SENSOR
  millis_t LcdUI::print_millis = 0;
#endif

// Encoder Handling
#if HAS_ENCODER_ACTION
  uint32_t LcdUI::encoderPosition;
  volatile int8_t encoderDiff; // Updated in update_buttons, added to encoderPosition every LCD update
#endif

#if HAS_LCD_MENU

  #if HAS_SD_SUPPORT
  
    #if ENABLED(SCROLL_LONG_FILENAMES)
      uint8_t LcdUI::filename_scroll_pos, LcdUI::filename_scroll_max;
    #endif

    const char * LcdUI::scrolled_filename(SDCard &theCard, const uint8_t maxlen, uint8_t hash, const bool doScroll) {
      const char *outstr = theCard.fileName;
      if (theCard.fileName[0]) {
        #if ENABLED(SCROLL_LONG_FILENAMES)
          if (doScroll) {
            for (uint8_t l = FILENAME_LENGTH; l--;)
              hash = ((hash << 1) | (hash >> 7)) ^ theCard.fileName[l];             // rotate, xor
            static uint8_t filename_scroll_hash;
            if (filename_scroll_hash != hash) {                                     // If the hash changed...
              filename_scroll_hash = hash;                                          // Save the new hash
              filename_scroll_max = MAX(0, utf8_strlen(theCard.fileName) - maxlen); // Update the scroll limit
              filename_scroll_pos = 0;                                              // Reset scroll to the start
              status_update_delay = 8;                                          // Don't scroll right away
            }
            outstr += filename_scroll_pos;
          }
        #endif
      }
      return outstr;
    }

  #endif

  screenFunc_t LcdUI::currentScreen;

  #if ENABLED(ENCODER_RATE_MULTIPLIER)
    bool LcdUI::encoderRateMultiplierEnabled;
    millis_t LcdUI::lastEncoderMovementMillis = 0;
    void LcdUI::enable_encoder_multiplier(const bool onoff) {
      encoderRateMultiplierEnabled = onoff;
      lastEncoderMovementMillis = 0;
    }
  #endif

  #if ENABLED(REVERSE_MENU_DIRECTION)
    int8_t LcdUI::encoderDirection = 1;
  #endif

  bool LcdUI::lcd_clicked;
  float move_menu_scale;

  bool LcdUI::use_click() {
    const bool click = lcd_clicked;
    lcd_clicked = false;
    return click;
  }

  #if ENABLED(AUTO_BED_LEVELING_UBL) || ENABLED(G26_MESH_VALIDATION)

    bool LcdUI::external_control; // = false

    void LcdUI::wait_for_release() {
      while (button_pressed()) printer.safe_delay(50);
      printer.safe_delay(50);
    }

  #endif

#endif

void LcdUI::init() {

  init_lcd();

  #if HAS_DIGITAL_BUTTONS

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

  #endif // !HAS_DIGITAL_BUTTONS

  #if HAS_SHIFT_ENCODER

    #if ENABLED(SR_LCD_2W_NL) // Non latching 2 wire shift register

      SET_OUTPUT(SR_DATA_PIN);
      SET_OUTPUT(SR_CLK_PIN);

    #elif ENABLED(SHIFT_CLK)

      SET_OUTPUT(SHIFT_CLK);
      OUT_WRITE(SHIFT_LD, HIGH);
      #if ENABLED(SHIFT_EN) && SHIFT_EN >= 0
        OUT_WRITE(SHIFT_EN, LOW);
      #endif
      SET_INPUT_PULLUP(SHIFT_OUT);

    #endif // SR_LCD_2W_NL

  #endif // !HAS_DIGITAL_BUTTONS

  #if HAS_SD_SUPPORT && PIN_EXISTS(SD_DETECT)
    SET_INPUT_PULLUP(SD_DETECT_PIN);
    lcd_sd_status = 2; // UNKNOWN
  #endif

  #if HAS_ENCODER_ACTION && HAS_SLOW_BUTTONS
    slow_buttons = 0;
  #endif

  update_buttons();

  #if HAS_ENCODER_ACTION
    encoderDiff = 0;
  #endif

}

bool LcdUI::get_blink() {
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

#if ENABLED(REPRAPWORLD_KEYPAD) && HAS_ENCODER_ACTION

  volatile uint8_t LcdUI::keypad_buttons;

  #if HAS_LCD_MENU && !HAS_ADC_BUTTONS

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

  bool LcdUI::handle_keypad() {

    #if HAS_ADC_BUTTONS

      #define ADC_MIN_KEY_DELAY 100
      if (keypad_buttons) {
        #if HAS_ENCODER_ACTION
          refresh(LCDVIEW_REDRAW_NOW);
          #if HAS_LCD_MENU
            if (encoderDirection == -1) { // side effect which signals we are inside a menu
              if      (RRK(EN_KEYPAD_DOWN))   encoderPosition -= ENCODER_STEPS_PER_MENU_ITEM;
              else if (RRK(EN_KEYPAD_UP))     encoderPosition += ENCODER_STEPS_PER_MENU_ITEM;
              else if (RRK(EN_KEYPAD_LEFT))   { MenuItem_back::action(); quick_feedback(); }
              else if (RRK(EN_KEYPAD_RIGHT))  { return_to_status(); quick_feedback(); }
            }
            else
          #endif
          {
            #if HAS_LCD_MENU
                   if (RRK(EN_KEYPAD_UP))     encoderPosition -= ENCODER_PULSES_PER_STEP;
              else if (RRK(EN_KEYPAD_DOWN))   encoderPosition += ENCODER_PULSES_PER_STEP;
              else if (RRK(EN_KEYPAD_LEFT))   { MenuItem_back::action(); quick_feedback(); }
              else if (RRK(EN_KEYPAD_RIGHT))  encoderPosition = 0;
            #else
                   if (RRK(EN_KEYPAD_UP)   || RRK(EN_KEYPAD_LEFT))  encoderPosition -= ENCODER_PULSES_PER_STEP;
              else if (RRK(EN_KEYPAD_DOWN) || RRK(EN_KEYPAD_RIGHT)) encoderPosition += ENCODER_PULSES_PER_STEP;
            #endif
          }
        #endif
        next_button_update_ms = millis() + ADC_MIN_KEY_DELAY;
        return true;
      }

    #else // !ADC_KEYPAD

      static uint8_t keypad_debounce = 0;

      if (!RRK( EN_KEYPAD_F1    | EN_KEYPAD_F2
              | EN_KEYPAD_F3    | EN_KEYPAD_DOWN
              | EN_KEYPAD_RIGHT | EN_KEYPAD_MIDDLE
              | EN_KEYPAD_UP    | EN_KEYPAD_LEFT )
      ) {
        if (keypad_debounce > 0) keypad_debounce--;
      }
      else if (!keypad_debounce) {
        keypad_debounce = 2;

        const bool homed = mechanics.isHomedAll();

        #if HAS_LCD_MENU

          if (RRK(EN_KEYPAD_MIDDLE))  goto_screen(menu_move);

          #if !MECH(DELTA) && Z_HOME_DIR == -1
            if (RRK(EN_KEYPAD_F2))    _reprapworld_keypad_move(Z_AXIS,  1);
          #endif

          if (homed) {
            #if MECH(DELTA) || Z_HOME_DIR != -1
              if (RRK(EN_KEYPAD_F2))  _reprapworld_keypad_move(Z_AXIS,  1);
            #endif
            if (RRK(EN_KEYPAD_F3))    _reprapworld_keypad_move(Z_AXIS, -1);
            if (RRK(EN_KEYPAD_LEFT))  _reprapworld_keypad_move(X_AXIS, -1);
            if (RRK(EN_KEYPAD_RIGHT)) _reprapworld_keypad_move(X_AXIS,  1);
            if (RRK(EN_KEYPAD_DOWN))  _reprapworld_keypad_move(Y_AXIS,  1);
            if (RRK(EN_KEYPAD_UP))    _reprapworld_keypad_move(Y_AXIS, -1);
          }

        #endif // HAS_LCD_MENU

        if (!homed && RRK(EN_KEYPAD_F1)) commands.enqueue_and_echo_P(PSTR("G28"));
        return true;
      }

    #endif // !ADC_KEYPAD

    return false;
  }

#endif // REPRAPWORLD_KEYPAD

/**
 * Status Screen
 *
 * This is very display-dependent, so the lcd implementation draws this.
 */

#if ENABLED(LCD_PROGRESS_BAR)
  millis_t LcdUI::progress_bar_ms = 0;
  #if PROGRESS_MSG_EXPIRE > 0
    millis_t LcdUI::expire_status_ms = 0;
  #endif
#endif

void LcdUI::status_screen() {

  #if HAS_LCD_MENU
    encoder_direction_normal();
    ENCODER_RATE_MULTIPLY(false);
  #endif

  #if ENABLED(LCD_PROGRESS_BAR)

    //
    // HD44780 implements the following message blinking and
    // message expiration because Status Line and Progress Bar
    // share the same line on the display.
    //

    #if DISABLED(PROGRESS_MSG_ONCE) || (PROGRESS_MSG_EXPIRE > 0)
      millis_t ms = millis();
    #endif

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
            status_message[0] = '\0';
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

  #if HAS_LCD_MENU

    if (use_click()) {
      #if (HAS_LCD_FILAMENT_SENSOR && ENABLED(SDSUPPORT)) || HAS_LCD_POWER_SENSOR
        previous_status_ms = millis();  // Show status message for 5s
      #endif
      goto_screen(menu_main);
      init_lcd(); // May revive the LCD if static electricity killed it
      return;
    }

  #endif // HAS_LCD_MENU

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

  draw_status_screen();

}

void LcdUI::kill_screen(PGM_P lcd_msg) {
  init();
  set_alert_status_P(lcd_msg);
  draw_kill_screen();
}

void LcdUI::quick_feedback(const bool clear_buttons/*=true*/) {

  #if HAS_LCD_MENU
    refresh();
  #endif

  #if HAS_ENCODER_ACTION
    if (clear_buttons) buttons = 0;
    next_button_update_ms = millis() + 500;
  #else
    UNUSED(clear_buttons);
  #endif

  // Buzz and wait. The delay is needed for buttons to settle!
  sound.playTone(LCD_FEEDBACK_FREQUENCY_DURATION_MS, LCD_FEEDBACK_FREQUENCY_HZ);

  #if HAS_LCD_MENU
    #if ENABLED(LCD_USE_I2C_BUZZER)
      HAL::delayMilliseconds(10);
    #endif
  #endif
}

////////////////////////////////////////////
/////////////// Manual Move ////////////////
////////////////////////////////////////////

#if HAS_LCD_MENU

  extern bool no_reentry; // Flag to prevent recursion into menu handlers

  int8_t manual_move_axis = (int8_t)NO_AXIS;
  millis_t manual_move_start_time = 0;

  #if IS_KINEMATIC
    bool LcdUI::processing_manual_move = false;
    float manual_move_offset = 0;
  #endif

  #if E_MANUAL > 1
    int8_t LcdUI::manual_move_e_index = 0;
  #endif

  /**
   * If the most recent manual move hasn't been fed to the planner yet,
   * and the planner can accept one, send a move immediately.
   */
  void LcdUI::manage_manual_move() {

    if (processing_manual_move) return;

    if (manual_move_axis != (int8_t)NO_AXIS && ELAPSED(millis(), manual_move_start_time) && !planner.is_full()) {

      #if IS_KINEMATIC

        const float old_feedrate = mechanics.feedrate_mm_s;
        mechanics.feedrate_mm_s = MMM_TO_MMS(manual_feedrate_mm_m[manual_move_axis]);

        #if EXTRUDERS > 1
          const int8_t old_extruder = tools.active_extruder;
          if (manual_move_axis == E_AXIS) tools.active_extruder = manual_move_e_index;
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

#endif // HAS_LCD_MENU

/**
 * Update the LCD, read encoder buttons, etc.
 *   - Read button states
 *   - Check the SD Card slot state
 *   - Act on RepRap World keypad input
 *   - Update the encoder position
 *   - Apply acceleration to the encoder position
 *   - Do refresh(LCDVIEW_CALL_REDRAW_NOW) on controller events
 *   - Reset the Info Screen timeout if there's any input
 *   - Update status indicators, if any
 *
 *   Run the current LCD menu handler callback function:
 *   - Call the handler only if lcdDrawUpdate != LCDVIEW_NONE
 *   - Before calling the handler, LCDVIEW_CALL_NO_REDRAW => LCDVIEW_NONE
 *   - Call the menu handler. Menu handlers should do the following:
 *     - If a value changes, set lcdDrawUpdate to LCDVIEW_REDRAW_NOW and draw the value
 *       (Encoder events automatically set lcdDrawUpdate for you.)
 *     - if (should_draw()) { redraw }
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

LCDViewActionEnum LcdUI::lcdDrawUpdate = LCDVIEW_CLEAR_CALL_REDRAW;

bool LcdUI::detected() {
  return
    #if (ENABLED(LCD_I2C_TYPE_MCP23017) || ENABLED(LCD_I2C_TYPE_MCP23008)) && defined(DETECT_DEVICE)
      lcd.LcdDetected() == 1
    #else
      true
    #endif
  ;
}

void LcdUI::update() {

  static uint16_t max_display_update_time = 0;
  static millis_t next_lcd_update_ms;

  #if HAS_LCD_MENU

    #if LCD_TIMEOUT_TO_STATUS
      static millis_t return_to_status_ms = 0;
    #endif

    // Handle any queued Move Axis motion
    manage_manual_move();

    // Update button states for button_pressed(), etc.
    // If the state changes the next update may be delayed 300-500ms.
    update_buttons();

    // If the action button is pressed...
    static bool wait_for_unclick; // = 0
    if (!external_control && button_pressed()) {
      if (!wait_for_unclick) {                                  // If not waiting for a debounce release:
        wait_for_unclick = true;                                //  - Set debounce flag to ignore continous clicks
        lcd_clicked = !printer.isWaitForUser() && !no_reentry;  //  - Keep the click if not waiting for a user-click
        printer.setWaitForUser(false);                          //  - Any click clears wait for user
        quick_feedback();                                       //  - Always make a click sound
      }
    }
    else wait_for_unclick = false;

    #if BUTTON_EXISTS(BACK)
      if (LCD_BACK_CLICKED()) {
        quick_feedback();
        goto_previous_screen();
      }
    #endif

  #endif // HAS_LCD_MENU

  #if HAS_SD_SUPPORT && PIN_EXISTS(SD_DETECT)

    const uint8_t sd_status = (uint8_t)IS_SD_INSERTED();
    if (sd_status != lcd_sd_status && detected()) {

      uint8_t old_sd_status = lcd_sd_status; // prevent re-entry to this block!
      lcd_sd_status = sd_status;

      if (sd_status) {
        printer.safe_delay(500);  // Some boards need a delay to get settled
        card.mount();
        if (old_sd_status == 2)
          card.beginautostart();  // Initial boot
        else
          set_status_P(PSTR(MSG_SD_INSERTED));
      }
      else {
        card.unmount();
        if (old_sd_status != 2) set_status_P(PSTR(MSG_SD_REMOVED));
      }

      refresh();
      init_lcd(); // May revive the LCD if static electricity killed it
    }

  #endif // HAS_SD_SUPPORT && SD_DETECT_PIN

  const millis_t ms = millis();
  if (ELAPSED(ms, next_lcd_update_ms)
    #if HAS_GRAPHICAL_LCD
      || drawing_screen
    #endif
  ) {

    next_lcd_update_ms = ms + LCD_UPDATE_INTERVAL;

    #if ENABLED(LCD_HAS_STATUS_INDICATORS)
      update_indicators();
    #endif

    #if HAS_ENCODER_ACTION

      #if HAS_SLOW_BUTTONS
        slow_buttons = read_slow_buttons(); // Buttons that take too long to read in interrupt context
      #endif

      #if ENABLED(REPRAPWORLD_KEYPAD)

        if (handle_keypad()) {
          #if HAS_LCD_MENU && LCD_TIMEOUT_TO_STATUS
            return_to_status_ms = ms + LCD_TIMEOUT_TO_STATUS;
          #endif
        }

      #endif

      const float abs_diff = ABS(encoderDiff);
      const bool encoderPastThreshold = (abs_diff >= (ENCODER_PULSES_PER_STEP));
      if (encoderPastThreshold || lcd_clicked) {
        if (encoderPastThreshold) {

          #if HAS_LCD_MENU && ENABLED(ENCODER_RATE_MULTIPLIER)

            int32_t encoderMultiplier = 1;

            if (encoderRateMultiplierEnabled) {
              const float encoderMovementSteps = abs_diff / (ENCODER_PULSES_PER_STEP);

              if (lastEncoderMovementMillis) {
                // Note that the rate is always calculated between two passes through the
                // loop and that the abs of the encoderDiff value is tracked.
                const float encoderStepRate = encoderMovementSteps / float(ms - lastEncoderMovementMillis) * 1000;

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

          encoderPosition += (encoderDiff * encoderMultiplier) / (ENCODER_PULSES_PER_STEP);
          encoderDiff = 0;
        }
        #if HAS_LCD_MENU && LCD_TIMEOUT_TO_STATUS
          return_to_status_ms = ms + LCD_TIMEOUT_TO_STATUS;
        #endif
        refresh(LCDVIEW_REDRAW_NOW);
      }

    #endif

    // This runs every ~100ms when idling often enough.
    // Instead of tracking changes just redraw the Status Screen once per second.
    if (on_status_screen() && !status_update_delay--) {
      status_update_delay = 9
        #if HAS_GRAPHICAL_LCD
          + 3
        #endif
      ;
      max_display_update_time--;
      refresh(LCDVIEW_REDRAW_NOW);
    }

    #if HAS_LCD_MENU && ENABLED(SCROLL_LONG_FILENAMES)
      // If scrolling of long file names is enabled and we are in the sd card menu,
      // cause a refresh to occur until all the text has scrolled into view.
      if (currentScreen == menu_sdcard && filename_scroll_pos < filename_scroll_max && !status_update_delay--) {
        status_update_delay = 6;
        refresh(LCDVIEW_REDRAW_NOW);
        filename_scroll_pos++;
        #if LCD_TIMEOUT_TO_STATUS
          return_to_status_ms = ms + LCD_TIMEOUT_TO_STATUS;
        #endif
      }
    #endif

    // then we want to use 1/2 of the time only.
    uint16_t bbr2 = planner.block_buffer_runtime() >> 1;

    if ((should_draw() || drawing_screen) && (!bbr2 || bbr2 > max_display_update_time)) {

      // Change state of drawing flag between screen updates
      if (!drawing_screen) switch (lcdDrawUpdate) {
        case LCDVIEW_CALL_NO_REDRAW:
          refresh(LCDVIEW_NONE);
          break;
        case LCDVIEW_CLEAR_CALL_REDRAW:
        case LCDVIEW_CALL_REDRAW_NEXT:
          refresh(LCDVIEW_REDRAW_NOW);
        case LCDVIEW_REDRAW_NOW:        // set above, or by a handler through LCDVIEW_CALL_REDRAW_NEXT
        case LCDVIEW_NONE:
          break;
      } // switch

      #if HAS_ADC_BUTTONS
        keypad_buttons = 0;
      #endif

      #if HAS_GRAPHICAL_LCD

        if (!drawing_screen) {                // If not already drawing pages
          u8g.firstPage();                    // Start the first page
          drawing_screen = first_page = true; // Flag as drawing pages
        }
        set_font(FONT_MENU);                  // Setup font for every page draw
        u8g.setColorIndex(1);                 // And reset the color
        run_current_screen();                 // Draw and process the current screen
        first_page = false;

        // The screen handler can clear drawing_screen for an action that changes the screen.
        // If still drawing and there's another page, update max-time and return now.
        // The nextPage will already be set up on the next call.
        if (drawing_screen && (drawing_screen = u8g.nextPage())) {
          NOLESS(max_display_update_time, millis() - ms);
          return;
        }

      #else

        run_current_screen();

      #endif

      #if HAS_LCD_MENU
        lcd_clicked = false;
      #endif

      // Keeping track of the longest time for an individual LCD update.
      // Used to do screen throttling when the planner starts to fill up.
      NOLESS(max_display_update_time, millis() - ms);
    }

    #if HAS_LCD_MENU && LCD_TIMEOUT_TO_STATUS
      // Return to Status Screen after a timeout
      if (on_status_screen() || defer_return_to_status)
        return_to_status_ms = ms + LCD_TIMEOUT_TO_STATUS;
      else if (ELAPSED(ms, return_to_status_ms))
        return_to_status();
    #endif

    // Change state of drawing flag between screen updates
    if (!drawing_screen) switch (lcdDrawUpdate) {
      case LCDVIEW_CLEAR_CALL_REDRAW:
        clear_lcd(); break;
      case LCDVIEW_REDRAW_NOW:
        refresh(LCDVIEW_NONE);
      case LCDVIEW_NONE:
      case LCDVIEW_CALL_REDRAW_NEXT:
      case LCDVIEW_CALL_NO_REDRAW:
      default: break;
    } // switch

  } // ELAPSED(ms, next_lcd_update_ms)
}

#if HAS_ADC_BUTTONS

  typedef struct {
    uint16_t ADCKeyValueMin, ADCKeyValueMax;
    uint8_t  ADCKeyNo;
  } _stADCKeypadTable_;

  static const _stADCKeypadTable_ stADCKeyTable[] PROGMEM = {
    // VALUE_MIN, VALUE_MAX, KEY
    { 250, 256, BLEN_KEYPAD_F1 + 1      },  // F1
    { 250, 256, BLEN_KEYPAD_F2 + 1      },  // F2
    { 250, 256, BLEN_KEYPAD_F3 + 1      },  // F3
    {  18,  32, BLEN_KEYPAD_LEFT + 1    },  // LEFT
    { 118, 138, BLEN_KEYPAD_RIGHT + 1   },  // RIGHT
    {  34,  54, BLEN_KEYPAD_UP + 1      },  // UP
    { 166, 180, BLEN_KEYPAD_DOWN + 1    },  // DOWN
    {  70,  90, BLEN_KEYPAD_MIDDLE + 1  },  // ENTER
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

#if HAS_ENCODER_ACTION

  #if DISABLED(ADC_KEYPAD) && (ENABLED(REPRAPWORLD_KEYPAD) || !HAS_DIGITAL_BUTTONS)

    /**
     * Setup Rotary Encoder Bit Values (for two pin encoders to indicate movement)
     * These values are independent of which pins are used for EN_A and EN_B indications
     * The rotary encoder part is also independent to the chipset used for the LCD
     */
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

  #endif

  /**
   * Read encoder buttons from the hardware registers
   * Warning: This function is called from interrupt context!
   */
  void LcdUI::update_buttons() {
    const millis_t now = millis();
    if (ELAPSED(now, next_button_update_ms)) {

      #if HAS_DIGITAL_BUTTONS

        #if BUTTON_EXISTS(EN1) || BUTTON_EXISTS(EN2) || BUTTON_EXISTS(ENC) || BUTTON_EXISTS(BACK)

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

        #else

          constexpr uint8_t newbutton = 0;

        #endif

        //
        // Directional buttons
        //
        #if LCD_HAS_DIRECTIONAL_BUTTONS

          const int8_t pulses = (ENCODER_PULSES_PER_STEP) * encoderDirection;

          if (false) {
            // for the else-ifs below
          }
          #if BUTTON_EXISTS(UP)
            else if (BUTTON_PRESSED(UP)) {
              encoderDiff = (ENCODER_STEPS_PER_MENU_ITEM) * pulses;
              next_button_update_ms = now + 300;
            }
          #endif
          #if BUTTON_EXISTS(DWN)
            else if (BUTTON_PRESSED(DWN)) {
              encoderDiff = -(ENCODER_STEPS_PER_MENU_ITEM) * pulses;
              next_button_update_ms = now + 300;
            }
          #endif
          #if BUTTON_EXISTS(LFT)
            else if (BUTTON_PRESSED(LFT)) {
              encoderDiff = -pulses;
              next_button_update_ms = now + 300;
            }
          #endif
          #if BUTTON_EXISTS(RT)
            else if (BUTTON_PRESSED(RT)) {
              encoderDiff = pulses;
              next_button_update_ms = now + 300;
            }
          #endif

        #endif // LCD_HAS_DIRECTIONAL_BUTTONS

        buttons = newbutton
          #if HAS_SLOW_BUTTONS
            | slow_buttons
          #endif
        ;

      #elif HAS_ADC_BUTTONS

        buttons = 0;
        if (keypad_buttons == 0) {
          const uint8_t b = get_ADC_keyValue();
          if (WITHIN(b, 1, 8)) keypad_buttons = _BV(b - 1);
        }

      #endif

      #if HAS_SHIFT_ENCODER

        GET_SHIFT_BUTTON_STATES(
          #if ENABLED(REPRAPWORLD_KEYPAD)
            keypad_buttons
          #else
            buttons
          #endif
        );

      #endif

    } // next_button_update_ms

    #if HAS_ENCODER_WHEEL
      static uint8_t lastEncoderBits;

      #define encrot0 0
      #define encrot1 2
      #define encrot2 3
      #define encrot3 1

      // Manage encoder rotation
      #define ENCODER_SPIN(_E1, _E2) switch (lastEncoderBits) { case _E1: encoderDiff += encoderDirection; break; case _E2: encoderDiff -= encoderDirection; }

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
        if (external_control) {
          #if ENABLED(AUTO_BED_LEVELING_UBL)
            ubl.encoder_diff = encoderDiff;   // Make encoder rotation available to UBL G29 mesh editing.
          #endif
          encoderDiff = 0;                    // Hide the encoder event from the current screen handler.
        }
        lastEncoderBits = enc;
      }

    #endif // HAS_ENCODER_WHEEL
  }

  bool LcdUI::button_pressed() { return BUTTON_CLICK(); }

  #if HAS_SLOW_BUTTONS

    uint8_t LcdUI::read_slow_buttons() {
      #if ENABLED(LCD_I2C_TYPE_MCP23017)
        // Reading these buttons this is likely to be too slow to call inside interrupt context
        // so they are called during normal lcdui.update
        uint8_t slow_bits = lcd.readButtons() << B_I2C_BTN_OFFSET;
        #if ENABLED(LCD_I2C_VIKI)
          if ((slow_bits & (B_MI | B_RI)) && PENDING(millis(), next_button_update_ms)) // LCD clicked
            slow_bits &= ~(B_MI | B_RI); // Disable LCD clicked buttons if screen is updated
        #endif // LCD_I2C_VIKI
        return slow_bits;
      #endif // LCD_I2C_TYPE_MCP23017
    }

  #endif // LCD_HAS_SLOW_BUTTONS

#endif // HAS_ENCODER_ACTION

////////////////////////////////////////////
/////////////// Status Line ////////////////
////////////////////////////////////////////

void LcdUI::finishstatus(const bool persist) {

  #if !(ENABLED(LCD_PROGRESS_BAR) && (PROGRESS_MSG_EXPIRE > 0))
    UNUSED(persist);
  #endif

  #if ENABLED(LCD_PROGRESS_BAR)
    progress_bar_ms = millis();
    #if PROGRESS_MSG_EXPIRE > 0
      expire_status_ms = persist ? 0 : progress_bar_ms + PROGRESS_MSG_EXPIRE;
    #endif
  #endif

  #if (HAS_LCD_FILAMENT_SENSOR && ENABLED(SDSUPPORT)) || HAS_LCD_POWER_SENSOR
    previous_status_ms = millis(); // Show status message for 5s
  #endif

  #if ENABLED(STATUS_MESSAGE_SCROLLING)
    status_scroll_offset = 0;
  #endif

  refresh();
}

bool LcdUI::has_status() { return (status_message[0] != '\0'); }

void LcdUI::set_status(const char * const message, const bool persist) {
  if (status_message_level > 0) return;

  // Here we have a problem. The message is encoded in UTF8, so
  // arbitrarily cutting it will be a problem. We MUST be sure
  // that there is no cutting in the middle of a multibyte character!

  // Get a pointer to the null terminator
  const char* pend = message + strlen(message);

  //  If length of supplied UTF8 string is greater than
  // our buffer size, start cutting whole UTF8 chars
  while ((pend - message) > MAX_MESSAGE_LENGTH) {
    --pend;
    while (!START_OF_UTF8_CHAR(*pend)) --pend;
  };

  // At this point, we have the proper cut point. Use it
  uint8_t maxLen = pend - message;
  strncpy(status_message, message, maxLen);
  status_message[maxLen] = '\0';

  finishstatus(persist);
}

#include <stdarg.h>

void LcdUI::status_printf_P(const uint8_t level, PGM_P const fmt, ...) {
  if (level < status_message_level) return;
  status_message_level = level;
  va_list args;
  va_start(args, fmt);
  vsnprintf_P(status_message, MAX_MESSAGE_LENGTH, fmt, args);
  va_end(args);
  finishstatus(level > 0);
}

void LcdUI::set_status_P(PGM_P const message, int8_t level/*=0*/) {
  if (level < 0) level = status_message_level = 0;
  if (level < status_message_level) return;
  status_message_level = level;

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
  strncpy_P(status_message, message, maxLen);
  status_message[maxLen] = '\0';

  finishstatus(level > 0);
}

void LcdUI::set_alert_status_P(PGM_P const message) {
  set_status_P(message, 1);
  #if HAS_LCD_MENU
    return_to_status();
  #endif
}

/**
 * Reset the status message
 */
void LcdUI::reset_status() {
  static const char paused[] PROGMEM = MSG_PRINT_PAUSED;
  static const char printing[] PROGMEM = MSG_PRINTING;
  static const char welcome[] PROGMEM = WELCOME_MSG;
  #if ENABLED(SERVICE_TIME_1)
    static const char service1[] PROGMEM = { "> " SERVICE_NAME_1 "!" };
  #endif
  #if ENABLED(SERVICE_TIME_2)
    static const char service2[] PROGMEM = { "> " SERVICE_NAME_2 "!" };
  #endif
  #if ENABLED(SERVICE_TIME_3)
    static const char service3[] PROGMEM = { "> " SERVICE_NAME_3 "!" };
  #endif
  PGM_P msg;
  if (!IS_SD_PRINTING() && print_job_counter.isPaused())
    msg = paused;
  #if HAS_SD_SUPPORT
    else if (IS_SD_PRINTING())
      return lcdui.set_status(card.fileName, true);
  #endif
  else if (print_job_counter.isRunning())
    msg = printing;
  #if ENABLED(SERVICE_TIME_1)
    else if (print_job_counter.needService(1)) msg = service1;
  #endif
  #if ENABLED(SERVICE_TIME_2)
    else if (print_job_counter.needService(2)) msg = service2;
  #endif
  #if ENABLED(SERVICE_TIME_3)
    else if (print_job_counter.needService(3)) msg = service3;
  #endif
  else
    msg = welcome;

  lcdui.set_status_P(msg, -1);
}

/**
 * Print pause, resume and stop
 */
void LcdUI::pause_print() {
  #if HAS_SD_RESTART
    if (restart.enabled && IS_SD_PRINTING()) restart.save_job(true, false);
  #endif

  #if ENABLED(PARK_HEAD_ON_PAUSE)
    lcd_advanced_pause_show_message(ADVANCED_PAUSE_MESSAGE_INIT, ADVANCED_PAUSE_MODE_PAUSE_PRINT, tools.active_extruder);
    commands.enqueue_and_echo_P(PSTR("M25 P"));
  #elif HAS_SD_SUPPORT
    commands.enqueue_and_echo_P(PSTR("M25"));
  #else
    SERIAL_L(REQUESTPAUSE);
  #endif

  planner.synchronize();

}

void LcdUI::resume_print() {
  #if HAS_SD_SUPPORT
    commands.enqueue_and_echo_P(PSTR("M24"));
  #else
    SERIAL_L(REQUESTCONTINUE);
  #endif
}

void LcdUI::stop_print() {
  #if HAS_SD_SUPPORT
    printer.setWaitForHeatUp(false);
    printer.setWaitForUser(false);
    if (IS_SD_PRINTING()) card.setAbortSDprinting(true);
  #endif
  SERIAL_L(REQUESTSTOP);
  set_status_P(PSTR(MSG_PRINT_ABORTED), -1);
  return_to_status();
}

#endif // HAS_SPI_LCD
