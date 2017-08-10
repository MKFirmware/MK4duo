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

bool endstop_monitor_flag = false;

#define NAME_FORMAT "%-35s" // one place to specify the format of all the sources of names
                            // "-" left justify, "28" minimum width of name, pad with blanks

#define _PIN_SAY(NAME) { sprintf(buffer, NAME_FORMAT, NAME); SERIAL_TXT(buffer); return true; }
#define PIN_SAY(NAME) if (pin == NAME) _PIN_SAY(#NAME);

#define _ANALOG_PIN_SAY(NAME) { sprintf(buffer, NAME_FORMAT, NAME); SERIAL_TXT(buffer); pin_is_analog = true; return true; }
#define ANALOG_PIN_SAY(NAME) if (pin == analogInputToDigitalPin(NAME)) _ANALOG_PIN_SAY(#NAME);

#if ENABLED(ARDUINO_ARCH_SAM)
  #define IS_ANALOG(P) ((uint8_t)(P) >= analogInputToDigitalPin(0) && (uint8_t)(P) <= analogInputToDigitalPin(MAX_ANALOG_PIN_NUMBER))
#else
  #define IS_ANALOG(P) ((P) >= analogInputToDigitalPin(0) && ((P) <= analogInputToDigitalPin(15) || (P) <= analogInputToDigitalPin(5)))
#endif

#if ENABLED(ARDUINO_ARCH_SAM)
  #define LAST_PIN                        PINS_COUNT // Arduino Due's NUM_DIGITAL_PINS only includes the digital only pins
  #define PIN_TO_BASEREG(pin)             (&(digitalPinToPort(pin)->PIO_PER))
  #define PIN_TO_OSRREG(pin)              (&(digitalPinToPort(pin)->PIO_OSR))  // "0" means it's an input
  #define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
  #define IO_REG_TYPE uint32_t
  #define DIRECT_READ(base, mask)         (((*((base)+15)) & (mask)) ? 1 : 0)
  #define digitalPinToTimer(pin)          digitalPinHasPWM(pin)
#else
  #define LAST_PIN                        NUM_DIGITAL_PINS
  #define PIN_TO_BASEREG(pin)             (portInputRegister(digitalPinToPort(pin)))
  #define PIN_TO_OSRREG(pin)              (portInputRegister(digitalPinToPort(pin)))
  #define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
  #define IO_REG_TYPE uint8_t
  #define DIRECT_READ(base, mask)         (((*(base)) & (mask)) ? 1 : 0)
#endif

IO_REG_TYPE rBit;   // receive pin's ports and bitmask
volatile IO_REG_TYPE *rReg;

int digitalRead_mod(int8_t pin) { // same as digitalRead except the PWM stop section has been removed
  rBit = PIN_TO_BITMASK(pin);     // get receive pin's ports and bitmask
  rReg = PIN_TO_BASEREG(pin);
  return DIRECT_READ(rReg, rBit);
}

// Report pin name for a given fastio digital pin index
static bool report_pin_name(int8_t pin, bool &pin_is_analog) {

  char buffer[30];   // for the sprintf statements
  pin_is_analog = false;   // default to digital pin

  if (IS_ANALOG(pin)) {
    sprintf(buffer, "(A%2d)  ", int(pin - analogInputToDigitalPin(0)));
    SERIAL_TXT(buffer);
  }
  else SERIAL_MSG("       ");

  #if ENABLED(RXD) && RXD >= 0
    if (pin == 0) { sprintf(buffer, NAME_FORMAT, "RXD"); SERIAL_TXT(buffer); return true; }
  #endif

  #if ENABLED(TXD) && TXD >= 0
    if (pin == 1) { sprintf(buffer, NAME_FORMAT, "TXD"); SERIAL_TXT(buffer); return true; }
  #endif

  // Pin list updated from 7 OCT RCBugfix branch
  #if ENABLED(__FD) && __FD >= 0
    PIN_SAY(__FD)
  #endif
  #if ENABLED(__FS) && __FS >= 0
    PIN_SAY(__FS)
  #endif
  #if ENABLED(__GD) && __GD >= 0
    PIN_SAY(__GD)
  #endif
  #if ENABLED(__GS) && __GS >= 0
    PIN_SAY(__GS)
  #endif

  #if PIN_EXISTS(AVR_MISO)
    PIN_SAY(AVR_MISO_PIN);
  #endif
  #if PIN_EXISTS(AVR_MOSI)
    PIN_SAY(AVR_MOSI_PIN);
  #endif
  #if PIN_EXISTS(AVR_SCK)
    PIN_SAY(AVR_SCK_PIN);
  #endif
  #if PIN_EXISTS(AVR_SS)
    PIN_SAY(AVR_SS_PIN);
  #endif
  #if PIN_EXISTS(BEEPER)
    PIN_SAY(BEEPER_PIN);
  #endif
  #if ENABLED(BTN_CENTER) && BTN_CENTER >= 0
    PIN_SAY(BTN_CENTER);
  #endif
  #if ENABLED(BTN_DOWN) && BTN_DOWN >= 0
    PIN_SAY(BTN_DOWN);
  #endif
  #if ENABLED(BTN_DWN) && BTN_DWN >= 0
    PIN_SAY(BTN_DWN);
  #endif
  #if ENABLED(BTN_EN1) && BTN_EN1 >= 0
    PIN_SAY(BTN_EN1);
  #endif
  #if ENABLED(BTN_EN2) && BTN_EN2 >= 0
    PIN_SAY(BTN_EN2);
  #endif
  #if ENABLED(BTN_ENC) && BTN_ENC >= 0
    PIN_SAY(BTN_ENC);
  #endif
  #if ENABLED(BTN_HOME) && BTN_HOME >= 0
    PIN_SAY(BTN_HOME);
  #endif
  #if ENABLED(BTN_LEFT) && BTN_LEFT >= 0
    PIN_SAY(BTN_LEFT);
  #endif
  #if ENABLED(BTN_LFT) && BTN_LFT >= 0
    PIN_SAY(BTN_LFT);
  #endif
  #if ENABLED(BTN_RIGHT) && BTN_RIGHT >= 0
    PIN_SAY(BTN_RIGHT);
  #endif
  #if ENABLED(BTN_RT) && BTN_RT >= 0
    PIN_SAY(BTN_RT);
  #endif
  #if ENABLED(BTN_UP) && BTN_UP >= 0
    PIN_SAY(BTN_UP);
  #endif
  #if PIN_EXISTS(CONTROLLERFAN)
    PIN_SAY(CONTROLLERFAN_PIN);
  #endif
  #if PIN_EXISTS(DAC_DISABLE)
    PIN_SAY(DAC_DISABLE_PIN);
  #endif
  #if ENABLED(DAC_STEPPER_GAIN) && DAC_STEPPER_GAIN >= 0
    PIN_SAY(DAC_STEPPER_GAIN);
  #endif
  #if ENABLED(DAC_STEPPER_VREF) && DAC_STEPPER_VREF >= 0
    PIN_SAY(DAC_STEPPER_VREF);
  #endif
  #if PIN_EXISTS(DEBUG)
    PIN_SAY(DEBUG_PIN);
  #endif
  #if PIN_EXISTS(DIGIPOTSS)
    PIN_SAY(DIGIPOTSS_PIN);
  #endif
  #if ENABLED(DOGLCD_A0) && DOGLCD_A0 >= 0
    PIN_SAY(DOGLCD_A0);
  #endif
  #if ENABLED(DOGLCD_CS) && DOGLCD_CS >= 0
    PIN_SAY(DOGLCD_CS);
  #endif
  #if ENABLED(DOGLCD_MOSI) && DOGLCD_MOSI >= 0
    PIN_SAY(DOGLCD_MOSI);
  #endif
  #if ENABLED(DOGLCD_SCK) && DOGLCD_SCK >= 0
    PIN_SAY(DOGLCD_SCK);
  #endif
  #if PIN_EXISTS(E0_ATT)
    PIN_SAY(E0_ATT_PIN);
  #endif
  #if PIN_EXISTS(H0_AUTO_FAN)
    PIN_SAY(H0_AUTO_FAN_PIN);
  #endif
  #if PIN_EXISTS(H1_AUTO_FAN)
    PIN_SAY(H1_AUTO_FAN_PIN);
  #endif
  #if PIN_EXISTS(H2_AUTO_FAN)
    PIN_SAY(H2_AUTO_FAN_PIN);
  #endif
  #if PIN_EXISTS(H3_AUTO_FAN)
    PIN_SAY(H3_AUTO_FAN_PIN);
  #endif
  #if PIN_EXISTS(E0_DIR)
    PIN_SAY(E0_DIR_PIN);
  #endif
  #if PIN_EXISTS(E0_ENABLE)
    PIN_SAY(E0_ENABLE_PIN);
  #endif
  #if PIN_EXISTS(E0_MS1)
    PIN_SAY(E0_MS1_PIN);
  #endif
  #if PIN_EXISTS(E0_MS2)
    PIN_SAY(E0_MS2_PIN);
  #endif
  #if PIN_EXISTS(E0_STEP)
    PIN_SAY(E0_STEP_PIN);
  #endif
  #if PIN_EXISTS(E0_CS)
    PIN_SAY(E0_CS_PIN)
  #endif
  #if PIN_EXISTS(SOL0)
    PIN_SAY(SOL0_PIN)
  #endif
  #if PIN_EXISTS(E0_ENC)
    PIN_SAY(E0_ENC_PIN)
  #endif
  #if PIN_EXISTS(E1_DIR)
    PIN_SAY(E1_DIR_PIN);
  #endif
  #if PIN_EXISTS(E1_ENABLE)
    PIN_SAY(E1_ENABLE_PIN);
  #endif
  #if PIN_EXISTS(E1_MS1)
    PIN_SAY(E1_MS1_PIN);
  #endif
  #if PIN_EXISTS(E1_MS2)
    PIN_SAY(E1_MS2_PIN);
  #endif
  #if PIN_EXISTS(E1_STEP)
    PIN_SAY(E1_STEP_PIN);
  #endif
  #if PIN_EXISTS(E1_CS)
    PIN_SAY(E1_CS_PIN)
  #endif
  #if PIN_EXISTS(SOL1)
    PIN_SAY(SOL1_PIN)
  #endif
  #if PIN_EXISTS(E1_ENC)
    PIN_SAY(E1_ENC_PIN)
  #endif
  #if PIN_EXISTS(E2_DIR)
    PIN_SAY(E2_DIR_PIN);
  #endif
  #if PIN_EXISTS(E2_ENABLE)
    PIN_SAY(E2_ENABLE_PIN);
  #endif
  #if PIN_EXISTS(E2_STEP)
    PIN_SAY(E2_STEP_PIN);
  #endif
  #if PIN_EXISTS(E2_CS)
    PIN_SAY(E2_CS_PIN)
  #endif
  #if PIN_EXISTS(SOL2)
    PIN_SAY(SOL2_PIN)
  #endif
  #if PIN_EXISTS(E2_ENC)
    PIN_SAY(E2_ENC_PIN)
  #endif
  #if PIN_EXISTS(E3_DIR)
    PIN_SAY(E3_DIR_PIN);
  #endif
  #if PIN_EXISTS(E3_ENABLE)
    PIN_SAY(E3_ENABLE_PIN);
  #endif
  #if PIN_EXISTS(E3_STEP)
    PIN_SAY(E3_STEP_PIN);
  #endif
  #if PIN_EXISTS(E3_CS)
    PIN_SAY(E3_CS_PIN)
  #endif
  #if PIN_EXISTS(SOL3)
    PIN_SAY(SOL3_PIN)
  #endif
  #if PIN_EXISTS(E3_ENC)
    PIN_SAY(E3_ENC_PIN)
  #endif
  #if PIN_EXISTS(E4_DIR)
    PIN_SAY(E4_DIR_PIN);
  #endif
  #if PIN_EXISTS(E4_ENABLE)
    PIN_SAY(E4_ENABLE_PIN);
  #endif
  #if PIN_EXISTS(E4_STEP)
    PIN_SAY(E4_STEP_PIN);
  #endif
  #if PIN_EXISTS(E4_CS)
    PIN_SAY(E4_CS_PIN)
  #endif
  #if PIN_EXISTS(SOL4)
    PIN_SAY(SOL4_PIN)
  #endif
  #if PIN_EXISTS(E4_ENC)
    PIN_SAY(E4_ENC_PIN)
  #endif
  #if PIN_EXISTS(E5_DIR)
    PIN_SAY(E5_DIR_PIN)
  #endif
  #if PIN_EXISTS(E5_ENABLE)
    PIN_SAY(E5_ENABLE_PIN)
  #endif
  #if PIN_EXISTS(E5_STEP)
    PIN_SAY(E5_STEP_PIN)
  #endif
  #if PIN_EXISTS(E5_CS)
    PIN_SAY(E5_CS_PIN)
  #endif
  #if PIN_EXISTS(SOL5)
    PIN_SAY(SOL5_PIN)
  #endif
  #if PIN_EXISTS(E5_ENC)
    PIN_SAY(E5_ENC_PIN)
  #endif
  #if ENABLED(encrot1) && encrot1 >= 0
    PIN_SAY(encrot1);
  #endif
  #if ENABLED(encrot2) && encrot2 >= 0
    PIN_SAY(encrot2);
  #endif
  #if ENABLED(encrot3) && encrot3 >= 0
    PIN_SAY(encrot3);
  #endif
  #if ENABLED(EXT_AUX_A0_IO) && EXT_AUX_A0_IO >= 0
    PIN_SAY(EXT_AUX_A0_IO);
  #endif
  #if ENABLED(EXT_AUX_A1) && EXT_AUX_A1 >= 0
    PIN_SAY(EXT_AUX_A1);
  #endif
  #if ENABLED(EXT_AUX_A1_IO) && EXT_AUX_A1_IO >= 0
    PIN_SAY(EXT_AUX_A1_IO);
  #endif
  #if ENABLED(EXT_AUX_A2) && EXT_AUX_A2 >= 0
    PIN_SAY(EXT_AUX_A2);
  #endif
  #if ENABLED(EXT_AUX_A2_IO) && EXT_AUX_A2_IO >= 0
    PIN_SAY(EXT_AUX_A2_IO);
  #endif
  #if ENABLED(EXT_AUX_A3) && EXT_AUX_A3 >= 0
    PIN_SAY(EXT_AUX_A3);
  #endif
  #if ENABLED(EXT_AUX_A3_IO) && EXT_AUX_A3_IO >= 0
    PIN_SAY(EXT_AUX_A3_IO);
  #endif
  #if ENABLED(EXT_AUX_A4) && EXT_AUX_A4 >= 0
    PIN_SAY(EXT_AUX_A4);
  #endif
  #if ENABLED(EXT_AUX_A4_IO) && EXT_AUX_A4_IO >= 0
    PIN_SAY(EXT_AUX_A4_IO);
  #endif
  #if ENABLED(EXT_AUX_PWM_D24) && EXT_AUX_PWM_D24 >= 0
    PIN_SAY(EXT_AUX_PWM_D24);
  #endif
  #if ENABLED(EXT_AUX_RX1_D2) && EXT_AUX_RX1_D2 >= 0
    PIN_SAY(EXT_AUX_RX1_D2);
  #endif
  #if ENABLED(EXT_AUX_SDA_D1) && EXT_AUX_SDA_D1 >= 0
    PIN_SAY(EXT_AUX_SDA_D1);
  #endif
  #if ENABLED(EXT_AUX_TX1_D3) && EXT_AUX_TX1_D3 >= 0
    PIN_SAY(EXT_AUX_TX1_D3);
  #endif

  #if PIN_EXISTS(FAN)
    PIN_SAY(FAN_PIN);
  #endif
  #if PIN_EXISTS(FAN1)
    PIN_SAY(FAN1_PIN);
  #endif
  #if PIN_EXISTS(FAN2)
    PIN_SAY(FAN2_PIN);
  #endif
  #if PIN_EXISTS(FAN3)
    PIN_SAY(FAN3_PIN);
  #endif
  #if PIN_EXISTS(FIL_RUNOUT)
    PIN_SAY(FIL_RUNOUT_PIN);
  #endif
  #if PIN_EXISTS(FILWIDTH)
    ANALOG_PIN_SAY(FILWIDTH_PIN);
  #endif
  #if ENABLED(GEN7_VERSION) && GEN7_VERSION >= 0
    PIN_SAY(GEN7_VERSION);
  #endif
  #if PIN_EXISTS(HEATER_0)
    PIN_SAY(HEATER_0_PIN);
  #endif
  #if PIN_EXISTS(HEATER_1)
    PIN_SAY(HEATER_1_PIN);
  #endif
  #if PIN_EXISTS(HEATER_2)
    PIN_SAY(HEATER_2_PIN);
  #endif
  #if PIN_EXISTS(HEATER_3)
    PIN_SAY(HEATER_3_PIN);
  #endif
  #if PIN_EXISTS(HEATER_BED)
    PIN_SAY(HEATER_BED_PIN);
  #endif
  #if PIN_EXISTS(HEATER_CHAMBER)
    PIN_SAY(HEATER_CHAMBER_PIN);
  #endif
  #if PIN_EXISTS(COOLER)
    PIN_SAY(COOLER_PIN);
  #endif
  #if ENABLED(I2C_SCL) && I2C_SCL >= 0
    PIN_SAY(I2C_SCL);
  #endif
  #if ENABLED(I2C_SDA) && I2C_SDA >= 0
    PIN_SAY(I2C_SDA);
  #endif
  #if PIN_EXISTS(KILL)
    PIN_SAY(KILL_PIN);
  #endif
  #if PIN_EXISTS(LCD_BACKLIGHT)
    PIN_SAY(LCD_BACKLIGHT_PIN);
  #endif
  #if ENABLED(LCD_CONTRAST) && LCD_CONTRAST >= 0
    PIN_SAY(LCD_CONTRAST);
  #endif
  #if ENABLED(LCD_PINS_D4) && LCD_PINS_D4 >= 0
    PIN_SAY(LCD_PINS_D4);
  #endif
  #if ENABLED(LCD_PINS_D5) && LCD_PINS_D5 >= 0
    PIN_SAY(LCD_PINS_D5);
  #endif
  #if ENABLED(LCD_PINS_D6) && LCD_PINS_D6 >= 0
    PIN_SAY(LCD_PINS_D6);
  #endif
  #if ENABLED(LCD_PINS_D7) && LCD_PINS_D7 >= 0
    PIN_SAY(LCD_PINS_D7);
  #endif
  #if ENABLED(LCD_PINS_ENABLE) && LCD_PINS_ENABLE >= 0
    PIN_SAY(LCD_PINS_ENABLE);
  #endif
  #if ENABLED(LCD_PINS_RS) && LCD_PINS_RS >= 0
    PIN_SAY(LCD_PINS_RS);
  #endif
  #if ENABLED(LCD_SDSS) && LCD_SDSS >= 0
    PIN_SAY(LCD_SDSS);
  #endif
  #if PIN_EXISTS(LED)
    PIN_SAY(LED_PIN);
  #endif
  #if PIN_EXISTS(CASE_LIGHT)
    PIN_SAY(CASE_LIGHT_PIN);
  #endif
  #if PIN_EXISTS(MAIN_VOLTAGE_MEASURE)
    PIN_SAY(MAIN_VOLTAGE_MEASURE_PIN);
  #endif
  #if ENABLED(MAX6675_SS) && MAX6675_SS >= 0
    PIN_SAY(MAX6675_SS);
  #endif
  #if PIN_EXISTS(MISO)
    PIN_SAY(MISO_PIN);
  #endif
  #if PIN_EXISTS(MOSFET_D)
    PIN_SAY(MOSFET_D_PIN);
  #endif
  #if PIN_EXISTS(MOSI)
    PIN_SAY(MOSI_PIN);
  #endif
  #if PIN_EXISTS(MOTOR_CURRENT_PWM_E)
    PIN_SAY(MOTOR_CURRENT_PWM_E_PIN);
  #endif
  #if PIN_EXISTS(MOTOR_CURRENT_PWM_XY)
    PIN_SAY(MOTOR_CURRENT_PWM_XY_PIN);
  #endif
  #if PIN_EXISTS(MOTOR_CURRENT_PWM_Z)
    PIN_SAY(MOTOR_CURRENT_PWM_Z_PIN);
  #endif
  #if ENABLED(NUM_TLCS) && NUM_TLCS >= 0
    PIN_SAY(NUM_TLCS);
  #endif
  #if PIN_EXISTS(PHOTOGRAPH)
    PIN_SAY(PHOTOGRAPH_PIN);
  #endif
  #if PIN_EXISTS(PS_ON)
    PIN_SAY(PS_ON_PIN);
  #endif
  #if PIN_EXISTS(RAMPS_D10)
    PIN_SAY(RAMPS_D10_PIN);
  #endif
  #if PIN_EXISTS(RAMPS_D8)
    PIN_SAY(RAMPS_D8_PIN);
  #endif
  #if PIN_EXISTS(RAMPS_D9)
    PIN_SAY(RAMPS_D9_PIN);
  #endif
  #if PIN_EXISTS(RX_ENABLE)
    PIN_SAY(RX_ENABLE_PIN);
  #endif
  #if PIN_EXISTS(SAFETY_TRIGGERED)
    PIN_SAY(SAFETY_TRIGGERED_PIN);
  #endif
  #if PIN_EXISTS(SCK)
    PIN_SAY(SCK_PIN);
  #endif
  #if PIN_EXISTS(SD_DETECT)
    PIN_SAY(SD_DETECT_PIN);
  #endif
  #if ENABLED(SDPOWER) && SDPOWER >= 0
    PIN_SAY(SDPOWER);
  #endif
  #if PIN_EXISTS(SS_PIN)
    PIN_SAY(SS_PIN);
  #endif
  #if PIN_EXISTS(SERVO0)
    PIN_SAY(SERVO0_PIN);
  #endif
  #if PIN_EXISTS(SERVO1)
    PIN_SAY(SERVO1_PIN);
  #endif
  #if PIN_EXISTS(SERVO2)
    PIN_SAY(SERVO2_PIN);
  #endif
  #if PIN_EXISTS(SERVO3)
    PIN_SAY(SERVO3_PIN);
  #endif
  #if ENABLED(SHIFT_CLK) && SHIFT_CLK >= 0
    PIN_SAY(SHIFT_CLK);
  #endif
  #if ENABLED(SHIFT_EN) && SHIFT_EN >= 0
    PIN_SAY(SHIFT_EN);
  #endif
  #if ENABLED(SHIFT_LD) && SHIFT_LD >= 0
    PIN_SAY(SHIFT_LD);
  #endif
  #if ENABLED(SHIFT_OUT) && SHIFT_OUT >= 0
    PIN_SAY(SHIFT_OUT);
  #endif
  #if PIN_EXISTS(SLED)
    PIN_SAY(SLED_PIN);
  #endif
  #if PIN_EXISTS(SLEEP_WAKE)
    PIN_SAY(SLEEP_WAKE_PIN);
  #endif
  #if PIN_EXISTS(SOL1)
    PIN_SAY(SOL1_PIN);
  #endif
  #if PIN_EXISTS(SOL2)
    PIN_SAY(SOL2_PIN);
  #endif
  #if PIN_EXISTS(SPINDLE_ENABLE)
    PIN_SAY(SPINDLE_ENABLE_PIN);
  #endif
  #if PIN_EXISTS(SPINDLE_SPEED)
    PIN_SAY(SPINDLE_SPEED_PIN);
  #endif
  #if PIN_EXISTS(SS)
    PIN_SAY(SS_PIN);
  #endif
  #if PIN_EXISTS(STAT_LED_BLUE_PIN)
    PIN_SAY(STAT_LED_BLUE_PIN);
  #endif
  #if PIN_EXISTS(STAT_LED_RED_PIN)
    PIN_SAY(STAT_LED_RED_PIN);
  #endif
  #if PIN_EXISTS(STEPPER_RESET)
    PIN_SAY(STEPPER_RESET_PIN);
  #endif
  #if PIN_EXISTS(SUICIDE)
    PIN_SAY(SUICIDE_PIN);
  #endif

  #if ENABLED(ARDUINO_ARCH_AVR)
    #if ENABLED(TC1) && TC1 >= 0
      ANALOG_PIN_SAY(TC1);
    #endif
    #if ENABLED(TC2) && TC2 >= 0
      ANALOG_PIN_SAY(TC2);
    #endif
  #endif

  #if PIN_EXISTS(TEMP_0)
    ANALOG_PIN_SAY(TEMP_0_PIN);
  #endif
  #if PIN_EXISTS(TEMP_1)
    ANALOG_PIN_SAY(TEMP_1_PIN);
  #endif
  #if PIN_EXISTS(TEMP_2)
    ANALOG_PIN_SAY(TEMP_2_PIN);
  #endif
  #if PIN_EXISTS(TEMP_3)
    ANALOG_PIN_SAY(TEMP_3_PIN);
  #endif
  #if PIN_EXISTS(TEMP_BED)
    ANALOG_PIN_SAY(TEMP_BED_PIN);
  #endif
  #if PIN_EXISTS(TEMP_CHAMBER)
    ANALOG_PIN_SAY(TEMP_CHAMBER_PIN);
  #endif
  #if PIN_EXISTS(TEMP_COOLER)
    ANALOG_PIN_SAY(TEMP_COOLER_PIN);
  #endif
  #if ENABLED(TLC_BLANK_BIT) && TLC_BLANK_BIT >= 0
    PIN_SAY(TLC_BLANK_BIT);
  #endif
  #if PIN_EXISTS(TLC_BLANK)
    PIN_SAY(TLC_BLANK_PIN);
  #endif
  #if ENABLED(TLC_CLOCK_BIT) && TLC_CLOCK_BIT >= 0
    PIN_SAY(TLC_CLOCK_BIT);
  #endif
  #if PIN_EXISTS(TLC_CLOCK)
    PIN_SAY(TLC_CLOCK_PIN);
  #endif
  #if ENABLED(TLC_DATA_BIT) && TLC_DATA_BIT >= 0
    PIN_SAY(TLC_DATA_BIT);
  #endif
  #if PIN_EXISTS(TLC_DATA)
    PIN_SAY(TLC_DATA_PIN);
  #endif
  #if PIN_EXISTS(TLC_XLAT)
    PIN_SAY(TLC_XLAT_PIN);
  #endif
  #if PIN_EXISTS(TX_ENABLE)
    PIN_SAY(TX_ENABLE_PIN);
  #endif
  #if ENABLED(UNUSED_PWM) && UNUSED_PWM >= 0
    PIN_SAY(UNUSED_PWM);
  #endif
  #if PIN_EXISTS(X_ATT)
    PIN_SAY(X_ATT_PIN);
  #endif
  #if PIN_EXISTS(X_DIR)
    PIN_SAY(X_DIR_PIN);
  #endif
  #if PIN_EXISTS(X_ENABLE)
    PIN_SAY(X_ENABLE_PIN);
  #endif
  #if PIN_EXISTS(X_MAX)
    PIN_SAY(X_MAX_PIN);
  #endif
  #if PIN_EXISTS(X_MIN)
    PIN_SAY(X_MIN_PIN);
  #endif
  #if PIN_EXISTS(X_MS1)
    PIN_SAY(X_MS1_PIN);
  #endif
  #if PIN_EXISTS(X_MS2)
    PIN_SAY(X_MS2_PIN);
  #endif
  #if PIN_EXISTS(X_STEP)
    PIN_SAY(X_STEP_PIN);
  #endif
  #if PIN_EXISTS(X_STOP)
    PIN_SAY(X_STOP_PIN);
  #endif
  #if PIN_EXISTS(X2_DIR)
    PIN_SAY(X2_DIR_PIN);
  #endif
  #if PIN_EXISTS(X2_ENABLE)
    PIN_SAY(X2_ENABLE_PIN);
  #endif
  #if PIN_EXISTS(X2_STEP)
    PIN_SAY(X2_STEP_PIN);
  #endif
  #if PIN_EXISTS(Y_ATT)
    PIN_SAY(Y_ATT_PIN);
  #endif
  #if PIN_EXISTS(Y_DIR)
    PIN_SAY(Y_DIR_PIN);
  #endif
  #if PIN_EXISTS(Y_ENABLE)
    PIN_SAY(Y_ENABLE_PIN);
  #endif
  #if PIN_EXISTS(Y_MAX)
    PIN_SAY(Y_MAX_PIN);
  #endif
  #if PIN_EXISTS(Y_MIN)
    PIN_SAY(Y_MIN_PIN);
  #endif
  #if PIN_EXISTS(Y_MS1)
    PIN_SAY(Y_MS1_PIN);
  #endif
  #if PIN_EXISTS(Y_MS2)
    PIN_SAY(Y_MS2_PIN);
  #endif
  #if PIN_EXISTS(Y_STEP)
    PIN_SAY(Y_STEP_PIN);
  #endif
  #if PIN_EXISTS(Y_STOP)
    PIN_SAY(Y_STOP_PIN);
  #endif
  #if PIN_EXISTS(Y2_DIR)
    PIN_SAY(Y2_DIR_PIN);
  #endif
  #if PIN_EXISTS(Y2_ENABLE)
    PIN_SAY(Y2_ENABLE_PIN);
  #endif
  #if PIN_EXISTS(Y2_STEP)
    PIN_SAY(Y2_STEP_PIN);
  #endif
  #if PIN_EXISTS(Z_ATT)
    PIN_SAY(Z_ATT_PIN);
  #endif
  #if PIN_EXISTS(Z_DIR)
    PIN_SAY(Z_DIR_PIN);
  #endif
  #if PIN_EXISTS(Z_ENABLE)
    PIN_SAY(Z_ENABLE_PIN);
  #endif
  #if PIN_EXISTS(Z_MAX)
    PIN_SAY(Z_MAX_PIN);
  #endif
  #if PIN_EXISTS(Z_MIN)
    PIN_SAY(Z_MIN_PIN);
  #endif
  #if PIN_EXISTS(Z_PROBE)
    PIN_SAY(Z_PROBE_PIN);
  #endif
  #if PIN_EXISTS(Z_MS1)
    PIN_SAY(Z_MS1_PIN);
  #endif
  #if PIN_EXISTS(Z_MS2)
    PIN_SAY(Z_MS2_PIN);
  #endif
  #if PIN_EXISTS(Z_STEP)
    PIN_SAY(Z_STEP_PIN);
  #endif
  #if PIN_EXISTS(Z_STOP)
    PIN_SAY(Z_STOP_PIN);
  #endif
  #if PIN_EXISTS(Z2_MIN)
    PIN_SAY(Z2_MIN_PIN)
  #endif
  #if PIN_EXISTS(Z3_MAX)
    PIN_SAY(Z3_MAX_PIN)
  #endif
  #if PIN_EXISTS(Z3_MIN)
    PIN_SAY(Z3_MIN_PIN)
  #endif
  #if PIN_EXISTS(Z4_MAX)
    PIN_SAY(Z4_MAX_PIN)
  #endif
  #if PIN_EXISTS(Z4_MIN)
    PIN_SAY(Z4_MIN_PIN)
  #endif
  #if PIN_EXISTS(Z2_DIR)
    PIN_SAY(Z2_DIR_PIN);
  #endif
  #if PIN_EXISTS(Z2_ENABLE)
    PIN_SAY(Z2_ENABLE_PIN);
  #endif
  #if PIN_EXISTS(Z2_STEP)
    PIN_SAY(Z2_STEP_PIN);
  #endif

  sprintf(buffer, NAME_FORMAT, "<unused> ");
  SERIAL_TXT(buffer);

  return false;
} // report_pin_name

#define PWM_PRINT(V) do{ sprintf(buffer, "PWM:  %4d", V); SERIAL_TXT(buffer); }while(0)
#define PWM_CASE(N) \
  case TIMER##N: \
    if (TCCR##N & (_BV(COM## N ##1) | _BV(COM## N ##0))) { \
      PWM_PRINT(OCR##N); \
      return true; \
    } else return false

/**
 * Print a pin's PWM status.
 * Return true if it's currently a PWM pin.
 */
static bool pwm_status(uint8_t pin) {
  char buffer[20];   // for the sprintf statements

  switch(digitalPinToTimer(pin)) {

    #if ENABLED(TCCR0A) && ENABLED(COM0A1)
      PWM_CASE(0A);
      PWM_CASE(0B);
    #endif

    #if ENABLED(TCCR1A) && ENABLED(COM1A1)
      PWM_CASE(1A);
      PWM_CASE(1B);
      PWM_CASE(1C);
    #endif

    #if ENABLED(TCCR2A) && ENABLED(COM2A1)
      PWM_CASE(2A);
      PWM_CASE(2B);
    #endif

    #if ENABLED(TCCR3A) && ENABLED(COM3A1)
      PWM_CASE(3A);
      PWM_CASE(3B);
      PWM_CASE(3C);
    #endif

    #ifdef TCCR4A
      PWM_CASE(4A);
      PWM_CASE(4B);
      PWM_CASE(4C);
    #endif

    #if ENABLED(TCCR5A) && ENABLED(COM5A1)
      PWM_CASE(5A);
      PWM_CASE(5B);
      PWM_CASE(5C);
    #endif

    case NOT_ON_TIMER:
    default:
      return false;
  }
  SERIAL_MSG("  ");
} // pwm_status

#if DISABLED(ARDUINO_ARCH_SAM)

  #define WGM_MAKE3(N) ((TEST(TCCR##N##B, WGM##N##2) >> 1) | (TCCR##N##A & (_BV(WGM##N##0) | _BV(WGM##N##1))))
  #define WGM_MAKE4(N) (WGM_MAKE3(N) | (TEST(TCCR##N##B, WGM##N##3) >> 1))
  #define TIMER_PREFIX(T,L,N) do{ \
      WGM = WGM_MAKE##N(T); \
      SERIAL_MSG("    TIMER"); \
      SERIAL_MSG(STRINGIFY(T) STRINGIFY(L)); \
      SERIAL_MV("    WGM: ", WGM); \
      SERIAL_MV("    TIMSK" STRINGIFY(T) ": ", TIMSK##T); \
    }while(0)

  #define WGM_TEST1 (WGM == 0 || WGM == 2 || WGM == 4 || WGM == 6)
  #define WGM_TEST2 (WGM == 0 || WGM == 4 || WGM == 12 || WGM == 13)

  static void err_is_counter() {
    SERIAL_EM("   Can't be used as a PWM because of counter mode");
  }
  static void err_is_interrupt() {
    SERIAL_EM("   Can't be used as a PWM because it's being used as an interrupt");
  }
  static void err_prob_interrupt() {
    SERIAL_EM("   Probably can't be used as a PWM because counter/timer is being used as an interrupt");
  }
  static void can_be_used() {
    SERIAL_MSG("   can be used as PWM   ");
  }

  static void pwm_details(uint8_t pin) {

    uint8_t WGM;

    switch(digitalPinToTimer(pin)) {

      #if ENABLED(TCCR0A) && ENABLED(COM0A1)
        case TIMER0A:
          TIMER_PREFIX(0,A,3);
          if (WGM_TEST1) err_is_counter();
          else if (TEST(TIMSK0, OCIE0A)) err_is_interrupt();
          else if (TEST(TIMSK0, TOIE0)) err_prob_interrupt();
          else can_be_used();
          break;
        case TIMER0B:
          TIMER_PREFIX(0,B,3);
          if (WGM_TEST1) err_is_counter();
          else if (TEST(TIMSK0, OCIE0B)) err_is_interrupt();
          else if (TEST(TIMSK0, TOIE0)) err_prob_interrupt();
          else can_be_used();
          break;
      #endif

      #if ENABLED(TCCR1A) && ENABLED(COM1A1)
        case TIMER1A:
          TIMER_PREFIX(1,A,4);
          if (WGM_TEST2) err_is_counter();
          else if (TEST(TIMSK1, OCIE1A)) err_is_interrupt();
          else if (TIMSK1 & (_BV(TOIE1) | _BV(ICIE1))) err_prob_interrupt();
          else can_be_used();
          break;
        case TIMER1B:
          TIMER_PREFIX(1,B,4);
          if (WGM_TEST2) err_is_counter();
          else if (TEST(TIMSK1, OCIE1B)) err_is_interrupt();
          else if (TIMSK1 & (_BV(TOIE1) | _BV(ICIE1))) err_prob_interrupt();
          else can_be_used();
          break;
        case TIMER1C:
          TIMER_PREFIX(1,C,4);
          if (WGM_TEST2) err_is_counter();
          else if (TEST(TIMSK1, OCIE1C)) err_is_interrupt();
          else if (TIMSK1 & (_BV(TOIE1) | _BV(ICIE1))) err_prob_interrupt();
          else can_be_used();
          break;
      #endif

      #if ENABLED(TCCR2A) && ENABLED(COM2A1)
        case TIMER2A:
          TIMER_PREFIX(2,A,3);
          if (WGM_TEST1) err_is_counter();
          else if (TIMSK2 & (_BV(TOIE2) | _BV(OCIE2A))) err_is_interrupt();
          else if (TEST(TIMSK2, TOIE2)) err_prob_interrupt();
          else can_be_used();
          break;
        case TIMER2B:
          TIMER_PREFIX(2,B,3);
          if (WGM_TEST1) err_is_counter();
          else if (TEST(TIMSK2, OCIE2B)) err_is_interrupt();
          else if (TEST(TIMSK2, TOIE2)) err_prob_interrupt();
          else can_be_used();
          break;
      #endif

      #if ENABLED(TCCR3A) && ENABLED(COM3A1)
        case TIMER3A:
          TIMER_PREFIX(3,A,3);
          if (WGM_TEST2) err_is_counter();
          else if (TEST(TIMSK3, OCIE3A)) err_is_interrupt();
          else if (TIMSK3 & (_BV(TOIE3) | _BV(ICIE3))) err_prob_interrupt();
          else can_be_used();
          break;
        case TIMER3B:
          TIMER_PREFIX(3,B,3);
          if (WGM_TEST2) err_is_counter();
          else if (TEST(TIMSK3, OCIE3B)) err_is_interrupt();
          else if (TIMSK3 & (_BV(TOIE3) | _BV(ICIE3))) err_prob_interrupt();
          else can_be_used();
          break;
        case TIMER3C:
          TIMER_PREFIX(3,C,3);
          if (WGM_TEST2) err_is_counter();
          else if (TEST(TIMSK3, OCIE3C)) err_is_interrupt();
          else if (TIMSK3 & (_BV(TOIE3) | _BV(ICIE3))) err_prob_interrupt();
          else can_be_used();
          break;
      #endif

      #ifdef TCCR4A
        case TIMER4A:
          TIMER_PREFIX(4,A,4);
          if (WGM_TEST2) err_is_counter();
          else if (TEST(TIMSK4, OCIE4A)) err_is_interrupt();
          else if (TIMSK4 & (_BV(TOIE4) | _BV(ICIE4))) err_prob_interrupt();
          else can_be_used();
          break;
        case TIMER4B:
          TIMER_PREFIX(4,B,4);
          if (WGM_TEST2) err_is_counter();
          else if (TEST(TIMSK4, OCIE4B)) err_is_interrupt();
          else if (TIMSK4 & (_BV(TOIE4) | _BV(ICIE4))) err_prob_interrupt();
          else can_be_used();
          break;
        case TIMER4C:
          TIMER_PREFIX(4,C,4);
          if (WGM_TEST2) err_is_counter();
          else if (TEST(TIMSK4, OCIE4C)) err_is_interrupt();
          else if (TIMSK4 & (_BV(TOIE4) | _BV(ICIE4))) err_prob_interrupt();
          else can_be_used();
          break;
      #endif

      #if ENABLED(TCCR5A) && ENABLED(COM5A1)
        case TIMER5A:
          TIMER_PREFIX(5,A,4);
          if (WGM_TEST2) err_is_counter();
          else if (TEST(TIMSK5, OCIE5A)) err_is_interrupt();
          else if (TIMSK5 & (_BV(TOIE5) | _BV(ICIE5))) err_prob_interrupt();
          else can_be_used();
          break;
        case TIMER5B:
          TIMER_PREFIX(5,B,4);
          if (WGM_TEST2) err_is_counter();
          else if (TEST(TIMSK5, OCIE5B)) err_is_interrupt();
          else if (TIMSK5 & (_BV(TOIE5) | _BV(ICIE5))) err_prob_interrupt();
          else can_be_used();
          break;
        case TIMER5C:
          TIMER_PREFIX(5,C,4);
          if (WGM_TEST2) err_is_counter();
          else if (TEST(TIMSK5, OCIE5C)) err_is_interrupt();
          else if (TIMSK5 & (_BV(TOIE5) | _BV(ICIE5))) err_prob_interrupt();
          else can_be_used();
          break;
      #endif

      case NOT_ON_TIMER: break;

    }
    SERIAL_MSG("  ");
  }  // pwm_details

#endif

inline void report_pin_state(int8_t pin) {
  SERIAL_VAL((int)pin);
  SERIAL_CHR(' ');
  bool dummy;
  if (report_pin_name(pin, dummy)) {
    if (printer.pin_is_protected(pin))
      SERIAL_MSG(" (protected)");
    else {
      SERIAL_MSG(" = ");
      pinMode(pin, INPUT_PULLUP);
      SERIAL_VAL(digitalRead(pin));
      if (IS_ANALOG(pin)) {
        SERIAL_CHR(' '); SERIAL_CHR('(');
        SERIAL_VAL(analogRead(pin - analogInputToDigitalPin(0)));
        SERIAL_CHR(')');
      }
    }
  }
  SERIAL_EOL();
}

bool get_pinMode(uint8_t pin) {
  rBit = PIN_TO_BITMASK(pin);
  rReg = PIN_TO_OSRREG(pin);

  #if ENABLED(ARDUINO_ARCH_SAM)
    if (*rReg & rBit)
      return true;
    else
      return false;  
  #else
    if (*rReg & rBit)
      return false;
    else
      return true;
  #endif    
}

// pretty report with PWM info
inline void report_pin_state_extended(Pin pin, bool ignore) {

  char buffer[30];   // for the sprintf statements

  // report pin number
  sprintf(buffer, "PIN:% 3d ", pin);
  SERIAL_TXT(buffer);

  // report pin name
  bool analog_pin;
  report_pin_name(pin, analog_pin);

  // report pin state
  if (printer.pin_is_protected(pin) && !ignore)
    SERIAL_MSG("protected ");
  else {
    if (analog_pin) {
      #if DISABLED(ARDUINO_ARCH_SAM)
        sprintf(buffer, "Analog in =% 5d", analogRead(pin - analogInputToDigitalPin(0)));
        SERIAL_TXT(buffer);
      #endif
    }
    else {
      if (!get_pinMode(pin)) {
        pinMode(pin, INPUT_PULLUP);  // make sure input isn't floating
        SERIAL_MT("Input  = ", digitalRead_mod(pin));
      }
      else if (pwm_status(pin)) {
        // do nothing
      }
      else SERIAL_MT("Output = ", digitalRead_mod(pin));
    }
  }

  #if DISABLED(ARDUINO_ARCH_SAM)
    // report PWM capabilities
    pwm_details(pin);
  #endif

  SERIAL_EOL();
}
