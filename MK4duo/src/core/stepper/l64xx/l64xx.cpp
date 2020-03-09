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
 * l64xx.cpp
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#include "../../../../MK4duo.h"

#if HAS_L64XX

L64XX_Manager l64xxManager;

/** Public Parameters */
l64xx_flag_t L64XX_Manager::flag;

L64XX_Manager::L64XX_shadow_t L64XX_Manager::shadow;

/** Public Function */
void L64XX_Manager::create_l64() {

  #define L64_MODEL_DEFINE(ST)    do{ if (!driver[ST##_DRV]->l64)   { driver[ST##_DRV]->l64   = new MKL64XX(ST##_DRV, L6470_CHAIN_SS_PIN);    driver[ST##_DRV]->l64->set_chain_info(ST##_DRV, -1);      }}while(0)
  #define L64_MODEL_DEFINE_E(ST)  do{ if (!driver.e[ST##_DRV]->l64) { driver.e[ST##_DRV]->l64 = new MKL64XX(ST##_DRV+7, L6470_CHAIN_SS_PIN);  driver.e[ST##_DRV]->l64->set_chain_info(ST##_DRV+7, -1);  }}while(0)

  ENABLE_RESET_L64XX_CHIPS(LOW);
  DELAY_US(100);
  ENABLE_RESET_L64XX_CHIPS(HIGH);
  DELAY_US(1000);

  #if AXIS_HAS_L64XX(X)
    L64_MODEL_DEFINE(X);
  #endif
  #if AXIS_HAS_L64XX(Y)
    L64_MODEL_DEFINE(Y);
  #endif
  #if AXIS_HAS_L64XX(Z)
    L64_MODEL_DEFINE(Z);
  #endif
  #if AXIS_HAS_L64XX(E0)
    L64_MODEL_DEFINE_E(E0);
  #endif
  #if AXIS_HAS_L64XX(E1)
    L64_MODEL_DEFINE_E(E1);
  #endif
  #if AXIS_HAS_L64XX(E2)
    L64_MODEL_DEFINE_E(E2);
  #endif
  #if AXIS_HAS_L64XX(E3)
    L64_MODEL_DEFINE_E(E3);
  #endif
  #if AXIS_HAS_L64XX(E4)
    L64_MODEL_DEFINE_E(E4);
  #endif
  #if AXIS_HAS_L64XX(E5)
    L64_MODEL_DEFINE_E(E5);
  #endif

  spi_init();

}

void L64XX_Manager::factory_parameters() {

  constexpr uint16_t  l64_stepper_current[]         = { X_CURRENT, Y_CURRENT, Z_CURRENT,
                                                        X_CURRENT, Y_CURRENT, Z_CURRENT, Z_CURRENT },
                      l64_stepper_current_e[]       = { E0_CURRENT, E1_CURRENT, E2_CURRENT, E3_CURRENT, E4_CURRENT, E5_CURRENT },
                      l64_stepper_microstep[]       = { X_MICROSTEPS, Y_MICROSTEPS, Z_MICROSTEPS,
                                                        X_MICROSTEPS, Y_MICROSTEPS, Z_MICROSTEPS, Z_MICROSTEPS },
                      l64_stepper_microstep_e[]     = { E0_MICROSTEPS, E1_MICROSTEPS, E2_MICROSTEPS, E3_MICROSTEPS, E4_MICROSTEPS, E5_MICROSTEPS };

  constexpr uint32_t  l64_stepper_stallcurrent[]    = { X_STALLCURRENT, Y_STALLCURRENT, Z_STALLCURRENT,
                                                        X_STALLCURRENT, Y_STALLCURRENT, Z_STALLCURRENT, Z_STALLCURRENT },
                      l64_stepper_stallcurrent_e[]  = { E0_STALLCURRENT, E1_STALLCURRENT, E2_STALLCURRENT,
                                                        E3_STALLCURRENT, E4_STALLCURRENT, E5_STALLCURRENT };

  LOOP_DRV_ALL_XYZ() {
    Driver* drv = driver[d];
    if (drv && drv->l64)
      init_chip(drv->l64, l64_stepper_microstep[d], l64_stepper_current[d], l64_stepper_stallcurrent[d], 127, 1);
  }

  LOOP_DRV_EXT() {
    Driver* drv = driver.e[d];
    if (drv && drv->l64)
      init_chip(drv->l64, l64_stepper_microstep_e[d], l64_stepper_current_e[d], l64_stepper_stallcurrent_e[d], 127, 1);
  }

}

uint16_t L64XX_Manager::get_stepper_status(Driver* drv) {

  shadow.STATUS_AXIS_RAW                  = drv->l64->getStatus();
  shadow.STATUS_AXIS                      = shadow.STATUS_AXIS_RAW;
  shadow.STATUS_AXIS_LAYOUT               = drv->l64->L6470_status_layout;
  shadow.AXIS_OCD_TH_MAX                  = drv->l64->OCD_TH_MAX;
  shadow.AXIS_STALL_TH_MAX                = drv->l64->STALL_TH_MAX;
  shadow.AXIS_OCD_CURRENT_CONSTANT_INV    = drv->l64->OCD_CURRENT_CONSTANT_INV;
  shadow.AXIS_STALL_CURRENT_CONSTANT_INV  = drv->l64->STALL_CURRENT_CONSTANT_INV;
  shadow.L6470_AXIS_CONFIG                = drv->l64->L64XX_CONFIG;
  shadow.L6470_AXIS_STATUS                = drv->l64->L64XX_STATUS;
  shadow.STATUS_AXIS_OCD                  = drv->l64->STATUS_OCD;
  shadow.STATUS_AXIS_SCK_MOD              = drv->l64->STATUS_SCK_MOD;
  shadow.STATUS_AXIS_STEP_LOSS_A          = drv->l64->STATUS_STEP_LOSS_A;
  shadow.STATUS_AXIS_STEP_LOSS_B          = drv->l64->STATUS_STEP_LOSS_B;
  shadow.STATUS_AXIS_TH_SD                = drv->l64->STATUS_TH_SD;
  shadow.STATUS_AXIS_TH_WRN               = drv->l64->STATUS_TH_WRN;
  shadow.STATUS_AXIS_UVLO                 = drv->l64->STATUS_UVLO;
  shadow.STATUS_AXIS_WRONG_CMD            = drv->l64->STATUS_WRONG_CMD;
  shadow.STATUS_AXIS_CMD_ERR              = drv->l64->STATUS_CMD_ERR;
  shadow.STATUS_AXIS_NOTPERF_CMD          = drv->l64->STATUS_NOTPERF_CMD;

  switch (shadow.STATUS_AXIS_LAYOUT) {
    case L6470_STATUS_LAYOUT: {   // L6470
      shadow.L6470_ERROR_MASK       = shadow.STATUS_AXIS_UVLO | shadow.STATUS_AXIS_TH_WRN | shadow.STATUS_AXIS_TH_SD | shadow.STATUS_AXIS_OCD | shadow.STATUS_AXIS_STEP_LOSS_A | shadow.STATUS_AXIS_STEP_LOSS_B;
      shadow.STATUS_AXIS           ^= (shadow.STATUS_AXIS_WRONG_CMD | shadow.STATUS_AXIS_NOTPERF_CMD);
      break;
    }
    case L6474_STATUS_LAYOUT: {   // L6474
      shadow.L6470_ERROR_MASK       = shadow.STATUS_AXIS_UVLO | shadow.STATUS_AXIS_TH_WRN | shadow.STATUS_AXIS_TH_SD | shadow.STATUS_AXIS_OCD ;
      shadow.STATUS_AXIS           ^= (shadow.STATUS_AXIS_WRONG_CMD | shadow.STATUS_AXIS_NOTPERF_CMD);
      break;
    }
    case L6480_STATUS_LAYOUT: {   // L6480
      shadow.L6470_ERROR_MASK       = shadow.STATUS_AXIS_UVLO | shadow.STATUS_AXIS_TH_WRN | shadow.STATUS_AXIS_TH_SD | shadow.STATUS_AXIS_OCD | shadow.STATUS_AXIS_STEP_LOSS_A | shadow.STATUS_AXIS_STEP_LOSS_B;
      shadow.STATUS_AXIS           ^= (shadow.STATUS_AXIS_CMD_ERR | shadow.STATUS_AXIS_TH_WRN | shadow.STATUS_AXIS_TH_SD);
      break;
    }
  }

  return shadow.STATUS_AXIS;

}

void L64XX_Manager::monitor_update(Driver* drv) {
  if (flag.spi_abort) return;
  const L64XX_shadow_t &sh = shadow;
  get_stepper_status(drv);
  uint16_t status = sh.STATUS_AXIS;
  uint8_t kval_hold, tval;
  char  temp_buf[120],
        *p = temp_buf;

  drv->l64->driver_status = status;
  uint16_t _status = ~status;

  if (status == 0 || status == 0xFFFF) {
    if (drv->l64->com_counter == 0) {
      drv->l64->com_counter++;
      print_driver_err(drv, PSTR(" - communications lost\n"));
    }
    else {
      drv->l64->com_counter++;
      if (drv->l64->com_counter > 240) {
        drv->l64->com_counter = 1;
        print_driver_err(drv, PSTR(" - still no communications\n"));
      }
    }
  }
  else {
    if (drv->l64->com_counter) {
      drv->l64->com_counter = 0;
      print_driver_err(drv, PSTR(" - communications re-established\n.. setting all drivers to default values\n"));
      factory_parameters();
    }
    else {
      // no com problems - do the usual checks
      if (_status & sh.L6470_ERROR_MASK) {
        print_driver_err(drv);

        if (status & STATUS_HIZ) {
          drv->l64->is_hi_Z = true;
          p += sprintf_P(p, PSTR("%cIS SHUT DOWN"), ' ');
          if (_status & sh.STATUS_AXIS_TH_WRN) {
            p += sprintf_P(p, PSTR("%cdue to over temperature"), ' ');
            drv->l64->is_ot = true;
            if (sh.STATUS_AXIS_LAYOUT == L6474_STATUS_LAYOUT) {
              tval = drv->l64->GetParam(L6474_TVAL) - 2 * KVAL_HOLD_STEP_DOWN;
              drv->l64->SetParam(L6474_TVAL, tval);
              p += sprintf_P(p, PSTR(" - TVAL reduced by %d to %d mA"), uint16_t (2 * KVAL_HOLD_STEP_DOWN * sh.AXIS_STALL_CURRENT_CONSTANT_INV), uint16_t ((tval + 1) * sh.AXIS_STALL_CURRENT_CONSTANT_INV));
            }
            else {
              kval_hold = drv->l64->GetParam(L6470_KVAL_HOLD) - 2 * KVAL_HOLD_STEP_DOWN;
              drv->l64->SetParam(L6470_KVAL_HOLD, kval_hold);
              p += sprintf_P(p, PSTR(" - KVAL_HOLD reduced by %d to %d"), 2 * KVAL_HOLD_STEP_DOWN, kval_hold);
            }
          }
          else
            drv->l64->is_ot = false;
        }
        else {
          drv->l64->is_hi_Z = false;

          if (_status & sh.STATUS_AXIS_TH_WRN) {
            drv->l64->is_otw = true;
            drv->l64->otw_counter++;
            kval_hold = drv->l64->GetParam(L6470_KVAL_HOLD);
            if (drv->l64->otw_counter > 4) {
              drv->l64->otw_counter = 0;
              drv->l64->is_otw = true;
              if (sh.STATUS_AXIS_LAYOUT == L6474_STATUS_LAYOUT) {
                tval = drv->l64->GetParam(L6474_TVAL) - KVAL_HOLD_STEP_DOWN;
                drv->l64->SetParam(L6474_TVAL, tval);
                p += sprintf_P(p, PSTR(" - TVAL reduced by %d to %d mA"), uint16_t (KVAL_HOLD_STEP_DOWN * sh.AXIS_STALL_CURRENT_CONSTANT_INV), uint16_t ((tval + 1) * sh.AXIS_STALL_CURRENT_CONSTANT_INV));
              }
              else {
                kval_hold = drv->l64->GetParam(L6470_KVAL_HOLD) - KVAL_HOLD_STEP_DOWN;
                drv->l64->SetParam(L6470_KVAL_HOLD, kval_hold);
                p += sprintf_P(p, PSTR(" - KVAL_HOLD reduced by %d to %d"), KVAL_HOLD_STEP_DOWN, kval_hold);
              }
            }
            else if (drv->l64->otw_counter)
              p += sprintf_P(p, PSTR("%c- thermal warning"), ' ');
          }
        }

        #if ENABLED(L6470_CHITCHAT)
          if (_status & sh.STATUS_AXIS_OCD)
            p += sprintf_P(p, PSTR("%c  over current"), ' ');

          if (_status & (sh.STATUS_AXIS_STEP_LOSS_A | sh.STATUS_AXIS_STEP_LOSS_B))
            p += sprintf_P(p, PSTR("%c  stall"), ' ');

          if (_status & sh.STATUS_AXIS_UVLO)
            p += sprintf_P(p, PSTR("%c  under voltage lock out"), ' ');

          p += sprintf_P(p, PSTR("%c\n"), ' ');
        #endif

        DEBUG_ET(temp_buf);
      }
      else {
        drv->l64->is_ot = false;
        drv->l64->otw_counter = 0;
        drv->l64->is_otw = false;
      }

    }
  }
}

void L64XX_Manager::monitor_driver() {

  static short_timer_t next_cot_timer(millis());

  if (next_cot_timer.expired(500)) {
    if (!flag.monitor_paused) {
      flag.spi_active = true;
      LOOP_DRV_XYZ() {
        if (driver[d] && driver[d]->l64) monitor_update(driver[d]);
      }
      LOOP_DRV_EXT() {
        if (driver.e[d] && driver.e[d]->l64) monitor_update(driver.e[d]);
      }
      DEBUG_EOL();
      flag.spi_active = false;
      flag.spi_abort = false;
    }
  }

}

void L64XX_Manager::say_axis(Driver* drv, const uint8_t label/*=true*/) {
  if (label) SERIAL_MSG("AXIS:");
  SERIAL_CHR(' ');
  drv->printLabel();
  SERIAL_CHR(' ');
}

/**
 * Spi function
 */
void L64XX_Manager::spi_init() {
  OUT_WRITE(L6470_CHAIN_SS_PIN, HIGH);
  OUT_WRITE(L6470_CHAIN_SCK_PIN, HIGH);
  OUT_WRITE(L6470_CHAIN_MOSI_PIN, HIGH);
  SET_INPUT(L6470_CHAIN_MISO_PIN);

  #if PIN_EXISTS(L6470_BUSY)
    SET_INPUT(L6470_BUSY_PIN);
  #endif

  OUT_WRITE(L6470_CHAIN_MOSI_PIN, HIGH);
}

inline uint8_t L6470_SpiTransfer_Mode_0(uint8_t b) {
  for (uint8_t bits = 8; bits--;) {
    WRITE(L6470_CHAIN_MOSI_PIN, b & 0x80);
    b <<= 1;

    WRITE(L6470_CHAIN_SCK_PIN, HIGH);
    DELAY_NS(125);

    b |= (READ(L6470_CHAIN_MISO_PIN) != 0);

    WRITE(L6470_CHAIN_SCK_PIN, LOW);
    DELAY_NS(125);
  }
  return b;
}

inline uint8_t L6470_SpiTransfer_Mode_3(uint8_t b) {
  for (uint8_t bits = 8; bits--;) {
    WRITE(L6470_CHAIN_SCK_PIN, LOW);
    WRITE(L6470_CHAIN_MOSI_PIN, b & 0x80);
    DELAY_NS(125);
    WRITE(L6470_CHAIN_SCK_PIN, HIGH);
    DELAY_NS(125);
    b <<= 1;
    b |= (READ(L6470_CHAIN_MISO_PIN) != 0);
  }
  DELAY_NS(125);
  return b;
}

void L64XX_Manager::transfer(uint8_t L64XX_buf[], const uint8_t length) {
  if (flag.spi_active) {
    WRITE(L6470_CHAIN_SS_PIN, HIGH);
    DELAY_US(1);
  }
  WRITE(L6470_CHAIN_SS_PIN, LOW);
  for (uint8_t i = length; i >= 1; i--)
    L6470_SpiTransfer_Mode_3(uint8_t(L64XX_buf[i]));
  WRITE(L6470_CHAIN_SS_PIN, HIGH);
}

uint8_t L64XX_Manager::transfer_single(uint8_t data, int16_t ss_pin) {
  // First device in chain has data sent last
  HAL::digitalWrite(ss_pin, LOW);

  DISABLE_ISRS(); // Disable interrupts during SPI transfer (can't allow partial command to chips)
  const uint8_t data_out = L6470_SpiTransfer_Mode_3(data);
  ENABLE_ISRS();  // Enable interrupts

  HAL::digitalWrite(ss_pin, HIGH);
  return data_out;
}

uint8_t L64XX_Manager::transfer_chain(uint8_t data, int16_t ss_pin, uint8_t chain_position) {
  uint8_t data_out = 0;
  HAL::digitalWrite(ss_pin, LOW);

  for (uint8_t i = L64XX::chain[0]; !flag.spi_abort && i >= 1; i--) {
    DISABLE_ISRS();
    const uint8_t temp = L6470_SpiTransfer_Mode_3(uint8_t(i == chain_position ? data : dSPIN_NOP));
    ENABLE_ISRS();
    if (i == chain_position) data_out = temp;
  }

  HAL::digitalWrite(ss_pin, HIGH);
  return data_out;
}

/** Private Function */
void L64XX_Manager::init_chip(L64XX* st, const int ms, const int oc, const int sc, const int mv, const int slew_rate) {
  st->set_handlers(l64xxManager.spi_init, l64xxManager.transfer_single, l64xxManager.transfer_chain);
  switch (st->L6470_status_layout) {
    case L6470_STATUS_LAYOUT: {
      st->resetDev();
      st->softFree();
      st->SetParam(st->L64XX_CONFIG, CONFIG_PWM_DIV_1 | CONFIG_PWM_MUL_2 | CONFIG_OC_SD_DISABLE | CONFIG_VS_COMP_DISABLE | CONFIG_SW_HARD_STOP | CONFIG_INT_16MHZ);
      st->SetParam(L6470_KVAL_RUN, 0xFF);
      st->SetParam(L6470_KVAL_ACC, 0xFF);
      st->SetParam(L6470_KVAL_DEC, 0xFF);
      st->setMicroSteps(ms);
      st->setOverCurrent(oc);
      st->setStallCurrent(sc);
      st->SetParam(L6470_KVAL_HOLD, mv);
      st->SetParam(L6470_ABS_POS, 0);
      uint32_t config_temp = st->GetParam(st->L64XX_CONFIG);
      config_temp &= ~CONFIG_POW_SR;
      switch (slew_rate) {
        case 0: st->SetParam(st->L64XX_CONFIG, config_temp | CONFIG_SR_75V_us); break;
        default:
        case 1: st->SetParam(st->L64XX_CONFIG, config_temp | CONFIG_SR_110V_us); break;
        case 3:
        case 2: st->SetParam(st->L64XX_CONFIG, config_temp | CONFIG_SR_260V_us); break;
      }
      st->getStatus();
      st->getStatus();
      break;
    }

    case L6474_STATUS_LAYOUT: {
      st->free();
      st->setMicroSteps(ms);
      st->setOverCurrent(oc);
      st->setTVALCurrent(sc);
      st->SetParam(L6470_ABS_POS, 0);
      uint32_t config_temp = st->GetParam(st->L64XX_CONFIG);
      config_temp &= ~CONFIG_POW_SR & ~CONFIG_EN_TQREG;  // clear out slew rate and set current to be controlled by TVAL register
      switch (slew_rate) {
        case 0: st->SetParam(st->L64XX_CONFIG, config_temp | CONFIG_SR_75V_us); break;
        default:
        case 1: st->SetParam(st->L64XX_CONFIG, config_temp | CONFIG_SR_110V_us); break;
        case 3:
        case 2: st->SetParam(st->L64XX_CONFIG, config_temp | CONFIG_SR_260V_us); break;
      }
      st->getStatus();
      st->getStatus();
      break;
    }

    case L6480_STATUS_LAYOUT: {
      st->resetDev();
      st->softFree();
      st->SetParam(st->L64XX_CONFIG, CONFIG_PWM_DIV_1 | CONFIG_PWM_MUL_2 | CONFIG_OC_SD_DISABLE | CONFIG_VS_COMP_DISABLE | CONFIG_SW_HARD_STOP | CONFIG_INT_16MHZ);
      st->SetParam(L6470_KVAL_RUN, 0xFF);
      st->SetParam(L6470_KVAL_ACC, 0xFF);
      st->SetParam(L6470_KVAL_DEC, 0xFF);
      st->setMicroSteps(ms);
      st->setOverCurrent(oc);
      st->setStallCurrent(sc);
      st->SetParam(+-L6470_KVAL_HOLD, mv);
      st->SetParam(L6470_ABS_POS, 0);
      st->SetParam(st->L64XX_CONFIG,(st->GetParam(st->L64XX_CONFIG) | PWR_VCC_7_5V));
      st->getStatus();     // must clear out status bits before can set slew rate
      st->getStatus();
      switch (slew_rate) {
        case 0: st->SetParam(L6470_GATECFG1, CONFIG1_SR_220V_us); st->SetParam(L6470_GATECFG2, CONFIG2_SR_220V_us); break;
        default:
        case 1: st->SetParam(L6470_GATECFG1, CONFIG1_SR_400V_us); st->SetParam(L6470_GATECFG2, CONFIG2_SR_400V_us); break;
        case 2: st->SetParam(L6470_GATECFG1, CONFIG1_SR_520V_us); st->SetParam(L6470_GATECFG2, CONFIG2_SR_520V_us); break;
        case 3: st->SetParam(L6470_GATECFG1, CONFIG1_SR_980V_us); st->SetParam(L6470_GATECFG2, CONFIG2_SR_980V_us); break;
      }
      break;
    }
  }
}

void L64XX_Manager::print_driver_err(Driver* drv, const char * const err/*=nullptr*/) {
  DEBUG_MSG("Stepper ");
  drv->debugLabel();
  if (err) DEBUG_TXT(err);
}

#endif // HAS_L64XX
