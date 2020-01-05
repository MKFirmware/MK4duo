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
 * fanmanager.cpp
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"
#include "sanitycheck.h"

FanManager fanManager;

/** Public Parameters */
fans_data_t FanManager::data;

#if DISABLED(SOFTWARE_PDM)
  uint8_t FanManager::pwm_soft_count = 0;
#endif

/** Public Function */
void FanManager::init() { LOOP_FAN() if (fans[f]) fans[f]->init(); }

void FanManager::spin() { LOOP_FAN() if (fans[f]) fans[f]->spin(); }

void FanManager::create_object() {
  LOOP_FAN() {
    if (!fans[f]) {
      fans[f] = new Fan();
      SERIAL_LMV(ECHO, "Create Fan", int(f));
      fans_factory_parameters(f);
    }
  }
}

void FanManager::factory_parameters() {
  data.fans       = FAN_COUNT;
  data.frequency  = FAN_PWM_FREQUENCY;
  create_object();
  LOOP_FAN() { if (fans[f]) fans_factory_parameters(f); }
}

void FanManager::change_number_fan(const uint8_t f) {
  if (data.fans < f) {
    data.fans = f;
    create_object();
  }
  else if (data.fans > f) {
    for (uint8_t ff = f; ff < MAX_FAN; ff++) {
      if (fans[ff]) {
        delete (fans[ff]);
        fans[ff] = nullptr;
        SERIAL_LMV(ECHO, "Delete Fan", int(ff));
      }
    }
    data.fans = f;
  }
}

void FanManager::set_output_pwm() {
  LOOP_FAN() {
    if (fans[f]) {
      if (fans[f]->kickstart) fans[f]->kickstart--;
      fans[f]->set_output_pwm();
    }
  }

  #if DISABLED(SOFTWARE_PDM)
    pwm_soft_count += SOFT_PWM_STEP;
  #endif
}

void FanManager::print_parameters() { LOOP_FAN() if (fans[f]) print_M106(f); }

void FanManager::report_speed() {
  #if HAS_FAN
    LOOP_FAN() {
      SERIAL_MV(" F", int(f));
      SERIAL_MV(":", fans[f]->speed);
    }
  #endif
}

void FanManager::print_M106(const uint8_t f) {

  bool found_auto = false;

  Fan* act = fans[f];

  SERIAL_SM(CFG, "Fans: P<Fan> U<Pin> L<Min Speed> X<Max Speed> I<Hardware Inverted 0-1> H<Auto mode> T<Trig Temp>");
  if (act->data.ID == 0) SERIAL_MSG(" F<Freq>");
  SERIAL_EOL();
  SERIAL_SMV(CFG, "  M106 P", (int)act->data.ID);
  SERIAL_MV(" U", act->data.pin);
  SERIAL_MV(" L", act->data.speed_limit.min);
  SERIAL_MV(" X", act->data.speed_limit.max);
  if (act->data.ID == 0) SERIAL_MV(" F", data.frequency);
  SERIAL_MV(" I", act->isHWinvert());
  SERIAL_MSG(" H");
  LOOP_HOTEND() {
    if (TEST(act->data.auto_monitor, h)) {
      SERIAL_VAL((int)h);
      SERIAL_MV(" T", act->data.trigger_temperature);
      found_auto = true;
      break;
    }
  }
  if (!found_auto) {
    if (TEST(act->data.auto_monitor, 7))
      SERIAL_CHR('7');
    else
      SERIAL_MSG("-1");
  }
  SERIAL_EOL();
}

bool FanManager::fans_isActive() {
  LOOP_FAN() if (fans[f]->speed > 0) return true;
  return false;
}

bool FanManager::get_target_fan(uint8_t& f) {
  f = parser.seen('P') ? parser.value_byte() : 0;
  if (WITHIN(f, 0 , data.fans - 1)) return true;
  else {
    SERIAL_LM(ER, "Invalid Fan");
    return false;
  }
}

/** Private Function */
void FanManager::fans_factory_parameters(const uint8_t f) {

  constexpr pin_t   fanCh[]   = FANS_CHANNELS;
  constexpr int8_t  fanAuto[] = AUTO_FAN;

  #if ENABLED(TACHOMETRIC)
    constexpr pin_t tacho_temp_pin[] = { TACHO0_PIN, TACHO1_PIN, TACHO2_PIN, TACHO3_PIN, TACHO4_PIN, TACHO5_PIN };
  #endif

  Fan*        act   = fans[f];
  fan_data_t* fdata = &act->data;

  fdata->ID                   = f;
  fdata->pin                  = fanCh[f];
  fdata->speed_limit.min      = FAN_MIN_PWM;
  fdata->speed_limit.max      = FAN_MAX_PWM;
  fdata->trigger_temperature  = HOTEND_AUTO_FAN_TEMPERATURE;
  fdata->auto_monitor         = 0;
  fdata->flag.all             = false;
  act->set_auto_monitor(fanAuto[f]);
  act->setHWinvert(FAN_INVERTED);
  #if ENABLED(TACHOMETRIC)
    fdata->tacho.pin          = tacho_temp_pin[f];
  #endif

  LOOP_HOTEND() {
    if (TEST(fdata->auto_monitor, h)) {
      fdata->speed_limit.min  = HOTEND_AUTO_FAN_MIN_SPEED;
      fdata->speed_limit.max  = HOTEND_AUTO_FAN_SPEED;
    }
  }

  if (TEST(fdata->auto_monitor, 7)) {
    fdata->speed_limit.min    = CONTROLLERFAN_MIN_SPEED;
    fdata->speed_limit.max    = CONTROLLERFAN_SPEED;
  }

}
