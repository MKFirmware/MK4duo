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
 * sensor.cpp - heater object
 */

#include "../../../MK4duo.h"

#if ENABLED(SUPPORT_MAX6675)

  #define MAX6675_ERROR_MASK 4
  #define MAX6675_DISCARD_BITS 3

  int16_t read_max6675(const Pin cs_pin, const int8_t h) {

    static millis_t last_max6675_read[HOTENDS]  = ARRAY_BY_HOTENDS(0);
    static int16_t  max6675_temp[HOTENDS]       = ARRAY_BY_HOTENDS(2000);

    if (HAL::timeInMilliseconds() - last_max6675_read[h] > 230) {

      HAL::spiBegin();
      HAL::spiInit(2);

      HAL::digitalWrite(cs_pin, LOW); // enable TT_MAX6675

      // ensure 100ns delay - a bit extra is fine
      #if ENABLED(ARDUINO_ARCH_SAM)
        HAL::delayMicroseconds(1);
      #else
        asm("nop"); // 50ns on 20Mhz, 62.5ns on 16Mhz
        asm("nop"); // 50ns on 20Mhz, 62.5ns on 16Mhz
      #endif

      max6675_temp[h] = HAL::spiReceive(0);
      max6675_temp[h] <<= 8;
      max6675_temp[h] |= HAL::spiReceive(0);

      HAL::digitalWrite(cs_pin, HIGH); // disable TT_MAX6675
      last_max6675_read[h] = millis();
    }

    if (max6675_temp[h] & MAX6675_ERROR_MASK) {
      SERIAL_LM(ER, "MAX6675 Temp measurement error!");
      max6675_temp[h] = 2000; // thermocouple open
    }
    else {
      max6675_temp[h] >> MAX6675_DISCARD_BITS;
    }

    return max6675_temp[h];
  }

#endif //HEATER_0_USES_MAX6675

#if ENABLED(SUPPORT_MAX31855)

  #define MAX31855_DISCARD_BITS 18

  int16_t read_max31855(const Pin cs_pin) {

    uint32_t data = 0;
    int16_t temperature;

    HAL::spiBegin();
    HAL::spiInit(2);

    HAL::digitalWrite(cs_pin, LOW); // enable TT_MAX31855

    // ensure 100ns delay - a bit extra is fine
    #if ENABLED(ARDUINO_ARCH_SAM)
      HAL::delayMicroseconds(1);
    #else
      asm("nop"); // 50ns on 20Mhz, 62.5ns on 16Mhz
      asm("nop"); // 50ns on 20Mhz, 62.5ns on 16Mhz
    #endif

    for (uint16_t byte = 0; byte < 4; byte++) {
      data <<= 8;
      data |= HAL::spiReceive();
    }

    HAL::digitalWrite(cs_pin, 1); // disable TT_MAX31855

    // Process temp
    if (data & 0x00010000)
      return 20000; // Some form of error.
    else {
      data = data >> MAX31855_DISCARD_BITS;
      temperature = data & 0x00001FFF;

      if (data & 0x00002000) {
        data = ~data;
        temperature = -1 * ((data & 0x00001FFF) + 1);
      }
    }

    return temperature;
  }

#endif

float TemperatureSensor::GetTemperature(const uint8_t h) {

  int16_t type  = this->type;
  int16_t raw   = this->raw;

  #if ENABLED(SUPPORT_MAX31855)
    if (type == -3)
      return read_max31855(this->pin);
  #endif
  #if ENABLED(SUPPORT_MAX6675)
    if (type == -2)
      return read_max6675(this->pin, h);
  #else
    UNUSED(h);
  #endif
  #if HEATER_USES_AD595
    if (type == -1)
      return ((raw * (((HAL_VOLTAGE_PIN) * 100.0) / AD_RANGE)) * this->ad595_gain) + this->ad595_offset;
  #endif
  if (WITHIN(type, 1, 9)) {
    const float denom = (float)(AD_RANGE + (int)this->adcHighOffset - raw) - 0.5;
    if (denom <= 0.0) return ABS_ZERO;

    const float resistance = this->pullupR * ((float)(raw - (int)this->adcLowOffset) + 0.5) / denom;
    const float logResistance = LOG(resistance);
    const float recipT = this->shA + this->shB * logResistance + this->shC * logResistance * logResistance * logResistance;
    return (recipT > 0.0) ? (1.0 / recipT) + ABS_ZERO : 2000.0;
  }
  #if ENABLED(DHT_SENSOR)
    if (type == 11)
      return dhtsensor.readTemperature();
  #endif
  #if HEATER_USES_AMPLIFIER
    #define PGM_RD_W(x) (short)pgm_read_word(&x)
    static uint8_t  ttbllen_map = COUNT(temptable_amplifier);
    float celsius = 0;
    uint8_t i;

    if (type == 20) {
      for (i = 1; i < ttbllen_map; i++) {
        if (PGM_RD_W(temptable_amplifier[i][0]) > raw) {
          celsius = PGM_RD_W(temptable_amplifier[i - 1][1]) +
                    (raw - PGM_RD_W(temptable_amplifier[i - 1][0])) *
                    (float)(PGM_RD_W(temptable_amplifier[i][1]) - PGM_RD_W(temptable_amplifier[i - 1][1])) /
                    (float)(PGM_RD_W(temptable_amplifier[i][0]) - PGM_RD_W(temptable_amplifier[i - 1][0]));
          break;
        }
      }

      // Overflow: Set to last value in the table
      if (i == ttbllen_map) celsius = PGM_RD_W(temptable_amplifier[i - 1][1]);

      return celsius;
    }
  #endif
  if (type == 998) return DUMMY_THERMISTOR_998_VALUE;
  if (type == 999) return DUMMY_THERMISTOR_999_VALUE;

  return 25;
}

void TemperatureSensor::CalcDerivedParameters() {
	this->shB = 1.0 / this->beta;
	const float lnR25 = LOG(this->r25);
	this->shA = 1.0 / (25.0 - ABS_ZERO) - this->shB * lnR25 - this->shC * lnR25 * lnR25 * lnR25;
}
