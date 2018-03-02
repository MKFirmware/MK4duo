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

#include "../../../../MK4duo.h"

#if ENABLED(SUPPORT_MAX6675)

  #define MAX6675_HEAT_INTERVAL 250u
  #define MAX6675_ERROR_MASK      4
  #define MAX6675_DISCARD_BITS    3

  int16_t TemperatureSensor::read_max6675(const pin_t cs_pin) {

    static millis_t next_max6675_ms = 0;
    static uint16_t max6675_temp = 2000;

    millis_t ms = millis();

    if (PENDING(ms, next_max6675_ms)) return (int)max6675_temp;

    next_max6675_ms = ms + MAX6675_HEAT_INTERVAL;

    #if ENABLED(CPU_32_BIT)
      HAL::spiBegin();
    #else
      CBI(
        #ifdef PRR
          PRR
        #elif defined(PRR0)
          PRR0
        #endif
          , PRSPI);
      SPCR = _BV(MSTR) | _BV(SPE) | _BV(SPR0);
    #endif

    HAL::digitalWrite(cs_pin, LOW); // enable TT_MAX6675

    // ensure 100ns delay - a bit extra is fine
    #if ENABLED(CPU_32_BIT)
      HAL::delayMicroseconds(1);
    #else
      asm("nop"); // 50ns on 20Mhz, 62.5ns on 16Mhz
      asm("nop"); // 50ns on 20Mhz, 62.5ns on 16Mhz
    #endif

    // Read a big-endian temperature value
    max6675_temp = 0;
    for (uint8_t i = sizeof(max6675_temp); i--;) {
      #if ENABLED(CPU_32_BIT)
        max6675_temp |= HAL::spiReceive();
      #else
        SPDR = 0;
        for (;!TEST(SPSR, SPIF););
        max6675_temp |= SPDR;
      #endif
      if (i > 0) max6675_temp <<= 8; // shift left if not the last byte
    }

    HAL::digitalWrite(cs_pin, HIGH); // disable TT_MAX6675

    if (max6675_temp & MAX6675_ERROR_MASK) {
      SERIAL_LM(ER, "MAX6675 Temp measurement error!");
      max6675_temp = 2000; // thermocouple open
    }
    else {
      max6675_temp >>= MAX6675_DISCARD_BITS;
    }

    return (int)max6675_temp;
  }

#endif //HEATER_0_USES_MAX6675

#if ENABLED(SUPPORT_MAX31855)

  #define MAX31855_DISCARD_BITS 18

  int16_t TemperatureSensor::read_max31855(const pin_t cs_pin) {

    uint32_t data = 0;
    int16_t temperature;

    HAL::spiBegin();

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

    HAL::digitalWrite(cs_pin, HIGH); // disable TT_MAX31855

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

float TemperatureSensor::getTemperature() {

  const int16_t s_type      = type,
                adcReading  = raw;

  #if ENABLED(SUPPORT_MAX31855)
    if (s_type == -3)
      return 0.25 * read_max31855(pin);
  #endif
  #if ENABLED(SUPPORT_MAX6675)
    if (s_type == -2)
      return 0.25 * read_max6675(pin);
  #endif
  #if HEATER_USES_AD595
    if (s_type == -1)
      return ((adcReading * (((HAL_VOLTAGE_PIN) * 100.0) / (AD_RANGE))) * ad595_gain) + ad595_offset;
  #endif

  if (WITHIN(s_type, 1, 9)) {
    const int32_t averagedVssaReading = 2 * adcLowOffset,
                  averagedVrefReading = AD_RANGE + 2 * adcHighOffset;

    // Calculate the resistance
    const float denom = (float)(averagedVrefReading - adcReading) - 0.5;
    if (denom <= 0.0) return ABS_ZERO;

    const float resistance = pullupR * ((float)(adcReading - averagedVssaReading) + 0.5) / denom;
    const float logResistance = LOG(resistance);
    const float recipT = shA + shB * logResistance + shC * logResistance * logResistance * logResistance;

    /*
    SERIAL_MV("Debug denom:", denom, 5);
    SERIAL_MV(" resistance:", resistance, 5);
    SERIAL_MV(" logResistance:", logResistance, 5);
    SERIAL_MV(" shA:", shA, 5);
    SERIAL_MV(" shB:", shB, 5);
    SERIAL_MV(" shC:", shC, 5);
    SERIAL_MV(" recipT:", recipT, 5);
    SERIAL_EOL();
    */

    return (recipT > 0.0) ? (1.0 / recipT) + (ABS_ZERO) : 2000.0;
  }

  #if ENABLED(DHT_SENSOR)
    if (s_type == 11)
      return dhtsensor.Temperature;
  #endif

  #if HEATER_USES_AMPLIFIER

    #define PGM_RD_W(x) (short)pgm_read_word(&x)
    static uint8_t  ttbllen_map = COUNT(temptable_amplifier);
    float celsius = 0;
    uint8_t i;

    if (s_type == 20) {
      for (i = 1; i < ttbllen_map; i++) {
        if (PGM_RD_W(temptable_amplifier[i][0]) > adcReading) {
          celsius = PGM_RD_W(temptable_amplifier[i - 1][1]) +
                    (adcReading - PGM_RD_W(temptable_amplifier[i - 1][0])) *
                    (float)(PGM_RD_W(temptable_amplifier[i][1]) - PGM_RD_W(temptable_amplifier[i - 1][1])) /
                    (float)(PGM_RD_W(temptable_amplifier[i][0]) - PGM_RD_W(temptable_amplifier[i - 1][0]));
          break;
        }
      }

      // Overflow: Set to last value in the table
      if (i == ttbllen_map) celsius = PGM_RD_W(temptable_amplifier[i - 1][1]);

      return celsius;
    }

  #endif // HEATER_USES_AMPLIFIER

  if (s_type == 998) return DUMMY_THERMISTOR_998_VALUE;
  if (s_type == 999) return DUMMY_THERMISTOR_999_VALUE;

  return 25;
}

void TemperatureSensor::CalcDerivedParameters() {
	shB = 1.0 / beta;
	const float lnR25 = LOG(r25);
	shA = 1.0 / (25.0 - ABS_ZERO) - shB * lnR25 - shC * lnR25 * lnR25 * lnR25;

  /*
  SERIAL_EMV(" R25:", r25, 2);
  SERIAL_EMV(" lnR25:", lnR25, 2);
  SERIAL_EMV(" shA:", shA, 15);
  SERIAL_EMV(" shB:", shB, 15);
  SERIAL_EMV(" shC:", shC, 15);
  */
}
