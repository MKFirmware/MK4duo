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
#pragma once

/**
 * sensor.h - sensor object
 */

#include "thermistor.h"
#include "pt100.h"

typedef struct {

  public: /** Public Parameters */

    pin_t   pin;
    int16_t type,
            adc_raw,
            adc_low_offset,
            adc_high_offset;
    float   res_25,
            beta,
            pullup_res,
            shA,
            shB,
            shC;

    #if HAS_AD8495 || HAS_AD595
      float ad595_offset,
            ad595_gain;
    #endif

  public: /** Public Function */

    void CalcDerivedParameters() {
      shB = 1.0 / beta;
      const float lnR25 = LOG(res_25);
      shA = 1.0 / (25.0 - ABS_ZERO) - shB * lnR25 - shC * lnR25 * lnR25 * lnR25;
    }

    float getTemperature() {

      #if HAS_MAX6675 || HAS_MAX31855
        if (type == -4 || type == -3)
          return 0.25 * adc_raw;
      #endif
      #if HAS_AD8495
        if (type == -2)
          return (adc_raw * float(AD8495_MAX) / float(AD_RANGE)) * ad595_gain + ad595_offset;
      #endif
      #if HAS_AD595
        if (type == -1)
          return (adc_raw * float(AD595_MAX) / float(AD_RANGE)) * ad595_gain + ad595_offset;
      #endif

      if (WITHIN(type, 1, 9)) {

        // Calculate the resistance
        #if HAS_VREF_MONITOR
          const int32_t adc_mv      = HAL::analog2mv(adc_raw),
                        adc_low     = 2 * adc_low_offset,
                        adc_max     = HAL_VREF + (2 * adc_high_offset);
          const float   resistance  = pullup_res * (float)(adc_mv - adc_low) / (float)(adc_max - adc_mv);
        #else
          const int32_t adc_low = 2 * adc_low_offset,
                        adc_max = AD_RANGE + (2 * adc_high_offset);
          const float   adc_inverse = (float)(adc_max - adc_raw) - 0.5f;
          if (adc_inverse <= 0.0) return ABS_ZERO;
          const float   resistance = pullup_res * ((float)(adc_raw - adc_low) + 0.5f) / adc_inverse;
        #endif

        const float logResistance = LOG(resistance);
        const float recipT = shA + shB * logResistance + shC * logResistance * logResistance * logResistance;

        /*
        SERIAL_MV("Debug adc_inverse:", adc_inverse, 5);
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

      #if HAS_DHT
        if (type == 11)
          return dhtsensor.temperature;
      #endif

      if (type == 20) {
        static uint8_t ttbllen_map = COUNT(pt100);
        uint8_t i = 0,
                j = 0,
                len = ttbllen_map;
        for (;;) {
          j = (i + len) >> 1;
          if (!j) return short(pgm_read_word(&pt100[0][1]));
          if (j == i || j == len) return short(pgm_read_word(&pt100[ttbllen_map - 1][1]));
          short v00 = short(pgm_read_word(&pt100[j - 1][0])),
                v10 = short(pgm_read_word(&pt100[j - 0][0]));
          if (adc_raw < v00) len = j;
          else if (adc_raw > v10) i = j;
          else {
            const short v01 = short(pgm_read_word(&pt100[j - 1][1])),
                        v11 = short(pgm_read_word(&pt100[j - 0][1]));
            return v01 + (adc_raw - v00) * float(v11 - v01) / float(v10 - v00);
          }
        }
      }

      if (type == 998) return DUMMY_THERMISTOR_998_VALUE;
      if (type == 999) return DUMMY_THERMISTOR_999_VALUE;

      return 25;
    }

    bool set_pullup_res(const float value) {
      if (!WITHIN(value, 1, 1000000)) return false;
      pullup_res = value;
      return true;
    }

    bool set_res_25(const float value) {
      if (!WITHIN(value, 1, 10000000)) return false;
      res_25 = value;
      return true;
    }

    bool set_beta(const float value) {
      if (!WITHIN(value, 1, 1000000)) return false;
      beta = value;
      return true;
    }

    bool set_shC(const float value) {
      if (!WITHIN(value, -0.01f, 0.01f)) return false;
      shC = value;
      return true;
    }

    bool set_LowOffset(const int16_t value) {
      if (!WITHIN(value, -1000, 1000)) return false;
      adc_low_offset = value;
      return true;
    }

    bool set_HighOffset(const int16_t value) {
      if (!WITHIN(value, -1000, 1000)) return false;
      adc_high_offset = value;
      return true;
    }

    #if HAS_MAX6675

      #define MAX6675_HEAT_INTERVAL 250u
      #define MAX6675_ERROR_MASK      4
      #define MAX6675_DISCARD_BITS    3

      int16_t read_max6675() {

        static short_timer_t next_max6675_timer(millis());
        static uint16_t max6675_temp = 2000;

        if (next_max6675_timer.pending(MAX6675_HEAT_INTERVAL))
          return int16_t(max6675_temp);

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

        HAL::digitalWrite(pin, LOW); // enable TT_MAX6675

        // ensure 100ns delay
        HAL::delayNanoseconds(100);

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

        HAL::digitalWrite(pin, HIGH); // disable TT_MAX6675

        if (max6675_temp & MAX6675_ERROR_MASK) {
          SERIAL_LM(ER, "MAX6675 Temp measurement error!");
          max6675_temp = 2000; // thermocouple open
        }
        else {
          max6675_temp >>= MAX6675_DISCARD_BITS;
        }

        return int16_t(max6675_temp);
      }

    #endif // HAS_MAX6675

    #if HAS_MAX31855

      #define MAX31855_HEAT_INTERVAL 250u
      #define MAX31855_DISCARD_BITS 18

      int16_t read_max31855() {

        static short_timer_t next_max31855_timer(millis());
        static uint16_t last_max31855_temp  = 2000;

        uint32_t data = 0;

        if (next_max31855_timer.pending(MAX31855_HEAT_INTERVAL))
          return int16_t(last_max31855_temp);

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

        HAL::digitalWrite(pin, LOW); // enable TT_MAX31855

        // ensure 100ns delay
        HAL::delayNanoseconds(100);

        for (uint16_t byte = 0; byte < 4; byte++) {
          data <<= 8;
          data |= HAL::spiReceive();
        }

        HAL::digitalWrite(pin, HIGH); // disable TT_MAX31855

        // Process temp
        if (data & 0x00010000)
          return 20000; // Some form of error.
        else {
          data = data >> MAX31855_DISCARD_BITS;
          last_max31855_temp = data & 0x00001FFF;

          if (data & 0x00002000) {
            data = ~data;
            last_max31855_temp = -1 * ((data & 0x00001FFF) + 1);
          }
        }

        return int16_t(last_max31855_temp);
      }

    #endif // HAS_MAX6675

} sensor_data_t;
