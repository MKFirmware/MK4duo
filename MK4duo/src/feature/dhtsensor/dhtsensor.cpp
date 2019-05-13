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

/**
 * dhtsensor.cpp
 *
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"

#if ENABLED(DHT_SENSOR)

#define DHT_TIMEOUT -1

constexpr millis_s  DHTMinimumReadInterval = 2000, // ms
                    DHTMaximumReadTime     = 20;   // ms

DHTSensor dhtsensor;

/** Public Parameters */
dht_data_t DHTSensor::data;

float DHTSensor::Temperature  = 25,
      DHTSensor::Humidity     = 50;

/** Private Parameters */
const millis_l DHTSensor::_maxcycles = microsecondsToClockCycles(1000);
uint8_t DHTSensor::read_data[5] = { 0, 0, 0, 0, 0 };

/** Public Function */
void DHTSensor::init() {
  HAL::pinMode(data.pin, OUTPUT);
}

void DHTSensor::factory_parameters() {
  data.pin  = DHT_DATA_PIN;
  data.type = DHTEnum(DHT_TYPE);
}

void DHTSensor::change_type(const DHTEnum dhtType) {
  switch (dhtType) {
    case DHT11:
      data.type = DHT11;
      break;
    case DHT12:
      data.type = DHT12;
      break;
    case DHT21:
      data.type = DHT21;
      break;
    case DHT22:
      data.type = DHT22;
      break;
    default:
      SERIAL_LM(ER, "Invalid DHT sensor type");
      break;
  }
}

void DHTSensor::print_M305() {
  SERIAL_LM(CFG, "DHT sensor parameters: P<Pin> S<type 11-21-22>:");
  SERIAL_SM(CFG, "  M305 D0");
  SERIAL_MV(" P", data.pin);
  SERIAL_MV(" S", data.type);
  SERIAL_EOL();
}

void DHTSensor::spin() {

  static millis_s min_read_ms = millis();

  if (expired(&min_read_ms, DHTMinimumReadInterval)) {

    // Reset 40 bits of received data to zero.
    ZERO(read_data);

    // Start the reading process
    HAL::pinMode(data.pin, INPUT_PULLUP);
    delay(1);

    // Go into high impedence state to let pull-up raise data line level and
    // start the reading process.
    HAL::pinMode(data.pin, OUTPUT);
    HAL::digitalWrite(data.pin, LOW);
    switch (data.type) {
      case DHT22:
      case DHT21:
        HAL::delayMicroseconds(1100); // data sheet says "at least 1ms"
        break;
      case DHT11:
      default:
        delay(20); // data sheet says at least 18ms, 20ms just to be safe
        break;
    }

    uint32_t cycles[80];

    // End the start signal by setting data line high for 40 microseconds.
    HAL::pinMode(data.pin, INPUT_PULLUP);

    HAL::delayMicroseconds(60); // Delay a bit to let sensor pull data line low.

    // Turn off interrupts temporarily because the next sections
    // are timing critical and we don't want any interruptions.
    InterruptLock lock;

    // First expect a low signal for ~80 microseconds followed by a high signal
    // for ~80 microseconds again.
    if (expectPulse(LOW) == DHT_TIMEOUT) {
      if (printer.debugFeature())
        DEBUG_EM("DHT timeout waiting for start signal low pulse.");
      return;
    }
    if (expectPulse(HIGH) == DHT_TIMEOUT) {
      if (printer.debugFeature())
        DEBUG_EM("DHT timeout waiting for start signal high pulse.");
      return;
    }

    // Now read the 40 bits sent by the sensor.  Each bit is sent as a 50
    // microsecond low pulse followed by a variable length high pulse.  If the
    // high pulse is ~28 microseconds then it's a 0 and if it's ~70 microseconds
    // then it's a 1.  We measure the cycle count of the initial 50us low pulse
    // and use that to compare to the cycle count of the high pulse to determine
    // if the bit is a 0 (high state cycle count < low state cycle count), or a
    // 1 (high state cycle count > low state cycle count). Note that for speed all
    // the pulses are read into a array and then examined in a later step.
    for (int i = 0; i < 80; i += 2) {
      cycles[i]   = expectPulse(LOW);
      cycles[i+1] = expectPulse(HIGH);
    }

    // Inspect pulses and determine which ones are 0 (high state cycle count < low
    // state cycle count), or 1 (high state cycle count > low state cycle count).
    for (int i = 0; i < 40; ++i) {
      uint32_t lowCycles  = cycles[2 * i];
      uint32_t highCycles = cycles[2 * i + 1];
      if ((lowCycles == DHT_TIMEOUT) || (highCycles == DHT_TIMEOUT)) {
        if (printer.debugFeature())
          DEBUG_EM("DHT timeout waiting for pulse.");
        return;
      }
      read_data[i/8] <<= 1;
      // Now compare the low and high cycle times to see if the bit is a 0 or 1.
      if (highCycles > lowCycles) {
        // High cycles are greater than 50us low cycle count, must be a 1.
        read_data[i/8] |= 1;
      }
    }

    // Verify checksum
    if (((read_data[0] + read_data[1] + read_data[2] + read_data[3]) & 0xFF) == read_data[4]) {
      // Generate final results
      Temperature = read_temperature();
      Humidity    = read_humidity();
    }
    else {
      if (printer.debugFeature()) DEBUG_EM("DHT checksum failure!");
    }
  }

}

float DHTSensor::dewPoint() {
  // (1) Saturation Vapor Pressure = ESGG(T)
  const float RATIO = 373.15 / (273.15 + Temperature);
  float RHS = -7.90298 * (RATIO - 1);
  RHS += 5.02808 * log10(RATIO);
  RHS += -1.3816e-7 * (POW(10, (11.344 * (1 - 1/RATIO ))) - 1) ;
  RHS += 8.1328e-3 * (POW(10, (-3.49149 * (RATIO - 1))) - 1) ;
  RHS += log10(1013.246);

  // factor -3 is to adjust units - Vapor Pressure SVP * Humidity
  const float VP = POW(10, RHS - 3) * Humidity;

  // (2) DEWPOINT = F(Vapor Pressure)
  float T = LOG(VP / 0.61078);   // temp var
  return (241.88 * T) / (17.558 - T);
}

float DHTSensor::dewPointFast() {
	const float a = 17.271f,
              b = 237.7f,
              temp = (a * Temperature) / (b + Temperature) + LOG(Humidity * 0.01f),
              Td = (b * temp) / (a - temp);
  return Td;
}

/** Private Function */
uint32_t DHTSensor::expectPulse(bool level) {

  #if (F_CPU > 16000000L)
    uint32_t count = 0;
  #else
    uint16_t count = 0;
  #endif

  while (HAL::digitalRead(data.pin) == level) {
    if (count++ >= _maxcycles) {
      return DHT_TIMEOUT; // Exceeded timeout, fail.
    }
  }

  return count;
}

float DHTSensor::read_temperature() {
  float f = NAN;

  switch (data.type) {
    case DHT11:
      f = read_data[2];
      if (read_data[3] & 0x80) f = -1 - f;
      f += (read_data[3] & 0x0f) * 0.1;
      break;
    case DHT12:
      f = read_data[2] + (read_data[3] & 0x0f) * 0.1;
      if (read_data[2] & 0x80) f *= -1;
      break;
    case DHT21:
    case DHT22:
      f = (read_data[2] & 0x7F) << 8 | read_data[3];
      f *= 0.1;
      if (read_data[2] & 0x80) f *= -1;
      break;
    default: break;
  }
  return f;
}

float DHTSensor::read_humidity() {
  float f = NAN;

  switch (data.type) {
    case DHT11:
    case DHT12:
      f = read_data[0] + read_data[1] * 0.1;
      break;
    case DHT21:
    case DHT22:
      f = read_data[0] << 8 | read_data[1];
      f *= 0.1;
      break;
    default: break;
  }
  return f;
}

#endif // ENABLED(DHT_SENSOR)
