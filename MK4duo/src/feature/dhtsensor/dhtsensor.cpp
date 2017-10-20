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
 * dhtsensor.cpp
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"

#if ENABLED(DHT_SENSOR)

  #define DHTMINREADINTERVAL 2000  // ms

  DhtSensor::DhtSensor(const Pin _pin, const uint8_t _type) {
    pin   = _pin;
    type  = _type;

    #if ENABLED(__AVR__)
      _bit = digitalPinToBitMask(pin);
      _port = digitalPinToPort(pin);
    #endif

    maxcycles = microsecondsToClockCycles(1000);  // 1 millisecond timeout for
                                                  // reading pulses from DHT sensor.
  }

  void DhtSensor::init(void) {
    HAL::pinMode(pin, INPUT_PULLUP);
    lastreadtime = -DHTMINREADINTERVAL;
  }

  void DhtSensor::change_type(const uint8_t dhtType) {
    switch (dhtType) {
      case DHT11:
        type = DHT11;
        break;
      case DHT21:
        type = DHT21;
        break;
      case DHT22:
        type = DHT22;
        break;
      default:
        SERIAL_LM(ER, "Invalid DHT sensor type");
        break;
    }
  }

  void DhtSensor::print_parameters(void) {
    SERIAL_SMV(CFG, " DHT sensor Pin:", pin);
    SERIAL_EMV(" Type:DHT", type);
  }

  float DhtSensor::readTemperature(const bool force/*=false*/) {
    float f = 0.0;

    if (read(force)) {
      switch (type) {
        case DHT11:
          f = data[2];
          break;
        case DHT22:
        case DHT21:
          f = data[2] & 0x7F;
          f *= 256;
          f += data[3];
          f *= 0.1;
          if (data[2] & 0x80) f *= -1;
          break;
      }
    }
    return f;
  }

  float DhtSensor::readHumidity() {
    float u = 0.0;

    if (read()) {
      switch (type) {
        case DHT11:
          u = data[0];
          break;
        case DHT22:
        case DHT21:
          u = data[0];
          u *= 256;
          u += data[1];
          u *= 0.1;
          break;
      }
    }
    return u;
  }

  bool DhtSensor::read(const bool force/*=false*/) {
    // Check if sensor was read less than two seconds ago and return early
    // to use last reading.
    millis_t currenttime = millis();
    if (!force && ((currenttime - lastreadtime) < 2000)) {
      return lastresult; // return last correct measurement
    }
    lastreadtime = currenttime;

    // Reset 40 bits of received data to zero.
    data[0] = data[1] = data[2] = data[3] = data[4] = 0;

    // Send start signal.  See DHT datasheet for full signal diagram:
    //   http://www.adafruit.com/datasheets/Digital%20humidity%20and%20temperature%20sensor%20AM2302.pdf

    // Go into high impedence state to let pull-up raise data line level and
    // start the reading process.
    HAL::digitalWrite(pin, HIGH);
    HAL::delayMicroseconds(150);

    // First set data line low for 20 milliseconds.
    HAL::pinMode(pin, OUTPUT);
    HAL::digitalWrite(pin, LOW);
    delay(20);

    uint32_t cycles[80];

    // Turn off interrupts temporarily because the next sections are timing critical
    // and we don't want any interruptions.
    InterruptLock lock;

    // End the start signal by setting data line high for 40 microseconds.
    HAL::digitalWrite(pin, HIGH);
    HAL::delayMicroseconds(40);

    // Now start reading the data line to get the value from the DHT sensor.
    HAL::pinMode(pin, INPUT_PULLUP);
    HAL::delayMicroseconds(10);  // Delay a bit to let sensor pull data line low.

    // First expect a low signal for ~80 microseconds followed by a high signal
    // for ~80 microseconds again.
    if (expectPulse(LOW) == 0) {
      lastresult = false;
      return lastresult;
    }
    if (expectPulse(HIGH) == 0) {
      lastresult = false;
      return lastresult;
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
      cycles[i + 1] = expectPulse(HIGH);
    }
    // Timing critical code is now complete.

    // Inspect pulses and determine which ones are 0 (high state cycle count < low
    // state cycle count), or 1 (high state cycle count > low state cycle count).
    for (int i = 0; i < 40; ++i) {
      uint32_t lowCycles  = cycles[2 * i];
      uint32_t highCycles = cycles[2 * i + 1];
      if ((lowCycles == 0) || (highCycles == 0)) {
        lastresult = false;
        return lastresult;
      }
      data[i/8] <<= 1;
      // Now compare the low and high cycle times to see if the bit is a 0 or 1.
      if (highCycles > lowCycles) {
        // High cycles are greater than 50us low cycle count, must be a 1.
        data[i/8] |= 1;
      }
      // Else high cycles are less than (or equal to, a weird case) the 50us low
      // cycle count so this must be a zero.  Nothing needs to be changed in the
      // stored data.
    }

    // Check we read 40 bits and that the checksum matches.
    if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
      lastresult = true;
      return lastresult;
    }
    else {
      lastresult = false;
      return lastresult;
    }
  }

  uint32_t DhtSensor::expectPulse(bool level) {

    uint32_t count = 0;

    // On AVR platforms use direct GPIO port access as it's much faster and better
    // for catching pulses that are 10's of microseconds in length:
    #if ENABLED(__AVR__)

      uint8_t portState = level ? _bit : 0;
      while ((*portInputRegister(_port) & _bit) == portState) {
        if (count++ >= maxcycles) {
          return 0; // Exceeded timeout, fail.
        }
      }

    #else

      while (HAL::digitalRead(pin) == level) {
        if (count++ >= maxcycles) {
          return 0; // Exceeded timeout, fail.
        }
      }

    #endif

    return count;
  }

  DhtSensor dhtsensor(DHT_DATA_PIN, DHT_TYPE);

#endif // ENABLED(DHT_SENSOR)
