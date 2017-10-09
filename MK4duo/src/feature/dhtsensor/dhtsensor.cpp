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

#include "../../../base.h"

#if ENABLED(DHT_SENSOR)

  #define DHTMINREADINTERVAL 2000  // ms
  #define DHTMAXREADTIME      500  // ms

  DhtSensor dhtsensor;

  DhtSensorType DhtSensor::type = DhtSensorType::Dht11;

  millis_t  DhtSensor::lastReadTime = 0,
            DhtSensor::lastOperationTime = 0;
  float     DhtSensor::temperature = 0.0,
            DhtSensor::humidity = 0.0;

  DhtSensor::SensorState DhtSensor::state = Initialising;

  void DhtSensor::init() {
    OUT_WRITE(DHT_DATA_PIN, LOW);
    Spin();
  }

  void DhtSensor::Configure(const uint8_t dhtType) {
    switch (dhtType) {
      case 11:
        type = DhtSensorType::Dht11;
        break;
      case 21:
        type = DhtSensorType::Dht21;
        break;
      case 22:
        type = DhtSensorType::Dht22;
        break;
      default:
        SERIAL_EM("Invalid DHT sensor type");
        break;
    }
  }

  millis_t  lastPulseTime,
            pulses[41];   // 1 start bit + 40 data bits 

  volatile uint8_t numPulses;

  void DhtDataTransition() {
    const millis_t now = micros();

    if (HAL::digitalRead(DHT_DATA_PIN) == HIGH)
      lastPulseTime = now;
    else if (lastPulseTime > 0) {
      pulses[numPulses++] = now - lastPulseTime;
      if (numPulses == COUNT(pulses)) {
        detachInterrupt(DHT_DATA_PIN);
      }
    }
  }

  void DhtSensor::Run() {

    if ((millis() - lastReadTime) < DHTMINREADINTERVAL) return;

    switch (state) {
      case Initialising:
        // Send start signal. See DHT datasheet for full signal diagram:
        // http://www.adafruit.com/datasheets/Digital%20humidity%20and%20temperature%20sensor%20AM2302.pdf
        HAL::digitalWrite(DHT_DATA_PIN, HIGH);

        state = Starting;
        lastOperationTime = millis();
      break;

      case Starting:
        if ((millis() - lastOperationTime) >= 250) { // Wait 250ms
          OUT_WRITE(DHT_DATA_PIN, LOW);

          state = Starting2;
          lastOperationTime = millis();
        }
        break;

      case Starting2:
        if ((millis() - lastOperationTime) >= 20) { // Wait 20ms
          // End the start signal by setting data line high for 40 microseconds
          HAL::digitalWrite(DHT_DATA_PIN, HIGH);
          HAL::delayMicroseconds(40);

          // Now start reading the data line to get the value from the DHT sensor
          SET_INPUT_PULLUP(DHT_DATA_PIN);
          HAL::delayMicroseconds(10);

          // Read from the DHT sensor using an ISR, because we cannot use delays
          // due to the fact that this messes with the stepping ISR
          numPulses = 0;
          lastPulseTime = 0;
          attachInterrupt(DHT_DATA_PIN, DhtDataTransition, CHANGE);

          // Wait for the next operation to complete
          lastOperationTime = millis();
          state = Reading;
        }
        break;

      case Reading:
        // Make sure we don't time out
        if ((millis() - lastOperationTime) > DHTMAXREADTIME) {
          detachInterrupt(DHT_DATA_PIN);
          state = Initialising;
          lastReadTime = millis();
          break;
        }

        // Wait for the reading to complete (1 start bit + 40 data bits)
        if (numPulses != 41) break;

        // We're reading now - reset the state
        state = Initialising;
        lastReadTime = millis();

        // Check start bit
        if (pulses[0] < 40) break;

        // Reset 40 bits of received data to zero
        uint8_t data[5] = { 0, 0, 0, 0, 0 };

        // Inspect each high pulse and determine which ones
        // are 0 (less than 40us) or 1 (more than 40us)
        for (size_t i = 0; i < 40; ++i) {
          data[i / 8] <<= 1;
          if (pulses[i + 1] > 40) data[i / 8] |= 1;
        }

        // Verify checksum
        if (((data[0] + data[1] + data[2] + data[3]) & 0xFF) != data[4]) break;

        // Generate final results
        switch (type) {
          case DhtSensorType::Dht11:
            humidity = data[0];
            temperature = data[2];
            break;

          case DhtSensorType::Dht21:
          case DhtSensorType::Dht22:
            humidity = ((data[0] * 256) + data[1]) * 0.1;
            temperature = (((data[2] & 0x7F) * 256) + data[3]) * 0.1;
            if (data[2] & 0x80) temperature *= -1.0;
            break;

          default:
            humidity = 0.0;
            temperature = 0.0;
            break;
        }
      break;
    }
  }

#endif // ENABLED(DHT_SENSOR)
