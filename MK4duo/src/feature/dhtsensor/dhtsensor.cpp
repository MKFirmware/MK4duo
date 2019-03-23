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

  constexpr uint32_t MinimumReadInterval  = 2000; // ms
  constexpr uint32_t MaximumReadTime      = 20;   // ms

  DHTSensor dhtsensor;

  /** Public Parameters */
  dht_data_t  DHTSensor::data;

  float   DHTSensor::Temperature  = 20,
          DHTSensor::Humidity     = 10;

  /** Private Parameters */
  uint8_t DHTSensor::read_data[5] = { 0, 0, 0, 0, 0 };
  DHTSensor::SensorState DHTSensor::state = Init;

  // ISR
  uint16_t pulses[41];  // 1 start bit + 40 data bits
  volatile uint16_t lastPulseTime;
  volatile uint8_t numPulses;

  void DHT_ISR() {
    const uint32_t now = micros();
    if (HAL::digitalRead(dhtsensor.data.pin) == HIGH)
      lastPulseTime = now;
    else if (lastPulseTime > 0) {
      pulses[numPulses++] = now - lastPulseTime;
      if (numPulses == COUNT(pulses))
        detachInterrupt(dhtsensor.data.pin);
    }
  }

  void DHTSensor::init() {
    HAL::pinMode(data.pin, OUTPUT);
    state = Init;
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

    static watch_t  min_read_watch(MinimumReadInterval),
                    operation_watch;

    if (!min_read_watch.elapsed()) return;

    switch (state) {

      case Init:
        HAL::digitalWrite(data.pin, HIGH);
        state = Wait_250ms;
        operation_watch.start();
        break;

      case Wait_250ms:
        if (operation_watch.elapsed(250)) {
          HAL::pinMode(data.pin, OUTPUT);
          HAL::digitalWrite(data.pin, LOW);
          state = Wait_20ms;
          operation_watch.start();
        }
        break;

      case Wait_20ms:
        if (operation_watch.elapsed(20)) {

          // End the start signal by setting data line high for 40 microseconds.
          HAL::pinMode(data.pin, INPUT_PULLUP);

          // Now start reading the data line to get the value from the DHT sensor.
          HAL::delayMicroseconds(60); // Delay a bit to let sensor pull data line low.

          // Read from the DHT sensor using an DHT_ISR
          numPulses = COUNT(pulses);
          attachInterrupt(digitalPinToInterrupt(data.pin), DHT_ISR, CHANGE);
          lastPulseTime = 0;
          numPulses = 0;

          // Wait for the next operation to complete
          state = Read;
          operation_watch.start();
        }
        break;

      case Read:
        // Make sure we don't time out
        if (operation_watch.elapsed(MaximumReadTime)) {
          detachInterrupt(data.pin);
          state = Init;
          min_read_watch.start();
          break;
        }

        // Wait for the reading to complete (1 start bit + 40 data bits)
        if (numPulses != COUNT(pulses)) break;

        // We're reading now - reset the state
        state = Init;
        min_read_watch.start();

        // Check start bit
        if (pulses[0] < 40) break;

        // Reset 40 bits of received data to zero.
        ZERO(read_data);

        // Inspect each high pulse and determine which ones
        // are 0 (less than 40us) or 1 (more than 40us)
        for (uint8_t i = 0; i < 40; ++i) {
          read_data[i / 8] <<= 1;
          if (pulses[i + 1] > 40)
            read_data[i / 8] |= 1;
        }

        // Verify checksum
        if (((read_data[0] + read_data[1] + read_data[2] + read_data[3]) & 0xFF) != read_data[4])
          break;

        // Generate final results
        Temperature = readTemperature();
        Humidity    = readHumidity();

        break;
    }
  }

  /** Private Function */
  float DHTSensor::readTemperature() {
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

  float DHTSensor::readHumidity() {
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
