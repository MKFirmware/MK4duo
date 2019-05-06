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

constexpr millis_s  DHTMinimumReadInterval = 2000, // ms
                    DHTMaximumReadTime     = 20;   // ms

DHTSensor dhtsensor;

/** Public Parameters */
dht_data_t DHTSensor::data;

float DHTSensor::Temperature  = 20,
      DHTSensor::Humidity     = 10;

/** Private Parameters */
uint8_t DHTSensor::read_data[5] = { 0, 0, 0, 0, 0 };

/** ISR */
uint16_t pulses[41];  // 1 start bit + 40 data bits
volatile uint16_t lastPulseTime;
volatile uint8_t numPulses;

void DHT_ISR() {
  const uint32_t now = micros();
  if (HAL::digitalRead(dhtsensor.data.pin) == HIGH)
    lastPulseTime = now;
  else if (lastPulseTime != 0) {
    pulses[numPulses++] = now - lastPulseTime;
    if (numPulses == COUNT(pulses))
      detachInterrupt(dhtsensor.data.pin);
  }
}

/** Public Function */
#if ENABLED(FREE_RTOS_SYSTEM)

  /**
   * Task DHT
   */
  extern "C" void DhtTask(void *pvParameters) {
    for (;;) {
      dhtsensor.sensor_read();
      vTaskDelay(DHTMinimumReadInterval); // Sleep DHTMinimumReadInterval
    }
  }

#else

  void DHTSensor::spin() {
    static millis_s min_read_ms = millis();
    if (expired(&min_read_ms, DHTMinimumReadInterval))
      sensor_read();
  }

#endif // ENABLED(FREE_RTOS_SYSTEM)

void DHTSensor::init() {
  #if ENABLED(FREE_RTOS_SYSTEM)
    // Create Task DHT
    xTaskCreate(DhtTask, "DHT", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
  #endif // FREE_RTOS_SYSTEM
}

void DHTSensor::factory_parameters() {
  data.pin  = DHT_DATA_PIN;
  data.type = DHTEnum(DHT_TYPE);
}

void DHTSensor::sensor_read() {

  // Send the start bit. This must be at least 18ms for the DHT11, 0.8ms for the DHT21, and 1ms long for the DHT22.
  HAL::pinMode(data.pin, OUTPUT);
  delay(20);

  // End the start signal by setting data line high. the sensor will respond with the start bit in 20 to 40us.
  // We need only force the data line high long enough to charge the line capacitance, after that the pullup resistor keeps it high.
  HAL::digitalWrite(data.pin, HIGH);
  HAL::delayMicroseconds(3);

  // Now start reading the data line to get the value from the DHT sensor
  HAL::pinMode(data.pin, INPUT_PULLUP);

  // It appears that switching the pin to an output disables the interrupt, so we need to call attachInterrupt here
  // We are likely to get an immediate interrupt at this point corresponding to the low-to-high transition. We must ignore this.
  numPulses = COUNT(pulses);
  attachInterrupt(digitalPinToInterrupt(data.pin), DHT_ISR, CHANGE);
  lastPulseTime = 0;
  numPulses = 0;

  delay(DHTMaximumReadTime);

  detachInterrupt(data.pin);

  // Check enough bits received and check start bit
  if (numPulses != COUNT(pulses) || pulses[0] < 40) return;

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
    return;

  // Generate final results
  Temperature = read_temperature();
  Humidity    = read_humidity();

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

/** Private Function */
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
