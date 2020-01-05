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
 * dhtsensor.cpp
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#include "../../../../MK4duo.h"
#include "sanitycheck.h"

#if HAS_DHT

constexpr millis_s  DHTMinimumReadInterval = 2000, // ms
                    DHTMaximumReadTime     = 20;   // ms

DHTSensor dhtsensor;

/** Public Parameters */
dht_data_t DHTSensor::data;

float DHTSensor::temperature  = 25,
      DHTSensor::humidity     = 50;

/** Private Parameters */
uint8_t DHTSensor::read_data[5] = { 0, 0, 0, 0, 0 };

// ISR Parameters
volatile uint16_t lastPulseTime;
volatile uint8_t numPulses;
uint16_t pulses[41];    // 1 start bit + 40 data bits

// Pulse ISR
extern "C" void DHT_ISR() {
  if (numPulses < COUNT(pulses)) {
    const uint32_t now = micros();
    if (HAL::digitalRead(dhtsensor.data.pin))
      lastPulseTime = now;
    else if (lastPulseTime)
      pulses[numPulses++] = now - lastPulseTime;
  }
}

/** Public Function */
void DHTSensor::init() { /*HAL::pinMode(data.pin, OUTPUT);*/ }

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

  static short_timer_t min_read_timer(millis());

  if (min_read_timer.pending(DHTMinimumReadInterval)) return;

  // Start the reading process
  HAL::pinMode(data.pin, INPUT_PULLUP);
  delay(1);

  // First set data line low for a period according to sensor type
  HAL::pinMode(data.pin, OUTPUT_LOW);
  switch (data.type) {
    case DHT22:
    case DHT21:
      HAL::delayMicroseconds(1100); // data sheet says "at least 1ms"
      break;
    default:
      delay(20); // data sheet says at least 18ms, 20ms just to be safe
      break;
  }

  // End the start signal by setting data line high. The sensor will respond with the start bit in 20 to 40us.
  // We need only force the data line high long enough to charge the line capacitance, after that the pullup resistor keeps it high.
  // This will generate an interrupt, but we will ignore it
  HAL::digitalWrite(data.pin, HIGH);
  HAL::delayMicroseconds(3);

  // End the start signal by setting data line high for 40 microseconds.
  HAL::pinMode(data.pin, INPUT_PULLUP);
  // Delay a bit to let sensor pull data line low.
  HAL::delayMicroseconds(55);

  // Turn off interrupts temporarily because the next sections
  // are timing critical and we don't want any interruptions.
  DISABLE_ISRS();
    // Now start reading the data line to get the value from the DHT sensor.
    // Read from the DHT sensor using an DHT_ISR
    numPulses = COUNT(pulses);
    attachInterrupt(digitalPinToInterrupt(data.pin), DHT_ISR, CHANGE);
    lastPulseTime = 0;
    numPulses = 0;
  ENABLE_ISRS();

  // Wait for the incoming signal to be read by the ISR (1 start bit + 40 data bits), or until timeout.
  // We don't have the ISR wake the process up, because that would require the priority of the pin change interrupt to be reduced.
  // So we just delay for long enough for the data to have been sent. It takes typically 4 to 5ms.
  delay(DHTMaximumReadTime);

  detachInterrupt(data.pin);

  // Attempt to convert the signal into temp + RH values
  process_reading();

}

float DHTSensor::dewPoint() {
  // (1) Saturation Vapor Pressure = ESGG(T)
  const float RATIO = 373.15 / (273.15 + temperature);
  float RHS = -7.90298 * (RATIO - 1);
  RHS += 5.02808 * log10(RATIO);
  RHS += -1.3816e-7 * (POW(10, (11.344 * (1 - 1/RATIO ))) - 1) ;
  RHS += 8.1328e-3 * (POW(10, (-3.49149 * (RATIO - 1))) - 1) ;
  RHS += log10(1013.246);

  // factor -3 is to adjust units - Vapor Pressure SVP * humidity
  const float VP = POW(10, RHS - 3) * humidity;

  // (2) DEWPOINT = F(Vapor Pressure)
  float T = LOG(VP / 0.61078);   // temp var
  return (241.88 * T) / (17.558 - T);
}

float DHTSensor::dewPointFast() {
	const float a = 17.271f,
              b = 237.7f,
              temp = (a * temperature) / (b + temperature) + LOG(humidity * 0.01f),
              Td = (b * temp) / (a - temp);
  return Td;
}

/** Private Function */
void DHTSensor::process_reading() {

  // Check start bit
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
  temperature = read_temperature();
  humidity    = read_humidity();

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
      f = (word(read_data[2] & 0x7F)) << 8 | read_data[3];
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
      f = word(read_data[0]) << 8 | read_data[1];
      f *= 0.1;
      break;
    default: break;
  }
  return f;
}

#endif // HAS_DHT
