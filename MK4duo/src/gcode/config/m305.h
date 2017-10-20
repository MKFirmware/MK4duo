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
 * mcode
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#if HEATER_COUNT > 0

  #define CODE_M305

  /**
   * M305: Set thermistor and ADC parameters
   *
   *  H[heaters] H = 0-3 Hotend, H = -1 BED, H = -2 CHAMBER, H = -3 COOLER
   *
   *    A[float]  Thermistor resistance at 25°C
   *    B[float]  BetaK
   *    C[float]  Steinhart-Hart C coefficien
   *    R[float]  Pullup resistor value
   *    L[int]    ADC low offset correction
   *    N[int]    ADC high offset correction
   *    P[int]    Sensor Pin
   *
   *  D DHT parameters
   *    S[int]    Type Sensor
   *    P[int]    Sensor Pin
   *
   */
  inline void gcode_M305(void) {

    #if ENABLED(DHT_SENSOR)
      if (parser.seen('D')) {
        dhtsensor.pin = parser.intval('P', DHT_DATA_PIN);
        if (parser.seen('S'))
          dhtsensor.change_type(parser.value_int());
        dhtsensor.init();
        dhtsensor.print_parameters();
        return;
      }
    #endif

    int8_t h = parser.seen('H') ? parser.value_int() : 0; // hotend being updated

    if (!commands.get_target_heater(h)) return;

    //if (parser.seen('S'))           heaters[h].sensor.name = parser.string_arg;
    heaters[h].sensor.r25           = parser.floatval('A', heaters[h].sensor.r25);
    heaters[h].sensor.beta          = parser.floatval('B', heaters[h].sensor.beta);
    heaters[h].sensor.shC           = parser.floatval('C', heaters[h].sensor.shC);
    heaters[h].sensor.pullupR       = parser.floatval('R', heaters[h].sensor.pullupR);
    heaters[h].sensor.adcLowOffset  = parser.intval('L', heaters[h].sensor.adcLowOffset);
    heaters[h].sensor.adcHighOffset = parser.intval('N', heaters[h].sensor.adcHighOffset);

    if (parser.seen('P')) {
      // Put off the heaters
      heaters[h].setTarget(0);

      const Pin new_pin = parser.value_pin();
      if (WITHIN(new_pin, 0 , MAX_ANALOG_PIN_NUMBER)) {
        const Pin old_pin = heaters[h].sensor.pin;
        heaters[h].sensor.pin = new_pin;
        HAL::AdcChangeChannel(old_pin, heaters[h].sensor.pin);
      }
    }

    heaters[h].sensor.CalcDerivedParameters();
    heaters[h].sensor_print_parameters(h);

 }

#endif // PIDTEMPCHAMBER
