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
   *    O[int]    ADC high offset correction
   *    P[int]    Sensor Pin
   *    T[int]    Sensor Type
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

    Heater *act = &heaters[h];

    act->sensor.r25           = parser.floatval('A', act->sensor.r25);
    act->sensor.beta          = parser.floatval('B', act->sensor.beta);
    act->sensor.shC           = parser.floatval('C', act->sensor.shC);
    act->sensor.pullupR       = parser.floatval('R', act->sensor.pullupR);
    act->sensor.adcLowOffset  = parser.intval('L', act->sensor.adcLowOffset);
    act->sensor.adcHighOffset = parser.intval('O', act->sensor.adcHighOffset);
    act->sensor.type          = parser.intval('T', act->sensor.type);

    if (parser.seen('P')) {
      // Put off the heaters
      act->setTarget(0);

      const pin_t new_pin = parser.value_pin();
      if (WITHIN(new_pin, 0 , NUM_ANALOG_INPUTS)) {
        const pin_t old_pin = act->sensor.pin;
        act->sensor.pin = new_pin;
        HAL::AdcChangePin(old_pin, act->sensor.pin);
      }
    }

    act->sensor.CalcDerivedParameters();
    act->print_sensor_parameters();

 }

#endif // HEATER_COUNT > 0
