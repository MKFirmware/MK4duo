/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2019 Alberto Cotronei @MagoKimbra
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
 * Copyright (c) 2019 Alberto Cotronei @MagoKimbra
 */

#if HEATER_COUNT > 0

#define CODE_M305

/**
 * M305: Set thermistor and ADC parameters
 *
 *   H[heaters] H = 0-5 Hotend, H = -1 BED, H = -2 CHAMBER
 *
 *    T[int]      0-3 For Select Beds or Chambers
 *
 *    A[float]  Thermistor resistance at 25°C
 *    B[float]  BetaK
 *    C[float]  Steinhart-Hart C coefficien
 *    R[float]  Pullup resistor value
 *    L[int]    ADC low offset correction
 *    O[int]    ADC high offset correction
 *    P[int]    Sensor Pin
 *    S[int]    Sensor Type
 *
 *  D DHT parameters
 *    S[int]    Type Sensor
 *    P[int]    Sensor Pin
 *
 */
inline void gcode_M305(void) {

  #if ENABLED(DHT_SENSOR)
    if (parser.seen('D')) {
      #if DISABLED(DISABLE_M503)
        // No arguments? Show M305 report.
        if (!parser.seen("PS")) {
          dhtsensor.print_M305();
          return;
        }
      #endif
      dhtsensor.data.pin = parser.intval('P', DHT_DATA_PIN);
      if (parser.seen('S'))
        dhtsensor.change_type(DHTEnum(parser.value_int()));
      dhtsensor.init();
      return;
    }
  #endif

  Heater *act = commands.get_target_heater();

  if (!act) return;

  #if DISABLED(DISABLE_M503)
    // No arguments? Show M305 report.
    if (!parser.seen("ABCRLOSP")) {
      act->print_M305();
      return;
    }
  #endif

  act->data.sensor.r25            = parser.floatval('A', act->data.sensor.r25);
  act->data.sensor.beta           = parser.floatval('B', act->data.sensor.beta);
  act->data.sensor.shC            = parser.floatval('C', act->data.sensor.shC);
  act->data.sensor.pullupR        = parser.floatval('R', act->data.sensor.pullupR);
  act->data.sensor.adcLowOffset   = parser.intval('L', act->data.sensor.adcLowOffset);
  act->data.sensor.adcHighOffset  = parser.intval('O', act->data.sensor.adcHighOffset);
  act->data.sensor.type           = parser.intval('S', act->data.sensor.type);

  if (parser.seen('P')) {
    // Put off the heaters
    act->setTarget(0);

    const pin_t new_pin = parser.analog_value_pin();
    if (new_pin != NoPin) {
      const pin_t old_pin = act->data.sensor.pin;
      act->data.sensor.pin = new_pin;
      HAL::AdcChangePin(old_pin, act->data.sensor.pin);
    }
  }

  act->data.sensor.CalcDerivedParameters();

}

#endif // HEATER_COUNT > 0
