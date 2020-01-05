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
 * mcode
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#if HAS_HEATER

#define CODE_M305

/**
 * M305: Set thermistor and ADC parameters
 *
 *   H[heaters] H = 0-5 Hotend, H = -1 BED, H = -2 CHAMBER
 *
 *    T[int]      0-3 For Select Beds or Chambers
 *
 *    A[ohms]   Thermistor resistance at 25°C
 *    B[beta]   Thermistor betaK value
 *    C[coeff]  Steinhart-Hart C coefficient
 *    R[ohms]   Pullup resistor value
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
inline void gcode_M305() {

  #if HAS_DHT
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

  Heater * const act = commands.get_target_heater();

  if (!act) return;

  #if DISABLED(DISABLE_M503)
    // No arguments? Show M305 report.
    if (!parser.seen("ABCRLOSP")) {
      act->print_M305();
      return;
    }
  #endif

  // Resistance at 25C// Resistance at 25C
  if (parser.seen('A')) {
    if (!act->data.sensor.set_res_25(parser.value_float()))
      SERIAL_EM("!Invalid 25C resistance. (0 < T < 10000000)");
  }

  // Beta value
  if (parser.seen('B')) {
    if (!act->data.sensor.set_beta(parser.value_float()))
      SERIAL_EM("!Invalid beta. (0 < B < 1000000)");
  }

  // Steinhart-Hart C coefficient
  if (parser.seen('C')) {
    if (!act->data.sensor.set_shC(parser.value_float()))
      SERIAL_EM("!Invalid Steinhart-Hart C coeff. (-0.01 < C < +0.01)");
  }

  // Pullup resistor value
  if (parser.seen('R')) {
    if (!act->data.sensor.set_pullup_res(parser.value_float()))
      SERIAL_EM("!Invalid series resistance. (0 < R < 1000000)");
  }

  // Adc Low offset
  if (parser.seen('L')) {
    if (!act->data.sensor.set_LowOffset(parser.value_int()))
      SERIAL_EM("!Invalid Low Offset. (-1000 < L < 1000)");
  }

  // Adc Low offset
  if (parser.seen('O')) {
    if (!act->data.sensor.set_HighOffset(parser.value_int()))
      SERIAL_EM("!Invalid High Offset. (-1000 < O < 1000)");
  }

  act->data.sensor.type = parser.intval('S', act->data.sensor.type);

  if (parser.seen('P')) {
    // Put off the heaters
    act->set_target_temp(0);

    const pin_t new_pin = HAL::analog_value_pin();
    if (new_pin != NoPin) {
      const pin_t old_pin = act->data.sensor.pin;
      act->data.sensor.pin = new_pin;
      HAL::AdcChangePin(old_pin, act->data.sensor.pin);
    }
  }

  act->data.sensor.CalcDerivedParameters();

}

#endif // HAS_HEATER
