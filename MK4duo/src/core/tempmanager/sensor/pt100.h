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

#define THERMISTOR_TABLE_ADC_RESOLUTION   _BV(10)
#define THERMISTOR_TABLE_SCALE            (AD_RANGE / (THERMISTOR_TABLE_ADC_RESOLUTION))
#define OV_SCALE(N)                       (float((N) * 5) / float(HAL_VOLTAGE_PIN))
#define OV(N)                             int16_t(OV_SCALE(N) * (THERMISTOR_TABLE_SCALE))

// Pt100 with INA826 amp on Ultimaker v2.0 electronics
const short pt100[][2] PROGMEM = {
  { OV(  0),    0 },
  { OV(227),    1 },
  { OV(236),   10 },
  { OV(245),   20 },
  { OV(253),   30 },
  { OV(262),   40 },
  { OV(270),   50 },
  { OV(279),   60 },
  { OV(287),   70 },
  { OV(295),   80 },
  { OV(304),   90 },
  { OV(312),  100 },
  { OV(320),  110 },
  { OV(329),  120 },
  { OV(337),  130 },
  { OV(345),  140 },
  { OV(353),  150 },
  { OV(361),  160 },
  { OV(369),  170 },
  { OV(377),  180 },
  { OV(385),  190 },
  { OV(393),  200 },
  { OV(401),  210 },
  { OV(409),  220 },
  { OV(417),  230 },
  { OV(424),  240 },
  { OV(432),  250 },
  { OV(440),  260 },
  { OV(447),  270 },
  { OV(455),  280 },
  { OV(463),  290 },
  { OV(470),  300 },
  { OV(478),  310 },
  { OV(485),  320 },
  { OV(493),  330 },
  { OV(500),  340 },
  { OV(507),  350 },
  { OV(515),  360 },
  { OV(522),  370 },
  { OV(529),  380 },
  { OV(537),  390 },
  { OV(544),  400 },
  { OV(614),  500 }
};
