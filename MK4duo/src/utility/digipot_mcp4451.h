/**
 * MK4duo 3D Printer Firmware
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 - 2017 Alberto Cotronei @MagoKimbra
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

#ifndef DIGIPOT_MCP4451_H
#define DIGIPOT_MCP4451_H

// Settings for the I2C based DIGIPOT (MCP4451) on Azteeg X3 Pro
#if MB(5DPRINT)
  #define DIGIPOT_I2C_FACTOR 117.96
  #define DIGIPOT_I2C_MAX_CURRENT 1.736
#else
  #define DIGIPOT_I2C_FACTOR 106.7
  #define DIGIPOT_I2C_MAX_CURRENT 2.5
#endif

static byte current_to_wiper(float current);
static void i2c_send(byte addr, byte a, byte b);
// This is for the MCP4451 I2C based digipot
extern void digipot_i2c_set_current(int channel, float current);
extern void digipot_i2c_init();

#endif
