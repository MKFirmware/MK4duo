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
 * Included in Marlin to Thinkyhead
 */
 
#include "../../../MK4duo.h"

#if HAS_LCD_MENU && HAS_DHT

#include "menu.h"

void menu_dht() {
  START_MENU();
  BACK_ITEM(MSG_MAIN);
  switch (dhtsensor.data.type) {
    case DHT11:
      STATIC_ITEM(MSG_DHT_11);
      break;
    case DHT12:
      STATIC_ITEM(MSG_DHT_12);
      break;
    case DHT21:
      STATIC_ITEM(MSG_DHT_21);
      break;
    case DHT22:
      STATIC_ITEM(MSG_DHT_22);
      break;
    default: break;
  }
  STATIC_ITEM(MSG_DHT_TEMPERATURE, SS_LEFT, ftostr52sp(dhtsensor.temperature));
  STATIC_ITEM(MSG_DHT_HUMIDITY, SS_LEFT, ftostr52sp(dhtsensor.humidity));
  STATIC_ITEM(MSG_DHT_DEWPOINT, SS_LEFT, ftostr52sp(dhtsensor.dewPoint()));
  END_MENU();
}

#endif // HAS_LCD_MENU && HAS_DHT
