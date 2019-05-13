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
 * Included in Marlin to Thinkyhead
 */
 
#include "../../../MK4duo.h"

#if ENABLED(DHT_SENSOR)

#include "menu.h"

void menu_dht() {
  START_MENU();
  MENU_BACK(MSG_MAIN);
  switch (dhtsensor.data.type) {
    case DHT11:
      STATIC_ITEM(MSG_DHT_11, false, false);
      break;
    case DHT12:
      STATIC_ITEM(MSG_DHT_12, false, false);
      break;
    case DHT21:
      STATIC_ITEM(MSG_DHT_21, false, false);
      break;
    case DHT22:
      STATIC_ITEM(MSG_DHT_22, false, false);
      break;
    default: break;
  }
  STATIC_ITEM(MSG_TEMPERATURE " (C):", false, false, ftostr52sp(dhtsensor.Temperature));
  STATIC_ITEM(MSG_HUMIDITY " (%):", false, false, ftostr52sp(dhtsensor.Humidity));
  STATIC_ITEM(MSG_DEWPOINT " (C):", false, false, ftostr52sp(dhtsensor.dewPoint()));
  END_MENU();
}

#endif // DHT_SENSOR
