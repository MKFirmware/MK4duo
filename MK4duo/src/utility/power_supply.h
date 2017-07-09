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

#ifndef _POWER_SUPPLY_H_
#define _POWER_SUPPLY_H_

#if HAS_POWER_SWITCH

  class Power {

    public:

      static bool powersupply_on;

      static void check();
      static void power_on();
      static void power_off();

    private:

      static bool is_power_needed();

  };

  extern Power powerManager;

#endif // HAS_POWER_SWITCH

#endif /* _POWER_SUPPLY_H_ */
