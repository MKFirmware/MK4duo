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
#pragma once

/**
 * Saved variable and restore
 */
template<typename T>
class restorer {

  public: /** Constructor */

    restorer(T& perm) : ref_(perm), val_(perm) {}
    restorer(T& perm, T temp_val) : ref_(perm), val_(perm) { perm = temp_val; }

  public: /** Destructor */

    ~restorer() { restore(); }

  public: /** Public Parameters */

    T& ref_;
    T  val_;

  public: /** Public Function */

    inline void restore() { ref_ = val_; }

};

#define REMEMBER(NAME,X,...)  restorer<typeof(X)> restorer_##NAME(X, ##__VA_ARGS__)
#define RESTORE(NAME)         restorer_##NAME.restore()
