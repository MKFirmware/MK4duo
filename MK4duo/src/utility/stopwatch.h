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
 * @brief Stopwatch class
 * @details This class acts as a timer proving stopwatch functionality including
 * the ability to pause the running time counter.
 */
class Stopwatch {

  private: /** Private Parameters */

    enum State {
      STOPPED,
      RUNNING,
      PAUSED
    };

    static Stopwatch::State state;

    static watch_t  Timestamp;

    static millis_t accumulator;

  public: /** Public Function */

    /**
     * @brief Initialize the stopwatch
     */
    FORCE_INLINE static void init() { reset(); }

    /**
     * @brief Stops the stopwatch
     * @details Stops the running timer, it will silently ignore the request if
     * no timer is currently running.
     * @return true is method was successful
     */
    static bool stop();

    /**
     * @brief Pauses the stopwatch
     * @details Pauses the running timer, it will silently ignore the request if
     * no timer is currently running.
     * @return true is method was successful
     */
    static bool pause();

    /**
     * @brief Starts the stopwatch
     * @details Starts the timer, it will silently ignore the request if the
     * timer is already running.
     * @return true is method was successful
     */
    static bool start();

    /**
     * @brief Resume the stopwatch
     * @details Resume a timer from a given this_time
     */
    static void resume(const millis_t this_time);

    /**
     * @brief Resets the stopwatch
     * @details Resets all settings to their default values.
     */
    static void reset();

    /**
     * @brief Checks if the timer is running
     * @details Returns true if the timer is currently running, false otherwise.
     * @return true if stopwatch is running
     */
    FORCE_INLINE static bool isRunning() { return state == RUNNING; }

    /**
     * @brief Checks if the timer is paused
     * @details Returns true if the timer is currently paused, false otherwise.
     * @return true if stopwatch is paused
     */
    FORCE_INLINE static bool isPaused() { return state == PAUSED; }

    /**
     * @brief Gets the running time
     * @details Returns the total number of seconds the timer has been running.
     * @return the delta since starting the stopwatch
     */
    static millis_t duration();

};
