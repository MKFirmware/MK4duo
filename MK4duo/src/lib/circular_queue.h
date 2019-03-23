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
 * @brief   Circular Queue class
 * @details Implementation of the classic ring buffer data structure
 */
template<typename T, uint8_t N>
class Circular_Queue {

  private: /** Private Parameters */

    struct buffer_t {
      uint8_t head;   // Read position in Circular Queue
      uint8_t tail;   // Write position in Circular Queue
      uint8_t count;  // Number of commands in the Circular Queue
      uint8_t size;   // Size of Circular Queue
      T queue[N];     // Queue
    } buffer;

  public: /** Constructor */

    Circular_Queue<T, N>() {
      this->buffer.size = N;
      this->buffer.count = this->buffer.head = this->buffer.tail = 0;
    }

  public: /** Public Function */

    void clear() {
      this->buffer.count = this->buffer.head = this->buffer.tail = 0;
    }

    T dequeue() {
      if (this->isEmpty()) return T();

      uint8_t index = this->buffer.head;

      --this->buffer.count;
      if (++this->buffer.head == this->buffer.size)
        this->buffer.head = 0;

      return this->buffer.queue[index];
    }

    bool enqueue(T const &item) {
      if (this->isFull()) return false;

      this->buffer.queue[this->buffer.tail] = item;

      ++this->buffer.count;
      if (++this->buffer.tail == this->buffer.size)
        this->buffer.tail = 0;

      return true;
    }

    bool isEmpty() {
      return this->buffer.count == 0;
    }

    bool isFull() {
      return this->buffer.count >= this->buffer.size;
    }

    uint8_t size() {
      return this->buffer.size;
    }

    T peek() {
      return this->buffer.queue[this->buffer.head];
    }

    T peek(const uint8_t index) {
      return this->buffer.queue[index];
    }

    uint8_t count() {
      return this->buffer.count;
    }

    uint8_t head() {
      return this->buffer.head;
    }

    uint8_t tail() {
      return this->buffer.tail;
    }

};
