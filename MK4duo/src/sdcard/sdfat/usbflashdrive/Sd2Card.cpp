/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (C) 2013 Alberto Cotronei @MagoKimbra
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

#include "../../../../MK4duo.h"

#if ENABLED(USB_FLASH_DRIVE_SUPPORT)

#include "lib/Usb.h"
#include "lib/masstorage.h"

#include "Sd2Card.h"

USB usb;
BulkOnly bulk(&usb);

Sd2Card::state_t Sd2Card::state;

void Sd2Card::idle() {
  static uint32_t next_retry;

  switch (state) {
    case USB_HOST_DELAY_INIT:
      next_retry = millis() + 10000;
      state = USB_HOST_WAITING;
      break;
    case USB_HOST_WAITING:
      if (ELAPSED(millis(), next_retry)) {
        next_retry = millis() + 10000;
        state = USB_HOST_UNINITIALIZED;
      }
      break;
    case USB_HOST_UNINITIALIZED:
      SERIAL_EM("Starting USB host");
      if (!usb.start()) {
        SERIAL_EM("USB host failed to start. Will retry in 10 seconds.");
        state = USB_HOST_DELAY_INIT;
      }
      else {
        SERIAL_EM("USB host initialized");
        state = USB_HOST_INITIALIZED;
      }
      break;
    case USB_HOST_INITIALIZED:
      const uint8_t lastUsbTaskState = usb.getUsbTaskState();
      usb.Task();
      const uint8_t newUsbTaskState  = usb.getUsbTaskState();

      if (lastUsbTaskState == USB_STATE_RUNNING && newUsbTaskState != USB_STATE_RUNNING) {
        // the user pulled the flash drive. Make sure the bulk storage driver releases the address
        #if ENABLED(USB_DEBUG)
          SERIAL_EM("USB drive removed");
        #endif
        //bulk.Release();
      }
      if (lastUsbTaskState != USB_STATE_RUNNING && newUsbTaskState == USB_STATE_RUNNING) {
        #if ENABLED(USB_DEBUG)
          SERIAL_EM("USB drive inserted");
        #endif
      }
      break;
  }
}

bool Sd2Card::isInserted() {
  return usb.getUsbTaskState() == USB_STATE_RUNNING;
};

// Marlin calls this to initialize an SD card once it is inserted.
bool Sd2Card::init(uint8_t sckRateID, uint8_t chipSelectPin) {
  if (!ready()) return false;

  if (!bulk.LUNIsGood(0)) {
    SERIAL_EM("LUN zero is not good");
    return false;
  }

  const uint32_t sectorSize = bulk.GetSectorSize(0);
  if (sectorSize != 512) {
    SERIAL_MV("Expecting sector size of 512, got:", sectorSize);
    return false;
  }

  #if ENABLED(USB_DEBUG)
    lun0_capacity = bulk.GetCapacity(0);
    SERIAL_MV("LUN Capacity (in blocks): ", lun0_capacity);
  #endif
  return true;
}

// Returns the capacity of the card in blocks.
uint32_t Sd2Card::cardSize() {
  if (!ready()) return 0;
  #if DISABLED(USB_DEBUG)
    const uint32_t
  #endif
  lun0_capacity = bulk.GetCapacity(0);
  return lun0_capacity;
}

bool Sd2Card::readBlock(uint32_t block, uint8_t* dst) {
  if (!ready()) return false;
  #if ENABLED(USB_DEBUG)
    if (block >= lun0_capacity) {
      SERIAL_MV("Attempt to read past end of LUN: ", block);
      return false;
    }
    #if USB_DEBUG > 1
      SERIAL_MV("Read block ", block);
    #endif
  #endif
  return bulk.Read(0, block, 512, 1, dst) == 0;
}

bool Sd2Card::writeBlock(uint32_t block, const uint8_t* src) {
  if (!ready()) return false;
  #if ENABLED(USB_DEBUG)
    if (block >= lun0_capacity) {
      SERIAL_MV("Attempt to write past end of LUN: ", block);
      return false;
    }
    #if USB_DEBUG > 1
      SERIAL_MV("Write block ", block);
    #endif
  #endif
  return bulk.Write(0, block, 512, 1, src) == 0;
}

#endif // USB_FLASH_DRIVE_SUPPORT
