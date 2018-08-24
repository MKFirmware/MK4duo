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

/**
 * gcode.h
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

// Bedlevel Commands
#include "bedlevel/g26.h"                 // Mesh Validation
#include "bedlevel/g42.h"                 // Move to mesh
#include "bedlevel/m49.h"                 // Mesh Validation
#include "bedlevel/m420.h"                // Set ABL, MBL and UBL
#include "bedlevel/abl/g29.h"             // ABL
#include "bedlevel/abl/m421.h"            // Set ABL Manual
#include "bedlevel/mbl/g29.h"             // MBL
#include "bedlevel/mbl/m421.h"            // Set MBL Manual
#include "bedlevel/ubl/g29.h"             // UBL
#include "bedlevel/ubl/m421.h"            // Set UBL Manual

// Calibrate Commands
#include "calibrate/g28.h"                // Home
#include "calibrate/m48.h"                // Repeatability probe
#include "calibrate/m666.h"               // Set Two Endstops offsets

// Config Commands
#include "config/m92.h"
#include "config/m123_m124.h"             // Set Logic or Pullup endstop
#include "config/m200.h"
#include "config/m201.h"
#include "config/m203.h"
#include "config/m204.h"
#include "config/m205.h"
#include "config/m207_m209.h"             // FW RETRACT
#include "config/m218.h"                  // Set a tool offset
#include "config/m220.h"                  // Set speed percentage
#include "config/m221.h"                  // Set extrusion percentage
#include "config/m222.h"                  // Set density
#include "config/m301.h"                  // Set PID parameters Heater
#include "config/m302.h"                  // Allow cold extrudes
#include "config/m305.h"                  // Set thermistor and ADC parameters
#include "config/m306.h"                  // Set Heaters
#include "config/m595.h"                  // Set AD595 offset & Gain
#include "config/m569.h"                  // Set Stepper Direction
#include "config/m900.h"                  // Set and/or Get advance K factor
#include "config/m906.h"                  // Set Alligator motor currents or Set motor current in milliamps with have a TMC2130 driver
#include "config/m907.h"                  // Set digital trimpot motor current
#include "config/m908.h"                  // Control digital trimpot directly

// Control Commands
#include "control/m17.h"
#include "control/m18_m84.h"
#include "control/m42.h"
#include "control/m85.h"
#include "control/m106_m107.h"
#include "control/m112.h"
#include "control/m120.h"
#include "control/m121.h"
#include "control/m122.h"
#include "control/m226.h"                 // Wait until a pin
#include "control/m350_m351.h"            // Microstep
#include "control/m355.h"                 // Set Case Light
#include "control/m380_m381.h"            // Extruder Solenoid
#include "control/m400.h"                 // Finish all moves
#include "control/m410.h"                 // Quickstop
#include "control/m540.h"                 // Enable/disable SD card abort on endstop hit
#include "control/m605.h"                 // Set dual x-carriage movement mode
#include "control/m999.h"                 // Restart after being stopped

// Debug Commands
#include "debug/m43.h"
#include "debug/m44_pre_table.h"          // Debug Code Info

// Delta Commands
#include "delta/g33_type1.h"              // Autocalibration 7 point
#include "delta/g33_type2.h"              // Autocalibration matrix
#include "delta/m666.h"                   // Set delta parameters

// EEPROM Commands
#include "eeprom/m500_m503.h"             // Eeprom read write and print

// Feature Commands
#include "feature/g12.h"
#include "feature/g27.h"
#include "feature/g60.h"
#include "feature/g61.h"
#include "feature/m99.h"                  // Hysteresis feature
#include "feature/m100.h"                 // Free Memory Watcher
#include "feature/m125.h"
#include "feature/m126_m129.h"            // Solenoid feature
#include "feature/m150.h"
#include "feature/m240.h"                 // Photo Camera
#include "feature/m600.h"                 // Advanced Pause change filament
#include "feature/m603.h"                 // Configure filament change
#include "feature/m701_m702.h"            // Load / Unload filament
#include "feature/m911_m915.h"            // Set TRINAMIC driver
#include "feature/m922.h"                 // TMC DEBUG

// Geometry Commands
#include "geometry/g17_g19.h"
#include "geometry/g92.h"
#include "geometry/m206.h"
#include "geometry/m428.h"                // Set the home_offset

// Host Commands
#include "host/m110.h"
#include "host/m111.h"
#include "host/m113.h"
#include "host/m114.h"
#include "host/m115.h"
#include "host/m118.h"
#include "host/m119.h"                    // Endstop status print
#include "host/m408.h"                    // Json output
#include "host/m530.h"                    // Enables explicit printing mode
#include "host/m531.h"                    // Define filename being printed
#include "host/m532.h"                    // Update current print state progress

// LCD Commands
#include "lcd/m0_m1.h"
#include "lcd/m117.h"
#include "lcd/m145.h"
#include "lcd/m250.h"                     // Set LCD contrast
#include "lcd/m300.h"                     // Buzzer

// Mixing Commands
#include "mixing/m163_m165.h"

// Motion Commands
#include "motion/g0_g1.h"
#include "motion/g2_g3.h"
#include "motion/g4.h"
#include "motion/g5.h"
#include "motion/g10_g11.h"
#include "motion/g90.h"
#include "motion/g91.h"
#include "motion/m290.h"

// MultiMode Commands (Laser - CNC)
#include "multimode/g7.h"
#include "multimode/m3_m4.h"
#include "multimode/m5.h"
#include "multimode/m6.h"
#include "multimode/m450_m453.h"
#include "multimode/m649.h"               // Set laser options

// Muve3D Commands
#include "muve3d/m650_m655.h"             // Muve3D control

// Nextion Commands
#include "nextion/m35.h"                  // Upload firmware to Nextion from SD
#include "nextion/m995_m996.h"            // Setting GFX for Nextion

// Power Commands
#include "power/m80.h"
#include "power/m81.h"

// Probe Commands
#include "probe/g30.h"
#include "probe/g31_g32.h"
#include "probe/g38.h"
#include "probe/m401_m402.h"              // Lower e Raise probe
#include "probe/m851.h"                   // Set probe offset

// Rfid Commands
#include "rfid/m522.h"                    // Rfid read and write

// Servo Commands
#include "servo/m280.h"                   // Servo move
#include "servo/m281.h"                   // Servo Angles

// Scara Commands
#include "scara/m360_m364.h"
#include "scara/m665.h"

// SDCard Commands
#include "sdcard/sdcard.h"

// Sensor Commands
#include "sensor/m70.h"
#include "sensor/m404_m407.h"             // Filament Sensor
#include "sensor/m512.h"                  // Extruder Encoder read pin
#include "sensor/m602_m604.h"             // Extruder Encoder settings (M602 still TODO)

// Stats Commands
#include "stats/m31.h"
#include "stats/m75.h"
#include "stats/m76.h"
#include "stats/m77.h"
#include "stats/m78.h"

// Temperature Commands
#include "temperature/m104.h"
#include "temperature/m105.h"
#include "temperature/m108.h"
#include "temperature/m109.h"
#include "temperature/m140.h"
#include "temperature/m141.h"
#include "temperature/m142.h"
#include "temperature/m155.h"
#include "temperature/m190.h"
#include "temperature/m191.h"
#include "temperature/m192.h"
#include "temperature/m303.h"             // PID autotune

// Tools Commands
#include "tools/tcode.h"

// Units Commands
#include "units/g20_g21.h"
#include "units/m82.h"
#include "units/m83.h"
#include "units/m149.h"

#if ENABLED(FASTER_GCODE_EXECUTE) || ENABLED(ARDUINO_ARCH_SAM)
  // Table for G and M code
  #include "table_gcode.h"
  #include "table_mcode.h"

  // Include m44 post define table for debugging
  #include "debug/m44_post_table.h"
#endif
