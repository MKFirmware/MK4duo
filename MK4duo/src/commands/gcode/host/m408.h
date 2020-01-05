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
 * mcode
 *
 * Copyright (c) 2020 Alberto Cotronei @MagoKimbra
 */

#if ENABLED(JSON_OUTPUT)

  #define CODE_M408

  char GetStatusCharacter(){
    return  print_job_counter.isRunning() ? 'P'   // Printing
          : print_job_counter.isPaused()  ? 'A'   // Paused / Stopped
          :                                 'I';  // Idle
  }

  /**
   * M408: JSON STATUS OUTPUT
   */
  inline void gcode_M408() {
    bool firstOccurrence;
    uint8_t type = 0;

    if (parser.seen('S')) type = parser.value_byte();

    char ch = GetStatusCharacter();
    SERIAL_MSG("{\"status\":\"");
    SERIAL_CHR(ch);

    SERIAL_MSG("\",\"coords\": {\"axesHomed\":[");
    if (mechanics.isHomedAll())
      SERIAL_MSG("1, 1, 1");
    else
      SERIAL_MSG("0, 0, 0");

    SERIAL_MV("],\"extr\":[", mechanics.position.e);
    SERIAL_MV("],\"xyz\":[", mechanics.position.x); // X AXIS
    SERIAL_MV(",", mechanics.position.y);           // Y AXIS
    SERIAL_MV(",", mechanics.position.z);           // Z AXIS

    SERIAL_MV("]},\"currentTool\":", toolManager.extruder.active);

    #if HAS_POWER_SWITCH
      SERIAL_MSG(",\"params\": {\"atxPower\":");
      SERIAL_CHR(powerManager.is_on() ? '1' : '0');
    #else
      SERIAL_MSG(",\"params\": {\"NormPower\":");
    #endif

    #if HAS_FAN
      SERIAL_MSG(",\"fanPercent\":[");
      SERIAL_VAL(fans[0]->speed);
    #endif

    SERIAL_MV("],\"speedFactor\":", mechanics.feedrate_percentage);

    SERIAL_MSG(",\"extrFactors\":[");
    firstOccurrence = true;
    LOOP_EXTRUDER() {
      if (!firstOccurrence) SERIAL_CHR(',');
      SERIAL_VAL(extruders[e]->flow_percentage);
      firstOccurrence = false;
    }
    SERIAL_EM("]},");

    SERIAL_MSG("\"temps\": {");
    #if HAS_BEDS
      SERIAL_MV("\"bed\": {\"current\":", beds[0]->deg_current(), 1);
      SERIAL_MV(",\"active\":", beds[0]->deg_target());
      SERIAL_MSG(",\"state\":");
      SERIAL_CHR(beds[0]->deg_target() > 0 ? '2' : '1');
      SERIAL_MSG("},");
    #endif
    SERIAL_MSG("\"heads\": {\"current\":[");
    firstOccurrence = true;
    for (int8_t h = 0; h < HOTENDS; h++) {
      if (!firstOccurrence) SERIAL_CHR(',');
      SERIAL_VAL(hotends[h]->deg_current(), 1);
      firstOccurrence = false;
    }
    SERIAL_MSG("],\"active\":[");
    firstOccurrence = true;
    LOOP_HOTEND() {
      if (!firstOccurrence) SERIAL_CHR(',');
      SERIAL_VAL(hotends[h]->deg_target());
      firstOccurrence = false;
    }
    SERIAL_MSG("],\"state\":[");
    firstOccurrence = true;
    LOOP_HOTEND() {
      if (!firstOccurrence) SERIAL_CHR(',');
      SERIAL_CHR(hotends[h]->deg_target() > HOTEND_AUTO_FAN_TEMPERATURE ? '2' : '1');
      firstOccurrence = false;
    }

    SERIAL_MV("]}},\"time\":", HAL::timeInMilliseconds());

    switch (type) {
      case 0:
      case 1:
        break;
      case 2:
        SERIAL_EM(",");
        SERIAL_MSG("\"coldExtrudeTemp\":0,\"coldRetractTemp\":0.0,\"geometry\":\"");
        #if MECH(CARTESIAN)
          SERIAL_MSG("cartesian");
        #elif MECH(COREXY)
          SERIAL_MSG("corexy");
        #elif MECH(COREYX)
          SERIAL_MSG("coreyx");
        #elif MECH(COREXZ)
          SERIAL_MSG("corexz");
        #elif MECH(COREZX)
          SERIAL_MSG("corezx");
        #elif MECH(DELTA)
          SERIAL_MSG("delta");
        #endif
        SERIAL_MSG("\",\"name\":\"");
        SERIAL_MSG(CUSTOM_MACHINE_NAME);
        SERIAL_MSG("\",\"tools\":[");
        firstOccurrence = true;
        LOOP_EXTRUDER() {
          if (!firstOccurrence) SERIAL_CHR(',');
          SERIAL_MV("{\"number\":", i + 1);
          #if HOTENDS > 1
            SERIAL_MV(",\"hotends\":[", i + 1);
            SERIAL_MSG("],");
          #else
            SERIAL_MSG(",\"hotends\":[1],");
          #endif
          if (driver.e[i]) {
            SERIAL_MV("\"drives\":[", i);
            SERIAL_MSG("]");
          }
          SERIAL_MSG("}");
          firstOccurrence = false;
        }
        break;
      case 3:
        SERIAL_EM(",");
        SERIAL_MSG("\"printer.currentLayer\":");
        #if HAS_SD_SUPPORT
          if (IS_SD_PRINTING() && card.layerHeight > 0) { // ONLY CAN TELL WHEN SD IS PRINTING
            SERIAL_VAL((int) (mechanics.position.z / card.layerHeight));
          }
          else SERIAL_VAL(0);
        #else
          SERIAL_VAL(-1);
        #endif
        SERIAL_MSG(",\"extrRaw\":[");
        firstOccurrence = true;
        LOOP_EXTRUDER() {
          if (!firstOccurrence) SERIAL_CHR(',');
          SERIAL_VAL(mechanics.position.e * toolManager.flow_percentage[e]);
          firstOccurrence = false;
        }
        SERIAL_MSG("],");
        #if HAS_SD_SUPPORT
          if (IS_SD_PRINTING()) {
            SERIAL_MSG("\"fractionPrinted\":");
            float fractionprinted;
            if (card.fileSize < 2000000) {
              fractionprinted = (float)card.sdpos / (float)card.fileSize;
            }
            else fractionprinted = (float)(card.sdpos >> 8) / (float)(card.fileSize >> 8);
            SERIAL_VAL((float) floorf(fractionprinted * 1000) / 1000);
            SERIAL_CHR(',');
          }
        #endif
        SERIAL_MSG("\"firstLayerHeight\":");
        #if HAS_SD_SUPPORT
          if (IS_SD_PRINTING()) SERIAL_VAL(card.firstlayerHeight);
          else SERIAL_MSG("0");
        #else
          SERIAL_MSG("0");
        #endif
        break;
      case 4:
      case 5:
        SERIAL_EM(",");
        SERIAL_MSG("\"axisMins\":[");
        SERIAL_VAL((int) X_MIN_BED);
        SERIAL_CHR(',');
        SERIAL_VAL((int) Y_MIN_BED);
        SERIAL_CHR(',');
        SERIAL_VAL((int) Z_MIN_BED);
        SERIAL_MSG("],\"axisMaxes\":[");
        SERIAL_VAL((int) X_MAX_BED);
        SERIAL_CHR(',');
        SERIAL_VAL((int) Y_MAX_BED);
        SERIAL_CHR(',');
        SERIAL_VAL((int) Z_MAX_BED);
        SERIAL_MSG("],\"planner.accelerations\":[");
        SERIAL_VAL(mechanics.data.max_acceleration_mm_per_s2.x);
        SERIAL_CHR(',');
        SERIAL_VAL(mechanics.data.max_acceleration_mm_per_s2.y);
        SERIAL_CHR(',');
        SERIAL_VAL(mechanics.data.max_acceleration_mm_per_s2.z);
        LOOP_EXTRUDER() {
          SERIAL_CHR(',');
          SERIAL_VAL(extruders[e]->data.max_acceleration_mm_per_s2);
        }
        SERIAL_MSG("],");

        #if MB(ALLIGATOR_R2) || MB(ALLIGATOR_R3)
          SERIAL_MSG("\"currents\":[");
          SERIAL_VAL(driver.x->data.ma);
          SERIAL_CHR(',');
          SERIAL_VAL(driver.y->data.ma);
          SERIAL_CHR(',');
          SERIAL_VAL(driver.z->data.ma);
          LOOP_DRV_EXT() {
            SERIAL_CHR(',');
            SERIAL_VAL(driver.e[d]->data.ma);
          }
          SERIAL_EM("],");
        #endif

        SERIAL_MSG("\"firmwareElectronics\":\"");
        #if MB(RAMPS_13_HFB) || MB(RAMPS_13_HHB) || MB(RAMPS_13_HFF) || MB(RAMPS_13_HHF) || MB(RAMPS_13_HHH)
          SERIAL_MSG("RAMPS");
        #elif MB(ALLIGATOR_R2)
          SERIAL_MSG("ALLIGATOR_R2");
        #elif MB(ALLIGATOR_R3)
          SERIAL_MSG("ALLIGATOR_R3");
        #elif MB(RADDS) || MB(RAMPS_FD_V1) || MB(RAMPS_FD_V2) || MB(SMART_RAMPS) || MB(RAMPS4DUE)
          SERIAL_MSG("Arduino due");
        #elif MB(ULTRATRONICS)
          SERIAL_MSG("ULTRATRONICS");
        #else
          SERIAL_MSG("AVR");
        #endif
        SERIAL_MSG("\",\"firmwareName\":\"");
        SERIAL_MSG(FIRMWARE_NAME);
        SERIAL_MSG(",\"firmwareVersion\":\"");
        SERIAL_MSG(SHORT_BUILD_VERSION);
        SERIAL_MSG("\",\"firmwareDate\":\"");
        SERIAL_MSG(STRING_REVISION_DATE);

        SERIAL_MSG("\",\"minFeedrates\":[0,0,0");
        LOOP_EXTRUDER() {
          SERIAL_MSG(",0");
        }
        SERIAL_MSG("],\"maxFeedrates\":[");
        SERIAL_VAL(mechanics.data.max_feedrate_mm_s.x);
        SERIAL_CHR(',');
        SERIAL_VAL(mechanics.data.max_feedrate_mm_s.y);
        SERIAL_CHR(',');
        SERIAL_VAL(mechanics.data.max_feedrate_mm_s.z);
        LOOP_EXTRUDER()
          SERIAL_CHR(',');
          SERIAL_VAL(extruders[e]->data.max_feedrate_mm_s);
        }
        SERIAL_CHR(']');
        break;
    }
    SERIAL_CHR('}');
    SERIAL_EOL();
  }

#endif // ENABLED(JSON_OUTPUT)
