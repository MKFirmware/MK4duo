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
 * mcode
 *
 * Copyright (C) 2017 Alberto Cotronei @MagoKimbra
 */

#if ENABLED(JSON_OUTPUT)

  #define CODE_M408

  /**
   * M408: JSON STATUS OUTPUT
   */
  inline void gcode_M408(void) {
    bool firstOccurrence;
    uint8_t type = 0;

    if (parser.seen('S')) type = parser.value_byte();

    SERIAL_MSG("{\"status\":\"");
    #if HAS_SDSUPPORT
      if (!print_job_counter.isRunning() && !card.sdprinting) SERIAL_CHR('I'); // IDLING
      else if (card.sdprinting) SERIAL_CHR('P');          // SD PRINTING
      else SERIAL_MSG("B");                               // SOMETHING ELSE, BUT SOMETHIG
    #else
      if (!print_job_counter.isRunning()) SERIAL_CHR('I');                     // IDLING
      else SERIAL_CHR('B');                               // SOMETHING ELSE, BUT SOMETHIG
    #endif

    SERIAL_MSG("\",\"coords\": {");
    SERIAL_MSG("\"axesHomed\":[");
    if (mechanics.axis_homed[X_AXIS] && mechanics.axis_homed[Y_AXIS] && mechanics.axis_homed[Z_AXIS])
      SERIAL_MSG("1, 1, 1");
    else
      SERIAL_MSG("0, 0, 0");

    SERIAL_MV("],\"extr\":[", mechanics.current_position[E_AXIS]);
    SERIAL_MV("],\"xyz\":[", mechanics.current_position[X_AXIS]); // X AXIS
    SERIAL_MV(",", mechanics.current_position[Y_AXIS]);           // Y AXIS
    SERIAL_MV(",", mechanics.current_position[Z_AXIS]);           // Z AXIS

    SERIAL_MV("]},\"currentTool\":", tools.active_extruder);

    #if HAS_POWER_SWITCH
      SERIAL_MSG(",\"params\": {\"atxPower\":");
      SERIAL_CHR(powerManager.powersupply_on ? '1' : '0');
    #else
      SERIAL_MSG(",\"params\": {\"NormPower\":");
    #endif

    #if FAN_COUNT > 0
      SERIAL_MSG(",\"fanPercent\":[");
      SERIAL_VAL(fans[0].Speed);
    #endif

    SERIAL_MV("],\"speedFactor\":", mechanics.feedrate_percentage);

    SERIAL_MSG(",\"extrFactors\":[");
    firstOccurrence = true;
    for (uint8_t e = 0; e < EXTRUDERS; e++) {
      if (!firstOccurrence) SERIAL_CHR(',');
      SERIAL_VAL(tools.flow_percentage[e]);
      firstOccurrence = false;
    }
    SERIAL_EM("]},");

    SERIAL_MSG("\"temps\": {");
    #if HAS_TEMP_BED
      SERIAL_MV("\"bed\": {\"current\":", heaters[BED_INDEX].current_temperature, 1);
      SERIAL_MV(",\"active\":", heaters[BED_INDEX].target_temperature);
      SERIAL_MSG(",\"state\":");
      SERIAL_CHR(heaters[BED_INDEX].target_temperature > 0 ? '2' : '1');
      SERIAL_MSG("},");
    #endif
    SERIAL_MSG("\"heads\": {\"current\":[");
    firstOccurrence = true;
    for (int8_t h = 0; h < HOTENDS; h++) {
      if (!firstOccurrence) SERIAL_CHR(',');
      SERIAL_VAL(heaters[h].current_temperature, 1);
      firstOccurrence = false;
    }
    SERIAL_MSG("],\"active\":[");
    firstOccurrence = true;
    for (int8_t h = 0; h < HOTENDS; h++) {
      if (!firstOccurrence) SERIAL_CHR(',');
      SERIAL_VAL(heaters[h].target_temperature);
      firstOccurrence = false;
    }
    SERIAL_MSG("],\"state\":[");
    firstOccurrence = true;
    for (int8_t h = 0; h < HOTENDS; h++) {
      if (!firstOccurrence) SERIAL_CHR(',');
      SERIAL_CHR(heaters[h].target_temperature > HOTEND_AUTO_FAN_TEMPERATURE ? '2' : '1');
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
        for (uint8_t i = 0; i < EXTRUDERS; i++) {
          if (!firstOccurrence) SERIAL_CHR(',');
          SERIAL_MV("{\"number\":", i + 1);
          #if HOTENDS > 1
            SERIAL_MV(",\"heaters\":[", i + 1);
            SERIAL_MSG("],");
          #else
            SERIAL_MSG(",\"heaters\":[1],");
          #endif
          #if DRIVER_EXTRUDERS > 1
            SERIAL_MV("\"drives\":[", i);
            SERIAL_MSG("]");
          #else
            SERIAL_MSG("\"drives\":[0]");
          #endif
          SERIAL_MSG("}");
          firstOccurrence = false;
        }
        break;
      case 3:
        SERIAL_EM(",");
        SERIAL_MSG("\"printer.currentLayer\":");
        #if HAS_SDSUPPORT
          if (card.sdprinting && card.layerHeight > 0) { // ONLY CAN TELL WHEN SD IS PRINTING
            SERIAL_VAL((int) (mechanics.current_position[Z_AXIS] / card.layerHeight));
          }
          else SERIAL_VAL(0);
        #else
          SERIAL_VAL(-1);
        #endif
        SERIAL_MSG(",\"extrRaw\":[");
        firstOccurrence = true;
        for (uint8_t e = 0; e < EXTRUDERS; e++) {
          if (!firstOccurrence) SERIAL_CHR(',');
          SERIAL_VAL(mechanics.current_position[E_AXIS] * tools.flow_percentage[e]);
          firstOccurrence = false;
        }
        SERIAL_MSG("],");
        #if HAS_SDSUPPORT
          if (card.sdprinting) {
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
        #if HAS_SDSUPPORT
          if (card.sdprinting) SERIAL_VAL(card.firstlayerHeight);
          else SERIAL_MSG("0");
        #else
          SERIAL_MSG("0");
        #endif
        break;
      case 4:
      case 5:
        SERIAL_EM(",");
        SERIAL_MSG("\"axisMins\":[");
        SERIAL_VAL((int) X_MIN_POS);
        SERIAL_CHR(',');
        SERIAL_VAL((int) Y_MIN_POS);
        SERIAL_CHR(',');
        SERIAL_VAL((int) Z_MIN_POS);
        SERIAL_MSG("],\"axisMaxes\":[");
        SERIAL_VAL((int) X_MAX_POS);
        SERIAL_CHR(',');
        SERIAL_VAL((int) Y_MAX_POS);
        SERIAL_CHR(',');
        SERIAL_VAL((int) Z_MAX_POS);
        SERIAL_MSG("],\"planner.accelerations\":[");
        SERIAL_VAL(mechanics.max_acceleration_mm_per_s2[X_AXIS]);
        SERIAL_CHR(',');
        SERIAL_VAL(mechanics.max_acceleration_mm_per_s2[Y_AXIS]);
        SERIAL_CHR(',');
        SERIAL_VAL(mechanics.max_acceleration_mm_per_s2[Z_AXIS]);
        for (uint8_t i = 0; i < EXTRUDERS; i++) {
          SERIAL_CHR(',');
          SERIAL_VAL(mechanics.max_acceleration_mm_per_s2[E_AXIS + i]);
        }
        SERIAL_MSG("],");

        #if MB(ALLIGATOR) || MB(ALLIGATOR_V3)
          SERIAL_MSG("\"currents\":[");
          SERIAL_VAL(printer.motor_current[X_AXIS]);
          SERIAL_CHR(',');
          SERIAL_VAL(printer.motor_current[Y_AXIS]);
          SERIAL_CHR(',');
          SERIAL_VAL(printer.motor_current[Z_AXIS]);
          for (uint8_t i = 0; i < DRIVER_EXTRUDERS; i++) {
            SERIAL_CHR(',');
            SERIAL_VAL(printer.motor_current[E_AXIS + i]);
          }
          SERIAL_EM("],");
        #endif

        SERIAL_MSG("\"firmwareElectronics\":\"");
        #if MB(RAMPS_13_HFB) || MB(RAMPS_13_HHB) || MB(RAMPS_13_HFF) || MB(RAMPS_13_HHF) || MB(RAMPS_13_HHH)
          SERIAL_MSG("RAMPS");
        #elif MB(ALLIGATOR)
          SERIAL_MSG("ALLIGATOR");
        #elif MB(ALLIGATOR_V3)
          SERIAL_MSG("ALLIGATOR_V3");
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
        SERIAL_MSG(STRING_DISTRIBUTION_DATE);

        SERIAL_MSG("\",\"minFeedrates\":[0,0,0");
        for (uint8_t i = 0; i < EXTRUDERS; i++) {
          SERIAL_MSG(",0");
        }
        SERIAL_MSG("],\"maxFeedrates\":[");
        SERIAL_VAL(mechanics.max_feedrate_mm_s[X_AXIS]);
        SERIAL_CHR(',');
        SERIAL_VAL(mechanics.max_feedrate_mm_s[Y_AXIS]);
        SERIAL_CHR(',');
        SERIAL_VAL(mechanics.max_feedrate_mm_s[Z_AXIS]);
        for (uint8_t i = 0; i < EXTRUDERS; i++) {
          SERIAL_CHR(',');
          SERIAL_VAL(mechanics.max_feedrate_mm_s[E_AXIS + i]);
        }
        SERIAL_CHR(']');
        break;
    }
    SERIAL_CHR('}');
    SERIAL_EOL();
  }

#endif // ENABLED(JSON_OUTPUT)
