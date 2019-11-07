/**
 * MK4duo Firmware for 3D Printer, Laser and CNC
 *
 * Based on Marlin, Sprinter and grbl
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2013 Alberto Cotronei @MagoKimbra
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
 * flowmeter.cpp - Flowmeter control library for Arduino - Version 1
 * Copyright (c) 2016 Franco (nextime) Lanza.  All right reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "../../../MK4duo.h"
#include "sanitycheck.h"

#if ENABLED(FLOWMETER_SENSOR)

FlowMeter flowmeter;

/** Public Parameters */
bool      FlowMeter::flow_firstread       = false;
float     FlowMeter::flowrate             = 0.0;
int       FlowMeter::flowrate_pulsecount  = 0;

/** Private Parameters */
millis_l  FlowMeter::lastflow             = 0;

/** Public Function */
void flowrate_pulsecounter() {
  // Increment the pulse counter
  flowmeter.flowrate_pulsecount++;
}

void FlowMeter::init() {
  flowrate = 0;
  flowrate_pulsecount = 0;
  HAL::pinMode(FLOWMETER_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(FLOWMETER_PIN), flowrate_pulsecounter, FALLING);
}

void FlowMeter::spin() {

  millis_l now = millis();

  detachInterrupt(digitalPinToInterrupt(FLOWMETER_PIN));
  flowrate  = (float)(((1000.0 / (float)((float)now - (float)lastflow)) * (float)flowrate_pulsecount) / (float)FLOWMETER_CALIBRATION);
  #if ENABLED(FLOWMETER_DEBUG)
    SERIAL_SM(DEB, "FLOWMETER DEBUG ");
    SERIAL_MV(" flowrate:", flowrate);
    SERIAL_MV(" flowrate_pulsecount:", flowrate_pulsecount);
    SERIAL_EMV(" CALIBRATION:", FLOWMETER_CALIBRATION);
  #endif
  lastflow = now;
  flowrate_pulsecount = 0;
  attachInterrupt(digitalPinToInterrupt(FLOWMETER_PIN), flowrate_pulsecounter, FALLING);

  #if ENABLED(MINFLOW_PROTECTION)
    if (flow_firstread && print_job_counter.isRunning() && (flowrate < (float)MINFLOW_PROTECTION)) {
      flow_firstread = false;
      printer.kill(GET_TEXT(MSG_KILLED));
    }
  #endif

}

void FlowMeter::print_flowrate() {
  #if ENABLED(MINFLOW_PROTECTION)
    if (flowrate >= (float)MINFLOW_PROTECTION) flow_firstread = true;
  #endif
  SERIAL_MV(" FLOW: ", flowrate, 3);
  SERIAL_MSG(" l/min ");
}

#endif // FLOWMETER_SENSOR
