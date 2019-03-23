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
 * fwretract.cpp - Implement firmware-based retraction
 *
 * Copyright (C) 2019 Alberto Cotronei @MagoKimbra
 */

#include "../../../MK4duo.h"

#if ENABLED(FWRETRACT)

  FWRetract fwretract;

  // private:
  #if EXTRUDERS > 1
    bool FWRetract::retracted_swap[EXTRUDERS];      // Which extruders are swap-retracted
  #endif

  // public:

  fwretract_data_t FWRetract::data;                 // M207 S F Z W, M208 S F W R

  bool  FWRetract::autoretract_enabled,             // M209 S - Autoretract switch
        FWRetract::retracted[EXTRUDERS];            // Which extruders are currently retracted
  float FWRetract::current_retract[EXTRUDERS],      // Retract value used by planner
        FWRetract::current_hop;

  void FWRetract::reset() {
    autoretract_enabled                     = false;
    data.retract_length                     = RETRACT_LENGTH;
    data.retract_feedrate_mm_s              = RETRACT_FEEDRATE;
    data.retract_zlift                      = RETRACT_ZLIFT;
    data.retract_recover_length             = RETRACT_RECOVER_LENGTH;
    data.retract_recover_feedrate_mm_s      = RETRACT_RECOVER_FEEDRATE;
    data.swap_retract_length                = RETRACT_LENGTH_SWAP;
    data.swap_retract_recover_length        = RETRACT_RECOVER_LENGTH_SWAP;
    data.swap_retract_recover_feedrate_mm_s = RETRACT_RECOVER_FEEDRATE_SWAP;
    current_hop                             = 0.0;

    for (uint8_t e = 0; e < EXTRUDERS; ++e) {
      retracted[e] = false;
      #if EXTRUDERS > 1
        retracted_swap[e] = false;
      #endif
      current_retract[e] = 0.0;
    }
  }

  /**
   * Retract or recover according to firmware settings
   *
   * This function handles retract/recover moves for G10 and G11,
   * plus auto-retract moves sent from G0/G1 when E-only moves are done.
   *
   * To simplify the logic, doubled retract/recover moves are ignored.
   *
   * Note: Z lift is done transparently to the planner. Aborting
   *       a print between G10 and G11 may corrupt the Z position.
   *
   * Note: Auto-retract will apply the set Z hop in addition to any Z hop
   *       included in the G-code. Use M207 Z0 to to prevent double hop.
   */
  void FWRetract::retract(const bool retracting
    #if EXTRUDERS > 1
      , bool swapping /* =false */
    #endif
  ) {

    // Simply never allow two retracts or recovers in a row
    if (retracted[tools.active_extruder] == retracting) return;

    #if EXTRUDERS > 1
      // Allow G10 S1 only after G10
      if (swapping && retracted_swap[tools.active_extruder] == retracting) return;
      // G11 priority to recover the long retract if activated
      if (!retracting) swapping = retracted_swap[tools.active_extruder];
    #else
      constexpr bool swapping = false;
    #endif

    const float old_feedrate_mm_s = mechanics.feedrate_mm_s,
                unscale_e = RECIPROCAL(tools.e_factor[tools.active_extruder]),
                unscale_fr = 100.0 / mechanics.feedrate_percentage, // Disable feedrate scaling for retract moves
                base_retract = swapping ? data.swap_retract_length : data.retract_length;

    // The current position will be the destination for E and Z moves
    mechanics.set_destination_to_current();

    if (retracting) {
      // Retract by moving from a faux E position back to the current E position
      mechanics.feedrate_mm_s = data.retract_feedrate_mm_s * unscale_fr;
      current_retract[tools.active_extruder] = base_retract * unscale_e;
      mechanics.prepare_move_to_destination();  // set_current_to_destination
      planner.synchronize();                    // Wait for move to complete

      // Is a Z hop set, and has the hop not yet been done?
      if (data.retract_zlift > 0.01 && !current_hop) {   // Apply hop only once
        current_hop += data.retract_zlift;               // Add to the hop total (again, only once)
        mechanics.feedrate_mm_s = mechanics.data.max_feedrate_mm_s[Z_AXIS] * unscale_fr; // Maximum Z feedrate
        mechanics.prepare_move_to_destination();    // Raise up, set_current_to_destination
        planner.synchronize();                      // Wait for move to complete
      }
    }
    else {
      // If a hop was done and Z hasn't changed, undo the Z hop
      if (current_hop) {
        current_hop = 0.0;
        mechanics.feedrate_mm_s = mechanics.data.max_feedrate_mm_s[Z_AXIS] * unscale_fr; // Z feedrate to max
        mechanics.prepare_move_to_destination();    // Lower Z and update current_position
        planner.synchronize();                      // Wait for move to complete
      }

      const float extra_recover = swapping ? data.swap_retract_recover_length : data.retract_recover_length;
      if (extra_recover != 0.0) {
        mechanics.current_position[E_AXIS] -= extra_recover;  // Adjust the current E position by the extra amount to recover
        mechanics.sync_plan_position_e();                     // Sync the planner position so the extra amount is recovered
      }

      current_retract[tools.active_extruder] = 0.0;
      mechanics.feedrate_mm_s = (swapping ? data.swap_retract_recover_feedrate_mm_s : data.retract_recover_feedrate_mm_s) * unscale_fr;
      mechanics.prepare_move_to_destination();                // Recover E, set_current_to_destination
      planner.synchronize();                                  // Wait for move to complete
    }

    mechanics.feedrate_mm_s = old_feedrate_mm_s;              // Restore original feedrate
    retracted[tools.active_extruder] = retracting;            // Active extruder now retracted / recovered

    // If swap retract/recover then update the retracted_swap flag too
    #if EXTRUDERS > 1
      if (swapping) retracted_swap[tools.active_extruder] = retracting;
    #endif

  }

#endif // FWRETRACT
