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

#ifndef _EEPROM_H_
#define _EEPROM_H_

class EEPROM {

  public: /** Constructor */

    EEPROM() {}

  public: /** Public Parameters */

  private: /** Private Parameters */

    #if HAS_EEPROM

      static bool eeprom_error;
 
      #if ENABLED(AUTO_BED_LEVELING_UBL) // Eventually make these available if any leveling system
                                         // That can store is enabled
        static uint16_t meshes_begin;
        static constexpr uint16_t meshes_end = E2END - 128; // 128 is a placeholder for the size of the MAT; the MAT will always
                                                            // live at the very end of the eeprom

      #endif
    #endif

  public: /** Public Function */

    FORCE_INLINE static bool Init() {
      bool success = true;
      Factory_Settings();
      #if HAS_EEPROM
        if ((success = Store_Settings())) {
          #if ENABLED(AUTO_BED_LEVELING_UBL)
            success = Load_Settings(); // UBL uses load() to know the end of EEPROM
          #elif ENABLED(EEPROM_CHITCHAT)
            Print_Settings();
          #endif
        }
      #endif
      return success;
    }

    static void Factory_Settings();
    static bool Store_Settings();

    #if HAS_EEPROM
      static bool Load_Settings();  // Return 'true' if data was loaded ok

      #if ENABLED(AUTO_BED_LEVELING_UBL) // Eventually make these available if any leveling system
                                         // That can store is enabled
        FORCE_INLINE static uint16_t  meshes_start_index()  { return meshes_begin; }
        FORCE_INLINE static uint16_t  meshes_end_index()    { return meshes_end; }
        static uint16_t calc_num_meshes();
        static int mesh_slot_offset(const int8_t slot);
        static void store_mesh(const int8_t slot);
        static void load_mesh(const int8_t slot, void * const into=NULL);

        //static void delete_mesh();    // necessary if we have a MAT
        //static void defrag_meshes();  // "
      #endif
    #else
      FORCE_INLINE static bool Load_Settings() { Factory_Settings(); Print_Settings(); return true; }
    #endif

    #if DISABLED(DISABLE_M503)
      static void Print_Settings();
    #else
      FORCE_INLINE static void Print_Settings() { }
    #endif

  private: /** Private Function */

    static void Postprocess();

};

extern EEPROM eeprom;

#endif /* _EEPROM_H_ */
