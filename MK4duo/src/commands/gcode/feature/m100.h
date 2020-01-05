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

/**
 * M100 Free Memory Watcher
 *
 * This code watches the free memory block between the bottom of the heap and the top of the stack.
 * This memory block is initialized and watched via the M100 command.
 *
 * M100 I   Initializes the free memory block and prints vitals statistics about the area
 *
 * M100 F   Identifies how much of the free memory block remains free and unused. It also
 *          detects and reports any corruption within the free memory block that may have
 *          happened due to errant firmware.
 *
 * M100 D   Does a hex display of the free memory block along with a flag for any errant
 *          data that does not match the expected value.
 *
 * M100 C x Corrupts x locations within the free memory block. This is useful to check the
 *          correctness of the M100 F and M100 D commands.
 *
 * Also, there are two support functions that can be called from a developer's C code.
 *
 *    uint16_t check_for_free_memory_corruption(PGM_P const start_free_memory);
 *    void M100_dump_routine(PGM_P const title, char *start, char *end);
 *
 * Initial version by Roxy-3D
 */

#if ENABLED(M100_FREE_MEMORY_WATCHER)

#define CODE_M100
#define TEST_BYTE ((char) 0xE5)

#ifdef ARDUINO_ARCH_SAM

  extern char _ebss;

  char  *end_bss            = &_ebss,
        *free_memory_start  = end_bss,
        *free_memory_end    = 0,
        *stacklimit         = 0,
        *heaplimit          = 0;

  #define MEMORY_END_CORRECTION 0x10000  // need to stay well below 0x20080000 or M100 F crashes

#else

  extern char __bss_end;

  char  *end_bss            = &__bss_end,
        *free_memory_start  = end_bss,
        *free_memory_end    = 0,
        *stacklimit         = 0,
        *heaplimit          = 0;

  #define MEMORY_END_CORRECTION 0

#endif

//
// Utility functions
//

// Location of a variable on its stack frame. Returns a value above
// the stack (once the function returns to the caller).
char* top_of_stack() {
  char x;
  return &x + 1; // x is pulled on return;
}

// Count the number of test bytes at the specified location.
inline int32_t count_test_bytes(const char * const start_free_memory) {
  for (uint32_t i = 0; i < 32000; i++)
    if (char(start_free_memory[i]) != TEST_BYTE)
      return i - 1;

  return -1;
}

//
// M100 sub-commands
//

#if ENABLED(M100_FREE_MEMORY_DUMPER)

  /**
   * M100 D
   *  Dump the free memory block from __brkval to the stack pointer.
   *  malloc() eats memory from the start of the block and the stack grows
   *  up from the bottom of the block. Solid test bytes indicate nothing has
   *  used that memory yet. There should not be anything but test bytes within
   *  the block. If so, it may indicate memory corruption due to a bad pointer.
   *  Unexpected bytes are flagged in the right column.
   */
  inline void dump_free_memory(char *start_free_memory, char *end_free_memory) {

    gcode_t tmp = commands.buffer_ring.peek();

    //
    // Start and end the dump on a nice 16 byte boundary
    // (even though the values are not 16-byte aligned).
    //
    start_free_memory = (char*)((ptr_int_t)((uint32_t)start_free_memory & 0xFFFFFFF0)); // Align to 16-byte boundary
    end_free_memory  = (char*)((ptr_int_t)((uint32_t)end_free_memory  | 0x0000000F));   // Align end_free_memory to the 15th byte (at or above end_free_memory)

    // Dump command main loop
    while (start_free_memory < end_free_memory) {
      print_hex_address(start_free_memory);             // Print the address
      SERIAL_CHR(':');
      for (uint8_t i = 0; i < 16; i++) {  // and 16 data bytes
        if (i == 8) SERIAL_CHR('-');
        print_hex_byte(start_free_memory[i]);
        SERIAL_CHR(' ');
      }
      HAL::delayMilliseconds(25);
      SERIAL_CHR('|');                      // Point out non test bytes
      for (uint8_t i = 0; i < 16; i++) {
        char ccc = (char)start_free_memory[i]; // cast to char before automatically casting to char on assignment, in case the compiler is broken
        if (&start_free_memory[i] >= (char*)tmp.gcode && &start_free_memory[i] < (char*)tmp.gcode + sizeof(tmp)) { // Print out ASCII in the command buffer area
          if (!WITHIN(ccc, ' ', 0x7E)) ccc = ' ';
        }
        else { // If not in the command buffer area, flag bytes that don't match the test byte
          ccc = (ccc == TEST_BYTE) ? ' ' : '?';
        }
        SERIAL_CHR(ccc);
      }
      SERIAL_EOL();
      start_free_memory += 16;
      HAL::delayMilliseconds(25);
      printer.idle();
    }
  }

  void M100_dump_routine(PGM_P const title, char *start, char *end) {
    SERIAL_ET(title);
    //
    // Round the start and end locations to produce full lines of output
    //
    start = (char*)((ptr_int_t)((uint32_t)start & 0xFFFFFFF0)); // Align to 16-byte boundary
    end   = (char*)((ptr_int_t)((uint32_t)end   | 0x0000000F)); // Align sp to the 15th byte (at or above sp)
    dump_free_memory(start, end);
  }

#endif // M100_FREE_MEMORY_DUMPER

inline int check_for_free_memory_corruption(PGM_P const title) {
  SERIAL_STR(title);

  char  *start_free_memory = free_memory_start,
        *end_free_memory = free_memory_end;
  int n = end_free_memory - start_free_memory;

  SERIAL_MV("\nfmc() n=", n);
  SERIAL_MV("\nfree_memory_start=", hex_address(free_memory_start));
  SERIAL_EMV("  end_free_memory=",  hex_address(end_free_memory));

  if (end_free_memory < start_free_memory)  {
    SERIAL_MSG(" end_free_memory < Heap ");
    // SET_INPUT_PULLUP(63);          // if the developer has a switch wired up to their controller board
    // HAL::delayMilliseconds(5);     // this code can be enabled to pause the display as soon as the
    // while ( READ(63))              // malfunction is detected.   It is currently defaulting to a switch
    //   printer.idle();              // being on pin-63 which is unassigend and available on most controller
    // HAL::delayMilliseconds(20);    // boards.
    // while ( !READ(63))
    //   printer.idle();
    HAL::delayMilliseconds(20);
    #if ENABLED(M100_FREE_MEMORY_DUMPER)
      M100_dump_routine(PSTR("   Memory corruption detected with end_free_memory<Heap\n"), (char*)0x1B80, (char*)0x21FF);
    #endif
  }

  // Scan through the range looking for the biggest block of 0xE5's we can find
  int block_cnt = 0;
  for (int i = 0; i < n; i++) {
    if (start_free_memory[i] == TEST_BYTE) {
      int32_t j = count_test_bytes(start_free_memory + i);
      if (j > 8) {
        // SERIAL_MV("Found ", j);
        // SERIAL_EMV(" bytes free at ", hex_address(start_free_memory + i));
        i += j;
        block_cnt++;
        SERIAL_MV(" (", block_cnt);
        SERIAL_MV(") found=", j);
        SERIAL_EM("   ");
      }
    }
  }
  SERIAL_MV("  block_found=", block_cnt);

  if (block_cnt != 1)
    SERIAL_EM("\nMemory Corruption detected in free memory area.");

  if (block_cnt == 0)       // Make sure the special case of no free blocks shows up as an
    block_cnt = -1;         // error to the calling code!

  SERIAL_MSG(" return=");
  if (block_cnt == 1) {
    SERIAL_CHR('0');        // if the block_cnt is 1, nothing has broken up the free memory
    SERIAL_EOL();             // area and it is appropriate to say 'no corruption'.
    return 0;
  }
  SERIAL_EM("true");
  return block_cnt;
}

/**
 * M100 F
 *  Return the number of free bytes in the memory pool,
 *  with other vital statistics defining the pool.
 */
inline void free_memory_pool_report(char * const start_free_memory, const int32_t size) {
  int32_t max_cnt = -1, block_cnt = 0;
  char *max_addr = nullptr;
  // Find the longest block of test bytes in the buffer
  for (int32_t i = 0; i < size; i++) {
    char *addr = start_free_memory + i;
    if (*addr == TEST_BYTE) {
      const int32_t j = count_test_bytes(addr);
      if (j > 8) {
        SERIAL_MV("Found ", j);
        SERIAL_EMV(" bytes free at ", hex_address(addr));
        if (j > max_cnt) {
          max_cnt  = j;
          max_addr = addr;
        }
        i += j;
        block_cnt++;
      }
    }
  }
  if (block_cnt > 1) {
    SERIAL_EM("\nMemory Corruption detected in free memory area.");
    SERIAL_MV("\nLargest free block is ", max_cnt);
    SERIAL_EMV(" bytes at ", hex_address(max_addr));
  }
  const int fmc = check_for_free_memory_corruption("M100 F ");
  SERIAL_EMV("check_for_free_memory_corruption() = ", fmc);
}

#if ENABLED(M100_FREE_MEMORY_CORRUPTOR)
  /**
   * M100 C<num>
   *  Corrupt <num> locations in the free memory pool and report the corrupt addresses.
   *  This is useful to check the correctness of the M100 D and the M100 F commands.
   */
  inline void corrupt_free_memory(char *start_free_memory, const uint32_t size) {
    start_free_memory += 8;
    const uint32_t near_top = top_of_stack() - start_free_memory - 250, // -250 to avoid interrupt activity that's altered the stack.
                   j = near_top / (size + 1);

    SERIAL_EM("Corrupting free memory block.\n");
    for (uint32_t i = 1; i <= size; i++) {
      char * const addr = start_free_memory + i * j;
      *addr = i;
      SERIAL_MV("\nCorrupting address: ", hex_address(addr));
    }
    SERIAL_EOL();
  }
#endif // M100_FREE_MEMORY_CORRUPTOR

/**
 * M100 I
 *  Init memory for the M100 tests. (Automatically applied on the first M100.)
 */
inline void init_free_memory(char *start_free_memory, int32_t size) {
  SERIAL_EM("Initializing free memory block.\n\n");

  size -= 250;    // -250 to avoid interrupt activity that's altered the stack.
  if (size < 0) {
    SERIAL_EM("Unable to initialize.\n");
    return;
  }

  start_free_memory += 8;       // move a few bytes away from the heap just because we don't want
                  // to be altering memory that close to it.
  memset(start_free_memory, TEST_BYTE, size);

  SERIAL_VAL(size);
  SERIAL_EM(" bytes of memory initialized.\n");

  for (int32_t i = 0; i < size; i++) {
    if (start_free_memory[i] != TEST_BYTE) {
      SERIAL_MV("? address : ", hex_address(start_free_memory + i));
      SERIAL_EMV("=", hex_byte(start_free_memory[i]));
    }
  }
}

/**
 * M100: Free Memory Check
 */
inline void gcode_M100() {

  char *sp = top_of_stack();
  if (!free_memory_end) free_memory_end = sp - MEMORY_END_CORRECTION;
  SERIAL_MV("\nbss_end               : ", hex_address(end_bss));
  if (heaplimit) SERIAL_MV("\n__heaplimit           : ", hex_address(heaplimit ));
  SERIAL_MV("\nfree_memory_start     : ", hex_address(free_memory_start));
  if (stacklimit) SERIAL_MV("\n__stacklimit          : ", hex_address(stacklimit));
  SERIAL_MV("\nfree_memory_end       : ", hex_address(free_memory_end  ));
  if (MEMORY_END_CORRECTION)  SERIAL_MV("\nMEMORY_END_CORRECTION: ", MEMORY_END_CORRECTION );
  SERIAL_EMV("\nStack Pointer         : ", hex_address(sp));

  // Always init on the first invocation of M100
  static bool m100_not_initialized = true;
  if (m100_not_initialized || parser.seen('I')) {
    m100_not_initialized = false;
    init_free_memory(free_memory_start, free_memory_end - free_memory_start);
  }

  #if ENABLED(M100_FREE_MEMORY_DUMPER)
    if (parser.seen('D'))
      return dump_free_memory(free_memory_start, free_memory_end);
  #endif

  if (parser.seen('F'))
    return free_memory_pool_report(free_memory_start, free_memory_end - free_memory_start);

  #if ENABLED(M100_FREE_MEMORY_CORRUPTOR)

    if (parser.seen('C'))
      return corrupt_free_memory(free_memory_start, parser.value_int());

  #endif
}

#endif // M100_FREE_MEMORY_WATCHER
