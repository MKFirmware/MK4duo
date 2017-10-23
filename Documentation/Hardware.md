# Supported Hardware

__Please note:__ We need more feedback from users weather their board is actually working!

Supported boards are listed under: [/MK3duo/boards.h]


##### RAMPS

http://reprap.org/wiki/RAMPS

```
#define BOARD_RAMPS_OLD         3    // MEGA/RAMPS up to 1.2
#define BOARD_RAMPS_13_EFB      33   // RAMPS 1.3 / 1.4 (Power outputs: Extruder, Fan, Bed)
#define BOARD_RAMPS_13_EEB      34   // RAMPS 1.3 / 1.4 (Power outputs: Extruder0, Extruder1, Bed)
#define BOARD_RAMPS_13_EFF      35   // RAMPS 1.3 / 1.4 (Power outputs: Extruder, Fan, Fan)
#define BOARD_RAMPS_13_EEF      36   // RAMPS 1.3 / 1.4 (Power outputs: Extruder0, Extruder1, Fan)
```

##### Generation 3 Electronics

http://reprap.org/wiki/Generation_3_Electronics

```
#define BOARD_GEN3_PLUS         9    // Gen3+
#define BOARD_GEN3_MONOLITHIC   22   // Gen3 Monolithic Electronics
```

##### Generation 6 Electronics

http://reprap.org/wiki/Generation_6_Electronics

```
#define BOARD_GEN6              5    // Gen6
#define BOARD_GEN6_DELUXE       51   // Gen6 deluxe
```

##### Generation 7 Electronics

http://reprap.org/wiki/Generation_7_Electronics

```
#define BOARD_GEN7_CUSTOM       10   // Gen7 custom (Alfons3 Version) "https://github.com/Alfons3/Generation_7_Electronics"
#define BOARD_GEN7_12           11   // Gen7 v1.1, v1.2
#define BOARD_GEN7_13           12   // Gen7 v1.3
#define BOARD_GEN7_14           13   // Gen7 v1.4
```


```
#define BOARD_CHEAPTRONIC       2    // Cheaptronic v1.0
#define BOARD_SETHI             20   // Sethi 3D_1
```

##### Sanguinololu

http://reprap.org/wiki/Sanguinololu

```
#define BOARD_SANGUINOLOLU_11   6    // Sanguinololu < 1.2
#define BOARD_SANGUINOLOLU_12   62   // Sanguinololu 1.2 and above

```

##### Melzi

http://reprap.org/wiki/Melzi

```
#define BOARD_MELZI             63   // Melzi
#define BOARD_MELZI_1284        66   // Melzi with ATmega1284 (MaKr3d version)
```

##### RUMBA

http://reprap.org/wiki/RUMBA

```
#define BOARD_RUMBA             80   // Rumba
```

##### Azteeg

  - http://reprap.org/wiki/Azteeg_X1
  - http://reprap.org/wiki/Azteeg_X3


```
#define BOARD_AZTEEG_X1         65   // Azteeg X1
#define BOARD_AZTEEG_X3         67   // Azteeg X3
#define BOARD_AZTEEG_X3_PRO     68   // Azteeg X3 Pro
```

#### Others

```
#define BOARD_DUEMILANOVE_328P  4    // Duemilanove w/ ATMega328P pin assignment
#define BOARD_STB_11            64   // STB V1.1
#define BOARD_ULTIMAKER         7    // Ultimaker
#define BOARD_ULTIMAKER_OLD     71   // Ultimaker (Older electronics. Pre 1.5.4. This is rare)
#define BOARD_ULTIMAIN_2        72   // Ultimainboard 2.x (Uses TEMP_SENSOR 20)
#define BOARD_3DRAG             77   // 3Drag Controller
#define BOARD_TEENSYLU          8    // Teensylu
#define BOARD_PRINTRBOARD       81   // Printrboard (AT90USB1286)
#define BOARD_BRAINWAVE         82   // Brainwave (AT90USB646)
#define BOARD_SAV_MKI           83   // SAV Mk-I (AT90USB1286)
#define BOARD_TEENSY2           84   // Teensy++2.0 (AT90USB1286) - CLI compile: DEFINES=AT90USBxx_TEENSYPP_ASSIGNMENTS HARDWARE_MOTHERBOARD=84  make
#define BOARD_MEGATRONICS       70   // Megatronics
#define BOARD_MEGATRONICS_2     701  // Megatronics v2.0
#define BOARD_MINITRONICS       702  // Minitronics v1.0
#define BOARD_MEGATRONICS_3     703  // Megatronics v3.0
#define BOARD_OMCA_A            90   // Alpha OMCA board
#define BOARD_OMCA              91   // Final OMCA board
#define BOARD_RAMBO             301  // Rambo
#define BOARD_ELEFU_3           21   // Elefu Ra Board (v3)
#define BOARD_5DPRINT           88   // 5DPrint D8 Driver Board
#define BOARD_LEAPFROG          999  // Leapfrog
```

## SAM3X8E processor

##### Piccolo 3D

```
#define BOARD_PICCOLO_3D      1400    // PICCOLO_3D ARM 32 Arduino DUE
```

##### RADDS

http://doku.radds.org/dokumentation/radds/

```
#define BOARD_RADDS           1401    // RADDS ARM 32 bit board
```

##### RAMPS FD V1 and V2

http://reprap.org/wiki/RAMPS-FD

```
#define BOARD_RAMPS_FD_V1     1403    // RAMPS-FD V1 ARM 32 bit board
#define BOARD_RAMPS_FD_V2     1404    // RAMPS-FD V2 ARM 32 bit board
```

##### RAMPS SMART

http://reprap.org/wiki/SMART_RAMPS

```
#define BOARD_RAMPS_SMART_HFB 1411    // RAMPS-SMART (Power outputs: Hotend, Fan, Bed)
#define BOARD_RAMPS_SMART_HHB 1412    // RAMPS-SMART (Power outputs: Hotend0, Hotend1, Bed)
#define BOARD_RAMPS_SMART_HFF 1413    // RAMPS-SMART (Power outputs: Hotend, Fan0, Fan1)
#define BOARD_RAMPS_SMART_HHF 1414    // RAMPS-SMART (Power outputs: Hotend0, Hotend1, Fan)
```

##### RAMPS4DUE

http://forums.reprap.org/read.php?219,479626,page=1

```
#define BOARD_RAMPS4DUE       1433    // RAMPS4DUE with AndrewBCN's RAMPS mods
```

##### ALLIGATOR

http://www.3dartists.org/

```
#define BOARD_ALLIGATOR       1502    // ALLIGATOR R2 ARM 32 bit board
#define BOARD_ALLIGATOR_V3    1503    // ALLIGATOR R3 ARM 32 bit board
```

##### ULTRATRONICS

https://reprapworld.com/documentation/datasheet_ultratronics10_05.pdf

```
#define BOARD_ULTRATRONICS    1705    // Ultratronics v1.0 ARM 32 bit board
```
