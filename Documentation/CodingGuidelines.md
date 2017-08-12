# MK4duo Coding Guidelines
_written by [iosonopersia](https://github.com/iosonopersia), August 2017_

### Introduction
**MK4duo** is a _Marlin_ fork that wants to provide a cleaner, simpler to understand code to its user-base. It was the first Marlin fork to exploit the possibility of _recursive subfolder compilation_ offered by the Arduino IDE starting from version 1.6.3.

Code was separated between different subfolders in order to make clear to the programmers which thematic group a file belongs to.

The work needed to clean-up **MK4duo** code isn't finished yet: this is the reason why it's very important that everyone who's willing to contribute to this project knows well **how** we want to achieve that goal.

_Remember: a cleaner code is a **more robust** code!_

### Memory consumption and coding standards
Try to minimize the number of variables used and to define them inside the function scope: this way, when the function will return, that memory will be freed. For coding standards you can refer to the good _Marlin_ documentation written by [thinkyhead](https://github.com/thinkyhead): [Coding Standards](https://github.com/MarlinFirmware/MarlinDocumentation/blob/master/_development/coding_standards.md).

### Subfolders
**MK4duo** code is spread across a lot of **src/** subfolders. Each one is named after its role in the code: this makes really intuitive finding the part of the code you think should be modified. Dividing code files in subfolders is an important step in achieving our goal but we shouldn't abuse of this method: **a lot of folders are confusing just like a lot of files!**

**As for now**, we think we've reached a good balance. If you think something should be changed, feel free to contact the developers!

### Code refactoring
Digging deeper in **MK4duo** code, we encounter the problem of code refactoring. **MK4duo** contains a lot of functions: following the C coding style won't help us cleaning our code. Actually, we need to provide a gerarchical code structure to our functions. Two main possibilities are offered by the C++ language: **namespaces** and **classes**.

Since our need is simply to collect functions in thematic groups, the better way would be to use **namespaces**. The only problem here is that with this tecnique it's difficult to hide private variables and functions, since **namespaces** are thought to expose groups of public variables and functions.

This is why we went the other way around, choosing to use **classes**: this makes really simple to separate in a clear way public elements from the private ones, enforcing _information hiding_ and _encapsulation_ paradigms. However, since every variable and every method in our classes is marked as static (**remember that this is NOT an OOP project, we use classes ONLY to separate similar functions**), we do not make use of keywords such as **new** and **delete**. Let me explain this just another time: our classes DO NOT define objects, they ONLY do the job of namespaces but better (in this case, of course).

- For every class we have a CLASSNAME.h file in which we put the declaration of the class (**everything there is static!**) and a CLASSNAME.cpp in which we write the body of every function declared in the header file (except for inline functions);
- Public variables and functions should be accessed using the C++ scope resolution operator: CLASSNAME::aVariable , CLASSNAME::aFunction();
- No object should be initialised with **new** nor used to access public elements of a class (_for example like this: CLASSNAME obj; obj.something;_), since for every object (initialised or not) at least one pointer is needed (**and we don't want to waste memory, right?**).

### Endless switch statements
Before the release of MK4duo version 4.3.25, code for gcode execution was something like this:

```cpp
switch (gcode_number) {
  case 0:
  case 1: gcode_G0_G1(); break;

  [...]

  #if ENABLED(SOME_FEATURE)
    case 98: gcode_G98(); break;
  #endif
  case 99: gcode_G99(); break;
}
```

Not to mention the mcode-related switch (mcodes are defined in the range **between 0 and 999!!!**). This caused a huge and confusing code to be mantained and controlled carefully, because that's a critical section of the firmware.

In version 4.3.25 we overcame this problem, which still affects _Marlin_. We've collected every gcode in an array containing pointers to the _gcode handler functions_: more structure equals less code! That gave us the opportunity to replace those big switch statements with a simple and fast binary search algorithm which finds the correct _gcode handler function_ pointer inside that array.

**This improved code readability, length (and of course PROGMEM usage), efficiency and speed, while using only some additional bytes of SRAM.**
**Always remember this as an example of how the code can be improved a lot avoiding big and complex switch statements!**

### Useless MACROS
**MK4duo** is already full of macros, since it embraces _conditional compiling_. A lot of other macros are used to prevent code duplication and others provide some useful services too. But they make the code less readable: we try to avoid and delete useless macros, **and you should try also!**
