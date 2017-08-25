## Issue template
Please, follow this template when opening a new issue: this will help us a lot in the process of helping you.

Try to give us every information you can: sometimes **big** problems are caused by _little_ details!

### Issue description
Tell us what happened: don't miss any valuable detail, try to be clear and concise, don't use UPPERCASE or abbreviations which can make understanding what you wrote quite difficult.

### Compile errors - IMPORTANT
If you have problems compiling MK4duo, report here the error messages Arduino IDE gives you.

Before posting this issue, however, take a long breath and check this two things:
1. If you're not using the latest version of Arduino IDE, please retry compiling with the latest one. Sometimes errors are caused by old versions of the compiler and not by MK4duo code.
2. Read carefully every error message you get: if an error starts with `DEPENDENCY ERROR: _something wrong!_`, well, that's not a compiling error! **DO NOT POST DEPENDENCY ERRORS**, please! Actually, they are self-explanatory messages which tell you what configuration option was set badly. You just have to go back and fix your configuration.

**Dependency errors are not MK4duo errors, are YOUR errors!** If you also get other types of error, please be sure to have fixed dependency errors **before** posting this issue!

### Firmware version
Always remember to write what version of MK4duo you're using

### My setup
1. If you configured MK4duo with the online configurator, attach here your `Configuration_Overall.h` file.

2. Otherwise, if you configured MK4duo manually through the config files, write here a list of the main settings you think may be related to the issue, like this:
- SETTING_1    true
- SETTING_2    25
- SETTING_3    "blablabla"
- ...

_If needed, you'll be asked for other settings you modified._

Please be patient, we'll be trying to help you as soon as possible.
