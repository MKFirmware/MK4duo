# Configuring and compilation

  1. Install the LATEST non-beta Arduino IDE: http://www.arduino.cc/en/Main/Software and download LATEST board SAM!
  2. Download the MK4duo firmware: https://github.com/MagoKimbra/MK4duo (Use the green "Download Zip" button on the right)
  3. Some boards require special files and/or libraries from the Arduino directory.
  For Alligator Board add this link in Arduino IDE File -> Preferences -> Additional Boards Manager URLs: http://www.chew-z.it/download/alligator/package_Alligator_r2_index.json
  4. Start the Arduino IDE
  5. Select File -> Preferences -> "Compiler warning" and select "none"
  6. Select Tools -> Board -> Arduino Due (Native USB port) or Alligator Board R2 3D printer controller (USB/MCU native)
  7. Select the correct serial port in Tools ->Serial Port
  8. Open MK4duo.ino
  9. Click the Verify/Compile button
  10. Click the Upload button. If all goes well the firmware is uploading to the board!


Guida in Italiano per la compilazione dei campi.
http://forums.reprap.org/read.php?352,440672
