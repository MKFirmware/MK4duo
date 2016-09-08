Power consumption sensor (Hall effect sensor like ACS712)
---------------------------------------------------------
Support for a current sensor (Hall effect sensor like ACS712) for measure the power consumption
Since it's more simple to deal with, we measure the DC current and we assume that POWER_VOLTAGE that comes from your power supply it's almost stable.
You have to change the POWER_SENSITIVITY with the one that you can find in the datasheet. (in case of ACS712: set to .100 for 20A version or set .066 for 30A version)

After setted POWER_VOLTAGE and POWER_SENSITIVITY you have to found correct value for POWER_ZERO.
You can do it by using "M70 Z" gcode and read the calculated value from serial messages.
Before calling "M70 Z" you have to disconnect the cable for measure the current from the sensor leaving only +5, OUT and GND connections.
Insert new values into FW and recompile.
Now you can reconnect the current cable to the sensor.

Now you have to set right value for POWER_ERROR.
Get a good multimeter and meacure DC current coming out from the power supply.
In order to get an accurate value power-on something (Eg. Heater, Motor, Fan) DO NOT POWER-ON THE BED OR YOU MAY KILL IT!!!!
Call "M70 Ax" where 'x' is the value measured by the multimeter.
Insert new values into FW and recompile.

With this module we measure the Printer power consumption ignoring the Power Supply power consumption,
so we consider the POWER_EFFICIENCY of our supply to be 100%.
WARNING: from this moment the procedure can be REALLY HARMFUL to health unless you have a little experience so DO NOT DO IT IF YOU DO NOT KNOW WHAT YOU ARE DOING!!!
If you want to approximately add the supply consumption you have measure the AC current with a good multimeter
and moltiple it with the mains voltage (110V AC - 220V AC).
MULTIMETER_WATT = MULTIMETER_CURRENTMAINS_VOLTAGE
Call "M70 Wx" where 'x' is MULTIMETER_WATT;
Insert new values into FW and recompile.

Now you AC712 it should be calibrated.