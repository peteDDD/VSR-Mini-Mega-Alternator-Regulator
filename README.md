# VSR-Mini-Mega-Rev-1

This project is a refactoring of the amazing work of William A. Thomason ("Al") to create an open source very smart alternator regulator.  That work can be seen here:
http://arduinoalternatorregulator.blogspot.com/

That open-source hardware is no longer orderable through Al.  In January 2019, that product "went pro" and is now available as a commercial product as you can see here:
http://wakespeed.com/products.html

In December 2020 I began refactoring Al's code to remove CAN and Bluetooth related code and move the hardware to a new PC board with a Mini Mega Pro 2560 daughter card
as the processor. Thus "VSR MINI MEGA" was born.

I wanted to remove some functionality and add other functionality.  Also, moving to the Mini Mega Pro 2560 provided more memory for expansion.

New hardware features include:
  - Dual INA226 current voltage sensors (for Alternator and Battery)    * Al's designs supported a second shunt but required a separate battery monitor to implement 
      that very unique and powerful functionality.  THe VSR MINI MEGA directly support two shunts.
  - Three Feature-in Ports and Three Feature-out Ports (functionality and port assignments selectable in Config.h)  
  - Two serial port screw headers, separate from port used to USB programming, one can support external full function display 
       (A remote display based on an Arduino Uno and a TFT color display will be published in the near future)
  - Two I2C pin headers to add an in-case OLED display or other features
  - Red and green LED output connections (independent of the Feature-out option which preceeded it).  
 
New firmware features include, selectable and configurable in Config.h include  :  
  - ENABLE_FEATURE_IN_SCUBA  - uses one of the Feature-in ports to shift the alternator field PWM way down so a small engine can run a SCUBA compressor while maintaining a 
      bit of output for keeping up with an electric clutch and relays used by the SCUBA compressor.
 -  FEATURE_OUT_GREEN_LED, FEATURE_OUT_RED_LED - echo the red and green status LED to your choice of Feature-out ports.
 -  FEATURE_OUT_LIFEPO_SHUTDOWN_ALARM - provides a Feature-out alarm which follows the Force-o-Float Feature-in
 -  USE_OLED - provides a class-based OLED display which displays realtime alternator and battery voltage, current, and temperatures, field PWM, current charging mode, 
      time in that mode, flags like "Forced Float" and "SCUBA", and error messages.  The display also reflects the DIP switch settings in human-speak at boot up.
 -  USE_SERIAL_DISPLAY - supports Arduio Uno-based color TFT display which alternately displays a screen showing alternator amps and battery voltage and a graphical display
      of the actual charging curve (amps and volts) over time
      
Features and firmware removed include (as best I can so far)
 -  CAN support
 -  Bluetooth artifacts from earlier VSR versions
 -  Engine control artifacts from Al's 6HP 12V diesel genset/water maker which preceded the VSR alternator regulator.
 -  Remote battery monitor artifacts
 
Future (near-term) enhancements planned:
 -  There is a stub in this release to support monitoring LiFePO4 battery management system's output for a pending battery disconnect.  This will then trigger a "Force-to-float"
      on the Mini-Mega to protect the alternator from a battery disconnect.
      
Also developed:
 -  Bench testing board to mimic battery and alternator voltages, currents, and temperatures, and feature-in and feature-out functions (switches and LEDs respectively)  
      This board also can have small volt and current meters on it and bannana jacks for stator input (from a signal generator) and tach out (to a frequency counter or 
      oscilloscope.
 -  The previously-mentioned remote color graphical display
 -  3D-printed case for VSR Mega-Mini with 0.93" OLED
 -  3D-printed bezel for the remote color graphical display
 
 ###
 
