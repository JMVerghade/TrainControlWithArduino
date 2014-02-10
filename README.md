TrainControlWithArduino
=======================

Model railroad DCC command with Arduino

Started the 10th of february 2014 by Jean-Marie Verghade

My goal is to developp software and hardware to control a small model railroad with DCC protocol.
I come from software development, so Arduino looks the best way to start quickly to build a small DCC command control connected to my laptop PC that runs MS Windows. An infraRed remote control plus a 2x16 LCD display shall also provide the system with a very simple interface to select trains and pilot locomotives throttle without having the laptop connected. The PC will be used later for automation. For the moment, a small windows allows to control locomotives.

Modbus protocol was an evidence for me. Very simple to implement, very basic and cheap hardware, already available with USB-RS232 on the Arduino UNO. For one Arduino RS232 is OK, but let's imagine i want to add in the future other Arduinos to have feedbacks or ton control turnouts, why not use Modbus RTU on RS485 serial link ? 

After a quick search on the web, several pieces of code were revelant for my Arduino project :
- a very cool lib for DCC on Arduino : CmdrArduino from RailStars at https://github.com/Railstars/CmdrArduino
- one lib for Modbus communication written by Jason Vreeland [CodeRage] at http://code.google.com/p/arduino-modbus-slave/
- one lib for InfraRed telecommand : IRremote at http://github.com/shirriff/Arduino-IRremote
- the Arduino lib for the LCD display : LiquidCrystal at http://www.arduino.cc/en/Tutorial/LiquidCrystal

For the PC part, i decided to use Microsoft Visual Studio Express and to write in C##, because it's free, powerfull and i have always program with Microsoft IDE in the past. Once again, a search on the web give me a simple code library :
- MODBUS_SEUL_2008.cs by B.Vannier at www.va2i.com juil 2008

And that's all !

I developp first with an Arduino UNO, but i get soon short in memory with 10 locomotives and 9 functions to manage.
Unfortunately, i made a short on my Arduino board with the DCC 20VAC... So i decided to buy a MEGA and try again.
However, i have something that works fine, even if code is not perfect...

