Goals:
- [x] esp32 port of Mahowii (Multiwii)
- [x] control with PS3 controller
- [x] MSP communication using ESP32 bluetooth with EZ-GUI for configuration 
- [x] stable flight in ACRO, HORIZON, ANGLE mode 
- [x] stable fight in AltHold mode 
- [x] stable fight in MagHold mode 
- [x] control with Jumper T-Lite, HX_ESPNOW_RC library https://github.com/RomanLut/hx_espnow_rc 
- [x] MSP communnication with EZ-GUI through HX_ESPNOW_RC library 
- [x] add blackbox, download logs from onboard ftp 
- [x] better documentaion, video
- [ ] ~~develop ESP32 flight controller board in EasyEDA~~
- [ ] ~~poshold mode~~
- [ ] ~~RTH~~
- [ ] ~~RTH failsafe~~
- [ ] ~~waypoint navigation~~
- [ ] ~~try other firmware (ardupilot, inav) ?~~

# About this project

Click to see video: http://www.youtube.com/watch?v=lwrkspHjSu4

![](/doc/quad1.jpg)

The goal is to build quadcopter based on ESP32 microcontroller, Eachine E58 motors and 3D-printed shell.

Drone can be controlled with Dualshock 3 gamepad, or DIY controller based on https://github.com/RomanLut/hx_espnow_rc

Firmware is Mahowii port for ESP32.

Mahowii is modified Multiwii firmware enchanced for quadcopters. Original Mahowii repository: https://github.com/mahowik/mahowii

Changes implemented:
- ESP32 port code fixes (32 bit arch fixes, EEPROM stub)
- PlatformIO project setup
- Added PS3 controller rx
- implemented bluetooth serial communication for EZGUI
- implemented changes for specific hardware ( brushed motors, arm leds )
- added blackbox (Cleanflight format)
- added WifI softAP and ftp server (for blackbox)
- changed altitude mode: throtle controls target altitude
- changed maghold mode: yaw stick controls target direction
- adjusted default PIDs
- control and MSP communication with HX_ESPNOW_RC library https://github.com/RomanLut/hx_espnow_rc
- disabled GPS code

![](/doc/quad2.jpg)

*This project requiers understading of multiwii basics. You will have to make it work for you. No step-by-step build guide is provided here.*

# Connection diagrams

![](/doc/wiring.jpg)

![](/doc/led_wiring.jpg)

# EZ_GUI connection

![](/doc/ezgui1.png)

EZ-GUI application can be used to see telemetry and edit drone settings: https://play.google.com/store/apps/details?id=com.ezio.multiwii

This is optional. When you flash ESP32 first time, PIDs and AUX settings described below are applied.

If drone is controlled with Dualshock controller, connect directly to drone, which is visible as "QUADX" BLE device.

If drone is controlled with HX_ESPNOW_RC controller, you have to connect to RC Controller instead, because ESP32 can not handle WIFI and BLE simultaneously.

# AUX settings 
- AUX1 Mid:  Arm
- AUX1 High: Arm
- AUX2 Mid:  Angle
- AUX2 High: Angle + Alt Hold
- AUX4 High: Beeper

![](/doc/ezgui3.png)

# PID settings 

![](/doc/ezgui2.png)

# PS3 Controler 

![](/doc/dualshock3.jpg)

Drone can be controlled using Dualshock 3 gamepad (original only!).

Effective distance is <50m.

Refer to esp32-ps3 https://github.com/jvpernis/esp32-ps3 how to pair drone with controller. 

Controls:
- Arm/Disarm - Start. Arm with Throttle down!
- Acro(Mode 0) - PS3 Left Thumb
- Angle(Mode 1) - PS3 Square 
- Alt Hold(Mode 2) - PS3 Triangle 

Dualshock 3 D-pad is used for ACC trim in flight.

# Jumper T-Lite setup

![](/doc/ExternalModule.jpg)

Drone can be controlled by Jumper T-lite or similar controller with HX_ESPNOW_RC external module https://github.com/RomanLut/hx_espnow_rc/blob/main/doc/tx_external_module.md

Jumper t-lite have to be configured to use HX_ESPNOW_RC profile.

Effective distance is more then 200m.

Channel assignments:
- AERT
- CH5: SWC
- CH6: SWA 
- CH8: SWD
- CH9: MAX, Switch: tEu 
- CH10: MAX, Switch: tEd
- CH11: MAX, Switch: tAu
- CH12: MAX, Switch: tAd
- CH16: MAX, Weight:0,Offset -80% ( hx_espnow_rc LR profile selection - profile 2)

Disable trims in FlightMode 0. Right stick trim buttons are used to trim ACC in flight. 

Controls:
- SWC - Arm
- SWA - Acro / Angle / AltHold
- SWD - Beeper
- Right stick trims are used to trim ACC in flight.

Battery voltage, RSSI, height and other telemetry sensors are sent as SPORT telemetry.

# Calibration


# Gyro calibration
Performed automaticaly on power on.

Yaw left + Thottle Low + Pitch LOW.

Blue and red leds blinking fast.

# Accelerometer calibration

Should be done once.

Yaw left + Throttle Hi + Pitch LOW.

Blue and green leds blinking fast

Simple procedure: Place on level surface, do not move.

This firmware also contains more precise calibration procedure:

Calibrate quad facing up, then calibrate quad facing down.

# Arm LEDs colors
RC signal:
- Front arms no lights - no RC signal
- Front arms blue solid - ok RC signal

Battry status:
- Back arms green: battery ok
- Back arms yellow: battery near low
- Back arms red: battery low
- Back arms red blink: battery critical, land now

Calibration:
- Red and Blue leds blinking fast: gyro calibration
- Red and Green leds blinking fast: acc calibration


# Blackbox

Drone can write blackbox log to SPIFFS filesystem in Cleanflight log format https://github.com/cleanflight/blackbox-log-viewer

![](/doc/blackbox.png)

To enable logging, uncomment `#define BLACKBOX` in config.h.

Logs can be downloaded using HTPP of FTP connection. Connect to drone's Wifi AP: user"quad", password:"12345678". Browse "http://192.168.4.1" or connect to ftp://192.168.4.1 using User: "anonymous", password: "anonymous".

![](/doc/filemanager.png)

*Note: After creating/deleting files, SPIFFS filesystem became fragmented. It's perfomance will degrade and affect cycle time. Drone will start to "hickup". Log to fresh SPIFFS. To refresh SPIFFS, "Upload filesystem" using PlatformIO. Disable blackbox for normal usage.*

# Pin allocation

Used pins:
- 1 - TX0
- 3 - RX0
- 
- 4 - Left Front Blue LED  
- 12 - Right Front Blue LED
- 13 - Left Back, Right Back Green LEDS
- 14 - Right Back Red LED   
- 15 - Left Back Red LED   
-
- 21 - SDA 
- 22 - SCL 
-
- 36 - VBAT ADC
-
- 25 - Left back Motor
- 26 - Right Back Motor
-
- 27 - BUZZER EN
-
- 32 - Right Front Motor
- 33 - Left Front Motor

Free pins:
- 0 - (not available on 30 pin board)
- 2 - board LED_PIN
- 5 
-
- 16 - RX2 
- 17 - TX2 
-
- 18 - SCK
- 19 - MISO
- 23- MOSI
-
- 35 - (inpput only)
- 39 - (input only)

# STL files for printing

![](/doc/drawings.jpg)

Solidworks 2018 drawings and STL files are available in **Drawings** folder and on Thingiverse https://www.thingiverse.com/thing:5460208

# No GPS modes?

Originaly I had a plan to implement Poshold, RTH and Waypoint navigation. Mahowii has code for that. But due to various reasons I decided to abandon this functionality:
- quadcopter does not have enough power to fight the wind
- I do not have enought outdoor time in calm weather for PID adjustments and testing
- GPS sensor is additional weight
- I little bit tired to fix and enhance old code :( It is better to try Ardupilot ESP32 port https://github.com/sh83/ardupilot/tree/esp32/libraries/AP_HAL_ESP32

So I disabled GPS modes currently.

Old GPS wiring: https://github.com/RomanLut/mahowii/blob/upstream_shared/doc/gps_code/gps_code.md

# Does it fly Ok?

Drone flyies Ok in Acro and Angle modes. Drone does not have a lot of power, E58 motors are not powerfull for any acrobatics. It is slow flier.

Althold mode could be better. I think performance is limited by Multiwii firmware.  


# Other projects

https://github.com/qqqlab/madflight

https://habr.com/ru/articles/814127/

https://github.com/PepeTheFroggie/EspCopter

https://github.com/PepeTheFroggie/EspCopter32

https://github.com/cnlohr/wiflier

http://espcopter.com/community/

https://www.bitcraze.io/products/crazyflie-2-1/

https://github.com/joshuasrcho/ESP32-Quadcopter

https://docs.espressif.com/projects/espressif-esp-drone/en/latest/gettingstarted.html

https://www.youtube.com/watch?v=KIt8LpW8Ajs

https://www.youtube.com/watch?v=BB96u04KlrY

https://github.com/AnishDey27/NodeMCU-FlightController-MPU6050

https://github.com/davidbuzz/ardupilot/blob/esp32_master_new_flat/libraries/AP_HAL_ESP32/README.md

https://github.com/RomanLut/hx_espnow_rc

esp32-ps3 https://github.com/jvpernis/esp32-ps3

https://github.com/simondlevy/Hackflight

https://github.com/rtlopez/esp-fc

https://github.com/diegovazquez/ESPPlane

https://github.com/davidoises/FlightController

https://github.com/dgnemre/Simple-Quadcopter-Flight-Controller

https://github.com/espressif/esp-drone/blob/master/docs/en/rst/gettingstarted.rst

https://www.instructables.com/WIFI-CONTROLLED-RC-PLANE/

https://github.com/subramaniyasais/QuadRotor-Drone

https://github.com/rohanverma94/The-Open-Copter

https://github.com/pratikPhadte/ESP32-Flight-controller-

# Mahowii repository 

https://github.com/mahowik/mahowii
