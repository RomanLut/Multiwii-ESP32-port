Goals:
- [x] esp32 port of Mahowii (Multiwii)
- [x] control with PS3 controller
- [x] MSP communication using ESP32 bluetooth with EZ-GUI for configuration 
- [x] stable flight in ACRO, HORIZON, ANGLE mode 
- [x] stable fight in AltHold mode 
- [x] stable fight in MagHold mode 
- [x] control with Jumper T-Lite, HX_ESPNOW_RC library https://github.com/RomanLut/hx_espnow_rc (done)
- [x] MSP communnication with EZ-GUI through HX_ESPNOW_RC library 
- [x] add blackbox, download logs from onboard ftp 
- [ ] better documentaion, video
- [ ] ~~develop ESP32 flight controller board in EasyEDA~~
- [ ] ~~poshold mode~~
- [ ] ~~RTH~~
- [ ] ~~RTH failsafe~~
- [ ] ~~waypoint navigation~~
- [ ] ~~try other firmware (ardupilot, inav) ?~~

# About this project

![](/doc/quad1.jpg)

The goal is to build quadcopter based on ESP32 microcontroller, Eachine E58 motors and 3D-printed shell.

Drone can be controlled with Dualshock 3 controller, or DIY controller based on https://github.com/RomanLut/hx_espnow_rc

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

![](/doc/dualshock3.png)

Drone can be controlled using Dualshock 3 controller (original only!).

Refer to esp32-ps3 https://github.com/jvpernis/esp32-ps3 how to pair drone with controller. 

Controls:
- Arm/Disarm - Start. Arm with Throttle down!
- Acro(Mode 0) - PS3 Left Thumb
- Angle(Mode 1) - PS3 Square 
- Alt Hold(Mode 2) - PS3 Triangle 

Dualshick 3 D-pad is used for ACC trim in flight.

# Jumper T-Lite setup

![](/doc/ExternalModule.png)

Drone can be controlled with Jumper T-lite or similar controller with HX_ESPNOW_RC external module https://github.com/RomanLut/hx_espnow_rc/blob/main/doc/tx_external_module.md

Jumper t-lite have to be configured to use HX_ESPNOW_RC profile.

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

Battery voltage, RSSI and more telemetry sensors are sent as SPORT telemetry.

# Calibration


# Gyro calibration
Performed automaticaly on start

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


# Pin allocation

Used pins:
- 1 - TX0
- 3 - RX0
- 
- 4 - LF Blue LED  
- 12 - RF Blue LED
- 13 - LB RB Green LEDS
- 14 - RB Red LED   
- 15 - LB Red LED   
-
- 21 - SDA 
- 22 - SCL 
-
- 36 - VBAT ADC
-
- 25 - BL Motor
- 26 - BR Motor
-
- 27 - BUZZER EN
-
- 32 - FR Motor
- 33 - FL Motor

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
TODO share STL files on thingiverse

# No GPS modes?

Originaly I had a plan to implement Poshold, RTH and Waypoint navigation. Mahowii has code for that. But due to various reasons I decided to abandon this functionality:
- quadcopter does not have enough power to fight the wind
- I do not have enought outdoor time in calm weather for PID adjustments and testing
- GPS sensor is additional weight
- I little bit tired to fix and enhance old code :( It is better to try Ardupilot ESP32 port https://github.com/sh83/ardupilot/tree/esp32/libraries/AP_HAL_ESP32

So I disabled GPS modes currently.

Old GPS wiring: https://github.com/RomanLut/mahowii/blob/upstream_shared/doc/gps_code/gps_code.md

# Other projects

https://github.com/PepeTheFroggie/EspCopter

https://github.com/PepeTheFroggie/EspCopter32

https://github.com/cnlohr/wiflier

http://espcopter.com/community/

https://www.bitcraze.io/products/crazyflie-2-1/

https://readdevops.com/espressif/esp-drone

https://github.com/joshuasrcho/ESP32-Quadcopter

https://docs.espressif.com/projects/espressif-esp-drone/en/latest/gettingstarted.html

https://github.com/davidbuzz/ardupilot/blob/esp32_master_new_flat/libraries/AP_HAL_ESP32/README.md

https://github.com/RomanLut/hx_espnow_rc

esp32-ps3 https://github.com/jvpernis/esp32-ps3

# Mahowii repository 

https://github.com/mahowik/mahowii
