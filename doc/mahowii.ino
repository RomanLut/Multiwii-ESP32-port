
/*
 * Welcome to MultiWii.
 *
 * If you see this message, chances are you are using the Arduino IDE. That is ok.
 * To get the MultiWii program configured for your copter, you must switch to the tab named 'config.h'.
 * Maybe that tab is not visible in the list at the top, then you must use the drop down list at the right
 * to access that tab. In that tab you must enable your baord or sensors and optionally various features.
 * For more info go to http://www.multiwii.com/wiki/index.php?title=Main_Page
 *
 * Have fun, and do not forget MultiWii is made possible and brought to you under the GPL License.
 *
 */


/*
 * ########## Extended by Mahowik ############ 
 */

/*

  ESP32 port by Roman Lut
  - Ported to ESP32
  - Added support for QMC5883 magnetometer QMC5883
  - added support for BMP280 barometer
  - Bluetooth for EZ-GUI ESP32_BLUETOOTH_MSP
  - implemented arm LEDs ARM_LEDS
  - changed output to PWM, for brushed motors
  - changed VBAT monitoring for 3.7V batery
  - added Cabell protocol with NRF24lL01 CABELL, telemetry: RSSI, VBAT, satelites count 
  - changed altitude hold mode: trottle controls altitude, not vario
  - control with Ps3 controller

  Should be compiled against esp32-sdk 1.0.04 because never version does not contain required low-level bluetooth API for esp32-ps3 (PS3Controller) library!!!

  GPIOS:
  Reference: https://circuits4you.com/2018/12/31/esp32-devkit-esp32-wroom-gpio-pinout/

  ARM LEDS:
  4 - LF Blue   120 ohm
  12 - RF Blue   120 ohm
  14 - RB Red    68 ohm
  15 - LB Red    68 ohm
  13 - LB RB Green   100 ohm

  Indication:
   Front arms no lights - no RC signal
   Front arms blue solid - on, ok RC signal
   Front arms blue blink two times once per second - RTH, Failsafe or Landing

   Back arms green: battery ok, GPS fix
   Back arms yellow : battery ok, no GPS fix
   Back arms red: battery low
   Back arms red blink: battery critical, land now

   Red and Blue leds blinking fast: gyro calibration
   Red and Green lads blinking fast: acc calibration
   Red and blue leds moving in cyrcle: Magnetometer calibration

  *OLD:
  *2 - LED (LEDPIN, internal LED), ARM_LEDS_BLUE 2n7002 + 68 Ohm  -
  *12 - STABLE PIN LED (either acro/horison or GPS FIX indicator depending on GPS_LED_INDICATOR define)
  *14 - ARM_LEDS_GREEN 2n7002 + 100 Ohm
  *15 - ARM_LEDS_RED 2n7002 + 120 Ohm


  21 - SDA - add 2.4K...3.3K pullup resistor to 3.3V
  22 - SCL - add 2.4K...3.3K pullup resistor to 3.3V

  1 - TX0
  3 - RX0
  16 - RX2 - GPS TX
  17 - TX2 - GPS RX

  4 - NRF24L01 CE  - is it required?
  5 - NRG24L01 CSN
  23- MOSI
  19 - MISO
  18 - SCK

  36 - VBAT ADC  (10K + 5k6 divisor for 3.7V battery)


  uint8_t PWM_PIN[8] = { 26, 32, 25, 33 };
  - 82 Ohm to gate
  FL - 33
  FR - 32
  BL - 25
  BR - 26

  Free pins:
  0 - (not available on 30 pin board)
  2(LED_PIN),35,39

  Serial ports:
  Serial0 - USB serial
  Serial1 - Harware is unused. UART1 is emuated by HX_ESPNOW_RC as telemetry port.
  Serial2 - GPS
  Serial3 - Bluetooth
  Serial4 - TODO: TCP Socket for EZ-gui


   - remove 01d resistor to lower cutoff voltage
   - connect EN to VIN
   - change 91kOhm to 147kOhm ( 91+56) to change output from 3.3 to 5


  TODO:
  - OTA updates?
  - FRSky telemetry?
  - DAC - voiceover?  (DAC 25 26)


  PS3 contorller:
  Pair using SixAxisPairTool

  Start - ARM/DISARM. Arm with throttle stick down.
  Throttle is absolute(middle position = 50%, 1500).

  DPad - trim

  Triangle - mode 0 (default), AUX1=2000 == Angle+Althold
  Square - mode 1, AUX2=2000 == Angle
  Cross  - mode 2, AUX3=2000 == Angle+Althold+Mag
  Circle - mode 3, AUX4=2000 == Angle+Althold+Mag+GPSPoshold

  Mode is indicated by Player leds while armed. Disarmed - all leds on.

*/

