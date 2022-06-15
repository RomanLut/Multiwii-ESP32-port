/*
  To upload through terminal you can use: curl -F "image=@firmware.bin" esp8266-webupdate.local/update
*/

#include "Arduino.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>

#include "mywifi.h"

#include <esp_wifi.h>

#include <ESPxWebFlMgr.h>
#include <ESP-FTP-Server-Lib.h>

const char* ssid = "quad";
const char* password = "12345678";

ESPxWebFlMgr filemgr(80);
FTPServer ftp;

void Wifi_setup(void)
{
  WiFi.softAP(ssid, password, WIFI_CHANNEL);

  ftp.addUser("anonymous", "anonymous");
  ftp.addFilesystem("SPIFFS", &SPIFFS);
  ftp.begin();  

  filemgr.begin();
}

void Wifi_handle(void)
{
  ftp.handle();
  filemgr.handleClient();
}
