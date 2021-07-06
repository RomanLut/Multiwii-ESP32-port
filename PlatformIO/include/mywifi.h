#pragma once

//wifi channel on which AP will be initialized, and where ESPNOW rc will work
#define WIFI_CHANNEL 3

void Wifi_setup(void);
void Wifi_handle(void);
