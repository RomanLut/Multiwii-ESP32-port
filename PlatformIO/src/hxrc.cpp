#include "hxrc.h"
#include "myWifi.h"
#include "HX_ESPNOW_RC_Serialbuffer.h"

#define HXRC_SERIAL_SIZE        128

static HXRCSlave hxrcSlave;

static HXRCSerialBuffer<HXRC_SERIAL_SIZE> serialTelemetry(&hxrcSlave);

unsigned long lastStats = millis();

void HXRCInit()
{
    hxrcSlave.init(
        HXRCConfig(
            WIFI_CHANNEL,
            0,
            false,
            -1, false 
        )
    );
}

void HXRCLoop()
{
    hxrcSlave.loop();
    serialTelemetry.flush();

  if (millis() - lastStats > 1000)
  {
    lastStats = millis();
    hxrcSlave.getTransmitterStats().printStats();
    hxrcSlave.getReceiverStats().printStats();
  }
}

int HXRCReadRawRC(int ch)
{
    if ( hxrcSlave.getReceiverStats().isFailsafe() ) return 0;
    //if ( ch == 3 ) Serial.println(hxrcSlave.getChannels().getChannelValue(ch));
    return hxrcSlave.getChannels().getChannelValue(ch);

}

uint16_t HXRCSerialAvailable()
{
    return serialTelemetry.getAvailable();
}

uint8_t HXRCSerialRead()
{
    return serialTelemetry.read();
}

uint16_t HXRCSerialAvailableForWrite()
{
    //Serial.println(serialTelemetry.getAvailableForWrite());
    return serialTelemetry.getAvailableForWrite();
}

void HXRCSerialWrite(uint8_t c)
{
    serialTelemetry.write(c);
}

bool HXRCSerialTxFree()
{
    return serialTelemetry.getAvailableForWrite() == HXRC_SERIAL_SIZE;
}

//0...1023
int HXRCRSSI()
{
    return ((uint32_t)hxrcSlave.getReceiverStats().getRSSI()) * 1023 / 100;
}

void HXRCSetVoltage(uint8_t value)
{
    hxrcSlave.setA1( value );//transfer voltage*10 value directly. set ratio to "-" in opentx, scale will be 0.1. otherwise scale would be ratio/255.
}

extern void HXRCSetNumSat(uint8_t value)
{
    hxrcSlave.setA2( value*10 );//transfer numsats as numsats*10. set ratio to "-" in opentx. set accuracy to 0.
}