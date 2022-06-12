#include "hxrc.h"
#include "myWifi.h"
#include "HX_ESPNOW_RC_Serialbuffer.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MahoWii.h"
#include "myEEPROM.h"

#define HXRC_SERIAL_SIZE        128

static HXRCSlave hxrcSlave;

static HXRCSerialBuffer<HXRC_SERIAL_SIZE> serialTelemetry(&hxrcSlave);

unsigned long lastStats = millis();

static int8_t prevTrim = -1;
static int8_t thisTrim = -1;

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

    //detect trims
    prevTrim = thisTrim;

    thisTrim = -1;
    if ( HXRCReadRawRC(8) > 1750 ) thisTrim = 0; //CH9 trUP
    else if ( HXRCReadRawRC(9) > 1750 ) thisTrim = 1;  //CH10 trDN
    else if ( HXRCReadRawRC(10) > 1750 ) thisTrim = 2;  //CH11 trLEFT
    else if ( HXRCReadRawRC(11) > 1750 ) thisTrim = 3;  //CH12 trRIGHT

    if ( prevTrim == -1 )
    {
        switch ( thisTrim )
        {
            case 0: 
                conf.angleTrim[PITCH] +=2;
                Serial.println("TrimUP");
            break;

            case 1: 
                conf.angleTrim[PITCH] -=2;
                Serial.println("TrimDn");
            break;

            case 2:
                conf.angleTrim[ROLL] -=2;
                Serial.println("TrimLeft");
            break;

            case 3:
                conf.angleTrim[ROLL] +=2;
                Serial.println("TrimRight");
            break;
        }
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