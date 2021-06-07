#include <Ps3Controller.h>

#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MahoWii.h"
#include "myEEPROM.h"

#include "ps3rx.h"


#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#define DEAD_ZONE 10
#define DEBOUNCE 300000ul

static uint16_t aux[4] = {1000,1000,1000,1000};
static uint32_t lastCheck = micros();
static uint32_t reinitTime = micros()+1000000;
static uint32_t debounceTime = 0;
static bool armed = false;
static uint8_t mode = 0;

static bool startPressed = false;
static bool upPressed = false;
static bool downPressed = false;
static bool leftPressed = false;
static bool rightPressed = false;

static bool trianglePressed = false;
static bool squarePressed = false;
static bool crossPressed = false;
static bool circlePressed = false;

static bool reinitOnce = false;

static bool debounce( uint32_t delta)
{
  if (delta > debounceTime)
  {
    debounceTime -= delta;
  }
  else
  {
    debounceTime = 0;
  }

  return debounceTime != 0;
}

void ps3rxInit()
{
  Ps3.begin();
}

static void updateLeds()
{
  if (armed)
  {
    Ps3.setPlayer(mode + 1);
  }
  else
  {
    Ps3.setPlayer(10);
  }
}

static void updateAux()
{
  aux[0] = (mode == 0) && armed ? 2000 : 1000;
  aux[1] = (mode == 1) && armed ? 2000 : 1000;
  aux[2] = (mode == 2) && armed ? 2000 : 1000;
  aux[3] = (mode == 3) && armed ? 2000 : 1000;
}


static void overrideBT()
{
  //PS3Controller has disabled BT discoverabilty, reenable it again after 1 second, when
  //BT is surelly initialized

  if (reinitTime != 0)
  {
    if (micros() > reinitTime)
    {
      esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
      reinitTime = 0;
      updateLeds();
    }
  }
}

static int16_t mapExp(int16_t value, int16_t fromMin, int16_t fromMax, int16_t toMin, int16_t toMax)
{
  int16_t v = map(value, fromMin, fromMax, toMin, toMax);

  v -= toMin;
  toMax -= toMin;

  int32_t d = (int32_t)toMax * toMax;

  if (v > 0)
  {
    v = ((((int32_t)v)*v*v) + d - 1) / d;
  }
  else
  {
    v = ((((int32_t)v)*v*v) - d + 1) / d;
  }

  return toMin + v;
}

static uint16_t remapStickY(int16_t val)
{
  if (val > -DEAD_ZONE && val < DEAD_ZONE) return 1500;
  if (val < 0)
  {
    return mapExp(val, -DEAD_ZONE, -128, 1500, 2000);
  }
  else
  {
    return mapExp(val, DEAD_ZONE, 127, 1500, 1000);
  }
}

static uint16_t remapStickX(int16_t val)
{
  if (val > -DEAD_ZONE && val < DEAD_ZONE) return 1500;
  if (val < 0)
  {
    return mapExp(val, -DEAD_ZONE, -128, 1500, 1000);
  }
  else
  {
    return mapExp(val, DEAD_ZONE, 127, 1500, 2000);
  }
}

static int16_t remapThrottle(int16_t val)
{
  if (val > -DEAD_ZONE && val < DEAD_ZONE) return 0;
  if (val < 0)
  {
    return map(val, DEAD_ZONE, -128, 0, 500);
  }
  else
  {
    return map(val, 127, DEAD_ZONE, -500, 0);
  }
}

static bool pressed(uint8_t key, bool* flag)
{
  if (key && !(*flag))
  {
    *flag = true;
    debounceTime = DEBOUNCE;
    return true;
  }
  else if (!key && (*flag))
  {
    *flag = false;
    debounceTime = DEBOUNCE;
  }
  return false;
}

uint16_t ps3rx_readRawRC(uint8_t chan)
{
  overrideBT();

  if (!Ps3.isConnected())
  {
    reinitOnce = true;
    return 0;
  }

  if (reinitOnce)
  {
    reinitOnce = false;
    updateLeds();
    updateLeds();
  }

  uint32_t t = micros();
  uint32_t delta = t - lastCheck;
  lastCheck = t;


  if (!debounce(delta))
  {
    if (pressed(Ps3.data.button.start, &startPressed))
    {
      armed = !armed;
      updateAux();
      updateLeds();
    }

    if (pressed(Ps3.data.button.triangle, &trianglePressed))
    {
      mode = 0;
      updateAux();
      updateLeds();
    }

    if (pressed(Ps3.data.button.square, &squarePressed))
    {
      mode = 1;
      updateAux();
      updateLeds();
    }

    if (pressed(Ps3.data.button.cross, &crossPressed))
    {
      mode = 2;
      updateAux();
      updateLeds();
    }

    if (pressed(Ps3.data.button.circle, &circlePressed))
    {
      mode = 3;
      updateAux();
      updateLeds();
    }

    if (pressed( Ps3.data.button.left, &leftPressed))
    {
      conf.angleTrim[ROLL] -=2;
      //writeParams(1);
    }

    if (pressed(Ps3.data.button.right, &rightPressed))
    {
      conf.angleTrim[ROLL] +=2;
      //writeParams(1);
    }

    if (pressed(Ps3.data.button.up, &upPressed))
    {
      conf.angleTrim[PITCH] +=2;
      //writeParams(1);
    }

    if (pressed(Ps3.data.button.down, &downPressed))
    {
      conf.angleTrim[PITCH] -=2;
      //writeParams(1);
    }


  }


  /*
  Serial.print(Ps3.data.analog.stick.lx);
  Serial.print("\t");
  Serial.print(Ps3.data.analog.stick.ly);
  Serial.print("\t");
  Serial.print(Ps3.data.analog.stick.rx);
  Serial.print("\t");
  Serial.print(Ps3.data.analog.stick.ry);
  Serial.print("\t");
  Serial.print(remapStickX( Ps3.data.analog.stick.lx));
  Serial.print("\t");
  Serial.print(remapStickY(Ps3.data.analog.stick.ly));
  Serial.print("\t");
  Serial.print(remapStickX(Ps3.data.analog.stick.rx));
  Serial.print("\t");
  Serial.print(remapStickY(Ps3.data.analog.stick.ry));
  Serial.print("\t");
  Serial.println(aux[0]);
  */

  /*
  Serial.print(conf.angleTrim[ROLL]);
  Serial.print("\t");
  Serial.println(conf.angleTrim[PITCH]);
  */

  switch (chan)
  {
  case ROLL:
    /*
    Serial.print("Roll");
    Serial.println(remapStickX(Ps3.data.analog.stick.rx));
    */

    return remapStickX( Ps3.data.analog.stick.rx );

  case PITCH:
    return remapStickY(Ps3.data.analog.stick.ry);

  case YAW:
    return remapStickX(Ps3.data.analog.stick.lx);

  case THROTTLE:
    return remapStickY(Ps3.data.analog.stick.ly);

  case 4:
    return aux[0];

  case 5:
    return aux[1];

  case 6:
    return aux[2];

  case 7:
    return aux[3];
  }

  return 1000;
}
