/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

#include <SPIFFS.h> 

//#include "blackbox.h"
#include "blackbox_io.h"


// How many bytes can we transmit per loop iteration when writing headers?
static uint8_t blackboxMaxHeaderBytesPerIteration;

static File file;
static bool isOpen = false;

// How many bytes can we write *this* iteration without overflowing transmit buffers or overstressing the OpenLog?
int32_t blackboxHeaderBudget;

/*
void blackboxOpen(void)
{
}
*/

void blackboxWrite(uint8_t value)
{
  if (file)
  {
    int32_t t1 = millis();
    file.write(value);
    int32_t t2 = millis();

    int32_t delta = t2 - t1;
    if (delta > 10)
    {
      Serial.print("File: write: ");
      Serial.println(delta);
    }
  }
  //flashfsWriteByte(value); // Write byte asynchronously
}

// Print the null-terminated string 's' to the blackbox device and return the number of bytes written
int blackboxPrint(const char *s)
{
  Serial.println("BLACKBOX: print");
  /*
    int length;
    const uint8_t *pos;

    length = strlen(s);
    flashfsWrite((const uint8_t*) s, length, false); // Write asynchronously

    return length;
    */

  if (!isOpen) return 0;

    int length = strlen(s);
    file.write((const uint8_t*)s, length);

    return length;
}

/**
 * If there is data waiting to be written to the blackbox device, attempt to write (a portion of) that now.
 *
 * Intended to be called regularly for the blackbox device to perform housekeeping.
 */
void blackboxDeviceFlush(void)
{
  //flashfsFlushAsync();
}

/**
 * If there is data waiting to be written to the blackbox device, attempt to write (a portion of) that now.
 *
 * Returns true if all data has been written to the device.
 */
bool blackboxDeviceFlushForce(void)
{
 // return flashfsFlushAsync();
  return true;
}

/**
 * Attempt to open the logging device. Returns true if successful.
 */
bool blackboxDeviceOpen(void)
{
  /*
    if (flashfsGetSize() == 0 || isBlackboxDeviceFull()) {
        return false;
    }

    blackboxMaxHeaderBytesPerIteration = BLACKBOX_TARGET_HEADER_BUDGET_PER_ITERATION;
    */

  Serial.println("BLACKBOX:OPEN");
  char fname[33];

  for (int num = 1; num < 10000; num++)
  {
    sprintf(fname, "/log%d.dat", num);
    if (!SPIFFS.exists(fname))
    {
      file = SPIFFS.open(fname, FILE_WRITE);
      break;
    }
  }
  if (!file) return false;
  isOpen = true;

  Serial.println("BLACKBOX:OPEN Ok");

  return true;
}

/**
 * Close the Blackbox logging device immediately without attempting to flush any remaining data.
 */
void blackboxDeviceClose(void)
{
  Serial.println("BLACKBOX:close");

  if (file)
  {
    file.close();
    isOpen = false;
  }

}

/**
 * Begin a new log (for devices which support separations between the logs of multiple flights).
 *
 * Keep calling until the function returns true (open is complete).
 */
bool blackboxDeviceBeginLog(void)
{
  Serial.println("BLACKBOX:Begin log");
  return true;
  //return blackboxSDCardBeginLog();
}

/**
 * Terminate the current log (for devices which support separations between the logs of multiple flights).
 *
 * retainLog - Pass true if the log should be kept, or false if the log should be discarded (if supported).
 *
 * Keep calling until this returns true
 */
bool blackboxDeviceEndLog(bool retainLog)
{
  Serial.println("BLACKBOX:Begin log");
  return true;
}

bool isBlackboxDeviceFull(void)
{
  return SPIFFS.totalBytes() - SPIFFS.usedBytes() < 10024;
  //return flashfsIsEOF();
}

/**
 * Call once every loop iteration in order to maintain the global blackboxHeaderBudget with the number of bytes we can
 * transmit this iteration.
 */
void blackboxReplenishHeaderBudget(void)
{
  Serial.println("BLACKBOX:ReplenishHeaderBudget");

  //removeme
  blackboxHeaderBudget = BLACKBOX_MAX_ACCUMULATED_HEADER_BUDGET;

  /*
    int32_t freeSpace;

    freeSpace = flashfsGetWriteBufferFreeSpace();

    blackboxHeaderBudget = MIN(MIN(freeSpace, blackboxHeaderBudget + blackboxMaxHeaderBytesPerIteration), BLACKBOX_MAX_ACCUMULATED_HEADER_BUDGET);
    */
}

/**
 * You must call this function before attempting to write Blackbox header bytes to ensure that the write will not
 * cause buffers to overflow. The number of bytes you can write is capped by the blackboxHeaderBudget. Calling this
 * reservation function doesn't decrease blackboxHeaderBudget, so you must manually decrement that variable by the
 * number of bytes you actually wrote.
 *
 * When the Blackbox device is FlashFS, a successful return code guarantees that no data will be lost if you write that
 * many bytes to the device (i.e. FlashFS's buffers won't overflow).
 *
 * When the device is a serial port, a successful return code guarantees that Cleanflight's serial Tx buffer will not
 * overflow, and the outgoing bandwidth is likely to be small enough to give the OpenLog time to absorb MicroSD card
 * latency. However the OpenLog could still end up silently dropping data.
 *
 * Returns:
 *  BLACKBOX_RESERVE_SUCCESS - Upon success
 *  BLACKBOX_RESERVE_TEMPORARY_FAILURE - The buffer is currently too full to service the request, try again later
 *  BLACKBOX_RESERVE_PERMANENT_FAILURE - The buffer is too small to ever service this request
 */
blackboxBufferReserveStatus_e blackboxDeviceReserveBufferSpace(int32_t bytes)
{
  return BLACKBOX_RESERVE_SUCCESS;
  /*
    if (bytes <= blackboxHeaderBudget) {
        return BLACKBOX_RESERVE_SUCCESS;
    }

    // Handle failure:
    if (bytes > (int32_t) flashfsGetWriteBufferSize()) {
        return BLACKBOX_RESERVE_PERMANENT_FAILURE;
    }

    if (bytes > (int32_t) flashfsGetWriteBufferFreeSpace()) {
        /*
          * The write doesn't currently fit in the buffer, so try to make room for it. Our flushing here means
          * that the Blackbox header writing code doesn't have to guess about the best time to ask flashfs to
          * flush, and doesn't stall waiting for a flush that would otherwise not automatically be called.
          *
        flashfsFlushAsync();
    }

    return BLACKBOX_RESERVE_TEMPORARY_FAILURE;
  */  
}
