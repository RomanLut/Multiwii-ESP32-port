#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "Serial.h"
#include "MahoWii.h"

#ifndef ESP32
static volatile uint8_t serialHeadRX[UART_NUMBER],serialTailRX[UART_NUMBER];
static uint8_t serialBufferRX[RX_BUFFER_SIZE][UART_NUMBER];
static volatile uint8_t serialHeadTX[UART_NUMBER],serialTailTX[UART_NUMBER];
static uint8_t serialBufferTX[TX_BUFFER_SIZE][UART_NUMBER];
#else

  #ifdef ESP32_BLUETOOTH_MSP
  #include <BluetoothSerial.h>

  static BluetoothSerial SerialBT;

  //BluetoothSerial library does not have availableForWrite() method.
  //In order to emulate it, we will simulate 50% of 115200 baud rate ( 115200 bytes per second ) flow
  //micros() when some bytes has been written to BT
  #define SERIAL_BT_QUEUE_SIZE 128
  #define SERIAL_BT_BAUD 115200
  #define SERIAL_BT_MICROS_PER_BYTE (2 * 1000000 / (SERIAL_BT_BAUD / 10))
  static uint32_t bt_lastCheck;
  static uint32_t bt_bytesInFlight;
  #endif

  //here we remember what number returns Serial.AvailableForWrite() for completely empty buffer.
  //There is no dedicated function for checking if all data has been sent. We emulate it using AvailabeForWrite().
  static uint32_t serialBufferTXSize[UART_NUMBER];

#endif

// *******************************************************
// For Teensy 2.0, these function emulate the API used for ProMicro
// it cant have the same name as in the arduino API because it wont compile for the promini (eaven if it will be not compiled)
// *******************************************************
#if defined(TEENSY20)
  unsigned char T_USB_Available(){
    int n = Serial.available();
    if (n > 255) n = 255;
    return n;
  }
#endif

// *******************************************************
// Interrupt driven UART transmitter - using a ring buffer
// *******************************************************


#if defined(PROMINI) || defined(MEGA)
  #if defined(PROMINI)
  ISR(USART_UDRE_vect) {  // Serial 0 on a PROMINI
  #endif
  #if defined(MEGA)
  ISR(USART0_UDRE_vect) { // Serial 0 on a MEGA
  #endif
    uint8_t t = serialTailTX[0];
    if (serialHeadTX[0] != t) {
      if (++t >= TX_BUFFER_SIZE) t = 0;
      UDR0 = serialBufferTX[t][0];  // Transmit next byte in the ring
      serialTailTX[0] = t;
    }
    if (t == serialHeadTX[0]) UCSR0B &= ~(1<<UDRIE0); // Check if all data is transmitted . if yes disable transmitter UDRE interrupt
  }
#endif
#if defined(MEGA) || defined(PROMICRO)
  ISR(USART1_UDRE_vect) { // Serial 1 on a MEGA or on a PROMICRO
    uint8_t t = serialTailTX[1];
    if (serialHeadTX[1] != t) {
      if (++t >= TX_BUFFER_SIZE) t = 0;
      UDR1 = serialBufferTX[t][1];  // Transmit next byte in the ring
      serialTailTX[1] = t;
    }
    if (t == serialHeadTX[1]) UCSR1B &= ~(1<<UDRIE1);
  }
#endif
#if defined(MEGA)
  ISR(USART2_UDRE_vect) { // Serial 2 on a MEGA
    uint8_t t = serialTailTX[2];
    if (serialHeadTX[2] != t) {
      if (++t >= TX_BUFFER_SIZE) t = 0;
      UDR2 = serialBufferTX[t][2];
      serialTailTX[2] = t;
    }
    if (t == serialHeadTX[2]) UCSR2B &= ~(1<<UDRIE2);
  }
  ISR(USART3_UDRE_vect) { // Serial 3 on a MEGA
    uint8_t t = serialTailTX[3];
    if (serialHeadTX[3] != t) {
      if (++t >= TX_BUFFER_SIZE) t = 0;
      UDR3 = serialBufferTX[t][3];
      serialTailTX[3] = t;
    }
    if (t == serialHeadTX[3]) UCSR3B &= ~(1<<UDRIE3);
  }
#endif

void UartSendData(uint8_t port) {
  #if defined(PROMINI)
    UCSR0B |= (1<<UDRIE0);
  #endif
  #if defined(PROMICRO)
    switch (port) {
      case 0:
        while(serialHeadTX[0] != serialTailTX[0]) {
           if (++serialTailTX[0] >= TX_BUFFER_SIZE) serialTailTX[0] = 0;
           #if !defined(TEENSY20)
             USB_Send(USB_CDC_TX,serialBufferTX[serialTailTX[0]],1);
           #else
             Serial.write(serialBufferTX[serialTailTX[0]],1);
           #endif
         }
        break;
      case 1: UCSR1B |= (1<<UDRIE1); break;
    }
  #endif
  #if defined(MEGA)
    switch (port) {
      case 0: UCSR0B |= (1<<UDRIE0); break;
      case 1: UCSR1B |= (1<<UDRIE1); break;
      case 2: UCSR2B |= (1<<UDRIE2); break;
      case 3: UCSR3B |= (1<<UDRIE3); break;
    }
  #endif
  //this method does nothing on ESP32. Data is already in output buffer and is sent by Serial library.
}

#ifdef ESP32_BLUETOOTH_MSP
uint16_t SerialBTAvailableForWrite()
{
  uint32_t t = micros();
  uint32_t d = t - bt_lastCheck;
  d = d / SERIAL_BT_MICROS_PER_BYTE; //number of bytes which theoretically have been sent from last check
  if (d >= bt_bytesInFlight)
  {
    bt_bytesInFlight = 0;
    bt_lastCheck = t;
  }
  else
  {
    bt_bytesInFlight -= d;
    bt_lastCheck += d * SERIAL_BT_MICROS_PER_BYTE;
  }

  /*
  Serial.print("BT:AvailableForWrite=");
  Serial.println(max(0u, SERIAL_BT_QUEUE_SIZE - bt_bytesInFlight));
  */

  return max(0u, SERIAL_BT_QUEUE_SIZE - bt_bytesInFlight);
}
#endif

#if defined(GPS_SERIAL)
bool SerialTXfree(uint8_t port)
{
#ifdef ESP32
  switch (port)
  {
    case 0: return serialBufferTXSize[0] == Serial.availableForWrite();
    case 2: return serialBufferTXSize[2] == Serial2.availableForWrite();
    #ifdef ESP32_BLUETOOTH_MSP
    case 3: return serialBufferTXSize[3] == SerialBTAvailableForWrite();
    #endif
  }
  return false;
#else
    return (serialHeadTX[port] == serialTailTX[port]);
#endif
  }
#endif

void SerialOpen(uint8_t port, uint32_t baud) {
  uint8_t h = ((F_CPU  / 4 / baud -1) / 2) >> 8;
  uint8_t l = ((F_CPU  / 4 / baud -1) / 2);
  switch (port) {
    #if defined(PROMINI)
      case 0: UCSR0A  = (1<<U2X0); UBRR0H = h; UBRR0L = l; UCSR0B |= (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0); break;
    #endif
    #if defined(PROMICRO)
      #if (ARDUINO >= 100) && !defined(TEENSY20)
        case 0: UDIEN &= ~(1<<SOFE); break;// disable the USB frame interrupt of arduino (it causes strong jitter and we dont need it)
      #endif
      case 1: UCSR1A  = (1<<U2X1); UBRR1H = h; UBRR1L = l; UCSR1B |= (1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1); break;
    #endif
    #if defined(MEGA)
      case 0: UCSR0A  = (1<<U2X0); UBRR0H = h; UBRR0L = l; UCSR0B |= (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0); break;
      case 1: UCSR1A  = (1<<U2X1); UBRR1H = h; UBRR1L = l; UCSR1B |= (1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1); break;
      case 2: UCSR2A  = (1<<U2X2); UBRR2H = h; UBRR2L = l; UCSR2B |= (1<<RXEN2)|(1<<TXEN2)|(1<<RXCIE2); break;
      case 3: UCSR3A  = (1<<U2X3); UBRR3H = h; UBRR3L = l; UCSR3B |= (1<<RXEN3)|(1<<TXEN3)|(1<<RXCIE3); break;
    #endif
    #if defined(ESP32)
      case 0:  Serial.begin(baud); serialBufferTXSize[0] = Serial.availableForWrite(); break;
      case 2:  Serial2.begin(baud, SERIAL_8N1, 16, 17); serialBufferTXSize[2] = Serial2.availableForWrite(); break;
#ifdef ESP32_BLUETOOTH_MSP
      case 3: SerialBT.begin("QUADX"); serialBufferTXSize[3] = SERIAL_BT_QUEUE_SIZE; bt_bytesInFlight = 0; bt_lastCheck = micros(); break;
#endif

#endif
  }
}

void SerialEnd(uint8_t port) {
  switch (port) {
    #if defined(PROMINI)
      case 0: UCSR0B &= ~((1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)|(1<<UDRIE0)); break;
    #endif
    #if defined(PROMICRO)
      case 1: UCSR1B &= ~((1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1)|(1<<UDRIE1)); break;
    #endif
    #if defined(MEGA)
      case 0: UCSR0B &= ~((1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)|(1<<UDRIE0)); break;
      case 1: UCSR1B &= ~((1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1)|(1<<UDRIE1)); break;
      case 2: UCSR2B &= ~((1<<RXEN2)|(1<<TXEN2)|(1<<RXCIE2)|(1<<UDRIE2)); break;
      case 3: UCSR3B &= ~((1<<RXEN3)|(1<<TXEN3)|(1<<RXCIE3)|(1<<UDRIE3)); break;
    #endif
    #if defined(ESP32)
      case 0:  Serial.end(); break;
      case 2:  Serial2.end(); break;
      #ifdef ESP32_BLUETOOTH_MSP
      case 3: SerialBT.end();
      #endif
#endif
  }
}

#ifndef ESP32
// we don't care about ring buffer overflow (head->tail) to avoid a test condition : data is lost anyway if it happens 
void store_uart_in_buf(uint8_t data, uint8_t portnum) {
 #if defined(SERIAL_RX)
    if (portnum == RX_SERIAL_PORT) {
      if (!spekFrameFlags) { 
        sei();
        uint32_t spekTimeNow = (timer0_overflow_count << 8) * (64 / clockCyclesPerMicrosecond()); //Move timer0_overflow_count into registers so we don't touch a volatile twice
        uint32_t spekInterval = spekTimeNow - spekTimeLast;                                       //timer0_overflow_count will be slightly off because of the way the Arduino core timer interrupt handler works; that is acceptable for this use. Using the core variable avoids an expensive call to millis() or micros()
        spekTimeLast = spekTimeNow;
        if (spekInterval > 2500) {  //Potential start of a Spektrum frame, they arrive every 11 or every 22 ms. Mark it, and clear the buffer. 
          serialTailRX[portnum] = 0;
          serialHeadRX[portnum] = 0;
          spekFrameFlags = 0x01;
        }
        cli();
      }
    }
  #endif

  uint8_t h = serialHeadRX[portnum];
  serialBufferRX[h++][portnum] = data;
  if (h >= RX_BUFFER_SIZE) h = 0;
  serialHeadRX[portnum] = h;
}
#endif

#if defined(PROMINI)
  ISR(USART_RX_vect)  { store_uart_in_buf(UDR0, 0); }
#endif
#if defined(PROMICRO)
  ISR(USART1_RX_vect)  { store_uart_in_buf(UDR1, 1); }
#endif
#if defined(MEGA)
  ISR(USART0_RX_vect)  { store_uart_in_buf(UDR0, 0); }
  ISR(USART1_RX_vect)  { store_uart_in_buf(UDR1, 1); }
  ISR(USART2_RX_vect)  { store_uart_in_buf(UDR2, 2); }
  ISR(USART3_RX_vect)  { store_uart_in_buf(UDR3, 3); }
#endif

uint8_t SerialRead(uint8_t port) {
  #if defined(PROMICRO)
    #if defined(TEENSY20)
      if(port == 0) return Serial.read();
    #else
      #if (ARDUINO >= 100)
        if(port == 0) USB_Flush(USB_CDC_TX);
      #endif
      if(port == 0) return USB_Recv(USB_CDC_RX);      
    #endif
  #endif

#ifdef ESP32
  switch (port)
  {
    case 0: return Serial.read();
    case 2: return Serial2.read();
    #ifdef ESP32_BLUETOOTH_MSP
    case 3: return SerialBT.read();
    #endif
  }
  return 255;
#else
  uint8_t t = serialTailRX[port];
  uint8_t c = serialBufferRX[t][port];
  if (serialHeadRX[port] != t) {
    if (++t >= RX_BUFFER_SIZE) t = 0;
    serialTailRX[port] = t;
  }
  return c;
#endif
}

#if defined(SERIAL_RX)
  uint8_t SerialPeek(uint8_t port) {
    uint8_t c = serialBufferRX[serialTailRX[port]][port];
    if ((serialHeadRX[port] != serialTailRX[port])) return c; else return 0;
  }
#endif

  uint8_t SerialAvailable(uint8_t port)
  {
#if defined(PROMICRO)
#if !defined(TEENSY20)
    if (port == 0) return USB_Available(USB_CDC_RX);
#else
    if (port == 0) return T_USB_Available();
#endif
#endif
#ifdef ESP32
  switch (port)
  {
    case 0: return min(255u,(unsigned int)Serial.available());
    case 2: return min(255u, (unsigned int)Serial2.available());
    #ifdef ESP32_BLUETOOTH_MSP
    case 3: return min(255u,(unsigned int)SerialBT.available());
    #endif
  }
  return 0;
#else
  return ((uint8_t)(serialHeadRX[port] - serialTailRX[port]))%RX_BUFFER_SIZE;
#endif
}

/*
  uint8_t SerialUsedTXBuff(uint8_t port)
  {
#ifdef ESP32
  switch (port)
  {
  case 0: return serialBufferTXSize[0] - Serial.availableForWrite();
  case 2: return serialBufferTXSize[2] - Serial2.availableForWrite();
  }
  return 0;
#else
  return ((uint8_t)(serialHeadTX[port] - serialTailTX[port]))%TX_BUFFER_SIZE;
#endif
}
*/

uint16_t SerialAvailableForWrite(uint8_t port)
{
#ifdef ESP32
    switch (port)
    {
    case 0: return Serial.availableForWrite();
    case 2: return Serial2.availableForWrite();
    #ifdef ESP32_BLUETOOTH_MSP
    case 3: return SerialBTAvailableForWrite();
    #endif
    }
    return 127;
#else
    return TX_BUFFER_SIZE - ( ((uint8_t)(serialHeadTX[port] - serialTailTX[port])) % TX_BUFFER_SIZE);
#endif
}

void SerialSerialize(uint8_t port,uint8_t a) {
#ifdef ESP32
  switch (port)
  {
    case 0: Serial.write(a); break;
    case 2: Serial2.write(a); break;
    #ifdef ESP32_BLUETOOTH_MSP
    case 3: SerialBT.write(a); break;
    #endif
  }
#else
  uint8_t t = serialHeadTX[port];
  if (++t >= TX_BUFFER_SIZE) t = 0;
  serialBufferTX[t][port] = a;
  serialHeadTX[port] = t;
#endif;
}

void SerialWrite(uint8_t port,uint8_t c){
  SerialSerialize(port,c);UartSendData(port);
}
