/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * Copyright (c) 2014 h.zeller@acm.org. GNU public License.
 */

#include "serial-com.h"

#include <avr/io.h>
#include <avr/interrupt.h>

/*
 * Newer chips, such as the mega328p have different names for registers,
 * qualified with 0 (zero) to be able to distinguish more UARTs.
 */
#ifndef UCSRA
#  define USART_RXC_vect USART_RX_vect
#  define UCSRA UCSR0A
#  define RXC   RXC0
#  define UDR   UDR0
#  define UBRRH UBRR0H
#  define UBRRL UBRR0L
#  define RXCIE RXCIE0
#  define RXEN  RXEN0
#  define TXEN  TXEN0
#  define UCSRB UCSR0B
#  define UCSRC UCSR0C
#  define URSEL URSE0L
#  define UCSZ1 UCSZ01
#  define UCSZ0 UCSZ00
#  define UDRE  UDRE0
#  define UCSRC_SELECT 0
#else
#  define UCSRC_SELECT (1<<URSEL)
#endif

static volatile SerialCom *global_ser = 0; // ISR needs to access the Serial.

// Work around private visibility.
class SerialComISRWriter {
public:
  static void StuffByte(char c) {
    global_ser->StuffByte(c);
  }
};

ISR(USART_RXC_vect) {
  if (UCSRA & (1<<RXC)) {
    SerialComISRWriter::StuffByte(UDR);
  }
}

SerialCom::SerialCom() : dropped_reads_(0) {
  global_ser = this;  // our ISR needs access.
#if FEATURE_BAUD_CHANGE
  SetBaud(SERIAL_BAUDRATE);
#else
  const unsigned int divider = (F_CPU  / 16 / SERIAL_BAUDRATE) - 1;
  UBRRH = (unsigned char)(divider >> 8);
  UBRRL = (unsigned char) divider;
#endif
  UCSRB = (1<<RXCIE) | (1<<RXEN) | (1<<TXEN);  // read and write; interrupt read
  UCSRC = UCSRC_SELECT | (1<<UCSZ1) | (1<<UCSZ0); /*8bit*/
  sei();  // Enable interrupts.
}

#if FEATURE_BAUD_CHANGE
// For sanity (e.g. garbage in EEPROM), we only allow a certain set of
// valid baudrates.
bool SerialCom::IsValidBaud(uint16_t bd) {
  switch (bd) {
  case 300:  case 600:  case 1200:  case 2400:
  case 4800: case 9600: case 19200: case 38400:
    return true;
  default:
    return false;
  }
}

void SerialCom::SetBaud(uint16_t bd) {
  if (!IsValidBaud(bd)) bd = SERIAL_BAUDRATE;
  // The following is really expensive code-wise (>100 bytes), as we do a
  // 32bit division on a 8 bit processor. Right now, space is not tight yet. If
  // that ever becomes a problem, just compiling in a fixed baudrate would be
  // the answer.
  const unsigned int divider = (F_CPU  / 16 / bd) - 1;
  UBRRH = (unsigned char)(divider >> 8);
  UBRRL = (unsigned char) divider;
  baud_ = bd;
}
#endif

void SerialCom::write(char c) {
  while ( !( UCSRA & (1<<UDRE)) )  // wait for transmit buffer to be ready.
    ;
  UDR = c;
}

void SerialCom::StuffByte(char c) volatile {
  if (rx_buffer_.write_available())
    rx_buffer_.write(c);
  else
    ++dropped_reads_;
}

unsigned char SerialCom::read_available() volatile {
  return rx_buffer_.read_available();
}

char SerialCom::read() volatile {
  return rx_buffer_.read();
}
