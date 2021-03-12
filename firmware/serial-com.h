/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * Copyright (c) 2014 h.zeller@acm.org. GNU public License.
 */

#ifndef _AVR_SERIAL_H_
#define _AVR_SERIAL_H_

#include <stdint.h>
#include "ringbuffer.h"

class SerialCom {
  enum { RX_BUFFER_BITS = 6 };  // Buffer uses 2^BUFFER_BITS bytes.
public:
  // Setting up the serial interface. Baudrate is given as -DSERIAL_BAUDRATE
  // Otherwise simply 8N1.
  // Also assumes -DF_CPU to be set.
  // Internally maintains an incoming buffer with 2^BUFFER_BITS size.
  SerialCom();

#if FEATURE_BAUD_CHANGE
  // Supported baudrates.
  static bool IsValidBaud(uint16_t bd);

  // Set baud to one of the supported ones or default-baudrate.
  void SetBaud(uint16_t baud);

  // Returns current baudrate.
  uint16_t baud() const { return baud_; }
#endif

  // Write a single character. Blocks if buffer full.
  void write(char c);

  // Bytes ready to read. Can be up to buffer-size.
  unsigned char read_available() volatile;

  // Read a single chracter. Blocks if nothing in buffer.
  char read() volatile;

  // Number of incoming bytes that were dropped on the floor because read()
  // was not called in time to pick them up.
  unsigned short dropped_rx() const { return dropped_reads_; }

private:
  friend class SerialComISRWriter;
  void StuffByte(char c) volatile;  // Stuff into buffer. Called by ISR.
#if FEATURE_BAUD_CHANGE
  uint16_t baud_;
#endif
  uint16_t dropped_reads_;

  RingBuffer<char, RX_BUFFER_BITS> rx_buffer_;
  // Note: using interrupt driven transmit didn't change performance at all
  // so not bothering for code simplicity sake.
};

#endif  // _AVR_SERIAL_H_
