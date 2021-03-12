/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * Copyright (c) 2014 h.zeller@acm.org. GNU public License.
 */

#ifndef RINGBUFFER_H_
#define RINGBUFFER_H_

#include <stdint.h>

// A fixed allocated ringbuffer.
// Prepared to be used one-sided in an interrupt handler, so everything is
// volatile.
//
// Template parameter is type and bits to represent the buffer size, so buffer
// size is 2^BUFFER_BITS (that way, modulo operations are cheap mask
// operations).
template<typename T, int kBufferBits, bool kBlockingWrite = true>
class RingBuffer {
public:
  typedef uint8_t size_type;

  // Number of spots available to write.
  size_type write_available() const volatile {
      return (read_pos_ - (write_pos_ + 1)) & MODULO_MASK;
  }

  // Write. Blocks until write_available()
  void write(T c) volatile {
    while (!write_available()) {
      if (!kBlockingWrite) get_then_advance_read_pos();  // Discard.
    }
    buffer_[write_pos_] = c;
    write_pos_ = (write_pos_ + 1) & MODULO_MASK;
  }

  // Number of bytes ready to read.
  size_type read_available() const volatile {
      return (write_pos_ - read_pos_) & MODULO_MASK;
  }

  // Return relative to the current read position. No bounds checking,
  // make sure to not read more than read_available().
  const T peek(int pos) const volatile {
    return buffer_[(read_pos_ + pos) & MODULO_MASK];
  }

  // Read a byte. Blocks if read_available() == 0.
  const T read() volatile {
    while (!read_available())
      ;
    return buffer_[get_then_advance_read_pos()];
  }

private:
  static constexpr uint8_t MODULO_MASK = (1 << kBufferBits) - 1;

  size_type get_then_advance_read_pos() volatile {
    size_type old_pos = read_pos_;
    read_pos_ = (read_pos_ + 1) & MODULO_MASK;
    return old_pos;
  }
  volatile size_type write_pos_ = 0;
  volatile size_type read_pos_ = 0;
  T buffer_[1 << kBufferBits];
};

#endif  // RINGBUFFER_H_
