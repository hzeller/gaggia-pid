// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
//
// Copyright (C) 2021 Henner Zeller <h.zeller@acm.org>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "serial-util.h"

void print(SerialCom *out, ProgmemPtr str) {
 char c;
 while ((c = pgm_read_byte(str.data++)) != 0x00)
   out->write(c);
}

void println(SerialCom *out) { print(out, _P("\r\n")); }
void println(SerialCom *out, ProgmemPtr str) {
  print(out, str);
  println(out);
}

void printRAM(SerialCom *out, const char *str) {
  while (*str) out->write(*str++);
}

const char* readln(SerialCom *comm, char *buffer, int size, bool edit) {
  char *pos = buffer;
  if (edit) {
    for (/**/; pos < buffer + size && *pos; pos++) {
      comm->write(*pos);
    }
  }
  for (;;) {
    char c = comm->read();
    if (c == '\033') {
      *buffer = '\0';   // Cancel editing. Return empty string.
      return buffer;
    }
    if (c == '\r' || c == '\n')
      break;  // Newline.
    if (c == 127) {  // Backspace. Provide rudimentary editing.
      if (pos > buffer) {
        --pos;
        comm->write('\b');
        comm->write(' ');
        comm->write('\b');
      }
      continue;
    }
    if (pos < buffer + size - 1) {
      comm->write(c);
      *pos++ = c;
    } else {
      comm->write('\a');  // Too long input. beep.
    }
  }
  *pos = '\0';
  return buffer;
}
