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

#ifndef SERIAL_UTIL_H
#define SERIAL_UTIL_H

#include "serial-com.h"
#include "progmem.h"

// Print from progmem to serial. Use the _P() macro to get such a string.
void print(SerialCom *out, ProgmemPtr str);

// Print a newline.
void println(SerialCom *out);

// Print string, append newline.
void println(SerialCom *out, ProgmemPtr str);

// Printing from RAM with a slightly more obnoxious name to remind of
// prefering the Progmem-kind.
void printRAM(SerialCom *out, const char *str);

// Read line into buffer. Line will not contain newline chracter
// and \0-terminated
// If "edit" is true, starts out with the existing content in the buffer
// (in which case, it has to be \0 terminated)
const char* readln(SerialCom *comm, char *buffer, int size, bool edit = false);

#endif  // SERIAL_UTIL_H
