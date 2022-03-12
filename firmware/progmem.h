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

#ifndef PROGMEM_H
#define PROGMEM_H

#include <avr/pgmspace.h>

// Pointer to progmem string. Wrapped into separate type to have a type-safe
// way to deal with it.
struct ProgmemPtr {
  explicit constexpr ProgmemPtr(const char *d) : data(d) {}
  const char *data;
};
#define _P(s) ProgmemPtr(PSTR(s))

#endif // PROGMEM_H
