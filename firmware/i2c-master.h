// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
//
// Copyright (C) 2019 Henner Zeller <h.zeller@acm.org>
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
#ifndef I2C_MASTER_H
#define I2C_MASTER_H

#include <stdint.h>

// A simple I2C master needed to communicate with the display.
class I2CMaster {
public:
  static void Init();

  static void Enable(bool b);
  static void StartTransmission(uint8_t address);
  static void Write(uint8_t b);
  static uint8_t Read(bool ack);
  static void FinishTransmission();
};

#endif // I2C_MASTER_H
