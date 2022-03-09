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

#ifndef MAX31725_SENSOR_H
#define MAX31725_SENSOR_H

#include <stdint.h>

// https://datasheets.maximintegrated.com/en/ds/MAX31725-MAX31726.pdf
class Max31725TempSensor {
public:
  static constexpr uint8_t kBusAddress = 0x90;  // A0,1,2 all on GND

  // Get temperature in celsius.
  static float GetTemp();
};

#endif  // MAX31725_SENSOR_H
