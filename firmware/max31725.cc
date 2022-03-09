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

#include "max31725.h"
#include "i2c-master.h"

float Max31725TempSensor::GetTemp() {
  I2CMaster::StartTransmission(kBusAddress);
  I2CMaster::Write(0);  // Address zero - temperature register
  I2CMaster::FinishTransmission();

  I2CMaster::StartTransmission(kBusAddress + 1);  // Let's see what we got
  int16_t result = I2CMaster::Read(true) << 8;
  result |= I2CMaster::Read(false);
  I2CMaster::FinishTransmission();

  return result / 256.0f;
}
