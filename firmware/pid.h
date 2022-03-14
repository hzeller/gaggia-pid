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

#ifndef PID_H
#define PID_H

#include "serial-com.h"

// PID with output range 0..1.0
class PID {
public:
  // Initialize PID controller with the given delta-T time update period.
  PID(float dt) : dt_(dt) {}

  // Public fields to be modified directly or read/written from config.
  float setpoint = 50.0f;

  float kp = 1.0f;  // proportional factor.
  float kd = 0.0f;  // differential factor.
  float ki = 0.0f;  // integral factor.

  // Update with new measurement. Return new control output value which
  // is in the range of 0 ... 1.0
  float Update(float measurement);

  // Print current status to serial.
  void Print(SerialCom *com, bool include_last_values) const;

  // Read configuration from something that has the fields we're interested in
  template <typename C>
  void FromConfig(const C &c) {
    setpoint = c.setpoint;
    kp = c.kp;
    kd = c.kd;
    ki = c.ki;
    Sanitize();  // make sure these things are reasonable
  }

  template <typename C>
  void ToConfig(C *c) {
    c->setpoint = setpoint;
    c->kp = kp;
    c->kd = kd;
    c->ki = ki;
  }

  // Sanitize the internal constants to some valid value.
  void Sanitize();

private:
  const float dt_;   // Time interval
  float last_error_ = 0;
  float integral_sum_ = 0;
  bool integral_on_hold_ = false;

  // Mostly for printing
  float last_measurement_ = 0;
  float last_control_output_ = 0;
};

#endif  // PID_H
