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

#include <math.h>

#include "pid.h"

#include "progmem.h"
#include "serial-util.h"
#include "strfmt.h"

float PID::Update(float measurement) {
  const float error = setpoint - measurement;
  const float p_val = kp * error;

  if (!integral_on_hold_) integral_sum_ += error * dt_;
  const float i_val = ki * integral_sum_;

  const float d_val = kd * (error - last_error_) / dt_;
  last_error_ = error;

  const float raw_output = p_val + i_val + d_val;
  float control_output = raw_output;
  if (control_output < 0.0f) control_output = 0.0f;  // clamp.
  if (control_output > 1.0f) control_output = 1.0f;

  // We're in saturation, if we had to clamp the output.
  const bool in_saturation = (control_output != raw_output);

  // If we are in saturation and currently keep pushing towards correcting
  // the error (i.e. output and error have the same sign), let's stop
  // integrating (we already do our best); if we kept adding to our integration,
  // it would take a long while to undo the term that went beyond clamping.
  const bool same_sign = !((error < 0.0f) ^ (raw_output < 0.0f));
  integral_on_hold_ = in_saturation && same_sign;

  last_measurement_ = measurement;
  last_control_output_ = control_output;

  return control_output;
}

void PID::Print(SerialCom *com, bool include_last_values) const {
#define PRINT_VAL(desc, value, resolution)              \
  print(com, _P(desc));                                 \
  printRAM(com, strfmt_float(value, resolution));       \
  print(com, _P("; "));

  PRINT_VAL("setpoint=", setpoint, 2);
  PRINT_VAL("Kp=", kp, 3);
  PRINT_VAL("Ki=", ki, 3);
  PRINT_VAL("Kd=", kd, 3);

  if (include_last_values) {
    PRINT_VAL("LAST: input=", last_measurement_, 2);
    PRINT_VAL("last-output=", last_control_output_, 2);

    // Some debug output of the last calculation that took place.
    // (don't have differential value available right now, but currently
    // not using it)
    const float p_val = kp * last_error_;
    const float i_val = ki * integral_sum_;

    print(com, _P("kp*error + ki*integral = ("));
    printRAM(com, strfmt_float(p_val, 3));
    print(com, _P(") + ("));
    printRAM(com, strfmt_float(i_val, 3));
    print(com, _P(") = "));
    printRAM(com, strfmt_float(p_val + i_val, 3));
    if (integral_on_hold_) print(com, _P(" integration on hold"));
  }
  println(com);
#undef PRINT_VAL
}

static void sanitize_range(float min, float max, float *f, float sanitized) {
  if (isnan(*f) || *f < min || *f > max)
    *f = sanitized;
}
void PID::Sanitize() {
  // If values are out-of-whack, set to safe defaults.
  sanitize_range(0.0f, 110.0f, &setpoint, 85.0f);
  sanitize_range(-10.0f, 10.0f, &kp, 0.1);
  sanitize_range(-10.0f, 10.0f, &ki, 0);
  sanitize_range(-10.0f, 10.0f, &kd, 0);
}
