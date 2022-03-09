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

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <math.h>
#include <util/delay.h>

#include "i2c-master.h"
#include "max31725.h"
#include "serial-com.h"
#include "strfmt.h"

/* Note, we have plenty of flash, so we use the luxory of float numbers */

/*
 * Regular interval to take measurements and calculate a new PID control output
 */
static constexpr uint16_t kTimeBaseMs = 100;

/*
 * Printing to serial interface utility functions
 */
// Pointer to progmem string. Wrapped into separate type to have a type-safe
// way to deal with it.
struct ProgmemPtr {
  explicit constexpr ProgmemPtr(const char *d) : data(d) {}
  const char *data;
};
#define _P(s) ProgmemPtr(PSTR(s))

static void print(SerialCom *out, ProgmemPtr str) {
 char c;
 while ((c = pgm_read_byte(str.data++)) != 0x00)
   out->write(c);
}
static void println(SerialCom *out) { print(out, _P("\r\n")); }
static void println(SerialCom *out, ProgmemPtr str) {
  print(out, str);
  println(out);
}

// Printing from RAM with a slightly more obnoxious name to remind of
// prefering the Progmem-kind.
static void printRAM(SerialCom *out, const char *str) {
  while (*str) out->write(*str++);
}

// PID with output range 0..1.0
class PID {
public:
  PID(float dt) : dt_(dt) {}

  // Update with new measurement. Return new control output value.
  float Update(float measurement) {
    const float error = setpoint - measurement;
    const float p_val = kp * error;

    integral_sum_ += error * dt_;
    const float i_val = ki * integral_sum_;

    const float d_val = kd * (error - last_error_) / dt_;
    last_error_ = error;

    const float control_output = p_val + i_val + d_val;
    if (control_output < 0.0f) return 0.0f;
    if (control_output > 1.0f) return 1.0f;
    return control_output;
  }

  // Public fields to be modified directly.
  float setpoint = 0.0f;

  float kp = 1.0f;  // proportional factor
  float kd = 0.0f;  // differential factor
  float ki = 0.0f;  // integrate factor.

private:
  const float dt_;   // Time interval
  float last_error_ = 0;
  float integral_sum_ = 0;
};

class Heater {
  static constexpr uint8_t PORTB_LED_BIT = (1<<5);
  static constexpr uint8_t PORTD_SSR_BIT = (1<<3);

public:
  static constexpr uint8_t kRange = 16;

  Heater() {
    DDRB |= PORTB_LED_BIT;
    DDRD |= PORTD_SSR_BIT;
  }

  // Value of 0..1.0
  // For safety, needs to be actively set once per period, otherwise will
  // fall-back to zero. This prevents lock-up situations.
  void SetValue(float v) {
    if (v < 0.0) v = 0;
    if (v > 1.0) v = 1.0;
    value_ = v * kRange;
  }

  void HandleTimeInterrupt(uint8_t global_time) volatile {
    if ((global_time % kRange) == 0) {
      pwm_state_ = value_;
      value_ = 0;  // Will require to SetValue() again.
    }

    SetOn(pwm_state_ != 0);
    if (pwm_state_ > 0) --pwm_state_;
  }

private:
  void SetOn(bool b) volatile {
    if (b) {
      PORTB |= PORTB_LED_BIT;
      PORTD |= PORTD_SSR_BIT;
    } else {
      PORTB &= ~PORTB_LED_BIT;
      PORTD &= ~PORTD_SSR_BIT;
    }
  }

  volatile uint8_t value_ = 0;      // Range from 0x00..kRange
  volatile uint8_t pwm_state_ = 0;  // Current PWM counter.
};

/*
 * Setting up timer to trigger an interrupt every kTimeBaseMs
 */
static constexpr uint16_t kTimerCounts = F_CPU / 256 * kTimeBaseMs / 1000;
static constexpr uint16_t kTimerRegister = 65535 - kTimerCounts;

static Heater sHeater;
volatile uint8_t measure_period = 0;

ISR(TIMER1_OVF_vect) {
  measure_period++;
  TCNT1 = kTimerRegister;
  sHeater.HandleTimeInterrupt(measure_period);
}

void initTimer() {
  TCNT1 = kTimerRegister;
  TCCR1A = 0x00;
  TCCR1B = (1 << CS12); // prescale 256 (also see kTimerCounts)
  TIMSK1 = (1 << TOIE1);
}

// Print temperature and control value, and show in 'graphical' form.
void printTemp(SerialCom *com, bool markline,
               float measured, float control_value,
               float target_temp,
               bool with_graph) {
  const ProgmemPtr vt100Underline = _P("\e[4m");
  const ProgmemPtr vt100Inverse   = _P("\e[7m");
  const ProgmemPtr vt100Reset     = _P("\e[0m");

  control_value *= 100.0f;  // Display in percent.
  // Raw values, formatted with tab to be easily parsed if needed.
  printRAM(com, strfmt_float(measured, 2));
  com->write('\t');
  printRAM(com, strfmt_float(target_temp, 1));
  com->write('\t');
  printRAM(com, strfmt_float(control_value, 0));
  print(com, _P("%\t"));

  if (!with_graph) {
    println(com);
    return;
  }

  // Useful for human consumption: some graph representation of above values
  constexpr float kBaselineOffset = 15;  // Cut off boring lower part.
  const uint8_t degree_display = measured - kBaselineOffset;
  const uint8_t target_display = target_temp > kBaselineOffset ?
    target_temp - kBaselineOffset : 0;
  uint8_t max_run = target_display;
  if (degree_display > max_run) max_run = degree_display;
  if (control_value > max_run)  max_run = control_value;

  const int decidegrees = 10.0f * measured;
  bool reset_needed = markline || target_display > 0;
  if (markline) print(com, vt100Underline);
  for (uint8_t s = 0; s < max_run+1; ++s) {
    // The target temperature is an inverted character cell, so that we can
    // see the temperature line 'underneath'.
    if (s > 0 && s == target_display) print(com, vt100Inverse);

    if (s == degree_display) {     // These are full degrees
      switch (decidegrees % 10) {  // Graphically divide decis into sub-blocks
      case 0: case 1: case 2:         print(com, _P("\u258f")); break;
      case 3: case 4: case 5: case 6: print(com, _P("|"));      break;
      case 7: case 8: case 9:         print(com, _P("\u2595")); break;
      }
    }
    else if (s == (uint8_t)control_value)
      com->write('*');
    else
      com->write(' ');

    if (s > 0 && s == target_display) {
      print(com, vt100Reset);
      reset_needed = false;
    }
  }

  if (reset_needed) print(com, vt100Reset);
  println(com);
}

void printUsage(SerialCom *com) {
  print(com, _P("\e[7m            ☕ Gaggia PID controller            \e[0m\r\n"
                "(c) 2021 Henner Zeller  |  GNU Public License\r\n"
                "Commands\r\n"
                "\tl - Start log to console. Stop with any key.\r\n"
                "\tL - Ditto. With 'graph'\r\n"
                "\tn - Now: current temperature and control PWM output.\r\n"
                "\ts <setpoint> - set setpoint.\r\n"
                "\tp <Kp>       - set proportional gain.\r\n"
                "\td <Kd>       - set derivative gain.\r\n"
                "\ti <Ki>       - set integral gain.\r\n"
                "\th - This help\r\n"
                ));
}

int main() {
  I2CMaster::Init();
  initTimer();

  PID pid(Heater::kRange * 0.1);  // Time interval once per PWM cycle.
  Max31725TempSensor sensor;
  SerialCom com;

  uint8_t last_measure_period = 0;
  char last_command = ' ';

  float measured_temp = 42.0;

  // Testing with low temperature for now.
  pid.setpoint = 93.0f;  // TODO: get from EEPROM
  pid.kp = 0.1;
  pid.kd = 0.0;
  pid.ki = 0.0;
  // Logging enable status and its current time.
  bool logging = false;
  bool logging_with_graph = false;
  uint16_t log_time = 0;

  // Get ready to sleep. We're only doing anything when timer or serial calls.
  sleep_enable();
  sei();

  float pid_control_output = 0.0f;

  const ProgmemPtr kLoggingCSVHeader = _P("TIME\tTEMP\tTARGET\tCONTROL");
  for (;;) {
    sleep_cpu();  // Wake on interrupts from timer or serial.

    if (measure_period != last_measure_period) {  // timer triggered
      last_measure_period = measure_period;
      measured_temp = sensor.GetTemp();

      // Update the PID just before the heater starts its next PWM cycle.
      if (last_measure_period % Heater::kRange == Heater::kRange - 1) {
        pid_control_output = pid.Update(measured_temp);
        sHeater.SetValue(pid_control_output);
      }

      if (logging) {
        printRAM(&com, strfmt(log_time, 3));
        com.write('\t');
        printTemp(&com, log_time==0, measured_temp, pid_control_output,
                  pid.setpoint, logging_with_graph);
        log_time += kTimeBaseMs;
        log_time %= 1000;
      }
    }

    // Check for commands.
    while (com.read_available()) {
      const char c = com.read();
      if (logging) {   // stops at any key. Consume it.
        logging = false;
        println(&com, _P("logging stopped."));
        continue;
      }
      switch (c) {
      case 'n':
      case 'N': {   // Print value 'now'
        if (last_command != 'n')  // First time: print header
          println(&com, kLoggingCSVHeader);
        print(&com, _P("Now\t"));
        printTemp(&com, false, measured_temp, pid_control_output,
                  pid.setpoint, c == 'N');
        break;
      }

      case 'l':    // Trigger logging.
      case 'L':    // Trigger logging.
        logging = true;
        logging_with_graph = (c == 'L');
        println(&com, _P("logging started. Stop with any key."));
        println(&com, kLoggingCSVHeader);
        break;

      case '\r': case '\n':
        // Ignore. Allows to idempotently reset logging before starting with 'l'
        break;

      default:
        com.write(c);
        println(&com, _P(" : Unknown command."));
        /* fallthrough */
      case '?': case 'h':
        printUsage(&com);
        break;
      }
      last_command = c;
    }
  }
}
