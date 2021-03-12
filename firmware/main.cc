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
#include <avr/sleep.h>
#include <util/delay.h>

#include "i2c-master.h"
#include "max31725.h"
#include "serial-com.h"
#include "sh1106-display.h"
#include "strfmt.h"

#include "font-smalltext.h"   // generated from smalltext.chars

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

/*
 * Setting up timer to trigger an interrupt every kTimeBaseMs
 */
static constexpr uint16_t kTimerCounts = F_CPU / 256 * kTimeBaseMs / 1000;
static constexpr uint16_t kTimerRegister = 65535 - kTimerCounts;
volatile uint8_t measure_period = 0;
ISR(TIMER1_OVF_vect) {
  measure_period++;
  TCNT1 = kTimerRegister;
}

void initTimer() {
  TCNT1 = kTimerRegister;
  TCCR1A = 0x00;
  TCCR1B = (1 << CS12); // prescale 256 (also see kTimerCounts)
  TIMSK1 = (1 << TOIE1);
}

// Print temperature and control value, and show in 'graphical' form.
void printTemp(SerialCom *com, bool markline,
               uint16_t decidegrees, uint8_t control_value,
               uint8_t target_temp) {
  const ProgmemPtr vt100Underline = _P("\e[4m");
  const ProgmemPtr vt100Inverse   = _P("\e[7m");
  const ProgmemPtr vt100Reset     = _P("\e[0m");

  // Raw values, formatted with tab to be easily parsed if needed.
  printRAM(com, strfmt(decidegrees, 1));
  com->write('\t');
  printRAM(com, strfmt(control_value, 0));
  com->write('\t');

  // Useful for human consumption: some graph representation of above values
  constexpr int kBaselineOffset = 15;  // Cut off boring lower part.
  decidegrees += 4;  // Full degree matches up with center of character cell
  const int degree_display = (decidegrees / 10) - kBaselineOffset;
  const int target_display = target_temp > kBaselineOffset ?
    target_temp - kBaselineOffset : 0;
  uint8_t max_run = target_display;
  if (degree_display > max_run) max_run = degree_display;
  if (control_value > max_run)  max_run = control_value;

  bool reset_needed = markline || target_display > 0;
  if (markline) print(com, vt100Underline);
  for (int s = 0; s < max_run+1; ++s) {
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
    else if (s == control_value)
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
                "\tn - Now: current temperature and control PWM output.\r\n"
                "\th - This help\r\n"
                ));
}

int main() {
  I2CMaster::Init();
  initTimer();

  SH1106Display display;
  Max31725TempSensor sensor;
  SerialCom com;

  uint8_t last_measure_period = 0;
  char last_command = ' ';

  // Current state
  uint16_t decidegrees = 0;   // From sensor
  uint8_t pid_control_output = 12;
  uint8_t set_temperature = 94;  // TODO: configure and store in EEPROM

  // Logging enable status and its current time.
  bool logging = false;
  uint16_t log_time = 0;

  // Unmodified Arduino Nano will reset on terminal connect. Make that visible.
  display.ClearScreen();
  display.Print(font_smalltext, 0, 24, "Reset");
  _delay_ms(200);
  display.ClearScreen();

  // Get ready to sleep. We're only doing anything when timer or serial calls.
  sleep_enable();
  sei();

  const ProgmemPtr kLoggingCSVHeader = _P("TIME\tTEMP\tCONTROL\tGRAPH");
  for (;;) {
    sleep_cpu();  // Wake on interrupts from timer or serial.

    if (measure_period != last_measure_period) {  // timer triggered
      last_measure_period = measure_period;
      const int32_t thirty_twos_degrees = sensor.GetTemp() >> 3;
      decidegrees = thirty_twos_degrees > 0
        ? (thirty_twos_degrees * 3125 + 5000) / 10000
        : 0;

      uint8_t pos = display.Print(font_smalltext, 0, 24,
                                  strfmt(decidegrees, 1));
      pos = display.Print(font_smalltext, pos, 24, "°C");
      display.FillStripeRange(pos, 127, 24, 0x00);  // Clear rest of line

      if (logging) {
        printRAM(&com, strfmt(log_time, 3));
        com.write('\t');
        printTemp(&com, log_time==0, decidegrees, pid_control_output,
                  set_temperature);
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
      case 'n': {   // Print value 'now'
        if (last_command != 'n')  // First time: print header
          println(&com, kLoggingCSVHeader);
        print(&com, _P("Now\t"));
        printTemp(&com, false, decidegrees, pid_control_output,
                  set_temperature);
        break;
      }

      case 'l':    // Trigger logging.
        logging = true;
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
