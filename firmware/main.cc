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
#include <stdlib.h>
#include <string.h>

#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>

#include "i2c-master.h"
#include "max31725.h"
#include "pid.h"
#include "progmem.h"
#include "serial-com.h"
#include "serial-util.h"
#include "strfmt.h"

/* Note, we have plenty of flash, so we use the luxory of float numbers */

/*
 * Regular interval to take measurements and calculate a new PID control output
 */
static constexpr uint16_t kTimeBaseMs = 50;

static struct ConfigData {
  char dummy;  // Leave first char free, sometimes containing garbage.

  // PID info.
  float setpoint;
  float kp;
  float ki;
  float kd;

  void Write() {
    eeprom_write_float(&setpoint, setpoint);
    eeprom_write_float(&kp, kp);
    eeprom_write_float(&ki, ki);
    eeprom_write_float(&kd, kd);
  }
  void Read() {
    setpoint = eeprom_read_float(&setpoint);
    kp = eeprom_read_float(&kp);
    ki = eeprom_read_float(&ki);
    kd = eeprom_read_float(&kd);
  }
} EEMEM eeConfigData;

// Read number from serial and indicate if it was successful.
// Pre-fill the editline with the existing number with given digits.
static bool readNumber(SerialCom *comm, float *value, int digits) {
  char buffer[10];
  strcpy(buffer, strfmt_float(*value, digits));
  const char *str = readln(comm, buffer, sizeof(buffer), true);
  char *endptr = nullptr;
  float converted = strtod(str, &endptr);
  if (endptr != str) {
    *value = converted;
    println(comm, _P(" ✓"));
    return true;
  } else {
    println(comm, _P(" ✗ (not changed)"));
    return false;
  }
}

// A slow PWM of the heater. There is not much point in high-speed PWM with
// a zero-cross SSD, so we manually trigger with a timer interrupt.
// We go through one full cycle in kRange time interrupts.
//
// For safety, the value needs to be set for each cycle individually, otherwise
// we fail-safe to zero output.
class HeaterPWM {
  static constexpr uint8_t PORTB_LED_BIT = (1<<5);
  static constexpr uint8_t PORTD_SSR_BIT = (1<<3);

public:
  static constexpr uint8_t kRange = 32;

  HeaterPWM() {
    DDRB |= PORTB_LED_BIT;
    DDRD |= PORTD_SSR_BIT;
    SetOn(false);  // Safe starting point.
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

// There is only one global instance, accessible to the timer interrupt
static HeaterPWM sHeater;

/*
 * Setting up timer to trigger an interrupt every kTimeBaseMs
 */
static constexpr uint16_t kTimerCounts = F_CPU / 256 * kTimeBaseMs / 1000;
static constexpr uint16_t kTimerRegister = 65535 - kTimerCounts;

// Global time, increasing with timer interrupt. Wrapping around at 8 bit.
volatile uint8_t sGlobalTime = 0;

ISR(TIMER1_OVF_vect) {
  sGlobalTime++;
  TCNT1 = kTimerRegister;
  sHeater.HandleTimeInterrupt(sGlobalTime);
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

  // Raw values, formatted with tab to be easily parsed if needed.
  printRAM(com, strfmt_float(measured, 2));
  com->write('\t');
  printRAM(com, strfmt_float(target_temp, 1));
  com->write('\t');
  printRAM(com, strfmt_float(control_value, 2));
  com->write('\t');

  if (!with_graph) {
    println(com);
    return;
  }

  control_value *= 100.0f;  // Display in percent.

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
  // Prefix all lines with '#' to make it easy to filter out lines meant
  // for human consumption.
  print(com,
        _P("#\e[7m            ☕ Gaggia PID controller            \e[0m\r\n"
           "#(c) 2021 Henner Zeller  |  GNU Public License\r\n"
           "#Commands\r\n"
           "#\tl - Start log to console. Stop with any key.\r\n"
           "#\tL - Ditto. With 'graph'\r\n"
           "#\tn - Now. Print PID parameters, last temp and control out.\r\n"
           "#\r\n"
           "#\ts <setpoint> - set setpoint.\r\n"
           "#\tp <Kp>       - set proportional gain.\r\n"
           "#\ti <Ki>       - set integral gain.\r\n"
           "#\td <Kd>       - set derivative gain.\r\n"
           "#\r\n"
           "#\tw - Write PID parameters to EEPROM.\r\n"
           "#\tr - Restore PID parameters from EEPROM.\r\n"
           "#\th - This help\r\n"
           ));
}

int main() {
  I2CMaster::Init();
  initTimer();

  // PID interval once per full PWM cycle.
  PID pid(HeaterPWM::kRange * kTimeBaseMs * 0.001);
  Max31725TempSensor sensor;
  SerialCom com;

  uint8_t last_measure_period = 0;

  float measured_temp = 42.0;
  float pid_control_output = 0.0f;

  // Logging enable status and its current time.
  bool logging = false;
  bool logging_with_graph = false;
  uint16_t log_time = 0;

  // Serial line connect apparently resets Arduino Nano.
  // Use that as an opportunity to print welcome and usage message.
  printUsage(&com);

  eeConfigData.Read();
  pid.FromConfig(eeConfigData);

  // Get ready to sleep. We're only doing anything when timer or serial calls.
  sleep_enable();
  sei();

  const ProgmemPtr kLoggingCSVHeader = _P("TIME\tTEMP\tTARGET\tCONTROL");
  for (;;) {
    sleep_cpu();  // Wake on interrupts from timer or serial.

    if (sGlobalTime != last_measure_period) {  // timer triggered
      last_measure_period = sGlobalTime;
      measured_temp = sensor.GetTemp();

      // Update the PID just before the heater starts its next PWM cycle.
      if (last_measure_period % HeaterPWM::kRange == HeaterPWM::kRange - 1) {
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
        println(&com, _P("# logging stopped."));
        continue;
      }
      switch (c) {
      case 'l':    // Trigger logging.
      case 'L':    // ... with graph.
        logging = true;
        logging_with_graph = (c == 'L');
        println(&com, _P("# logging started. Stop with any key."));
        println(&com, kLoggingCSVHeader);
        break;

      case 's':
        print(&com, _P("set setpoint = "));
        readNumber(&com, &pid.setpoint, 1);
        pid.Sanitize();
        pid.Print(&com, false);
        break;

      case 'p':
        print(&com, _P("set Kp = "));
        readNumber(&com, &pid.kp, 3);
        pid.Sanitize();
        pid.Print(&com, false);
        break;

      case 'i':
        print(&com, _P("set Ki = "));
        readNumber(&com, &pid.ki, 3);
        pid.Sanitize();
        pid.Print(&com, false);
        break;

      case 'd':
        print(&com, _P("set Kd = "));
        readNumber(&com, &pid.kd, 3);
        pid.Sanitize();
        pid.Print(&com, false);
        break;

      case 'n':
        pid.Print(&com, true);
        break;

      case 'w':
        println(&com, _P("Storing PID parameters to EEPROM."));
        pid.Print(&com, false);
        pid.ToConfig(&eeConfigData);
        eeConfigData.Write();
        break;

      case 'r':
        println(&com, _P("Restore PID parameters from EEPROM."));
        eeConfigData.Read();
        pid.FromConfig(eeConfigData);
        pid.Print(&com, false);
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
    }
  }
}
