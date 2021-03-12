struct LogEntry {
  static constexpr uint8_t kShiftTemp = 150;
  // Temperature shifted: we only have range 0..1023 and but would like
  // to encode temperatures around 1000 deciidegrees (100.0°C). So let's
  // shift it and cut off some uninteresting lower range.
  LogEntry(uint16_t degrees, uint8_t pwm) {
    v.temp_internal = degrees > kShiftTemp ? degrees - kShiftTemp : 0;
    v.pwm = pwm;
  }

  explicit LogEntry(uint16_t serialized) : raw(serialized) {}


  uint16_t degrees() const { return v.temp_internal + kShiftTemp; }
  uint8_t pwm() const { return v.pwm; }
  uint16_t serialize() const { return raw; }

  union {
    struct {
      uint16_t temp_internal : 10;
      uint16_t pwm : 6;
    } v;
    uint16_t raw;
  };
};
static_assert(sizeof(struct LogEntry) == 2, "Too big");

static uint8_t flip(uint8_t b) {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}

      uint8_t lower = (temp >> 3) & 0x1F;
      uint32_t bargraph = 0xC0000000 >> (31 - lower);
      display.FillStripeRange(pos + 3, pos + 7, 24, flip(bargraph >> 24));
      display.FillStripeRange(pos + 3, pos + 7, 32, flip((bargraph >> 16)&0xff));
      display.FillStripeRange(pos + 3, pos + 7, 40, flip((bargraph >> 8)&0xff));
      display.FillStripeRange(pos + 3, pos + 7, 48, flip(bargraph & 0xff));
