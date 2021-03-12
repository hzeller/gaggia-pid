# <h.zeller@acm.org>

DEFINES=-DF_CPU=16000000UL
DEFINES+=-DSERIAL_BAUDRATE=38400

# If the oled display is using a SSD1306 controller (typically the 0.96" are),
# then use this define
#DEFINES+=-DDISP_SSD1306

# I2C address of the display. 0x78 a common one.
#DEFINES+=-DDISPLAY_I2C=0x78

# Defines can also be given on the command-line
DEFINES+=$(USER_DEFINES)

TARGET_ARCH=-mmcu=atmega328
CC=avr-gcc
CFLAGS=-Os -W -Wall -Wextra -ffunction-sections -fdata-sections -mcall-prologues $(DEFINES) -I.
CXX=avr-g++
CXXFLAGS=$(CFLAGS) -std=c++11
AVRDUDE_DEVICE ?= /dev/ttyUSB0
AVRDUDE     = avrdude -p m328p -c arduino -b57600 -P $(AVRDUDE_DEVICE)
FLASH_CMD   = $(AVRDUDE) -e -U flash:w:main.hex
LINK=avr-g++ $(TARGET_ARCH) -Wl,-gc-sections
OBJECTS=main.o bdfont-support.o font-smalltext.o strfmt.o serial-com.o \
        i2c-master.o sh1106-display.o max31725.o

all : main.hex

main.elf: $(OBJECTS)
	$(LINK) -o $@ $(OBJECTS)
	avr-size $@

disasm: main.elf
	avr-objdump -C -S main.elf

main.hex: main.elf
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex

main.cc : font-smalltext.c

# This requires http://github.com/hzeller/bdfont.data to be installed
# For distribution, we don't depend on %.chars, so that a git cloned project
# does not need bdfont-data-gen installed and compiles with provided files.
font-%.c: # %.chars
	bdfont-data-gen -s fonts/$*.bdf $* -C $*.chars

flash: main.hex
	$(FLASH_CMD)

clean:
	rm -f $(OBJECTS) main.elf main.hex

# Set to 8Mhz RC on Attiny85
#fuse:
#	$(AVRDUDE) -U lfuse:w:0xe2:m

# Make sure to recompile everything if cflags change.
.compiler-flags: FORCE
	@echo '$(CXX) $(CXXFLAGS)' | cmp -s - $@ || echo '$(CXX) $(CXXFLAGS)' > $@

%.o : %.c .compiler-flags
	$(CC) $(TARGET_ARCH) $(CFLAGS) -c -o $@ $<

%.o : %.cc .compiler-flags
	$(CXX) $(TARGET_ARCH) $(CXXFLAGS) -c -o $@ $<

.PHONY: FORCE
