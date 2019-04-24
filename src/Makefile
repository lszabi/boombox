# setup

COMPILE_OPTS = -mcpu=cortex-m3 -mthumb -Wall
#COMPILE_OPTS += -g -O0
INCLUDE_DIRS = -I . -I lib/inc
LIBRARY_DIRS = -L lib

CC = arm-none-eabi-gcc
CFLAGS = $(COMPILE_OPTS) $(INCLUDE_DIRS)

CXX = arm-none-eabi-g++
CXXFLAGS = $(COMPILE_OPTS) $(INCLUDE_DIRS)

AS = arm-none-eabi-gcc
ASFLAGS = $(COMPILE_OPTS) -c

LD = arm-none-eabi-gcc
LDFLAGS = -Wall -Wl,--gc-sections,-u,Reset_Handler -mcpu=cortex-m3 -mthumb $(INCLUDE_DIRS) $(LIBRARY_DIRS) -T stm32.ld

OBJCP = arm-none-eabi-objcopy
OBJCPFLAGS = -O binary

AR = arm-none-eabi-ar
ARFLAGS = cr

OBJS = main.o utils.o leds.o i2c.o dsp.o
LIBS = 
PROJECT = amp
LIBSTM32_OUT = lib/libstm32.a

# all

all: $(PROJECT).bin

# main

$(PROJECT).elf: $(OBJS) stm32f10x_it.o $(LIBSTM32_OUT)
	$(LD) $(LDFLAGS) stm32f10x_it.o $(OBJS) $(LIBSTM32_OUT) --output $@ $(LIBS)

$(PROJECT).bin: $(PROJECT).elf
	$(OBJCP) $(OBJCPFLAGS) $< $@


# flash

flash: $(PROJECT).bin
	dfu-util -a 1 -R -d 1EAF:0003 -D $(PROJECT).bin


# libstm32.a

LIBSTM32_OBJS = \
 lib/src/stm32f10x_adc.o \
 lib/src/stm32f10x_bkp.o \
 lib/src/stm32f10x_can.o \
 lib/src/stm32f10x_dma.o \
 lib/src/stm32f10x_exti.o \
 lib/src/stm32f10x_flash.o \
 lib/src/stm32f10x_gpio.o \
 lib/src/stm32f10x_i2c.o \
 lib/src/stm32f10x_iwdg.o \
 lib/src/stm32f10x_lib.o \
 lib/src/stm32f10x_nvic.o \
 lib/src/stm32f10x_pwr.o \
 lib/src/stm32f10x_rcc.o \
 lib/src/stm32f10x_rtc.o \
 lib/src/stm32f10x_spi.o \
 lib/src/stm32f10x_systick.o \
 lib/src/stm32f10x_tim.o \
 lib/src/stm32f10x_tim1.o \
 lib/src/stm32f10x_usart.o \
 lib/src/stm32f10x_vector.o \
 lib/src/stm32f10x_wwdg.o \
 lib/src/cortexm3_macro.o
 # lib/src/usb_core.o \
 # lib/src/usb_init.o \
 # lib/src/usb_mem.o \
 # lib/src/usb_regs.o \
 # lib/src/usb_sil.o

$(LIBSTM32_OUT): $(LIBSTM32_OBJS)
	$(AR) $(ARFLAGS) $@ $(LIBSTM32_OBJS)

$(LIBSTM32_OBJS): stm32f10x_conf.h

# clean

clean:
	rm -f *.o *.bin *.elf

clean-lib:
	rm -f lib/src/*.o $(LIBSTM32_OUT)

clean-all: clean-lib clean

#debug

%.S : %.c
	$(CC) $(CFLAGS) -S -o $@ $<