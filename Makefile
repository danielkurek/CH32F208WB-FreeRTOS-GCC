PROJECT = CH32F20x

CC	= arm-none-eabi-gcc
CPP	= arm-none-eabi-g++
AS	= arm-none-eabi-as
LD	= arm-none-eabi-g++
CP	= arm-none-eabi-objcopy
OS	= arm-none-eabi-size
OD	= arm-none-eabi-objdump

VECTOR	= lib/ch32f20x_libs/gcc_startupfile_ch32f208wb.s

LDSCRIPT = lib/ch32f20x_libs/gcc_linkerfile_ch32f208wb.ld
LDFLAGS  = -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -T$(LDSCRIPT) -lnosys -Wl,--gc-sections -Wl,-Map,$(PROJECT).map

INC = -I.\
      -Ilib/ch32f20x_libs/CMSIS\
      -Ilib/ch32f20x_libs/Debug\
      -Ilib/ch32f20x_libs/StdPeriphDriver/inc\
      -I./FreeRTOS/Source\
      -I./FreeRTOS/Source/include\
      -I./FreeRTOS/Source/portable/Common\
      -I./FreeRTOS/Source/portable/GCC/ARM_CM3\
      -I./FreeRTOS/Source/portable/MemMang\
      -Ilib/

SRC = $(VECTOR) \
      lib/ch32f20x_libs/CMSIS/core_cm3.c\
      lib/ch32f20x_libs/Debug/debug.c\
      lib/ch32f20x_libs/StdPeriphDriver/src/ch32f20x_adc.c\
      lib/ch32f20x_libs/StdPeriphDriver/src/ch32f20x_bkp.c\
      lib/ch32f20x_libs/StdPeriphDriver/src/ch32f20x_can.c\
      lib/ch32f20x_libs/StdPeriphDriver/src/ch32f20x_crc.c\
      lib/ch32f20x_libs/StdPeriphDriver/src/ch32f20x_dac.c\
      lib/ch32f20x_libs/StdPeriphDriver/src/ch32f20x_dbgmcu.c\
      lib/ch32f20x_libs/StdPeriphDriver/src/ch32f20x_dma.c\
      lib/ch32f20x_libs/StdPeriphDriver/src/ch32f20x_dvp.c\
      lib/ch32f20x_libs/StdPeriphDriver/src/ch32f20x_exti.c\
      lib/ch32f20x_libs/StdPeriphDriver/src/ch32f20x_flash.c\
      lib/ch32f20x_libs/StdPeriphDriver/src/ch32f20x_fsmc.c\
      lib/ch32f20x_libs/StdPeriphDriver/src/ch32f20x_gpio.c\
      lib/ch32f20x_libs/StdPeriphDriver/src/ch32f20x_i2c.c\
      lib/ch32f20x_libs/StdPeriphDriver/src/ch32f20x_iwdg.c\
      lib/ch32f20x_libs/StdPeriphDriver/src/ch32f20x_misc.c\
      lib/ch32f20x_libs/StdPeriphDriver/src/ch32f20x_opa.c\
      lib/ch32f20x_libs/StdPeriphDriver/src/ch32f20x_pwr.c\
      lib/ch32f20x_libs/StdPeriphDriver/src/ch32f20x_rcc.c\
      lib/ch32f20x_libs/StdPeriphDriver/src/ch32f20x_rng.c\
      lib/ch32f20x_libs/StdPeriphDriver/src/ch32f20x_rtc.c\
      lib/ch32f20x_libs/StdPeriphDriver/src/ch32f20x_sdio.c\
      lib/ch32f20x_libs/StdPeriphDriver/src/ch32f20x_spi.c\
      lib/ch32f20x_libs/StdPeriphDriver/src/ch32f20x_tim.c\
      lib/ch32f20x_libs/StdPeriphDriver/src/ch32f20x_usart.c\
      lib/ch32f20x_libs/StdPeriphDriver/src/ch32f20x_wwdg.c\
      lib/printf/printf.c\
      ./FreeRTOS/Source/croutine.c\
      ./FreeRTOS/Source/event_groups.c\
      ./FreeRTOS/Source/list.c\
      ./FreeRTOS/Source/portable/GCC/ARM_CM3/port.c\
      ./FreeRTOS/Source/portable/MemMang/heap_4.c\
      ./FreeRTOS/Source/queue.c\
      ./FreeRTOS/Source/stream_buffer.c\
      ./FreeRTOS/Source/tasks.c\
      ./FreeRTOS/Source/timers.c\
      ./Main.c\
      ./ch32f20x_it.c\
      ./system_ch32f20x.c\

#  C++ source files
CPPFILES = $(filter %.cpp, $(SRC))
#  C source files
CFILES = $(filter %.c, $(SRC))
#  Assembly source files
ASMFILES = $(filter %.s, $(SRC))

# Object files
CPPOBJ = $(CPPFILES:.cpp=.o)
COBJ = $(CFILES:.c=.o)
SOBJ = $(ASMFILES:.s=.o)
OBJ  = $(CPPOBJ) $(COBJ) $(SOBJ)

# Set target MCU
#  - CH32F20x_D6 for CH32F203K8-CH32F203C6-CH32F203C8
#  - CH32F20x_D8 for CH32F203CB-CH32F203RB-CH32F203RC-CH32F203VC
#  - CH32F20x_D8C for CH32F207x-CH32F205x
#  - CH32F20x_D8W for CH32F208x
TARGET = CH32F20x_D8W

# C/C++ optional flags
COPTFLAGS = -DDEBUG=4

# Compile thumb for cortex-m3 with debug info
CPPFLAGS  = -g -mthumb -D$(TARGET)=1 $(COPTFLAGS) -mcpu=cortex-m3 -mfloat-abi=soft -Og -fpack-struct -fdata-sections -ffunction-sections -fno-exceptions -fno-rtti -std=c++11
CFLAGS  = -g -mthumb -D$(TARGET)=1 $(COPTFLAGS) -mcpu=cortex-m3 -mfloat-abi=soft -Og -fpack-struct -fdata-sections -ffunction-sections -std=c99
ASFLAGS = -g -mthumb -mcpu=cortex-m3


all: $(SRC) $(PROJECT).elf $(PROJECT).hex $(PROJECT).bin

$(PROJECT).bin: $(PROJECT).elf
	@$(CP) -O binary $(PROJECT).elf $@

$(PROJECT).hex: $(PROJECT).elf
	@$(CP) -O ihex $(PROJECT).elf $@

$(PROJECT).elf: $(OBJ)
	@echo Linking
	@$(LD) $(LDFLAGS) $(OBJ) -o $@
	@$(OS) -t $(PROJECT).elf


$(CPPOBJ): %.o: %.cpp
	@echo $<
	@$(CPP) -c $(INC) $(CPPFLAGS) $< -o $@

$(COBJ): %.o: %.c
	@echo $<
	@$(CC) -c $(INC) $(CFLAGS) $< -o $@

$(SOBJ): %.o: %.s
	@echo $<
	@$(AS) -c $(ASFLAGS) $< -o $@

clean:
	@rm -f $(PROJECT).elf $(PROJECT).bin $(PROJECT).map $(PROJECT).hex $(PROJECT).lss $(OBJ)
