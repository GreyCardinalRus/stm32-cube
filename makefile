#/*
#
#    1 tab == 4 spaces!
#
#*/


lc = $(subst A,a,$(subst B,b,$(subst C,c,$(subst D,d,$(subst E,e,$(subst F,f,$(subst G,g,$(subst H,h,$(subst I,i,$(subst J,j,$(subst K,k,$(subst L,l,$(subst M,m,$(subst N,n,$(subst O,o,$(subst P,p,$(subst Q,q,$(subst R,r,$(subst S,s,$(subst T,t,$(subst U,u,$(subst V,v,$(subst W,w,$(subst X,x,$(subst Y,y,$(subst Z,z,$1))))))))))))))))))))))))))

###########################################################
#  common part for all my cortex-m0-4 projects
###########################################################
#=============================================================================#
# toolchain configuration
#=============================================================================#

	BASE		= .
	CC			= $(TOOL)gcc
	CXX			= $(TOOL)g++
	LD			= $(TOOL)gcc
#	LD			= $(TOOL)g++
	AS			= $(CC) -x assembler-with-cpp
	OBJCOPY		= $(TOOL)objcopy
	OBJDUMP		= $(TOOL)objdump
	SIZE		= $(TOOL)size -d
	FLASHER		= openocd
	RM			= rm -f
	CP			= cp
	MD			= mkdir


#  dirs
	INCDIR		= $(BASE)/inc
	SRCDIR		= $(BASE)/src
	OBJDIR		= $(BASE)/obj
	EXEDIR		= $(BASE)/exe
	LSTDIR		= $(BASE)/lst
	PRJDIR		= $(BASE)/prj
	BAKDIR		= $(BASE)/bak

#files
	HEX		= $(EXEDIR)/$(TARGET).hex
	BIN		= $(EXEDIR)/$(TARGET).bin
	AXF		= $(EXEDIR)/$(TARGET).axf
	ELF		= $(EXEDIR)/$(TARGET).elf
	MAP		= $(LSTDIR)/$(TARGET).map
	LSS		= $(LSTDIR)/$(TARGET).lss
	OK		= $(EXEDIR)/$(TARGET).ok

#############  program name
	TARGET	= main

# program version
	VER_MAJOR	= 0
	VER_MINOR	= 1

	TOOL	= arm-none-eabi-
#	TOOL	= arm-kgp-eabi-

	OPTIMIZE	= -O0
	USE_LTO		= NO

	DISCOVERY_DIR=
	TARGET_CHIP = 
	BOARD = 
	DISCOVERY =
	DEFS	=
	STARTUP =	
	TEMPLATE_DIR = 
	BSP = 
	
	DEFS	+= -DCMSIS_OS -DUSE_HAL_DRIVER 
	DEFS	+= -DTEMPLATE -DDEMO 
	DEFS	+= -DDEBUG

	TEMPLATE_DIR = ./Templates.RTOS


	BSP = STM32F4-Discovery


#########################################################
ifneq (,$(findstring STM32F4-Discovery,$(BSP)))
	CHIP	  = STM32F407VG
	DISCOVERY = Discovery
	DEFS	  += -DUSE_DISCOVERY 

endif

# compile options 
#	MCU			= cortex-m3
# Select family 
# STM32F10X_LD    : STM32 Low density devices
# STM32F10X_LD_VL : STM32 Low density Value Line devices
# CHIP		= STM32F10X_MD    #: STM32 Medium density devices
# STM32F10X_MD_VL : STM32 Medium density Value Line devices
# STM32F10X_HD    : STM32 High density devices
# STM32F10X_HD_VL : STM32 XL-density devices
# STM32F10X_CL    : STM32 Connectivity line devices
# STM32F10X_XL    : STM32 XL-density devices
#	CHIP		= STM32F10X_MD

#	CHIP	= STM32F407VG
#	CHIP	= STM32F303VC
#	CHIP	= NRF51822
	
	
#	DEFS	+= -DUSE_DISCOVERY 
#	DEFS	+= -DUSE_NUCLEO
	


ifneq (,$(findstring STM32F407,$(CHIP)))
	DEFS	+= -D__stm32f4xx
	MCU 	= cortex-m4
	DEVICE	= STM32F407xx
	CHIP_XX	= STM32F4
endif

ifneq (,$(findstring STM32F303,$(CHIP)))
	DEFS	+= -D__stm32f3xc
	MCU 	= cortex-m3
	DEVICE	= STM32F303xc
	CHIP_XX	= STM32F3
endif

ifneq (,$(findstring NRF51,$(CHIP)))
	DEFS	+= -D__nrf51
	MCU 	= cortex-m0
	DEVICE = NRF51
	DEVICESERIES = nrf51
#	CHIP_XX	= STM32F3
	TARGET_CHIP := NRF51822_QFAA_CA
	BOARD := BOARD_NRF6310
endif

	DEVICE_LOWER = $(call lc,$(DEVICE))
	CHIP_LOWER = $(call lc,$(CHIP_XX))
	STARTUP = ./Drivers/CMSIS/Device/ST/$(CHIP_XX)xx/Source/Templates/gcc/startup_$(DEVICE_LOWER).s
#	STARTUP = $(TEMPLATE)/GCC/startup_$(DEVICE_LOWER).s
ifneq (,$(findstring USE_HAL_DRIVE,$(DEFS)))
	HAL_DIR	= ./Drivers/$(CHIP_XX)xx_HAL_Driver/Inc
endif
	CMSIS_DRIVER_DIR= ./Drivers/CMSIS/Device/ST/$(CHIP_XX)xx/Include
ifneq (,$(findstring USE_DISCOVERY,$(DEFS)))
	DISCOVERY_DIR= ./Drivers/BSP/$(BSP)
endif


	RTOS_ROOT=./Middlewares/Third_Party/FreeRTOS
	CMSIS_DIR=./Drivers/CMSIS/Include

#defines
	DEFS		+= -D$(CHIP) -D$(DEVICE)
	DEFS		+= -DVER_MAJOR=$(VER_MAJOR)
	DEFS		+= -DVER_MINOR=$(VER_MINOR)


# linker script (chip dependent)
	LD_SCRIPT	= ./Drivers/CMSIS/Device/ST/$(CHIP_XX)xx/Source/Templates/gcc/linker/$(CHIP)_FLASH.ld
#	LD_SCRIPT	= $(TEMPLATE_DIR)/GCC/$(CHIP)_FLASH.ld


# source directories (all *.c, *.cpp and *.s files included)
	DIRS	:= $(SRCDIR)
#	DIRS	+= $(TEMPLATE)/Src
ifneq (,$(findstring USE_HAL_DRIVE,$(DEFS)))
	DIRS	+= ./Drivers/$(CHIP_XX)xx_HAL_Driver/Src
endif
#ifneq (,$(findstring __discovery,$(DEFS)))
	DIRS	+=  ./Drivers/BSP/$(BSP)
#endif
	DIRS	+= $(RTOS_ROOT)/Source/CMSIS_RTOS
	DIRS	+= $(RTOS_ROOT)/Source
	DIRS	+= $(RTOS_ROOT)/Source/portable/GCC/ARM_CM3
#	DIRS	+= $(DISCOVERY_DIR)
		
	INCS	:= $(INCDIR)
	INCS	+= $(HAL_DIR)
	INCS	+= $(CMSIS_DRIVER_DIR)
	INCS	+= $(CMSIS_DIR)
	INCS	+= $(DISCOVERY_DIR)
	INCS	+= $(TEMPLATE_DIR)/Inc
	INCS	+= $(RTOS_ROOT)/Source/include
	INCS	+= $(RTOS_ROOT)/Source/CMSIS_RTOS

# includes
	INCS	:= $(patsubst %, -I "%", $(INCS))

# individual source files
	SRCS	:= 

#calc obj files list
	OBJS	:= $(SRCS)
	OBJS	+= $(wildcard $(addsuffix /*.cpp, $(DIRS)))
	OBJS	+= $(wildcard $(addsuffix /*.c, $(DIRS)))
	OBJS	+= $(wildcard $(addsuffix /*.S, $(DIRS)))
	
	OBJS	+= $(STARTUP)

	OBJS	+= $(RTOS_ROOT)/Source/portable/MemMang/heap_2.c	
	OBJS	+= $(TEMPLATE_DIR)/$(CHIP_LOWER)xx_it.c
	OBJS	+= $(TEMPLATE_DIR)/system_$(CHIP_LOWER)xx.c
	OBJS	+= $(TEMPLATE_DIR)/template_$(BSP).c
	OBJS	+= $(TEMPLATE_DIR)/freertos.c
#	OBJS	+= $(TEMPLATE)/template_main.c	
	OBJS	:= $(notdir $(OBJS))
	OBJS	:= $(OBJS:.cpp=.o)
	OBJS	:= $(OBJS:.c=.o)
	OBJS	:= $(OBJS:.S=.o)
	OBJS	:= $(OBJS:.s=.o)
	OBJS	:= $(patsubst %, $(OBJDIR)/%, $(OBJS))

#files to archive
	ARCFILES	= \
		$(SRCDIR) \
		$(PRJDIR) \
		$(SCMDIR) \
		$(BASE)/makefile \
		$(BASE)/.cproject \
		$(BASE)/.project

# flags
	FLAGS	= -mcpu=$(MCU) -mthumb 
	FLAGS	+= -ggdb -g3
	FLAGS	+= -MD -Wl, -nostartfiles -nostdlib -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char 
	FLAGS	+= -ffunction-sections -fdata-sections  
	FLAGS	+= $(DEFS) -Wl,--relax --specs=nano.specs 
	FLAGS	+= -Wl,-static -Wl,--gc-sections
#	FLAGS	+= -Wall -Werror
#	FLAGS	+= -mabi=aapcs
#-DUSE_STDPERIPH_DRIVER
	FLAGS	+= -Wa,-adhlns=$(addprefix $(LSTDIR)/, $(notdir $(addsuffix .lst, $(basename $<))))
	FLAGS	+= $(INCS)

	AFLAGS	= $(FLAGS)

	CFLAGS	= $(FLAGS)
	CFLAGS	+= $(OPTIMIZE)
	CFLAGS	+= -std=gnu99
	CFLAGS	+= -D GCC_ARMCM3
	CFLAGS	+= -ffunction-sections -fdata-sections
#	CFLAGS	+= -Wall -Wextra
#	CFLAGS	+= -Wimplicit -Wcast-align -Wpointer-arith -Wredundant-decls
#	CFLAGS	+= -Wshadow -Wcast-qual -Wcast-align -Wnested-externs -pedantic

	CXXFLAGS	= $(FLAGS)
	CXXFLAGS	+= $(OPTIMIZE)
	CXXFLAGS	+= -fno-exceptions -fno-rtti
	CXXFLAGS	+= -ffunction-sections -fdata-sections
	CXXFLAGS	+= -fno-threadsafe-statics
	CXXFLAGS	+= -funsigned-bitfields -fshort-enums
	CXXFLAGS	+= -Wall -Wextra
	CXXFLAGS	+= -Winline
	CXXFLAGS	+= -Wpointer-arith -Wredundant-decls
	CXXFLAGS	+= -Wshadow -Wcast-qual -Wcast-align -pedantic

	LD_FLAGS	= -mcpu=$(MCU)
	LD_FLAGS	+= -mthumb 
	LD_FLAGS	+= -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -g3 
	LD_FLAGS	+= -T$(LD_SCRIPT) -Xlinker 
	#	LD_FLAGS	+= -nostartfiles 
	LD_FLAGS	+= --gc-sections -lm 
	LD_FLAGS	+= -Wl,-Map,"$(MAP)"
	LD_FLAGS	+= --specs=rdimon.specs -lc -lrdimon
#,--cref


ifeq ($(USE_LTO),YES)
	CFLAGS		+= -flto
	CXXFLAGS	+= -flto
	LD_FLAGS	+= -flto $(OPTIMIZE)
endif

#openocd command-line

# debug level (d0..d3)
	oocd_params		= -d3
# interface and board/target settings (using the OOCD target-library here)
#	oocd_params		+= -c "fast enable"
#	oocd_params		+= -f interface/arm-usb-ocd.cfg 
	oocd_params		+= --f board/stm32f4discovery.cfg
	oocd_params		+= -c init -c targets
	oocd_params_program	= $(oocd_params)
# commands to prepare flash-write
	oocd_params_program	+= -c "halt"
# flash-write and -verify
	oocd_params_program	+= -c "flash write_image erase $(ELF)"
	oocd_params_program	+= -c "verify_image $(ELF)"
# reset target
	oocd_params_program	+= -c "reset run"
# terminate OOCD after programming
	oocd_params_program	+= -c shutdown

	oocd_params_reset	= $(oocd_params)
	oocd_params_reset	+= -c "reset run"
	oocd_params_reset	+= -c shutdown

.SILENT :

.PHONY: all start dirs build clean program reset archive

############# targets

all : start dirs $(ELF) $(HEX) $(BIN) $(LSS) $(OK) 
	-@$(RM) $(OBJDIR)/*.d 2>/dev/null
	-@$(RM) $(OBJDIR)/*.o 2>/dev/null
	
#all : start dirs $(AXF) $(ELF) $(HEX) $(BIN) $(LSS) $(OK)

# make object files dependent on Makefile
$(OBJS) : makefile
# make .elf file dependent on linker script
$(ELF) : $(LD_SCRIPT)

build: clean all

start:
	@echo --- building $(TARGET) $(OBJS)
	@echo --Dirs: $(DIRS)
	
$(LSS): $(ELF) makefile
	@echo --- making asm-lst...
#	@$(OBJDUMP) -dStC $(ELF) > $(LSS)
	@$(OBJDUMP) -dC $(ELF) > $(LSS)

$(OK): $(ELF)
	@echo ...elf...
	@$(SIZE) $(ELF) 
	#$(HEX)
	@echo "Errors: none"

$(AXF):	$(OBJS) $(STARTUP) makefile
	@echo --- linking... axf
	$(LD) $(OBJS) $(LIBS) $(LD_FLAGS) -o "$(AXF)"
#	$(LD) $(OBJS) $(STARTUP) $(LIBS) $(LD_FLAGS) -o "$(AXF)"
	
$(ELF):	$(OBJS) 
	@echo ---------------------------------------------------
	@echo --- linking... ELF
#try	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -T "/home/valentin/workspace4.4/STM32F4Discovery/System/STM32F407VG_FLASH.ld" -Xlinker --gc-sections -Wl,-Map,"STM32F4Discovery.map" -o "STM32F4Discovery.elf" $(OBJS) $(USER_OBJS) $(LIBS)
#curr   arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -g3 -T./src/../Templates.RTOS/GCC/STM32F407VG_FLASH.ld -Xlinker -Wl,-Map,./lst/main.map -Wl,--gc-sections -lm -o ./exe/main.elf ./obj/freertos.o ./obj/main.o ./obj/stm32f4xx_it.o ./obj/system_stm32f4xx.o ./obj/stm32f4xx_hal_i2c_ex.o ./obj/stm32f4xx_hal_usart.o ./obj/stm32f4xx_hal_dcmi.o ./obj/stm32f4xx_hal_pcd.o ./obj/stm32f4xx_ll_fmc.o ./obj/stm32f4xx_hal_irda.o ./obj/stm32f4xx_ll_fsmc.o ./obj/stm32f4xx_ll_sdmmc.o ./obj/stm32f4xx_hal_rcc_ex.o ./obj/stm32f4xx_hal_msp_template.o ./obj/stm32f4xx_hal_adc.o ./obj/stm32f4xx_hal.o ./obj/stm32f4xx_hal_cryp.o ./obj/stm32f4xx_hal_rng.o ./obj/stm32f4xx_hal_pwr.o ./obj/stm32f4xx_hal_ltdc.o ./obj/stm32f4xx_hal_hcd.o ./obj/stm32f4xx_hal_pcd_ex.o ./obj/stm32f4xx_hal_uart.o ./obj/stm32f4xx_hal_spi.o ./obj/stm32f4xx_hal_i2c.o ./obj/stm32f4xx_hal_gpio.o ./obj/stm32f4xx_hal_smartcard.o ./obj/stm32f4xx_hal_iwdg.o ./obj/stm32f4xx_hal_dma2d.o ./obj/stm32f4xx_hal_sai.o ./obj/stm32f4xx_hal_cryp_ex.o ./obj/stm32f4xx_hal_hash.o ./obj/stm32f4xx_hal_flash_ramfunc.o ./obj/stm32f4xx_hal_cortex.o ./obj/stm32f4xx_hal_sd.o ./obj/stm32f4xx_hal_tim_ex.o ./obj/stm32f4xx_hal_adc_ex.o ./obj/stm32f4xx_hal_i2s_ex.o ./obj/stm32f4xx_ll_usb.o ./obj/stm32f4xx_hal_dma.o ./obj/stm32f4xx_hal_pccard.o ./obj/stm32f4xx_hal_crc.o ./obj/stm32f4xx_hal_dac.o ./obj/stm32f4xx_hal_can.o ./obj/stm32f4xx_hal_nand.o ./obj/stm32f4xx_hal_flash_ex.o ./obj/stm32f4xx_hal_rtc.o ./obj/stm32f4xx_hal_nor.o ./obj/stm32f4xx_hal_hash_ex.o ./obj/stm32f4xx_hal_i2s.o ./obj/stm32f4xx_hal_eth.o ./obj/stm32f4xx_hal_tim.o ./obj/stm32f4xx_hal_dma_ex.o ./obj/stm32f4xx_hal_sram.o ./obj/stm32f4xx_hal_dac_ex.o ./obj/stm32f4xx_hal_rcc.o ./obj/stm32f4xx_hal_wwdg.o ./obj/stm32f4xx_hal_flash.o ./obj/stm32f4xx_hal_rtc_ex.o ./obj/stm32f4xx_hal_sdram.o ./obj/stm32f4xx_hal_pwr_ex.o ./obj/stm32f4_discovery.o ./obj/stm32f4_discovery_accelerometer.o ./obj/stm32f4_discovery_audio.o ./obj/cmsis_os.o ./obj/queue.o ./obj/croutine.o ./obj/list.o ./obj/tasks.o ./obj/timers.o ./obj/port.o ./obj/startup_stm32f407xx.o ./obj/heap_2.o

	@echo --- $(LD)  $(LIBS) $(LD_FLAGS) -o "$(ELF)" $(OBJS)
	$(LD)  $(LIBS) $(LD_FLAGS) -o "$(ELF)" $(OBJS)
#	$(CXX) $(LD_FLAGS_F) $(OBJS) $(LIBS) -o $@

$(HEX): $(ELF)
	@echo --- make hex...
	@$(OBJCOPY) -O ihex $(ELF) $(HEX)

$(BIN): $(ELF)
	@echo --- make binary...
	@$(OBJCOPY) -O binary $(ELF) $(BIN)

program: $(ELF)
	@echo "Programming with OPENOCD"
	$(FLASHER) $(oocd_params_program)

reset:
	@echo Resetting device
	$(FLASHER) $(oocd_params_reset)

VPATH := $(DIRS) $(RTOS_ROOT)/Source/portable/MemMang/ $(TEMPLATE_DIR)/GCC $(TEMPLATE_DIR)/Src

$(OBJDIR)/%.o: %.cpp makefile
	@echo --- compiling $<... 
	$(CXX) -c $(CXXFLAGS) -o $@ $< 

$(OBJDIR)/%.o: %.c makefile
	@echo --- compiling $<... 
#	@echo --- $(CC) -c $(CFLAGS) -o $@ $<
#good	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g3 -DUSE_HAL_DRIVER -DSTM32F40XX -DSTM32F407xx -I"/home/valentin/workspace4.4/STM32F4Discovery/Inc" -I"/home/valentin/workspace4.4/STM32F4Discovery/Drivers/STM32F4xx_HAL_Driver/Inc" -I"/home/valentin/workspace4.4/STM32F4Discovery/Drivers/CMSIS/Include" -I"/home/valentin/workspace4.4/STM32F4Discovery/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/valentin/workspace4.4/STM32F4Discovery/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/valentin/workspace4.4/STM32F4Discovery/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/valentin/workspace4.4/STM32F4Discovery/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/valentin/workspace4.4/STM32Cube/Drivers/BSP/STM32F4-Discovery" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
#cur 	arm-none-eabi-gcc -c -mcpu=cortex-m4 -mthumb -ggdb -g  -MD -Wl, -nostartfiles -nostdlib -D__discovery -DCMSIS_OS -DUSE_HAL_DRIVER -DTEMPLATE -D__stm32f4xx -DSTM32F407VG -DSTM32F407xx -DVER_MAJOR=0 -DVER_MINOR=1 -ffunction-sections -Wl,--relax --specs=nano.specs -Wl,-static -fdata-sections -Wl,--gc-sections -Wa,-adhlns=./lst/freertos.lst -O0 -std=gnu99 -D GCC_ARMCM3 -g -ffunction-sections -fdata-sections -o obj/freertos.o ./src/freertos.c

	$(CC) -c $(CFLAGS) -o $@ $<
	

$(OBJDIR)/%.o: %.S makefile
	@echo --- assembling $<...
	$(AS) -c $(AFLAGS) -o $@ $<

$(OBJDIR)/%.o: %.s makefile
	@echo --- assembling $<...
	$(AS) -c $(AFLAGS) -o $@ $<

$(OBJDIR)/%.o: %.asm makefile
	@echo --- assembling $<...
	$(AS) -c $(AFLAGS) -o $@ $<	

dirs: $(OBJDIR) $(EXEDIR) $(LSTDIR) $(BAKDIR)

$(OBJDIR):
	-@$(MD) $(OBJDIR)

$(EXEDIR):
	-@$(MD) $(EXEDIR)

$(LSTDIR):
	-@$(MD) $(LSTDIR)

$(BAKDIR):
	-@$(MD) $(BAKDIR)

clean:
	-@$(RM) $(OBJDIR)/*.d 2>/dev/null
	-@$(RM) $(OBJDIR)/*.o 2>/dev/null
	-@$(RM) $(LSTDIR)/*.lst 2>/dev/null
	-@$(RM) $(ELF)
	-@$(RM) $(HEX)
	-@$(RM) $(LSS)
	-@$(RM) $(MAP)

archive:
	@echo --- archiving...
	7z a $(BAKDIR)/$(TARGET)_`date +%Y-%m-%d,%H-%M-%S` $(ARCFILES)
	@echo --- done!

#-----------------------------------------------------------------------------#
# print the size of the objects and the .elf file
#-----------------------------------------------------------------------------#

print_size :
	@echo 'Size of modules:'
	$(SIZE) -B -t --common $(OBJS) $(USER_OBJS)
	@echo ' '
	@echo 'Size of target .elf file:'
	$(SIZE) -B $(ELF)
	@echo ' '

# dependencies
ifeq (,$(findstring build,$(MAKECMDGOALS)))
 ifeq (,$(findstring clean,$(MAKECMDGOALS)))
  ifeq (,$(findstring dirs,$(MAKECMDGOALS)))
  -include $(wildcard $(OBJDIR)/*.d) 
  endif
 endif
endif
