########################################################################
#
# Makefile for compiling Arduino sketches from command line
# System part (i.e. project independent)
# Copyright (C) 2013 Anuj Deshpande http://anujdeshpande.com
#
# Copyright (C) 2013 Parav http://github.com/meanbot, based on work by these 
# awesome people:
#
# Copyright (C) 2012 Sudar <http://sudarmuthu.com>, based on
# - M J Oldfield work: https://github.com/mjoldfield/Arduino-Makefile
#
# Copyright (C) 2010,2011,2012 Martin Oldfield <m@mjo.tc>, based on
# work that is copyright Nicholas Zambetti, David A. Mellis & Hernando
# Barragan.
#
# This file is free software; you can redistribute it and/or modify it
# under the terms of the GNU Lesser General Public License as
# published by the Free Software Foundation; either version 2.1 of the
# License, or (at your option) any later version.
#
# Adapted from Arduino 0011 Makefile by M J Oldfield
#
# Original Arduino adaptation by mellis, eighthave, oli.keller
#
# Current version: 0.12.0
#
# Refer to HISTORY.md file for complete history of changes
#
########################################################################
#
# PATHS YOU NEED TO SET UP
#
# We need to worry about three different sorts of file:
#
# 1. Things which are included in this distribution e.g. ard-parse-boards
#    => ARDMK_DIR
#
# 2. Things which are always in the Arduino distribution e.g.
#    boards.txt, libraries, &c.
#    => ARDUINO_DIR
#
# 3. Things which might be bundled with the Arduino distribution, but
#    might come from the system. Most of the toolchain is like this:
#    on Linux it's supplied by the system.
#    => AVR_TOOLS_DIR
#
# Having set these three variables, we can work out the rest assuming
# that things are canonically arranged beneath the directories defined
# above.
#
# On the Mac you might want to set:
#
#   ARDUINO_DIR   = /Applications/Arduino.app/Contents/Resources/Java
#   ARDMK_DIR     = /usr/local
#
# On Linux, you might prefer:
#
#   ARDUINO_DIR   = /usr/share/arduino
#   ARDMK_DIR     = /usr/local
#   AVR_TOOLS_DIR = /usr
#
# You can either set these up in the Makefile, or put them in your
# environment e.g. in your .bashrc
#
# If you don't specify these, we can try to guess, but that might not work
# or work the way you want it to.
#
# If you don't install the ard-... binaries to /usr/local/bin, but
# instead copy them to e.g. /home/mjo/arduino.mk/bin then set
#   ARDML_DIR = /home/mjo/arduino.mk
#
# If you'd rather not see the configuration output, define ARDUINO_QUIET.
#
########################################################################
#
# DEPENDENCIES
#
# The Perl programs need a couple of libraries:
#    YAML
#    Device::SerialPort
#
########################################################################
#
# STANDARD ARDUINO WORKFLOW
#
# Given a normal sketch directory, all you need to do is to create
# a small Makefile which defines a few things, and then includes this one.
#
# For example:
#
#       ARDUINO_LIBS = Ethernet SPI
#       BOARD_TAG    = uno
#       ARDUINO_PORT = /dev/cu.usb*
#
#       include /usr/local/share/Arduino.mk
#
# Hopefully these will be self-explanatory but in case they're not:
#
#    ARDUINO_LIBS - A list of any libraries used by the sketch (we
#                   assume these are in $(ARDUINO_DIR)/hardware/libraries
#                   or your sketchbook's libraries directory)
#
#    ARDUINO_PORT - The port where the Arduino can be found (only needed
#                   when uploading)
#
#    BOARD_TAG    - The ard-parse-boards tag for the board e.g. uno or mega
#                   'make show_boards' shows a list
#
# If you have your additional libraries relative to your source, rather
# than in your "sketchbook", also set USER_LIB_PATH, like this example:
#
#        USER_LIB_PATH := $(realpath ../../libraries)
#
# If you've added the Arduino-Makefile repository to your git repo as a
# submodule (or other similar arrangement), you might have lines like this
# in your Makefile:
#
#        ARDMK_DIR := $(realpath ../../tools/Arduino-Makefile)
#        include $(ARDMK_DIR)/arduino-mk/Arduino.mk
#
# In any case, once this file has been created the typical workflow is just
#
#   $ make upload
#
# All of the object files are created in the build-{BOARD_TAG} subdirectory
# All sources should be in the current directory and can include:
#  - at most one .pde or .ino file which will be treated as C++ after
#    the standard Arduino header and footer have been affixed.
#  - any number of .c, .cpp, .s and .h files
#
# Included libraries are built in the build-{BOARD_TAG}/libs subdirectory.
#
# Besides make upload you can also
#   make             - no upload
#   make clean       - remove all our dependencies
#   make depends     - update dependencies
#   make monitor     - connect to the Arduino's serial port
#   make disasm      - generate a .lss file in build-cli that contains
#                      disassembly of the compiled file interspersed
#                      with your original source code.
#
########################################################################
#
# SERIAL MONITOR
#
# The serial monitor just invokes the GNU screen program with suitable
# options. For more information see screen (1) and search for
# 'character special device'.
#
# The really useful thing to know is that ^A-k gets you out!
#
# The fairly useful thing to know is that you can bind another key to
# escape too, by creating $HOME{.screenrc} containing e.g.
#
#    bindkey ^C kill
#
# If you want to change the baudrate, just set MONITOR_BAUDRATE. If you
# don't set it, it tries to read from the sketch. If it couldn't read
# from the sketch, then it defaults to 9600 baud.
#
########################################################################
#
# ARDUINO WITH ISP
#
# You need to specify some details of your ISP programmer and might
# also need to specify the fuse values:
#
#     ISP_PROG	   = stk500v2
#     ISP_PORT     = /dev/ttyACM0
#
# You might also need to set the fuse bits, but typically they'll be
# read from boards.txt, based on the BOARD_TAG variable:
#
#     ISP_LOCK_FUSE_PRE  = 0x3f
#     ISP_LOCK_FUSE_POST = 0xcf
#     ISP_HIGH_FUSE      = 0xdf
#     ISP_LOW_FUSE       = 0xff
#     ISP_EXT_FUSE       = 0x01
#
# You can specify to also upload the EEPROM file:
#     ISP_EEPROM   = 1
#
# I think the fuses here are fine for uploading to the ATmega168
# without bootloader.
#
# To actually do this upload use the ispload target:
#
#    make ispload
#
#
########################################################################

# Useful functions
# Returns the first argument (typically a directory), if the file or directory
# named by concatenating the first and optionally second argument
# (directory and optional filename) exists
dir_if_exists = $(if $(wildcard $(1)$(2)),$(1))

# For message printing: pad the right side of the first argument with spaces to
# the number of bytes indicated by the second argument.
space_pad_to = $(shell echo $(1) "                                                      " | head -c$(2))

arduino_output =
# When output is not suppressed and we're in the top-level makefile,
# running for the first time (i.e., not after a restart after
# regenerating the dependency file), then output the configuration.
ifndef ARDUINO_QUIET
    ifeq ($(MAKE_RESTARTS),)
        ifeq ($(MAKELEVEL),0)
            arduino_output = $(info $(1))
        endif
    endif
endif

# Call with some text, and a prefix tag if desired (like [AUTODETECTED]),
show_config_info = $(call arduino_output,- $(call space_pad_to,$(2),20) $(1))

# Call with the name of the variable, a prefix tag if desired (like [AUTODETECTED]),
# and an explanation if desired (like (found in $$PATH)
show_config_variable = $(call show_config_info,$(1) = $($(1)) $(3),$(2))

# Just a nice simple visual separator
show_separator = $(call arduino_output,-------------------------)

$(call show_separator)
$(call arduino_output,Userspace.mk Configuration:)

ifndef ARDUINO_DIR
	# Two levels above our own	
    ARDUINO_DIR := $(realpath $(dir $(realpath $(lastword $(MAKEFILE_LIST))))/../..) 
    ARDUINO_DIR := $(strip $(ARDUINO_DIR))
    $(call show_config_variable,ARDUINO_DIR,[DEFAULT])
else
    $(call show_config_variable,ARDUINO_DIR,[USER])
endif


# To put more focus on warnings, be less verbose as default
# Use 'make V=1' to see the full commands

ifeq ("$(origin V)", "command line")
  BUILD_VERBOSE = $(V)
endif
ifndef BUILD_VERBOSE
  BUILD_VERBOSE = 0
endif


########################################################################
#
# Default TARGET to pwd (ex Daniele Vergini)
ifndef TARGET
    TARGET  = $(notdir $(CURDIR))
endif


########################################################################
# Arduino and system paths
#

USERSPACE_CORE_PATH = $(ARDUINO_DIR)/libarduino/cores/virtual
$(call show_config_variable,USERSPACE_CORE_PATH,[DEFAULT])

ifndef USERSPACE_VAR_PATH
    USERSPACE_VAR_PATH  = $(ARDUINO_DIR)/libarduino/variants/beaglebone
    $(call show_config_variable,USERSPACE_VAR_PATH,[COMPUTED],(from USERSPACE_CORE_PATH))
endif

ifndef BOARDS_TXT
    BOARDS_TXT  = $(ARDUINO_DIR)/libarduino/boards.txt
    $(call show_config_variable,BOARDS_TXT,[COMPUTED],(from USERSPACE_CORE_PATH))
endif

########################################################################
# boards.txt parsing
#
ifndef BOARD_TAG
    BOARD_TAG   = userspace
    $(call show_config_variable,BOARD_TAG,[DEFAULT])
else
    # Strip the board tag of any extra whitespace, since it was causing the makefile to fail
    # https://github.com/sudar/Arduino-Makefile/issues/57
    BOARD_TAG := $(strip $(BOARD_TAG))
    $(call show_config_variable,BOARD_TAG,[USER])
endif

# Everything gets built in here (include BOARD_TAG now)
ifndef OBJDIR
    OBJDIR = build-$(BOARD_TAG)
    $(call show_config_variable,OBJDIR,[COMPUTED],(from BOARD_TAG))
else
    $(call show_config_variable,OBJDIR,[USER])
endif
#CROSS COMPILE
ifndef CROSS_COMPILE
    $(call show_config_variable,CROSS_COMPILE,[DEFAULT])
else
    $(call show_config_variable,CROSS_COMPILE,[USER])
endif

#UPLOAD_UTILITY
ifndef UPLOAD_UTILITY
    UPLOAD_UTILITY = ${ARDUINO_DIR}/utility/send_exec.sh
    $(call show_config_variable,UPLOAD_UTILITY,[DEFAULT])
else
    $(call show_config_variable,UPLOAD_UTILITY,[USER])
endif

########################################################################
# Local sources
#
LOCAL_C_SRCS    ?= $(wildcard *.c)
LOCAL_CPP_SRCS  ?= $(wildcard *.cpp)
LOCAL_CC_SRCS   ?= $(wildcard *.cc)
LOCAL_PDE_SRCS  ?= $(wildcard *.pde)
LOCAL_INO_SRCS  ?= $(wildcard *.ino)
LOCAL_AS_SRCS   ?= $(wildcard *.S)
LOCAL_SRCS      = $(LOCAL_C_SRCS)   $(LOCAL_CPP_SRCS) \
		$(LOCAL_CC_SRCS)   $(LOCAL_PDE_SRCS) \
		$(LOCAL_INO_SRCS) $(LOCAL_AS_SRCS)
LOCAL_OBJ_FILES = $(LOCAL_C_SRCS:.c=.o)   $(LOCAL_CPP_SRCS:.cpp=.o) \
		$(LOCAL_CC_SRCS:.cc=.o)   $(LOCAL_PDE_SRCS:.pde=.o) \
		$(LOCAL_INO_SRCS:.ino=.o) $(LOCAL_AS_SRCS:.S=.o)
LOCAL_OBJS      = $(patsubst %,$(OBJDIR)/%,$(LOCAL_OBJ_FILES))

# Variant sources
#
VARIANT_C_SRCS  = $(wildcard $(USERSPACE_VAR_PATH)/*.c)
VARIANT_CPP_SRCS  = $(wildcard $(USERSPACE_VAR_PATH)/*.cpp)

VARIANT_OBJ_FILES  = $(VARIANT_C_SRCS:.c=.o) $(VARIANT_CPP_SRCS:.cpp=.o)
VARIANT_OBJS       = $(patsubst $(USERSPACE_VAR_PATH)/%,  \
                $(OBJDIR)/%,$(VARIANT_OBJ_FILES))


# If NO_CORE is not set, then we need exactly one .pde or .ino file
ifeq ($(strip $(NO_CORE)),)

    ifeq ($(words $(LOCAL_PDE_SRCS) $(LOCAL_INO_SRCS)), 0)
        ifeq ($(strip $(NO_CORE)),)
            $(error No .pde or .ino files found. If you want to compile .c or .cpp files, then set NO_CORE)
        endif
    endif

    # Ideally, this should just check if there are more than one file
    ifneq ($(words $(LOCAL_PDE_SRCS) $(LOCAL_INO_SRCS)), 1)
        #TODO: Support more than one file. https://github.com/sudar/Arduino-Makefile/issues/49
        $(error Need exactly one .pde or .ino file)
    endif

endif

# core sources
ifeq ($(strip $(NO_CORE)),)
    ifdef USERSPACE_CORE_PATH
        CORE_C_SRCS     = $(wildcard $(USERSPACE_CORE_PATH)/*.c)
        CORE_CPP_SRCS   = $(wildcard $(USERSPACE_CORE_PATH)/*.cpp)
        ifneq ($(strip $(NO_CORE_MAIN_CPP)),)
            CORE_CPP_SRCS := $(filter-out %main.cpp, $(CORE_CPP_SRCS))
            $(call show_config_info,NO_CORE_MAIN_CPP set so core library will not include main.cpp,[MANUAL])
        endif
        CORE_OBJ_FILES  = $(CORE_C_SRCS:.c=.o) $(CORE_CPP_SRCS:.cpp=.o)
        CORE_OBJS       = $(patsubst $(USERSPACE_CORE_PATH)/%,  \
                $(OBJDIR)/%,$(CORE_OBJ_FILES))
    endif
else
    $(call show_config_info,NO_CORE set so core library will not be built,[MANUAL])
endif

########################################################################
# Determine ARDUINO_LIBS automatically
ARDUINO_LIBS	+=  $(ARDUINO_DIR)/libarduino/libraries/SPI
ARDUINO_LIBS	+=  $(ARDUINO_DIR)/libarduino/libraries/Wire
ARDUINO_LIBS	+=  $(ARDUINO_DIR)/libarduino/libraries/Stepper
ARDUINO_LIBS	+=  $(ARDUINO_DIR)/libarduino/libraries/LiquidCrystal

ifndef ARDUINO_LIBS
    # automatically determine included libraries
    ARDUINO_LIBS += $(filter $(notdir $(wildcard $(ARDUINO_DIR)/libarduino/libraries/*)), \
       $(shell sed -ne "s/^ *\# *include *[<\"]\(.*\)\.h[>\"]/\1/p" $(LOCAL_SRCS)))
    ARDUINO_LIBS += $(filter $(notdir $(wildcard $(ARDUINO_SKETCHBOOK)/libarduino/libraries/*)), \
        $(shell sed -ne "s/^ *\# *include *[<\"]\(.*\)\.h[>\"]/\1/p" $(LOCAL_SRCS)))
endif

########################################################################
# Include file to use for old .pde files
#
ifndef ARDUINO_HEADER
        ARDUINO_HEADER=linux-virtual.h
endif

########################################################################
# Rules for making stuff
#

# The name of the main targets
TARGET_ELF = $(OBJDIR)/$(TARGET).elf
TARGETS    = $(OBJDIR)/$(TARGET).*
CORE_LIB   = $(OBJDIR)/libcore.a

# Names of executables
CC      = $(CROSS_COMPILE)gcc
CXX     = $(CROSS_COMPILE)g++
AS      = $(CROSS_COMPILE)as
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump
AR      = $(CROSS_COMPILE)ar
SIZE    = $(CROSS_COMPILE)size
NM      = $(CROSS_COMPILE)nm


REMOVE  = rm -rf
MV      = mv -f
CAT     = cat
ECHO    = echo
MKDIR   = mkdir -p

# General arguments
USER_LIBS     = $(wildcard $(patsubst %,$(USER_LIB_PATH)/%,$(ARDUINO_LIBS)))
USER_LIB_NAMES= $(patsubst $(USER_LIB_PATH)/%,%,$(USER_LIBS))

# Let user libraries override system ones.
SYS_LIBS      = $(wildcard $(patsubst %,$(ARDUINO_LIB_PATH)/%,$(filter-out $(USER_LIB_NAMES),$(ARDUINO_LIBS))))
SYS_LIB_NAMES = $(patsubst $(ARDUINO_LIB_PATH)/%,%,$(SYS_LIBS))

# Error here if any are missing.
LIBS_NOT_FOUND = $(filter-out $(USER_LIB_NAMES) $(SYS_LIB_NAMES),$(ARDUINO_LIBS))
ifneq (,$(strip $(LIBS_NOT_FOUND)))
    $(error The following libraries specified in ARDUINO_LIBS could not be found (searched USER_LIB_PATH and ARDUINO_LIB_PATH): $(LIBS_NOT_FOUND))
endif

SYS_LIBS     := $(wildcard $(SYS_LIBS) $(addsuffix /utility,$(SYS_LIBS)))
USER_LIBS    := $(wildcard $(USER_LIBS) $(addsuffix /utility,$(USER_LIBS)))
SYS_INCLUDES  = $(patsubst %,-I%,$(SYS_LIBS))
USER_INCLUDES = $(patsubst %,-I%,$(USER_LIBS))
LIB_C_SRCS    = $(wildcard $(patsubst %,%/*.c,$(SYS_LIBS)))
LIB_CPP_SRCS  = $(wildcard $(patsubst %,%/*.cpp,$(SYS_LIBS)))
USER_LIB_CPP_SRCS   = $(wildcard $(patsubst %,%/*.cpp,$(USER_LIBS)))
USER_LIB_C_SRCS     = $(wildcard $(patsubst %,%/*.c,$(USER_LIBS)))
LIB_OBJS      = $(patsubst $(ARDUINO_LIB_PATH)/%.c,$(OBJDIR)/libs/%.o,$(LIB_C_SRCS)) \
        $(patsubst $(ARDUINO_LIB_PATH)/%.cpp,$(OBJDIR)/libs/%.o,$(LIB_CPP_SRCS))
USER_LIB_OBJS = $(patsubst $(USER_LIB_PATH)/%.cpp,$(OBJDIR)/libs/%.o,$(USER_LIB_CPP_SRCS)) \
        $(patsubst $(USER_LIB_PATH)/%.c,$(OBJDIR)/libs/%.o,$(USER_LIB_C_SRCS))

# Dependency files
DEPS            = $(LOCAL_OBJS:.o=.d) $(LIB_OBJS:.o=.d) $(USER_LIB_OBJS:.o=.d) $(CORE_OBJS:.o=.d) $(VARIANT_OBJS:.o=.d)

# Optimization level for the compiler.
# You can get the list of options at http://www.nongnu.org/avr-libc/user-manual/using_tools.html#gcc_optO
# Also read http://www.nongnu.org/avr-libc/user-manual/FAQ.html#faq_optflags
ifndef OPTIMIZATION_LEVEL
    OPTIMIZATION_LEVEL=s
    $(call show_config_variable,OPTIMIZATION_LEVEL,[DEFAULT])
else
    $(call show_config_variable,OPTIMIZATION_LEVEL,[USER])
endif

# Using += instead of =, so that CPPFLAGS can be set per sketch level
CPPFLAGS      += -I. -I$(USERSPACE_CORE_PATH) -I$(USERSPACE_VAR_PATH)/$(VARIANT) \
        $(SYS_INCLUDES) $(USER_INCLUDES) -g -Wall \
        -O$(OPTIMIZATION_LEVEL)
$(call show_config_variable,USERSPACE_CORE_PATH,[DEFAULT])

CFLAGS        += $(EXTRA_FLAGS) $(EXTRA_CFLAGS)
CXXFLAGS      += $(EXTRA_FLAGS) $(EXTRA_CXXFLAGS)
ASFLAGS       += -I. -x assembler-with-cpp
LDFLAGS       += -Wl,--gc-sections -lrt $(EXTRA_FLAGS) $(EXTRA_CXXFLAGS) -O$(OPTIMIZATION_LEVEL)
SIZEFLAGS     ?= -C

ifneq (,$(strip $(ARDUINO_LIBS)))
    $(call arduino_output,-)
    $(call show_config_info,ARDUINO_LIBS =)
endif

ifneq (,$(strip $(USER_LIB_NAMES)))
    $(foreach lib,$(USER_LIB_NAMES),$(call show_config_info,  $(lib),[USER]))
endif

ifneq (,$(strip $(SYS_LIB_NAMES)))
    $(foreach lib,$(SYS_LIB_NAMES),$(call show_config_info,  $(lib),[SYSTEM]))
endif

$(call show_config_variable,USERSPACE_VAR_PATH, [USER])

# end of config output
$(call show_separator)


ifeq ($(BUILD_VERBOSE),1)
  Q =
else
  Q = @
endif

# Implicit rules for building everything (needed to get everything in
# the right directory)
#
# Rather than mess around with VPATH there are quasi-duplicate rules
# here for building e.g. a system C++ file and a local C++
# file. Besides making things simpler now, this would also make it
# easy to change the build options in future

# library sources
$(OBJDIR)/libs/%.o: $(ARDUINO_LIB_PATH)/%.c
	@$(ECHO) Compiling $@
	$(MKDIR) $(dir $@)
	$(Q)$(CC) -MMD -c $(CPPFLAGS) $(CFLAGS) $< -o $@

$(OBJDIR)/libs/%.o: $(ARDUINO_LIB_PATH)/%.cpp
	@$(ECHO) Compiling $@
	$(MKDIR) $(dir $@)
	$(Q)$(CC) -MMD -c $(CPPFLAGS) $(CXXFLAGS) $< -o $@

$(OBJDIR)/libs/%.o: $(USER_LIB_PATH)/%.cpp
	@$(ECHO) Compiling $@
	$(MKDIR) $(dir $@)
	$(Q)$(CC) -MMD -c $(CPPFLAGS) $(CFLAGS) $< -o $@

$(OBJDIR)/libs/%.o: $(USER_LIB_PATH)/%.c
	@$(ECHO) Compiling $@
	$(MKDIR) $(dir $@)
	$(Q)$(CC) -MMD -c $(CPPFLAGS) $(CFLAGS) $< -o $@

ifdef COMMON_DEPS
    COMMON_DEPS := $(COMMON_DEPS) Makefile
else
    COMMON_DEPS := Makefile
endif


# normal local sources
$(OBJDIR)/%.o: %.c $(COMMON_DEPS) | $(OBJDIR)
	$(Q)$(CC) -MMD -c $(CPPFLAGS) $(CFLAGS) $< -o $@

$(OBJDIR)/%.o: %.cc $(COMMON_DEPS) | $(OBJDIR)
	$(Q)$(CXX) -MMD -c $(CPPFLAGS) $(CXXFLAGS) $< -o $@

$(OBJDIR)/%.o: %.cpp $(COMMON_DEPS) | $(OBJDIR)
	$(Q)$(CXX) -MMD -c $(CPPFLAGS) $(CXXFLAGS) $< -o $@

$(OBJDIR)/%.o: %.S $(COMMON_DEPS) | $(OBJDIR)
	$(Q)$(CC) -MMD -c $(CPPFLAGS) $(ASFLAGS) $< -o $@

$(OBJDIR)/%.o: %.s $(COMMON_DEPS) | $(OBJDIR)
	$(Q)$(CC) -c $(CPPFLAGS) $(ASFLAGS) $< -o $@

# the pde -> o file
$(OBJDIR)/%.o: %.pde $(COMMON_DEPS) | $(OBJDIR)
	@$(ECHO) Compiling $@
	$(Q)$(CXX) -x c++ -include $(ARDUINO_HEADER) -MMD -c $(CPPFLAGS) $(CXXFLAGS) $< -o $@

# the ino -> o file
$(OBJDIR)/%.o: %.ino $(COMMON_DEPS) | $(OBJDIR)
	@$(ECHO) Compiling $@
	$(Q)$(CXX) -x c++ -include linux-virtual.h -MMD -c $(CPPFLAGS) $(CXXFLAGS) $< -o $@

# generated assembly
$(OBJDIR)/%.s: %.pde $(COMMON_DEPS) | $(OBJDIR)
	@$(ECHO) Compiling $@
	$(Q)$(CXX) -x c++ -include $(ARDUINO_HEADER) -MMD -S -fverbose-asm $(CPPFLAGS) $(CXXFLAGS) $< -o $@

$(OBJDIR)/%.s: %.ino $(COMMON_DEPS) | $(OBJDIR)
	@$(ECHO) Compiling $@
	$(Q)$(CXX) -x c++ -include linux-virtual.h -MMD -S -fverbose-asm $(CPPFLAGS) $(CXXFLAGS) $< -o $@

# variants
$(OBJDIR)/%.o: $(USERSPACE_VAR_PATH)/%.c $(COMMON_DEPS) | $(OBJDIR)
	@$(ECHO) Compiling $@
	$(Q)$(CC) -MMD -c $(CPPFLAGS) $(CFLAGS) $< -o $@
$(OBJDIR)/%.o: $(USERSPACE_VAR_PATH)/%.cpp $(COMMON_DEPS) | $(OBJDIR)
	@$(ECHO) Compiling $@
	$(Q)$(CXX) -MMD -c $(CPPFLAGS) $(CXXFLAGS) $< -o $@

#$(OBJDIR)/%.lst: $(OBJDIR)/%.s
#	$(AS) -mmcu=$(MCU) -alhnd $< > $@

# core files
$(OBJDIR)/%.o: $(USERSPACE_CORE_PATH)/%.c $(COMMON_DEPS) | $(OBJDIR)
	@$(ECHO) Compiling $@
	$(Q)$(CC) -MMD -c $(CPPFLAGS) $(CFLAGS) $< -o $@

$(OBJDIR)/%.o: $(USERSPACE_CORE_PATH)/%.cpp $(COMMON_DEPS) | $(OBJDIR)
	@$(ECHO) Compiling $@
	$(Q)$(CXX) -MMD -c $(CPPFLAGS) $(CXXFLAGS) $< -o $@


# various object conversions
$(OBJDIR)/%.lss: $(OBJDIR)/%.elf $(COMMON_DEPS)
	$(OBJDUMP) -h --source --demangle --wide $< > $@

$(OBJDIR)/%.sym: $(OBJDIR)/%.elf $(COMMON_DEPS)
	$(NM) --size-sort --demangle --reverse-sort --line-numbers $< > $@


########################################################################
#
# Explicit targets start here
#

all: 		$(TARGET_ELF)

# Rule to create $(OBJDIR) automatically. All rules with recipes that
# create a file within it, but do not already depend on a file within it
# should depend on this rule. They should use a "order-only
# prerequisite" (e.g., put "| $(OBJDIR)" at the end of the prerequisite
# list) to prevent remaking the target when any file in the directory
# changes.
$(OBJDIR):
		$(MKDIR) $(OBJDIR)

$(TARGET_ELF): 	$(LOCAL_OBJS) $(CORE_LIB) $(OTHER_OBJS)
		@$(ECHO) Compiling $@
		$(Q)$(CXX) $(LDFLAGS) -o $@ $(LOCAL_OBJS) $(CORE_LIB) $(OTHER_OBJS)
		@$(ECHO) Build of $(TARGET) complete!
		@$(ECHO)

$(CORE_LIB):	$(CORE_OBJS) $(LIB_OBJS) $(USER_LIB_OBJS) $(VARIANT_OBJS)
		@$(ECHO) Creating $@
		$(Q)$(AR) rcs $@ $(CORE_OBJS) $(LIB_OBJS) $(USER_LIB_OBJS) $(VARIANT_OBJS)

# Use submake so we can guarantee the reset happens
# before the upload, even with make -j
upload:		$(TARGET_ELF)
		$(Q)$(MAKE) --no-print-directory do_upload

do_upload:
		$(Q)$(UPLOAD_UTILITY) $(TARGET)

clean:
		@$(ECHO) Removing all builds files in $(TARGET)/${OBJDIR}
		$(Q)$(REMOVE) $(LOCAL_OBJS) $(CORE_OBJS) $(LIB_OBJS) $(CORE_LIB) $(TARGETS) $(DEPS) $(USER_LIB_OBJS) ${OBJDIR}
		@$(ECHO)

monitor:
		$(MONITOR_CMD) $(call get_arduino_port) $(MONITOR_BAUDRATE)

disasm: $(OBJDIR)/$(TARGET).lss
	@$(ECHO) The compiled ELF file has been disassembled to $(OBJDIR)/$(TARGET).lss

symbol_sizes: $(OBJDIR)/$(TARGET).sym
	@$(ECHO) A symbol listing sorted by their size have been dumped to $(OBJDIR)/$(TARGET).sym

generate_assembly: $(OBJDIR)/$(TARGET).s
	@$(ECHO) Compiler-generated assembly for the main input source has been dumped to $(OBJDIR)/$(TARGET).s

generated_assembly: generate_assembly
	@$(ECHO) "generated_assembly" target is deprecated. Use "generate_assembly" target instead

.PHONY:	all upload clean depends monitor disasm symbol_sizes generated_assembly generate_assembly

# added - in the beginning, so that we don't get an error if the file is not present
-include $(DEPS)
