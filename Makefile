ARCHTUPLE=arm-none-eabi-
DEVICE=VEX EDR V5

# Flags/Compiler Parameters
MFLAGS=-mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=hard -g -Os # use -Og for debugging
CPPFLAGS=-D_POSIX_THREADS -D_UNIX98_THREAD_MUTEX_ATTRIBUTES -D_POSIX_TIMERS -D_POSIX_MONOTONIC_CLOCK $(INC_FLAGS) -MMD -MP -DFMT_STATIC_THOUSANDS_SEPERATOR=',' -DFMT_USE_LONG_DOUBLE=0 -DFMT_USE_FLOAT128=0 -DFMT_USE_FLOAT=0 -DFMT_USE_USER_DEFINED_LITERALS=0 -DFMT_USE_FULL_CACHE_DRAGONBOX=0 -DFREERTOS
GCCFLAGS=-ffunction-sections -fdata-sections -fdiagnostics-color -funwind-tables #-fanalyzer

# Check if the llemu files in libvgl exist. If they do, define macros that the
# llemu headers in the kernel repo can use to conditionally include the libvgl
# versions
ifneq (,$(wildcard ./include/liblvgl/llemu.h))
	CPPFLAGS += -D_PROS_INCLUDE_LIBLVGL_LLEMU_H
endif
ifneq (,$(wildcard ./include/liblvgl/llemu.hpp))
	CPPFLAGS += -D_PROS_INCLUDE_LIBLVGL_LLEMU_HPP
endif

#WARNFLAGS=-Wall -Wno-psabi -fno-elide-type -fdiagnostics-show-template-tree -Wpedantic -Wvla -Wextra-semi -Wnull-dereference -Wswitch-enum -fvar-tracking-assignments -Wduplicated-cond -Wduplicated-branches -Wsuggest-override
WARNFLAGS=-Wall -Wno-psabi -fno-elide-type -fdiagnostics-show-template-tree -Wpedantic -Wvla -Wnull-dereference -fvar-tracking-assignments -Wduplicated-cond -Wduplicated-branches # no suggest-override, no switch-enum, no extra-semis
SPACE := $() $()
COMMA := ,

ASMFLAGS=$(MFLAGS) $(WARNFLAGS)
CFLAGS=$(MFLAGS) $(CPPFLAGS) $(WARNFLAGS) $(GCCFLAGS) --std=gnu2x
CXXFLAGS=$(MFLAGS) $(CPPFLAGS) $(WARNFLAGS) $(GCCFLAGS) --std=gnu++23
LDFLAGS=$(MFLAGS) $(WARNFLAGS) -nostdlib $(GCCFLAGS)
SIZEFLAGS=-d --common
NUMFMTFLAGS=--to=iec --format %.2f --suffix=B
DEPFLAGS=-MT $@ -MMD -MP -MF $(DEPDIR)/$*.d

LIBRARIES+=$(wildcard $(FWDIR)/*.a)
# Cannot include newlib and libc because not all of the req'd stubs are implemented
EXCLUDE_COLD_LIBRARIES+=$(FWDIR)/libc.a $(FWDIR)/libm.a
COLD_LIBRARIES=$(filter-out $(EXCLUDE_COLD_LIBRARIES), $(LIBRARIES))
wlprefix=-Wl,$(subst $(SPACE),$(COMMA),$1)
LNK_FLAGS=--gc-sections --start-group $(strip $(LIBRARIES)) -lgcc -lstdc++ --end-group -T$(FWDIR)/v5-common.ld --no-warn-rwx-segments

# Directories
ROOT=.
FWDIR:=$(ROOT)/firmware
BINDIR=$(ROOT)/bin
SRCDIR=$(ROOT)/src
DEPDIR=$(ROOT)/.d
INCDIR=$(ROOT)/include $(SRCDIR) $(ROOT)/extern
INC_FLAGS:= $(addprefix -I,$(INCDIR))

# Functions
rwildcard=$(wildcard $1$2)$(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2))

# cross platform mechanics
# on windows, make uses cmd.exe
ifeq ($(shell echo "check_quotes"),"check_quotes")
	WINDOWS := yes
else
	WINDOWS := no
endif

ifeq ($(WINDOWS),yes)
#getCWD = $(subst \\,/,%cd:~-23%)
	getCWD = $(subst \\,/,%cd:\=/%)
	mkdir = mkdir $(subst /,\,$(1)) >nul 2>&1 || (exit 0)
	rm = $(wordlist 2,65535,$(foreach FILE,$(subst /,\,$(1)),& del /Q $(FILE) >nul 2>&1)) || (exit 0)
	rmdir = rmdir /S /Q $(subst /,\,$(1)) >nul 2>&1 || (exit 0)
	cp = copy $(subst /,\,$(1)) $(subst /,\,$(2)) >nul 2>&1 || exit(0)
	mv = move /Y $(subst /,\,$(1)) $(subst /,\,$(2)) >nul 2>&1 || exit(0)
	echo = echo $(1)
	ECHO_QUOTES = $()
	SED = powershell -Command "Get-Content (Get-ChildItem -Path ./bin/src/*.cpp.o.json -Recurse -Force) -Raw | ConvertFrom-Json | ConvertTo-Json" > compile_commands.json
	END_JSON = $()
	UNIX_TIMESTAMP = powershell -Command "[int](Get-Date -UFormat %s -Millisecond 0)
	UNIX_TIMEZONE = powershell -Command "[int](Get-Date -UFormat %Z -Millisecond 0) * 100"
else
#getCWD = $(shell pwd | tail -c 23)
	getCWD = $(shell pwd)
	mkdir = mkdir -p $(1)
	rm = rm -f $(1) > /dev/null 2>&1 || true
	rmdir = rm -rf $(1) > /dev/null 2>&1 || true
	cp = cp $(1) $(2)
	mv = mv -f $(1) $(2)
	echo = echo "$(subst ",\",$(1))"
	ECHO_QUOTES = '
# GNU Sed
#SED = sed -e '1s/^/[\n/' -e '$s/,$/\n]/' $(BINDIR)/*.o.json > compile_commands.json
# Bash/Zsh/Ksh Sed
# SED = sed -e '1s/^/[\'$'\n''/' -e '$s/,$/\'$'\n'']/' ./bin/src/*/**.o.json > compile_commands.json
	SED = ./makeCompileDB.sh
	END_JSON = ,
	UNIX_TIMESTAMP = date +%s
	UNIX_TIMEZONE = date +%-z
endif

MAKEDEPFOLDER=$(call mkdir, $(DEPDIR)/$(dir $(patsubst $(BINDIR)/%, %, $(ROOT)/$@)))
RENAMEDEPENDENCYFILE=$(call mv, $(DEPDIR)/$*.d, $(patsubst $(SRCDIR)/%, $(DEPDIR)/%.d, $(ROOT)/$<))

AR:=$(ARCHTUPLE)ar
# using arm-none-eabi-as generates a listing by default. This produces a super verbose output.
# Using gcc accomplishes the same thing without the extra output
AS:=$(ARCHTUPLE)gcc
CC:=$(ARCHTUPLE)gcc
CXX:=$(ARCHTUPLE)g++
LD:=$(ARCHTUPLE)g++
OBJCOPY:=$(ARCHTUPLE)objcopy
SIZETOOL:=$(ARCHTUPLE)size
READELF:=$(ARCHTUPLE)readelf
STRIP:=$(ARCHTUPLE)strip

SRCS:=$(call rwildcard,$(SRCDIR),*.cpp) $(call rwildcard,$(SRCDIR),*.c) $(call rwildcard,$(SRCDIR),*.s)
OBJS:=$(SRCS:%=$(BINDIR)/%.o)
#DEPS:=$(SRCS:%=$(DEPDIR)/%.d)
DEPS:=$(patsubst $(SRCDIR)/%,$(DEPDIR)/%.d,$(SRCS))

ARCHIVE_TEXT_LIST=$(subst $(SPACE),$(COMMA),$(notdir $(basename $(LIBRARIES))))
LDTIMEOBJ:=$(BINDIR)/_pros_ld_timestamp.o

MONOLITH_BIN:=$(BINDIR)/monolith.bin
MONOLITH_ELF:=$(basename $(MONOLITH_BIN)).elf

HOT_BIN:=$(BINDIR)/hot.package.bin
HOT_ELF:=$(basename $(HOT_BIN)).elf
COLD_BIN:=$(BINDIR)/cold.package.bin
COLD_ELF:=$(basename $(COLD_BIN)).elf

DEFAULT_BIN=$(HOT_BIN)
INCLUDE:=$(foreach dir,$(INCDIR),-iquote"$(dir)")
ESCAPED_INCLUDE:=$(foreach dir,$(INCDIR),-iquote\"$(dir)\")


# Compile Commands Database
COMPDB_ENTRIES = $(OBJS:%=%.json)
COMPDB_ENTRY = $@.json
ABSPATH:=$(abspath $(dir $(ROOT)))


.DEFAULT_GOAL=quick

.PHONY: all clean quick

-include $(wildcard $(FWDIR)/*.mk)

quick: $(DEFAULT_BIN)

monolith: $(MONOLITH_BIN)

all: clean $(DEFAULT_BIN)

clean:
	@echo Cleaning project
	@$(call rmdir,$(BINDIR)/)
	@$(call rmdir,$(DEPDIR)/)
	@$(call rm,$(ROOT)/compile_commands.json)

makeDirectory: $(DEPDIR)
	@$(call mkdir,$(DEPDIR))

$(MONOLITH_BIN): $(MONOLITH_ELF) $(BINDIR)
	@echo Creating monolith elf for $(DEVICE)
	@$(OBJCOPY) $< -O binary -R .hot_init $@

$(MONOLITH_ELF): $(OBJS) $(LIBRARIES)
	@$(call createCompileCommands)
	@$(call _pros_ld_timestamp)
	@echo Linking project with $(ARCHIVE_TEXT_LIST)
	@$(LD) $(LDFLAGS) $(LDFLAGS) $(OBJS) $(LDTIMEOBJ) $(call wlprefix,-T$(FWDIR)/v5.ld $(LNK_FLAGS) -o $@)
	@echo Section sizes:
	@$(SIZETOOL) $(SIZEFLAGS) $@ $(SIZES_NUMFMT)

$(COLD_BIN): $(COLD_ELF)
	@echo Creating cold package binary for $(DEVICE)
	@$(OBJCOPY) $< -O binary -R .hot_init $@


$(COLD_ELF): $(COLD_LIBRARIES)
	@$(call mkdir,$(dir $@))
	@echo Creating cold package with $(ARCHIVE_TEXT_LIST)
	@$(LD) $(LDFLAGS) $(call wlprefix,--gc-keep-exported --whole-archive $^ -lstdc++ --no-whole-archive) $(call wlprefix,-T$(FWDIR)/v5.ld $(LNK_FLAGS) -o $@)
	@echo Stripping cold package
	@$(OBJCOPY) --strip-symbol=install_hot_table --strip-symbol=__libc_init_array --strip-symbol=_PROS_COMPILE_DIRECTORY --strip-symbol=_PROS_COMPILE_TIMESTAMP --strip-symbol=_PROS_COMPILE_TIMESTAMP_INT $@ $@
	@echo Section sizes:
	@$(SIZETOOL) $(SIZEFLAGS) $@ $(SIZES_NUMFMT)

$(HOT_BIN): $(HOT_ELF) $(COLD_BIN)
	@echo Creating $@ for $(DEVICE)
	@$(OBJCOPY) $< -O binary $@

$(HOT_ELF): $(COLD_ELF) $(OBJS)
	@$(call createCompileCommands)
	@$(call _pros_ld_timestamp)
	@echo Linking hot project with $(COLD_ELF) and $(ARCHIVE_TEXT_LIST)
	@$(LD) -nostartfiles $(LDFLAGS) $(call wlprefix,-R $<) $(filter-out $<,$^) $(LDTIMEOBJ) $(LIBRARIES) $(call wlprefix,-T$(FWDIR)/v5-hot.ld $(LNK_FLAGS) -o $@)

$(BINDIR)/%.s.o: %.s
	@$(call mkdir,$(dir $@))
	@$(call rm,$(COMPDB_ENTRY))
	@echo     {>> $(COMPDB_ENTRY)
	@echo         "directory": "$(ABSPATH)",>> $(COMPDB_ENTRY)
	@echo         "command": "$(AS) -c $(ASMFLAGS) -o $@ $<",>> $(COMPDB_ENTRY)
	@echo         "file": "$<">>$(COMPDB_ENTRY)
	@echo     }$(END_JSON)>>$(COMPDB_ENTRY)
	@echo Compiling $<
	@$(AS) -c $(ASMFLAGS) -o $@ $<

$(BINDIR)/%.c.o: %.c $(DEPDIR)/%.c.d
	@$(call mkdir,$(dir $@))
	@$(call rm,$(COMPDB_ENTRY))
	@echo $(ECHO_QUOTES)    {$(ECHO_QUOTES)>> $(COMPDB_ENTRY)
	@echo $(ECHO_QUOTES)        "directory": "$(ABSPATH)",$(ECHO_QUOTES)>> $(COMPDB_ENTRY)
	@echo $(ECHO_QUOTES)        "command": "$(CC) -c $(ESCAPED_INCLUDE) -iquote\"$(INCDIR)/$(dir $*)\" $(CFLAGS) $(DEPFLAGS) -o $@ $<",$(ECHO_QUOTES)>> $(COMPDB_ENTRY)
	@echo $(ECHO_QUOTES)        "file": "$<"$(ECHO_QUOTES)>>$(COMPDB_ENTRY)
	@echo $(ECHO_QUOTES)    }$(END_JSON)$(ECHO_QUOTES)>>$(COMPDB_ENTRY)
	@$(MAKEDEPFOLDER)
	@echo Compiling $<
	@$(CC) -c $(INCLUDE) -iquote"$(INCDIR)/$(dir $*)" $(CFLAGS) $(DEPFLAGS) -o $@ $<
	@$(RENAMEDEPENDENCYFILE)

$(BINDIR)/%.cpp.o: %.cpp $(DEPDIR)/%.cpp.d
	@$(call mkdir,$(dir $@))
	@$(call rm,$(COMPDB_ENTRY))
	@echo $(ECHO_QUOTES)    {$(ECHO_QUOTES)>> $(COMPDB_ENTRY)
	@echo $(ECHO_QUOTES)        "directory": "$(ABSPATH)",$(ECHO_QUOTES)>> $(COMPDB_ENTRY)
	@echo $(ECHO_QUOTES)        "command": "$(CXX) -c $(ESCAPED_INCLUDE) -iquote\"$(INCDIR)/$(dir $*)\" $(CXXFLAGS) $(DEPFLAGS) -o $@ $<",$(ECHO_QUOTES)>> $(COMPDB_ENTRY)
	@echo $(ECHO_QUOTES)        "file": "$<"$(ECHO_QUOTES)>>$(COMPDB_ENTRY)
	@echo $(ECHO_QUOTES)    }$(END_JSON)$(ECHO_QUOTES)>>$(COMPDB_ENTRY)
	@$(MAKEDEPFOLDER)
	@echo Compiling $<
	@$(CXX) -c $(INCLUDE) -iquote"$(INCDIR)/$(dir $*)" $(CXXFLAGS) $(DEPFLAGS) -o $@ $<
	@$(RENAMEDDEPENDENCYFILE)

define createCompileCommands
@echo Creating compile_commands.json
$(SED)
endef

# Pipe a line of code defining _PROS_COMPILE_TOOLSTAMP and _PROS_COMPILE_DIRECTORY into GCC,
# which allows compilation from stdin. We define _PROS_COMPILE_DIRECTORY using a command line-defined macro
# which is the pwd | tail bit, which will truncate the path to the last 23 characters
# 
# const int _PROS_COMPILE_TIMESTAMP_INT = $(( $(date +%s) - $(date +%z) * 3600 ))
# char const * const _PROS_COMPILE_TIEMSTAMP = __DATE__ " " __TIME__
# char const * const _PROS_COMPILE_DIRECTORY = "$(shell pwd | tail -c 23)";
#
# The shell command $$(($$(date +%s)+($$(date +%-z)/100*3600))) fetches the current
# unix timestamp, and then adds the UTC timezone offset to account for time zones.
define _pros_ld_timestamp
@$(call mkdir,$(dir $(LDTIMEOBJ)))
@echo Adding timestamp
@echo $(ECHO_QUOTES)const int _PROS_COMPILE_TIMESTAMP_INT = 0; char const * const _PROS_COMPILE_TIMESTAMP = __DATE__ " " __TIME__; char const * const _PROS_COMPILE_DIRECTORY = "$(wildcard $(getCWD))";$(ECHO_QUOTES) | $(CC) -c -x c $(CFLAGS) -o $(LDTIMEOBJ) -
endef

$(DEPDIR)/%.d: ;
.PRECIOUS: $(DEPDIR)/%.d
-include $(DEPS)
include $(wildcard $(patsubst $(SRCDIR)/%,$(DEPDIR)/%.d,$(CSRC) $(CXXSRC)))