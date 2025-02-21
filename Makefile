.DEFAULT_GOAL=quick

ROOT=.
FWDIR:=$(ROOT)/firmware
BINDIR=$(ROOT)/bin
SRCDIR=$(ROOT)/src
DEPDIR=$(ROOT)/.d
INCDIR=$(ROOT)/include $(SRCDIR) $(ROOT)/extern

# variables to use for the library that will be linked alongside cold package
LIBNAME=lib7405
ENABLE_LIB=1
# specify entire dirs that are lib src files
LIB_SRC_DIRS:=$(SRCDIR)/fmt $(SRCDIR)/lib $(SRCDIR)/subsystems
EXTRA_LIB_SRC_FILES:=$(SRCDIR)/RobotBase.cpp
# specify individual lib src files
#LIB_SRC_DIRS:=$(SRCDIR)/fmt $(SRCDIR)/lib
#EXTRA_LIB_SRC_FILES=$(SRCDIR)/RobotBase.cpp $(SRCDIR)/subsystems/Odometry.cpp $(SRCDIR)/subsystems/Drive.cpp $(SRCDIR)/subsystems/Controller.cpp $(SRCDIR)/subsystems/Pnooomatics.cpp
# files to include in hot package, even if they're in LIB_SRC_DIRS
EXCLUDE_LIB_SRC_FILES:=$(SRCDIR)/lib/trajectory/TrajectoryGen_hot.cpp $(SRCDIR)/subsystems/Intake.cpp

### Don't touch stuff below here
-include common.mk