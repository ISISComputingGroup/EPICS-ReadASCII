TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC += ReadASCII

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install ReadASCII.dbd into <top>/dbd
DBD += ReadASCII.dbd

# specify all source files to be compiled and added to the library
ReadASCII_SRCS += ReadASCII.cpp get_calib_metadata_impl.cpp get_calib_metadata.c

ReadASCII_LIBS += asyn utilities
ReadASCII_LIBS += asubFunctions
ReadASCII_LIBS += $(EPICS_BASE_IOC_LIBS)

# googletets runner
GTESTPROD_HOST += runner
runner_SRCS += ReadASCII_tests.cpp
runner_LIBS += ReadASCII $(ReadASCII_LIBS) pcrecpp pcre
GTESTS += runner

# fix PATH for when tests are run, need to use : on linux, so need to generalise
export PATH := $(ASYN)/bin/$(EPICS_HOST_ARCH);$(PATH)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

-include $(GTEST)/cfg/compat.RULES_BUILD
