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
ReadASCII_SRCS += ReadASCII.cpp get_units_impl.cpp get_units.c

ReadASCII_LIBS += $(EPICS_BASE_IOC_LIBS)
ReadASCII_LIBS += asyn utilities
ReadASCII_LIBS += asubFunctions
ReadASCII_SYS_LIBS_WIN32 += ws2_32

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

