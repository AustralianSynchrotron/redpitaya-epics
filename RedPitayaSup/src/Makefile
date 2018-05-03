TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

LIBRARY_IOC = drvRedPitaya
# RedPitaya.dbd will be created and installed
DBD += drvRedPitaya.dbd

SCRIPTS += load_fpga_image.sh

drvRedPitaya_SRCS += drvRedPitaya.cpp

# Add all the support libraries needed by this IOC
drvRedPitaya_LIBS += asyn rp
drvRedPitaya_LIBS += $(EPICS_BASE_IOC_LIBS)

rp_DIR = /opt/redpitaya/lib

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

