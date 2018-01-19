TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = RedPitayaTest
# RedPitayaTest.dbd will be created and installed
DBD += RedPitayaTest.dbd

# RedPitayaTest.dbd will be made up from these files:
RedPitayaTest_DBD += base.dbd
RedPitayaTest_DBD += asyn.dbd
RedPitayaTest_DBD += drvRedPitaya.dbd

# Add all the support libraries needed by this IOC
RedPitayaTest_LIBS += asyn
RedPitayaTest_LIBS += drvRedPitaya

# RedPitayaTest_registerRecordDeviceDriver.cpp derives from RedPitayaTest.dbd
RedPitayaTest_SRCS += RedPitayaTest_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
RedPitayaTest_SRCS_DEFAULT += RedPitayaTestMain.cpp
RedPitayaTest_SRCS_vxWorks += -nil-


# Add support from base/src/vxWorks if needed
#RedPitayaTest_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
RedPitayaTest_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

