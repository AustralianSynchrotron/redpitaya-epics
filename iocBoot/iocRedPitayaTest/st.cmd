#!../../bin/linux-arm/RedPitayaTest

## You may have to change RedPitayaTest to something else
## everywhere it appears in this file

< envPaths

cd ${TOP}

## Register all support components
dbLoadDatabase("dbd/RedPitayaTest.dbd",0,0)
RedPitayaTest_registerRecordDeviceDriver(pdbbase) 

RedPitaya_Initialise (4)

# RedPitaya_Configure (portName, triggerPollingInterval in seconds)
RedPitaya_Configure ("RP", 0.1)

## Load record instances
dbLoadTemplate("db/redpitaya.substitutions")

cd ${TOP}/iocBoot/${IOC}

iocInit()

# end
