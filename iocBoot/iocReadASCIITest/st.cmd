#!../../bin/windows-x64/ReadASCIITest

## You may have to change ReadASCIITest to something else
## everywhere it appears in this file

# Increase this if you get <<TRUNCATED>> or discarded messages warnings in your errlog output
errlogInit2(65536, 256)

< envPaths

epicsEnvSet "RAMP_DIR" "$(TOP)/example_settings"

cd ${TOP}

## Register all support components
dbLoadDatabase("dbd/ReadASCIITest.dbd")
ReadASCIITest_registerRecordDeviceDriver pdbbase

##ISIS## Run IOC initialisation 
< $(IOCSTARTUP)/init.cmd

## Load record instances

##ISIS## Load common DB records 
##< $(IOCSTARTUP)/dbload.cmd

## Load our record instances
#dbLoadRecords("db/xxx.db","user=ffv81422Host")
ReadASCIIConfigure("testREAD", "$(RAMP_DIR)", 1, 20)
dbLoadRecords("$(TOP)/db/ReadASCII.db","P=$(MYPVPREFIX)$(IOCNAME):,READ=testREAD,ADDR=0,TIMEOUT=1, RDIR=$(RAMP_DIR)")

##ISIS## Stuff that needs to be done after all records are loaded but before iocInit is called 
##< $(IOCSTARTUP)/preiocinit.cmd

cd ${TOP}/iocBoot/${IOC}
iocInit

## Start any sequence programs
#seq sncxxx,"user=ffv81422Host"

##ISIS## Stuff that needs to be done after iocInit is called e.g. sequence programs 
#< $(IOCSTARTUP)/postiocinit.cmd
