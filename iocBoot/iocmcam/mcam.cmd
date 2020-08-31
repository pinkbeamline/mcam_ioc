#!../../bin/linux-arm/mcam

## You may have to change mcam to something else
## everywhere it appears in this file

< envPaths

epicsEnvSet("EPICS_CA_MAX_ARRAY_BYTES", "1000000")

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/mcam.dbd"
mcam_registerRecordDeviceDriver pdbbase

## Load record instances
dbLoadRecords("db/mcam.db","BL=PINK,DEV=MCAM")

cd "${TOP}/iocBoot/${IOC}"

set_savefile_path("${TOP}/iocBoot/${IOC}/autosave")
set_pass0_restoreFile("auto_settings.sav")

iocInit

create_monitor_set("auto_settings.req", 30, "BL=PINK,DEV=MCAM")

## Start any sequence programs
#seq sncxxx,"user=pi"
