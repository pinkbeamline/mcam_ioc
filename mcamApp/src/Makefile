TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = mcam
# mcam.dbd will be created and installed
DBD += mcam.dbd

# mcam.dbd will be made up from these files:
mcam_DBD += base.dbd

# Include dbd files from all support applications:
mcam_DBD += asSupport.dbd

# Add all the support libraries needed by this IOC
mcam_LIBS += autosave

# mcam_registerRecordDeviceDriver.cpp derives from mcam.dbd
mcam_SRCS += mcam_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
mcam_SRCS_DEFAULT += mcamMain.cpp
mcam_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#mcam_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
mcam_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

