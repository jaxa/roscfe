###############################################################################
# File: CFS Application Makefile 
#
# $Id: Makefile 1.6 2009/07/09 11:41:26EDT rmcgraw Exp  $
#
# $Log: Makefile  $
# Revision 1.6 2009/07/09 11:41:26EDT rmcgraw 
# DCR8291:1 Changed CFE_MISSION_INC to CFS_MISSION_INC and added log if needed
#
###############################################################################
#
# Subsystem produced by this makefile.
#
APPTARGET = convert_lib

# 
# Entry Point for task
# 
ENTRY_PT = CONVERT_LibInit 

#
# Object files required to build subsystem.
#
OBJS = convert_lib.o /usr/lib/i386-linux-gnu/libstdc++.so.6

#
# Source files required to build subsystem; used to generate dependencies.
# As long as there are no assembly files this can be automated.
#
SOURCES = $(OBJS:.o=.cpp)

##
## Specify extra C Flags needed to build this subsystem
##
LOCAL_COPTS = -D_LINUX_=1


##
## EXEDIR is defined here, just in case it needs to be different for a custom
## build
##
EXEDIR=../exe

##
## Certain OSs and Application Loaders require the following option for
## Shared libraries. Currently only needed for vxWorks 5.5 and RTEMS.
## For each shared library that this app depends on, you need to have an
## entry like the following:
##  -R../tst_lib/tst_lib.elf
##
SHARED_LIB_LINK = 

########################################################################
# Should not have to change below this line, except for customized 
# Mission and cFE directory structures
########################################################################

#
# Set build type to CFE_APP. This allows us to 
# define different compiler flags for the cFE Core and Apps.
# 
BUILD_TYPE = CFE_APP

## 
## Include all necessary cFE make rules
## Any of these can be copied to a local file and 
## changed if needed.
##
##
##       cfe-config.mak contains PSP and OS selection
##
include ../cfe/cfe-config.mak
##
##       debug-opts.mak contains debug switches
##
include ../cfe/debug-opts.mak
##
##       compiler-opts.mak contains compiler definitions and switches/defines
##
include $(CFE_PSP_SRC)/$(PSP)/make/compiler-opts.mak

##
## Setup the include path for this subsystem
## The OS specific includes are in the build-rules.make file
##
## If this subsystem needs include files from another app, add the path here.
##
INCLUDE_PATH = \
-I$(OSAL_SRC)/inc \
-I$(CFE_CORE_SRC)/inc \
-I$(CFE_PSP_SRC)/inc \
-I$(CFE_PSP_SRC)/$(PSP)/inc \
-I$(CFS_APP_SRC)/inc \
-I$(CFS_APP_SRC)/$(APPTARGET)/fsw/src \
-I$(CFS_APP_SRC)/$(APPTARGET)/fsw/public_inc \
-I$(CFS_MISSION_INC) \
-I../cfe/inc \
-I../inc

##
## Define the VPATH make variable. 
## This can be modified to include source from another directory.
## If there is no corresponding app in the cfs-apps directory, then this can be discarded, or
## if the mission chooses to put the src in another directory such as "src", then that can be 
## added here as well.
##
VPATH = $(CFS_APP_SRC)/$(APPTARGET)/fsw/src 

##
## Include the common make rules for building a cFE Application
##
# include $(CFE_CORE_SRC)/make/app-rules.mak
default: $(APPTARGET).o
.cpp.o:
	$(COMPILER) $(LOCAL_COPTS) -std=c++11 -m32 -Wall $(INCLUDE_PATH) -g -O0 -DOS_DEBUG_LEVEL=$(DEBUG_LEVEL) -c -o $@ $< 

default: $(APPTARGET).so
.s.o:
	gcc $(LOCAL_COPTS) $(INCLUDE_PATH) $(ASOPTS) $(COPTS) $(DEBUG_OPTS) -DOS_DEBUG_LEVEL=$(DEBUG_LEVEL) -c -o $@ $< 

depend: $(SOURCES)
	$(COMPILER) -MM $(LOCAL_COPTS) $(INCLUDE_PATH) $(COPTS_D) $(DEBUG_OPTS) -DOS_DEBUG_LEVEL=$(DEBUG_LEVEL) -c $^ > $(APPTARGET).d

##
## Include the rule for linking the cFE application.
## This is architecture/compiler/os specific, so it must be included from the ARCH directory
##
include $(CFE_PSP_SRC)/$(PSP)/make/link-rules.mak

install:
	$(CP) $(APPTARGET).$(APP_EXT) $(EXEDIR)

installdocs:
	-mkdir -p $(CFS_MISSION)/docs/users_guide/html/$(CPUNAME)/$(APPTARGET)/
	-mkdir -p $(CFS_MISSION)/docs/users_guide/latex/$(CPUNAME)/$(APPTARGET)/
	-$(CP) ../docs/users_guide/html/$(APPTARGET)/*.*      $(CFS_MISSION)/docs/users_guide/html/$(CPUNAME)/$(APPTARGET)/
	-$(CP) ../docs/users_guide/latex/$(APPTARGET)/*.*     $(CFS_MISSION)/docs/users_guide/latex/$(CPUNAME)/$(APPTARGET)/
	-mkdir -p $(CFS_MISSION)/docs/detailed_design/html/$(CPUNAME)/$(APPTARGET)/
	-mkdir -p $(CFS_MISSION)/docs/detailed_design/latex/$(CPUNAME)/$(APPTARGET)/
	-$(CP) ../docs/detailed_design/html/$(APPTARGET)/*.*  $(CFS_MISSION)/docs/detailed_design/html/$(CPUNAME)/$(APPTARGET)/
	-$(CP) ../docs/detailed_design/latex/$(APPTARGET)/*.* $(CFS_MISSION)/docs/detailed_design/latex/$(CPUNAME)/$(APPTARGET)/

##
## Default rule for generating a users guide
##
usersguide::
	-mkdir -p ../docs/users_guide/html/$(APPTARGET)/
	-mkdir -p ../docs/users_guide/latex/$(APPTARGET)/
	doxygen user_doxy
	
##
## Default rule for generating a detailed design documentation
##
detaileddocs::
	-mkdir -p ../docs/detailed_design/html/$(APPTARGET)/
	-mkdir -p ../docs/detailed_design/latex/$(APPTARGET)/
	doxygen detail_doxy
	
##
## Default rule for generating code walk-through documentation
##
codewalkdocs::
	sh ../docs/dox_src/get_date.sh
	-mkdir -p $(CFS_APP_SRC)/$(APPTARGET)/docs/cwt/html
	doxygen codewalk_doxy
	

##
## Rule for cleaning
## All of the possible extentions are included here so we can clean up after
## an old build
##
clean::
	-$(RM) *.o
	-$(RM) *.g*
	-$(RM) *.lis
	-$(RM) *.bundle
	-$(RM) *.so
	-$(RM) *.dll
	-$(RM) *.d
	-$(RM) *.tbl

cleandocs::
	-$(RM) ../docs/users_guide/html/$(APPTARGET)/*.html
	-$(RM) ../docs/users_guide/html/$(APPTARGET)/*.css
	-$(RM) ../docs/users_guide/html/$(APPTARGET)/*.png
	-$(RM) ../docs/users_guide/html/$(APPTARGET)/*.dot
	-$(RM) ../docs/users_guide/html/$(APPTARGET)/*.gif
	-$(RM) ../docs/users_guide/latex/$(APPTARGET)/*.tex
	-$(RM) ../docs/users_guide/latex/$(APPTARGET)/*.dot
	-$(RM) ../docs/users_guide/latex/$(APPTARGET)/*.sty
	-$(RM) ../docs/users_guide/latex/$(APPTARGET)/*.ttf
	-$(RM) ../docs/users_guide/latex/$(APPTARGET)/Makefile
	-$(RM) ../docs/detailed_design/html/$(APPTARGET)/*.html
	-$(RM) ../docs/detailed_design/html/$(APPTARGET)/*.css
	-$(RM) ../docs/detailed_design/html/$(APPTARGET)/*.png
	-$(RM) ../docs/detailed_design/html/$(APPTARGET)/*.dot
	-$(RM) ../docs/detailed_design/html/$(APPTARGET)/*.gif
	-$(RM) ../docs/detailed_design/latex/$(APPTARGET)/*.tex
	-$(RM) ../docs/detailed_design/latex/$(APPTARGET)/*.dot
	-$(RM) ../docs/detailed_design/latex/$(APPTARGET)/*.sty
	-$(RM) ../docs/detailed_design/latex/$(APPTARGET)/*.ttf
	-$(RM) ../docs/detailed_design/latex/$(APPTARGET)/Makefile

#
# Include the dependancy list
#
-include $(APPTARGET).d
