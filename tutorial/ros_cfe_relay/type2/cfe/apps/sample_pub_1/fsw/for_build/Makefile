APPTARGET = sample_pub_1
ENTRY_PT = main_sample_pub_1_
MYDIR = $(CFS_APP_SRC)/$(APPTARGET)/fsw/src
OBJS = sample_pub.o /usr/lib/i386-linux-gnu/libstdc++.so.6
SOURCES = sample_pub.cpp
LOCAL_COPTS = -D_LINUX_ -Wl,stack,131072 
EXEDIR=../exe
SHARED_LIB_LINK = 
BUILD_TYPE = CFE_APP
include ../cfe/cfe-config.mak
include ../cfe/debug-opts.mak
include $(CFE_PSP_SRC)/$(PSP)/make/compiler-opts.mak
INCLUDE_PATH = -I$(OSAL_SRC)/inc -I$(CFE_CORE_SRC)/inc -I$(CFE_PSP_SRC)/inc -I$(CFE_PSP_SRC)/$(PSP)/inc -I$(CFS_APP_SRC)/inc -I$(CFS_APP_SRC)/$(APPTARGET)/fsw/src -I$(CFS_MISSION_INC) -I../cfe/inc -I../inc -I$(CFS_APP_SRC)/$(APPTARGET)/fsw/platform_inc -I$(CFS_APP_SRC)/$(APPTARGET)/fsw/src/include
VPATH = $(CFS_APP_SRC)/$(APPTARGET)/fsw/src
default: $(APPTARGET).$(APP_EXT)
.cpp.o:
	$(COMPILER) $(LOCAL_COPTS) -std=c++11 -m32 -Wall $(INCLUDE_PATH) -g -O0 -DOS_DEBUG_LEVEL=$(DEBUG_LEVEL) -c -o $@ $<
.s.o:
	$(COMPILER) $(LOCAL_COPTS) $(INCLUDE_PATH) $(ASOPTS) $(COPTS) $(DEBUG_OPTS) -DOS_DEBUG_LEVEL=$(DEBUG_LEVEL) -c -o $@ $<
depend: $(SOURCES)
	$(COMPILER) $(LOCAL_COPTS) $(INCLUDE_PATH) $(ASOPTS) $(COPTS) $(DEBUG_OPTS) -DOS_DEBUG_LEVEL=$(DEBUG_LEVEL) -c -o $@ $<
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
usersguide::
	-mkdir -p ../docs/users_guide/html/$(APPTARGET)/
	-mkdir -p ../docs/users_guide/latex/$(APPTARGET)/
	doxygen user_doxy
detaileddocs::
	-mkdir -p ../docs/detailed_design/html/$(APPTARGET)/
	-mkdir -p ../docs/detailed_design/latex/$(APPTARGET)/
	doxygen detail_doxy
codewalkdocs::
	sh ../docs/dox_src/get_date.sh
	-mkdir -p $(CFS_APP_SRC)/$(APPTARGET)/docs/cwt/html
	doxygen codewalk_doxy
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

-include $(APPTARGET).d
