"""
Copyright (c) 2020 Japan Aerospace Exploration Agency.
All Rights Reserved.

This file is covered by the LICENSE.txt in the root of this project.
"""


class MakefileCreator:
    """makefilecreator."""

    def __init__(self):
        """Constructor."""        
        pass

    def _createMakeFile(self, apptarget_name, entry_point, main_src_name, cfe_makefile_dir_path):
        """createMakeFile.
        
        Args:
            apptarget_name (str): Target name of app
            entry_point (str): Entry point
            main_src_name (str): Name of source file of main function
            cfe_makefile_dir_path (str): Path to the directory that contains Makefile

        """        
        wfp = open(cfe_makefile_dir_path + "/Makefile", "w")
        wfp.write("APPTARGET = " + apptarget_name + "\n")
        wfp.write("ENTRY_PT = " + entry_point + "\n")
        wfp.write("MYDIR = $(CFS_APP_SRC)/$(APPTARGET)/fsw/src\n")
        wfp.write("OBJS = " + main_src_name.replace(".cpp", "") + ".o \n")
        wfp.write("SOURCES = " + main_src_name + "\n")
        wfp.write("LOCAL_COPTS = -D_LINUX_ -Wl,stack,131072 \n")
        wfp.write("EXEDIR=../exe\n")
        wfp.write("SHARED_LIB_LINK = \n")
        wfp.write("BUILD_TYPE = CFE_APP\n")
        wfp.write("include ../cfe/cfe-config.mak\n")
        wfp.write("include ../cfe/debug-opts.mak\n")
        wfp.write("include $(CFE_PSP_SRC)/$(PSP)/make/compiler-opts.mak\n")
        wfp.write("INCLUDE_PATH = -I$(OSAL_SRC)/inc -I$(CFE_CORE_SRC)/inc -I$(CFE_PSP_SRC)/inc "
                  "-I$(CFE_PSP_SRC)/$(PSP)/inc "
                  "-I$(CFS_APP_SRC)/inc -I$(CFS_APP_SRC)/$(APPTARGET)/fsw/src -I$(CFS_MISSION_INC) "
                  "-I../cfe/inc -I../inc "
                  "-I$(CFS_APP_SRC)/$(APPTARGET)/fsw/platform_inc -I$(CFS_APP_SRC)/$(APPTARGET)/fsw/src/include\n")
        wfp.write("VPATH = $(CFS_APP_SRC)/$(APPTARGET)/fsw/src\n")
        wfp.write("default: $(APPTARGET).$(APP_EXT)\n")
        wfp.write(".cpp.o:\n")
        wfp.write("	$(COMPILER) $(LOCAL_COPTS) -std=c++11 -m32 -Wall $(INCLUDE_PATH) "
                  "-g -O0 -DOS_DEBUG_LEVEL=$(DEBUG_LEVEL) -c -o $@ $<\n")
        wfp.write(".s.o:\n")
        wfp.write("	$(COMPILER) $(LOCAL_COPTS) $(INCLUDE_PATH) $(ASOPTS) $(COPTS) $(DEBUG_OPTS) "
                  "-DOS_DEBUG_LEVEL=$(DEBUG_LEVEL) -c -o $@ $<\n")
        wfp.write("depend: $(SOURCES)\n")
        wfp.write("	$(COMPILER) $(LOCAL_COPTS) $(INCLUDE_PATH) $(ASOPTS) $(COPTS) $(DEBUG_OPTS) "
                  "-DOS_DEBUG_LEVEL=$(DEBUG_LEVEL) -c -o $@ $<\n")
        wfp.write("include $(CFE_PSP_SRC)/$(PSP)/make/link-rules.mak\n")
        wfp.write("install:\n")
        wfp.write("	$(CP) $(APPTARGET).$(APP_EXT) $(EXEDIR)\n")
        wfp.write("installdocs:\n")
        wfp.write("	-mkdir -p $(CFS_MISSION)/docs/users_guide/html/$(CPUNAME)/$(APPTARGET)/\n")
        wfp.write("	-mkdir -p $(CFS_MISSION)/docs/users_guide/latex/$(CPUNAME)/$(APPTARGET)/\n")
        wfp.write("	-$(CP) ../docs/users_guide/html/$(APPTARGET)/*.*      "
                  "$(CFS_MISSION)/docs/users_guide/html/$(CPUNAME)/$(APPTARGET)/\n")
        wfp.write("	-$(CP) ../docs/users_guide/latex/$(APPTARGET)/*.*     "
                  "$(CFS_MISSION)/docs/users_guide/latex/$(CPUNAME)/$(APPTARGET)/\n")
        wfp.write("	-mkdir -p $(CFS_MISSION)/docs/detailed_design/html/$(CPUNAME)/$(APPTARGET)/\n")
        wfp.write("	-mkdir -p $(CFS_MISSION)/docs/detailed_design/latex/$(CPUNAME)/$(APPTARGET)/\n")
        wfp.write("	-$(CP) ../docs/detailed_design/html/$(APPTARGET)/*.*  "
                  "$(CFS_MISSION)/docs/detailed_design/html/$(CPUNAME)/$(APPTARGET)/\n")
        wfp.write("	-$(CP) ../docs/detailed_design/latex/$(APPTARGET)/*.* "
                  "$(CFS_MISSION)/docs/detailed_design/latex/$(CPUNAME)/$(APPTARGET)/\n")
        wfp.write("usersguide::\n")
        wfp.write("	-mkdir -p ../docs/users_guide/html/$(APPTARGET)/\n")
        wfp.write("	-mkdir -p ../docs/users_guide/latex/$(APPTARGET)/\n")
        wfp.write("	doxygen user_doxy\n")
        wfp.write("detaileddocs::\n")
        wfp.write("	-mkdir -p ../docs/detailed_design/html/$(APPTARGET)/\n")
        wfp.write("	-mkdir -p ../docs/detailed_design/latex/$(APPTARGET)/\n")
        wfp.write("	doxygen detail_doxy\n")
        wfp.write("codewalkdocs::\n")
        wfp.write("	sh ../docs/dox_src/get_date.sh\n")
        wfp.write("	-mkdir -p $(CFS_APP_SRC)/$(APPTARGET)/docs/cwt/html\n")
        wfp.write("	doxygen codewalk_doxy\n")
        wfp.write("clean::\n")
        wfp.write("	-$(RM) *.o\n")
        wfp.write("	-$(RM) *.g*\n")
        wfp.write("	-$(RM) *.lis\n")
        wfp.write("	-$(RM) *.bundle\n")
        wfp.write("	-$(RM) *.so\n")
        wfp.write("	-$(RM) *.dll\n")
        wfp.write("	-$(RM) *.d\n")
        wfp.write("	-$(RM) *.tbl\n")
        wfp.write("cleandocs::\n")
        wfp.write("	-$(RM) ../docs/users_guide/html/$(APPTARGET)/*.html\n")
        wfp.write("	-$(RM) ../docs/users_guide/html/$(APPTARGET)/*.css\n")
        wfp.write("	-$(RM) ../docs/users_guide/html/$(APPTARGET)/*.png\n")
        wfp.write("	-$(RM) ../docs/users_guide/html/$(APPTARGET)/*.dot\n")
        wfp.write("	-$(RM) ../docs/users_guide/html/$(APPTARGET)/*.gif\n")
        wfp.write("	-$(RM) ../docs/users_guide/latex/$(APPTARGET)/*.tex\n")
        wfp.write("	-$(RM) ../docs/users_guide/latex/$(APPTARGET)/*.dot\n")
        wfp.write("	-$(RM) ../docs/users_guide/latex/$(APPTARGET)/*.sty\n")
        wfp.write("	-$(RM) ../docs/users_guide/latex/$(APPTARGET)/*.ttf\n")
        wfp.write("	-$(RM) ../docs/users_guide/latex/$(APPTARGET)/Makefile\n")
        wfp.write("	-$(RM) ../docs/detailed_design/html/$(APPTARGET)/*.html\n")
        wfp.write("	-$(RM) ../docs/detailed_design/html/$(APPTARGET)/*.css\n")
        wfp.write("	-$(RM) ../docs/detailed_design/html/$(APPTARGET)/*.png\n")
        wfp.write("	-$(RM) ../docs/detailed_design/html/$(APPTARGET)/*.dot\n")
        wfp.write("	-$(RM) ../docs/detailed_design/html/$(APPTARGET)/*.gif\n")
        wfp.write("	-$(RM) ../docs/detailed_design/latex/$(APPTARGET)/*.tex\n")
        wfp.write("	-$(RM) ../docs/detailed_design/latex/$(APPTARGET)/*.dot\n")
        wfp.write("	-$(RM) ../docs/detailed_design/latex/$(APPTARGET)/*.sty\n")
        wfp.write("	-$(RM) ../docs/detailed_design/latex/$(APPTARGET)/*.ttf\n")
        wfp.write("	-$(RM) ../docs/detailed_design/latex/$(APPTARGET)/Makefile\n")
        wfp.write("\n")
        wfp.write("-include $(APPTARGET).d\n")
        wfp.close()
