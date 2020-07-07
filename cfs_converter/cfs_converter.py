"""
Copyright (c) 2020 Japan Aerospace Exploration Agency.
All Rights Reserved.

This file is covered by the LICENSE.txt in the root of this project.
"""

import converter
import file_operator
import cfs_logger
import searcher
import msg_creator
import makefile_creator
import topic_manager

import sys
import os
import xml.etree.ElementTree as ET
# import json
# import re
import gc
import traceback


class CfsConverter:
    """Convert cfs files.

    Raises:
        Exception: Not enough arguments
        Exception: ROS node does not exist
        Exception: The specified launch file was not found
        Exception: The specified launch name did not exist

    Returns:
        dict: launch_dic

    """

    output_path = ""
    log_dir_path = ""
    convert_file_path = ""
    data_file_path = ""
    main_func_path = ""
    # save main function name(ex. main_AppName_)
    main_function_name = ""

    conv = None
    fop = None
    logger = None
    searcher = None
    topic_manager = None

    def __init__(self):
        """Constructor.

        Raises:
            Exception: ROS node does not exist

        """
        argvs = sys.argv
        argc = len(argvs)

        # Initialize
        if not (argc != 5 or argc != 7):
            raise Exception('Not enough arguments')

        self.output_path = argvs[1]
        self.log_dir_path = argvs[2]
        self.main_func_path = argvs[3]
        self.fop = file_operator.FileOperator(self.output_path)
        self.convert_file_path, self.data_file_path = self.fop._getWorkFilesPath()
        self.logger = cfs_logger.CfsLogger(self.log_dir_path)

        self.searcher = searcher.Searcher()
        self.topic_manager = topic_manager.TopicManager(
            self.convert_file_path, self.data_file_path)
        self.conv = converter.Converter(
            self.output_path, self.convert_file_path, self.data_file_path,
            self.fop, self.logger, self.topic_manager)
        self.nh_name = None

    def _convert(self):
        """Convert file.

        Raises:
            Exception: ROS node does not exist

        """
        # Read arguments
        argvs = sys.argv
        argc = len(argvs)

        node_path = argvs[4]
        if not os.path.exists(node_path):
            raise Exception("ROS node({})does not exist".format(node_path))

        # Read launch data
        launch_dic = {}
        if argc == 7:
            launch_dic = self._getLaunchDictionary(argvs, node_path)

        # Create output directory
        # Get directory name by split up a node path
        if node_path[len(node_path) - 1] == "/":
            node_path = node_path[:-1]
        path_splited = node_path.split("/")
        path_depth = len(path_splited)
        node_name = path_splited[path_depth - 1]
        if self.output_path[len(self.output_path) - 1] == "/":
            self.output_path = self.output_path[:-1]

        # Create a conversion list of topic name
        if os.path.exists(self.convert_file_path):
            self.topic_manager._getTopicConvertList()

        if os.path.exists(self.data_file_path):
            self.topic_manager._getMaxPipeNumber()

        # Search for package name
        pkg_name = self.searcher._searchPkgName(node_path)

        # Check the number of occurrences of "ROS_INFO,ROS_DEBUG,ROS_WARN"
        ros_print_count = self.searcher._searchRosPrint(node_path)

        # Create directory for cfe application
        cfe_inc_dir_path, cfe_src_dir_path, cfe_apl_name, create_no, cfe_platform_dir_path, cfe_makefile_dir_path = \
            self.fop._createCfeAppDir(
                node_name,
                self.topic_manager.create_list,
                node_path, pkg_name)

        # Get specified main function name from setting file
        setting_main_func_name = ""
        main_func_list = self.fop._readYaml(self.main_func_path)
        if node_path in main_func_list:
            setting_main_func_name = main_func_list[node_path] \
                + "_{}".format(create_no)

        # Convert Msg file to Header file
        # Read message file
        # Get a list of message files
        ros_msg_dir_path = node_path + "/msg"
        if os.path.exists(ros_msg_dir_path):
            ros_msg_files = os.listdir(ros_msg_dir_path)
            creator = msg_creator.MsgCreator(self.fop)
            creator._msg2header(ros_msg_files,
                                ros_msg_dir_path, node_name,
                                cfe_platform_dir_path, pkg_name)

        # Copy ROS source files
        self.fop._copyIncludeFiles(node_path, cfe_inc_dir_path)

        self.fop._copySorceFiles(node_path, cfe_src_dir_path)

        self.fop._copyOtherFiles(node_path, cfe_src_dir_path)

        # Convert tabs to single-byte spaces
        self.fop._tabToSpace(cfe_inc_dir_path)
        self.fop._tabToSpace(cfe_src_dir_path)

        # Convert from the subscribe that does not specify the data type
        # to data type specification format.
        callback_list = []
        self.searcher._searchSubscribeCallback(cfe_src_dir_path)
        callback_list = self.searcher._searchCallback(cfe_src_dir_path)
        self.fop._addDataTypeToSubscribe(callback_list)

        # Search main function in src directory
        main_src_file = self.searcher._searchMainSourceFile(cfe_src_dir_path)

        self.nh_name = self.searcher._searchNameSpace(cfe_src_dir_path)

        # Get variable name of ros::NodeHandle
        self.searcher._searchNodeHandle(cfe_src_dir_path)

        # Get global variables from all files
        self.searcher._searchAllGlobalMembers(cfe_inc_dir_path)
        self.searcher._searchAllGlobalMembers(cfe_src_dir_path)

        # Convert source file
        self._executeConvert(cfe_src_dir_path, node_name, launch_dic,
                             cfe_apl_name, setting_main_func_name,
                             ros_print_count)

        # Save the topic information
        self.topic_manager._saveTopicInfo(node_name, create_no)

        # Create Makefile
        creator = makefile_creator.MakefileCreator()
        creator._createMakeFile(cfe_apl_name, self.main_function_name,
                                main_src_file, cfe_makefile_dir_path)

        self.logger._close()

    def _executeConvert(self, cfe_src_dir_path, node_name, launch_dic,
                        cfe_apl_name, setting_main_func_name, ros_print_count):
        """Exectue convert file.

        Args:
            cfe_src_dir_path (str): Path to the directory of cFE sources
            node_name (str): Name of ROS node
            launch_dic (dict): Contents of launch file
            cfe_apl_name (str): cFE app name
            setting_main_func_name (str): ROS node's main function name
            ros_print_count (int): Number of occurrences of "ROS_INFO,ROS_DEBUG,ROS_WARN"

        """
        # Store rows in an array and replace
        # The order in which the substitutions are performed is
        # Substitute global variables and functions
        # Substitute sequentially according to the design document
        # Finally overwrite the file
        print(cfe_src_dir_path + "Following files are substituting now")
        cfe_src_files = os.listdir(cfe_src_dir_path)
        convert_dirs = []

        for cfe_src_file in cfe_src_files:
            file_path = cfe_src_dir_path + "/" + cfe_src_file
            if os.path.isfile(file_path):
                # If file name is Makefile,makefile,.lib,.def,.a
                if self.conv._isUnnecessaryFile(file_path):
                    continue
                print("substituting({})".format(file_path))

                if cfe_src_file.find(".cpp") != -1 or cfe_src_file.find(".c") != -1:
                    self._executeCppConvert(cfe_src_dir_path, cfe_src_file,
                                            node_name, launch_dic,
                                            cfe_apl_name,
                                            setting_main_func_name,
                                            ros_print_count)
                else:
                    self._executeHeaderConvert(cfe_src_dir_path,
                                               cfe_src_file, cfe_apl_name)

            else:
                # Call recursively
                file_dirs = file_path.split("/")
                file_name = file_dirs[len(file_dirs) - 1]
                if file_path.find("include") >= 0 or file_name.find(node_name) >= 0:
                    convert_dirs.append(file_path)
        del cfe_src_files
        gc.collect()
        for dir_path in convert_dirs:
            self._executeConvert(dir_path, node_name, launch_dic, cfe_apl_name,
                                 setting_main_func_name, ros_print_count)
        del convert_dirs
        gc.collect()

    def _executeCppConvert(self, cfe_src_dir_path, cfe_src_file, node_name,
                           launch_dic, cfe_apl_name, setting_main_func_name,
                           ros_print_count):
        """Execute convert cpp file.

        Args:
            cfe_src_dir_path (str): Path to the directory of cFE sources
            cfe_src_file (str): Source file name of cFE app
            node_name (str): Name of ROS node
            launch_dic (dict): Contents of launch file
            cfe_apl_name (str): cFE app name
            setting_main_func_name (str): ROS node's main function name
            ros_print_count (int): Number of occurrences of "ROS_INFO,ROS_DEBUG,ROS_WARN"

        """
        file_path = cfe_src_dir_path + "/" + cfe_src_file

        # Search global variables etc. from header file
        self.searcher._search_initialize()
        header_file = cfe_src_file.replace(".cpp", ".h")
        header_path = cfe_src_dir_path + "/include/" + node_name + "/" + header_file
        # Search for header files in a include directory
        if os.path.exists(header_path):
            # Search for include files with the same name,
            # search for global variables listed
            # self.searcher._searchGlobalMembers(header_path)
            # If cpp, search for the definition of NodeHandle
            self.searcher._searchNodeHandleMember(header_path)

        # # Get the file's own global variables and global functions
        # self.searcher._searchGlobalMembers(file_path)

        # There is no leading single-byte space,
        # it is separated by a single-byte space,
        # there is no ::, and the line with (is regarded as a line that
        # declares a global function
        self.searcher._searchGlobalFunctions(file_path)

        topic_no_format = "topicNo_" + cfe_apl_name + "_{}"
        rfp = open(file_path, "r")

        source_line, topic_no_list, is_ros_init_exists, ros_topic_count, topic_list, topic_datatype_list, \
            self.main_function_name = self.conv._convertCpp(cfe_apl_name, rfp, cfe_src_file, topic_no_format,
                                                            file_path, self.main_function_name,
                                                            self.searcher, launch_dic, self.nh_name,
                                                            setting_main_func_name)

        rfp.close()

        # Substitute topic number when publishing
        source_line = self.conv._convertTopicByPublish(source_line,
                                                       topic_no_list)

        # Create cpp directory
        dst_lines = self.conv._addSourceToCpp(source_line, file_path,
                                              cfe_apl_name, is_ros_init_exists,
                                              ros_topic_count, topic_no_format,
                                              ros_print_count)
        self.fop._writeFile(file_path, dst_lines, False)

        # Substitute topic name and pipe name
        # Create a conversion list
        if ros_topic_count > 0:
            self.topic_manager._convertTopicAndPipeName(topic_list,
                                                        topic_datatype_list,
                                                        file_path)

        # Delete of unnecessary memory
        gc.collect()

    def _executeHeaderConvert(self, cfe_src_dir_path, cfe_src_file,
                              cfe_apl_name):
        """Execute covert header file.

        Args:
            cfe_src_dir_path (str): Path to the directory of cFE sources
            cfe_src_file (str): Source file name of cFE app
            cfe_apl_name (str): cFE app name

        """
        # Substitute processing
        self.searcher._search_initialize()
        file_path = cfe_src_dir_path + "/" + cfe_src_file

        # # Get the file's own global variables and global functions,
        # a single-byte space, there is no ::, and the line with
        # (is regarded as a line that declares a global function
        self.searcher._searchGlobalFunctions(file_path)

        rfp = open(file_path, "r")
        source_line = self.conv._convertHeader(rfp, self.searcher,
                                               cfe_src_file, file_path,
                                               cfe_apl_name)

        rfp.close()

        # Create header file
        dst_lines = self.conv._addSourceToHeader(source_line, file_path)
        self.fop._writeFile(file_path, dst_lines, False)

        # Delete of unnecessary memory
        gc.collect()

    def _getLaunchDictionary(self, argvs, node_path):
        """Get launch path.

        Args:
            argvs (list): Arguments
            node_path (str): Path of ROS node's sources

        Raises:
            Exception: The specified launch file was not found
            Exception: The specified launch name did not exist

        Returns:
            dic: launch_dic

        """
        launch_dic = {}
        launch_file = argvs[5]
        launch_name = argvs[6]
        launch_path = node_path + "/launch/" + launch_file
        if not os.path.exists(launch_path):
            raise Exception(
                "The specified launch file ({}) was not found".format(
                    launch_file))
        xml_data = ""
        for line in open(launch_path, "r"):
            xml_data = xml_data + line.replace("\n", "")
        root = ET.fromstring(xml_data)
        name_exists_flag = False
        for child in root:
            if child.attrib["name"] == launch_name:
                name_exists_flag = True
                for param in child:
                    launch_dic[param.attrib["name"]] = {
                        "val": param.attrib["value"]}
                break
        if name_exists_flag is False:
            raise Exception(
                "The specified launch name ({}) did not exist"
                .format(launch_name))
        return launch_dic


if __name__ == '__main__':
    # call CfsConverter
    cfs_conv = CfsConverter()
    try:
        cfs_conv._convert()
    except Exception as e:
        print(e)
        t, v, tb = sys.exc_info()
        for dst_line in traceback.format_exception(t, v, tb):
            print(dst_line)
        sys.exit(1)
    sys.exit(0)
