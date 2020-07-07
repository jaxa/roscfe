"""
Copyright (c) 2020 Japan Aerospace Exploration Agency.
All Rights Reserved.

This file is covered by the LICENSE.txt in the root of this project.
"""

import re

# import file_operator


class Converter:
    """Converter.

    Raises:
        Exception: Parameter not found from within launch file. Please check for other similar errors.

    """

    # save main function name and return type
    main_function_code = ""
    # save file name with main function
    main_function_file = ""

    output_path = ""
    convert_file_path = ""
    data_file_path = ""

    node_handle_member = ""

    fop = None
    logger = None
    topic_manager = None

    TEMP_TOPIC_NO = "TEMP::TOPIC::NO:_:"

    namespace = None
    ros_info_count = 0

    def __init__(self, output_path, convert_file_path, data_file_path, fop,
                 logger, topic_manager):
        """Constructor.

        Args:
            output_path (str): Output directory path
            convert_file_path (str): Converted file's path
            data_file_path (tuple): Working directory path
            fop (FileOperator): File operator
            logger (CfsLogger): Logger
            topic_manager (TopicManager): Topic manager

        """
        self.output_path = output_path
        self.convert_file_path = convert_file_path
        self.data_file_path = data_file_path
        self.fop = fop
        self.logger = logger
        self.topic_manager = topic_manager
        self.namespace = None

    def _convertHeader(self, rfp, searcher, cfe_src_file,
                       file_path, cfe_apl_name):
        """Convert header file.

        Args:
            rfp (IO): File object
            searcher (Searcher): Searcher object
            cfe_src_file (str): Source file name of cFE app
            file_path (str): File path of cFE app's source
            cfe_apl_name (str): cFE app name

        Returns:
            Str: Converted line

        """
        source_line = []
        bracket_flag = False
        subscribe_callback_data_type = ""
        subscribe_callback_bracket_count = 0
        subscribe_member_name = ""
        for line in rfp:
            original_line = line

            # If the line is a comment only, proceed to the next process
            if line.find("//") >= 0:
                comment_check = re.sub(r'//.+\n', "", line)
                comment_check = comment_check.strip()
                if comment_check == "":
                    source_line.append(line)
                    continue

            # Read file and Substitute global variables and functions
            for member in searcher.global_members:
                line = line.replace(member, member + "_" + cfe_apl_name + "_")

            # Substitution of Global variable
            for function in searcher.global_functions:
                if not function == "":
                    line = line.replace(function,
                                        function + "_" + cfe_apl_name + "_")

            # ros::Duration(0.1).sleep()
            if line.find("ros::Duration") >= 0 and line.find("sleep") >= 0:
                line = self._convertRosDurationSleep(line)

            # Subscribe callback function
            if line.find("::ConstPtr&") >= 0 and line.find(",") == -1:
                line, subscribe_callback_data_type, \
                    subscribe_callback_bracket_count, subscribe_member_name, \
                    bracket_flag = self._convertSubscribeCallback(line, file_path,
                                                                  bracket_flag)
            elif line.find("::ConstPtr&") >= 0:
                line = line.replace("::ConstPtr&", "*")
            # Subscribe callback function by ConstPtr argument
            if line.find("ConstPtr ") >= 0 and line.find(",") == -1:
                # callback_list contains a list of callback functions
                # at the time of Subscribe
                is_target = False
                for callback in searcher.subscribe_callback_list:
                    if line.find(callback["method"] + "(") != -1:
                        is_target = True
                        break
                if is_target is True:
                    line, subscribe_callback_data_type, \
                        subscribe_callback_bracket_count, subscribe_member_name, \
                        bracket_flag = self._convertSubscribeCallbackConstPtr(
                            line, file_path, bracket_flag)
            elif line.find("ConstPtr") >= 0:
                line = line.replace("ConstPtr", "*")

            if line.find("ros::Duration") >= 0:
                line = self._convertRosDuration(line)

            if line.find("ros::Time") >= 0:
                line = self._convertRosTime(line)

            # #include <ros/ros.h>
            # #include "ros/ros.h"
            # #include <iostream>
            # #include <tf/transform_listener.h>
            # #include <std_msgs/Float32MultiArray.h>
            if line.find("#include <ros/ros.h>") >= 0 or line.find(
                "#include \"ros/ros.h\"") >= 0 or line.find(
                    "#include <iostream>") >= 0 or line.find(
                        "#include <tf/transform_listener.h>") >= 0 or \
                    line.find("#include <std_msgs/Float32MultiArray.h>") >= 0:
                line = ""

            # ros::Publisher
            # ros::Subscriber
            # ros::NodeHandle
            if line.find("ros::Publisher") >= 0 or line.find("ros::Subscriber") >= 0 or \
               line.find("ros::NodeHandle") >= 0:
                line = "\n"

            # #include <tf/transform_broadcaster.h>
            if line.find("#include <tf/transform_broadcaster.h>") >= 0:
                line = "#include <tf/LinearMath/Transform.h>\n"
                line = line + "#include <tf/LinearMath/StampedTransform.h>\n"
            source_line.append(line)
            self.logger._dumpLog(original_line, line, cfe_src_file)
        return source_line

    def _convertCpp(self, cfe_apl_name, rfp, cfe_src_file, topic_no_format,
                    file_path, main_function_name, searcher, launch_dic,
                    nh_name, setting_main_func_name):
        """Convert Cpp file.

        Args:
            cfe_apl_name (str): cFE app name
            rfp (IO): File object
            cfe_src_file (str): Source file name of cFE ap
            topic_no_format (str): Topic name with number
            file_path (str): File path of cFE app's source
            main_function_name (str): Name of main function
            searcher (Searcher): Searcher object
            launch_dic (dict): Contents of launch file
            nh_name (str): Name of node handle
            setting_main_func_name (str): ROS node's main function name

        Returns:
            tuple: (source_line, topic_no_list, is_ros_init_exists, ros_topic_count, \
            topic_list, topic_datatype_list, main_function_name)

        """
        source_line = []
        bracket_flag = False
        ros_topic_count = 0
        topic_list = []
        topic_datatype_list = []
        topic_no_list = {}
        subscribe_callback_data_type = ""
        subscribe_callback_bracket_count = 0
        is_ros_init_exists = False
        subscribe_member_name = ""
        main_bracket_count = 0
        self.namespace = nh_name
        self.node_handle_member = searcher.node_handle_member
        for line in rfp:
            original_line = line
            # Prevent double creation of { at function start
            if bracket_flag is True:
                if line.find("{") != -1:
                    line = "\n"
                bracket_flag = False

            # If the line is a comment only, proceed to the next process
            if line.find("//") >= 0:
                comment_check = re.sub(r'//.+\n', "", line)
                comment_check = comment_check.strip()
                if comment_check == "":
                    source_line.append(line)
                    continue

            # Detect the end of the subscribe callback function,
            # and add a process to discard data at the end
            # Also, change vector type variable names for JointState,
            # 64MultiArray, and MultiArrayLayout
            if subscribe_callback_bracket_count > 0:
                if line.find("{") >= 0:
                    subscribe_callback_bracket_count = subscribe_callback_bracket_count + 1
                if line.find("}") >= 0:
                    subscribe_callback_bracket_count = subscribe_callback_bracket_count - 1
                line = self._convertVectorInStruct(line, subscribe_callback_data_type)
                if subscribe_callback_bracket_count == 0:
                    line = "    " + subscribe_member_name + "->deleteData();\n" + "}"

            # Detects the end of the main function and deletes the return statement
            if main_bracket_count > 0:
                if line.find("{") >= 0:
                    main_bracket_count = main_bracket_count + 1
                if line.find("}") >= 0:
                    main_bracket_count = main_bracket_count - 1
                if line.find("return") >= 0:
                    head_space = self._getHeadSpace(line)
                    line = "\n"

            # getParam
            if line.find("getParam(") != -1:
                line = self._convertGetParam(line, launch_dic)

            # Read file and Substitute global variables and functions
            for member in searcher.global_members:
                line = line.replace(member, member + "_" + cfe_apl_name + "_")

            # Substitution of main statement
            if line.find("int main") >= 0:
                bracket_flag, main_bracket_count, line, main_function_name = \
                    self._convertMainFunc(cfe_apl_name, file_path, main_bracket_count,
                                          line, bracket_flag, setting_main_func_name)

            # Substitution of Global variable
            for function in searcher.global_functions:
                if not function == "":
                    line = line.replace(function, function + "_" + cfe_apl_name + "_")

            # ros::init()
            if line.find("ros::init") != -1:
                is_ros_init_exists, line = self._convertRosInit(line, cfe_apl_name)

            # NodeHandle.advertise
            if line.find(".advertise<") != -1:
                ros_topic_count, topic_no_list, topic_list, topic_datatype_list, line = \
                    self._convertAdvertise(line, topic_no_format, ros_topic_count,
                                           topic_list, topic_no_list, topic_datatype_list)

            # ros::Rate
            if line.find("ros::Rate") != -1:
                line = self._convertRosRate(line)

            # ros::ok()
            if line.find("ros::ok()") != -1:
                line = self._convertRosOk(line)

            # NodeHandle::ok()
            if len(self.node_handle_member) > 0:
                if line.find(self.node_handle_member + ".ok()") != -1:
                    line = self._convertNodeHandleOk(line)

            # ros_pub.publish
            if line.find(".publish(") != -1:
                line = self._convertPublish(line)

            # ros::Time::now().toSec()
            if line.find("ros::Time::now().toSec()") != -1:
                line = self._convertRosTimeNowToSec(line)

            # printf
            if line.find(" printf") != -1:
                line = self._convertPrintf(line)

            # NodeHandle::param<DataType>("A", b, 1)
            if line.find(".param<") >= 0:
                line = self._convertParam(line)

            # ros::Duration(0.1).sleep()
            if line.find("ros::Duration") >= 0 and line.find("sleep") >= 0:
                line = self._convertRosDurationSleep(line)

            # sleep()
            if line.find("sleep()") >= 0:
                line = self._convertSleep(line, topic_no_format)

            # Subscribe callback function by ::ConstPtr& argument
            if line.find("::ConstPtr&") >= 0 and line.find(",") == -1:
                line, subscribe_callback_data_type, subscribe_callback_bracket_count, \
                    subscribe_member_name, bracket_flag = self._convertSubscribeCallback(line, file_path, bracket_flag)
            elif line.find("::ConstPtr&") >= 0:
                line = line.replace("::ConstPtr&", "*")
            # Subscribe callback function by ConstPtr argument
            if line.find("ConstPtr ") >= 0 and line.find(",") == -1:
                # callback_list contains a list of callback functions at the time of Subscribe
                is_target = False
                for callback in searcher.subscribe_callback_list:
                    if line.find(callback["method"] + "(") != -1:
                        is_target = True
                        break
                if is_target is True:
                    line, subscribe_callback_data_type, subscribe_callback_bracket_count, subscribe_member_name, \
                        bracket_flag = self._convertSubscribeCallbackConstPtr(line, file_path, bracket_flag)
            elif line.find("ConstPtr") >= 0:
                line = line.replace("ConstPtr", "*")

            # ROS_INFO
            if line.find("ROS_INFO(") >= 0:
                line = self._convertRosInfo(line, cfe_apl_name)

            # ROS_DEBUG
            if line.find("ROS_DEBUG(") >= 0:
                line = self._convertRosDebug(line, cfe_apl_name)

            # ROS_WARN
            if line.find("ROS_WARN(") >= 0:
                line = self._convertRosWarn(line, cfe_apl_name)

            # include <iostream>
            if line.find("#include <iostream>") >= 0:
                line = ""

            # cerr
            if line.find("cerr") >= 0:
                line = self._convertCerr(line)

            # NodeHandle.subscribe
            if line.find("subscribe<") >= 0:
                line, ros_topic_count, topic_list, topic_datatype_list = \
                    self._convertSubscribe(line, ros_topic_count, topic_no_format, topic_list, topic_datatype_list)

            # ros::spinOnce
            if line.find("ros::spinOnce()") != -1:
                line = self._convertSpinOnce(line)

            # ros::spin
            if line.find("ros::spin()") != -1:
                line = self._convertRosSpin(line)

            # NodeHandle member in Constructor
            if line.find(searcher.node_handle_member) >= 0 and searcher.node_handle_member != "":
                line = self._deleteNodeHandleInitializer(line)

            # ros::Publisher
            if line.find("ros::Publisher") >= 0 or line.find("ros::Subscriber") >= 0 or \
               line.find("ros::NodeHandle") >= 0:
                line = "\n"

            # ros::Time::now()
            if line.find("ros::Time::now()") >= 0:
                line = self._convertRosTimeNow(line)

            if line.find("ros::param::get(") >= 0:
                line = self._convertRosParamGet(line, launch_dic)

            if line.find("ros::package::getPath") >= 0:
                line = self._convertRosPackageGetPath(line, cfe_apl_name)

            if line.find("resolveName(") >= 0:
                line = self._convertRosNodeHandleResolveName(line)

            if line.find("ros::Duration") >= 0:
                line = self._convertRosDuration(line)

            # --------------------------------->>

            if line.find("ros::Time") >= 0:
                line = self._convertRosTime(line)

            # #include "ros/ros.h"
            # #include "ros/package.h"
            # #include <ros/package.h>
            if line.find("#include \"ros/ros.h\"") >= 0 or line.find("#include \"ros/package.h\"") >= 0 or \
               line.find("#include <ros/package.h>") >= 0:
                line = ""

            source_line.append(line)
            self.logger._dumpLog(original_line, line, cfe_src_file)
        return source_line, topic_no_list, is_ros_init_exists, ros_topic_count, \
            topic_list, topic_datatype_list, main_function_name

    def _addSourceToHeader(self, source_line, file_path):
        """Add source to header.

        Args:
            source_line (list): Line of source
            file_path (str): File path of cFE app's source

        Returns:
            list: dst_lines

        """
        cfe_include_flag = True

        dst_lines = []
        for line in source_line:
            dst_lines.append(line)
            if cfe_include_flag is True:
                dst_lines.append("extern \"C\"{ \n")
                dst_lines.append("    #include \"cfe.h\" \n")
                dst_lines.append("}\n")
                dst_lines.append("#include \"convert_lib.h\"\n")
                cfe_include_flag = False

        if self.main_function_file != "":
            if file_path.find(self.main_function_file) >= 0:
                dst_lines.append("extern \"C\"{ \n")
                dst_lines.append("    " + self.main_function_code + "\n")
                dst_lines.append("}\n")
        return dst_lines

    def _addSourceToCpp(self, source_line, file_path, cfe_apl_name, is_ros_init_exists, ros_topic_count,
                        topic_no_format, ros_print_count):
        """Add source to cpp file.

        Args:
            source_line (list): Line of source
            file_path (str): File path of cFE app's source
            cfe_apl_name (str): cFE app name
            is_ros_init_exists (bool): Whether "ros::init" is in ROS source or not
            ros_topic_count (int): Counter of ros topics
            topic_no_format (str): Topic name with number
            ros_print_count (int): Number of occurrences of "ROS_INFO,ROS_DEBUG,ROS_WARN"

        Returns:
            list: Converted line of source

        """
        evt_count_flag = True
        ros_topic_flag = True
        cfe_include_flag = True

        dst_lines = []
        for line in source_line:
            if line.find("CONVERT_RosInit") >= 0:
                for i in range(ros_print_count):
                    head_space = self._getHeadSpace(line)
                    dst_lines.append(head_space + "EventFilters" +
                                     cfe_apl_name + "[{}].EventID = 0x{:02x};\n".format(i, i))
                    dst_lines.append(head_space + "EventFilters" +
                                     cfe_apl_name + "[{}].Mask = CFE_EVS_NO_FILTER;\n".format(i))
            if cfe_include_flag is True:
                dst_lines.append("extern \"C\"{ \n")
                dst_lines.append("    #include \"cfe.h\" \n")
                if self.main_function_file != "":
                    if file_path.find(self.main_function_file) == -1:
                        dst_lines.append("    " + self.main_function_code + "\n")
                dst_lines.append("}\n")
                dst_lines.append("#include \"convert_lib.h\"\n")
                cfe_include_flag = False

            dst_lines.append(line)

            if line.find("#include") >= 0 and evt_count_flag is True:
                # Add constants related to EventFilters
                if is_ros_init_exists:
                    if ros_print_count > 0:
                        dst_lines.append("#define EVT_COUNT {}\n".format(ros_print_count))
                        dst_lines.append("CFE_EVS_BinFilter_t EventFilters" + cfe_apl_name + "[EVT_COUNT];\n")
                    else:
                        dst_lines.append("#define EVT_COUNT 0\n")
                        dst_lines.append("CFE_EVS_BinFilter_t EventFilters" + cfe_apl_name + "[EVT_COUNT];\n")
                    is_ros_init_exists = False
                evt_count_flag = False

            if line.find("#include") >= 0 and ros_topic_count > 0 and ros_topic_flag is True:
                for i in range(ros_topic_count):
                    dst_lines.append("int " + topic_no_format.format(i) + ";\n")
                ros_topic_flag = False

        if self.main_function_file != "":
            if file_path.find(self.main_function_file) >= 0:
                dst_lines.append("extern \"C\"{ \n")
                dst_lines.append("    " + self.main_function_code + "\n")
                dst_lines.append("}\n")
        return dst_lines

    def _isUnnecessaryFile(self, file_path):
        """Check necessary file.

        Args:
            file_path (str): File path

        Returns:
            bool: Whether give file is necessary or not

        """
        if file_path.find(".lib") >= 0 or file_path.find(".a") >= 0 or file_path.find(".def") >= 0 or \
           file_path.find("makefile") >= 0 or file_path.find("Makefile") >= 0:
            return True
        return False

    def _deleteNodeHandleInitializer(self, line):
        """Delete node handle.

        Args:
            line (str): Line of source

        Returns:
            (str): Converted line

        """
        line = line.replace(" ", "").replace("::", "_CONSTRCUTOR_")
        line_splits = line.split(":")
        end_char = ""
        if line.find("{") >= 0:
            end_char = "{"
        line = line_splits[0].replace("_CONSTRCUTOR_", "::") + end_char + "\n"
        return line

    def _getHeadSpace(self, target_line):
        """Get leading space.

        Args:
            target_line (str): Line of source

        Returns:
            str: Leading space

        """
        pattern = re.compile(r"^\s+")
        match = pattern.search(target_line)
        if match is None:
            return ""
        head_space = match.group(0)
        return head_space

    def _getDataType(self, target_line):
        """Get data type.

        Args:
            target_line (str): Line of source

        Returns:
            str: data_type

        """
        pattern = re.compile(r"<.+>")
        match = pattern.search(target_line)
        data_type = match.group(0).replace("<", "").replace(">", "")
        return data_type

    def _getArgument(self, target_line):
        """Get Argument.

        Args:
            target_line (str): Line of source

        Returns:
            str: Arguments

        """
        pattern = re.compile(r"\(.+\)")
        match = pattern.search(target_line)
        argv_code = match.group(0)
        argv_code = argv_code.replace("(", "")
        argv_code = argv_code.replace(")", "")
        return argv_code

    def _convertVectorInStruct(self, line, data_type):
        """Convert vector.

        Args:
            line (str): Line of source
            data_type (str): Data type stored in vector

        Returns:
            str: Converted line

        """
        if data_type == "sensor_msgs::JointState":
            if line.find("position[") >= 0:
                line = line.replace("position[", "positionData[")
            if line.find("velocity[") >= 0:
                line = line.replace("velocity[", "velocityData[")
            if line.find("effort[") >= 0:
                line = line.replace("effort[", "effortData[")
        if data_type == "std_msgs::MultiArrayLayout":
            if line.find("dim[") >= 0:
                line = line.replace("dim[", "dimData[")
        if data_type == "std_msgs::Float64MultiArray":
            if line.find("data[") >= 0:
                line = line.replace("data[", "dataData[")
        return line

    def _convertTopicByPublish(self, source_line, topic_no_list):
        """Convert topic.

        Args:
            source_line (list): Line of source
            topic_no_list (dict): Topics defined in source

        Returns:
            list: temp_line

        """
        temp_line = []
        for line in source_line:
            if line.find(self.TEMP_TOPIC_NO) >= 0:
                # split
                argv_code = self._getArgument(line)
                argv_split = argv_code.split(",")
                topic_no_code = argv_split[2]
                # Get variable name
                topic_no_split = topic_no_code.split(":_:")
                member_name = topic_no_split[1]
                topic_no_name = topic_no_list[member_name]
                line = line.replace(topic_no_code, topic_no_name)
            temp_line.append(line)
        return temp_line

    def _convertRosSpin(self, line):
        """Convert rosSpin.

        Args:
            line (str): Line of source

        Returns:
            str: line

        """
        line = line.replace("ros::spin", "CONVERT_RosSpin")
        return line

    def _convertSpinOnce(self, line):
        """Convert spin at once.

        Args:
            line (str): Line of source

        Returns:
            str: line

        """
        line = line.replace("ros::spinOnce", "CONVERT_RosSpinOnce")
        return line

    def _convertSubscribe(self, line, ros_topic_count, topic_no_format, topic_list, topic_datatype_list):
        """Convert subscribe.

        Args:
            line (str): Line of source
            ros_topic_count (int): Counter of ros topics
            topic_no_format (str): Topic name with number
            topic_list (list): List of topics
            topic_datatype_list (list): List of data type of each topics

        Returns:
            tuple: (line, ros_topic_count, topic_list, topic_datatype_list)

        """
        topic_datatype_list.append(self._getTopicDataType(line))

        argv_code = self._getArgument(line)
        argv_code = argv_code.replace("(", "")
        argv_code = argv_code.replace(")", "")
        argv_splits = argv_code.split(",".strip())
        topic_name = argv_splits[0].replace(".c_str()", "").replace(".c_str", "")
        queue_size = argv_splits[1]
        callback_func = argv_splits[2]
        data_type = self._getDataType(line)
        head_space = self._getHeadSpace(line)
        this_flag = False
        pipe_name = "0x{0:013x}".format(self.topic_manager.max_pipe_no)
        self.topic_manager._incrementMaxPipeNo()
        if line.find("this") >= 0:
            this_flag = True
        if this_flag:
            line = head_space + topic_no_format.format(ros_topic_count) + \
                " = CONVERT_RosNodeHandleSubscribe(\"" + pipe_name + "\", " + topic_name + ", " + queue_size + \
                ", std::bind(" + callback_func + ", this, std::placeholders::_1), sizeof(" + data_type + "));\n"
        else:
            line = head_space + "std::function<void(void*)> f = " + callback_func + ";\n"
            line = line + head_space + topic_no_format.format(ros_topic_count) + \
                " = CONVERT_RosNodeHandleSubscribe(\"" + pipe_name + "\", " + topic_name + ", " + queue_size + \
                ", f, sizeof(" + data_type + "));\n"

        ros_topic_count = ros_topic_count + 1
        topic_list.append(topic_name.replace(".c_str()", "").replace(".c_str", ""))
        return line, ros_topic_count, topic_list, topic_datatype_list

    def _convertCerr(self, line):
        """Convert replace.

        Args:
            line (str): Line of source

        Returns:
            str: line

        """
        line = line.replace("std::cerr << ", "OS_printf(")
        line = line.replace("<< std::endl", ")")
        return line

    def _convertRosInfo(self, line, cfe_apl_name):
        """Convert ros info.

        Args:
            line (str): Line of source
            cfe_apl_name (str): cFE app name

        Returns:
            str: Converted line

        """
        return self._convertRosPrint(line, cfe_apl_name, "CFE_EVS_INFORMATION", "ROS_INFO")

    def _convertRosDebug(self, line, cfe_apl_name):
        """Debug convert Rosfile.

        Args:
            line (list): Line of source
            cfe_apl_name (str): cFE app name

        Returns:
            str: Converted line

        """
        return self._convertRosPrint(line, cfe_apl_name, "CFE_EVS_DEBUG", "ROS_DEBUG")

    def _convertRosWarn(self, line, cfe_apl_name):
        """Conver ros warn.

        Args:
            line (str): Line of source
            cfe_apl_name (str): cFE app name

        Returns:
            str: Converted line

        """
        return self._convertRosPrint(line, cfe_apl_name, "CFE_EVS_INFORMATION", "ROS_WARN")

    def _convertRosPrint(self, line, cfe_apl_name, event_id, target_str):
        """Convert ros print.

        Args:
            line (str): Line of source
            cfe_apl_name (str): cFE app name
            event_id (str): Event ID
            target_str (str): Words to be converted

        Returns:
            str: Converted line

        """
        line = line.replace(target_str + "(", "CFE_EVS_SendEvent(0x{:02x}, {}, ".format(self.ros_info_count, event_id))
        self.ros_info_count = self.ros_info_count + 1
        return line

    def _convertSubscribeCallback(self, line, file_path, bracket_flag):
        """Convert subscription callback.

        Args:
            line (str): Line of source
            file_path (str): File path
            bracket_flag (bool): Whether bracket '{}' exist or not

        Returns:
            tuple: (line, subscribe_callback_data_type, subscribe_callback_bracket_count,
            subscribe_member_name, bracket_flag)

        """
        # Extract data type and name of argument
        argv_code = self._getArgument(line)
        argv_code = argv_code.replace("(", "")
        argv_code = argv_code.replace(")", "")
        argv_code = argv_code.replace("const ", "")
        argv_code = argv_code.replace("::ConstPtr&", "")
        # split
        argv_splits = argv_code.split(" ")
        data_type = argv_splits[0]
        data_name = argv_splits[1]

        # (Erase the following
        line = re.sub(r"\(.+\n", "", line)
        line = line + "(void* callback_msg_cfsconverter)"
        if file_path.find(".cpp") >= 0:
            line = line + "{\n"
            line = line + "    " + data_type + "* " + data_name + " = (" + data_type + "*)callback_msg_cfsconverter;\n"
            line = line + "    " + data_name + "->pointer2string();\n"
            line = line + "    " + data_name + "->pointer2vector();\n"
            # It is not known whether the original source code has entered a line break and entered {},
            # so check it at the next reading
            bracket_flag = True
        else:
            line = line + ";\n"
        subscribe_callback_data_type = data_type
        subscribe_callback_bracket_count = 1
        subscribe_member_name = data_name
        return line, subscribe_callback_data_type, subscribe_callback_bracket_count, subscribe_member_name, bracket_flag

    def _convertSubscribeCallbackConstPtr(self, line, file_path, bracket_flag):
        """Convert subscribe callback const.

        Args:
            line (str): Line of source
            file_path (str): File path
            bracket_flag (bool): Whether bracket '{}' exist or not

        Returns:
            tuple: (line, subscribe_callback_data_type, subscribe_callback_bracket_count, subscribe_member_name, bracket_flag)

        """
        return self._convertSubscribeCallbackCommon(line, file_path, bracket_flag, "ConstPtr")

    def _convertSubscribeCallbackCommon(self, line, file_path, bracket_flag, replace_str):
        """Convert subscribe callback common.

        Args:
            line (str): Line of source
            file_path (str): File path
            bracket_flag (bool): Whether bracket '{}' exist or not
            replace_str (str): Replaced string.

        Returns:
            tuple: (line, subscribe_callback_data_type, subscribe_callback_bracket_count, subscribe_member_name, bracket_flag)

        """
        # Extract data type and name of argument
        argv_code = self._getArgument(line)
        argv_code = argv_code.replace("(", "")
        argv_code = argv_code.replace(")", "")
        argv_code = argv_code.replace("const ", "")
        argv_code = argv_code.replace(replace_str, "")
        # split
        argv_splits = argv_code.split(" ")
        data_type = argv_splits[0]
        data_name = argv_splits[1]

        # (Erase the following
        line = re.sub(r"\(.+\n", "", line)
        line = line + "(void* callback_msg_cfsconverter)"
        if file_path.find(".cpp") >= 0:
            line = line + "{\n"
            line = line + "    " + data_type + "* " + data_name + " = (" + data_type + "*)callback_msg_cfsconverter;\n"
            line = line + "    " + data_name + "->pointer2string();\n"
            line = line + "    " + data_name + "->pointer2vector();\n"
            # It is not known whether the original source code has entered a line break and entered {},
            # so check it at the next reading
            bracket_flag = True
        else:
            line = line + ";\n"
        subscribe_callback_data_type = data_type
        subscribe_callback_bracket_count = 1
        subscribe_member_name = data_name
        return line, subscribe_callback_data_type, subscribe_callback_bracket_count, subscribe_member_name, bracket_flag

    def _convertSleep(self, line, topic_no_format):
        """Convert sleep.

        Args:
            line (str): Line of source
            topic_no_format (str): Topic name with number

        Returns:
            str: Converted line

        """
        ros_rate_member = re.sub(r"\.sleep\(\).+", "", line)
        line = re.sub(ros_rate_member.strip() + r".+\n", "", line)
        line = line + "CONVERT_RosRateSleep(" + ros_rate_member.strip() + ", " + topic_no_format.format(0) + ");\n"
        return line

    def _convertRosDurationSleep(self, line):
        """Convert ros dulation sleep.

        Args:
            line (str): Line of source

        Returns:
            str: Converted line

        """
        pattern = re.compile(r"[0-9\.]+")
        match = pattern.search(line)
        line = re.sub(r"ros::Duration.+\n", "", line)
        line = line + "CONVERT_RosDurationSleep(" + match.group(0) + ");\n"
        return line

    def _convertParam(self, line):
        """Convert parameter.

        Args:
            line (str): Line of source

        Returns:
            str: Converted line

        """
        # Get the character string in () and reconstruct
        argv_code = self._getArgument(line)
        argv_splits = argv_code.split(",")
        param_name = argv_splits[0].strip()
        member = argv_splits[1].strip()
        default_param = argv_splits[2]
        combined_code = member + " = " + default_param
        head_space = self._getHeadSpace(line)
        line = head_space + combined_code + ";\n"
        return line

    def _convertPrintf(self, line):
        """Convert print.

        Args:
            line (str): Line of source

        Returns:
            str: Converted line

        """
        line = line.replace(" printf", " OS_printf")
        return line

    def _convertRosTimeNowToSec(self, line):
        """Convert ros time.

        Args:
            line (str): Line of source

        Returns:
            str: Converted line

        """
        line = line.replace("ros::Time::now().toSec()", "CONVERT_RosTimeNowToSec()")
        return line

    def _convertPublish(self, line):
        """Convert publish.

        Args:
            line (str): Line of source

        Returns:
            str: Converted line

        """
        line_splits = line.strip().split(".")
        member_name = line_splits[0]

        # Get the contents of () and delete ()
        pattern = re.compile(r"\(.+\)")
        match = pattern.search(line)
        member = match.group(0)[1:-1]

        head_space = self._getHeadSpace(line)
        added_head_space = head_space + "    "

        # If () is present, the data type must be found in publish
        line = head_space + "{\n"
        if member.find("()") != -1:
            data_type = member.replace("()", "")
            line = line + added_head_space + data_type + " publish_msg;\n"
            line = line + added_head_space + \
                "CONVERT_RosPublisherPublish((CFE_SB_Msg_t*)(&publish_msg), sizeof(publish_msg), " + \
                self.TEMP_TOPIC_NO + member_name + ");\n"
        else:
            # topic_no_name = topic_no_list[member_name]
            line = line + added_head_space + member + ".vector2pointer();\n"
            line = line + added_head_space + member + ".string2pointer();\n"
            # Enter a temporary string and replace it later, as conversion of Advertise may not be complete
            line = line + added_head_space + \
                "CONVERT_RosPublisherPublish((CFE_SB_Msg_t*)(&" + member + "), sizeof(" + member + "), " + \
                self.TEMP_TOPIC_NO + member_name + ");\n"
        line = line + head_space + "}\n"

        return line

    def _convertGetParam(self, line, launch_dic):
        argv_code = self._getArgument(line)
        argv_splits = argv_code.split(",")
        param_name = argv_splits[0].strip()
        member = argv_splits[1].strip()
        # Substitution with data obtained from launch file
        default_param = launch_dic[param_name.replace("\"", "")]["val"]
        default_type = launch_dic[param_name.replace("\"", "")]["type"]
        if default_type == "string":
            combined_code = member + " = \"" + default_param + "\""
        else:
            combined_code = member + " = " + default_param
        head_space = self._getHeadSpace(line)
        line = head_space + combined_code + ";\n"
        return line

    def _convertMainFunc(self, cfe_apl_name, file_path, main_bracket_count, line, bracket_flag, setting_main_func_name):
        """Convert main function.

        Args:
            cfe_apl_name (str): cFE app name
            file_path (str): File path
            main_bracket_count (int): Number of brackets '{}' in main function
            line (str): Line of source
            bracket_flag (bool): Whether bracket '{}' exist or not
            setting_main_func_name (str): ROS node's main function name

        Returns:
            tuple: (bracket_flag, main_bracket_count, line, main_function_name)

        """
        brancket_flag = False
        if line.find("{") >= 0:
            bracket_flag = True

        if len(setting_main_func_name) == 0:
            main_function_name = "main_" + cfe_apl_name + "_"
        else:
            main_function_name = setting_main_func_name + "_"
        self.main_function_code = "void " + main_function_name + "();"
        file_path_splits = file_path.split("/")
        self.main_function_file = file_path_splits[len(file_path_splits) - 1]
        self.main_function_file = self.main_function_file.replace(".cpp", ".h")
        line = re.sub(r'\(.+\n', "", line)
        line = line + "()"
        line = line.replace("int", "void")
        line = line.replace("main", main_function_name)
        if bracket_flag is True:
            line = line + "{\n"
        main_bracket_count = main_bracket_count + 1
        return bracket_flag, main_bracket_count, line, main_function_name

    def _convertRosOk(self, line):
        """Convert ros is ok.

        Args:
            line (str): Line of source

        Returns:
            str: Converted line

        """
        line = line.replace("ros::ok()", "CONVERT_RosOk()")
        return line

    def _convertNodeHandleOk(self, line):
        """Convert node handle is ok.

        Args:
            line (str): Line of source

        Returns:
            str: Converted line

        """
        line = line.replace(self.node_handle_member + ".ok()", "CONVERT_RosOk()")
        return line

    def _convertRosRate(self, line):
        """Conver ros rate.

        Args:
            line (str): Line of source

        Returns:
            str: Converted line

        """
        pattern = re.compile(r"[0-9]+")
        match = pattern.search(line)
        # Branch depending on whether a numerical value is inserted in
        # the variable of ros::Rate or a variable is inserted
        rate_num = ""
        if match is not None:
            rate_num = str(match.group(0))
        else:
            pattern = re.compile(r"\(.+\)")
            match = pattern.search(line)
            rate_num = match.group(0).replace("(", "").replace(")", "")
        line = re.sub(r"ros::Rate\s+", "", line)
        member = re.sub(r'\(.+\).+', "", line)
        head_space = self._getHeadSpace(line)
        line = head_space + "uint32 " + member + " = CONVERT_RosRate(" + rate_num + ");\n"
        return line

    def _getTopicDataType(self, line):
        """Get topic datatype.

        Args:
            line (str): Line of source

        Returns:
            str: Data type

        """
        match_obj = re.search(r'<.+>', line)
        datatype = match_obj.group()
        datatype = datatype.replace("<", "").replace(">", "")
        return datatype

    def _convertAdvertise(self, line, topic_no_format, ros_topic_count, topic_list, topic_no_list, topic_datatype_list):
        """Convert advertise.

        Args:
            line (str): Line of source
            topic_no_format (str): Topic name with number
            ros_topic_count (int): Counter of ros topics
            topic_list (list): List of topics
            topic_no_list (dict): Topics defined in source
            topic_datatype_list (list): List of data types of each topics

        Returns:
            tuple: (ros_topic_count, topic_no_list, topic_list, topic_datatype_list, line)

        """
        topic_datatype_list.append(self._getTopicDataType(line))

        line_splits = line.strip().replace(" ", "").split("=")
        member_name = line_splits[0]
        data_type = self._getDataType(line)
        head_space = self._getHeadSpace(line)
        argv_code = self._getArgument(line)
        argv_splits = argv_code.split(",")
        topic_name = argv_splits[0].strip().replace(".c_str()", "").replace(".c_str", "")
        queue_size = argv_splits[1].strip()
        pipe_name = "0x{0:013x}".format(self.topic_manager.max_pipe_no)
        self.topic_manager._incrementMaxPipeNo()
        line = head_space + topic_no_format.format(ros_topic_count) + \
            " = CONVERT_RosNodeHandleAdvertise(\"" + pipe_name + "\", " + topic_name + ", " + queue_size + ");\n"
        topic_list.append(topic_name.replace(".c_str()", "").replace(".c_str", ""))
        topic_no_list[member_name] = topic_no_format.format(ros_topic_count)
        ros_topic_count = ros_topic_count + 1
        return ros_topic_count, topic_no_list, topic_list, topic_datatype_list, line

    def _convertRosInit(self, line, cfe_apl_name):
        """Convert ros init.

        Args:
            line (str): Line of source
            cfe_apl_name (str): cFE app name

        Returns:
            tuple: (True, line)

        """
        head_space = self._getHeadSpace(line)
        line = head_space + "CONVERT_RosInit(EVT_COUNT, EventFilters" + cfe_apl_name + ");\n"
        return True, line

    def _convertRosTimeNow(self, line):
        """Convert ros time.

        Args:
            line (str): Line of source

        Returns:
            str: Converted line

        """
        line = line.replace("ros::Time::now()", "CONVERT_RosTimeNow()")
        return line

    def _convertRosParamGet(self, line, launch_dic):
        """Get convert ros parameter.

        Args:
            line (str): Line of source
            launch_dic (dict): Contents of launch file

        Raises:
            Exception: If KeyError occurs in launch_dict.

        Returns:
            str: Converted line

        """
        head_space = self._getHeadSpace(line)
        method, param_name, var_name = line.split("\"")
        try:
            val = launch_dic[param_name]["val"]
            if len(val) == 0:
                val = "\"\""
        except KeyError:
            raise Exception("Parameter<%s> not found from within launch file. Please check for other similar errors"
                            % param_name)
        arg_list = self._getArgument(line).split(",")
        member = arg_list[1]
        line = member + " = \"" + val + "\";\n"
        line = head_space + line
        return line

    def _convertRosPackageGetPath(self, line, cfe_apl_name):
        """Get convert rps package path.

        Args:
            line (str): Line of source
            cfe_apl_name (str): cFE app name

        Returns:
            str: Converted line

        """
        replace_line = "CONVERT_RosPackageGetPath(\"" + cfe_apl_name + "\")"
        line = re.sub(r'ros::package::getPath\(.+\)', replace_line, line)
        return line

    def _convertRosNodeHandleResolveName(self, line):
        """Convert ros node handle resolve name.

        Args:
            line (str): Line of source

        Returns:
            str: Converted line

        """
        head_space = self._getHeadSpace(line)
        var_name = line.split("=")[0]
        topic_name = line.split("\"")[1]
        if self.namespace is None:
            line = var_name + " = \"/" + topic_name + "\";\n"
        else:
            line = var_name + " = \"" + self.namespace + "/" + topic_name + "\";\n"
        line = head_space + line
        return line

    def _convertRosDuration(self, line):
        """Convert ros duration.

        Args:
            line (str): Line of source

        Returns:
            str: Converted line

        """
        line = line.replace("ros::Duration", "RosDuration")
        return line

    def _convertRosTime(self, line):
        """Convert ros time.

        Args:
            line (str): Line of source

        Returns:
            str: Converted line

        """
        line = line.replace("ros::Time", "RosTime")
        return line

    # --------------------------------->>
