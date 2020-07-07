"""
Copyright (c) 2020 Japan Aerospace Exploration Agency.
All Rights Reserved.

This file is covered by the LICENSE.txt in the root of this project.
"""

import glob
import re
# import os
import json

import file_operator
import relay_convert_utility


class RelayNodeCreator:
    """Create relay node."""

    # defines
    BASE_DIR = "./relay/relay_node/"
    MSG_DIR = ""
    CONV_INC_DIR = ""
    SERI_DIR = ""
    CONV_SRC_DIR = ""

    fop = None
    util = None

    def __init__(self):
        """Constructor."""

        self.fop = file_operator.FileOperator("./")
        self.util = relay_convert_utility.RelayConvertUtility()

        self.MSG_DIR = self.BASE_DIR + "msg/"
        self.CONV_INC_DIR = self.BASE_DIR + "include/convert/"
        self.SERI_DIR = self.BASE_DIR + "include/ros_serializer/"
        self.CONV_SRC_DIR = self.BASE_DIR + "src/convert/"

    def _convertCMakeListsTxt(self, msg_data_types):
        """Convert cmakelist.

        Args:
            msg_data_types (list): List of message types

        """        

        file_path = self.BASE_DIR + "CMakeLists.txt"

        msg_files_str = ""
        for msg in msg_data_types:
            msg_files_str = msg_files_str + self.util._addIndent(1) + msg + ".msg\n"

        dst_lines = []
        for line in open(file_path, "r"):
            line = line.replace("\n", "")
            line = line.replace("$$_MESSAGE_FILES_$$", msg_files_str)
            dst_lines.append(line)

        self.fop._writeFile(file_path, dst_lines)

    def _convertPublisherH(self, relay_setting_reader):
        """Convert publisher.h file.

        Args:
            relay_setting_reader (RelaySettingReader): RelaySettingReader object

        """

        file_path = self.CONV_INC_DIR + "publisher.h"
        cnt = 1

        dst_str = ""
        for correspond in relay_setting_reader.corresponds:
            if correspond.sender == correspond.SENDER_ROS:
                continue
            dst_str = dst_str + self.util._addIndent(1) + "ros::Publisher pub{:03d}_;\n".format(cnt)
            cnt = cnt + 1

        dst_lines = []
        for line in open(file_path, "r"):
            line = line.replace("\n", "")
            line = line.replace("$$_PUBLISHER_COUNT_$$", dst_str)
            dst_lines.append(line)
        self.fop._writeFile(file_path, dst_lines)

    # Postulate that ros_serializer is prepared
    def _convertPublisherCpp(self, relay_setting_reader):
        """Postulate that ros_serializer is prepared.

        Args:
            relay_setting_reader (RelaySettingReader): RelaySettingReader object

        """

        file_path = self.CONV_SRC_DIR + "publisher.cpp"
        cnt = 1

        # Create an include statement
        # Read all files from serializer directory
        ser_list = self.util._searchSerializer(self.SERI_DIR)
        include_list = []
        for ser_path in ser_list:
            # Remove path preceding ros_serializer
            units = ser_path.split("/")
            path = ""
            is_write = 0
            for unit in units:
                if unit == "ros_serializer":
                    is_write = 1
                if is_write == 1:
                    if len(path) > 0:
                        path = path + "/"
                    path = path + unit
            include_list.append("#include \"" + path + "\"")

        include_str = ""
        for include in include_list:
            include_str = include_str + include + "\n"

        # Create advertise
        advertise_str = ""
        cnt = 1
        for correspond in relay_setting_reader.corresponds:
            if correspond.sender == correspond.SENDER_ROS:
                continue
            advertise_str = advertise_str + self.util._addIndent(1) + \
                "this->pub{:03d}_ = this->nh_.advertise<{}>(\"{}\", 100);\n"\
                .format(cnt, correspond.ros_data_type, correspond.topic)
            cnt = cnt + 1

        # Create publish
        publish_list = []
        cnt = 1
        for correspond in relay_setting_reader.corresponds:
            if correspond.sender == correspond.SENDER_ROS:
                continue
            # data_type = re.sub(r'relay_node::', "", correspond.ros_data_type)
            data_type = correspond.ros_data_type
            if cnt == 1:
                publish_list.append(self.util._addIndent(1) + "if (msg_id == " + correspond.msg_id + ") {")
            else:
                publish_list.append(self.util._addIndent(1) + "else if (msg_id == " + correspond.msg_id + ") {")
            publish_list.append(self.util._addIndent(2) + "ros_serializer::" + data_type + " serializer;")
            publish_list.append(self.util._addIndent(2) + correspond.ros_data_type + " msg;")
            publish_list.append(self.util._addIndent(2) + "msg = serializer.deserialize(body.data, length);")
            publish_list.append(self.util._addIndent(2) + "this->pub{:03d}_.publish(msg);".format(cnt))
            publish_list.append(self.util._addIndent(1) + "}")
            cnt = cnt + 1
        if cnt > 1:
            publish_list.append(self.util._addIndent(1) + "else {")
            publish_list.append(self.util._addIndent(2) + "ROS_ERROR(\"Not Compatible MsgId(%04x).\", msg_id);")
            publish_list.append(self.util._addIndent(1) + "}")

        dst_lines = []
        for line in open(file_path, "r"):
            line = line.replace("\n", "")
            line = line.replace("$$_PUB_INCLUDE_$$", include_str)
            line = line.replace("$$_ADVERTISE_$$", advertise_str)
            if line.find("$$_PUBLISH_$$") > -1:
                line = ""
                for publish in publish_list:
                    line = line + publish + "\n"
            dst_lines.append(line)

        self.fop._writeFile(file_path, dst_lines)

    def _convertSubscriberH(self, relay_setting_reader):
        """Convert subscriber.h file.

        Args:
            relay_setting_reader (RelaySettingReader): RelaySettingReader object

        """

        file_path = self.CONV_INC_DIR + "subscriber.h"

        # Create an include statement
        # Read all files from serializer directory
        ser_list = self.util._searchSerializer(self.SERI_DIR)
        include_list = []
        for ser_path in ser_list:
            # Remove path preceding ros_serializer
            units = ser_path.split("/")
            path = ""
            is_write = 0
            for unit in units:
                if unit == "ros_serializer":
                    is_write = 1
                if is_write == 1:
                    if len(path) > 0:
                        path = path + "/"
                    path = path + unit
            include_list.append("#include \"" + path + "\"")

        include_str = ""
        for include in include_list:
            include_str = include_str + include + "\n"

        callback_str = ""
        for correspond in relay_setting_reader.corresponds:
            if correspond.sender == correspond.SENDER_CFE:
                continue
            callback_str = callback_str + self.util._addIndent(1) + "void " + \
                correspond.topic.lower().replace("/", "_") + \
                "_callback(" + correspond.ros_data_type + "::ConstPtr msg);\n"

        cnt = 1
        dst_str = ""
        for correspond in relay_setting_reader.corresponds:
            if correspond.sender == correspond.SENDER_CFE:
                continue
            dst_str = dst_str + self.util._addIndent(1) + "ros::Subscriber sub{:03d}_;\n".format(cnt)
            cnt = cnt + 1

        dst_lines = []
        for line in open(file_path, "r"):
            line = line.replace("\n", "")
            line = line.replace("$$_SUB_INCLUDE_$$", include_str)
            line = line.replace("$$_SUBSCRIBE_CALLBACK_$$", callback_str)
            line = line.replace("$$_SUBSCRIBER_COUNT_$$", dst_str)
            dst_lines.append(line)

        self.fop._writeFile(file_path, dst_lines)

    def _convertSubscriberCpp(self, relay_setting_reader):
        """Convert subscriber.cpp file.

        Args:
            relay_setting_reader (RelaySettingReader): RelaySettingReader object

        """

        file_path = self.CONV_SRC_DIR + "subscriber.cpp"

        callback_list = []
        callback_methods = []
        for correspond in relay_setting_reader.corresponds:
            if correspond.sender == correspond.SENDER_CFE:
                continue
            method = correspond.topic.lower().replace("/", "_") + "_callback"
            callback_methods.append(method)
            callback_list.append("void Subscriber::" + method + "(" + correspond.ros_data_type + "::ConstPtr msg) {")
            callback_list.append(self.util._addIndent(1) + "int header_size = 0;")
            callback_list.append(self.util._addIndent(1) + "int result = 0;")
            callback_list.append(self.util._addIndent(1) + "RosCfeNwBridgeHeader header;")
            # data_type = re.sub(r'relay_node::', "", correspond.ros_data_type)
            data_type = correspond.ros_data_type
            callback_list.append(self.util._addIndent(1) + "ros_serializer::" + data_type + " serializer;")
            callback_list.append(self.util._addIndent(1) +
                                 "uint32_t body_size = serializer.serialize(*msg, this->body_data_);")
            callback_list.append(self.util._addIndent(1) +
                                 "header = this->input_header_data(body_size, " + correspond.msg_id + ");")
            callback_list.append(self.util._addIndent(1) +
                                 "header_size = serialize_ros_cfe_header(header, this->header_data_);")
            callback_list.append(self.util._addIndent(1) + "char ret = 0;")
            callback_list.append(self.util._addIndent(1) + "this->comm_->write(header_size, this->header_data_, &ret);")
            callback_list.append(self.util._addIndent(1) + "if (ret < 0) {")
            callback_list.append(self.util._addIndent(2) + "ROS_ERROR(\"write header is failed in " + method + ".\");")
            callback_list.append(self.util._addIndent(2) + "return;")
            callback_list.append(self.util._addIndent(1) + "}")
            callback_list.append(self.util._addIndent(1) + "this->comm_->write(body_size, this->body_data_, &ret);")
            callback_list.append(self.util._addIndent(1) + "if (ret < 0) {")
            callback_list.append(self.util._addIndent(2) + "ROS_ERROR(\"write body is failed in " + method + ".\");")
            callback_list.append(self.util._addIndent(2) + "return;")
            callback_list.append(self.util._addIndent(1) + "}")
            callback_list.append("}")
        callback_str = ""
        for callback in callback_list:
            callback_str = callback_str + callback + "\n"

        cnt = 1
        subscribe_str = ""
        for correspond in relay_setting_reader.corresponds:
            if correspond.sender == correspond.SENDER_CFE:
                continue
            method = correspond.topic.lower().replace("/", "_") + "_callback"
            subscribe_str = subscribe_str + self.util._addIndent(1) + \
                "this->sub{:03d}_".format(cnt) + " = this->nh_.subscribe(\"" + \
                correspond.topic + "\", 100, &Subscriber::" + method + ", this);\n"
            cnt = cnt + 1

        dst_lines = []
        for line in open(file_path, "r"):
            line = line.replace("\n", "")
            line = line.replace("$$_SUB_CALLBACK_$$", callback_str)
            line = line.replace("$$_SUBSCRIBE_$$", subscribe_str)
            dst_lines.append(line)

        self.fop._writeFile(file_path, dst_lines)

    def _createSerializers(self):
        """Create serialiser."""

        # Read all files from msg directory
        msg_list = glob.glob(self.MSG_DIR + "*.msg")

        msg_dict_list = []
        for msg in msg_list:
            msg_dict_list.append({"path": msg, "pkg_name": "relay_node"})

        # Get and add msg file created from work file
        file_path = self.fop._getMsgListFilePath()
        f = open(file_path, "r")
        files = json.load(f)
        f.close()
        for data_type, path in files.items():
            pkg_name = re.sub(r'::.+', "", data_type)
            msg_dict_list.append({"path": path, "pkg_name": pkg_name})

        # Create a list of msg data types
        msg_data_types = []
        for msg in msg_list:
            msg_type = self.util._getMsgFileName(msg)
            msg_data_types.append(msg_type)

        # Now that the message list has been created, convert CMakeLists.txt here
        self._convertCMakeListsTxt(msg_data_types)

        # Create Serializer for each message type
        for msg_data in msg_dict_list:
            file_path = msg_data["path"]
            pkg_name = msg_data["pkg_name"]
            self._createSerializer(file_path, msg_data_types, pkg_name)

    def _createSerializer(self, file_path, msg_data_types, pkg_name):
        """Create serializer.

        Args:
            file_path (str): File path
            msg_data_types (list): List of message types
            pkg_name (str): Package name

        """

        # Contents of serialize function
        serializer_lines = []
        # Contents of deserialize function
        deserializer_lines = []
        # Include other msg data types
        include_lines = []

        msg_type = self.util._getMsgFileName(file_path)

        lines = open(file_path, "r")
        prev_lines = []
        # Preprocessing(Remove comments and more)
        for line in lines:
            line = re.sub(r'//.*', "", line)
            line = line.strip()
            prev_lines.append(line)
        lines.close()

        # Add serialize and deserialize processing according to the data type
        for line in prev_lines:
            # Delete comment
            line = re.sub(r'#.+', "", line)
            # Skip if empty character
            if len(line) == 0:
                continue
            # Skip if find of equal
            if line.find("=") != -1:
                continue

            is_array = 0
            if line.find("[") > -1 and line.find("]") > -1:
                is_array = 1
                line = re.sub(r'\[.*\]', "", line)
            units = line.split()
            data_type = units[0]
            member = units[1]
            # uint8[12] TlmHeade
            # Case of  basic data types
            if data_type in self.util.basic_data_types:
                # Check if it is an array
                # Convert data types
                converted_data_type = self.util.convert_data_types[data_type]
                # Prepare Serialize and Deserialize
                under_bar_data_type = converted_data_type.replace(" ", "_")

                if is_array == 0:
                    # Pattern of not array
                    # Serializer
                    ser_line = self.util._addIndent(3) + "len += serialize_" + under_bar_data_type + \
                        "(send_data." + member + ", buffer + len);"
                    serializer_lines.append(ser_line)
                    # Deserializer
                    des_line = self.util._addIndent(3) + "msg." + member + " = deserialize_" + under_bar_data_type + \
                        "(buffer + length, len);"
                    deserializer_lines.append(des_line)
                else:
                    # Pattern of array
                    # Serializer
                    ser_line = self.util._addIndent(3) + "std::vector<" + converted_data_type + "> " + member + ";\n"\
                        + self.util._addIndent(3) + "for (size_t ii = 0; ii < send_data." \
                        + member + ".size(); ii++) {\n"\
                        + self.util._addIndent(4) + member + ".push_back(send_data." + member + "[ii]);\n" \
                        + self.util._addIndent(3) + "}\n"\
                        + self.util._addIndent(3) + "len += serialize_array_" \
                        + under_bar_data_type + "(" + member + ", buffer + len);\n"
                    serializer_lines.append(ser_line)
                    # Deserializer
                    des_line = self.util._addIndent(3) + "std::vector<" + converted_data_type + "> " \
                        + member + " = deserialize_array_" + under_bar_data_type + "(buffer + length, len);\n" \
                        + self.util._addIndent(3) + "for (size_t ii = 0; ii < " + member + ".size(); ii++) {\n" \
                        + self.util._addIndent(4) + "msg." + member + "[ii] = " + member + "[ii];\n" \
                        + self.util._addIndent(3) + "}\n"
                    deserializer_lines.append(des_line)
            # Case of other structures
            else:
                if data_type in ["Point", "Pose", "PoseStamped", "PoseWithCovariance", "Quaternion", "Twist",
                                 "TwistWithCovariance", "Vector3", "Wrench", "WrenchStamped"]:
                    include_lines.append("#include \"ros_serializer/geometry_msgs_" +
                                         data_type.lower() + "_serializer.h\"")
                elif data_type in ["Odometry"]:
                    include_lines.append("#include \"ros_serializer/nav_msgs_" + data_type.lower() + "_serializer.h\"")
                elif data_type in ["JointState"]:
                    include_lines.append("#include \"ros_serializer/sensor_msgs_" +
                                         data_type.lower() + "_serializer.h\"")
                elif data_type in ["Float64MultiArray", "Header", "RosTime", "MultiArrayDimension", "MultiArrayLayout"]:
                    include_lines.append("#include \"ros_serializer/std_msgs_" + data_type.lower() + "_serializer.h\"")
                    data_type = "std_msgs::" + data_type
                elif data_type in msg_data_types:
                    # Case of other data types of relay_node
                    include_lines.append("#include \"ros_serializer/" + data_type.lower() + "_serializer.h\"")
                    data_type = "relay_node::" + data_type
                # Serializer
                ser_line = self.util._addIndent(3) + "ros_serializer::" + data_type + " " + member + ";\n" \
                    + self.util._addIndent(3) + "len += " + member + ".serialize(send_data." \
                    + member + ", buffer + len);\n"
                serializer_lines.append(ser_line)
                # Deserializer
                des_line = self.util._addIndent(3) + "ros_serializer::" + data_type + " " + member + ";\n"
                des_line = des_line + self.util._addIndent(3) + "msg." + member + " = " + member \
                    + ".deserialize(buffer + length, len);"
                deserializer_lines.append(des_line)

        dst_lines = self._createFileLines(serializer_lines, deserializer_lines, include_lines, msg_type, pkg_name)

        dir_path = self.SERI_DIR
        ser_file_path = self.SERI_DIR + msg_type.lower() + "_serializer.h"
        self.fop._writeFile(ser_file_path, dst_lines)

    def _createFileLines(self, serializer_lines, deserializer_lines, include_lines, msg_type, pkg_name):
        """Create list of file line.

        Args:
            serializer_lines (list): List of line which serializing method is applied
            deserializer_lines (list): List of line which deserializing method is applied
            include_lines (list): List of include-lines
            msg_type (str): Message type
            pkg_name (str): Package name

        Returns:
            list: List of converted lines

        """        

        dst_lines = []
        dst_lines.append("#ifndef _" + msg_type.lower() + "_serializer_h_")
        dst_lines.append("#define _" + msg_type.lower() + "_serializer_h_")
        dst_lines.append("")
        dst_lines.append("#include \"" + pkg_name + "/" + msg_type + ".h\"")
        dst_lines.append("")

        dst_lines.append("#include <vector>")
        dst_lines.append("#include \"relay_node/util.h\"")
        dst_lines.append("")

        for include in include_lines:
            dst_lines.append(include)
        dst_lines.append("")
        dst_lines.append("namespace ros_serializer {")
        dst_lines.append("namespace " + pkg_name + " {")
        dst_lines.append(self.util._addIndent(1) + "class " + msg_type + "{")
        dst_lines.append(self.util._addIndent(1) + "public:")
        # Serialize method
        dst_lines.append(self.util._addIndent(2) + "int serialize(const ::" + pkg_name + "::"
                         + msg_type + " &send_data, unsigned char *buffer) {")
        dst_lines.append(self.util._addIndent(3) + "unsigned int len = 0;")
        dst_lines.append("")
        for ser in serializer_lines:
            dst_lines.append(ser)
            dst_lines.append("")
        dst_lines.append(self.util._addIndent(3) + "return len;")
        dst_lines.append(self.util._addIndent(2) + "}")
        dst_lines.append("")
        # Deserialize method
        dst_lines.append(self.util._addIndent(2) + "::" + pkg_name + "::" + msg_type
                         + " deserialize(const unsigned char *buffer, unsigned int &length) {")
        dst_lines.append(self.util._addIndent(3) + "::" + pkg_name + "::" + msg_type + " msg;")
        dst_lines.append(self.util._addIndent(3) + "unsigned int ii;")
        dst_lines.append(self.util._addIndent(3) + "unsigned int len;")
        dst_lines.append(self.util._addIndent(3) + "length = 0;")
        dst_lines.append("")
        for des in deserializer_lines:
            dst_lines.append(des)
            dst_lines.append(self.util._addIndent(3) + "length += len;")
            dst_lines.append("")
        dst_lines.append(self.util._addIndent(3) + "return msg;")
        dst_lines.append(self.util._addIndent(2) + "}")
        dst_lines.append(self.util._addIndent(1) + "};")
        dst_lines.append("};")
        dst_lines.append("};")
        dst_lines.append("")

        dst_lines.append("#endif // _" + msg_type.lower() + "_serializer_h_")
        dst_lines.append("")

        return dst_lines
