"""
Copyright (c) 2020 Japan Aerospace Exploration Agency.
All Rights Reserved.

This file is covered by the LICENSE.txt in the root of this project.
"""

import re
import glob
import json

import file_operator
import relay_convert_utility


class RelayAppCreator:
    """Create cpp file."""

    # defines
    BASE_DIR = "./relay/relay_app/fsw/"
    SRC_DIR = ""
    CONVERT_DIR = ""
    BUILD_DIR = ""
    SERI_DIR = ""

    fop = None
    util = None
    relay_setting_reader = None

    def __init__(self, relay_setting_reader):
        """Constructor.
        
        Args:
            relay_setting_reader (RelaySettingReader): RelaySettingReader object

        """        
        self.fop = file_operator.FileOperator("./")
        self.util = relay_convert_utility.RelayConvertUtility()

        self.SRC_DIR = self.BASE_DIR + "src/"
        self.CONVERT_DIR = self.SRC_DIR + "convert/"
        self.BUILD_DIR = self.BASE_DIR + "for_build/"
        self.SERI_DIR = self.SRC_DIR + "cfs_serializer/"

        self.relay_setting_reader = relay_setting_reader

    def _convertCommunicationH(self):
        """ Convert Communication.h file."""
        
        file_path = self.SRC_DIR + "communication/communication.h"
        dst_lines = []
        for line in open(file_path, "r"):
            line = line.replace("$$_COM_IP_ADDR_$$", "\"" + self.relay_setting_reader.cfe_setting.ip_addr + "\"")
            line = line.replace("$$_PORT_$$", self.relay_setting_reader.cfe_setting.port_send)
            line = line.replace("$$_PORT_SEND_$$", self.relay_setting_reader.cfe_setting.port_send)
            line = line.replace("$$_PORT_RECV_$$", self.relay_setting_reader.cfe_setting.port_recv)
            dst_lines.append(line.replace("\n", ""))
        self.fop._writeFile(file_path, dst_lines)

    def _convertPublisherCpp(self):
        """Convert publisher.cpp file."""
        
        file_path = self.CONVERT_DIR + "publisher.cpp"
        # Create include function
        # Read all files from serializer directory
        ser_list = self.util._searchSerializer(self.SERI_DIR)
        include_list = []
        for ser_path in ser_list:
            # Get only files ending in .h
            if re.match(r".+\.cpp", ser_path):
                continue
            # Remove path preceding cfs_serializer
            units = ser_path.split("/")
            path = ""
            is_write = 0
            for unit in units:
                if unit == "cfs_serializer":
                    is_write = 1
                if is_write == 1:
                    if len(path) > 0:
                        path = path + "/"
                    path = path + unit
            include_list.append("#include \"" + path + "\"")
        include_str = ""
        for include in include_list:
            include_str = include_str + include + "\n"

        # Issue a Topic number
        topic_no_str = ""
        cnt = 1
        for correspond in self.relay_setting_reader.corresponds:
            if correspond.sender == correspond.SENDER_CFE:
                continue
            topic_no_str = topic_no_str + "int relay_pub_topic_no{:03d};\n".format(cnt)
            cnt = cnt + 1

        # Create Advertise
        advertise_str = ""
        cnt = 1
        for correspond in self.relay_setting_reader.corresponds:
            if correspond.sender == correspond.SENDER_CFE:
                continue
            advertise_str = advertise_str + self.util._addIndent(1) + "relay_pub_topic_no{:03d}".format(cnt) + \
                " = CONVERT_RosNodeHandleAdvertise(\"relay_pub_{:05x}\", {}, 100);\n".format(cnt, correspond.msg_id)
            cnt = cnt + 1

        # Judgment of Publish
        publish_str = ""
        cnt = 1
        for correspond in self.relay_setting_reader.corresponds:
            if correspond.sender == correspond.SENDER_CFE:
                continue
            if cnt == 1:
                publish_str = publish_str + self.util._addIndent(1) + "if (msg_id == " + correspond.msg_id + ") {\n"
            else:
                publish_str = publish_str + self.util._addIndent(1) + "else if (msg_id == " + \
                    correspond.msg_id + ") {\n"
            publish_str = publish_str + self.util._addIndent(2) + correspond.cfe_data_type + " msg = deserialize_" + \
                correspond.cfe_data_type.lower().replace("::", "_") + "(body.data, length);\n"
            if not re.match("^CFE_SB_", correspond.cfe_data_type) and not re.match("^CCSDS_", correspond.cfe_data_type):
                publish_str = publish_str + self.util._addIndent(2) + "msg.vector2pointer();\n"
                publish_str = publish_str + self.util._addIndent(2) + "msg.string2pointer();\n"
            publish_str = publish_str + self.util._addIndent(2) + \
                "CONVERT_RosPublisherPublish((CFE_SB_Msg_t*)(&msg), sizeof(" + \
                correspond.cfe_data_type + "), relay_pub_topic_no{:03d});\n".format(cnt)
            publish_str = publish_str + self.util._addIndent(1) + "}\n"
            cnt = cnt + 1
        if cnt > 1:
            publish_str = publish_str + self.util._addIndent(1) + "else {\n"
            publish_str = publish_str + self.util._addIndent(2) + \
                "OS_printf(\"[publisher_publish] Not Compatible MsgId(%04x). \\n\", msg_id);\n"
            publish_str = publish_str + self.util._addIndent(1) + "}\n"

        # Substitution of file contents
        dst_lines = []
        for line in open(file_path, "r"):
            line = line.replace("\n", "")
            line = line.replace("$$_PUB_INCLUDE_$$", include_str)
            line = line.replace("$$_TOPIC_NO_DEFINE_$$", topic_no_str)
            line = line.replace("$$_ADVERTISE_$$", advertise_str)
            line = line.replace("$$_PUBLISH_$$", publish_str)
            dst_lines.append(line)

        self.fop._writeFile(file_path, dst_lines)

    def _convertSubscriberCpp(self):
        """Convert subscriber.cpp file."""        
        file_path = self.CONVERT_DIR + "subscriber.cpp"
        # include
        # Read all files from serializer directory
        ser_list = self.util._searchSerializer(self.SERI_DIR)
        include_list = []
        for ser_path in ser_list:
            # Get only files ending in .h
            if re.match(r".+\.cpp", ser_path):
                continue
            # Remove path preceding cfs_serializer
            units = ser_path.split("/")
            path = ""
            is_write = 0
            for unit in units:
                if unit == "cfs_serializer":
                    is_write = 1
                if is_write == 1:
                    if len(path) > 0:
                        path = path + "/"
                    path = path + unit
            include_list.append("#include \"" + path + "\"")
        include_str = ""
        for include in include_list:
            include_str = include_str + include + "\n"

        # callback
        callback_str = ""
        for correspond in self.relay_setting_reader.corresponds:
            if correspond.sender == correspond.SENDER_ROS:
                continue
            callback_str = callback_str + "void " + correspond.topic.lower().replace("/", "_") + \
                "_callback(void* msg) {\n"
            callback_str = callback_str + self.util._addIndent(1) + correspond.cfe_data_type + "* data = (" + \
                correspond.cfe_data_type + "*)msg;\n"
            if not re.match("^CFE_SB_", correspond.cfe_data_type) and not re.match("^CCSDS_", correspond.cfe_data_type):
                callback_str = callback_str + self.util._addIndent(1) + "data->pointer2vector();\n"
                callback_str = callback_str + self.util._addIndent(1) + "data->pointer2string();\n"
            callback_str = callback_str + self.util._addIndent(1) + "int header_size = 0;\n"
            callback_str = callback_str + self.util._addIndent(1) + "int body_size = 0;\n"
            callback_str = callback_str + self.util._addIndent(1) + "RosCfeNwBridgeHeader header;\n"
            callback_str = callback_str + "\n"
            callback_str = callback_str + self.util._addIndent(1) + \
                "body_size = serialize_" + correspond.cfe_data_type.lower().replace("::", "_") + "(*data, body_data);\n"
            callback_str = callback_str + "\n"
            callback_str = callback_str + self.util._addIndent(1) + \
                "header = input_header_data(body_size, " + correspond.msg_id + ");\n"
            callback_str = callback_str + self.util._addIndent(1) + \
                "header_size = serialize_ros_cfe_header(header, header_data);\n"
            callback_str = callback_str + "\n"
            callback_str = callback_str + self.util._addIndent(1) + "char ret = 0;\n"
            callback_str = callback_str + self.util._addIndent(1) + \
                "communication_write(header_size, header_data, &ret);\n"
            callback_str = callback_str + self.util._addIndent(1) + "if (ret < 0) {\n"
            callback_str = callback_str + self.util._addIndent(2) + \
                "OS_printf(\"write header is failed in " + \
                correspond.topic.lower().replace("/", "_") + "_callback" + ". \\n\");\n"
            callback_str = callback_str + self.util._addIndent(2) + "return;\n"
            callback_str = callback_str + self.util._addIndent(1) + "}\n"
            callback_str = callback_str + "\n"
            callback_str = callback_str + self.util._addIndent(1) + "communication_write(body_size, body_data, &ret);\n"
            callback_str = callback_str + self.util._addIndent(1) + "if (ret < 0) {\n"
            callback_str = callback_str + self.util._addIndent(2) + \
                "OS_printf(\"write body is failed in " + \
                correspond.topic.lower().replace("/", "_") + "_callback" + ". \\n\");\n"
            callback_str = callback_str + self.util._addIndent(2) + "return;\n"
            callback_str = callback_str + self.util._addIndent(1) + "}\n"
            callback_str = callback_str + self.util._addIndent(1) + "data->deleteData();\n"
            callback_str = callback_str + "}\n"

        # subscribe
        subscribe_str = ""
        cnt = 1
        for correspond in self.relay_setting_reader.corresponds:
            if correspond.sender == correspond.SENDER_ROS:
                continue
            subscribe_str = subscribe_str + self.util._addIndent(1) + \
                "CONVERT_RosNodeHandleSubscribe(\"relay_sub_{:05x}\", ".format(cnt) + correspond.msg_id + \
                ", 100, &" + correspond.topic.lower().replace("/", "_") + \
                "_callback, sizeof(" + correspond.cfe_data_type + "));\n"
            cnt = cnt + 1

        # Substitution of file contents
        prev_lines = []
        for line in open(file_path, "r"):
            prev_lines.append(line.replace("\n", ""))
        dst_lines = []
        for line in prev_lines:
            line = line.replace("$$_SUB_INCLUDE_$$", include_str)
            line = line.replace("$$_CALLBACK_$$", callback_str)
            line = line.replace("$$_SUBSCRIBE_$$", subscribe_str)
            dst_lines.append(line)

        self.fop._writeFile(file_path, dst_lines)

    def _convertDefineH(self):
        """Convert Define.h file."""        

        file_path = self.SRC_DIR + "define.h"
        dst_lines = []
        for line in open(file_path, "r"):
            line = line.replace("\n", "")
            line = line.replace("$$_RING_BUF_LEN_$$", self.relay_setting_reader.cfe_setting.ring_buf_len)
            dst_lines.append(line)
        self.fop._writeFile(file_path, dst_lines)

    def _convertMakefile(self):
        """Convert makefile."""
        
        file_path = self.BUILD_DIR + "Makefile"
        # Read all files from serializer directory
        ser_list = self.util._searchSerializer(self.SERI_DIR)
        src_list = []
        for ser_path in ser_list:
            # Get only files ending in .cpp
            if re.match(r".+\.h", ser_path):
                continue
            # Remove path preceding cfs_serializer
            units = ser_path.split("/")
            src_list.append(units[len(units) - 1])
        o_str = ""
        build_str = ""
        for src in src_list:
            if len(o_str) > 0:
                o_str = o_str + "\n"
            o_str = o_str + "{} \\".format(src.replace(".cpp", ".o"))
            build_str = build_str + "\t$(COMPILER) $(LOCAL_COPTS) -std=c++11 -m32 -Wall $(INCLUDE_PATH) -g -O0 "\
                                    "-DOS_DEBUG_LEVEL=$(DEBUG_LEVEL) -c -o {} $(MYDIR)/cfs_serializer/{} \n"\
                                    .format(src.replace(".cpp", ".o"), src)

        dst_lines = []
        for line in open(file_path, "r"):
            line = line.replace("\n", "")
            line = line.replace("$$_MAKEFILE_OBJ_$$", o_str)
            line = line.replace("$$_SRC_BUILD_$$", build_str)
            dst_lines.append(line)
        self.fop._writeFile(file_path, dst_lines)

    def _createSerializers(self, ros_msg_dir_path, cfe_ser_dir_path):
        """Create Data type. Read all files from msg directory of relay node on ROS side.
        
        Args:
            ros_msg_dir_path (str): Path to the directory containing message files
            cfe_ser_dir_path (str): Path to the 'cfs_serializer' directory

        """

        msg_list = glob.glob(ros_msg_dir_path + "*.msg")

        msg_dict_list = []
        for msg in msg_list:
            msg_dict_list.append({"path": msg, "pkg_name": "relay_app"})

        # Get and add msg file created from work file
        other_msg_list = []
        file_path = self.fop._getMsgListFilePath()
        f = open(file_path, "r")
        files = json.load(f)
        f.close()
        for data_type, path in files.items():
            pkg_name = re.sub(r'::.+', "", data_type)
            other_msg_list.append(data_type)
            msg_dict_list.append({"path": str(path), "pkg_name": pkg_name})

        for msg in msg_list:
            # Become only name of type
            msg_type = self.util._getMsgFileName(msg)
            other_msg_list.append(msg_type)

        # Read cFE data type and corresponding ROS data type
        for msg_data in msg_dict_list:
            file_path = msg_data["path"]
            pkg_name = msg_data["pkg_name"]
            # Become only name of type
            msg_type = self.util._getMsgFileName(file_path)
            if pkg_name == "relay_app":
                # Use the data type name +_t of the msg file as the type name
                cfe_data_type = msg_type + "_t"
            else:
                cfe_data_type = msg_type
            self._createSerializer(file_path, cfe_data_type, other_msg_list, pkg_name)

    # msg file path
    # cfe_data_type
    def _createSerializer(self, msg_file_path, cfe_data_type, other_msg_list, pkg_name):
        """Create serialize list.
        
        Args:
            msg_file_path (str): Path to the message file
            cfe_data_type (str): Data type in cFE
            other_msg_list (list): List of unsupported data type
            pkg_name (str): Package name

        """

        # serialize func
        serializer_lines = []
        serializer_method = ""
        # deserialize func
        deserializer_lines = []
        deserializer_method = ""
        # Include other msg data types
        include_list = []
        # Create struct
        struct_list = []

        lines = open(msg_file_path, "r")
        prev_lines = []
        const_list = []
        # Preprocessing(Remove comments and more)
        for line in lines:
            # Delete comment
            line = re.sub(r'#.+', "", line)
            line = line.strip()
            # Skip if empty character
            if len(line) == 0:
                continue
            # Skip if find of equal
            if line.find("=") != -1:
                const_list.append(line)
                continue

            prev_lines.append(line)
        lines.close()

        # Create a data type definition
        # However, the processing is skipped because "CFE_SB_" (cFE software bus data type) and
        # "CCSDS_" (data type defined by the Consultative Committee for Space Data System) may
        # conflict with cFE internal data types
        if not re.match("^CFE_SB_", cfe_data_type) and not re.match("^CCSDS_", cfe_data_type):
            for line in prev_lines:
                array_idx = ""
                if line.find("[") > -1:
                    start = line.find("[")
                    end = line.find("]")
                    array_idx = line[start:end + 1]
                line = re.sub(r'\[.*\]', "", line)

                units = line.split()
                data_type = units[0]
                member = units[1]
                if data_type in self.util.basic_data_types:
                    data_type = self.util.convert_data_types[data_type]
                if len(array_idx) > 0:
                    member = member + array_idx
                # If data_type is **, you need to add a namespace
                struct_data_type = data_type
                if data_type in ["Point", "Pose", "PoseStamped", "PoseWithCovariance", "Quaternion", "Twist",
                                 "TwistWithCovariance", "Vector3", "Wrench", "WrenchStamped"]:
                    struct_data_type = "geometry_msgs::" + struct_data_type
                elif data_type in ["Odometry"]:
                    struct_data_type = "nav_msgs::" + struct_data_type
                elif data_type in ["JointState", "Image"]:
                    struct_data_type = "sensor_msgs::" + struct_data_type
                elif data_type in ["Float64MultiArray", "Header", "RosTime", "MultiArrayDimension", "MultiArrayLayout",
                                   "RosDuration", "String", "Empty"]:
                    struct_data_type = "std_msgs::" + struct_data_type
                struct_list.append(self.util._addIndent(1) + "{} {};".format(struct_data_type, member))

        # Create an include statement by looking at the contents of the msg file
        for line in prev_lines:
            units = line.split()
            data_type = units[0]
            member = units[1]
            # case of other structures
            if data_type in ["Point", "Pose", "PoseStamped", "PoseWithCovariance", "Quaternion", "Twist",
                             "TwistWithCovariance", "Vector3", "Wrench", "WrenchStamped"]:
                include_list.append("#include \"cfs_serializer/geometry_msgs_" + data_type.lower() + "_serializer.h\"")
            elif data_type in ["Odometry"]:
                include_list.append("#include \"cfs_serializer/nav_msgs_" + data_type.lower() + "_serializer.h\"")
            elif data_type in ["JointState", "Image"]:
                include_list.append("#include \"cfs_serializer/sensor_msgs_" + data_type.lower() + "_serializer.h\"")
            elif data_type in ["Float64MultiArray", "Header", "RosTime", "MultiArrayDimension", "MultiArrayLayout",
                               "RosDuration", "String", "Empty"]:
                include_list.append("#include \"cfs_serializer/std_msgs_" + data_type.lower() + "_serializer.h\"")
            elif data_type in other_msg_list:
                # case of  other data types of elay_node
                include_list.append("#include \"cfs_serializer/" + data_type.lower() + "_t_serializer.h\"")

        # Create serializer, deserializer
        method_pkg_name = ""
        if pkg_name != "relay_app":
            method_pkg_name = pkg_name + "_"
        serializer_method = "serialize_" + method_pkg_name + cfe_data_type.lower()
        deserializer_method = "deserialize_" + method_pkg_name + cfe_data_type.lower()

        self._createSerializerHeader(cfe_data_type, include_list, struct_list, serializer_method, deserializer_method,
                                     pkg_name, const_list)

        # Create function of contents
        for line in prev_lines:
            # Check if it is an array
            is_array = 0
            if line.find("[") > -1 and line.find("]") > -1:
                is_array = 1
                line = re.sub(r'\[.*\]', "", line)
            # Check the data type and create an operation
            units = line.split()
            data_type = units[0]
            member = units[1]
            # case of  basic data
            if data_type in self.util.basic_data_types:
                # Convert data types
                converted_data_type = self.util.convert_data_types[data_type]
                # Prepare Serialize, Deserialize
                under_bar_data_type = converted_data_type.replace(" ", "_")
                if is_array == 0:
                    # Case of not array
                    # Serializer
                    ser_line = self.util._addIndent(1) + "len += serialize_" + under_bar_data_type + "(send_data." + \
                        member + ", buffer + len);"
                    serializer_lines.append(ser_line)
                    # Deserializer
                    des_line = self.util._addIndent(1) + "msg." + member + " = deserialize_" + under_bar_data_type + \
                        "(buffer + length, len);"
                    deserializer_lines.append(des_line)
                    des_line = self.util._addIndent(1) + "length += len;"
                    deserializer_lines.append(des_line)
                else:
                    # Case of array
                    # Serializer
                    ser_line = self.util._addIndent(1) + "std::vector<" + converted_data_type + "> " + member + \
                        "(send_data." + member + ", std::end(send_data." + member + "));\n"
                    serializer_lines.append(ser_line)

                    ser_line = self.util._addIndent(1) + "len += serialize_array_" + under_bar_data_type + \
                        "(" + member + ", buffer + len);"
                    serializer_lines.append(ser_line)

                    # Deserializer
                    des_line = self.util._addIndent(1) + "std::vector<" + converted_data_type + "> " + member + ";"
                    deserializer_lines.append(des_line)
                    des_line = self.util._addIndent(1) + member + " = deserialize_array_" + \
                        under_bar_data_type + "(buffer + length, len);"
                    deserializer_lines.append(des_line)
                    des_line = self.util._addIndent(1) + "for (ii = 0; ii < " + member + ".size(); ii++) {"
                    deserializer_lines.append(des_line)
                    des_line = self.util._addIndent(2) + "msg." + member + "[ii] = " + member + "[ii];"
                    deserializer_lines.append(des_line)
                    des_line = self.util._addIndent(1) + "}"
                    deserializer_lines.append(des_line)
                    des_line = self.util._addIndent(1) + "length += len;"
                    deserializer_lines.append(des_line)

            # Case of other structures
            else:
                pkg_data_type = data_type.lower()
                # Serializer
                if pkg_name != "relay_app":
                    pkg_data_type = self._get_datatype_name_add_pkg(data_type)
                    ser_line = self.util._addIndent(1) + "len += serialize_" + \
                        pkg_data_type + "(send_data." + member + ", buffer + len);"
                else:
                    ser_line = self.util._addIndent(1) + "len += serialize_" + \
                        data_type.lower() + "_t(send_data." + member + ", buffer + len);"
                serializer_lines.append(ser_line)
                # Deserializer
                if pkg_name != "relay_app":
                    pkg_data_type = self._get_datatype_name_add_pkg(data_type)
                    des_line = self.util._addIndent(1) + "msg." + member + " = deserialize_" + \
                        pkg_data_type + "(buffer + length, len);\n" \
                        + self.util._addIndent(1) + "length += len;"
                else:
                    des_line = self.util._addIndent(1) + "msg." + member + " = deserialize_" + \
                        data_type.lower() + "_t(buffer + length, len);\n" \
                        + self.util._addIndent(1) + "length += len;"
                deserializer_lines.append(des_line)

        self._createSerializerSource(serializer_method, deserializer_method, serializer_lines, deserializer_lines,
                                     cfe_data_type, pkg_name)

    def _createSerializerHeader(self, cfe_data_type, include_list, struct_list, ser_method_name, des_method_name,
                                pkg_name, const_list):
        """Create Serialize heder file.
        
        Args:
            cfe_data_type (str): Data type in cFE
            include_list (list): List of include in source
            struct_list (list): List of structures in source
            ser_method_name (str): Serializing function name
            des_method_name (str): Deserializing function name
            pkg_name (str): Package name
            const_list (list): List of constants

        """

        header_file_path = self.SERI_DIR + cfe_data_type.lower() + "_serializer.h"

        # Create header file
        header_list = []
        header_list.append("#ifndef {}_serializer_h_".format(cfe_data_type.lower()))
        header_list.append("#define {}_serializer_h_".format(cfe_data_type.lower()))
        header_list.append("")
        header_list.append("#include \"cfe.h\"")
        header_list.append("#include \"util.h\"")
        header_list.append("")
        for include in include_list:
            header_list.append(include)
        header_list.append("")

        if len(struct_list) > 0:
            if pkg_name != "relay_app":
                header_list.append("namespace " + pkg_name + " {")
            header_list.append("typedef struct")
            header_list.append("{")
            if pkg_name != "relay_app":
                header_list.append("  uint8 TlmHeader[CFE_SB_TLM_HDR_SIZE];")
            for struct in struct_list:
                header_list.append(struct)
            header_list.append("")
            header_list.append("  void vector2pointer()")
            header_list.append("  {")
            for struct in struct_list:
                if struct.find("::") != -1:
                    str_list = struct.strip().replace(";", "").split(" ")
                    member = str_list[1]
                    header_list.append("    " + member + ".vector2pointer();")
            header_list.append("  }")
            header_list.append("")
            header_list.append("  void pointer2vector()")
            header_list.append("  {")
            for struct in struct_list:
                if struct.find("::") != -1:
                    str_list = struct.strip().replace(";", "").split(" ")
                    member = str_list[1]
                    header_list.append("    " + member + ".pointer2vector();")
            header_list.append("  }")
            header_list.append("")
            header_list.append("  void deleteData()")
            header_list.append("  {")
            for struct in struct_list:
                if struct.find("::") != -1:
                    str_list = struct.strip().replace(";", "").split(" ")
                    member = str_list[1]
                    header_list.append("    " + member + ".deleteData();")
            header_list.append("  }")
            header_list.append("")
            header_list.append("  void string2pointer()")
            header_list.append("  {")
            for struct in struct_list:
                if struct.find("::") != -1:
                    str_list = struct.strip().replace(";", "").split(" ")
                    member = str_list[1]
                    header_list.append("    " + member + ".string2pointer();")
            header_list.append("  }")
            header_list.append("")
            header_list.append("  void pointer2string()")
            header_list.append("  {")
            for struct in struct_list:
                if struct.find("::") != -1:
                    str_list = struct.strip().replace(";", "").split(" ")
                    member = str_list[1]
                    header_list.append("    " + member + ".pointer2string();")
            header_list.append("  }")
            for const_line in const_list:
                header_list.append("  " + const_line + ";")
            header_list.append("} " + cfe_data_type + " ;")
            if pkg_name != "relay_app":
                header_list.append("};")
        header_list.append("")

        data_type = cfe_data_type
        if pkg_name != "relay_app":
            data_type = pkg_name + "::" + cfe_data_type
        header_list.append("int " + ser_method_name + "(const " + data_type + " &send_data, unsigned char *buffer);")
        header_list.append(data_type + " " + des_method_name + "(const unsigned char *buffer, unsigned int &length);")
        header_list.append("")

        header_list.append("#endif // {}_serializer_h_".format(cfe_data_type.lower()))
        header_list.append("")

        self.fop._writeFile(header_file_path, header_list)

    def _createSerializerSource(self, serializer_method, deserializer_method, serializer_lines, deserializer_lines,
                                cfe_data_type, pkg_name):
        """Create serialize source file.
        
        Args:
            serializer_method (str): Serializing method
            deserializer_method (str): Deserializing method
            serializer_lines (list): List of line which serializing method is applied
            deserializer_lines (list): List of line which deserializing method is applied
            cfe_data_type (str): Data type in cFE
            pkg_name (str): Package name

        """

        cpp_file_path = self.SERI_DIR + cfe_data_type.lower() + "_serializer.cpp"

        # Create cpp file
        source_list = []
        source_list.append("#include \"cfs_serializer/{}_serializer.h\"".format(cfe_data_type.lower()))
        source_list.append("")

        data_type = cfe_data_type
        if pkg_name != "relay_app":
            data_type = pkg_name + "::" + cfe_data_type
        source_list.append("int " + serializer_method + "(const " + data_type + " &send_data, unsigned char *buffer) {")
        source_list.append(self.util._addIndent(1) + "unsigned int len = 0;")
        for ser in serializer_lines:
            source_list.append(ser)
        source_list.append(self.util._addIndent(1) + "return len;")
        source_list.append("}")

        source_list.append("")

        source_list.append(data_type + " " + deserializer_method
                           + "(const unsigned char *buffer, unsigned int &length) {")
        source_list.append(self.util._addIndent(1) + data_type + " msg;")
        source_list.append(self.util._addIndent(1) + "unsigned int ii;")
        source_list.append(self.util._addIndent(1) + "unsigned int len;")
        source_list.append(self.util._addIndent(1) + "length = 0;")

        for des in deserializer_lines:
            source_list.append(des)

        source_list.append(self.util._addIndent(1) + "return msg;")
        source_list.append("}")

        self.fop._writeFile(cpp_file_path, source_list)

    def _get_datatype_name_add_pkg(self, data_type):
        """Get add datatype package name.

        Args:
            data_type (str): Data type
        
        Returns:
            str: String of data type with package name

        """        


        if data_type in ["Point", "Pose", "PoseStamped", "PoseWithCovariance", "Quaternion", "Twist",
                         "TwistWithCovariance", "Vector3", "Wrench", "WrenchStamped"]:
            return "geometry_msgs_" + data_type.lower()
        elif data_type in ["Odometry"]:
            return "nav_msgs_" + data_type.lower()
        elif data_type in ["JointState", "Image"]:
            return "sensor_msgs_" + data_type.lower()
        elif data_type in ["Float64MultiArray", "Header", "RosTime", "MultiArrayDimension", "MultiArrayLayout",
                           "RosDuration", "String", "Empty"]:
            return "std_msgs_" + data_type.lower()
        else:
            return data_type.lower()
