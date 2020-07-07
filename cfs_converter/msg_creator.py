"""
Copyright (c) 2020 Japan Aerospace Exploration Agency.
All Rights Reserved.

This file is covered by the LICENSE.txt in the root of this project.
"""
import re
import json


class MsgCreator():
    """Create message."""

    fop = None

    def __init__(self, fop):
        """Constructor.
        
        Args:
            fop (FileOperator): File operator object

        """        
        self.fop = fop

    def _msg2header(self, msg_files, msg_dir_path, node_name,
                    cfe_platform_dir_path, pkg_name):
        """Message to header file.
        
        Args:
            msg_files (list): List of message files
            msg_dir_path (str): Path to the directory of message files
            node_name (str): Name of ROS node
            cfe_platform_dir_path (str): Path to the directory containing header-files
                                         under 'platform_inc' of cFE sources
            pkg_name (str): Package name


        """        
        basic_data_types = ["int8", "int16", "int32", "int64", "float32",
                            "float64", "string", "time", "duration", "uint8",
                            "uint16", "uint32", "uint64"]
        convert_data_types = {"int8": "char", "int16": "short", "int32": "int",
                              "int64": "long", "float32": "float",
                              "float64": "double", "string": "std::string",
                              "time": "RosTime", "duration": "duration",
                              "uint8": "unsigned char",
                              "uint16": "unsigned short",
                              "uint32": "unsigned int",
                              "uint64": "unsigned long"}
        for msg_file_name in msg_files:
            include_list = []
            include_list.append("#include <string>")
            include_list.append("#include <vector>")
            include_list.append("#include \"cfe.h\"")
            data_list = []
            vector_list = []
            struct_list = []
            # Read message file
            rfp = open(msg_dir_path + "/" + msg_file_name, 'r')
            msg_lines = []
            const_list = []
            for row in rfp:
                # Read message fileRead message file
                row = row[:row.find("#")]
                if len(row) == 0:
                    continue
                if row.find("=") != -1:
                    const_list.append(row)
                    continue
                line = re.sub(r'[\s\t]+', " ", row)
                msg_lines.append(line)

            for row in msg_lines:
                # Split row
                splited_row = row.strip().split(" ")
                array_flag = 0
                data_name = splited_row[1]
                # Case of basic data
                if splited_row[0] in basic_data_types:
                    data_type = convert_data_types[splited_row[0]]
                    if data_type.find("[]") >= 0:
                        array_flag = 1
                        data_type = data_type.replace("[]", "")
                    if data_type == "RosTime":
                        include_list.append(
                            "#include \"../std_msg/RosTime.h\"")
                # Case of other structures
                else:
                    struct_list.append(data_name)
                    data_type = splited_row[0]
                    if data_type.find("[]") >= 0:
                        array_flag = 1
                        data_type = data_type.replace("[]", "")

                    data_pkg_name = ""
                    if data_type in ["Point", "Pose", "PoseStamped",
                                     "PoseWithCovariance", "Quaternion",
                                     "Twist", "TwistWithCovariance", "Vector3",
                                     "Wrench", "WrenchStamped"]:
                        include_list.append(
                            "#include \"../geometry_msgs/" + data_type + ".h\"")
                        data_pkg_name = "geometry_msgs"
                    elif data_type in ["Odometry"]:
                        include_list.append(
                            "#include \"../nav_msgs/" + data_type + ".h\"")
                        data_pkg_name = "nav_msgs"
                    elif data_type in ["JointState", "Image"]:
                        include_list.append(
                            "#include \"../sensor_msgs/" + data_type + ".h\"")
                        data_pkg_name = "sensor_msgs"
                    elif data_type in ["Float64MultiArray", "Header",
                                       "RosTime", "MultiArrayDimension",
                                       "MultiArrayLayout", "String", "Empty"]:
                        include_list.append(
                            "#include \"../std_msgs/" + data_type + ".h\"")
                        data_pkg_name = "std_msgs"
                    else:
                        include_list.append(
                            "#include \"../" + node_name + "/" + data_type + ".h\"")
                        data_pkg_name = node_name
                    data_type = data_pkg_name + "::" + data_type

                # Case of arrays
                if array_flag == 1:
                    vector_list.append([data_type, data_name])
                    data_list.append("std::vector<" +
                                     data_type + "> " + data_name)
                    data_list.append(data_type + "* " + data_name + "Data")
                    data_list.append("uint32 " + data_name + "DataSize")
                # Case of not an array
                else:
                    data_list.append(data_type + " " + data_name)
            rfp.close()
            # Write definition to header file
            wfp = open(cfe_platform_dir_path + "/" +
                       msg_file_name.replace(".msg", ".h"), "w")
            define_name = "_" + msg_file_name.upper().replace(".", "_") + "_"
            wfp.write("#ifndef " + define_name + "\n")
            wfp.write("#define " + define_name + "\n")
            wfp.write("\n")
            for include_line in include_list:
                wfp.write(include_line + "\n")
            wfp.write("\n")
            wfp.write("namespace " + pkg_name + "\n")
            wfp.write("{\n")
            wfp.write("    typedef struct\n")
            wfp.write("    {\n")
            wfp.write("        uint8 TlmHeader[CFE_SB_TLM_HDR_SIZE];\n")
            for data_line in data_list:
                wfp.write("        " + data_line + ";\n")
            wfp.write("\n")
            wfp.write("        void vector2pointer()\n")
            wfp.write("        {\n")
            wfp.write("\n")
            for struct_line in struct_list:
                wfp.write("            " + struct_line +
                          ".vector2pointer();\n")
            wfp.write("\n")
            for vector_line in vector_list:
                vector_type = vector_line[0]
                vector_name = vector_line[1]
                wfp.write("            " + vector_name + "Data = (" + vector_type +
                          "*)malloc(" + vector_name + ".size() * sizeof(" + vector_type + "));\n")
                wfp.write("            for(int ii = 0; ii < " +
                          vector_name + ".size(); ii++)\n")
                wfp.write("                " + vector_name +
                          "Data[ii] = " + vector_name + "[ii];\n")
                wfp.write("                " + vector_name +
                          "DataSize = " + data_name + ".size();\n")
                wfp.write("                std::vector<" +
                          vector_type + ">().swap(" + vector_name + ");\n")
            wfp.write("        }\n")
            wfp.write("\n")
            wfp.write("        void pointer2vector()\n")
            wfp.write("        {\n")
            wfp.write("\n")
            for struct_line in struct_list:
                wfp.write("            " + struct_line +
                          ".pointer2vector();\n")
            wfp.write("\n")
            for vector_line in vector_list:
                vector_type = vector_line[0]
                vector_name = vector_line[1]
                wfp.write("            uint32 " + vector_name +
                          "_size = " + vector_name + "DataSize;\n")
                wfp.write("            std::vector<" + vector_type +
                          ">().swap(" + vector_name + ");\n")
                wfp.write("            for (size_t ii = 0; ii < " +
                          vector_name + "_size; ii++) {\n")
                wfp.write("                " + vector_name +
                          ".push_back(" + vector_name + "Data[ii]);\n")
                wfp.write("            }\n")
            wfp.write("        }\n")
            wfp.write("\n")
            wfp.write("        void deleteData()\n")
            wfp.write("        {\n")
            for struct_line in struct_list:
                wfp.write("            " + struct_line + ".deleteData();\n")
            wfp.write("\n")
            for vector_line in vector_list:
                vector_name = vector_line[1]
                wfp.write("            free(" + vector_name + ");\n")
            wfp.write("        }\n")
            wfp.write("\n")
            wfp.write("        void string2pointer()\n")
            wfp.write("        {\n")
            for struct_line in struct_list:
                wfp.write("            " + struct_line +
                          ".string2pointer();\n")
            wfp.write("\n")
            wfp.write("        }\n")
            wfp.write("\n")
            wfp.write("        void pointer2string()\n")
            wfp.write("        {\n")
            for struct_line in struct_list:
                wfp.write("            " + struct_line +
                          ".pointer2string();\n")
            wfp.write("\n")
            wfp.write("        }\n")
            wfp.write("\n")

            # Define constants
            for const_line in const_list:
                wfp.write("        " + const_line + ";\n")
            wfp.write("\n")
            wfp.write("    } " + msg_file_name.replace(".msg", "") + ";\n")

            wfp.write("    typedef " + msg_file_name.replace(".msg", "") +
                      "* const " + msg_file_name.replace(".msg", "") + "ConstPtr;\n")

            wfp.write("}\n")
            wfp.write("#endif // " + define_name + "\n")
            wfp.close()

            # Record of message file pass
            self._saveMsgFilePath(pkg_name, msg_dir_path + "/" + msg_file_name,
                                  msg_file_name.replace(".msg", ""))

    def _saveMsgFilePath(self, pkg_name, msg_path, datatype):
        """Save message file path.
        
        Args:
            pkg_name (str): Package name
            msg_path (str): Path to the message file
            datatype (str): Data type

        """        
        msg_list_path = self.fop._getMsgListFilePath()

        f = open(msg_list_path, "r")
        data = json.load(f)
        f.close()

        # Add path using pkg_name :: datatype as a key
        key = pkg_name + "::" + datatype
        if key not in data:
            data[key] = msg_path

        # save
        f = open(msg_list_path, "w")
        json.dump(data, f)
        f.close()
