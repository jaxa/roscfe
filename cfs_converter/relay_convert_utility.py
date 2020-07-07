"""
Copyright (c) 2020 Japan Aerospace Exploration Agency.
All Rights Reserved.

This file is covered by the LICENSE.txt in the root of this project.
"""

import glob
import os


class RelayConvertUtility:
    """RelayConvertUtility."""

    basic_data_types = ["int8", "int16", "int32", "int64", "float32", "float64",
                        "string", "time", "duration", "uint8", "uint16", "uint32", "uint64"]
    convert_data_types = {"int8": "char", "int16": "short", "int32": "int", "int64": "long", "float32": "float",
                          "float64": "double", "string": "std::string", "time": "RosTime", "duration": "duration",
                          "uint8": "unsigned char", "uint16": "unsigned short",
                          "uint32": "unsigned int", "uint64": "unsigned long"}

    def __init__(self):
        """Constructor."""        
        pass

    def _addIndent(self, num):
        """Add indent.
        
        Args:
            num (int): Number of indent
        
        Returns:
            str: Spaces

        """

        indent_str = ""
        for ii in range(num):
            indent_str = indent_str + "  "
        return indent_str

    def _searchSerializer(self, target_dir_path):
        """Search serializer

        Args:
            target_dir_path (str): Path to the target directory
        
        Returns:
            list: List of file path

        """        

        ser_list = []
        file_list = glob.glob(target_dir_path + "/*")

        for file_path in file_list:
            # Determine whether it is a file or a directory, and if it is a file, add it to ser_list
            # If it is a directory, call this function
            if os.path.isfile(file_path):
                ser_list.append(file_path)
            elif os.path.isdir(file_path):
                files = self._searchSerializer(file_path)
                for path in files:
                    ser_list.append(path)
        return ser_list

    def _getMsgFileName(self, file_path):
        """Get message file name.

        Args:
            file_path (str): File path
        
        Returns:
            str: Name of message file

        """

        units = file_path.split("/")
        msg_file = units[len(units) - 1]
        msg_type = msg_file.replace(".msg", "")
        return msg_type
