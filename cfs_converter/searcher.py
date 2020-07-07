"""
Copyright (c) 2020 Japan Aerospace Exploration Agency.
All Rights Reserved.

This file is covered by the LICENSE.txt in the root of this project.
"""

import re
import os


class Searcher:
    """Searcher

    Raises:
        Exception: Could not find file with main function.
        Exception: The package name could not be acquired.

    """

    global_members = []
    global_functions = []
    node_handle_member = ""
    nh_name = None
    ros_print_count = 0
    subscribe_callback_list = []

    def __init__(self):
        """Constructor."""

        pass

    def _search_initialize(self):
        """Search initialize name."""

        self.global_members = []
        self.global_functions = []
        self.nh_name = None

    def _searchGlobalFunctions(self, file_path):
        """Search global functions.

        Args:
            file_path (str): File path

        """

        for line in open(file_path, "r"):
            # Delete comment
            line = re.sub(r'//.+', "", line)
            # #include, #define, #pragma, #if, #end, typedef, using namespace, struct have ignore
            ignores = ["#define", "#include", "#pragma", "#if",
                       "#end", "typedef", "using namespace", "struct"]
            continue_flag = False
            for ignore in ignores:
                if line.find(ignore) != -1:
                    continue_flag = True
                    break

            if continue_flag is True:
                # Ignored if the string specified in ignores is included
                continue

            if line.find(" ") != 0 and len(line.strip().split(" ")) > 1 and line.find("(") != -1 \
               and re.sub(r'\(.+', "", line).find("::") == -1:
                result = line.strip().split(" ")
                del result[0]
                combined_function = ""
                for word in result:
                    combined_function = combined_function + word
                combined_function = re.sub(r"\(.+", "", combined_function)
                if not combined_function == "":
                    self.global_functions.append(combined_function)
        # The main function is deleted because there are other conversion methods
        if "main" in self.global_functions:
            for i in range(0, len(self.global_functions)):
                if "main" == self.global_functions[i]:
                    del self.global_functions[i]

    def _searchNodeHandleMember(self, header_path):
        """Search node hendle menber.

        Args:
            header_path (list): Path to header file

        """        

        for line in open(header_path, "r"):
            if line.find("NodeHandle") != -1:
                line = line.strip()
                line_splits = line.split(" ")
                self.node_handle_member = line_splits[1].strip().replace(
                    ";", "")
                break

    def _searchNodeHandle(self, path):
        """Search node handle.

        Args:
            path (str): Path to the target directory/file

        """

        if len(self.node_handle_member) > 0:
            return
        if os.path.isdir(path):
            files = os.listdir(path)
            for f in files:
                self._searchNodeHandle(path + "/" + f)
        else:
            self._searchNodeHandleMember(path)

    def _searchAllGlobalMembers(self, path):
        """Search all global menbers.

        Args:
            path (str): Path to the target directory/file

        """

        if os.path.isdir(path):
            files = os.listdir(path)
            for f in files:
                self._searchAllGlobalMembers(path + "/" + f)
        else:
            self._searchGlobalMembers(path)

    def _searchGlobalMembers(self, file_path):
        """Search global menbers.

        Args:
            file_path (str): File path

        """

        # Get all global variables
        brackets_num = 0
        comment_num = 0
        for line in open(file_path, "r"):
            # Delete comment
            line = re.sub(r'//.+', "", line)

            # Remove trailing spaces
            line = re.sub(r'\s+$', "", line)

            # #include, #define, #if, #end have ignore
            ignores = ["#define", "#include", "#if", "#end", "using namespace", "typedef",
                       "struct", "const", "#undef", "#pragma"]
            continue_flag = False
            for ignore in ignores:
                if line.find(ignore) != -1:
                    continue_flag = True

            if continue_flag is True:
                # Ignored if the string specified in ignores is included
                continue

            # If { is present, it is ignored until the end of the declaration, corresponding to }
            if line.find("/*") != -1:
                if line.find("*/") == -1:
                    comment_num = comment_num + 1
            elif line.find("*/") != -1:
                comment_num = comment_num - 1
            elif line.find("{") != -1:
                if line.find("}") == -1:
                    brackets_num = brackets_num + 1
            elif line.find("}") != -1:
                if line.find("{") == -1:
                    brackets_num = brackets_num - 1
            elif line.find("class") != -1:
                pass
            else:
                if brackets_num == 0 and comment_num == 0:
                    # Remove static, const, etc. because they interfere with processing
                    line = line.replace("static ", "")
                    line = line.replace("const ", "")

                    # Check if it is a variable and put it in global_members
                    # Lines without (), no leading single-byte space, and separated by single-byte spaces are
                    # considered to be lines declaring global variables
                    if line.find(" ") != 0 and len(line.strip().split(" ")) > 1:
                        if line.find("(") == -1:
                            # Data type and variable name are split
                            result = line.strip().split(" ")
                            del result[0]
                            combined_member = ""
                            equal_flag = False
                            for word in result:
                                if equal_flag is True:
                                    equal_flag = False
                                elif word.find("=") >= 0:
                                    equal_flag = True
                                else:
                                    combined_member = combined_member + word
                            members = combined_member.split(",")
                            for member in members:
                                combined_member = member.strip().replace(";", "")
                                if not combined_member == "":
                                    combined_member = re.sub(
                                        r'.+::', "", combined_member)
                                    self.global_members.append(combined_member)
                elif line.find("static ") != -1:
                    # Variables of static need to be Substitution in the class definition
                    line = line.replace(";", "")
                    line = line.strip(" ")
                    line = re.sub(r'\s+', " ", line)
                    str_list = line.split(" ")
                    # It should be of the form static data_type member, so others are not global constants
                    if len(str_list) != 3:
                        continue
                    self.global_members.append(str_list[-1])

    def _searchMainSourceFile(self, cfe_src_dir_path):
        """Search main source file.
        
        Args:
            cfe_src_dir_path (str): Path to the directory of cFE sources
        
        Raises:
            Exception: Could not find file with main function.

        Returns:
            str: Name of file including main function

        """

        # Search main function in src directory
        cfe_src_files = os.listdir(cfe_src_dir_path)
        main_src_file = None
        for cfe_src_file in cfe_src_files:
            cfe_src_file_path = cfe_src_dir_path + "/" + cfe_src_file
            if os.path.isfile(cfe_src_file_path):
                # open and read line
                is_search = False
                for line in open(cfe_src_file_path, "r"):
                    # check "int main("
                    result_list = re.findall(r"int\s+main\(", line)
                    # if exist, break this loop
                    if len(result_list) > 0:
                        is_search = True
                        break
                if is_search is True:
                    main_src_file = cfe_src_file
                    break

        if main_src_file is None:
            raise Exception("Error: Could not find file with main function")

        return main_src_file

    def _searchNameSpace(self, path):
        """Search name space.

        Args:
            path (str): Path to the target directory/file

        Returns:
            str: Name of node handle

        """        

        if os.path.isdir(path):
            files = os.listdir(path)
            for f in files:
                self._searchNameSpace(path + "/" + f)
                if self.nh_name is not None:
                    return self.nh_name

        else:
            for line in open(path, "r"):
                result = re.search(r"ros::NodeHandle .+\(.+\);", line)
                if result is not None:
                    searched_str = result.group()
                    self.nh_name = searched_str.replace(";", "").split("\"")[1]
                    return self.nh_name

        if self.nh_name is not None:
            return self.nh_name

    def _searchPkgName(self, pkg_path):
        """Search package

        Args:
            pkg_path (str): Path to the package

        Raises:
            Exception: The package name could not be acquired

        Returns:
            str: Package name

        """

        while len(pkg_path) > 0:
            # Find package.xml directly under pkg_path
            # If not found, search further up one the hierarchy
            if os.path.exists(pkg_path + "/package.xml"):
                # The name of the layer above package.xml is the package name
                dir_list = pkg_path.split("/")
                pkg_name = dir_list[len(dir_list) - 1]
                return pkg_name
            dir_list = pkg_path.split("/")
            dir_list = dir_list[0:len(dir_list) - 1]
            pkg_path = ""
            for dir_name in dir_list:
                if len(dir_name) > 0:
                    pkg_path = pkg_path + "/" + dir_name
        raise Exception("The package name could not be acquired")

    def _searchRosPrint(self, path):
        """Search fos print.
        
        Args:
            path (str): Path to the target directory/file
        
        Returns:
            int: Number of occurrences of "ROS_INFO,ROS_DEBUG,ROS_WARN"
        """        
        if os.path.isdir(path):
            files = os.listdir(path)
            for f in files:
                self._searchRosPrint(path + "/" + f)

        else:
            for line in open(path, "r"):
                if line.find("ROS_INFO(") != -1:
                    self.ros_print_count = self.ros_print_count + 1
                if line.find("ROS_DEBUG(") != -1:
                    self.ros_print_count = self.ros_print_count + 1
                if line.find("ROS_WARN(") != -1:
                    self.ros_print_count = self.ros_print_count + 1
        return self.ros_print_count

    def _searchSubscribeCallback(self, path):
        """Search subscribe callback.

        Args:
            path (str): Path to the target directory/file

        """

        if os.path.isdir(path):
            files = os.listdir(path)
            for f in files:
                self._searchSubscribeCallback(path + "/" + f)

        else:
            for line in open(path, "r"):
                if line.find(".subscribe(") != -1:
                    # Decompose argument
                    argument = self._getArgument(line)
                    arg_list = argument.split(",")
                    callback = arg_list[2].strip(" ")
                    callback_method = callback.split(" ")[0]
                    method = re.sub(r".+::", "", callback_method)
                    self.subscribe_callback_list.append(
                        {"method": method, "data_type": "", "path": path})

    def _getArgument(self, target_line):

        pattern = re.compile(r"\(.+\)")
        match = pattern.search(target_line)
        argv_code = match.group(0)
        argv_code = argv_code.replace("(", "")
        argv_code = argv_code.replace(")", "")
        return argv_code

    def _searchCallback(self, path):
        if os.path.isdir(path):
            files = os.listdir(path)
            for f in files:
                self._searchCallback(path + "/" + f)
        else:
            for line in open(path, "r"):
                for callback in self.subscribe_callback_list:
                    method = callback["method"]
                    data_type = callback["data_type"]
                    if len(data_type) > 0:
                        continue

                    if line.find("::" + method + "(") != -1 or line.find(" " + method + "(") != -1:
                        argument = self._getArgument(line)
                        arg_list = argument.replace("const", "").replace(
                            "::ConstPtr", "").replace("ConstPtr", "").strip(" ").split(" ")
                        data_type = arg_list[0]
                        callback["data_type"] = data_type
                        break
        return self.subscribe_callback_list
