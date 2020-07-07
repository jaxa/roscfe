"""
Copyright (c) 2020 Japan Aerospace Exploration Agency.
All Rights Reserved.

This file is covered by the LICENSE.txt in the root of this project.
"""

import os
import shutil
import yaml


class FileOperator:
    """FileOperator.

    Raises:
        Exception: file does not exist.
        Exception: directory does not exist.

    """

    output_path = ""

    FSW_DIR = "/fsw"
    BUILF_DIR = "/for_build"
    MISINC_DIR = "/mission_inc"
    PLATINC_DIR = "/platform_inc"
    SRC_DIR = "/src"
    INC_DIR = "/include"

    PORT_DIR = "./porting_files"
    GEO_DIR = "/geometry_msgs"
    NAV_DIR = "/nav_msgs"
    SEN_DIR = "/sensor_msgs"
    STD_DIR = "/std_msgs"
    TF_DIR = "/tf"
    ARDRONE_DIR = "/ardrone_autonomy"

    def __init__(self, output_path):
        """Constructor.

        Args:
            output_path (str): Output directory path

        """
        self.output_path = output_path

    def _writeFile(self, file_path, lines, is_add_linefeed=True):
        """Write file.

        Args:
            file_path (str): File path
            lines (list): Lines to be written in file
            is_add_linefeed (bool, optional): Whether line feed is added on every line or not. Defaults to True.

        """
        wfp = open(file_path, "w")
        for line in lines:
            wfp.write(line)
            if is_add_linefeed:
                wfp.write("\n")
        wfp.close()

    def _getMsgListFilePath(self):
        """Get Message list file path.

        Returns:
            str: File path about message list.

        """
        dir_path = os.environ['HOME'] + "/.cfs_converter"
        if not os.path.exists(dir_path):
            os.mkdir(dir_path)
        file_path = dir_path + "/msg_list.json"
        if not os.path.exists(file_path):
            self._writeFile(file_path, ["{}"], False)
        return file_path

    def _getWorkFilesPath(self):
        """Get work file path.

        Returns:
            tuple: (convert_file_path, data_file_path)

        """
        dir_path = os.environ['HOME'] + "/.cfs_converter"
        if not os.path.exists(dir_path):
            os.mkdir(dir_path)
        convert_file_path = dir_path + "/convert.txt"
        data_file_path = dir_path + "/data.txt"
        return convert_file_path, data_file_path

    def _createCfeAppDir(self, node_name, create_list, node_path, pkg_name):
        """Create cfe application directory.

        Args:
            node_name (str): Name of ROS node
            create_list (list): List of nodes to be converted as cFE app
            node_path (str): Path of ROS node's sources
            pkg_name (str): Package name

        Returns:
            tuple: (cfe_inc_dir_path, cfe_src_dir_path, cfe_apl_name, create_no,
            cfe_platform_dir_path, cfe_makefile_dir_path)

        """
        create_no = 1
        if node_name in create_list:
            create_no = create_list[node_name]
        cfe_apl_name = node_name + "_{}".format(create_no)
        dir_path = self.output_path + "/" + cfe_apl_name
        if os.path.exists(dir_path):
            shutil.rmtree(dir_path)
        os.mkdir(dir_path)
        os.mkdir(dir_path + self.FSW_DIR)
        cfe_makefile_dir_path = dir_path + self.FSW_DIR + self.BUILF_DIR
        os.mkdir(cfe_makefile_dir_path)
        os.mkdir(dir_path + self.FSW_DIR + self.MISINC_DIR)
        os.mkdir(dir_path + self.FSW_DIR + self.PLATINC_DIR)
        cfe_platform_dir_path = dir_path + self.FSW_DIR \
            + self.PLATINC_DIR + "/" + pkg_name
        os.mkdir(cfe_platform_dir_path)
        cfe_src_dir_path = dir_path + self.FSW_DIR + self.SRC_DIR
        os.mkdir(cfe_src_dir_path)
        shutil.copytree(self.PORT_DIR + self.GEO_DIR,
                        cfe_src_dir_path + self.GEO_DIR)
        shutil.copytree(self.PORT_DIR + self.NAV_DIR,
                        cfe_src_dir_path + self.NAV_DIR)
        shutil.copytree(self.PORT_DIR + self.SEN_DIR,
                        cfe_src_dir_path + self.SEN_DIR)
        shutil.copytree(self.PORT_DIR + self.STD_DIR,
                        cfe_src_dir_path + self.STD_DIR)
        shutil.copytree(self.PORT_DIR + self.TF_DIR,
                        cfe_src_dir_path + self.TF_DIR)
        shutil.copytree(self.PORT_DIR + self.ARDRONE_DIR,
                        cfe_src_dir_path + self.ARDRONE_DIR)
        cfe_inc_dir_path = dir_path + self.FSW_DIR \
            + self.SRC_DIR + self.INC_DIR
        os.mkdir(cfe_inc_dir_path)
        return cfe_inc_dir_path, cfe_src_dir_path, cfe_apl_name, create_no, \
            cfe_platform_dir_path, cfe_makefile_dir_path

    def _tabToSpace(self, cfe_dir_path):
        """Change tap to space.

        Args:
            cfe_dir_path (str): Path to the directory of cFE sources

        """
        cfe_files = os.listdir(cfe_dir_path)
        for cfe_file in cfe_files:
            cfe_file = cfe_dir_path + "/" + cfe_file
            if os.path.isfile(cfe_file):
                dst_lines = []
                rfp = open(cfe_file, "r")
                for line in rfp:
                    dst_lines.append(line.replace("\t", "    "))
                rfp.close()
                self._writeFile(cfe_file, dst_lines, False)
            else:
                self._tabToSpace(cfe_file)

    def _copyOtherFiles(self, node_path, cfe_src_dir_path):
        """Copy to other files.

        Args:
            node_path (str): Path of ROS node's sources
            cfe_src_dir_path (str): Path to the directory of cFE sources

        """
        # Copy directories and files other than CMakeLists.txt, package.xml,
        # launch, msg, srv, include, and src
        ros_node_files = os.listdir(node_path)
        no_copy_files = ["CMakeLists.txt", "package.xml", "launch", "msg",
                         "srv", "include", "src"]
        for ros_node_file in ros_node_files:
            if ros_node_file not in no_copy_files:
                ros_node_file_path = node_path + "/" + ros_node_file
                cfe_node_file_path = cfe_src_dir_path + "/" + ros_node_file
                if os.path.isfile(ros_node_file_path):
                    shutil.copyfile(ros_node_file_path, cfe_node_file_path)
                else:
                    shutil.copytree(ros_node_file_path, cfe_node_file_path)

    def _copySorceFiles(self, node_path, cfe_src_dir_path):
        """Copy sorce files.

        Args:
            node_path (str): Path of ROS node's sources
            cfe_src_dir_path (str): Path to the directory of cFE sources

        """
        # Copy with directories and files directly under src
        ros_src_dir_path = node_path + self.SRC_DIR
        if os.path.exists(ros_src_dir_path):
            ros_src_files = os.listdir(ros_src_dir_path)
            for ros_src_name in ros_src_files:
                ros_src_path = ros_src_dir_path + "/" + ros_src_name
                cfe_src_path = cfe_src_dir_path + "/" + ros_src_name
                if os.path.isfile(ros_src_path):
                    shutil.copyfile(ros_src_path, cfe_src_path)
                else:
                    shutil.copytree(ros_src_path, cfe_src_path)

    def _copyIncludeFiles(self, node_path, cfe_inc_dir_path):
        """Copy include files.

        Args:
            node_path (str): Path of ROS node's sources
            cfe_inc_dir_path (str): Path to the directory of cFE 'include' files

        """
        # Copy with directories and files directly under include
        ros_inc_dir_path = node_path + self.INC_DIR
        if os.path.exists(ros_inc_dir_path):
            ros_inc_files = os.listdir(ros_inc_dir_path)
            for ros_inc_name in ros_inc_files:
                ros_inc_path = ros_inc_dir_path + "/" + ros_inc_name
                cfe_inc_path = cfe_inc_dir_path + "/" + ros_inc_name
                if os.path.isfile(ros_inc_path):
                    shutil.copyfile(ros_inc_path, cfe_inc_path)
                else:
                    shutil.copytree(ros_inc_path, cfe_inc_path)

    def _copyFiles(self, src, dst):
        """Copy files.

        Args:
            src (str): Source path
            dst (str): Destination path

        Raises:
            Exception: file does not exist.

        """
        if not os.path.exists(src):
            raise Exception("{}does not exist.".format(src))
        if os.path.exists(dst):
            shutil.rmtree(dst)
        if os.path.isfile(src):
            shutil.copyfile(src, dst)
        else:
            shutil.copytree(src, dst)

    def _removeDir(self, src):
        """Remove directory/file.

        Args:
            src (str): Directory/file path

        Raises:
            Exception: Directory/file does not exist.

        """
        if not os.path.exists(src):
            raise Exception("{}does not exist.".format(src))
        if os.path.isfile(src):
            os.remove(src)
        else:
            shutil.rmtree(src)

    def _replaceDir(self, src, dst):
        """Replace directory.

        Args:
            src (str): Source path
            dst (str): Destination path

        """
        if os.path.exists(dst):
            # Delete output destination if it already exists
            self._removeDir(dst)
        shutil.move(src, dst)

    def _readYaml(self, path):
        """Read yaml.

        Args:
            path (str): Yaml file path

        Returns:
            dict: Contents of yaml file

        """
        yfp = open(path, "r")
        data = yaml.load(yfp)
        return data

    def _addDataTypeToSubscribe(self, callback_list):
        """Add data type to subscribe.

        Args:
            callback_list (list): List of callbacks

        """
        for callback in callback_list:
            path = callback["path"]
            data_type = callback["data_type"]
            method = callback["method"]
            dst_lines = []
            for line in open(path, "r"):
                if line.find(".subscribe(") != -1 and line.find(method) != -1:
                    line = line.replace(
                        ".subscribe(", ".subscribe<" + data_type + ">(")
                dst_lines.append(line)

            self._writeFile(path, dst_lines, False)
