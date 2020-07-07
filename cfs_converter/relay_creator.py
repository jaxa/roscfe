"""
Copyright (c) 2020 Japan Aerospace Exploration Agency.
All Rights Reserved.

This file is covered by the LICENSE.txt in the root of this project.
"""

import relay_app_creator
import relay_node_creator
import relay_setting_reader
import file_operator
import relay_logger
import topic_manager

import sys
import traceback


class RelayCreator:
    """Create of relay application."""

    app_creator = None
    node_creator = None
    setting_reader = None
    fop = None
    logger = None
    topic_manager = None

    BASE_PATH = "./relay_base/"
    DST_PATH = "./relay/"
    APP_PATH = ""
    NODE_PATH = ""
    APP_BASE_PATH = ""
    NODE_BASE_PATH = ""
    PORTING_PATH = ""
    CONV_PATH = ""
    cfe_dir_path = ""
    ros_dir_path = ""

    def __init__(self, logger, cfe_dir_path, ros_dir_path):
        """Constructor.

        Args:
            logger (RelayLogger): RelayLogger object
            cfe_dir_path (str): Path to the cfe_relay_app directory
            ros_dir_path (str): Path to the ros_relay_node directory

        """

        self.fop = file_operator.FileOperator("./")
        convert_file_path, data_file_path = self.fop._getWorkFilesPath()
        self.topic_manager = topic_manager.TopicManager(
            convert_file_path, data_file_path)

        self.setting_reader = relay_setting_reader.RelaySettingReader(
            self.topic_manager)
        self.app_creator = relay_app_creator.RelayAppCreator(
            self.setting_reader)
        self.node_creator = relay_node_creator.RelayNodeCreator()
        self.logger = logger

        self.APP_PATH = self.DST_PATH + "relay_app/"
        self.NODE_PATH = self.DST_PATH + "relay_node/"
        self.APP_BASE_PATH = self.BASE_PATH + "relay_app_base/"
        self.NODE_BASE_PATH = self.BASE_PATH + "relay_node_base/"
        self.PORTING_PATH = "./porting_files/"
        self.CONV_PATH = self.APP_BASE_PATH + "fsw/src/convert/"

        self.cfe_dir_path = cfe_dir_path + "/relay_app"
        self.ros_dir_path = ros_dir_path + "/relay_node"

    def _create(self):
        """Create and move relay node on ROS side, relay application on CFE side."""        
        # Read setting file
        self.setting_reader._read()
        self.logger._dumpLog("Setting file read completed")

        # Copy to base about on the file that defines the data type on cFE
        self._copyPortingFiles()

        # Copy from base
        try:
            self.fop._removeDir(self.DST_PATH)
        except Exception as ex:
            pass
        self.fop._copyFiles(self.APP_BASE_PATH, self.APP_PATH)
        self.fop._copyFiles(self.NODE_BASE_PATH, self.NODE_PATH)

        # Create of relay node on ROS side
        self._createRelayNode()
        self.logger._dumpLog("Create of relay node on ROS side completed")

        # Create of relay application on CFE side
        self._createRelayApp()
        self.logger._dumpLog(
            "Create of relay application on CFE side completed")

        # Move the created relay application on CFE side and elay node on ROS side to the specified location
        self.fop._replaceDir(self.APP_PATH, self.cfe_dir_path)
        self.fop._replaceDir(self.NODE_PATH, self.ros_dir_path)

        self.fop._removeDir(self.DST_PATH)

    def _copyPortingFiles(self):
        """Copy files."""        
        self.fop._copyFiles(self.PORTING_PATH + "geometry_msgs",
                            self.CONV_PATH + "geometry_msgs")
        self.fop._copyFiles(self.PORTING_PATH + "nav_msgs",
                            self.CONV_PATH + "nav_msgs")
        self.fop._copyFiles(self.PORTING_PATH + "sensor_msgs",
                            self.CONV_PATH + "sensor_msgs")
        self.fop._copyFiles(self.PORTING_PATH + "std_msgs",
                            self.CONV_PATH + "std_msgs")
        self.fop._copyFiles(self.PORTING_PATH + "tf", self.CONV_PATH + "tf")
        self.fop._copyFiles(self.PORTING_PATH + "ardrone_autonomy",
                            self.CONV_PATH + "ardrone_autonomy")

    def _createRelayNode(self):
        """Create relay node."""

        self.node_creator._createSerializers()
        self.node_creator._convertPublisherH(self.setting_reader)
        self.node_creator._convertSubscriberH(self.setting_reader)
        self.node_creator._convertPublisherCpp(self.setting_reader)
        self.node_creator._convertSubscriberCpp(self.setting_reader)

    def _createRelayApp(self):
        """Create relay application."""
        
        ros_msg_dir_path = self.NODE_PATH + "msg/"
        cfe_ser_dir_path = self.APP_PATH + "fsw/src/cfs_serializer/"
        self.app_creator._convertCommunicationH()
        self.app_creator._convertDefineH()
        self.app_creator._createSerializers(ros_msg_dir_path, cfe_ser_dir_path)
        self.app_creator._convertPublisherCpp()
        self.app_creator._convertSubscriberCpp()
        self.app_creator._convertMakefile()


if __name__ == '__main__':
    # Create logger
    argvs = sys.argv
    argc = len(argvs)
    if not argc == 4:
        print('Invalid argument')
        exit()
    log_dir_path = argvs[1]
    cfe_dir_path = argvs[2]
    ros_dir_path = argvs[3]

    logger = relay_logger.RelayLogger(log_dir_path)

    # call RelayCreator
    relay_creator = RelayCreator(logger, cfe_dir_path, ros_dir_path)
    try:
        relay_creator._create()
    except Exception as e:
        print(e)
        t, v, tb = sys.exc_info()
        for dst_line in traceback.format_exception(t, v, tb):
            print(dst_line)
        sys.exit(1)
    sys.exit(0)
