"""
Copyright (c) 2020 Japan Aerospace Exploration Agency.
All Rights Reserved.

This file is covered by the LICENSE.txt in the root of this project.
"""
# import topic_manager

import xml.etree.ElementTree as ET


class CfeSetting:
    """Cfe setting."""  
  
    ip_addr = ""
    port = 0
    port_recv = 0
    port_send = 0
    ring_buf_len = 0

    def __init__(self):
        """Constructor."""

        pass


class Correspond:
    """Correspond."""

    topic = ""
    msg_id = ""
    cfe_data_type = ""
    ros_data_type = ""
    sender = 1
    SENDER_ROS = 0
    SENDER_CFE = 1

    def __init__(self):
        """Constructor."""

        pass


class RelaySettingReader:
    """Read relay settings."""

    BASE_DIR_PATH = ""
    CON_LAUNCH_FILE_PATH = ""
    MSG_LAUNCH_FILE_PATH = ""

    ROS_TAG = ""
    CFE_TAG = ""

    cfe_setting = None
    topic_manager = None
    corresponds = []

    def __init__(self, topic_manager):
        """Constructor.
        
        Args:
            topic_manager (TopicManager): TopicManager object

        """

        self.LAUNCH_DIR_PATH = "./relay_base/relay_node_base/launch/"
        self.CON_LAUNCH_FILE_PATH = self.LAUNCH_DIR_PATH + "Connection.launch"
        self.CFE_TAG = "cfe_relay_app"
        self.cfe_setting = CfeSetting()
        self.corresponds = []
        self.topic_manager = topic_manager

    def _read(self):
        """Read relay settings."""

        tree = ET.ElementTree(file=self.CON_LAUNCH_FILE_PATH)
        root = tree.getroot()
        msg_file_path = ""

        for launch in root:
            if (launch.tag == "cfe_relay_app"):
                self.cfe_setting.port = launch.attrib["port"]
                self.cfe_setting.port_recv = launch.attrib["port"]
                self.cfe_setting.port_send = launch.attrib["port_send"]
                self.cfe_setting.ip_addr = launch.attrib["ip_addr"]
                self.cfe_setting.ring_buf_len = launch.attrib["ring_length"]
            if (launch.tag == "include"):
                msg_file_path = self.LAUNCH_DIR_PATH + launch.attrib["file"]

        self._read_msgid_to_topic(msg_file_path)

    def _read_msgid_to_topic(self, file_path):
        """Read message id.

        Args:
            file_path (str): File path

        """

        tree = ET.ElementTree(file=file_path)
        root = tree.getroot()

        for launch in root:
            if (launch.tag == "correspond"):
                correspond = Correspond()
                correspond.topic = launch.attrib["topic"]
                correspond.msg_id = launch.attrib["msg_id"]
                correspond.cfe_data_type = launch.attrib["cfe_data_type"]
                correspond.ros_data_type = launch.attrib["ros_data_type"]
                if launch.attrib["sender"] == "0":
                    correspond.sender = Correspond.SENDER_ROS
                else:
                    correspond.sender = Correspond.SENDER_CFE
                self.corresponds.append(correspond)

        # Get Topic, msg_id, data type, communication direction of converted ROS side node
        self.topic_manager._getTopicConvertList()
        for key, value in self.topic_manager.convert_map.items():
            correspond = Correspond()
            correspond.topic = str(key)
            correspond.msg_id = str(value["msg_id"])
            correspond.cfe_data_type = str(value["datatype"])
            correspond.ros_data_type = str(value["datatype"])
            correspond.sender = value["direction"]
            self.corresponds.append(correspond)

    def _getDataTypePkgList(self):
        """Get data type of pakege list.

        Returns:
            list: List of packages

        """

        pkg_list = []
        for correspond in self.corresponds:
            parts = correspond.ros_data_type.split("::")
            if len(parts) == 2:
                pkg_list.append(parts[0])
        return pkg_list
