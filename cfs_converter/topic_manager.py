"""
Copyright (c) 2020 Japan Aerospace Exploration Agency.
All Rights Reserved.

This file is covered by the LICENSE.txt in the root of this project.
"""

import json
import re
import xml.etree.ElementTree as ET
import os


class TopicManager:
    """Topic manager.
    
    Raises:
        Exception: The specified launch file was not found.
        Exception: Include of MsgId Topic.launch did not exist.
        Exception: The specified launch file was not found.
        Exception: Issuing topic failed.

    """

    create_list = {}
    convert_map = {}
    max_pipe_no = 0
    convert_file_path = ""
    data_file_path = ""
    convert_map_num = 0
    msg_id_topic_map = []

    def __init__(self, convert_file_path, data_file_path):
        """Constructor.

        Args:
            convert_file_path (str): Path to the file to be converted
            data_file_path (str): Path to the file

        """

        self.convert_file_path = convert_file_path
        self.data_file_path = data_file_path
        self._readMsgId2Topic()
        self._getTopicConvertList()
        self.convert_map_num = len(self.convert_map)

    def _readMsgId2Topic(self):
        """Read MsgId2Topic.launch file.
        
        Raises:
            Exception: The specified launch file was not found.
            Exception: Include of MsgId Topic.launch did not exist.
            Exception: The specified launch file was not found.

        """        

        # Read Connect.launch
        connect_path = "relay_base/relay_node_base/launch/Connection.launch"
        if not os.path.exists(connect_path):
            raise Exception(
                "The specified launch file ({}) was not found".format(connect_path))
        xml_data = ""
        for line in open(connect_path, "r"):
            xml_data = xml_data + line.replace("\n", "")
        root = ET.fromstring(xml_data)
        file_path = ""
        for child in root:
            if child.tag == "include":
                file_path = child.attrib["file"]
                break
        if len(file_path) == 0:
            raise Exception("Include of MsgId Topic.launch did not exist")
        # Read MsgId2Topic.launch
        msg2topic_path = "relay_base/relay_node_base/launch/" + file_path
        if not os.path.exists(msg2topic_path):
            raise Exception(
                "The specified launch file ({}) was not found".format(msg2topic_path))
        xml_data = ""
        for line in open(msg2topic_path, "r"):
            xml_data = xml_data + line.replace("\n", "")
        root = ET.fromstring(xml_data)
        for child in root:
            if len(child.attrib["msg_id"]) > 0:
                self.msg_id_topic_map.append(child.attrib["msg_id"])

    def _getTopicConvertList(self):
        """ Get topic convert list."""

        json_string = ""
        if not os.path.exists(self.convert_file_path):
            return
        for line in open(self.convert_file_path, "r"):
            json_string = json_string + line.replace("\n", "").strip()
        dec = json.loads(json_string)
        self.convert_map = dec["topics"]
        self.create_list = dec["create_list"]

    def _getMaxPipeNumber(self):
        """Get max_pipe_no."""

        json_string = ""
        for line in open(self.data_file_path, "r"):
            json_string = json_string + line.replace("\n", "").strip()
        dec = json.loads(json_string)
        self.max_pipe_no = dec["max_pipe_no"]

    def _saveTopicInfo(self, node_name, create_no):
        """Save topic info.

        Args:
            node_name (str): Node name
            create_no (int): Node ID

        """        

        convert_data = {}
        convert_data["number"] = len(self.convert_map)
        convert_data["topics"] = self.convert_map
        convert_data["create_list"] = self.create_list
        convert_data["create_list"][node_name] = create_no + 1
        wfp = open(self.convert_file_path, "w")
        json.dump(convert_data, wfp)
        wfp.close()

        data_data = {}
        data_data["max_pipe_no"] = self.max_pipe_no
        wfp = open(self.data_file_path, "w")
        json.dump(data_data, wfp)
        wfp.close()

    def _incrementMaxPipeNo(self):
        """Increament max_pipe_no."""        
        self.max_pipe_no = self.max_pipe_no + 1

    def _addMsgId(self, topic_value, json_msg_id):
        """Add message ID.

        Args:
            topic_value (str): Topic name
            json_msg_id (str): Message ID

        """

        self.convert_map[topic_value] = {
            "msg_id": json_msg_id, "direction": 0, "datatype": ""}

    def _addComDirectionAndDatatype(self, topic_value, direction, datatype):
        """Add convert_map.direction and datatype.

        Args:
            topic_value (str): Topic name
            direction (int): Sending direction of message
            datatype (str): Message type

        """        

        # 0: ROS->cFE
        # 1: cFE->ROS
        if self.convert_map[topic_value]["direction"] == 0:
            # Overwrite if convert_map.direction is 0(When multiple nodes have publisher and subscriber,
            # the relay node gives priority to publisher)
            self.convert_map[topic_value]["direction"] = direction
        self.convert_map[topic_value]["datatype"] = datatype

    def _convertTopicAndPipeName(self, topic_list, topic_datatype_list, file_path):
        """Convert topic_name and pipe_name.

        Args:
            topic_list (list): List of topic name
            topic_datatype_list (list): List of topic data type
            file_path (str): File path

        Raises:
            Exception: Issuing topic failed

        """        

        topic_map = {}
        for topic_name in topic_list:
            if topic_name.find("\"") != -1:
                # It had entered as a strings direct
                # # Convert as it
                topic_value = topic_name.replace("\"", "")
            else:
                # It had entered as a variable
                # Find a variable and get its value
                topic_line = ""
                for line in open(file_path, "r"):
                    if line.find(topic_name) != -1 and line.find(topic_name) < line.find("="):
                        topic_line = line
                        break
                topic_line = re.sub(r".+=", "", topic_line)
                topic_value = topic_line.replace(
                    ";", "").replace("\"", "").strip()
            # Update the conversion table
            topic_map[topic_name] = topic_value
            if topic_value not in self.convert_map:
                json_msg_id = ""
                has_get_msg_id = False
                for ii in range(255):
                    json_msg_id = "0x19{:0>2}".format(self.convert_map_num)
                    self.convert_map_num = self.convert_map_num + 1
                    finish_flag = True
                    for msg_id in self.msg_id_topic_map:
                        if msg_id.lower() == json_msg_id.lower():
                            finish_flag = False
                    if finish_flag:
                        has_get_msg_id = True
                        break
                if not has_get_msg_id:
                    raise Exception("Issuing topic failed")
                self._addMsgId(topic_value, json_msg_id)

        # Read the file again, convert it according to the conversion table, and create the file again
        source_line = []
        for line in open(file_path, "r"):
            cnt = 0
            for topic_name in topic_list:
                if line.find(topic_name) != -1 and line.find(topic_name) > line.find("="):
                    line = line.replace(
                        topic_name, self.convert_map[topic_map[topic_name]]["msg_id"])
                    # Determine whether to Publish or Subscribe at the relay node
                    direction = 0
                    if line.find("CONVERT_RosNodeHandleAdvertise") > -1:
                        # cFE->ROS
                        direction = 1
                    if line.find("CONVERT_RosNodeHandleSubscribe") > -1:
                        # ROS->cFE
                        direction = 0
                    topic_value = topic_map[topic_name]
                    # Stores the direction of data flow and the data type used by the relay node
                    self._addComDirectionAndDatatype(
                        topic_value, direction, topic_datatype_list[cnt])
                    break
                cnt = cnt + 1
            source_line.append(line)
        wfp = open(file_path, "w")
        for line in source_line:
            wfp.write(line)
        wfp.close()
