#!/bin/bash
finalize() {
  if [ -e ~/.cfs_relay/convert.txt ]; then
    rm ~/.cfs_relay/convert.txt
  fi
  if [ -e ~/.cfs_relay/data.txt ]; then
    rm ~/.cfs_relay/data.txt
  fi
  if [ -e ~/.cfs_relay/msg_list.json ]; then
    rm ~/.cfs_relay/msg_list.json
  fi
}

# defines
INI_FILE=config.ini

SECTION=setting

# read config by ini file
is_read=0
cfe_path="./apps"
ros_path="./src"
log_dir_path="./"
create_relay_flag="1"

while read line
do
  is_section=`echo "${line}" | grep "\[${SECTION}\]"`
  is_bracket=`echo "${line}" | grep -E "\[.+\]"`
  is_comment=`echo "${line}" | grep ";"`
  if [ ${#is_section} -gt 0 ]; then
    is_read=1
  elif [ ${#is_bracket} -gt 0 ]; then
    is_read=0
  elif [ ${#is_comment} -gt 0 ]; then
    continue
  fi
  if [ ${is_read} -eq 1 ]; then
    is_cfe_path=`echo "${line}" | grep "cfe_path="`
    is_ros_path=`echo "${line}" | grep "ros_path="`
    is_log_dir_path=`echo "${line}" | grep "log_dir_path="`
    is_create_relay_flag=`echo "${line}" | grep "create_relay_flag="`
    if [ ${#is_cfe_path} -gt 0 ]; then
      cfe_path=`echo "${line}" | sed -e "s/cfe_path=//g"`
      echo "cfe_path=${cfe_path}"
    elif [ ${#is_ros_path} -gt 0 ]; then
      ros_path=`echo "${line}" | sed -e "s/ros_path=//g"`
      echo "ros_path=${ros_path}"
    elif [ ${#is_log_dir_path} -gt 0 ]; then
      log_dir_path=`echo "${line}" | sed -e "s/log_dir_path=//g"`
      echo "log_dir_path=${log_dir_path}"
    fi
  fi
done < ${INI_FILE}

python3 relay_creator.py ${log_dir_path} ${cfe_path} ${ros_path}
if [ $? == "1" ]; then
  finalize
  echo "error occurred by create relay_node and relay_app."
  exit -1
fi

# delete work files
finalize

# rm *.pyc
# rm *.log

echo "Finish creating cFS relay app & ROS relay node"
exit 0