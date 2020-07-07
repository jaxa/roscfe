#!/bin/bash
finalize() {
  if [ -e ~/.cfs_converter/convert.txt ]; then
    rm ~/.cfs_converter/convert.txt
  fi
  if [ -e ~/.cfs_converter/data.txt ]; then
    rm ~/.cfs_converter/data.txt
  fi
  if [ -e ~/.cfs_converter/msg_list.json ]; then
    rm ~/.cfs_converter/msg_list.json
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
list_file_path="./convert_list.txt"
create_relay_flag="1"
main_func_path="./main_func_name.yaml"

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
    is_target_path=`echo "${line}" | grep "target_path="`
    is_create_relay_flag=`echo "${line}" | grep "create_relay_flag="`
    is_main_func_path=`echo "${line}" | grep "main_func_path="`
    if [ ${#is_cfe_path} -gt 0 ]; then
      cfe_path=`echo "${line}" | sed -e "s/cfe_path=//g"`
    elif [ ${#is_ros_path} -gt 0 ]; then
      ros_path=`echo "${line}" | sed -e "s/ros_path=//g"`
    elif [ ${#is_log_dir_path} -gt 0 ]; then
      log_dir_path=`echo "${line}" | sed -e "s/log_dir_path=//g"`
    elif [ ${#is_target_path} -gt 0 ]; then
      list_file_path=`echo "${line}" | sed -e "s/target_path=//g"`
    elif [ ${#is_create_relay_flag} -gt 0 ]; then
      create_relay_flag=`echo "${line}" | sed -e "s/create_relay_flag=//g"`
    elif [ ${#is_main_func_path} -gt 0 ]; then
      main_func_path=`echo "${line}" | sed -e "s/main_func_path=//g"`
    fi
  fi
done < ${INI_FILE}

# read target ROS package path by list file
while read line
do
  is_comment=`echo ${line} | grep "#"`
  if [ ${#is_comment} -gt 0 ]; then
    continue
  fi
  python3 cfs_converter.py ${cfe_path} ${log_dir_path} ${main_func_path} ${line}
  if [ $? == "1" ]; then
    finalize
    for i in ${line}
    do
      path=${i}
      break
    done
    echo "error occurred by convert ROS Node(${path})."
    exit -1
  fi
done < ${list_file_path}

if [ ${create_relay_flag} -eq "1" ]; then
  python3 relay_creator.py ${log_dir_path} ${cfe_path} ${ros_path}
  if [ $? == "1" ]; then
    finalize
    echo "error occurred by create relay_node and relay_app."
    exit -1
  fi
fi

# delete work files
finalize

rm *.pyc
# rm *.log

echo "finish cfs converter"
exit 0