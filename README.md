# ROScFE project

This repository is a collection of libraries and tools which enable to implement Robot Operating System (ROS) nodes into  core Flight Executives (cFE). cFE is a core component of NASA-supplied spacecraft software, Core Flight System (CFS).

This project provides the following 3 components,
* CFS converter
* ROS/cFE interface library
* ROS/cFE network bridge

## License
This project is under the Apache 2.0 license. See LICENSE.txt.

## Release Notes
* ROScFE project suite 1.0.0 is released. 

## Directory List
* cfs_converter:    
    The Convertor that translates ROS nodes into CFS apps.
* convert_lib:  
    The library that is built with converted ROS nodes in CFS. This library provides the fundamental functions required from converted ROS nodes.  
* sample_pub:    
    Sample ROS publisher node used for conversion test.
* sample_sub:  
    Sample ROS subscriber node used for conversion test.
* hk_pub:    
    Sample ROS publisher node communicating with HK app.
* hk_sub:  
    Sample ROS subscriber node communicating with HK app.

## Dependency
* CFS  
    - [cFE 6.5.0](https://sourceforge.net/projects/coreflightexec/files/cFE-6.5.0/)

    - [OSAL 4.2.1a](https://sourceforge.net/projects/osal/files/osal-4.2.1a-release.tar.gz/download)

    - [HK App 2.4.1](https://sourceforge.net/projects/cfs-hk/files/HK-Version241.tar.gz/download)

* For build on linux 
    - [ROS Kinetic](http://wiki.ros.org/kinetic)  
    - gcc-multilib
    - g++-multilib
    - build-essential
    - libc-dev-i386
    - libstdc++6-4.7-dev:i386
    - Python 3.6+
        - pyyaml


## Setup on Linux
* Prerequisite: [ROS kinetic is installed](http://wiki.ros.org/kinetic/Installation/Ubuntu).

* Clone this repository.

    ```
    cd [your working directory]  
    git clone [repository URL]
    ```

* Get cFE and OSAL source and extract them on your working directory.

    ```
    tar -zxf cFE-6.5.0-OSS-release.tar.gz -C [your working directory]
    tar -zxf osal-4.2.1a-release.tar.gz -C [your working directory]
    ```

* Move extracted OSAL directory into the cFE directory as "osal" directory.   
    
    ```
    cd [your working directory]  
    cp -pr osal-4.2.1a-release/* cFE-6.5.0-OSS-release/osal/
    ```

* Change directory to cfs_converter/.
    
    ```
    cd roscfe/cfs_converter
    ```

* Edit "config.ini" according to your environment.  
  **[optional]** Set "create_relay_flag" to 1 if ROS/cFE network bridge is used.
    
    ```
    ...  
    cfe_path=[your working directory]/cFE-6.5.0-OSS-release/apps  
    ...  
    ros_path=[your catkin workspace]/src  
    ...  
    create_relay_flag=1     ; =1 : when ROS/cFE network bridge is used  
    ...  
    ```

* Add your ROS package path, launch file name, and ROS node name activated from launch file into "convert_list.txt".  
    Ex)
    If your ROS workspace is as follows, 
    
    ```
    catkin_ws  
        └─src  
           └─my_ros_package  
              ├─launch  
              |   └─bringup.launch  
              └─src  
              |   └─XXX.cpp  
              └─include  
                  └─YYY.h  
    ```     

    then edit "convert_list.txt" as follows.
    
    ```  
    # [ros_pkg path] [launch file name] [name of launch node(*)]  
    /home/test_user/catkin_ws/src/my_ros_package bringup.launch my_node   

    # OR  
    /home/test_user/catkin_ws/src/my_ros_package
    ```

    (*) [name of launch node] must be equal to the value of 'name' attribute in launch file.

* **[optional]** If you want to use ROS/cFE network bridge, edit "relay_base/relay_node_base/launch/Connection.launch". Available parameters are as follows.
    - protocol：tcp/udp
    - port：port-number for receiving messages
    - port_send：destination's port-number for sending messages
    - ip_addr：IP address of destination 
    - ring_length：buffer size for receiving messages

* **[optional]** If you want to use ROS/cFE network bridge, edit "relay_base/relay_node_base/launch/MsgId2Topic.launch". Available parameters are as follows.
    - topic：topic name published/subscribed by "ros_relay_node"
    - ros_data_type：ROS message type
    - msg_id：cFE message id used for publication/subscription on "cfe_relay_app"
    - cfe_data_type：cFE data type
    - sender：=0 when ROS-->cFE / =1 when cFE-->ROS

* Add execution authority to "start_cfs_converter.sh", run it.
    ```
    chmod +x start_cfs_converter.sh  
    ./start_cfs_converter.sh
    ```
  "start_cfs_converter.sh" generates 
    - "converted ROS node" at the path specified by cfe_path in "config.ini" 
    - **[optional]** "ros_relay_node" at the path specified by ros_path in "config.ini"
    - **[optional]** "cfe_relay_app" at the path specified by cfe_path in "config.ini"

* Copy convert_lib directory into apps directory of cFE.

    ```
    cp -pr [your working directory]/roscfe/convert_lib [your working directory]/cFE-6.5.0-OSS-release/apps/ 
    ```

* Move to the directory including <a id="app_makefile">"Makefile"</a> at "converted ROS node" in cFE.

    ```
    cd [your working directory]/cFE-6.5.0-OSS-release/apps/["converted ROS node"]/fsw/for_build/
    ```
    Modify "OBJS" and ".cpp.o" fields in "Makefile".
    Ex)
    ```
    OBJS = my_node.o /usr/lib/i386-linux-gnu/libstdc++.so.6
    ``` 
        
    ```
    .cpp.o:  
        $(COMPILER) $(LOCAL_COPTS) -std=c++11 -m32 -Wall $(INCLUDE_PATH) -g -O0 -DOS_DEBUG_LEVEL=$(DEBUG_LEVEL) -c -o my_node [cpp file path] 
    ```

* Move to the directory including "Makefile" at cFE build directory.
    
    ```
    cd [your working directory]/cFE-6.5.0-OSS-release/build/cpu1/Makefile
    ```
    Modify "THE_APPS" as follows,

    ```
    THE_APPS := convert_lib ["converted ROS node" DIRECTORY NAME]
    ```

    **[optional]** If you build "cfe_relay_app", add it into "THE_APPS"

    ```
    THE_APPS := convert_lib relay_app ["converted ROS node" DIRECTORY NAME]
    ```

* Move to the directory including "cfe_es_startup.scr".

    ```
    cd [your working directory]/cFE-6.5.0-OSS-release/build/cpu1/exe/cfe_es_startup.scr
    ```

    Add the activation-setting-line for "converted ROS node".  
    Ex)

    ```
    CFE_LIB, /cf/apps/convert_lib.so,   CONVERT_LibInit,CONVERT_LIB,  0,   0,    0x0, 0;
    CFE_APP, /cf/apps/["converted ROS node" directory name].so, [ENTRY_PT(*)],[name of app], [priority(**)], 8192, 0x0, 0;
    ```

    (*): It is defined at <a href="#app_makefile">"Makefile" modified at previous step</a>.  
    (**): The smaller number it is, the higher priority the app has.  
    **[optional]** If "cfe_relay_app" is added, set the lowest priority to it.

    ```
    CFE_LIB, /cf/apps/convert_lib.so,   CONVERT_LibInit,CONVERT_LIB,  0,   0,    0x0, 0;
    CFE_APP, /cf/apps/sample_sub_1.so,     sub_main_1_, SAMPLE_SUB,  10,   8192, 0x0, 0;
    CFE_APP, /cf/apps/relay_app.so, RELAY_AppMain,RELAY_APP, 20, 0, 0x0, 0;
    ```

## Build & Run on Linux 

### [optional] ROS build & run
If you want to run "ros_relay_app", follow the below instructions.
- Move to your ros workspace, execute build commands.

    ```
    cd [your catkin workspace]  
    catkin_make clean  
    catkin_make -i  
    catkin_make
    ```

- Execute  bellow command. 
    
    ```
    rosrun relay_node relay_node
    rosrun [my_ros_pkg] [my_ros_node] 
    ```

### cFE build & run
- Execute bellow commands.

    ```
    cd [your working directory]/cFE-6.5.0-OSS-release  
    . ./setvars.sh
    ```

- Remove old-build-directories.

    ``` 
    cd build/cpu1  
    rm -rf [old-build-directores]
    ```

- Execute build commands.

    ```
    make config  
    make 
    ```

- Run cFE apps.

    ```
    cd exe  
    sudo su  
    ./core-linux.bin
    ```

## Reference

* [Hiroki Kato, et al. "Software Framework for ROS Implementation into cFS." Flight Software Workshop 2019.](https://drive.google.com/file/d/1DGwODJcouwu5GrGTkGLvnyM2FXB7IiDx/view)