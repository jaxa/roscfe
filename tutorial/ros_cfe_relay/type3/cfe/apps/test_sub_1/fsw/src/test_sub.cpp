extern "C"{ 
    #include "cfe.h" 
    void main_test_sub_1_();
}
#include "convert_lib.h"
/*
**
**  Copyright_test_sub_1_ (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#include "test_sub/test_sub.h"
#define EVT_COUNT 1
CFE_EVS_BinFilter_t EventFilterstest_sub_1[EVT_COUNT];
int topicNo_test_sub_1_0;

void msgCallback_test_sub_1_(void* callback_msg_cfsconverter){
    geometry_msgs::Pose* msg = (geometry_msgs::Pose*)callback_msg_cfsconverter;
    msg->pointer2string();
    msg->pointer2vector();

  CFE_EVS_SendEvent(0x00, CFE_EVS_INFORMATION, "received data: %lf", msg->position.x); 
    msg->deleteData();
}
/**
 * Main function
 * @param argc Number of argument
 * @param argv Arguments
 * @return Result
 */
void main_test_sub_1_(){
  EventFilterstest_sub_1[0].EventID = 0x00;
  EventFilterstest_sub_1[0].Mask = CFE_EVS_NO_FILTER;
  CONVERT_RosInit(EVT_COUNT, EventFilterstest_sub_1);


  std::function<void(void*)> f =  msgCallback_test_sub_1_;
  topicNo_test_sub_1_0 = CONVERT_RosNodeHandleSubscribe("0x0000000000001", 0x1901,  100, f, sizeof(geometry_msgs::Pose));
  CONVERT_RosSpin();

}
