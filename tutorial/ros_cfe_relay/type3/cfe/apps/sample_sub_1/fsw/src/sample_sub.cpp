extern "C"{ 
    #include "cfe.h" 
    void main_sample_sub_1_();
}
#include "convert_lib.h"
/*
**
**  Copyright_sample_sub_1_ (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#include "sample_sub/sample_sub.h"
#define EVT_COUNT 1
CFE_EVS_BinFilter_t EventFilterssample_sub_1[EVT_COUNT];
int topicNo_sample_sub_1_0;

void msgCallback_sample_sub_1_(void* callback_msg_cfsconverter){
    std_msgs::Header* msg = (std_msgs::Header*)callback_msg_cfsconverter;
    msg->pointer2string();
    msg->pointer2vector();

  CFE_EVS_SendEvent(0x00, CFE_EVS_INFORMATION, "received data: %d", msg->seq); 
    msg->deleteData();
}
/**
 * Main function
 * @param argc Number of argument
 * @param argv Arguments
 * @return Result
 */
void main_sample_sub_1_(){
  EventFilterssample_sub_1[0].EventID = 0x00;
  EventFilterssample_sub_1[0].Mask = CFE_EVS_NO_FILTER;
  CONVERT_RosInit(EVT_COUNT, EventFilterssample_sub_1);


  std::function<void(void*)> f =  msgCallback_sample_sub_1_;
  topicNo_sample_sub_1_0 = CONVERT_RosNodeHandleSubscribe("0x0000000000000", 0x1900,  100, f, sizeof(std_msgs::Header));
  CONVERT_RosSpin();

}
