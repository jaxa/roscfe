extern "C"{ 
    #include "cfe.h" 
    void main_sample_pub_1_();
}
#include "convert_lib.h"
/*
**
**  Copyright_sample_pub_1_ (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/

#include "sample_pub/sample_pub.h"
#define EVT_COUNT 1
CFE_EVS_BinFilter_t EventFilterssample_pub_1[EVT_COUNT];
int topicNo_sample_pub_1_0;

/**
 * Main function
 * @param argc Number of arguments
 * @param argv Arguments
 * @return Result
 */
void main_sample_pub_1_(){
  EventFilterssample_pub_1[0].EventID = 0x00;
  EventFilterssample_pub_1[0].Mask = CFE_EVS_NO_FILTER;
  CONVERT_RosInit(EVT_COUNT, EventFilterssample_pub_1);


  topicNo_sample_pub_1_0 = CONVERT_RosNodeHandleAdvertise("0x0000000000000", 0x1900, 100);
  uint32   loop_rate
 = CONVERT_RosRate(100);
  int count = 0;
  while (CONVERT_RosOk()) {
    std_msgs::Header msg;
    msg.seq = count;
    CFE_EVS_SendEvent(0x00, CFE_EVS_INFORMATION, "send msg.data: %d, rostimenow.toSec: %f", msg.seq, CONVERT_RosTimeNowToSec());
    {
        msg.vector2pointer();
        msg.string2pointer();
        CONVERT_RosPublisherPublish((CFE_SB_Msg_t*)(&msg), sizeof(msg),topicNo_sample_pub_1_0);
    }
    CONVERT_RosRateSleep(loop_rate, topicNo_sample_pub_1_0);
    count++;
  }

}
