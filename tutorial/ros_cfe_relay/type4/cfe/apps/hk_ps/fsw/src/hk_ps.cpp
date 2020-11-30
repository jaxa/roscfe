/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#include <hk_ps.h>

#include <sys/time.h>
#include <time.h>

typedef struct {
  uint8 TlmHeader[CFE_SB_TLM_HDR_SIZE];
  uint8 CmdCounter;
  uint8 ErrCounter;
  uint16 Padding;
  uint16 CombinedPacketsSent;
  uint16 MissingDataCtr;
  uint32 MemPoolHandle;
} HK_HkPacket_t;

typedef struct { uint8 TlmHeader[CFE_SB_TLM_HDR_SIZE]; } HK_Send_t;

/**
 * Callback
 */
void msgCallback(void *msg) {
  const HK_HkPacket_t *data = (const HK_HkPacket_t *)msg;
  // printf("[HK_PS] message received. CmdCounter: %d, ErrCounter: %d \n",
  // data->CmdCounter, data->ErrCounter);
  struct timeval myTime;
  struct tm *time_st;
  gettimeofday(&myTime, NULL);
  time_st = localtime(&myTime.tv_sec);
  printf("[HK_Driver][%02d:%02d.%06d] subscribe 0x089B\n", time_st->tm_min,
         time_st->tm_sec, myTime.tv_usec);
}

/**
 * Main.
 */
void HK_PS_AppMain() {
  // OS_printf("[HK_PS] HK_PS_AppMain is started. \n");
  CONVERT_RosInit(0, NULL);

  TopicNo1_ = CONVERT_RosNodeHandleAdvertise("0x00hk_ps", 0x189B, 100);
  std::function<void(void *)> f = msgCallback;
  // uint16 TopicNo2_ = CONVERT_RosNodeHandleAdvertise("0x02hk_ps", 0x189A,
  // 100);
  CONVERT_RosNodeHandleSubscribe("0x01hk_ps", 0x089B, 100, f,
                                 sizeof(HK_HkPacket_t));

  int executeRate = 1;

  uint32 loop_rate = CONVERT_RosRate(executeRate);
  while (CONVERT_RosOk()) {
    getchar();
    CFE_SB_CmdHdr_t msg;
    msg.Sec.Command = 0;

    CONVERT_RosPublisherPublish((CFE_SB_Msg_t *)(&msg), sizeof(CFE_SB_CmdHdr_t),
                                TopicNo1_);
    // OS_printf("[HK_PS] send msg to TopicNo1_: %d, rostimenow.toSec: %f \n",
    // msg.Sec.Command, CONVERT_RosTimeNowToSec());
    // CONVERT_RosPublisherPublish((CFE_SB_Msg_t*)(&msg),
    // sizeof(CFE_SB_CmdHdr_t), TopicNo2_);
    // OS_printf("[HK_PS] send msg to TopicNo2_: %d, rostimenow.toSec: %f \n",
    // msg.Sec.Command, CONVERT_RosTimeNowToSec());
    struct timeval myTime;
    struct tm *time_st;
    gettimeofday(&myTime, NULL);
    time_st = localtime(&myTime.tv_sec);
    printf("[HK_Driver][%02d:%02d.%06d] publish 0x189B\n", time_st->tm_min,
           time_st->tm_sec, myTime.tv_usec);

    CONVERT_RosSpinOnce();
    CONVERT_RosRateSleep(loop_rate, TopicNo1_);
  }
  OS_printf("[HK_PS] HK_PS_AppMain is finished. \n");
}
