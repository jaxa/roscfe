/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
/*************************************************************************
** File: sample_lib.h
**
** Purpose:
**   Specification for the sample library functions.
**
*************************************************************************/
#ifndef _convert_lib_h_
#define _convert_lib_h_

/************************************************************************
** Includes
*************************************************************************/
extern "C" {
#include "cfe.h"
}
#ifdef _LINUX_
#include <functional>
#include <math.h>
#include <stdbool.h>
#include <string>
#include <thread>
#include <vector>
#else
// Real machine environment include
#endif

#include "std_msgs/RosTime.h"

#define RCV_MSG_TIMEOUT 0
#define MONITOR_INTERVAL 0.001
#define SPIN_INTERVAL 0.001

/************************************************************************
** Type Definitions
*************************************************************************/
typedef struct {
  CFE_SB_PipeId_t CmdPipe;
  const char *PipeName;
  CFE_SB_MsgId_t MsgId;
  std::function<void(void *)> SubFuncPtr;
  double Interval;
  bool IsSub;
  unsigned int MsgSize;
  double CalledRosRateSleepTime;
} PipeInfo_t;

typedef struct {
  uint16 TopicNo;
  CFE_SB_MsgPtr_t MsgPtr;
  unsigned int MsgSize;
  bool IsTreated;
} CallbackInfo_t;

/*************************************************************************
** Exported Functions
*************************************************************************/
extern "C" {
// Prevent function names from being changed, because to call from cFE
int32 CONVERT_LibInit(void);
}
void CONVERT_RosInit(uint16 EventCount, CFE_EVS_BinFilter_t *EvtFilters);
double CONVERT_RosTimeNowToSec(void);
bool CONVERT_RosOk(void);
void CONVERT_RosSpinOnce();
void CONVERT_RosSpin();
uint32 CONVERT_RosRate(int32 Frequency);
void CONVERT_RosRateSleep(uint32 Interval, uint16 TopicNo);
void CONVERT_RosDurationSleep(double sec);

uint16 CONVERT_RosNodeHandleSubscribe(const char *PipeName,
                                      CFE_SB_MsgId_t MsgId, int32 QueueSize,
                                      std::function<void(void *)> FuncPtr,
                                      unsigned int MsgSize);
void CONVERT_RosPublisherPublish(CFE_SB_Msg_t *MsgPtr, uint16 Length,
                                 uint16 TopicNo);
uint16 CONVERT_RosNodeHandleAdvertise(const char *PipeName,
                                      CFE_SB_MsgId_t MsgId, int32 QueueSize);
void CONVERT_ReallocPipeInfo(const char *PipeName, int16 TopicNo);
void CONVERT_Lock(void);
void CONVERT_Unlock(void);
void CONVERT_MonitorRcvMsg(void);
RosTime CONVERT_RosTimeNow(void);
std::string CONVERT_RosPackageGetPath(const std::string &AppName);

#endif /* _convert_lib_h_ */

/************************/
/*  End of File Comment */
/************************/
