/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
/*************************************************************************
** File:
**   $Id: convert_lib.c $
**
** Purpose:
**   Convert CFS library
**
*************************************************************************/

/*************************************************************************
** Includes
*************************************************************************/
#include "convert_lib.h"
#include "convert_lib_version.h"

#include "malloc.h"
#include <memory.h>
#include <pthread.h>
#include <sys/types.h>
#include <unistd.h>

/*************************************************************************
** Member Definitions
*************************************************************************/
#define RING_BUF_SIZE 100
#define FILE_PATH_MAX_LENGTH 1023
#define PATH_SEPARATOR "/"
PipeInfo_t *PipeInfo_;
uint16 TopicCount_ = 0;
CallbackInfo_t CallbackInfo_[RING_BUF_SIZE];
uint16 CallbackCount_ = 0;
pthread_t ThreadId = NULL;
pthread_mutex_t Mutex_ = PTHREAD_MUTEX_INITIALIZER;

/*************************************************************************
** Macro Definitions
*************************************************************************/

/**
 * Initialize and show version.
 */
int32 CONVERT_LibInit(void) {
  OS_printf("CONVERT Lib Initialized.  Version %d.%d.%d.%d\n",
            CONVERT_LIB_MAJOR_VERSION, CONVERT_LIB_MINOR_VERSION,
            CONVERT_LIB_REVISION, CONVERT_LIB_MISSION_REV);

  PipeInfo_ = (PipeInfo_t *)calloc(1, sizeof(PipeInfo_t));
  for (int ii = 0; ii < RING_BUF_SIZE; ii++) {
    CallbackInfo_[ii].TopicNo = -1;
    CallbackInfo_[ii].MsgPtr = NULL;
    CallbackInfo_[ii].MsgSize = 0;
    CallbackInfo_[ii].IsTreated = true;
  }
  CallbackCount_ = 0;

  if (!ThreadId) {
    std::thread t1(CONVERT_MonitorRcvMsg);
    t1.detach();
    ThreadId = 1;
  }

  return CFE_SUCCESS;

} /* End CONVERT_LibInit */

/**
 * Regist Application to cFE and set Event Service.
 * @param Number of event filters.
 * @param Event filters.
 */
void CONVERT_RosInit(uint16 EventCount, CFE_EVS_BinFilter_t *EvtFilters) {
  uint32 ResetSubType;

  CFE_ES_RegisterApp();

  CFE_ES_GetResetType(&ResetSubType);

  CFE_EVS_Register(EvtFilters, EventCount, CFE_EVS_BINARY_FILTER);
}

/**
 * Get second.
 * @return Second.
 */
double CONVERT_RosTimeNowToSec(void) {
  CFE_TIME_SysTime_t CurrentTime = CFE_TIME_GetTime();
  return (double)CurrentTime.Seconds +
         pow(2.0, -32) * (double)CurrentTime.Subseconds;
}

/**
 * Get publish continue or not.
 * @return Check result from run status.
 */
bool CONVERT_RosOk(void) {
  uint32 RunStatus = CFE_ES_APP_RUN;
  return CFE_ES_RunLoop(&RunStatus);
}

/**
 * Execute function for received cfe message.
 */
void CONVERT_RosSpinOnce() {
  int8 Count = 0;
  CONVERT_Lock();
  // CallbackCount_ -1 is the newest data
  // CallbackCount_ is the oldest data
  // The first thing to process is the CallbackCount_ data
  for (Count = CallbackCount_; Count < RING_BUF_SIZE; Count++) {
    if (CallbackInfo_[Count].IsTreated == false) {
      uint16 topicNo = CallbackInfo_[Count].TopicNo;
      PipeInfo_[topicNo].SubFuncPtr(CallbackInfo_[Count].MsgPtr);
      free(CallbackInfo_[Count].MsgPtr);
      CallbackInfo_[Count].IsTreated = true;
    }
  }
  for (Count = 0; Count < CallbackCount_; Count++) {
    if (CallbackInfo_[Count].IsTreated == false) {
      uint16 topicNo = CallbackInfo_[Count].TopicNo;
      PipeInfo_[topicNo].SubFuncPtr(CallbackInfo_[Count].MsgPtr);
      free(CallbackInfo_[Count].MsgPtr);
      CallbackInfo_[Count].IsTreated = true;
    }
  }
  CONVERT_Unlock();
}

/**
 * Execute function for received cfe message and repeat.
 */
void CONVERT_RosSpin() {
  while (CONVERT_RosOk()) {
    CONVERT_RosSpinOnce();
  }
}

/**
 * Set Waiting frequency.
 * @param Waiting frequency.
 */
uint32 CONVERT_RosRate(int32 Frequency) {
  uint32 Interval = 0;
  if (Frequency != 0) {
    Interval = (uint32)(1000 / Frequency);
  } else {
    OS_printf("Inputed zero. \n");
    Interval = 1;
  }
  return Interval;
}

/**
 * Wait by setting(CONVERT_RosRate).
 * @param Waiting interval[msec].
 * @param Topic number
 */
void CONVERT_RosRateSleep(uint32 Interval, uint16 TopicNo) {
  // It records the time of the previous calling,
  // and uses the difference between the previous call time and the current call
  // time --- (1)
  // and waits for the time of Interval-(1)
  double NowTime = CONVERT_RosTimeNowToSec();
  double DiffTime = NowTime - PipeInfo_[TopicNo].CalledRosRateSleepTime;
  double WaitingTime =
      Interval -
      (DiffTime * 1000); // DiffTime modified from seconds to milliseconds
  if (WaitingTime < 0.0) {
    WaitingTime = 0.0;
  }
  // Rounded off because less than 1.0 will be 0
  OS_TaskDelay((uint32)(floor(WaitingTime + 0.5)));
  PipeInfo_[TopicNo].CalledRosRateSleepTime = CONVERT_RosTimeNowToSec();
}

/**
 * Wait by argument.
 * @param Waiting time(second).
 */
void CONVERT_RosDurationSleep(double sec) {
  OS_TaskDelay((uint32)(sec * 1000));
}

/**
 * Monitoring published message.
 */
void CONVERT_MonitorRcvMsg(void) {
  int32 Count;
  while (TRUE) {
    // Judge only Subscribe and refer to it
    for (Count = 0; Count < TopicCount_; Count++) {
      if (PipeInfo_[Count].IsSub == TRUE) {
        int32 Status;
        CFE_SB_MsgPtr_t MsgPtr;
        Status =
            CFE_SB_RcvMsg(&MsgPtr, PipeInfo_[Count].CmdPipe, RCV_MSG_TIMEOUT);
        if (Status == CFE_SUCCESS) {
          CFE_SB_MsgId_t MsgId;
          MsgId = CFE_SB_GetMsgId(MsgPtr);

          if (MsgId == PipeInfo_[Count].MsgId) {
            CONVERT_Lock();
            if (CallbackInfo_[CallbackCount_].IsTreated == false) {
              // If it is unprocessed data, release memory etc.
              free(CallbackInfo_[CallbackCount_].MsgPtr);
            }
            CallbackInfo_[CallbackCount_].TopicNo = Count;
            CallbackInfo_[CallbackCount_].MsgPtr =
                (CFE_SB_MsgPtr_t)malloc(PipeInfo_[Count].MsgSize);
            memcpy(CallbackInfo_[CallbackCount_].MsgPtr, MsgPtr,
                   PipeInfo_[Count].MsgSize);
            CallbackInfo_[CallbackCount_].MsgSize = PipeInfo_[Count].MsgSize;
            if (CallbackInfo_[CallbackCount_].IsTreated == false) {
              OS_printf("[WARN] CallbackInfo is rewrote.\n");
            }
            CallbackInfo_[CallbackCount_].IsTreated = false;
            CallbackCount_++;
            CallbackCount_ = CallbackCount_ % RING_BUF_SIZE;
            CONVERT_Unlock();
          }
        }
      }
    }
  }
}

/**
 * Add Pipe Information.
 * @param Pipe name.
 * @param Topic number.
 */
void CONVERT_ReallocPipeInfo(const char *PipeName, int16 TopicNo) {
  PipeInfo_ =
      (PipeInfo_t *)realloc(PipeInfo_, sizeof(PipeInfo_t) * (TopicNo + 1));
  if (PipeInfo_ == NULL) {
    OS_printf("realloc is failed. \n");
    exit(1);
  }
  PipeInfo_[TopicNo].PipeName = PipeName;
  PipeInfo_[TopicNo].CmdPipe = 0;
  PipeInfo_[TopicNo].MsgId = 0;
  PipeInfo_[TopicNo].IsSub = FALSE;
  PipeInfo_[TopicNo].CalledRosRateSleepTime = 0.0;
}

/**
 * Set subscribe.
 * @param Pipe name.
 * @param Subscribe Message Id.
 * @param Queue size.
 * @param Call function address when subscribe.
 * @param Size of Message.
 */
uint16 CONVERT_RosNodeHandleSubscribe(const char *PipeName,
                                      CFE_SB_MsgId_t MsgId, int32 QueueSize,
                                      std::function<void(void *)> FuncPtr,
                                      unsigned int MsgSize) {
  int32 Status;

  CONVERT_ReallocPipeInfo(PipeName, TopicCount_);

  Status =
      CFE_SB_CreatePipe(&PipeInfo_[TopicCount_].CmdPipe, QueueSize, PipeName);
  if (Status != CFE_SUCCESS) {
    OS_printf("Error: Creating SB Pipe is failed by "
              "CONVERT_RosNodeHandleSubscribe. \n");
    exit(1);
  }

  Status = CFE_SB_Subscribe(MsgId, PipeInfo_[TopicCount_].CmdPipe);
  if (Status) {
    OS_printf("Error: Subscribe is failed. \n");
    exit(1);
  }
  PipeInfo_[TopicCount_].MsgId = MsgId;
  PipeInfo_[TopicCount_].SubFuncPtr = FuncPtr;
  PipeInfo_[TopicCount_].IsSub = TRUE;
  PipeInfo_[TopicCount_].MsgSize = MsgSize;

  TopicCount_++;
  return (TopicCount_ - 1);
}

/**
 * Publish to created pipe.
 * @param Publish message.
 * @param Length.
 * @param Topic number.
 */
void CONVERT_RosPublisherPublish(CFE_SB_Msg_t *MsgPtr, uint16 Length,
                                 uint16 TopicNo) {
  int32 Status;
  if (PipeInfo_[TopicNo].MsgId != 0) {
    CFE_SB_InitMsg(MsgPtr, PipeInfo_[TopicNo].MsgId, Length, FALSE);

    Status = CFE_SB_SendMsg(MsgPtr);
    if (Status != CFE_SUCCESS) {
      OS_printf("Error: Publish is failed. \n");
    }
  }
}

/**
 * Create pipe for publish.
 * @param Message Id.
 * @param Queue size.
 * @return Topic number
 */
uint16 CONVERT_RosNodeHandleAdvertise(const char *PipeName,
                                      CFE_SB_MsgId_t MsgId, int32 QueueSize) {
  int32 Status;

  CONVERT_ReallocPipeInfo(PipeName, TopicCount_);

  Status =
      CFE_SB_CreatePipe(&PipeInfo_[TopicCount_].CmdPipe, QueueSize, PipeName);
  if (Status != CFE_SUCCESS) {
    OS_printf("Error: Creating SB Pipe is failed by "
              "CONVERT_RosNodeHandleAdvertise. \n");
    exit(1);
  }
  PipeInfo_[TopicCount_].MsgId = MsgId;

  TopicCount_++;
  return (TopicCount_ - 1);
}

/**
 * Lock mutex.
 */
void CONVERT_Lock(void) {
  if (pthread_mutex_lock(&Mutex_) != 0) {
    OS_printf("Can not lock \n");
    exit(1);
  }
}

/**
 * Unlock mutex.
 */
void CONVERT_Unlock(void) {
  if (pthread_mutex_unlock(&Mutex_) != 0) {
    OS_printf("Can not unlock \n");
    exit(1);
  }
}

/**
 * Get now time.
 */
RosTime CONVERT_RosTimeNow(void) {
  double Now = CONVERT_RosTimeNowToSec();
  RosTime NowRosTime;
  NowRosTime.sec = (uint)Now;
  NowRosTime.nsec = (uint)((Now - NowRosTime.sec) * 1e+9);
  return NowRosTime;
}

/**
 * Get application source directory path.
 * @param Application name.
 */
std::string CONVERT_RosPackageGetPath(const std::string &AppName) {
  char Path[FILE_PATH_MAX_LENGTH];
  getcwd(Path, FILE_PATH_MAX_LENGTH);
  std::string FilePath = std::string(Path);
  // Split file path with slash
  uint Offset = 0;
  std::vector<std::string> DirList;
  while (1) {
    uint Pos = FilePath.find(PATH_SEPARATOR, Offset);
    if (Pos == std::string::npos) {
      DirList.push_back(FilePath.substr(Offset));
      break;
    }
    DirList.push_back(FilePath.substr(Offset, Pos - Offset));
    Offset = Pos + 1;
  }
  // Rebuild file paths (Combine from the second and onwards since the beginning
  // is an empty character)
  // Delete /build/cpu1/exe and add the application name at the end
  std::string DstFilePath = PATH_SEPARATOR;
  for (size_t ii = 1; ii < (DirList.size() - 3); ii++) {
    DstFilePath += DirList[ii] + PATH_SEPARATOR;
  }
  DstFilePath +=
      std::string("apps") + PATH_SEPARATOR + AppName + PATH_SEPARATOR;
  return DstFilePath;
}

/************************/
/*  End of File Comment */
/************************/
