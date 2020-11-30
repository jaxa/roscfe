/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef ROS_TIME_H
#define ROS_TIME_H

#include "RosDuration.h"
#include "cfe.h"

inline void normalizeSecNSec(uint64_t &sec, uint64_t &nsec) {
  uint64_t nsec_part = nsec % 1000000000UL;
  uint64_t sec_part = nsec / 1000000000UL;

  if (sec_part > UINT_MAX)
    throw std::runtime_error("RosTime is out of dual 32-bit range");

  sec += sec_part;
  nsec = nsec_part;
}

inline void normalizeSecNSec(uint32_t &sec, uint32_t &nsec) {
  uint64_t sec64 = sec;
  uint64_t nsec64 = nsec;

  normalizeSecNSec(sec64, nsec64);

  sec = (uint32_t)sec64;
  nsec = (uint32_t)nsec64;
}

inline void normalizeSecNSecUnsigned(int64_t &sec, int64_t &nsec) {
  int64_t nsec_part = nsec;
  int64_t sec_part = sec;

  while (nsec_part >= 1000000000L) {
    nsec_part -= 1000000000L;
    ++sec_part;
  }
  while (nsec_part < 0) {
    nsec_part += 1000000000L;
    --sec_part;
  }

  if (sec_part < 0 || sec_part > INT_MAX) {
    throw std::runtime_error("RosTime is out of dual 32-bit range");
  }

  sec = sec_part;
  nsec = nsec_part;
}

/*********************************************************************
** RosTime Classes
*********************************************************************/

template <class T, class D> class RosTimeBase {
public:
  uint8 TlmHeader[CFE_SB_TLM_HDR_SIZE];
  uint32_t sec, nsec;

  RosTimeBase() : sec(0), nsec(0) {}
  RosTimeBase(uint32_t _sec, uint32_t _nsec) : sec(_sec), nsec(_nsec) {
    normalizeSecNSec(sec, nsec);
  }
  explicit RosTimeBase(double t) { fromSec(t); }
  ~RosTimeBase() {}
  D operator-(const T &rhs) const;
  T operator+(const D &rhs) const;
  T operator-(const D &rhs) const;
  T &operator+=(const D &rhs);
  T &operator-=(const D &rhs);
  bool operator==(const T &rhs) const;
  inline bool operator!=(const T &rhs) const {
    return !(*static_cast<const T *>(this) == rhs);
  }
  bool operator>(const T &rhs) const;
  bool operator<(const T &rhs) const;
  bool operator>=(const T &rhs) const;
  bool operator<=(const T &rhs) const;

  double toSec() const { return (double)sec + 1e-9 * (double)nsec; };
  T &fromSec(double t) {
    sec = (uint32_t)floor(t);
    nsec = (uint32_t)round((t - sec) * 1e9);
    return *static_cast<T *>(this);
  }

  uint64_t toNSec() const {
    return (uint64_t)sec * 1000000000ull + (uint64_t)nsec;
  }
  T &fromNSec(uint64_t t);

  inline bool isZero() const { return sec == 0 && nsec == 0; }
  inline bool is_zero() const { return isZero(); }
};

class RosTime : public RosTimeBase<RosTime, RosDuration> {
public:
  RosTime() : RosTimeBase<RosTime, RosDuration>() {}

  RosTime(uint32_t _sec, uint32_t _nsec)
      : RosTimeBase<RosTime, RosDuration>(_sec, _nsec) {}

  explicit RosTime(double t) { fromSec(t); }

  static RosTime now() {
    CFE_TIME_SysTime_t currentTime = CFE_TIME_GetTime();
    RosTime ret = RosTime(currentTime.Seconds, currentTime.Subseconds);
    return ret;
  }
  static bool sleepUntil(const RosTime &end);

  static void init();
  static void shutdown();
  static void setNow(const RosTime &new_now);
  static bool useSystemRosTime();
  static bool isSimRosTime();
  static bool isSystemRosTime();

  static bool isValid();
  static bool waitForValid();
  static bool waitForValid(const WallRosDuration &timeout);

  void vector2pointer() {}

  void deleteData() {}
};

extern const RosTime TIME_MAX;
extern const RosTime TIME_MIN;

class WallRosTime : public RosTimeBase<WallRosTime, WallRosDuration> {
public:
  WallRosTime() : RosTimeBase<WallRosTime, WallRosDuration>() {}

  WallRosTime(uint32_t _sec, uint32_t _nsec)
      : RosTimeBase<WallRosTime, WallRosDuration>(_sec, _nsec) {}

  explicit WallRosTime(double t) { fromSec(t); }

  static WallRosTime now();

  static bool sleepUntil(const WallRosTime &end);

  static bool isSystemRosTime() { return true; }
};

// std::ostream &operator <<(std::ostream &os, const RosTime &rhs);
// std::ostream &operator <<(std::ostream &os, const WallRosTime &rhs);

template <class T, class D> T &RosTimeBase<T, D>::fromNSec(uint64_t t) {
  sec = (int32_t)(t / 1000000000);
  nsec = (int32_t)(t % 1000000000);

  normalizeSecNSec(sec, nsec);

  return *static_cast<T *>(this);
}

template <class T, class D> D RosTimeBase<T, D>::operator-(const T &rhs) const {
  return D((int32_t)sec - (int32_t)rhs.sec,
           (int32_t)nsec - (int32_t)rhs.nsec); // carry handled in ctor
}

template <class T, class D> T RosTimeBase<T, D>::operator-(const D &rhs) const {
  return *static_cast<const T *>(this) + (-rhs);
}

template <class T, class D> T RosTimeBase<T, D>::operator+(const D &rhs) const {
  int64_t sec_sum = (int64_t)sec + (int64_t)rhs.sec;
  int64_t nsec_sum = (int64_t)nsec + (int64_t)rhs.nsec;

  // Throws an exception if we go out of 32-bit range
  normalizeSecNSecUnsigned(sec_sum, nsec_sum);

  // now, it's safe to downcast back to uint32 bits
  return T((uint32_t)sec_sum, (uint32_t)nsec_sum);
}

template <class T, class D> T &RosTimeBase<T, D>::operator+=(const D &rhs) {
  *this = *this + rhs;
  return *static_cast<T *>(this);
}

template <class T, class D> T &RosTimeBase<T, D>::operator-=(const D &rhs) {
  *this += (-rhs);
  return *static_cast<T *>(this);
}

template <class T, class D>
bool RosTimeBase<T, D>::operator==(const T &rhs) const {
  return sec == rhs.sec && nsec == rhs.nsec;
}

template <class T, class D>
bool RosTimeBase<T, D>::operator<(const T &rhs) const {
  if (sec < rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec < rhs.nsec)
    return true;
  return false;
}

template <class T, class D>
bool RosTimeBase<T, D>::operator>(const T &rhs) const {
  if (sec > rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec > rhs.nsec)
    return true;
  return false;
}

template <class T, class D>
bool RosTimeBase<T, D>::operator<=(const T &rhs) const {
  if (sec < rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec <= rhs.nsec)
    return true;
  return false;
}

template <class T, class D>
bool RosTimeBase<T, D>::operator>=(const T &rhs) const {
  if (sec > rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec >= rhs.nsec)
    return true;
  return false;
}

#endif // ROS_TIME_H
