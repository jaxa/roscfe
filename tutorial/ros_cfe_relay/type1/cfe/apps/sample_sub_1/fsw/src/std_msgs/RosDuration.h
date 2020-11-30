/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef ROS_DURATION_H
#define ROS_DURATION_H

#include "cfe.h"
#include <climits>
#include <math.h>
#include <stdexcept>
#include <stdint.h>

inline void normalizeSecNSecSigned(int64_t &sec, int64_t &nsec) {
  int64_t nsec_part = nsec;
  int64_t sec_part = sec;

  while (nsec_part > 1000000000L) {
    nsec_part -= 1000000000L;
    ++sec_part;
  }
  while (nsec_part < 0) {
    nsec_part += 1000000000L;
    --sec_part;
  }

  if (sec_part < INT_MIN || sec_part > INT_MAX)
    throw std::runtime_error("Duration is out of dual 32-bit range");

  sec = sec_part;
  nsec = nsec_part;
}

inline void normalizeSecNSecSigned(int32_t &sec, int32_t &nsec) {
  int64_t sec64 = sec;
  int64_t nsec64 = nsec;

  normalizeSecNSecSigned(sec64, nsec64);

  sec = (int32_t)sec64;
  nsec = (int32_t)nsec64;
}

template <class T> class RosDurationBase {
public:
  int32_t sec, nsec;
  RosDurationBase() : sec(0), nsec(0) {}
  RosDurationBase(int32_t _sec, int32_t _nsec);
  explicit RosDurationBase(double t) { fromSec(t); };
  ~RosDurationBase() {}
  T operator+(const T &rhs) const;
  T operator-(const T &rhs) const;
  T operator-() const;
  T operator*(double scale) const;
  T &operator+=(const T &rhs);
  T &operator-=(const T &rhs);
  T &operator*=(double scale);
  bool operator==(const T &rhs) const;
  inline bool operator!=(const T &rhs) const {
    return !(*static_cast<const T *>(this) == rhs);
  }
  bool operator>(const T &rhs) const;
  bool operator<(const T &rhs) const;
  bool operator>=(const T &rhs) const;
  bool operator<=(const T &rhs) const;
  double toSec() const { return (double)sec + 1e-9 * (double)nsec; };
  int64_t toNSec() const {
    return (int64_t)sec * 1000000000ll + (int64_t)nsec;
  };
  T &fromSec(double t);
  T &fromNSec(int64_t t);
  bool isZero();
};

class RosDuration : public RosDurationBase<RosDuration> {
public:
  RosDuration() : RosDurationBase<RosDuration>() {}

  RosDuration(int32_t _sec, int32_t _nsec)
      : RosDurationBase<RosDuration>(_sec, _nsec) {}

  explicit RosDuration(double t) { fromSec(t); }

  bool sleep() const;
};

extern const RosDuration DURATION_MAX;
extern const RosDuration DURATION_MIN;

class WallRosDuration : public RosDurationBase<WallRosDuration> {
public:
  WallRosDuration() : RosDurationBase<WallRosDuration>() {}

  WallRosDuration(int32_t _sec, int32_t _nsec)
      : RosDurationBase<WallRosDuration>(_sec, _nsec) {}

  explicit WallRosDuration(double t) { fromSec(t); }

  bool sleep() const;
};

// std::ostream &operator <<(std::ostream &os, const RosDuration &rhs);
// std::ostream &operator <<(std::ostream &os, const WallRosDuration &rhs);

//
// RosDurationBase template member function implementation
//
template <class T>
RosDurationBase<T>::RosDurationBase(int32_t _sec, int32_t _nsec)
    : sec(_sec), nsec(_nsec) {
  normalizeSecNSecSigned(sec, nsec);
}

template <class T> T &RosDurationBase<T>::fromSec(double d) {
#ifdef HAVE_TRUNC
  sec = (int32_t)trunc(d);
#else
  // (morgan: why doesn't win32 provide trunc? argh. hacked this together
  // without much thought. need to test this conversion.
  if (d >= 0.0)
    sec = (int32_t)floor(d);
  else
    sec = (int32_t)floor(d) + 1;
#endif
  nsec = (int32_t)((d - (double)sec) * 1000000000);
  return *static_cast<T *>(this);
}

template <class T> T &RosDurationBase<T>::fromNSec(int64_t t) {
  sec = (int32_t)(t / 1000000000);
  nsec = (int32_t)(t % 1000000000);

  normalizeSecNSecSigned(sec, nsec);

  return *static_cast<T *>(this);
}

template <class T> T RosDurationBase<T>::operator+(const T &rhs) const {
  return T(sec + rhs.sec, nsec + rhs.nsec);
}

template <class T> T RosDurationBase<T>::operator*(double scale) const {
  return T(toSec() * scale);
}

template <class T> T RosDurationBase<T>::operator-(const T &rhs) const {
  return T(sec - rhs.sec, nsec - rhs.nsec);
}

template <class T> T RosDurationBase<T>::operator-() const {
  return T(-sec, -nsec);
}

template <class T> T &RosDurationBase<T>::operator+=(const T &rhs) {
  *this = *this + rhs;
  return *static_cast<T *>(this);
}

template <class T> T &RosDurationBase<T>::operator-=(const T &rhs) {
  *this += (-rhs);
  return *static_cast<T *>(this);
}

template <class T> T &RosDurationBase<T>::operator*=(double scale) {
  fromSec(toSec() * scale);
  return *static_cast<T *>(this);
}

template <class T> bool RosDurationBase<T>::operator<(const T &rhs) const {
  if (sec < rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec < rhs.nsec)
    return true;
  return false;
}

template <class T> bool RosDurationBase<T>::operator>(const T &rhs) const {
  if (sec > rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec > rhs.nsec)
    return true;
  return false;
}

template <class T> bool RosDurationBase<T>::operator<=(const T &rhs) const {
  if (sec < rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec <= rhs.nsec)
    return true;
  return false;
}

template <class T> bool RosDurationBase<T>::operator>=(const T &rhs) const {
  if (sec > rhs.sec)
    return true;
  else if (sec == rhs.sec && nsec >= rhs.nsec)
    return true;
  return false;
}

template <class T> bool RosDurationBase<T>::operator==(const T &rhs) const {
  return sec == rhs.sec && nsec == rhs.nsec;
}

template <class T> bool RosDurationBase<T>::isZero() {
  return sec == 0 && nsec == 0;
}

#endif // ROS_DURATION_H
