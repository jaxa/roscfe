/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef MIN_MAX_H
#define MIN_MAX_H

#include "cfe.h"

template <class T> TFSIMD_FORCE_INLINE const T &tfMin(const T &a, const T &b) {
  return a < b ? a : b;
}

template <class T> TFSIMD_FORCE_INLINE const T &tfMax(const T &a, const T &b) {
  return a > b ? a : b;
}

template <class T> TFSIMD_FORCE_INLINE void tfSetMin(T &a, const T &b) {
  if (b < a) {
    a = b;
  }
}

template <class T> TFSIMD_FORCE_INLINE void tfSetMax(T &a, const T &b) {
  if (a < b) {
    a = b;
  }
}

#endif // MIN_MAX_H
