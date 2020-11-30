/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef QUAD_WORD_H
#define QUAD_WORD_H

#include "cfe.h"

#include "MinMax.h"
#include "Scalar.h"

#if defined(__CELLOS_LV2) && defined(__SPU__)
#include <altivec.h>
#endif

namespace tf {
#ifndef USE_LIBSPE2
ATTRIBUTE_ALIGNED16(class)
QuadWord
#else
class QuadWord
#endif
{
protected:
#if defined(__SPU__) && defined(__CELLOS_LV2__)
  union {
    vec_float4 mVec128;
    tfScalar m_floats[4];
  };

public:
  vec_float4 get128() const { return mVec128; }

protected:
#else  //__CELLOS_LV2__ __SPU__
  tfScalar m_floats[4];
#endif //__CELLOS_LV2__ __SPU__

public:
  TFSIMD_FORCE_INLINE const tfScalar &getX() const { return m_floats[0]; }
  TFSIMD_FORCE_INLINE const tfScalar &getY() const { return m_floats[1]; }
  TFSIMD_FORCE_INLINE const tfScalar &getZ() const { return m_floats[2]; }
  TFSIMD_FORCE_INLINE void setX(tfScalar x) { m_floats[0] = x; };
  TFSIMD_FORCE_INLINE void setY(tfScalar y) { m_floats[1] = y; };
  TFSIMD_FORCE_INLINE void setZ(tfScalar z) { m_floats[2] = z; };
  TFSIMD_FORCE_INLINE void setW(tfScalar w) { m_floats[3] = w; };
  TFSIMD_FORCE_INLINE const tfScalar &x() const { return m_floats[0]; }
  TFSIMD_FORCE_INLINE const tfScalar &y() const { return m_floats[1]; }
  TFSIMD_FORCE_INLINE const tfScalar &z() const { return m_floats[2]; }
  TFSIMD_FORCE_INLINE const tfScalar &w() const { return m_floats[3]; }

  // TFSIMD_FORCE_INLINE tfScalar&       operator[](int i)       { return
  // (&m_floats[0])[i];       }
  // TFSIMD_FORCE_INLINE const tfScalar& operator[](int i) const { return
  // (&m_floats[0])[i]; }
  TFSIMD_FORCE_INLINE operator tfScalar *() { return &m_floats[0]; }
  TFSIMD_FORCE_INLINE operator const tfScalar *() const { return &m_floats[0]; }

  TFSIMD_FORCE_INLINE bool operator==(const QuadWord &other) const {
    return ((m_floats[3] == other.m_floats[3]) &&
            (m_floats[2] == other.m_floats[2]) &&
            (m_floats[1] == other.m_floats[1]) &&
            (m_floats[0] == other.m_floats[0]));
  }

  TFSIMD_FORCE_INLINE bool operator!=(const QuadWord &other) const {
    return !(*this == other);
  }

  TFSIMD_FORCE_INLINE void setValue(const tfScalar &x, const tfScalar &y,
                                    const tfScalar &z) {
    m_floats[0] = x;
    m_floats[1] = y;
    m_floats[2] = z;
    m_floats[3] = 0.f;
  }

  /*              void getValue(tfScalar *m) const
          {
                  m[0] = m_floats[0];
                  m[1] = m_floats[1];
                  m[2] = m_floats[2];
          }
  */
  TFSIMD_FORCE_INLINE void setValue(const tfScalar &x, const tfScalar &y,
                                    const tfScalar &z, const tfScalar &w) {
    m_floats[0] = x;
    m_floats[1] = y;
    m_floats[2] = z;
    m_floats[3] = w;
  }
  TFSIMD_FORCE_INLINE QuadWord()
  //      :m_floats[0](tfScalar(0.)),m_floats[1](tfScalar(0.)),m_floats[2](tfScalar(0.)),m_floats[3](tfScalar(0.))
  {}

  TFSIMD_FORCE_INLINE QuadWord(const tfScalar &x, const tfScalar &y,
                               const tfScalar &z) {
    m_floats[0] = x, m_floats[1] = y, m_floats[2] = z, m_floats[3] = 0.0f;
  }

  TFSIMD_FORCE_INLINE QuadWord(const tfScalar &x, const tfScalar &y,
                               const tfScalar &z, const tfScalar &w) {
    m_floats[0] = x, m_floats[1] = y, m_floats[2] = z, m_floats[3] = w;
  }

  TFSIMD_FORCE_INLINE void setMax(const QuadWord &other) {
    tfSetMax(m_floats[0], other.m_floats[0]);
    tfSetMax(m_floats[1], other.m_floats[1]);
    tfSetMax(m_floats[2], other.m_floats[2]);
    tfSetMax(m_floats[3], other.m_floats[3]);
  }
  TFSIMD_FORCE_INLINE void setMin(const QuadWord &other) {
    tfSetMin(m_floats[0], other.m_floats[0]);
    tfSetMin(m_floats[1], other.m_floats[1]);
    tfSetMin(m_floats[2], other.m_floats[2]);
    tfSetMin(m_floats[3], other.m_floats[3]);
  }
};
}

#endif // QUAD_WORD_H
