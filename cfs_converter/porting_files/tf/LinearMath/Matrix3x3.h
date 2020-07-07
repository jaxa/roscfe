/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#ifndef MATRIX_3X3_H
#define MATRIX_3X3_H

#include "cfe.h"

#include "Quaternion.h"
#include "Vector3.h"

namespace tf {
#define Matrix3x3Data Matrix3x3DoubleData

class Matrix3x3 {

  Vector3 m_el[3];

public:
  Matrix3x3() {}

  //              explicit Matrix3x3(const tfScalar *m) {
  //              setFromOpenGLSubMatrix(m); }

  explicit Matrix3x3(const Quaternion &q) { setRotation(q); }
  /*
  template <typename tfScalar>
  Matrix3x3(const tfScalar& yaw, const tfScalar& pitch, const tfScalar& roll)
  {
  setEulerYPR(yaw, pitch, roll);
  }
  */
  Matrix3x3(const tfScalar &xx, const tfScalar &xy, const tfScalar &xz,
            const tfScalar &yx, const tfScalar &yy, const tfScalar &yz,
            const tfScalar &zx, const tfScalar &zy, const tfScalar &zz) {
    setValue(xx, xy, xz, yx, yy, yz, zx, zy, zz);
  }
  TFSIMD_FORCE_INLINE Matrix3x3(const Matrix3x3 &other) {
    m_el[0] = other.m_el[0];
    m_el[1] = other.m_el[1];
    m_el[2] = other.m_el[2];
  }

  //             TFSIMD_FORCE_INLINE Matrix3x3& operator=(const Matrix3x3&
  //             other)
  //             {
  //                     m_el[0] = other.m_el[0];
  //                     m_el[1] = other.m_el[1];
  //                     m_el[2] = other.m_el[2];
  //                     return *this;
  //             }

  //             TFSIMD_FORCE_INLINE Vector3 getColumn(int i) const
  //             {
  //                     return Vector3(m_el[0][i],m_el[1][i],m_el[2][i]);
  //             }

  //             TFSIMD_FORCE_INLINE const Vector3& getRow(int i) const
  //             {
  //                     tfFullAssert(0 <= i && i < 3);
  //                     return m_el[i];
  //             }

  TFSIMD_FORCE_INLINE Vector3 &operator[](int i) {
    tfFullAssert(0 <= i && i < 3);
    return m_el[i];
  }

  TFSIMD_FORCE_INLINE const Vector3 &operator[](int i) const {
    tfFullAssert(0 <= i && i < 3);
    return m_el[i];
  }

  Matrix3x3 &operator*=(const Matrix3x3 &m);

  //             void setFromOpenGLSubMatrix(const tfScalar *m)
  //             {
  //                     m_el[0].setValue(m[0],m[4],m[8]);
  //                     m_el[1].setValue(m[1],m[5],m[9]);
  //                     m_el[2].setValue(m[2],m[6],m[10]);

  //             }
  void setValue(const tfScalar &xx, const tfScalar &xy, const tfScalar &xz,
                const tfScalar &yx, const tfScalar &yy, const tfScalar &yz,
                const tfScalar &zx, const tfScalar &zy, const tfScalar &zz) {
    m_el[0].setValue(xx, xy, xz);
    m_el[1].setValue(yx, yy, yz);
    m_el[2].setValue(zx, zy, zz);
  }

  void setRotation(const Quaternion &q) {
    tfScalar d = q.length2();
    tfFullAssert(d != tfScalar(0.0));
    tfScalar s = tfScalar(2.0) / d;
    tfScalar xs = q.x() * s;
    tfScalar ys = q.y() * s;
    tfScalar zs = q.z() * s;
    tfScalar wx = q.w() * xs;
    tfScalar wy = q.w() * ys;
    tfScalar wz = q.w() * zs;
    tfScalar xx = q.x() * xs;
    tfScalar xy = q.x() * ys;
    tfScalar xz = q.x() * zs;
    tfScalar yy = q.y() * ys;
    tfScalar yz = q.y() * zs;
    tfScalar zz = q.z() * zs;
    setValue(tfScalar(1.0) - (yy + zz), xy - wz, xz + wy, xy + wz,
             tfScalar(1.0) - (xx + zz), yz - wx, xz - wy, yz + wx,
             tfScalar(1.0) - (xx + yy));
  }

  //             void setEulerZYX(const tfScalar& yaw, const tfScalar& pitch,
  //             const tfScalar& roll) __attribute__((deprecated))
  //             {
  //                     setEulerYPR(yaw, pitch, roll);
  //             }

  //             void setEulerYPR(tfScalar eulerZ, tfScalar eulerY,tfScalar
  //             eulerX)  {
  //                     tfScalar ci ( tfCos(eulerX));
  //                     tfScalar cj ( tfCos(eulerY));
  //                     tfScalar ch ( tfCos(eulerZ));
  //                     tfScalar si ( tfSin(eulerX));
  //                     tfScalar sj ( tfSin(eulerY));
  //                     tfScalar sh ( tfSin(eulerZ));
  //                     tfScalar cc = ci * ch;
  //                     tfScalar cs = ci * sh;
  //                     tfScalar sc = si * ch;
  //                     tfScalar ss = si * sh;

  //                     setValue(cj * ch, sj * sc - cs, sj * cc + ss,
  //                             cj * sh, sj * ss + cc, sj * cs - sc,
  //                             -sj,      cj * si,      cj * ci);
  //             }

  //             void setRPY(tfScalar roll, tfScalar pitch,tfScalar yaw) {
  //                     setEulerYPR(yaw, pitch, roll);
  //             }

  //             void setIdentity()
  //             {
  //                     setValue(tfScalar(1.0), tfScalar(0.0), tfScalar(0.0),
  //                             tfScalar(0.0), tfScalar(1.0), tfScalar(0.0),
  //                             tfScalar(0.0), tfScalar(0.0), tfScalar(1.0));
  //             }

  //             static const Matrix3x3& getIdentity()
  //             {
  //                     static const Matrix3x3 identityMatrix(tfScalar(1.0),
  //                     tfScalar(0.0), tfScalar(0.0),
  //                             tfScalar(0.0), tfScalar(1.0), tfScalar(0.0),
  //                             tfScalar(0.0), tfScalar(0.0), tfScalar(1.0));
  //                     return identityMatrix;
  //             }

  //             void getOpenGLSubMatrix(tfScalar *m) const
  //             {
  //                     m[0]  = tfScalar(m_el[0].x());
  //                     m[1]  = tfScalar(m_el[1].x());
  //                     m[2]  = tfScalar(m_el[2].x());
  //                     m[3]  = tfScalar(0.0);
  //                     m[4]  = tfScalar(m_el[0].y());
  //                     m[5]  = tfScalar(m_el[1].y());
  //                     m[6]  = tfScalar(m_el[2].y());
  //                     m[7]  = tfScalar(0.0);
  //                     m[8]  = tfScalar(m_el[0].z());
  //                     m[9]  = tfScalar(m_el[1].z());
  //                     m[10] = tfScalar(m_el[2].z());
  //                     m[11] = tfScalar(0.0);
  //             }

  void getRotation(Quaternion &q) const {
    tfScalar trace = m_el[0].x() + m_el[1].y() + m_el[2].z();
    tfScalar temp[4];

    if (trace > tfScalar(0.0)) {
      tfScalar s = tfSqrt(trace + tfScalar(1.0));
      temp[3] = (s * tfScalar(0.5));
      s = tfScalar(0.5) / s;

      temp[0] = ((m_el[2].y() - m_el[1].z()) * s);
      temp[1] = ((m_el[0].z() - m_el[2].x()) * s);
      temp[2] = ((m_el[1].x() - m_el[0].y()) * s);
    } else {
      int i = m_el[0].x() < m_el[1].y() ? (m_el[1].y() < m_el[2].z() ? 2 : 1)
                                        : (m_el[0].x() < m_el[2].z() ? 2 : 0);
      int j = (i + 1) % 3;
      int k = (i + 2) % 3;

      tfScalar s = tfSqrt(m_el[i][i] - m_el[j][j] - m_el[k][k] + tfScalar(1.0));
      temp[i] = s * tfScalar(0.5);
      s = tfScalar(0.5) / s;

      temp[3] = (m_el[k][j] - m_el[j][k]) * s;
      temp[j] = (m_el[j][i] + m_el[i][j]) * s;
      temp[k] = (m_el[k][i] + m_el[i][k]) * s;
    }
    q.setValue(temp[0], temp[1], temp[2], temp[3]);
  }

  //             __attribute__((deprecated)) void getEulerZYX(tfScalar& yaw,
  //             tfScalar& pitch, tfScalar& roll, unsigned int solution_number =
  //             1) const
  //             {
  //                     getEulerYPR(yaw, pitch, roll, solution_number);
  //             };

  void getEulerYPR(tfScalar &yaw, tfScalar &pitch, tfScalar &roll,
                   unsigned int solution_number = 1) const {
    struct Euler {
      tfScalar yaw;
      tfScalar pitch;
      tfScalar roll;
    };

    Euler euler_out;
    Euler euler_out2; // second solution
    // get the pointer to the raw data

    // Check that pitch is not at a singularity
    // Check that pitch is not at a singularity
    if (tfFabs(m_el[2].x()) >= 1) {
      euler_out.yaw = 0;
      euler_out2.yaw = 0;

      // From difference of angles formula
      if (m_el[2].x() < 0) // gimbal locked down
      {
        tfScalar delta = tfAtan2(m_el[0].y(), m_el[0].z());
        euler_out.pitch = TFSIMD_PI / tfScalar(2.0);
        euler_out2.pitch = TFSIMD_PI / tfScalar(2.0);
        euler_out.roll = delta;
        euler_out2.roll = delta;
      } else // gimbal locked up
      {
        tfScalar delta = tfAtan2(-m_el[0].y(), -m_el[0].z());
        euler_out.pitch = -TFSIMD_PI / tfScalar(2.0);
        euler_out2.pitch = -TFSIMD_PI / tfScalar(2.0);
        euler_out.roll = delta;
        euler_out2.roll = delta;
      }
    } else {
      euler_out.pitch = -tfAsin(m_el[2].x());
      euler_out2.pitch = TFSIMD_PI - euler_out.pitch;

      euler_out.roll = tfAtan2(m_el[2].y() / tfCos(euler_out.pitch),
                               m_el[2].z() / tfCos(euler_out.pitch));
      euler_out2.roll = tfAtan2(m_el[2].y() / tfCos(euler_out2.pitch),
                                m_el[2].z() / tfCos(euler_out2.pitch));

      euler_out.yaw = tfAtan2(m_el[1].x() / tfCos(euler_out.pitch),
                              m_el[0].x() / tfCos(euler_out.pitch));
      euler_out2.yaw = tfAtan2(m_el[1].x() / tfCos(euler_out2.pitch),
                               m_el[0].x() / tfCos(euler_out2.pitch));
    }

    if (solution_number == 1) {
      yaw = euler_out.yaw;
      pitch = euler_out.pitch;
      roll = euler_out.roll;
    } else {
      yaw = euler_out2.yaw;
      pitch = euler_out2.pitch;
      roll = euler_out2.roll;
    }
  }

  void getRPY(tfScalar &roll, tfScalar &pitch, tfScalar &yaw,
              unsigned int solution_number = 1) const {
    getEulerYPR(yaw, pitch, roll, solution_number);
  }

  //             Matrix3x3 scaled(const Vector3& s) const
  //             {
  //                     return Matrix3x3(m_el[0].x() * s.x(), m_el[0].y() *
  //                     s.y(), m_el[0].z() * s.z(),
  //                             m_el[1].x() * s.x(), m_el[1].y() * s.y(),
  //                             m_el[1].z() * s.z(),
  //                             m_el[2].x() * s.x(), m_el[2].y() * s.y(),
  //                             m_el[2].z() * s.z());
  //             }

  //             tfScalar            determinant() const;
  //             Matrix3x3 adjoint() const;
  //             Matrix3x3 absolute() const;
  Matrix3x3 transpose() const;
  //             Matrix3x3 inverse() const;

  //             Matrix3x3 transposeTimes(const Matrix3x3& m) const;
  //             Matrix3x3 timesTranspose(const Matrix3x3& m) const;

  TFSIMD_FORCE_INLINE tfScalar tdotx(const Vector3 &v) const {
    return m_el[0].x() * v.x() + m_el[1].x() * v.y() + m_el[2].x() * v.z();
  }
  TFSIMD_FORCE_INLINE tfScalar tdoty(const Vector3 &v) const {
    return m_el[0].y() * v.x() + m_el[1].y() * v.y() + m_el[2].y() * v.z();
  }
  TFSIMD_FORCE_INLINE tfScalar tdotz(const Vector3 &v) const {
    return m_el[0].z() * v.x() + m_el[1].z() * v.y() + m_el[2].z() * v.z();
  }

  //             void diagonalize(Matrix3x3& rot, tfScalar threshold, int
  //             maxSteps)
  //             {
  //                     rot.setIdentity();
  //                     for (int step = maxSteps; step > 0; step--)
  //                     {
  //                             // find off-diagonal element [p][q] with
  //                             largest magnitude
  //                             int p = 0;
  //                             int q = 1;
  //                             int r = 2;
  //                             tfScalar max = tfFabs(m_el[0][1]);
  //                             tfScalar v = tfFabs(m_el[0][2]);
  //                             if (v > max)
  //                             {
  //                                     q = 2;
  //                                     r = 1;
  //                                     max = v;
  //                             }
  //                             v = tfFabs(m_el[1][2]);
  //                             if (v > max)
  //                             {
  //                                     p = 1;
  //                                     q = 2;
  //                                     r = 0;
  //                                     max = v;
  //                             }

  //                             tfScalar t = threshold * (tfFabs(m_el[0][0]) +
  //                             tfFabs(m_el[1][1]) + tfFabs(m_el[2][2]));
  //                             if (max <= t)
  //                             {
  //                                     if (max <= TFSIMD_EPSILON * t)
  //                                     {
  //                                             return;
  //                                     }
  //                                     step = 1;
  //                             }

  //                             // compute Jacobi rotation J which leads to a
  //                             zero for element [p][q]
  //                             tfScalar mpq = m_el[p][q];
  //                             tfScalar theta = (m_el[q][q] - m_el[p][p]) / (2
  //                             * mpq);
  //                             tfScalar theta2 = theta * theta;
  //                             tfScalar cos;
  //                             tfScalar sin;
  //                             if (theta2 * theta2 < tfScalar(/
  //                             TFSIMD_EPSILON))
  //                             {
  //                                     t = (theta >= 0) ? 1 / (theta +
  //                                     tfSqrt(1 + theta2))
  //                                             : 1 / (theta - tfSqrt(1 +
  //                                             theta2));
  //                                     cos = 1 / tfSqrt(1 + t * t);
  //                                     sin = cos * t;
  //                             }
  //                             else
  //                             {
  //                                     // approximation for large theta-value,
  //                                     i.e., a nearly diagonal matrix
  //                                     t = 1 / (theta * (2 + tfScalar(0.5) /
  //                                     theta2));
  //                                     cos = 1 - tfScalar(0.5) * t * t;
  //                                     sin = cos * t;
  //                             }

  //                             // apply rotation to matrix (this = J^T * this
  //                             * J)
  //                             m_el[p][q] = m_el[q][p] = 0;
  //                             m_el[p][p] -= t * mpq;
  //                             m_el[q][q] += t * mpq;
  //                             tfScalar mrp = m_el[r][p];
  //                             tfScalar mrq = m_el[r][q];
  //                             m_el[r][p] = m_el[p][r] = cos * mrp - sin *
  //                             mrq;
  //                             m_el[r][q] = m_el[q][r] = cos * mrq + sin *
  //                             mrp;

  //                             // apply rotation to rot (rot = rot * J)
  //                             for (int i = 0; i < 3; i++)
  //                             {
  //                                     Vector3& row = rot[i];
  //                                     mrp = row[p];
  //                                     mrq = row[q];
  //                                     row[p] = cos * mrp - sin * mrq;
  //                                     row[q] = cos * mrq + sin * mrp;
  //                             }
  //                     }
  //             }

  //             tfScalar cofac(int r1, int c1, int r2, int c2) const
  //             {
  //                     return m_el[r1][c1] * m_el[r2][c2] - m_el[r1][c2] *
  //                     m_el[r2][c1];
  //             }

  //             void    serialize(struct        Matrix3x3Data& dataOut) const;

  //             void    serializeFloat(struct   Matrix3x3FloatData& dataOut)
  //             const;

  //             void    deSerialize(const struct        Matrix3x3Data& dataIn);

  //             void    deSerializeFloat(const struct   Matrix3x3FloatData&
  //             dataIn);

  //             void    deSerializeDouble(const struct  Matrix3x3DoubleData&
  //             dataIn);
};

TFSIMD_FORCE_INLINE Matrix3x3 &Matrix3x3::operator*=(const Matrix3x3 &m) {
  setValue(m.tdotx(m_el[0]), m.tdoty(m_el[0]), m.tdotz(m_el[0]),
           m.tdotx(m_el[1]), m.tdoty(m_el[1]), m.tdotz(m_el[1]),
           m.tdotx(m_el[2]), m.tdoty(m_el[2]), m.tdotz(m_el[2]));
  return *this;
}

//     TFSIMD_FORCE_INLINE tfScalar
//     Matrix3x3::determinant() const
//     {
//             return tfTriple((*this)[0], (*this)[1], (*this)[2]);
//     }

//     TFSIMD_FORCE_INLINE Matrix3x3
//     Matrix3x3::absolute() const
//     {
//             return Matrix3x3(
//                     tfFabs(m_el[0].x()), tfFabs(m_el[0].y()),
//                     tfFabs(m_el[0].z()),
//                     tfFabs(m_el[1].x()), tfFabs(m_el[1].y()),
//                     tfFabs(m_el[1].z()),
//                     tfFabs(m_el[2].x()), tfFabs(m_el[2].y()),
//                     tfFabs(m_el[2].z()));
//     }

TFSIMD_FORCE_INLINE Matrix3x3 Matrix3x3::transpose() const {
  return Matrix3x3(m_el[0].x(), m_el[1].x(), m_el[2].x(), m_el[0].y(),
                   m_el[1].y(), m_el[2].y(), m_el[0].z(), m_el[1].z(),
                   m_el[2].z());
}

//     TFSIMD_FORCE_INLINE Matrix3x3
//     Matrix3x3::adjoint() const
//     {
//             return Matrix3x3(cofac(1, 1, 2, 2), cofac(0, 2, 2, 1), cofac(0,
//             1, 1, 2),
//                     cofac(1, 2, 2, 0), cofac(0, 0, 2, 2), cofac(0, 2, 1, 0),
//                     cofac(1, 0, 2, 1), cofac(0, 1, 2, 0), cofac(0, 0, 1, 1));
//     }

//     TFSIMD_FORCE_INLINE Matrix3x3
//     Matrix3x3::inverse() const
//     {
//             Vector3 co(cofac(1, 1, 2, 2), cofac(1, 2, 2, 0), cofac(1, 0, 2,
//             1));
//             tfScalar det = (*this)[0].dot(co);
//             tfFullAssert(det != tfScalar(0.0));
//             tfScalar s = tfScalar(1.0) / det;
//             return Matrix3x3(co.x() * s, cofac(0, 2, 2, 1) * s, cofac(0, 1,
//             1, 2) * s,
//                     co.y() * s, cofac(0, 0, 2, 2) * s, cofac(0, 2, 1, 0) * s,
//                     co.z() * s, cofac(0, 1, 2, 0) * s, cofac(0, 0, 1, 1) *
//                     s);
//     }

//     TFSIMD_FORCE_INLINE Matrix3x3
//     Matrix3x3::transposeTimes(const Matrix3x3& m) const
//     {
//             return Matrix3x3(
//                     m_el[0].x() * m[0].x() + m_el[1].x() * m[1].x() +
//                     m_el[2].x() * m[2].x(),
//                     m_el[0].x() * m[0].y() + m_el[1].x() * m[1].y() +
//                     m_el[2].x() * m[2].y(),
//                     m_el[0].x() * m[0].z() + m_el[1].x() * m[1].z() +
//                     m_el[2].x() * m[2].z(),
//                     m_el[0].y() * m[0].x() + m_el[1].y() * m[1].x() +
//                     m_el[2].y() * m[2].x(),
//                     m_el[0].y() * m[0].y() + m_el[1].y() * m[1].y() +
//                     m_el[2].y() * m[2].y(),
//                     m_el[0].y() * m[0].z() + m_el[1].y() * m[1].z() +
//                     m_el[2].y() * m[2].z(),
//                     m_el[0].z() * m[0].x() + m_el[1].z() * m[1].x() +
//                     m_el[2].z() * m[2].x(),
//                     m_el[0].z() * m[0].y() + m_el[1].z() * m[1].y() +
//                     m_el[2].z() * m[2].y(),
//                     m_el[0].z() * m[0].z() + m_el[1].z() * m[1].z() +
//                     m_el[2].z() * m[2].z());
//     }

//     TFSIMD_FORCE_INLINE Matrix3x3
//     Matrix3x3::timesTranspose(const Matrix3x3& m) const
//     {
//             return Matrix3x3(
//                     m_el[0].dot(m[0]), m_el[0].dot(m[1]), m_el[0].dot(m[2]),
//                     m_el[1].dot(m[0]), m_el[1].dot(m[1]), m_el[1].dot(m[2]),
//                     m_el[2].dot(m[0]), m_el[2].dot(m[1]), m_el[2].dot(m[2]));

//     }

TFSIMD_FORCE_INLINE Vector3 operator*(const Matrix3x3 &m, const Vector3 &v) {
  return Vector3(m[0].dot(v), m[1].dot(v), m[2].dot(v));
}

//     TFSIMD_FORCE_INLINE Vector3
//     operator*(const Vector3& v, const Matrix3x3& m)
//     {
//             return Vector3(m.tdotx(v), m.tdoty(v), m.tdotz(v));
//     }

TFSIMD_FORCE_INLINE Matrix3x3 operator*(const Matrix3x3 &m1,
                                        const Matrix3x3 &m2) {
  return Matrix3x3(m2.tdotx(m1[0]), m2.tdoty(m1[0]), m2.tdotz(m1[0]),
                   m2.tdotx(m1[1]), m2.tdoty(m1[1]), m2.tdotz(m1[1]),
                   m2.tdotx(m1[2]), m2.tdoty(m1[2]), m2.tdotz(m1[2]));
}

//     TFSIMD_FORCE_INLINE bool operator==(const Matrix3x3& m1, const Matrix3x3&
//     m2)
//     {
//             return ( m1[0][0] == m2[0][0] && m1[1][0] == m2[1][0] && m1[2][0]
//             == m2[2][0] &&
//                     m1[0][1] == m2[0][1] && m1[1][1] == m2[1][1] && m1[2][1]
//                     == m2[2][1] &&
//                     m1[0][2] == m2[0][2] && m1[1][2] == m2[1][2] && m1[2][2]
//                     == m2[2][2] );
//     }

//     struct  Matrix3x3FloatData
//     {
//             Vector3FloatData m_el[3];
//     };

//     struct  Matrix3x3DoubleData
//     {
//             Vector3DoubleData m_el[3];
//     };

//     TFSIMD_FORCE_INLINE     void    Matrix3x3::serialize(struct
//     Matrix3x3Data& dataOut) const
//     {
//             for (int i=0;i<3;i++)
//                     m_el[i].serialize(dataOut.m_el[i]);
//     }

//     TFSIMD_FORCE_INLINE     void    Matrix3x3::serializeFloat(struct
//     Matrix3x3FloatData& dataOut) const
//     {
//             for (int i=0;i<3;i++)
//                     m_el[i].serializeFloat(dataOut.m_el[i]);
//     }

//     TFSIMD_FORCE_INLINE     void    Matrix3x3::deSerialize(const struct
//     Matrix3x3Data& dataIn)
//     {
//             for (int i=0;i<3;i++)
//                     m_el[i].deSerialize(dataIn.m_el[i]);
//     }

//     TFSIMD_FORCE_INLINE     void    Matrix3x3::deSerializeFloat(const struct
//     Matrix3x3FloatData& dataIn)
//     {
//             for (int i=0;i<3;i++)
//                     m_el[i].deSerializeFloat(dataIn.m_el[i]);
//     }

//     TFSIMD_FORCE_INLINE     void    Matrix3x3::deSerializeDouble(const struct
//     Matrix3x3DoubleData& dataIn)
//     {
//             for (int i=0;i<3;i++)
//                     m_el[i].deSerializeDouble(dataIn.m_el[i]);
//     }
}

#endif // MATRIX_3X3_H
