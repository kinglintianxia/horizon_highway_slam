#ifndef SOPHUS_SE3_H
#define SOPHUS_SE3_H

#include "so3.h"

namespace Sophus
{
using namespace Eigen;
using namespace std;

typedef Matrix< double, 6, 1 > Vector6d;
typedef Matrix< double, 6, 6 > Matrix6d;

class SE3
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SE3                        ();

  SE3                        (const SO3 & so3,
                              const Vector3d & translation);
  SE3                        (const Matrix3d & rotation_matrix,
                              const Vector3d & translation);
  SE3                        (const Quaterniond & unit_quaternion,
                              const Vector3d & translation_);
  SE3                        (const SE3 & other);

  SE3 &
  operator=                  (const SE3 & other);

  SE3
  operator*                  (const SE3& other) const;

  SE3&
  operator*=                 (const SE3& other);

  SE3
  inverse                    () const;

  Vector6d
  log                        () const;

  Vector3d
  operator*                  (const Vector3d & xyz) const;

  static SE3
  exp                        (const Vector6d & update);

  static Vector6d
  log                        (const SE3 & se3);

  Matrix<double,4,4>
  matrix                     () const;

  Matrix<double, 6, 6>
  Adj                        () const;

  static Matrix4d
  hat                        (const Vector6d & omega);

  static Vector6d
  vee                        (const Matrix4d & Omega);

  static Vector6d
  lieBracket                 (const Vector6d & v1,
                              const Vector6d & v2);

  static Matrix6d
  d_lieBracketab_by_d_a      (const Vector6d & b);

  void
  setQuaternion              (const Quaterniond& quaternion);

  static SE3 from_SE3(const SE3 & se3)
  {
    return se3;
  }

  static SE3 to_SE3(const SE3 & se3)
  {
    return se3.to_SE3();
  }

  SE3 to_SE3() const
  {
    return *this;
  }

  const Vector3d & translation() const
  {
    return translation_;
  }

  Vector3d& translation()
  {
    return translation_;
  }

  const Quaterniond & unit_quaternion() const
  {
    return so3_.unit_quaternion();
  }

  Matrix3d rotation_matrix() const
  {
    return so3_.matrix();
  }

  void setRotationMatrix(const Matrix3d & rotation_matrix)
  {
    so3_.setQuaternion(Quaterniond(rotation_matrix));
  }

  const SO3& so3() const
  {
    return so3_;
  }

  SO3& so3()
  {
    return so3_;
  }

  static const int DoF = 6;

private:
  SO3 so3_;
  Vector3d translation_;

};

inline std::ostream& operator <<(std::ostream & out_str,
                                 const SE3 &  se3)
{
  out_str << se3.so3() << se3.translation().transpose() << std::endl;
  return out_str;
}

}


#endif
