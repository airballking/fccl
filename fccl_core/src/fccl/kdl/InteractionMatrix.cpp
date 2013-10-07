#include <fccl/kdl/InteractionMatrix.h>

namespace fccl
{
  namespace kdl
  {
      Eigen::Matrix<double, 6, 6> InteractionMatrix::calcTwistProjectorTranspose(const KDL::Frame& transform) const
      {
      // copy frame because some of Eigens functions do not guarantee constness
      KDL::Frame f = transform;
  
      // (transposed) Rotation matrix of f
      Eigen::Matrix3d Rt = Eigen::Map<Eigen::Matrix3d>(f.M.data);
    
      double x = f.p.x(), y = f.p.y(), z = f.p.z();
    
      // Skew symmetric matrix of p, [p]_x for expressing a cross product
      Eigen::Matrix3d px;
      px << 0, -z,  y,
            z,  0, -x,
           -y,  x,  0;
    
      // the inverse twist projection matrix
      Eigen::Matrix<double, 6, 6> Mi;
      Mi.block<3,3>(0,0) = Rt;
      Mi.block<3,3>(3,3) = Rt;
      Mi.block<3,3>(3,0) = -Rt*px;
      Mi.block<3,3>(0,3) = Eigen::Matrix3d::Zero();
  
      // return result
      return Mi;
    }
  } // namespace kdl
} // namespace fccl
