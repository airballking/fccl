#ifndef FCCL_SEMANTICS_EIGEN_EXTENSION_H
#define FCCL_SEMANTICS_EIGEN_EXTENSION_H

#include <Eigen/Core>
#include <fccl/semantics/SemanticsBase.h>

namespace Eigen
{
  template<> struct NumTraits<fccl::semantics::SemanticsBase> : NumTraits<unsigned int>
  {
    typedef std::size_t Nested;

    enum 
    {
      IsComplex = 0,
      IsInteger = 1,
      IsSigned = 0,
      RequireInitialization = 0,
      ReadCost = 1,
      AddCost = 3,
      MulCost = 3
    };
  };
} // namespace Eigen

#endif // FCCL_SEMANTICS_EIGEN_EXTENSION_H
