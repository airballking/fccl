#ifndef FCCL_BASE_FEATURE_TYPES_H
#define FCCL_BASE_FEATURE_TYPES_H

namespace fccl
{

  enum FeatureTypes
  {
    UNKNOWN_FEATURE = 0,
    
    POINT_FEATURE = 1,

    LINE_FEATURE = 2,

    PLANE_FEATURE = 3,

    FEATURE_COUNT
  };

  bool featureTypeValid(int feature_type);

} // namespace fccl
#endif // FCCL_BASE_FEATURE_TYPES_H
