#include <fccl_base/FeatureTypes.h>

namespace fccl
{

  bool featureTypeValid(int feature_type)
  {
    return (UNKNOWN_FEATURE < feature_type) && (feature_type < FEATURE_COUNT);
  }

} // namespace fccl
