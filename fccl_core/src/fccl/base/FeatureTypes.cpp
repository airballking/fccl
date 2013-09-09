#include <fccl/base/FeatureTypes.h>

namespace fccl
{
  namespace base
  {
    bool featureTypeValid(int feature_type)
    {
      return (UNKNOWN_FEATURE < feature_type) && (feature_type < FEATURE_COUNT);
    }
  } // namespace base
} // namespace fccl
