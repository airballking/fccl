#include <fccl/utils/Equalities.h>
#include <assert.h>

namespace fccl
{
  namespace utils
  {
    bool Equal(const std::vector<std::size_t>& v1, const std::vector<std::size_t>& v2)
    {
      if(v1.size() != v2.size())
        return false;
  
      for(std::size_t i = 0; i<v1.size(); i++)
        if(v1[i] != v2[i])
          return false;
  
      return true;
    }

    bool isJoint(const KDL::Segment& segment)
    {
      return (segment.getJoint().getType() != KDL::Joint::None);
    }
  } // namespace utils
} // namespace fccl
