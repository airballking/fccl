#include <fccl/utils/Equalities.h>
#include <assert.h>
#include <limits>
#include <math.h>

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

    bool Equal(const std::vector<std::string>& v1, const std::vector<std::string>& v2)
    {
      if(v1.size() != v2.size())
        return false;

      for(std::size_t i=0; i<v1.size(); i++)
         if(v1[i] != v2[i])
           return false;

      return true;
    }

    bool areEqual(double a, double b)
    {
      return fabs(a-b) < std::numeric_limits<double>::epsilon();
    }
 
    bool isJoint(const KDL::Segment& segment)
    {
      return (segment.getJoint().getType() != KDL::Joint::None);
    }
  } // namespace utils
} // namespace fccl
