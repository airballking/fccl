#include <fccl_utils/Equalities.h>
#include <assert.h>

namespace fccl
{

  bool Equal(const std::vector<std::size_t>& v1, const std::vector<std::size_t>& v2)
  {
    assert(v1.size() == v2.size());

    for(std::size_t i = 0; i<v1.size(); i++)
      if(v1[i] != v2[i])
        return false;

    return true;
  }

}
