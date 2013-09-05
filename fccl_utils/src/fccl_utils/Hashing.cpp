#include <fccl_utils/Hashing.h>

namespace fccl
{
  std::size_t hash(const std::string& in)
  {
    std::size_t result = 0;

    for(unsigned int i=0; i<in.size(); i++)
      result += 101*in[i];

    return result;
  }
} // namespace fccl
