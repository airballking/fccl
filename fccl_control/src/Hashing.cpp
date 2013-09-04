#include <fccl_control/Hashing.h>

namespace fccl
{
  unsigned long hash(const std::string& in)
  {
    unsigned long result = 0;

    for(unsigned int i=0; i<in.size(); i++)
      result += 37*in[i];

    return result;
  }
} // namespace fccl
