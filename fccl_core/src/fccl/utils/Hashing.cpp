#include <fccl/utils/Hashing.h>

std::size_t fccl::utils::hash(const std::string& in)
{
  std::size_t result = 0;

  for(unsigned int i=0; i<in.size(); i++)
    result += 101*in[i];

  return result;
}
