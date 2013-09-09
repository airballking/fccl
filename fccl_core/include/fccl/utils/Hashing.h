#ifndef FCCL_UTILS_HASHING_H
#define FCCL_UTILS_HASHING_H

#include <string>
#include <map>

namespace fccl
{
  namespace utils
  {  
    static std::map<std::size_t, std::string> hash_memory_;

    std::size_t hash(const std::string& value);
  
    void rememberHashValuePair(std::size_t hash, const std::string& value);

    const std::string& retrieveValue(std::size_t hash);
  } // namespace fccl::utils
} // namespace fccl
#endif // FCCL_UTILS_HASHING_H
