#ifndef FCCL_UTILS_HASHING_H
#define FCCL_UTILS_HASHING_H

#include <string>
#include <map>
#include <boost/functional/hash.hpp>

namespace fccl
{
  namespace utils
  {  
    static std::map<std::size_t, std::string> hash_memory_;

    static boost::hash<std::string> string_hash_;

    bool hasValue(std::size_t hash);

    std::size_t hash(const std::string& value);
  
    void rememberHashValuePair(std::size_t hash, const std::string& value);

    const std::string& retrieveValue(std::size_t hash);
  } // namespace fccl::utils
} // namespace fccl
#endif // FCCL_UTILS_HASHING_H
