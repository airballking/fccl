#ifndef FCCL_UTILS_HASHING_H
#define FCCL_UTILS_HASHING_H

#include <string>
#include <map>
#include <boost/functional/hash.hpp>

namespace fccl
{
  namespace utils
  {  
    class Hasher
    {
      public:
        static std::size_t hash(const std::string& value); 
        static const std::string& retrieveValue(std::size_t hash);

      private:
        // auxiliary functions
        static bool hasValue(std::size_t hash);
        static void rememberHashValuePair(std::size_t hash, const std::string& value);

        // internal getters to members and constants; need to avoid the
        // 'static initialization order fiasco'
        static std::map<std::size_t, std::string>& getMap();
        static const std::string& getEmptyString();
        static boost::hash<std::string>& getHashFunction();
    };
  } // namespace fccl::utils
} // namespace fccl
#endif // FCCL_UTILS_HASHING_H
