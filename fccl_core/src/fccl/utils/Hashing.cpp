#include <fccl/utils/Hashing.h>
#include <assert.h>

namespace fccl
{
  namespace utils
  {
    std::size_t Hasher::hash(const std::string& value)
    {
      std::size_t result = getHashFunction()(value);
      rememberHashValuePair(result, value);
    
      return result;
    }
    
    void Hasher::rememberHashValuePair(std::size_t hash, const std::string& value)
    {
      std::pair<std::map<std::size_t, std::string>::iterator, bool> ret;
      ret = getMap().insert(std::pair<std::size_t, std::string>(hash, value));
    }

    bool Hasher::hasValue(std::size_t hash)
    {
      std::map<std::size_t, std::string>::iterator it = getMap().find(hash);
    
      return it != getMap().end();
    }
    
    const std::string& Hasher::retrieveValue(std::size_t hash)
    {
      if(!hasValue(hash))
        return getEmptyString();
    
      std::map<std::size_t, std::string>::iterator it = getMap().find(hash);
    
      return it->second;
    }

    std::map<std::size_t, std::string>& Hasher::getMap()
    {
      static std::map<std::size_t, std::string> map;
      return map;
    }

    const std::string& Hasher::getEmptyString()
    {
      static const std::string empty_string = "";
      return empty_string;
    }

    boost::hash<std::string>& Hasher::getHashFunction()
    {
      static boost::hash<std::string> string_hasher;
      return string_hasher;
    }    
  } // namespace utils
} // namespace fccl
