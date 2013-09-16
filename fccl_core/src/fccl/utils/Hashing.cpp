#include <fccl/utils/Hashing.h>
#include <assert.h>

std::size_t fccl::utils::hash(const std::string& value)
{
  std::size_t result = string_hash_(value);
  rememberHashValuePair(result, value);

  return result;
}

void fccl::utils::rememberHashValuePair(std::size_t hash, const std::string& value)
{
  std::pair<std::map<std::size_t, std::string>::iterator, bool> ret;
  ret = hash_memory_.insert(std::pair<std::size_t, std::string>(hash, value));
}

const std::string& fccl::utils::retrieveValue(std::size_t hash)
{
  std::map<std::size_t, std::string>::iterator it = hash_memory_.find(hash);

  // TODO(Georg): Turns this into an exception
  assert(it != hash_memory_.end());

  return it->second;
}
