#ifndef FCCL_CONTROL_STRING_H
#define FCCL_CONTROL_STRING_H

#include <string>

namespace fccl
{

  // In theory, having strings in objects screws up realtime safety of creation!
  // (there is _no_ copy-on-write implementation for the GNU std::string) For
  // the time being, we just reserve 256 bytes for every string. This is a work-
  // around because realtime may get a hickup for longer names.
  #define STRING_SIZE 256

} // namespace fccl
#endif // FCCL_CONTROL_STRING_H
