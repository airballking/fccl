#ifndef FCCL_UTILS_DOUBLE_BUFFER_H
#define FCCL_UTILS_DOUBLE_BUFFER_H

#include <boost/thread/mutex.hpp>
#include <fccl/utils/TransformMap.h>

namespace fccl
{
  namespace utils
  {
    template <class T>
    class DoubleBuffer
    {
      public:
        DoubleBuffer()
        {
          in_buffer_ = new T();
          out_buffer_ = new T();
        }

        ~DoubleBuffer()
        {
          if(in_buffer_)
            delete in_buffer_;
          if(out_buffer_)
            delete out_buffer_;
        }

        T& inBuffer()
        {
          assert(in_buffer_);
          return *in_buffer_;
        }

        const T& inBuffer() const
        {
          assert(in_buffer_);
          return *in_buffer_;
        }

        T& outBuffer()
        {
          assert(out_buffer_);
          return *out_buffer_;
        }

        const T& outBuffer() const
        {
          assert(out_buffer_);
          return *out_buffer_;
        }
     
        void swap()
        {
          boost::mutex::scoped_lock scoped_lock1(outBuffer().getMutex());
          boost::mutex::scoped_lock scoped_lock2(inBuffer().getMutex());

          T* tmp = in_buffer_;
          in_buffer_ = out_buffer_;
          out_buffer_ = tmp;
        }

      private:
        T* in_buffer_;
        T* out_buffer_;

        DoubleBuffer(const DoubleBuffer& other) {}
        DoubleBuffer& operator=(const DoubleBuffer& other) {}
    };

    typedef DoubleBuffer<TransformMap> TransformDoubleBuffer;
  } // namespace utils
} // namespace fccl
#endif // FCCL_UTILS_DOUBLE_BUFFER_H
