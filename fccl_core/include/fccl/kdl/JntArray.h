#ifndef FCCL_KDL_JNTARRAY_H
#define FCCL_KDL_JNTARRAY_H

#include <fccl/kdl/Semantics.h>
#include <kdl/jntarray.hpp>
#include <string>
#include <iostream>

namespace fccl
{
  namespace kdl
  {
    class JntArray : public SemanticObjectN
    {
      public:
        JntArray();
        JntArray(const JntArray& other);
        JntArray(const SemanticObjectN& semantics, const KDL::JntArray& data);

        virtual ~JntArray();

        void init(const SemanticObjectN& semantics);

        JntArray& operator=(const JntArray& other);
        
        const KDL::JntArray& getData() const;
        void setData(const KDL::JntArray& data);

        double& operator()(std::size_t row);
        double operator()(std::size_t row) const;
 
        virtual bool numericsEqual(const JntArray& other) const;
        
        bool operator==(const JntArray& other) const;
        bool operator!=(const JntArray& other) const;

        virtual void resize(std::size_t new_size);
        bool isValid() const;

        friend std::ostream& operator<<(std::ostream& os, 
            const JntArray& jnt_array);
 
        KDL::JntArray data_;
    }; 
  } // namespace kdl
} // namespace fccl
#endif // FCCL_KDL_JNTARRAY_H
