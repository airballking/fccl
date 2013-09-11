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

        JntArray& operator=(const JntArray& other);
        
        SemanticObjectN getSemantics() const;
        void setSemantics(const SemanticObjectN& semantics);

        const KDL::JntArray& getData() const;
        void setData(const KDL::JntArray& data);

        virtual bool numericsEqual(const JntArray& other) const;
        
        bool operator==(const JntArray& other) const;
        bool operator!=(const JntArray& other) const;

        virtual void resize(std::size_t new_size);
        bool isValid() const;

        friend std::ostream& operator<<(std::ostream& os, 
            const JntArray& jnt_array);
 
      private:
        KDL::JntArray data_;
    }; 
  } // namespace kdl
} // namespace fccl
#endif // FCCL_KDL_JNTARRAY_H
