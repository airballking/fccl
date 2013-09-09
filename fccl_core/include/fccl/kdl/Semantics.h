#ifndef FCCL_KDL_SEMANTICS_H
#define FCCL_KDL_SEMANTICS_H

#include <string>

namespace fccl
{
  namespace kdl
  {
    class TwofoldSemanticObject
    {
      public:
        TwofoldSemanticObject();
        TwofoldSemanticObject(const TwofoldSemanticObject& other);
        TwofoldSemanticObject(const std::string& reference_name, const std::string& target_name);
        TwofoldSemanticObject(std::size_t reference_id, std::size_t target_id);
  
        virtual ~TwofoldSemanticObject();
  
        std::size_t getReferenceID() const;
        void setReferenceID(std::size_t reference_id);
  
        const std::string& getReferenceName() const;
        void setReferenceName(const std::string& reference_name);
  
        std::size_t getTargetID() const;
        void setTargetID(std::size_t target_id);
  
        const std::string& getTargetName() const;
        void setTargetName(const std::string& target_name);
  
        virtual bool semanticsEqual(const TwofoldSemanticObject& other) const;

      protected:
        // hash-ID of the reference
        std::size_t reference_id_;

        // hash-ID of the target
        std::size_t target_id_;
    };
  } // namespace kdl
} // namespace fccl
#endif // FCCL_KDL_SEMANTICS_H
