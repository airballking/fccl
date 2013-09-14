#ifndef FCCL_KDL_SEMANTICS_H
#define FCCL_KDL_SEMANTICS_H

#include <string>
#include <vector>
#include <iostream>

namespace fccl
{
  namespace kdl
  {
    enum SemanticTypes
    {
      SEMANTICS_UNKNOWN = 0,

      SEMANTICS_1x1 = 1,

      SEMANTICS_1xN = 2,

      SEMANTICS_NxN = 3,

      SEMANTICS_N = 4,

      SEMANTICS_COUNT
    };

    class SemanticObject
    {
      public:
        int getSemanticType() const;

        virtual bool semanticsEqual(const SemanticObject& other) const = 0;

        bool semanticTypesEqual(const SemanticObject& other) const;

      protected:
        int semantic_type_;
    };

    class SemanticObject1x1 : public SemanticObject
    {
      public:
        SemanticObject1x1();
        SemanticObject1x1(const SemanticObject1x1& other);
        SemanticObject1x1(const std::string& reference_name, const std::string& target_name);
        SemanticObject1x1(std::size_t reference_id, std::size_t target_id);
  
        virtual ~SemanticObject1x1();

        SemanticObject1x1& operator=(const SemanticObject1x1& other);

        const SemanticObject1x1& getSemantics() const;
        void setSemantics(const SemanticObject1x1& other);
  
        std::size_t getReferenceID() const;
        void setReferenceID(std::size_t reference_id);
  
        const std::string& getReferenceName() const;
        void setReferenceName(const std::string& reference_name);
  
        std::size_t getTargetID() const;
        void setTargetID(std::size_t target_id);
  
        const std::string& getTargetName() const;
        void setTargetName(const std::string& target_name);
  
        virtual bool semanticsEqual(const SemanticObject& other) const;

        SemanticObject1x1 inverse() const;

      protected:
        // internal representation of the semantics:
        // - first element of the pair holds the hash value of the reference name
        // - second element of the pair holds the hash value of the reference name
        std::pair<std::size_t, std::size_t> semantics_;
    };

    class SemanticObject1xN : public SemanticObject
    {
      public:
        SemanticObject1xN();
        SemanticObject1xN(const SemanticObject1xN& other);
        SemanticObject1xN(const std::string& reference_name, const 
            std::vector<std::string>& target_names);
        SemanticObject1xN(std::size_t reference_id, const std::vector<std::size_t>&
            target_ids);
  
        virtual ~SemanticObject1xN();
  
        SemanticObject1xN& operator=(const SemanticObject1xN& other);

        const SemanticObject1xN& getSemantics() const;
        void setSemantics(const SemanticObject1xN& semantics);

        std::size_t getReferenceID() const;
        void setReferenceID(std::size_t reference_id);
  
        const std::string& getReferenceName() const;
        void setReferenceName(const std::string& reference_name);
  
        std::size_t getTargetID(std::size_t index) const;
        void setTargetID(std::size_t index, std::size_t target_id);

        const std::string& getTargetName(std::size_t index) const;
        void setTargetName(std::size_t index, const std::string& target_name);

        const std::vector<std::size_t>& getTargetIDs() const;
        void setTargetIDs(const std::vector<std::size_t>& target_ids);
  
        // not real-time safe!
        std::vector<std::string> getTargetNames() const;
        void setTargetNames(const std::vector<std::string>& target_names);
  
        virtual bool semanticsEqual(const SemanticObject& other) const;

        std::size_t size() const;
        void resize(std::size_t new_size);

        bool targetIndexValid(std::size_t index) const;
        std::size_t getTargetIndex(size_t target_id) const;
        std::size_t getTargetIndex(const std::string& target_name) const;

        friend std::ostream& operator<<(std::ostream& os, 
            const SemanticObject1xN& semantics);
 
      protected:
        // hash-ID of the reference
        std::size_t reference_id_;

        // hash-ID of the target
        std::vector<std::size_t> target_ids_;
    };

    class SemanticObjectN : public SemanticObject
    {
      public:
        SemanticObjectN();
        SemanticObjectN(const SemanticObjectN& other);
        SemanticObjectN(const std::vector<std::size_t>& target_ids);
        SemanticObjectN(const std::vector<std::string>& target_names);
  
        virtual ~SemanticObjectN();

        SemanticObjectN& operator=(const SemanticObjectN& other);

        const SemanticObjectN& getSemantics() const;
        void setSemantics(const SemanticObjectN& semantics);
 
        std::size_t getTargetID(std::size_t index) const;
        void setTargetID(std::size_t index, std::size_t target_id);

        const std::string& getTargetName(std::size_t index) const;
        void setTargetName(std::size_t index, const std::string& target_name);

        const std::vector<std::size_t>& getTargetIDs() const;
        void setTargetIDs(const std::vector<std::size_t>& target_ids);
  
        // not real-time safe!
        std::vector<std::string> getTargetNames() const;
        void setTargetNames(const std::vector<std::string>& target_names);
  
        virtual bool semanticsEqual(const SemanticObject& other) const;

        std::size_t size() const;
        void resize(std::size_t new_size);

        bool indexValid(std::size_t index) const;
        std::size_t getIndex(size_t target_id) const;
        std::size_t getIndex(const std::string& target_name) const;

      protected:
        // hash-ID of the target
        std::vector<std::size_t> target_ids_;
    };

  } // namespace kdl
} // namespace fccl
#endif // FCCL_KDL_SEMANTICS_H
