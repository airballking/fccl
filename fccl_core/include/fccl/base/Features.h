#ifndef FCCL_BASE_FEATURES_H
#define FCCL_BASE_FEATURES_H

#include <fccl/base/FeatureTypes.h>
#include <fccl/kdl/Transform.h>
#include <fccl/kdl/Semantics.h>
#include <kdl/frames.hpp>
#include <string>
#include <iostream>

namespace fccl
{
  namespace base
  {
    class Feature : public fccl::kdl::SemanticObject1x1
    {
      public:
        Feature();
        Feature(const Feature& other);
        Feature(const SemanticObject1x1& semantics, const KDL::Vector& position, 
            const KDL::Vector& orientation, FeatureTypes type);
 
        virtual ~Feature();
  
        Feature& operator=(const Feature& rhs);
  
        const KDL::Vector& getPosition() const;
        void setPosition(const KDL::Vector& position);
  
        const KDL::Vector& getOrientation() const;
        void setOrientation(const KDL::Vector& orientation);
  
        FeatureTypes getType() const;
        void setType(FeatureTypes type);
  
        void changeReference(const fccl::kdl::Transform& transform);
  
        virtual bool operator==(const Feature& other) const;
        virtual bool operator!=(const Feature& other) const;
  
        virtual bool semanticsEqual(const Feature& other) const;
        bool numericsEqual(const Feature& other) const;
  
        friend std::ostream& operator<<(std::ostream& os, const Feature& feature);
  
      protected:
        // position of the feature
        KDL::Vector position_;
     
        // orientation of the feature
        KDL::Vector orientation_;
  
        // integer to uniquely identify every type of feature
        FeatureTypes type_;
    };
  } // namespace base
} // namespace fccl
#endif // FCCL_BASE_FEATURES_H
