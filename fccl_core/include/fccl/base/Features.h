#ifndef FCCL_BASE_FEATURES_H
#define FCCL_BASE_FEATURES_H

#include <fccl/semantics/FeatureSemantics.h>
#include <kdl/frames.hpp>
#include <fccl/kdl/Transform.h>
#include <fccl/utils/Printing.h>
#include <fccl/utils/Printing.h>
#include <iostream>

namespace fccl
{
  namespace base
  {
    class Feature
    {
      public:
        const KDL::Vector& position() const
        {
          return position_;
        }

        KDL::Vector& position() 
        {
          return position_;
        }

        const KDL::Vector& orientation() const
        {
          return orientation_;
        }

        KDL::Vector& orientation() 
        {
          return orientation_;
        }

        const fccl::semantics::FeatureSemantics& semantics() const
        {
          return semantics_;
        }

        fccl::semantics::FeatureSemantics& semantics() 
        {
          return semantics_;
        }
  
        bool equals(const Feature& other) const
        {
          return semantics().equals(other.semantics()) &&
            KDL::Equal(position(), other.position()) &&
            KDL::Equal(orientation(), other.orientation());
        }

        bool isValid() const
        {
          return semantics().isValid();
        }

        void changeReferenceFrame(const fccl::kdl::Transform& transform)
        {
          semantics().changeReferenceFrame(transform.semantics());

          position() = transform.numerics() * position();
          orientation() = transform.numerics() * orientation();
        }
          
        bool changeReferencePossible(const fccl::kdl::Transform& transform)
        {
          return semantics().changeReferencePossible(transform.semantics());
        }
  
      private:
        // position of the feature
        KDL::Vector position_;
     
        // orientation of the feature
        KDL::Vector orientation_;
  
        // semantics of this feature
        fccl::semantics::FeatureSemantics semantics_;
    };

    inline std::ostream& operator<<(std::ostream& os, const Feature& feature)
    {
      using fccl::utils::operator<<;

      os << "semantics: " << feature.semantics() << "\n";
      os << "position: " << feature.position() << "\n";
      os << "orientation: " << feature.orientation();

      return os;
    }
  } // namespace base
} // namespace fccl
#endif // FCCL_BASE_FEATURES_H
