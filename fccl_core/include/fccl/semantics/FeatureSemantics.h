#ifndef FCCL_SEMANTICS_FEATURE_SEMANTICS_H
#define FCCL_SEMANTICS_FEATURE_SEMANTICS_H

#include <fccl/semantics/SemanticsBase.h>
#include <fccl/semantics/TransformSemantics.h>
#include <iostream>

namespace fccl
{
  namespace semantics
  {
    enum FeatureTypes
    {
      UNKNOWN_FEATURE = 0,
      
      POINT_FEATURE = 1,
  
      LINE_FEATURE = 2,
  
      PLANE_FEATURE = 3,
  
      FEATURE_COUNT
    };

    inline bool featureTypeValid(int feature_type)
    {
      return (UNKNOWN_FEATURE < feature_type) && (feature_type < FEATURE_COUNT);
    }

    inline FeatureTypes featureTypeFromString(const std::string& type_string)
    {
      if (type_string.compare("POINT") == 0)
        return POINT_FEATURE;
      else if (type_string.compare("LINE") == 0)
        return LINE_FEATURE;
      else if (type_string.compare("PLANE") == 0)
        return PLANE_FEATURE;
      else
        return UNKNOWN_FEATURE;
    }

    // NOTE: Not realtime safe!
    inline std::string featureTypeToString(int feature_type)
    {
      if (feature_type == POINT_FEATURE)
        return "POINT";
      else if (feature_type == LINE_FEATURE)
        return "LINE";
      else if (feature_type == PLANE_FEATURE)
        return "PLANE";
      else
        return "UNKNOWN_FEATURE_TYPE";
    }

    class FeatureSemantics
    {
      public:
        const SemanticsBase& reference() const
        {
          return reference_;
        }

        SemanticsBase& reference()
        {
          return reference_;
        }

        const SemanticsBase& name() const
        {
          return name_;
        }

        SemanticsBase& name()
        {
          return name_;
        }

        const FeatureTypes& type() const
        {
          return type_;
        }

        FeatureTypes& type() 
        {
          return type_;
        }

        bool equals(const FeatureSemantics& other) const
        {
          return (type() == other.type()) &&
              reference().equals(other.reference()) &&
              name().equals(other.name());
        }

        bool isOrientationFeature() const
        {
          return (type() == LINE_FEATURE) || (type() == PLANE_FEATURE);
        }

        bool isValid() const
        {
          return featureTypeValid(type());
        }

        void changeReferenceFrame(const fccl::semantics::TransformSemantics& 
            transform_semantics)
        {
          assert(changeReferencePossible(transform_semantics));

          reference() = transform_semantics.reference();
        }
          
        bool changeReferencePossible(const fccl::semantics::TransformSemantics& 
            transform_semantics)
        {
          return reference().equals(transform_semantics.target());
        }
 
      private:
        SemanticsBase reference_;
        SemanticsBase name_;
        FeatureTypes type_; 
    };

    inline std::ostream& operator<<(std::ostream& os, const FeatureSemantics& obj)
    {
      os << "reference: " << obj.reference() << "\n";
      os << "name: " << obj.name() << "\n";
      os << "type: " << featureTypeToString(obj.type());
      return os;
    }
  } // namespace semantics
} // namespace fccl
#endif // FCCL_SEMANTICS_FEATURE_SEMANTICS_H
