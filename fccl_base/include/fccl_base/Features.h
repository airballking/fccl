#ifndef FCCL_BASE_FEATURES_H
#define FCCL_BASE_FEATURES_H

#include <fccl_base/FeatureTypes.h>
#include <fccl_kdl/Vector.h>
#include <fccl_kdl/Transform.h>
#include <string>
#include <iostream>

namespace fccl
{

  class Feature
  {
    public:
      Feature();
      Feature(const Feature& other);
      Feature(std::size_t id, const fccl::Vector& position, 
          const fccl::Vector& orientation, FeatureTypes type);
      Feature(const std::string& name, const fccl::Vector& position,
          const fccl::Vector& orientation, FeatureTypes type);

      virtual ~Feature();

      Feature& operator=(const Feature& rhs);

      std::size_t getID() const;
      void setName(const std::string& name);
      void setID(std::size_t it);

      const fccl::Vector& getPosition() const;
      void setPosition(const fccl::Vector& position);

      const fccl::Vector& getOrientation() const;
      void setOrientation(const fccl::Vector& orientation);

      FeatureTypes getType() const;
      void setType(FeatureTypes type);

      void changeReference(const fccl::Transform& transform);
      std::size_t getReferenceID() const; 

      virtual bool operator==(const Feature& other) const;
      virtual bool operator!=(const Feature& other) const;

      bool semanticsEqual(const Feature& other) const;
      bool numericsEqual(const Feature& other) const;

      friend std::ostream& operator<<(std::ostream& os, const Feature& feature);

    protected:
      // id of the feature given by knowledge base
      std::size_t id_;

      // position of the feature
      fccl::Vector position_;
   
      // orientation of the feature
      fccl::Vector orientation_;

      // integer to uniquely identify every type of feature
      FeatureTypes type_;
  };

} // namespace fccl
#endif // FCCL_BASE_FEATURES_H
