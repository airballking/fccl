#ifndef FCCL_CONTROL_FEATURES_H
#define FCCL_CONTROL_FEATURES_H

#include <fccl_control/Geometry.h>
#include <fccl_control/String.h>

namespace fccl
{
  class Feature
  {
    public:
      Feature();
      Feature(const std::string& name, const fccl::Position& position);

      virtual ~Feature();

      const std::string& getName() const;
      void setName(const std::string& name);

      const fccl::Position& getPosition() const;
      void setPosition(const fccl::Position& position);

      virtual void changeReferenceFrame(const fccl::Transform& transform) = 0;

    protected:
      // name/id of the feature given by knowledge base
      std::string name_;

      // position of the feature
      fccl::Position position_;
  };
} // namespace fccl
#endif // FCCL_CONTROL_FEATURES_H
