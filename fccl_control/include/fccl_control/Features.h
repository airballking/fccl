#ifndef FCCL_CONTROL_FEATURES_H
#define FCCL_CONTROL_FEATURES_H

#include <fccl_control/Geometry.h>
#include <fccl_control/String.h>

namespace fccl
{
  class Feature
  {
    public:
      Feature()
      {
        name_.reserve(STRING_SIZE);
      }

      Feature(const std::string& name, const fccl::Position& position) :
          name_(name), position_(position)
      {
        name_.reserve(STRING_SIZE);
      }

      virtual ~Feature() {}

      const std::string& getName() const
      {
        return name_;
      }

      void setName(const std::string& name)
      {
        name_ = name;
      }

      const fccl::Position& getPosition() const
      {
        return position_;
      }

      void setPosition(const fccl::Position& position)
      {
        position_ = position;
      }

      virtual void changeReferenceFrame(const fccl::Transform& transform) = 0;

    private:
      // name/id of the feature given by knowledge base
      std::string name_;

      // position of the feature
      fccl::Position position_;
  };
} // namespace fccl
#endif // FCCL_CONTROL_FEATURES_H
