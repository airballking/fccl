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
      Feature(const std::string& name, const fccl::Vector& position);

      virtual ~Feature();

      const std::string& getName() const;
      void setName(const std::string& name);

      const fccl::Vector& getPosition() const;
      void setPosition(const fccl::Vector& position);

      virtual void changeReferenceFrame(const fccl::Transform& transform) = 0;

    protected:
      // name/id of the feature given by knowledge base
      std::string name_;

      // position of the feature
      fccl::Vector position_;
  };

  class Point: public Feature
  {
    public:
      Point();
      Point(const std::string& name, const fccl::Vector& position);

      virtual ~Point();

      void changeReferenceFrame(const fccl::Transform& transform);
  };

} // namespace fccl
#endif // FCCL_CONTROL_FEATURES_H
