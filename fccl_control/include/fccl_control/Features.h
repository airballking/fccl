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

  class Plane: public Feature
  {
    public:
      Plane();
      Plane(const std::string& name, const fccl::Vector& position, const fccl::Vector& normal);

      virtual ~Plane();

      const fccl::Vector& getNormal() const;
      void setNormal(const fccl::Vector& normal);

      void changeReferenceFrame(const fccl::Transform& transform);
    
    private:
      // normal of the plane
      fccl::Vector normal_;
  };

  class Line: public Feature
  {
    public:
      Line();
      Line(const std::string& name, const fccl::Vector& position, const fccl::Vector& direction);

      virtual ~Line();

      const fccl::Vector& getDirection() const;
      void setDirection(const fccl::Vector& direction);

      void changeReferenceFrame(const fccl::Transform& transform);
    
    private:
      // direction of the plane
      fccl::Vector direction_;
  };
  
} // namespace fccl
#endif // FCCL_CONTROL_FEATURES_H
