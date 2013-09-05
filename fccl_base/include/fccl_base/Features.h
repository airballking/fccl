#ifndef FCCL_BASE_FEATURES_H
#define FCCL_BASE_FEATURES_H

#include <fccl_kdl/Vector.h>
#include <fccl_kdl/Transform.h>
#include <fccl_utils/Hashing.h>
#include <string>
#include <iostream>

namespace fccl
{
  class Feature
  {
    public:
      Feature();
      Feature(const Feature& other);
      Feature(std::size_t id, const fccl::Vector& position);
      Feature(const std::string& name, const fccl::Vector& position);

      virtual ~Feature();

      std::size_t getID() const;
      void setName(const std::string& name);
      void setID(std::size_t it);

      const fccl::Vector& getPosition() const;
      void setPosition(const fccl::Vector& position);

      int getType() const;

      virtual void changeReference(const fccl::Transform& transform) = 0;
      virtual std::size_t getReferenceID() const = 0; 

      bool operator==(const Feature& other) const;
      bool operator!=(const Feature& other) const;

      bool semanticsEqual(const Feature& other) const;
      virtual bool numericsEqual(const Feature& other) const;

    protected:
      // id of the feature given by knowledge base
      std::size_t id_;

      // position of the feature
      fccl::Vector position_;
   
      // integer to uniquely identify every type of feature
      int type_;
  };

  class OrientedFeature: public Feature
  {
    public:
      OrientedFeature();
      OrientedFeature(const OrientedFeature& other);
      OrientedFeature(std::size_t id, const fccl::Vector& position);
      OrientedFeature(const std::string& name, const fccl::Vector& position);

      virtual ~OrientedFeature();

      virtual const fccl::Vector& getOrientation() const = 0;
      virtual void setOrientation(const fccl::Vector& orientation) = 0;

      virtual bool numericsEqual(const Feature& other) const;

      OrientedFeature& operator=(const fccl::OrientedFeature& rhs);

      virtual void changeReference(const fccl::Transform& transform); 
      virtual std::size_t getReferenceID() const; 
  };

  class Point: public Feature
  {
    public:
      Point();
      Point(const Point& other);
      Point(std::size_t id, const fccl::Vector& position);
      Point(const std::string& name, const fccl::Vector& position);

      virtual ~Point();

      Point& operator=(const fccl::Point& rhs);

      virtual void changeReference(const fccl::Transform& transform);
      virtual std::size_t getReferenceID() const; 

      friend std::ostream& operator<<(std::ostream& os, const Point& point);
  };

  class Plane: public OrientedFeature
  {
    public:
      Plane();
      Plane(const Plane& other);
      Plane(std::size_t id, const fccl::Vector& position, const fccl::Vector& normal);
      Plane(const std::string& name, const fccl::Vector& position, const fccl::Vector& normal);

      virtual ~Plane();

      const fccl::Vector& getNormal() const;
      void setNormal(const fccl::Vector& normal);

      virtual const fccl::Vector& getOrientation() const;
      virtual void setOrientation(const fccl::Vector& orientation);

      friend std::ostream& operator<<(std::ostream& os, const Plane& plane);

    private:
      // normal of the plane
      fccl::Vector normal_;
  };

  class Line: public OrientedFeature
  {
    public:
      Line();
      Line(const Line& other);
      Line(std::size_t id, const fccl::Vector& position, const fccl::Vector& direction);
      Line(const std::string& name, const fccl::Vector& position, const fccl::Vector& direction);

      virtual ~Line();

      const fccl::Vector& getDirection() const;
      void setDirection(const fccl::Vector& direction);

      virtual const fccl::Vector& getOrientation() const;
      virtual void setOrientation(const fccl::Vector& orientation);

      friend std::ostream& operator<<(std::ostream& os, const Line& line);

    private:
      // direction of the plane
      fccl::Vector direction_;
  };
  
} // namespace fccl
#endif // FCCL_BASE_FEATURES_H
