#include <fccl_base/Features.h>
#include <fccl_utils/Hashing.h>
#include <exception>

namespace fccl
{

  Feature::Feature()
  {
    type_ = -1;
  }

  Feature::Feature(const Feature& other) :
      id_(other.getID()), position_(other.getPosition()), type_(other.getType())
  {
  }

  Feature::Feature(std::size_t id, const fccl::Vector& position) :
      id_(id), position_(position)
  {
    type_ = -1;
  }
 
  Feature::Feature(const std::string& name, const fccl::Vector& position) :
      id_(hash(name)), position_(position)
  {
    type_ = -1;
  }

  Feature::~Feature() {}

  std::size_t Feature::getID() const
  {
    return id_;
  }

  void Feature::setName(const std::string& name)
  {
    id_ = hash(name);
  }

  void Feature::setID(std::size_t id)
  {
    id_ = id;
  }

  const fccl::Vector& Feature::getPosition() const
  {
    return position_;
  }

  void Feature::setPosition(const fccl::Vector& position)
  {
    position_ = position;
  }

  int Feature::getType() const
  {
    return type_;
  }

  bool Feature::operator==(const Feature& other) const
  {
    return (type_ == other.getType())
        && (id_ == other.getID())
        && (position_ == other.getPosition());
  }

  bool Feature::operator!=(const Feature& other) const
  {
    return !(*this == other);
  }

  OrientedFeature::OrientedFeature() : Feature()
  {
    type_ = -2;
  }

  OrientedFeature::OrientedFeature(const OrientedFeature& other) :
      Feature(other)
  {
    type_ = -2;
  }

  OrientedFeature::OrientedFeature(std::size_t id, const fccl::Vector& position) : 
      Feature(id, position)
  {
    type_ = -2;
  }

  OrientedFeature::OrientedFeature(const std::string& name, const fccl::Vector& position) :
      Feature(name, position)
  {
    type_ =-2;
  }

  OrientedFeature::~OrientedFeature()
  {
  }

  bool OrientedFeature::operator==(const Feature& other) const
  {
    if(type_ != other.getType())
      return false;
    
    try 
    {
      const OrientedFeature* other_pointer = dynamic_cast<const OrientedFeature*>(&other);

      assert(other_pointer);

      return (id_ == other_pointer->getID())
          && (position_ == other_pointer->getPosition())
          && (this->getOrientation() == other_pointer->getOrientation());
    } 
    catch (std::exception& e) 
    {
      std::cout << "Exception in OrientedFeature::operator==: " << e.what();
    }

    return false;
  }

  bool OrientedFeature::operator!=(const Feature& other) const
  {
    return !(*this == other);
  }

  OrientedFeature& OrientedFeature::operator=(const fccl::OrientedFeature& rhs)
  {
    // protected against self assignment
    if(this != &rhs)
    {
      this->setID(rhs.getID());
      this->setPosition(rhs.getPosition());
      this->setOrientation(rhs.getOrientation());
    }

    return *this;
  }

  void OrientedFeature::changeReference(const fccl::Transform& transform)
  {
    setPosition(transform * getPosition());
    setOrientation(transform * getOrientation());
  }

  std::size_t OrientedFeature::getReferenceID() const
  {
    assert(getPosition().getReferenceID() == getOrientation().getReferenceID());

    return getPosition().getReferenceID();
  }
 
  Point::Point() : Feature()
  {
    type_ = 1;
  }

  Point::Point(const Point& other) : Feature(other)
  {
    type_ = 1;
  }

  Point::Point(std::size_t id, const fccl::Vector& position) : 
      Feature(id, position)
  {
    type_ = 1;
  }

  Point::Point(const std::string& name, const fccl::Vector& position) :
      Feature(name, position)
  {
    type_ = 1;
  }

  Point::~Point()
  {
  }

  Point& Point::operator=(const fccl::Point& rhs)
  {
    // protect against self-assignment
    if(this != &rhs)
    {
      position_ = rhs.getPosition();
      id_ = rhs.getID();
    }

    return *this;
  }

  void Point::changeReference(const fccl::Transform& transform)
  {
    position_.changeReferenceFrame(transform);
  }

  std::size_t Point::getReferenceID() const
  {
    return position_.getReferenceID();
  }

  std::ostream& operator<<(std::ostream& os, const Point& point)
  {
    os << "position:\n" << point.getPosition() << "\n";
    os << "id: " << point.getID() << "\n";
    os << "type: " << point.getType();

    return os;
  }

  Plane::Plane() : OrientedFeature()
  {
    type_ = 2;
  }

  Plane::Plane(const Plane& other) : OrientedFeature(other), normal_(other.getNormal())
  {
    type_ = 2;
  }

  Plane::Plane(std::size_t id, const fccl::Vector& position, const fccl::Vector& normal) :
      OrientedFeature(id, position), normal_(normal)
  {
    type_ = 2;
  }

  Plane::Plane(const std::string& name, const fccl::Vector& position, const fccl::Vector& normal) :
      OrientedFeature(name, position), normal_(normal)
  {
    type_ = 2;
  }

  Plane::~Plane()
  {
  }

  const fccl::Vector& Plane::getNormal() const
  {
    return normal_;
  }

  void Plane::setNormal(const fccl::Vector& normal)
  {
    normal_ = normal;
  }

  const fccl::Vector& Plane::getOrientation() const
  {
    return getNormal();
  }

  void Plane::setOrientation(const fccl::Vector& orientation)
  {
    setNormal(orientation);
  }

  std::ostream& operator<<(std::ostream& os, const Plane& plane)
  {
    os << "position:\n" << plane.getPosition() << "\n";
    os << "normal:\n" << plane.getNormal() << "\n";
    os << "id: " << plane.getID() << "\n";
    os << "type: " << plane.getType();

    return os;
  }

  Line::Line() : OrientedFeature()
  {
    type_ = 3;
  }

  Line::Line(const Line& other) : OrientedFeature(other), direction_(other.getDirection())
  {
    type_ = 3;
  }

  Line::Line(std::size_t id, const fccl::Vector& position, const fccl::Vector& direction) :
      OrientedFeature(id, position), direction_(direction)
  {
    type_ = 3;
  }

  Line::Line(const std::string& name, const fccl::Vector& position, const fccl::Vector& direction) :
      OrientedFeature(name, position), direction_(direction)
  {
    type_ = 3;
  }

  Line::~Line()
  {
  }

  const fccl::Vector& Line::getDirection() const
  {
    return direction_;
  }

  void Line::setDirection(const fccl::Vector& direction)
  {
    direction_ = direction;
  }

  const fccl::Vector& Line::getOrientation() const
  {
    return getDirection();
  } 

  void Line::setOrientation(const fccl::Vector& orientation)
  {
    setDirection(orientation);
  }

  std::ostream& operator<<(std::ostream& os, const Line& line)
  {
    os << "position:\n" << line.getPosition() << "\n";
    os << "direction:\n" << line.getDirection() << "\n";
    os << "id: " << line.getID() << "\n";
    os << "type: " << line.getType();

    return os;
  }

} // namespace fccl
