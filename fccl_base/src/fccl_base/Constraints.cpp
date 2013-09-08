#include <fccl_base/Constraints.h>
#include <fccl_utils/Hashing.h>

namespace fccl
{
  Constraint::Constraint()
  {
    type_ = UNKNOWN_CONSTRAINT;
    first_derivative_.resize(1);
  }

  Constraint::Constraint(const Constraint& other) :
      tool_feature_(other.getToolFeature()), object_feature_(other.getObjectFeature()),
      lower_boundary_(other.getLowerBoundary()), upper_boundary_(other.getUpperBoundary()),
      reference_id_(other.getReferenceID()), id_(other.getID())
  {
    type_ = UNKNOWN_CONSTRAINT;
    first_derivative_.resize(1);
  }

  Constraint::Constraint(const std::string& reference_name, const std::string&
          name, const fccl::Feature& tool_feature, const fccl::Feature& 
          object_feature, double lower_boundary, double upper_boundary) :
      tool_feature_(tool_feature), object_feature_(object_feature),
      lower_boundary_(lower_boundary), upper_boundary_(upper_boundary),
      reference_id_(hash(reference_name)), id_(hash(name))
  {
    type_ = UNKNOWN_CONSTRAINT;
    first_derivative_.resize(1);
  }

  Constraint::Constraint(std::size_t reference_id, std::size_t id,
          const fccl::Feature& tool_feature, const fccl::Feature& object_feature,
          double lower_boundary, double upper_boundary) :
      tool_feature_(tool_feature), object_feature_(object_feature),
      lower_boundary_(lower_boundary), upper_boundary_(upper_boundary),
      reference_id_(reference_id), id_(id)
  {
    type_ = UNKNOWN_CONSTRAINT;
    first_derivative_.resize(1);
  }


  Constraint::~Constraint()
  {
  }

  Constraint& Constraint::operator=(const Constraint& rhs)
  {
    // protect against self-assignment
    if(this != &rhs)
    {
      setToolFeature(rhs.getToolFeature());
      setObjectFeature(rhs.getObjectFeature());
      setLowerBoundary(rhs.getLowerBoundary());
      setUpperBoundary(rhs.getUpperBoundary());
      setReferenceID(rhs.getReferenceID());
      setID(rhs.getID());
    }

    return *this; 
  }

  std::size_t Constraint::getReferenceID() const
  {
    return reference_id_; 
  }

  void Constraint::setReferenceName(const std::string& reference_name)
  {
    reference_id_ = hash(reference_name);
  }

  void Constraint::setReferenceID(std::size_t reference_id)
  {
    reference_id_ = reference_id;
  }

  std::size_t Constraint::getID() const
  {
    return id_;
  }

  void Constraint::setName(const std::string& name)
  {
    id_ = hash(name);
  }

  void Constraint::setID(std::size_t id)
  {
    id_ = id;
  }

  const fccl::Feature& Constraint::getToolFeature() const
  {
    return tool_feature_;
  }

  void Constraint::setToolFeature(const fccl::Feature& tool_feature)
  {
    tool_feature_ = tool_feature;
  }

  const fccl::Feature& Constraint::getObjectFeature() const
  {
    return object_feature_;
  }

  void Constraint::setObjectFeature(const fccl::Feature& object_feature)
  {
    object_feature_ = object_feature;
  }

  double Constraint::getLowerBoundary() const
  {
    return lower_boundary_;
  }

  void Constraint::setLowerBoundary(double lower_boundary)
  {
    lower_boundary_ = lower_boundary;
  }

  double Constraint::getUpperBoundary() const
  {
    return upper_boundary_;
  }

  void Constraint::setUpperBoundary(double upper_boundary)
  {
    upper_boundary_ = upper_boundary;
  }

  int Constraint::getType() const
  {
    return type_;
  }

  double Constraint::calculateValue(const fccl::Transform& tool_transform,
      const fccl::Transform& object_transform) const
  {
    return 0.0;
  }

  const fccl::InteractionMatrix& Constraint::calculateFirstDerivative(const
      fccl::Transform& tool_transform, const fccl::Transform& object_transform,
      double delta)
  {
    assert(delta != 0.0);
    assert(first_derivative_.rows() == 1);
    assert(first_derivative_.columns() == 6);

    // prepare semantics of result
    first_derivative_.setReferenceID(tool_transform.getTargetID());
    first_derivative_.setTargetID(0, getID());

    // get current constraint value around which we differentiate
    double value = calculateValue(tool_transform, object_transform);

    // prepare six delta-transforms to simulate delta motions of tool
    Transform T[6];
    for(unsigned int i=0; i<6; i++)
    {
      T[i].setReferenceID(tool_transform.getTargetID());
      T[i].setTargetID(tool_transform.getTargetID());
    }
    double cd = cos(delta);
    double sd = sin(delta);
    T[0].setTransform(KDL::Frame(KDL::Vector(delta,0,0)));
    T[1].setTransform(KDL::Frame(KDL::Vector(0,delta,0)));
    T[2].setTransform(KDL::Frame(KDL::Vector(0,0,delta)));
    T[3].setTransform(KDL::Frame(KDL::Rotation(1,0,0,  0,cd,-sd,  0,sd,cd)));
    T[4].setTransform(KDL::Frame(KDL::Rotation(cd,0,sd,  0,1,0,  -sd,0,cd)));
    T[5].setTransform(KDL::Frame(KDL::Rotation(cd,-sd,0,  sd,cd,0,  0,0,1)));

     
    // calculate actual numeric calculation
    double delta_r = 1.0 / delta;
    for(unsigned int i=0; i < 6; i++)
    {
      first_derivative_(0,i) = (calculateValue(tool_transform*T[i], object_transform) - value) * delta_r;
    }

    return first_derivative_;
  }

  bool Constraint::operator==(const Constraint& other) const
  {
    return semanticsEqual(other) && numericsEqual(other);
  }

  bool Constraint::operator!=(const Constraint& other) const
  {
    return !(*this == other);
  }

  bool Constraint::semanticsEqual(const Constraint& other) const
  {
    return (getType() == other.getType())
        && (getID() == other.getID())
        && (getReferenceID() == other.getReferenceID());
  }
 
  bool Constraint::numericsEqual(const Constraint& other) const
  {
    return getLowerBoundary() == other.getLowerBoundary()
        && getUpperBoundary() == other.getUpperBoundary()
        && getToolFeature() == other.getToolFeature()
        && getObjectFeature() == other.getObjectFeature();
  }

  std::ostream& operator<<(std::ostream& os, const Constraint& constraint)
  {
    os << "type: " << constraint.getType() << "\n";
    os << "tool_feature:\n" << constraint.getToolFeature() << "\n";
    os << "object_feature:\n" << constraint.getObjectFeature() << "\n";
    os << "lower_boundary: " << constraint.getLowerBoundary();
    os << " upper_boundary: " << constraint.getUpperBoundary() << "\n";
    os << "reference_ID: " << constraint.getReferenceID() << "\n";
    os << "ID: " << constraint.getID();
    return os;
  }

  AboveConstraint::AboveConstraint()
  {
    type_ = ABOVE_CONSTRAINT;
  }

  AboveConstraint::AboveConstraint(const AboveConstraint& other) :
      Constraint(other)
  {
    type_ = ABOVE_CONSTRAINT;
  }

  AboveConstraint::AboveConstraint(const std::string& reference_name, const
          std::string& name, const fccl::Feature& tool_feature, const 
          fccl::Feature& object_feature, double lower_boundary, double 
          upper_boundary) :
      Constraint(reference_name, name, tool_feature, object_feature, lower_boundary,
          upper_boundary)
  {
    type_ = ABOVE_CONSTRAINT;
  }

  AboveConstraint::AboveConstraint(std::size_t reference_id, const std::size_t id,
          const fccl::Feature& tool_feature, const fccl::Feature& object_feature, 
          double lower_boundary, double upper_boundary) :
      Constraint(reference_id, id, tool_feature, object_feature, lower_boundary,
          upper_boundary)
  {
    type_ = ABOVE_CONSTRAINT;
  }

  AboveConstraint::~AboveConstraint()
  {
  }

  double AboveConstraint::calculateValue(const fccl::Transform& tool_transform,
      const fccl::Transform& object_transform) const
  {
    assert(getReferenceID() == tool_transform.getReferenceID());
    assert(getReferenceID() == object_transform.getReferenceID());

    Vector tool_position_in_view = tool_transform * tool_feature_.getPosition();
    Vector object_position_in_view = object_transform * object_feature_.getPosition();

    return  tool_position_in_view.getVector().z() - object_position_in_view.getVector().z();
  }

} // namespace fccl
