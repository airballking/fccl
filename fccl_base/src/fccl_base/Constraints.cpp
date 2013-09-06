#include <fccl_base/Constraints.h>
#include <fccl_utils/Hashing.h>

namespace fccl
{
  Constraint::Constraint()
  {
    type_ = -1;
  }

  Constraint::Constraint(const Constraint& other) :
      tool_feature_(other.getToolFeature()), object_feature_(other.getObjectFeature()),
      lower_boundary_(other.getLowerBoundary()), upper_boundary_(other.getUpperBoundary()),
      reference_id_(other.getReferenceID()), type_(other.getType()),
      T_tool_feature_in_constraint_frame_(other.getToolTransform()),
      T_object_feature_in_constraint_frame_(other.getObjectTransform())
  {
  }

  Constraint::Constraint(const std::string& reference_name, int type,
      const fccl::Feature& tool_feature, const fccl::Feature& object_feature,
      double lower_boundary, double upper_boundary,
      const fccl::Transform& tool_transform, const fccl::Transform& object_transform) :
      tool_feature_(tool_feature), object_feature_(object_feature),
      lower_boundary_(lower_boundary), upper_boundary_(upper_boundary),
      reference_id_(hash(reference_name)), type_(type),
      T_tool_feature_in_constraint_frame_(tool_transform),
      T_object_feature_in_constraint_frame_(object_transform)
  {
  }


  Constraint::Constraint(std::size_t reference_id, int type,
      const fccl::Feature& tool_feature, const fccl::Feature& object_feature,
      double lower_boundary, double upper_boundary,
      const fccl::Transform& tool_transform, const fccl::Transform& object_transform) :
      tool_feature_(tool_feature), object_feature_(object_feature),
      lower_boundary_(lower_boundary), upper_boundary_(upper_boundary),
      reference_id_(reference_id), type_(type),
      T_tool_feature_in_constraint_frame_(tool_transform),
      T_object_feature_in_constraint_frame_(object_transform)
  {
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
      setType(rhs.getType());
      setToolTransform(rhs.getToolTransform());
      setObjectTransform(rhs.getObjectTransform());
    }

    return *this; 
  }

  std::size_t Constraint::getReferenceID() const
  {
    return reference_id_; }

  void Constraint::setReferenceName(const std::string& reference_name)
  {
    reference_id_ = hash(reference_name);
  }

  void Constraint::setReferenceID(std::size_t reference_id)
  {
    reference_id_ = reference_id;
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
    object_feature_ = object_feature_;
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

  const fccl::Transform& Constraint::getToolTransform() const
  {
    return T_tool_feature_in_constraint_frame_;
  }

  void Constraint::setToolTransform(const fccl::Transform& tool_transform)
  {
    T_tool_feature_in_constraint_frame_ = tool_transform;
  }

  const fccl::Transform& Constraint::getObjectTransform() const
  {
    return T_object_feature_in_constraint_frame_;
  }

  void Constraint::setObjectTransform(const fccl::Transform& object_transform)
  {
    T_object_feature_in_constraint_frame_ = object_transform;
  }

  int Constraint::getType() const
  {
    return type_;
  }

  double Constraint::calculateValue() const
  {
    return 0.0;
  }

  InteractionMatrix Constraint::calculateDerivate() const
  {
    // TODO(Georg): implement me
    InteractionMatrix result;

    return result;
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
        && (getReferenceID() == other.getReferenceID());
  }
 
  bool Constraint::numericsEqual(const Constraint& other) const
  {
    return getLowerBoundary() == other.getLowerBoundary()
        && getUpperBoundary() == other.getUpperBoundary()
        && getToolFeature() == other.getToolFeature()
        && getObjectFeature() == other.getObjectFeature()
        && getToolTransform() == other.getToolTransform()
        && getObjectTransform() == other.getObjectTransform();
  }

} // namespace fccl
