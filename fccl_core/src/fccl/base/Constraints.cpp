#include <fccl/base/Constraints.h>

namespace fccl
{
  namespace base
  {
    Constraint::Constraint() : SemanticObject1x1()
    {
      type_ = UNKNOWN_CONSTRAINT;
      first_derivative_.resize(1);
    }
  
    Constraint::Constraint(const Constraint& other) :
        SemanticObject1x1(other), tool_feature_(other.getToolFeature()), 
        object_feature_(other.getObjectFeature()),
        lower_boundary_(other.getLowerBoundary()), 
        upper_boundary_(other.getUpperBoundary())
    {
      type_ = UNKNOWN_CONSTRAINT;
      first_derivative_.resize(1);
    }
  
    Constraint::Constraint(const std::string& reference_name, const std::string&
            target_name, const fccl::base::Feature& tool_feature, 
            const fccl::base::Feature& object_feature, 
            double lower_boundary, double upper_boundary) :
        SemanticObject1x1(reference_name, target_name),
        tool_feature_(tool_feature), object_feature_(object_feature),
        lower_boundary_(lower_boundary), upper_boundary_(upper_boundary)
    {
      type_ = UNKNOWN_CONSTRAINT;
      first_derivative_.resize(1);
    }
  
    Constraint::Constraint(std::size_t reference_id, std::size_t target_id,
            const fccl::base::Feature& tool_feature, 
            const fccl::base::Feature& object_feature,
            double lower_boundary, double upper_boundary) :
        SemanticObject1x1(reference_id, target_id),
        tool_feature_(tool_feature), object_feature_(object_feature),
        lower_boundary_(lower_boundary), upper_boundary_(upper_boundary)
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
        setTargetID(rhs.getTargetID());
      }
  
      return *this; 
    }
  
    const fccl::base::Feature& Constraint::getToolFeature() const
    {
      return tool_feature_;
    }
  
    void Constraint::setToolFeature(const fccl::base::Feature& tool_feature)
    {
      tool_feature_ = tool_feature;
    }
  
    const fccl::base::Feature& Constraint::getObjectFeature() const
    {
      return object_feature_;
    }
  
    void Constraint::setObjectFeature(const fccl::base::Feature& object_feature)
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
  
    double Constraint::calculateValue(const fccl::kdl::Transform& tool_transform,
        const fccl::kdl::Transform& object_transform) const
    {
      return 0.0;
    }
  
    const fccl::kdl::InteractionMatrix& Constraint::calculateFirstDerivative(const
        fccl::kdl::Transform& tool_transform, const fccl::kdl::Transform& 
        object_transform, double delta)
    {
      assert(delta != 0.0);
      assert(first_derivative_.rows() == 1);
      assert(first_derivative_.columns() == 6);
  
      // prepare semantics of result
      first_derivative_.setReferenceID(tool_transform.getTargetID());
      first_derivative_.setTargetID(0, getTargetID());
  
      // get current constraint value around which we differentiate
      double value = calculateValue(tool_transform, object_transform);
  
      // prepare six delta-transforms to simulate delta motions of tool
      fccl::kdl::Transform T[6];
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
          && SemanticObject1x1::semanticsEqual(other);
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
      os << "reference: " << constraint.getReferenceName() << " (";
      os << constraint.getReferenceID() << ")\n";
      os << "target: " << constraint.getTargetName() << " (";
      os << constraint.getTargetID() << ")";
      return os;
    }
  
    AboveConstraint::AboveConstraint() : Constraint()
    {
      type_ = ABOVE_CONSTRAINT;
    }
  
    AboveConstraint::AboveConstraint(const AboveConstraint& other) :
        Constraint(other)
    {
      type_ = ABOVE_CONSTRAINT;
    }
  
    AboveConstraint::AboveConstraint(const std::string& reference_name, 
            const std::string& target_name, const fccl::base::Feature& tool_feature,
            const fccl::base::Feature& object_feature, 
            double lower_boundary, double upper_boundary) :
        Constraint(reference_name, target_name, tool_feature, object_feature, 
            lower_boundary, upper_boundary)
    {
      type_ = ABOVE_CONSTRAINT;
    }
  
    AboveConstraint::AboveConstraint(std::size_t reference_id, 
            const std::size_t target_id, const fccl::base::Feature& tool_feature,
            const fccl::base::Feature& object_feature, 
            double lower_boundary, double upper_boundary) :
        Constraint(reference_id, target_id, tool_feature, object_feature,
            lower_boundary, upper_boundary)
    {
      type_ = ABOVE_CONSTRAINT;
    }
  
    AboveConstraint::~AboveConstraint()
    {
    }
  
    double AboveConstraint::calculateValue(const fccl::kdl::Transform& tool_transform,
        const fccl::kdl::Transform& object_transform) const
    {
      assert(getReferenceID() == tool_transform.getReferenceID());
      assert(getReferenceID() == object_transform.getReferenceID());
      assert(tool_feature_.getReferenceID() == tool_transform.getTargetID());
      assert(object_feature_.getReferenceID() == object_transform.getTargetID());
  
      KDL::Vector tool_position_in_view = tool_transform.getTransform() * tool_feature_.getPosition();
      KDL::Vector object_position_in_view = object_transform.getTransform() * object_feature_.getPosition();
  
      return  tool_position_in_view.z() - object_position_in_view.z();
    }
  } // namespace base
} // namespace fccl
