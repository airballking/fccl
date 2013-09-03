#include <fccl_control/Geometry.h>

namespace fccl
{
  Transform::Transform()
  {
    parent_frame_.reserve(STRING_SIZE);
    child_frame_.reserve(STRING_SIZE);
    tmp_string_.reserve(STRING_SIZE);
  }

  Transform::Transform(const std::string& parent_frame, const std::string& child_frame, const KDL::Frame transform) : 
      parent_frame_(parent_frame), child_frame_(child_frame), transform_(transform)
  {
    parent_frame_.reserve(STRING_SIZE);
    child_frame_.reserve(STRING_SIZE);
    tmp_string_.reserve(STRING_SIZE);
  }

  Transform::Transform(const Transform& other) :
      parent_frame_(other.getParentFrame()), child_frame_(other.getChildFrame()),
      transform_(other.getTransform())
  {
    parent_frame_.reserve(STRING_SIZE);
    child_frame_.reserve(STRING_SIZE);
    tmp_string_.reserve(STRING_SIZE);
  }
 
  Transform::~Transform()
  {
  }

  const std::string& Transform::getParentFrame() const
  { 
    return parent_frame_;
  }

  void Transform::setParentFrame(const std::string& parent_frame)
  { 
    parent_frame_ = parent_frame;
  }

  const std::string& Transform::getChildFrame() const
  {
    return child_frame_;
  }

  void Transform::setChildFrame(const std::string child_frame)
  {
    child_frame_ = child_frame;
  }

  const KDL::Frame& Transform::getTransform() const
  {
    return transform_;
  }

  void Transform::setTransform(const KDL::Frame& transform)
  {
    transform_ = transform;
  }

  void Transform::invert()
  {
    // swap semantics
    tmp_string_ = parent_frame_;
    parent_frame_ = child_frame_;
    child_frame_ = tmp_string_;
  
    // change the numerics, i.e. perform the actual calculation
    transform_ = transform_.Inverse();
  }

  void Transform::operator*=(const Transform& other)
  {
    assert(child_frame_.compare(other.getParentFrame()) == 0);
    
    // change semantics
    child_frame_ = other.getChildFrame();
    
    // perform calculation
    transform_ = transform_ * other.getTransform();
  }

  std::ostream& operator<<(std::ostream& os, const Transform& transform)
  {
    KDL::Frame f = transform.getTransform();
    os << "p:\n  " << f.p.x() << " " << f.p.y() << " " << f.p.z() << "\n";
    os << "M:\n";
    for(unsigned int i=0; i<3; i++)
    {
      os << "  ";
      for(unsigned int j=0; j<3; j++)
        os << f.M(i,j) << " ";
      os << "\n";
    }   
    os << "parent: " << transform.getParentFrame() << "\n";
    os << "child: " << transform.getChildFrame() << "\n";

    return os;
  }

  Vector::Vector()
  {
    frame_name_.reserve(STRING_SIZE);
  }

  Vector::Vector(const KDL::Vector& vector, const std::string& frame_name) :
      frame_name_(frame_name), vector_(vector)
  {
    frame_name_.reserve(STRING_SIZE);
  }
  
  Vector::~Vector() {}

  const std::string& Vector::getFrameName() const
  {
    return frame_name_;
  }

  void Vector::setFrameName(const std::string& frame_name)
  {
    frame_name_ = frame_name;
  }

  const KDL::Vector& Vector::getVector() const
  {
    return vector_;
  }

  void Vector::setVector(const KDL::Vector& vector)
  {
    vector_ = vector;
  }

  bool Vector::isTransformApplicable(const fccl::Transform& transform) const
  {
    return (frame_name_.compare(transform.getParentFrame()) == 0);
  }

  void Vector::changeReferenceFrame(const fccl::Transform& transform)
  {
    assert(isTransformApplicable(transform));

    frame_name_ = transform.getChildFrame();
    vector_ = transform.getTransform().Inverse(vector_);
  }

  bool Vector::operator==(const Vector &other) const {
    return (frame_name_.compare(other.getFrameName()) == 0)
       && (KDL::Equal(vector_, other.getVector()));
  }

  bool Vector::operator!=(const Vector &other) const
  {
    return !(*this == other);
  }

  TwistDerivative::TwistDerivative()
  {
    reference_frame_.reserve(STRING_SIZE);
    function_name_.reserve(STRING_SIZE);

    for(unsigned int i=0; i<6; i++)
      data(0, i) = 0.0;
  }

  TwistDerivative::TwistDerivative(const std::string& referenceFrame, const std::string& functionName) :
      reference_frame_(referenceFrame), function_name_(functionName)
  {
    reference_frame_.reserve(STRING_SIZE);
    function_name_.reserve(STRING_SIZE);

    for(unsigned int i=0; i<6; i++)
      data(0, i) = 0.0;
  }

  const std::string& TwistDerivative::getReferenceFrame() const
  {
    return reference_frame_;
  }

  void TwistDerivative::setReferenceFrame(const std::string& referenceFrame)
  {
    reference_frame_ = referenceFrame;
  }

  const std::string& TwistDerivative::getFunctionName() const
  {
    return function_name_;
  }

  void TwistDerivative::setFunctionName(const std::string& functionName)
  {
    function_name_ = functionName;
  }

  void TwistDerivative::changeReferenceFrame(const fccl::Transform& transform)
  {
    // Long explanation...
    // This data structure is basically an interaction matrix (H) mapping from
    // the space of derivatives from our function to the space of twist.
    //
    // So, for the sake of simplicity one could think of it as a transposed twist.
    // Right now, this twist is expressed w.r.t. to reference_frame_. We are using
    // Plücker coordinate transformations to change this reference frame.
    //
    // The Plücker coordinate transform B_X_A transforms a twist t_A which is
    // defined w.r.t. frame A into a twist t_B which is defined w.r.t. frame B:
    //
    //     (1) t_B = B_X_A * t_B.
    // 
    // However, we do have transposed twists here. This means:
    //
    //     (2) t_B.transpose = (B_X_A * t_A).transpose()
    //
    //                       = t_A.transpose() * B_X_A.transpose()
    //
    // With the help of some formulae from page 39 of the handbook of robotics
    // I proved to myself that the following holds for Plücker coordinate
    // transforms:
    //
    //                             [ B_R_A.transpose   -B_R_A.tranpose * S(B_p_A) ]
    //     (3) B_X_A.transpose() = [                                              ] 
    //                             [ 0                  B_R_A.transpose           ]
    //
    // where the homogeneous transform B_T_A from frame B to frame A contains
    // the rotation B_R_A and translation B_p_A, and S(v) is the skew-matrix of a
    // given 3D-vector v (also see handbook of robotics page 39 for skew-matrix).
    
    assert(reference_frame_.compare(transform.getParentFrame()) == 0);

    // perform the actual calculation 
    data = data*getTransposedTwistTransformationMatrix(transform.getTransform()); 
  
    // update the reference frame
    reference_frame_ = transform.getChildFrame();
  }

  double& TwistDerivative::operator()(int index)
  {
    assert(0 <= index && index <= 6);

    return data(0, index);
  }

  double TwistDerivative::operator()(int index) const
  {
    assert(0 <= index && index <= 6);

    return data(0, index);
  }

  double& TwistDerivative::operator[](int index)
  {
    assert(0 <= index && index <= 6);

    return data(0, index);
  }

  double TwistDerivative::operator[](int index) const
  {
    assert(0 <= index && index <= 6);

    return data(0, index);
  }

  void TwistDerivative::setZero()
  {
    for(unsigned int i=0; i<6; i++)
      data(0, i) = 0.0;
  }

  Eigen::Matrix< double, 6, 6> getTransposedTwistTransformationMatrix(const KDL::Frame& frame)
  {
    // Produces transpose of B_X_A which transforms twist t_A expressed w.r.t.
    // frame A into a twist t_B which is defined w.r.t. to frame B.
    //
    //     (1) t_B = B_X_A * t_A
    //
    //     (2) t_B.transpose = t_A.transpose * B_X_A.transpose
    //
    // frame is transform B_T_A, i.e. pose of frame A w.r.t. frame B.

    // copy frame because some of Eigens functions do not guarantee constness
    KDL::Frame f = frame;

    // (transposed) Rotation matrix of f
    Eigen::Matrix3d Rt = Eigen::Map<Eigen::Matrix3d>(f.M.data);
  
    double x = f.p.x(), y = f.p.y(), z = f.p.z();
  
    // Skew symmetric matrix of p, [p]_x for expressing a cross product
    Eigen::Matrix3d px;
    px << 0, -z,  y,
          z,  0, -x,
         -y,  x,  0;
  
    // the inverse twist projection matrix
    Eigen::Matrix<double, 6, 6> Mi;
    Mi.block<3,3>(0,0) = Rt;
    Mi.block<3,3>(3,3) = Rt;
    Mi.block<3,3>(0,3) = -Rt*px;
    Mi.block<3,3>(3,0) = Eigen::Matrix3d::Zero();

    return Mi;
  }

  InteractionMatrix::InteractionMatrix()
  {
    reference_frame_.reserve(STRING_SIZE);
    function_names_.clear();
  }

  InteractionMatrix::InteractionMatrix(unsigned int numberOfFunctions, const std::string&
      referenceFrame)
  {
    reference_frame_.reserve(STRING_SIZE);
    function_names_.resize(numberOfFunctions);
    for(unsigned int i=0; i<function_names_.size(); i++)
      function_names_[i].reserve(STRING_SIZE);
    data_.resize(numberOfFunctions, 6);
  } 

  void InteractionMatrix::resize(unsigned int numberOfFunctions)
  {
    function_names_.resize(numberOfFunctions);
    for(unsigned int i=0; i<function_names_.size(); i++)
      function_names_[i].reserve(STRING_SIZE);
    data_.resize(numberOfFunctions, 6);
  }

  unsigned int InteractionMatrix::getNumberOfFunctions() const
  {
    return this->rows();
  }

  unsigned int InteractionMatrix::rows() const
  {
    assert(data_.rows() == function_names_.size());

    return data_.rows();
  }

  unsigned int InteractionMatrix::columns() const
  {
    return data_.cols();
  }

  TwistDerivative InteractionMatrix::getDerivative(unsigned int row) const
  {
    assert(row<rows());

    // init with semantic information
    TwistDerivative result(reference_frame_, function_names_[row]);

    // copy actual numeric information
    for(unsigned int i=0; i<columns(); i++)
      result(i) = data_(row, i);

    return result;
  }

  void InteractionMatrix::setDerivative(unsigned int row, const TwistDerivative& derivative)
  {
    assert(row < rows());
    assert(derivative.getReferenceFrame().compare(reference_frame_) == 0);
    
    // copy semantic information
    function_names_[row] = derivative.getFunctionName();

    // copy numeric information
    for(unsigned int i=0; i<columns(); i++)
      data_(row, i) = derivative(i);
  }

  double& InteractionMatrix::operator()(unsigned int row, unsigned int column)
  {
    assert(row < rows());
    assert(column < columns());

    return data_(row, column);
  }

  double InteractionMatrix::operator()(unsigned int row, unsigned int column) const
  {
    assert(row < rows());
    assert(column < columns());

    return data_(row, column);
  }

  bool InteractionMatrix::hasDerivative(const std::string& functionName) const
  {
    return (getRowNumber(functionName) < rows());
  }

  unsigned int InteractionMatrix::getRowNumber(const std::string& functionName) const
  {
    for(unsigned int i=0; i<rows(); i++)
    {
      if(functionName.compare(function_names_[i]) == 0)
        return i;
    }
    
    // have not found the function name in our list, so return number of rows as
    // error code
    return rows();
  }

  const std::vector<std::string>& InteractionMatrix::getFunctionNames() const
  {
    return function_names_;
  }

  void InteractionMatrix::changeReferenceFrame(const fccl::Transform& transform)
  {
    assert(reference_frame_.compare(transform.getParentFrame()) == 0);

    // perform the actual calculation 
    data_ = data_*getTransposedTwistTransformationMatrix(transform.getTransform()); 
  
    // update the reference frame
    reference_frame_ = transform.getChildFrame();
  }

} // namespace fccl
