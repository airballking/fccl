#ifndef FCCL_CONTROL_GEOMETRY_H
#define FCCL_CONTROL_GEOMETRY_H

#include <fccl_control/String.h>
#include <kdl/frames.hpp>
#include <vector>
#include <Eigen/Core>

namespace fccl
{
  class Transform
  {
    public:
      Transform();
      Transform(const std::string& parent_frame, const std::string& child_frame,
          const KDL::Frame transform); 
      Transform(const Transform& other);

      virtual ~Transform();

      const std::string& getParentFrame() const;
      void setParentFrame(const std::string& parent_frame);

      const std::string& getChildFrame() const;
      void setChildFrame(const std::string child_frame);

      const KDL::Frame& getTransform() const;
      void setTransform(const KDL::Frame& transform);

      void invert();

      void operator*=(const Transform& other);

    private:
      // frame w.r.t. kinematic objects are defined before transformation
      std::string parent_frame_;

      // frame w.r.t. kinematic objects are defined after transformation
      std::string child_frame_;

      // actual numeric representation of transform
      KDL::Frame transform_;

      // temporary variables
      std::string tmp_string_;
  };

  class Vector
  {
    public:
      Vector();
      Vector(const KDL::Vector& vector, const std::string& frame_name);
      
      virtual ~Vector();

      const std::string& getFrameName() const;
      void setFrameName(const std::string& frame_name);

      const KDL::Vector& getVector() const;
      void setVector(const KDL::Vector& vector);
 
      bool isTransformApplicable(const fccl::Transform& transform) const;
      void changeReferenceFrame(const fccl::Transform& transform);
 
      bool operator==(const Vector &other) const;
      bool operator!=(const Vector &other) const; 

    private:
      // frame w.r.t. the vector is defined
      std::string frame_name_;

      // actual numeric representation of vector
      KDL::Vector vector_;
  };

  class TwistDerivative
  {
    public:
      TwistDerivative();
      TwistDerivative(const std::string& referenceFrame, const std::string&
          functionName);

      const std::string& getReferenceFrame() const;
      void setReferenceFrame(const std::string& referenceFrame);

      const std::string& getFunctionName() const;
      void setFunctionName(const std::string& functionName);

      void changeReferenceFrame(const fccl::Transform& transform);

      double& operator()(int index);
      double operator()(int index) const;

      double& operator[](int index);
      double operator[](int index) const;
      
      void setZero();

    private:
      // reference frame w.r.t. the twist is defined
      std::string reference_frame_;

      // name of function which differentiated w.r.t. a twist
      std::string function_name_;

      // numeric representation of the derivative 
      Eigen::Matrix< double, 1, 6 > data;

  };

  class InteractionMatrix
  {
    public:
      InteractionMatrix();
      InteractionMatrix(unsigned int numberOfFunctions, const std::string&
          referenceFrame);      

      void resize(unsigned int numberOfFunctions);
      unsigned int getNumberOfFunctions() const;

      unsigned int rows() const;
      unsigned int columns() const;

      TwistDerivative getDerivative(unsigned int row) const;
      void setDerivative(unsigned int row, const TwistDerivative& derivative);

      double& operator()(unsigned int row, unsigned int column);
      double operator()(unsigned int row, unsigned int column) const; 

      bool hasDerivative(const std::string& functionName) const;
      unsigned int getRowNumber(const std::string& functionName) const;
      const std::vector<std::string>& getFunctionNames() const;

      void changeReferenceFrame(const fccl::Transform& transform);

    private:
      // reference frame w.r.t. the twists are defined
      std::string reference_frame_;

      // name of the functions for which we're holding the derivatives
      std::vector<std::string> function_names_;

      // actual numeric representation of interaction matrix
      Eigen::Matrix< double, Eigen::Dynamic, 6 > data_;
  };

  // auxiliary function providing transposed twist transformation matrix
  Eigen::Matrix< double, 6, 6> getTransposedTwistTransformationMatrix(const KDL::Frame& frame);
 
} // namespace fccl
#endif // FCCL_CONTROL_GEOMETRY_H
