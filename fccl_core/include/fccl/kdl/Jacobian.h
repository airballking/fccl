#ifndef FCCL_KDL_JACOBIAN_H
#define FCCL_KDL_JACOBIAN_H

#include <fccl/kdl/Semantics.h>
#include <fccl/kdl/Twist.h>
#include <fccl/kdl/Transform.h>
#include <kdl/jacobian.hpp>

namespace fccl
{
  namespace kdl
  {
    class Jacobian : public SemanticObject1xN
    {
      public:
        Jacobian();
        Jacobian(const Jacobian& other);
        Jacobian(const SemanticObject1xN& semantics, const KDL::Jacobian& data);
 
        virtual ~Jacobian();
 
        Jacobian& operator=(const Jacobian& other);
 
        const KDL::Jacobian& getData() const;
        void setData(const KDL::Jacobian& jacobian);
 
        virtual void resize(std::size_t rows);
        virtual std::size_t size() const;

        bool isValid() const;
 
        std::size_t rows() const;
        std::size_t columns() const;

        bool rowIndexValid(std::size_t row) const;
        bool columnIndexValid(std::size_t column) const;
 
        double& operator()(unsigned int row, unsigned int column);
        double operator()(unsigned int row, unsigned int column) const;
  
        fccl::kdl::Twist getColumn(unsigned int column) const;
        void setColumn(unsigned int column, const fccl::kdl::Twist& twist);
  
        bool operator==(const Jacobian& other) const;
        bool operator!=(const Jacobian& other) const;
 
        bool numericsEqual(const Jacobian& other) const;
 
        void changeReferenceFrame(const fccl::kdl::Transform& transform);
        bool transformationPossible(const fccl::kdl::Transform& transform) const;
  
        friend std::ostream& operator<<(std::ostream& os, const Jacobian& jacobian);
 
      private:
        KDL::Jacobian jacobian_;
    };
  } // namespace kdl 
} // namespace fccl
#endif // FCCL_KDL_JACOBIAN_H
