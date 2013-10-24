#ifndef FCCL_SOLVERS_WEIGHTED_SOLVER_H
#define FCCL_SOLVERS_WEIGHTED_SOLVER_H

#include <fccl/solvers/WDLSSolver.h>
#include <fccl/kdl/JntArray.h>
#include <fccl/kdl/JointMappingMatrix.h>

using namespace fccl::kdl;

namespace fccl
{
  namespace solvers
  {
    class SolverWeighted
    { 
      public:
        void init(const fccl::semantics::JntArraySemantics& row_semantics,
            const fccl::semantics::JntArraySemantics& column_semantics)
        {
          solver_.reinitialise(row_semantics.size(), column_semantics.size());
          this->row_semantics_ = row_semantics;
          this->column_semantics_ = column_semantics;
        }

        bool solve(const JointMappingMatrix& A, const JntArray& ydot,
            const JointMappingMatrix& Wq, const JointMappingMatrix& Wy,
            JntArray& qdot)
        {
          assert(inputValid(A, ydot, Wq, Wy, qdot));

          solver_.solve(A.numerics(), ydot.numerics().data, Wq.numerics(),
              Wy.numerics(), qdot.numerics().data);
        }

    	bool solve(const JointMappingMatrix& A, const JntArray& ydot,
    			const JointMappingMatrix& Wq, const JointMappingMatrix& Wy,
    			JntArray& qdot, JointMappingMatrix& A_inv_weighted) 
        {
          assert(inputValid(A, ydot, Wq, Wy, qdot, A_inv_weighted));
 
          solver_.solve(A.numerics(), ydot.numerics().data, Wq.numerics(),
              Wy.numerics(), qdot.numerics().data, A_inv_weighted.numerics());
        }

      private:
        WDLSSolver solver_;
        fccl::semantics::JntArraySemantics row_semantics_, column_semantics_;

        bool inputValid(const JointMappingMatrix& A, const JntArray& ydot,
            const JointMappingMatrix& Wq, const JointMappingMatrix& Wy,
            const JntArray& qdot) const
        {
          return A.semantics().row_joints().equals(row_semantics_) &&
              A.semantics().column_joints().equals(column_semantics_) &&
              ydot.semantics().equals(row_semantics_) &&
              Wq.semantics().row_joints().equals(column_semantics_) &&
              Wq.semantics().column_joints().equals(column_semantics_) &&
              Wy.semantics().row_joints().equals(row_semantics_) &&
              Wy.semantics().column_joints().equals(row_semantics_) &&
              qdot.semantics().equals(column_semantics_);
        }

        bool inputValid(const JointMappingMatrix& A, const JntArray& ydot,
            const JointMappingMatrix& Wq, const JointMappingMatrix& Wy,
            const JntArray& qdot, const JointMappingMatrix& A_inv_weighted) const
        {
          return inputValid(A, ydot, Wq, Wy, qdot) &&
              A_inv_weighted.semantics().row_joints().equals(column_semantics_) &&
              A_inv_weighted.semantics().column_joints().equals(row_semantics_);
        } 
    };
  } // namespace solvers
} // namespace fccl
#endif /* FCCL_SOLVERS_WEIGHTED_SOLVER_H */
