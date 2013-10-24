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
          row_semantics_ = row_semantics;
          column_semantics_ = column_semantics;
        }

        bool solve(const JointMappingMatrix& A, const JntArray& ydot,
            const JointMappingMatrix& Wq, const JointMappingMatrix& Wy,
            JntArray& qdot)
        {
          assert(A.semantics().row_joints().equals(row_semantics_));
          assert(A.semantics().column_joints().equals(column_semantics_));
          assert(ydot.semantics().equals(row_semantics_));
          assert(Wq.semantics().row_joints().equals(row_semantics_));
          assert(Wq.semantics().column_joints().equals(row_semantics_));
          assert(Wy.semantics().row_joints().equals(column_semantics_));
          assert(Wy.semantics().column_joints().equals(column_semantics_));
          assert(qdot.semantics().equals(column_semantics_));

          solver_.solve(A.numerics(), ydot.numerics().data, Wq.numerics(),
              Wy.numerics(), qdot.numerics().data);
        }

    	bool solve(const JointMappingMatrix& A, const JntArray& ydot,
    			const JointMappingMatrix& Wq, const JointMappingMatrix& Wy,
    			JntArray& qdot, JointMappingMatrix& A_inv_weighted) 
        {
          assert(A.semantics().row_joints().equals(row_semantics_));
          assert(A.semantics().column_joints().equals(column_semantics_));
          assert(ydot.semantics().equals(row_semantics_));
          assert(Wq.semantics().row_joints().equals(row_semantics_));
          assert(Wq.semantics().column_joints().equals(row_semantics_));
          assert(Wy.semantics().row_joints().equals(column_semantics_));
          assert(Wy.semantics().column_joints().equals(column_semantics_));
          assert(qdot.semantics().equals(column_semantics_));
          assert(A_inv_weighted.semantics().row_joints().equals(column_semantics_));
          assert(A_inv_weighted.semantics().column_joints().equals(row_semantics_));
  
          solver_.solve(A.numerics(), ydot.numerics().data, Wq.numerics(),
              Wy.numerics(), qdot.numerics().data, A_inv_weighted.numerics());
        }

      private:
        WDLSSolver solver_;
        fccl::semantics::JntArraySemantics& row_semantics_, column_semantics_;
    };
  } // namespace solvers
} // namespace fccl
#endif /* FCCL_SOLVERS_WEIGHTED_SOLVER_H */
