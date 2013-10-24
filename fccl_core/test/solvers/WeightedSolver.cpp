#include <gtest/gtest.h>

#include <fccl/solvers/WeightedSolver.h>
#include <fccl/utils/Printing.h>

using namespace fccl::kdl;
using namespace fccl::solvers;

class WeightedSolverTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      joint_names.push_back("joint0");
      joint_names.push_back("joint1");
      joint_names.push_back("joint2");
      joint_names.push_back("joint3");
      joint_names.push_back("joint4");
      joint_names.push_back("joint5");
      joint_names.push_back("joint6");

      constraint_names.push_back("above");
      constraint_names.push_back("perpendicular");

      qdot.init(joint_names);
      ydot.init(constraint_names);
      A.init(constraint_names, joint_names);
      Wy.init(constraint_names, constraint_names);
      Wq.init(joint_names, joint_names);
      A_inv_weighted.init(joint_names, constraint_names);

      using Eigen::operator<<;
      qdot.numerics().data.setZero();
      ydot.numerics().data << 1.2, -0.3;
      A.numerics() << 0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7,
                      0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
      Wq.numerics().setIdentity();
      Wy.numerics().setIdentity();
    }

    virtual void TearDown()
    {
    }

    std::vector<std::string> constraint_names, joint_names;
    JntArray qdot, ydot;
    JointMappingMatrix A, Wy, Wq, A_inv_weighted;
};

TEST_F(WeightedSolverTest, Basics)
{
  WeightedSolver solver;
  solver.init(ydot.semantics(), qdot.semantics());
  ASSERT_TRUE(solver.solve(A, ydot, Wq, Wy, qdot));  
  
  ASSERT_TRUE(qdot.semantics().equals(Wq.semantics().row_joints()));
  Eigen::MatrixXd ydot_reconstructed = A.numerics()*qdot.numerics().data;
  ASSERT_EQ(ydot_reconstructed.rows(), 2);
  ASSERT_EQ(ydot_reconstructed.cols(), 1);
  for(std::size_t i=0; i<ydot.size(); i++)
    EXPECT_NEAR(ydot.numerics()(i), ydot_reconstructed(i,0), 0.0001);

  qdot.numerics().data.setZero();
  ASSERT_TRUE(solver.solve(A, ydot, Wq, Wy, qdot, A_inv_weighted));  

  ASSERT_TRUE(qdot.semantics().equals(Wq.semantics().row_joints()));
  ASSERT_TRUE(A_inv_weighted.semantics().row_joints().equals(qdot.semantics()));
  ASSERT_TRUE(A_inv_weighted.semantics().column_joints().equals(ydot.semantics()));
  ydot_reconstructed = A.numerics()*qdot.numerics().data;
  Eigen::MatrixXd qdot_reconstructed = 
      A_inv_weighted.numerics()*ydot.numerics().data;
  ASSERT_EQ(qdot_reconstructed.rows(), 7);
  ASSERT_EQ(qdot_reconstructed.cols(), 1);
  ASSERT_EQ(ydot_reconstructed.rows(), 2);
  ASSERT_EQ(ydot_reconstructed.cols(), 1);
  for(std::size_t i=0; i<ydot.size(); i++)
    EXPECT_NEAR(ydot.numerics()(i), ydot_reconstructed(i,0), 0.0001);
  for(std::size_t i=0; i<qdot.size(); i++)
    EXPECT_NEAR(qdot.numerics()(i), qdot_reconstructed(i,0), 0.0001);
}
