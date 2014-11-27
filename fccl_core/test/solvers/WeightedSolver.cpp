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

TEST_F(WeightedSolverTest, CartesianSolving)
{
  // variables
  std::vector<std::string> constraint_names;
  std::string tool_frame = "tool_frame";
  std::string world_frame = "world_frame";
 
  Twist twist;
  JntArray ydot;
  InteractionMatrix H;
  JointMappingMatrix Wt, Wy;

  WeightedSolver solver;
 
  // init
  constraint_names.push_back("above");
  constraint_names.push_back("perpendicular");

  ydot.init(constraint_names); 
  twist.semantics().init(world_frame, tool_frame);
  H.init(constraint_names, world_frame, tool_frame);

  Wt.resize(6, 6);
  Wy.init(constraint_names, constraint_names);

  using Eigen::operator<<;
  KDL::SetToZero(twist.numerics());
  ydot.numerics().data << 1.2, -0.3;
  H.numerics() << 0.1, -0.2, 0.3, -0.4, 0.5, -0.6,
                  0.0, 0.1, 0.2, 0.3, 0.4, 0.5;
  Wy.numerics().setIdentity();
  Wt.numerics().setIdentity();

  solver.init(ydot.semantics(), twist.semantics());  

  // running calculation
  solver.solve(H, ydot, Wt, Wy, twist);

  // checking output semantics
  ASSERT_STREQ(world_frame.c_str(), twist.semantics().reference().getName().c_str());
  ASSERT_STREQ(tool_frame.c_str(), twist.semantics().target().getName().c_str());

  // running reconstruction
  KDL::JntArray twist_array(6);
  TwistToJntArray(twist.numerics(), twist_array);
  Eigen::MatrixXd ydot_reconstructed = H.numerics()*twist_array.data;
 
  //checking reconstruction
  EXPECT_EQ(ydot_reconstructed.rows(), ydot.numerics().rows());
  EXPECT_EQ(ydot_reconstructed.cols(), 1);

  for(unsigned int i=0; i<ydot_reconstructed.rows(); ++i)
  {
    EXPECT_NEAR(ydot.numerics()(i), ydot_reconstructed(i,0), 0.001);
  }
}
