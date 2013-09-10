#include <fccl/kdl/Jacobian.h>
#include <fccl/kdl/Transform.h>
#include <fccl/kdl/Semantics.h>
#include <fccl/kdl/JntArray.h>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <boost/shared_ptr.hpp>
#include <string>
#include <map>

typedef boost::shared_ptr<KDL::ChainFkSolverPos_recursive> FkSolverPosPtr;
typedef boost::shared_ptr<KDL::ChainJntToJacSolver> JacobianSolverPtr;

class KinematicChain : public SemanticObject1x1
{
  public:
    KinematicChain();
    KinematicChain(const KinematicChain& other);
    KinematicChain(const fccl::kdl::Chain& chain); 

    virtual ~KinematicChain();

    KinematicChain& operator=(const KinematicChain& other); 

    size_t getReferenceID() const;
    const std::string& getReferenceName() const;

    size_t getTargetID() const;
    const std::string& getTargetName() const;

    const kdl::fccl::Chain& getChain() const;
    void setChain(const fccl::kdl::Chain& chain);

    void calculateJacobian(const fccl::kdl::JntArray& joint_state, 
        fccl::kdl::Jacobian& jacobian);
    const fccl::kdl::Jacobian& calculateJacobian(const fccl::kdl::JntArray&
        joint_state);

    void calculateForwardKinematics(const fccl::kdl::JntArray& joint_state, 
        fccl::kdl::Transform& transform);
    const fccl::kdl::Transform& calculateForwardKinematics(
        const fccl::kdl::JntArray& joint_state);

  private:
    fccl::kdl::Chain chain_;
    
    FkSolverPtr jnt_to_pose_solver_;
    
    JacobianSolverPtr jnt_to_jac_solver_;

    fccl::kdl::Jacobian jacobian_;

    fccl::kdl::Transform transform_;
};

typedef boost::shared_ptr<KinematicChain> KinematicChainPtr;

class Robot
{
  public:
    Robot();
    Robot(const Robot& other);
    Robot(const KDL::Tree& tree);

    virtual ~Robot();

    const KDL::Tree& getTree() const;
    void setTree(const KDL::Tree& tree);

    void addKinematicChain(const fccl::kdl::SemanticObject1x1& chain_semantics);
    void removeKinematicChain(const fccl::kdl::SemanticObject1x1& chain_semantics);

    KinematicChainPtr getKinematicChain(const fccl::kdl::SemanticObject1x1& chain_semantics) const;

  private:
    KDL::Tree kinematic_tree_;

    std::map<fccl::kdl::SemanticObject1x1, KinematicChainPtr> chain_map_;
};
