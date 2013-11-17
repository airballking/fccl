#include <gtest/gtest.h>
#include <fccl/control/FSM.h>

class FSMTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      state_names.push_back("Uninitialized");
      state_names.push_back("Initializing");
      state_names.push_back("Stopped");
      state_names.push_back("Running");
    }

    virtual void TearDown()
    {
    }

    const std::string& getStateName(const fccl::control::controller_fsm& fsm) const
    {
      assert(fccl::control::controller_fsm::nr_regions::value == 1);
      assert(0 <= fsm.current_state()[0]);
      assert(fsm.current_state()[0] < state_names.size());

      return state_names[fsm.current_state()[0]];
    }

    std::vector<std::string> state_names;
};

TEST_F(FSMTest, Basics)
{
  fccl::control::controller_fsm fsm;
  fsm.start();

  EXPECT_STREQ(getStateName(fsm).c_str(), "Uninitialized");
  fsm.process_event(fccl::control::InitEvent());
  EXPECT_STREQ(getStateName(fsm).c_str(), "Stopped");
  fsm.process_event(fccl::control::InitEvent());
  EXPECT_STREQ(getStateName(fsm).c_str(), "Stopped");
  fsm.process_event(fccl::control::StartEvent());
  EXPECT_STREQ(getStateName(fsm).c_str(), "Running");
  fsm.process_event(fccl::control::StopEvent());
  EXPECT_STREQ(getStateName(fsm).c_str(), "Stopped");
  fsm.process_event(fccl::control::InitEvent());
  EXPECT_STREQ(getStateName(fsm).c_str(), "Stopped");
} 

TEST_F(FSMTest, DeathTestsWrongSignals)
{
  fccl::control::controller_fsm fsm;
  fsm.start();

  ASSERT_STREQ(getStateName(fsm).c_str(), "Uninitialized");
  EXPECT_DEATH(fsm.process_event(fccl::control::StartEvent()), ""); 
  ASSERT_STREQ(getStateName(fsm).c_str(), "Uninitialized");
  EXPECT_DEATH(fsm.process_event(fccl::control::StopEvent()), ""); 

  ASSERT_STREQ(getStateName(fsm).c_str(), "Uninitialized");
  fsm.process_event(fccl::control::InitEvent());
  ASSERT_STREQ(getStateName(fsm).c_str(), "Stopped");
  EXPECT_DEATH(fsm.process_event(fccl::control::StopEvent()), ""); 

  ASSERT_STREQ(getStateName(fsm).c_str(), "Stopped");
  fsm.process_event(fccl::control::StartEvent());
  ASSERT_STREQ(getStateName(fsm).c_str(), "Running");
  EXPECT_DEATH(fsm.process_event(fccl::control::StartEvent()), ""); 
  ASSERT_STREQ(getStateName(fsm).c_str(), "Running");
  EXPECT_DEATH(fsm.process_event(fccl::control::InitEvent()), ""); 
}
