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

  public:
    void start()
    {
      std::cout << "FSMTest::start\n";
    }
    void stop()
    {
      std::cout << "FSMTest::stop\n";
    }
    void init()
    {
      std::cout << "FSMTest::init\n";
    }
};

TEST_F(FSMTest, Basics)
{
  fccl::control::controller_fsm fsm;
  fsm.start();

  EXPECT_STREQ(getStateName(fsm).c_str(), "Uninitialized");
  fsm.process_event(fccl::control::InitEvent(boost::bind(&FSMTest::init, this)));
  EXPECT_STREQ(getStateName(fsm).c_str(), "Stopped");
  fsm.process_event(fccl::control::InitEvent(boost::bind(&FSMTest::init, this)));
  EXPECT_STREQ(getStateName(fsm).c_str(), "Stopped");
  fsm.process_event(fccl::control::StartEvent(boost::bind(&FSMTest::start, this)));
  EXPECT_STREQ(getStateName(fsm).c_str(), "Running");
  fsm.process_event(fccl::control::StopEvent(boost::bind(&FSMTest::stop, this)));
  EXPECT_STREQ(getStateName(fsm).c_str(), "Stopped");
  fsm.process_event(fccl::control::InitEvent(boost::bind(&FSMTest::init, this)));
  EXPECT_STREQ(getStateName(fsm).c_str(), "Stopped");
} 

TEST_F(FSMTest, DeathTestsWrongSignals)
{
  fccl::control::controller_fsm fsm;
  fsm.start();

  ASSERT_STREQ(getStateName(fsm).c_str(), "Uninitialized");
  EXPECT_DEATH(fsm.process_event(fccl::control::StartEvent(boost::bind(&FSMTest::start, this))), ""); 
  ASSERT_STREQ(getStateName(fsm).c_str(), "Uninitialized");
  EXPECT_DEATH(fsm.process_event(fccl::control::StopEvent(boost::bind(&FSMTest::stop, this))), ""); 

  ASSERT_STREQ(getStateName(fsm).c_str(), "Uninitialized");
  fsm.process_event(fccl::control::InitEvent(boost::bind(&FSMTest::init, this)));
  ASSERT_STREQ(getStateName(fsm).c_str(), "Stopped");
  EXPECT_DEATH(fsm.process_event(fccl::control::StopEvent(boost::bind(&FSMTest::stop, this))), ""); 

  ASSERT_STREQ(getStateName(fsm).c_str(), "Stopped");
  fsm.process_event(fccl::control::StartEvent(boost::bind(&FSMTest::start, this)));
  ASSERT_STREQ(getStateName(fsm).c_str(), "Running");
  EXPECT_DEATH(fsm.process_event(fccl::control::StartEvent(boost::bind(&FSMTest::start, this))), ""); 
  ASSERT_STREQ(getStateName(fsm).c_str(), "Running");
  EXPECT_DEATH(fsm.process_event(fccl::control::InitEvent(boost::bind(&FSMTest::init, this))), ""); 
}
