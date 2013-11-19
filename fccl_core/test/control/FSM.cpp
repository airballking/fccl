#include <gtest/gtest.h>
#include <fccl/control/FSM.h>

using namespace fccl::control;

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

    const std::string& getStateName(const ControllerFSM<std::string>& fsm) const
    {
      assert(ControllerFSM<std::string>::nr_regions::value == 1);
      assert(0 <= fsm.current_state()[0]);
      assert(fsm.current_state()[0] < state_names.size());

      return state_names[fsm.current_state()[0]];
    }

    std::vector<std::string> state_names;

  public:
    void start()
    {
    }

    void stop()
    {
    }

    bool init(const std::string& msg)
    {
      return (0 == msg.compare("true"));
    }
};

TEST_F(FSMTest, Basics)
{
  ControllerFSM<std::string> fsm;
  fsm.start();

  EXPECT_STREQ(getStateName(fsm).c_str(), "Uninitialized");
  fsm.process_event(InitEvent<std::string>(boost::bind(&FSMTest::init, this, _1), "true"));
  EXPECT_STREQ(getStateName(fsm).c_str(), "Stopped");
  fsm.process_event(InitEvent<std::string>(boost::bind(&FSMTest::init, this, _1), "true"));
  EXPECT_STREQ(getStateName(fsm).c_str(), "Stopped");
  fsm.process_event(StartEvent(boost::bind(&FSMTest::start, this)));
  EXPECT_STREQ(getStateName(fsm).c_str(), "Running");
  fsm.process_event(StopEvent(boost::bind(&FSMTest::stop, this)));
  EXPECT_STREQ(getStateName(fsm).c_str(), "Stopped");
  fsm.process_event(InitEvent<std::string>(boost::bind(&FSMTest::init, this, _1), "true"));
  EXPECT_STREQ(getStateName(fsm).c_str(), "Stopped");
  fsm.process_event(InitEvent<std::string>(boost::bind(&FSMTest::init, this, _1), "false"));
  EXPECT_STREQ(getStateName(fsm).c_str(), "Uninitialized");
  fsm.process_event(InitEvent<std::string>(boost::bind(&FSMTest::init, this, _1), "true"));
  EXPECT_STREQ(getStateName(fsm).c_str(), "Stopped");
} 

TEST_F(FSMTest, DeathTestsWrongSignals)
{
  ControllerFSM<std::string> fsm;
  fsm.start();

  ASSERT_STREQ(getStateName(fsm).c_str(), "Uninitialized");
  EXPECT_DEATH(fsm.process_event(StartEvent(boost::bind(&FSMTest::start, this))), ""); 
  ASSERT_STREQ(getStateName(fsm).c_str(), "Uninitialized");
  EXPECT_DEATH(fsm.process_event(StopEvent(boost::bind(&FSMTest::stop, this))), ""); 

  ASSERT_STREQ(getStateName(fsm).c_str(), "Uninitialized");
  fsm.process_event(InitEvent<std::string>(boost::bind(&FSMTest::init, this, _1), "true"));
  ASSERT_STREQ(getStateName(fsm).c_str(), "Stopped");
  EXPECT_DEATH(fsm.process_event(StopEvent(boost::bind(&FSMTest::stop, this))), ""); 

  ASSERT_STREQ(getStateName(fsm).c_str(), "Stopped");
  fsm.process_event(StartEvent(boost::bind(&FSMTest::start, this)));
  ASSERT_STREQ(getStateName(fsm).c_str(), "Running");
  EXPECT_DEATH(fsm.process_event(StartEvent(boost::bind(&FSMTest::start, this))), ""); 
  ASSERT_STREQ(getStateName(fsm).c_str(), "Running");
  EXPECT_DEATH(fsm.process_event(InitEvent<std::string>(boost::bind(&FSMTest::init, this, _1), "true")), ""); 
}

TEST_F(FSMTest, DeathTestsMissingHooks)
{
  ControllerFSM<std::string> fsm;
  fsm.start();

  ASSERT_STREQ(getStateName(fsm).c_str(), "Uninitialized");
  EXPECT_DEATH(fsm.process_event(InitEvent<std::string>(NULL, "true")), "");

  ASSERT_STREQ(getStateName(fsm).c_str(), "Uninitialized");
  fsm.process_event(InitEvent<std::string>(boost::bind(&FSMTest::init, this, _1), "true"));
  ASSERT_STREQ(getStateName(fsm).c_str(), "Stopped");
  EXPECT_DEATH(fsm.process_event(StartEvent(NULL)), "");

  ASSERT_STREQ(getStateName(fsm).c_str(), "Stopped");
  fsm.process_event(StartEvent(boost::bind(&FSMTest::start, this)));
  ASSERT_STREQ(getStateName(fsm).c_str(), "Running");
  EXPECT_DEATH(fsm.process_event(StopEvent(NULL)), "");
  fsm.process_event(StopEvent(boost::bind(&FSMTest::stop, this)));
  EXPECT_STREQ(getStateName(fsm).c_str(), "Stopped");
}

TEST_F(FSMTest, StateQueries)
{
  ControllerFSM<std::string> fsm;
  fsm.start();

  EXPECT_TRUE(fsm.isUninitialized());
  EXPECT_FALSE(fsm.isRunning());
  EXPECT_FALSE(fsm.isStopped());

  fsm.process_event(InitEvent<std::string>(boost::bind(&FSMTest::init, this, _1), "true"));
  EXPECT_FALSE(fsm.isUninitialized());
  EXPECT_FALSE(fsm.isRunning());
  EXPECT_TRUE(fsm.isStopped());

  fsm.process_event(InitEvent<std::string>(boost::bind(&FSMTest::init, this, _1), "true"));
  EXPECT_FALSE(fsm.isUninitialized());
  EXPECT_FALSE(fsm.isRunning());
  EXPECT_TRUE(fsm.isStopped());

  fsm.process_event(StartEvent(boost::bind(&FSMTest::start, this)));
  EXPECT_FALSE(fsm.isUninitialized());
  EXPECT_TRUE(fsm.isRunning());
  EXPECT_FALSE(fsm.isStopped());


  fsm.process_event(StopEvent(boost::bind(&FSMTest::stop, this)));
  EXPECT_FALSE(fsm.isUninitialized());
  EXPECT_FALSE(fsm.isRunning());
  EXPECT_TRUE(fsm.isStopped());

  fsm.process_event(InitEvent<std::string>(boost::bind(&FSMTest::init, this, _1), "true"));
  EXPECT_FALSE(fsm.isUninitialized());
  EXPECT_FALSE(fsm.isRunning());
  EXPECT_TRUE(fsm.isStopped());

  fsm.process_event(InitEvent<std::string>(boost::bind(&FSMTest::init, this, _1), "false"));
  EXPECT_TRUE(fsm.isUninitialized());
  EXPECT_FALSE(fsm.isRunning());
  EXPECT_FALSE(fsm.isStopped());

  fsm.process_event(InitEvent<std::string>(boost::bind(&FSMTest::init, this, _1), "true"));
  EXPECT_FALSE(fsm.isUninitialized());
  EXPECT_FALSE(fsm.isRunning());
  EXPECT_TRUE(fsm.isStopped());
} 
