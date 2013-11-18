#ifndef FCCL_CONTROL_CONSTRAINT_CONTROLLER_H
#define FCCL_CONTROL_CONSTRAINT_CONTROLLER_H

#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <iostream>

// convenience abbrevations
namespace msm = boost::msm;
namespace mpl = boost::mpl;
using namespace boost::msm::front;

namespace fccl
{
  namespace control
  {
    // Function declarations for callback-hooks passed around in events
    typedef boost::function< void () >InitFunction;
    typedef boost::function< void () >StartFunction;
    typedef boost::function< void () >StopFunction;

    // Event declarations and definitions...
    // ... event 'init'
    struct InitEvent
    {
      InitEvent(const InitFunction& init_function) :
          init_function_(NULL)
      {
        if(init_function)
          init_function_ = init_function;
      }

      void operator()() const
      {
        assert(init_function_);
        init_function_();
      }

      InitFunction init_function_;
    };
    // ...event 'start'
    struct StartEvent
    {
      StartEvent(const StartFunction& start_function) :
          start_function_(NULL)
      {
        if(start_function)
          start_function_ = start_function;
      }

      void operator()() const
      {
        assert(start_function_);
        start_function_();
      }

      StartFunction start_function_;
    };
    // ...event 'stop'
    struct StopEvent
    {
      StopEvent(const StopFunction& stop_function) :
          stop_function_(NULL)
      {
        if(stop_function)
          stop_function_ = stop_function;
      }

      void operator()() const
      {
        assert(stop_function_);
        stop_function_();
      }

      StopFunction stop_function_;
    };
 
    // Front-end definition of the fsm
    struct controller_fsm_ : public msm::front::state_machine_def<controller_fsm_>
    {
      // State definitions...
      // ... state 'Uninitialized'
      struct Uninitialized : public msm::front::state<> 
      {
        template <class Event,class FSM>
        void on_entry(Event const&,FSM& )
          {std::cout << "entering: Uninitialized" << std::endl;}

        template <class Event,class FSM>
        void on_exit(Event const&,FSM& )
          {std::cout << "leaving: Uninitialized" << std::endl;}
      };
      // ... state 'Initializing'
      struct Initializing : public msm::front::state<> 
      {
        template <class Event,class FSM>
        void on_entry(Event const&,FSM& )
          {std::cout << "entering: Initializing" << std::endl;}

        template <class Event,class FSM>
        void on_exit(Event const&,FSM& )
          {std::cout << "leaving: Initializing" << std::endl;}
      };
      // ... state 'Running'
      struct Running : public msm::front::state<> 
      {
        template <class Event,class FSM>
        void on_entry(Event const&,FSM& )
          {std::cout << "entering: Running" << std::endl;}

        template <class Event,class FSM>
        void on_exit(Event const&,FSM& )
          {std::cout << "leaving: Running" << std::endl;}
      };
      // ... state 'Stopped'
      struct Stopped : public msm::front::state<> 
      {
        template <class Event,class FSM>
        void on_entry(Event const&,FSM& )
          {std::cout << "entering: Stopped" << std::endl;}

        template <class Event,class FSM>
        void on_exit(Event const&,FSM& )
          {std::cout << "leaving: Stopped" << std::endl;}
      };

      // Definition state 'Uninitialized' as initial state of FSM
      typedef Uninitialized initial_state;

      // Definitions transition actions...
      // ... transition 'Init'
      struct init_action 
      {
        template <class EVT,class FSM,class SourceState,class TargetState>
        void operator()(EVT const& event,FSM& ,SourceState& ,TargetState& )
        {
          event();
        }
      };
      // ... transition 'Init'
      struct start_action 
      {
        template <class EVT,class FSM,class SourceState,class TargetState>
        void operator()(EVT const& event,FSM& fsm,SourceState& ,TargetState& )
        {
          event();
        }
      };
      // ... transition 'Init'
      struct stop_action 
      {
        template <class EVT,class FSM,class SourceState,class TargetState>
        void operator()(EVT const& event,FSM& ,SourceState& ,TargetState& )
        {
          event();
        }
      };

      // Definition guard conditions...
      // ... guard checking whether "InitAction" succeeded
      struct init_succeeded 
      {
        template <class EVT,class FSM,class SourceState,class TargetState>
        bool operator()(EVT const& evt,FSM& fsm,SourceState& src,TargetState& tgt)
        {
            std::cout << "init succeeded" << std::endl;
            return true;
        }
      };
      struct init_failed 
      {
          template <class EVT,class FSM,class SourceState,class TargetState>
          bool operator()(EVT const& evt,FSM& fsm,SourceState& src,TargetState& tgt)
          {
              std::cout << "init did not fail" << std::endl;
              return false;
          }
      };

      // Definition of transition table of FSM
      struct transition_table : mpl::vector<
      //    Start     Event         Next      Action               Guard
      //  +---------+-------------+---------+---------------------+----------------------+
      Row < Uninitialized , InitEvent , Initializing, init_action >,
      //  +---------+-------------+---------+---------------------+----------------------+
      Row < Initializing , none , Uninitialized , none, init_failed >,
      Row < Initializing , none , Stopped , none, init_succeeded >,
      //  +---------+-------------+---------+---------------------+----------------------+
      Row < Stopped , StartEvent, Running , start_action >,
      Row < Stopped , InitEvent, Initializing , init_action >,
      //  +---------+-------------+---------+---------------------+----------------------+
      Row < Running, StopEvent, Stopped, stop_action > 
      //  +---------+-------------+---------+---------------------+----------------------+
      > {};
    }; // controller_fsm_

    // Choosing a back-end for the FSM
    typedef msm::back::state_machine<controller_fsm_> controller_fsm;
  } // namespace control
} // namespace fccl
#endif // FCCL_CONTROL_CONSTRAINT_CONTROLLER_H
