#ifndef FCCL_CONTROL_FSM_H
#define FCCL_CONTROL_FSM_H

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
    typedef boost::function< void () >StartFunction;
    typedef boost::function< void () >StopFunction;

    // Event declarations and definitions...
    // ... event 'init'
    template<class T>
    struct InitEvent
    {
      typedef boost::function< bool (const T&) >InitFunction;

      InitEvent(const InitFunction& init_function, const T& init_data) :
          init_function_(NULL), init_data_(init_data)
      {
        if(init_function)
          init_function_ = init_function;
      }

      bool operator()() const
      {
        assert(init_function_);
        return init_function_(init_data_);
      }

      InitFunction init_function_;
      T init_data_;
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
    // ...event 'update'
    struct UpdateEvent
    {
      typedef boost::function< void () >UpdateFunction;

      UpdateEvent(const UpdateFunction& update_function) :
          update_function_(NULL)
      {
        if(update_function)
          update_function_ = update_function;
      }

      void operator()() const
      {
        assert(update_function_);
        update_function_();
      }

      UpdateFunction update_function_;
    };
    // ...event 'init_failed'
    struct InitFailedEvent {};
    // ...event 'init_succeeded'
    struct InitSucceededEvent {};
 
    // Front-end definition of the fsm
    template< class T >
    struct controller_fsm_ : 
        public msm::front::state_machine_def< controller_fsm_< T > >
    {
      // State definitions...
      // ... state 'Uninitialized'
      struct Uninitialized : public msm::front::state<> 
      {
        template <class Event,class FSM>
        void on_entry(Event const&,FSM& ) {}

        template <class Event,class FSM>
        void on_exit(Event const&,FSM& ) {}
      };
      // ... state 'Initializing'
      struct Initializing : public msm::front::state<> 
      {
        template <class Event,class FSM>
        void on_entry(Event const&,FSM& ) {}

        template <class Event,class FSM>
        void on_exit(Event const&,FSM& ) {}
      };
      // ... state 'Running'
      struct Running : public msm::front::state<> 
      {
        template <class Event,class FSM>
        void on_entry(Event const&,FSM& ) {}

        template <class Event,class FSM>
        void on_exit(Event const&,FSM& ) {}
      };
      // ... state 'Stopped'
      struct Stopped : public msm::front::state<> 
      {
        template <class Event,class FSM>
        void on_entry(Event const&,FSM& ) {}

        template <class Event,class FSM>
        void on_exit(Event const&,FSM& ) {}
      };

      // Definition state 'Uninitialized' as initial state of FSM
      typedef Uninitialized initial_state;

      // Definitions transition actions...
      // ... transition 'Init'
      struct init_action 
      {
        template <class EVT,class FSM,class SourceState,class TargetState>
        void operator()(EVT const& event,FSM& fsm,SourceState& ,TargetState& )
        {
          // run hook for init-function inside InitEvent, and
          // signal to ourselves whether initializing was successful
          if (event())
          {
            fsm.process_event(InitSucceededEvent());
          }
          else
          {
            fsm.process_event(InitFailedEvent());
          }
        }
      };
      // ... transition 'Init'
      struct start_action 
      {
        template <class EVT,class FSM,class SourceState,class TargetState>
        void operator()(EVT const& event,FSM& fsm,SourceState& ,TargetState& )
        {
          // run hook for start-function inside StartEvent
          event();
        }
      };
      // ... transition 'Stop'
      struct stop_action 
      {
        template <class EVT,class FSM,class SourceState,class TargetState>
        void operator()(EVT const& event,FSM& ,SourceState& ,TargetState& )
        {
          // run hook for stop-function inside StopEvent
          event();
        }
      };
      // ... transition 'Update'
      struct update_action 
      {
        template <class EVT,class FSM,class SourceState,class TargetState>
        void operator()(EVT const& event,FSM& ,SourceState& ,TargetState& )
        {
          // run hook for update-function inside StopEvent
          event();
        }
      };

      // Definition of transition table of FSM
      struct transition_table : mpl::vector<
      //    Start          Event               Next          Action        Guard
      //  +--------------+-------------------+-------------+-------------+-------+
      Row < Uninitialized, InitEvent<T>      , Initializing, init_action >,
      //  +--------------+-------------------+-------------+-------------+-------+
      Row < Initializing , InitFailedEvent   , Uninitialized >,
      Row < Initializing , InitSucceededEvent, Stopped >,
      //  +--------------+-------------------+-------------+-------------+-------+
      Row < Stopped      , StartEvent        , Running     , start_action >,
      Row < Stopped      , InitEvent<T>      , Initializing, init_action >,
      //  +--------------+-------------------+-------------+-------------+-------+
      Row < Running      , StopEvent         , Stopped     , stop_action >, 
      Row < Running      , UpdateEvent       , Running     , update_action > 
      //  +--------------+-------------------+-------------+-------------+-------+

      > {};
    }; // controller_fsm_

    // Choosing a back-end for the FSM
    template< class T >
    struct ControllerFSM : msm::back::state_machine< controller_fsm_< T > > 
    {
      enum StateTypes
      {
        UNINITIALIZED = 0,
        
        INITIALIZING = 1,
        
        STOPPED = 2,
        
        RUNNING = 3,
        
        STATE_COUNT
      };

      bool isRunning() const
      {
        return RUNNING == currentState();
      }

      bool isStopped() const
      {
        return STOPPED == currentState();
      }

      bool isUninitialized() const
      {
        return UNINITIALIZED == currentState();
      }

      StateTypes currentState() const
      {
        assert(ControllerFSM<T>::nr_regions::value == 1);
        assert(0 <= this->current_state()[0]);
        assert(this->current_state()[0] < STATE_COUNT);

        return (StateTypes) this->current_state()[0];
      }
    };
  } // namespace control
} // namespace fccl
#endif // FCCL_CONTROL_FSM_H
