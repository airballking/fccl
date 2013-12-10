#ifndef FCCL_CONTROL_GAINS_H
#define FCCL_CONTROL_GAINS_H

#include <fccl/base/Array.h>
#include <fccl/kdl/JntArray.h>
#include <fccl/semantics/JntArraySemantics.h>
#include <fccl/utils/Equalities.h>
#include <vector>
#include <string>

using namespace fccl::base;
using namespace fccl::kdl;
using namespace fccl::semantics;
using namespace fccl::utils;
using namespace std;

namespace fccl
{
  namespace control
  {
    template <class T, class SemanticsPolicy>
    class PIDGains2 : public SemanticsPolicy
    {
      public:
        const T& p() const { return p_; }
        T& p() { return p_; }
       
        const T& i() const { return i_; }
        T& i() { return i_; }

        const T& d() const { return d_; }
        T& d() { return d_; }

        const T& i_max() const { return i_max_; }
        T& i_max() { return i_max_; }

        const T& i_min() const { return i_min_; }
        T& i_min() { return i_min_; }

        virtual bool equals(const PIDGains2& other) const
        {
          return this->semantics().equals(other.semantics()) &&
              areEqual(this->p(), other.p()) && areEqual(this->i(), other.i()) &&
              areEqual(this->d(), other.d()) && areEqual(this-i_max(), other.i_max()) &&
              areEqual(this->i_min(), other.i_min());
        }

      private:
        T p_, i_, d_, i_max_, i_min_;
    };

    typedef PIDGains2<double, SemanticsBase> StandardPIDGains;
    typedef Array<StandardPIDGains, SemanticsBase> StandardPIDGainArray;

    // TODO(Georg): refactor this into Array<template> + single-PIDGains
    class PIDGains
    {
      public:
        const JntArray& p() const { return p_; }
        JntArray& p() { return p_; }
       
        const JntArray& i() const { return i_; }
        JntArray& i() { return i_; }

        const JntArray& d() const { return d_; }
        JntArray& d() { return d_; }

        const JntArray& i_max() const { return i_max_; }
        JntArray& i_max() { return i_max_; }

        const JntArray& i_min() const { return i_min_; }
        JntArray& i_min() { return i_min_; }

        bool equals(const PIDGains& other) const
        {
          return p().equals(other.p()) &&
            i().equals(other.i()) &&
            d().equals(other.d()) &&
            i_max().equals(other.i_max()) &&
            i_min().equals(other.i_min());
        }

        void init(const JntArraySemantics& semantics)
        {
          p().init(semantics);
          i().init(semantics);
          d().init(semantics);
          i_max().init(semantics);
          i_min().init(semantics);
        }

        void init(const vector<string>& joint_names)
        {
          p().init(joint_names);
          i().init(joint_names);
          d().init(joint_names);
          i_max().init(joint_names);
          i_min().init(joint_names);
        }

        bool areValid() const
        {
          return p().semantics().equals(i().semantics()) &&
              p().semantics().equals(d().semantics()) &&
              p().semantics().equals(i_max().semantics()) &&
              p().semantics().equals(i_min().semantics());
        }

        size_t size() const { return p().size(); }

        void resize(size_t new_size)
        {
          p().resize(new_size);
          i().resize(new_size);
          d().resize(new_size);
          i_max().resize(new_size);
          i_min().resize(new_size);
        }

        void setSemantics(const JntArraySemantics& semantics) 
        {
          assert(p().size() == semantics.size());
          assert(i().size() == semantics.size());
          assert(d().size() == semantics.size());
          assert(i_max().size() == semantics.size());
          assert(i_min().size() == semantics.size());

          p().semantics() = semantics;
          i().semantics() = semantics;
          d().semantics() = semantics;
          i_max().semantics() = semantics;
          i_min().semantics() = semantics;
        }

      private:
        JntArray p_, i_, d_, i_max_, i_min_;
    }; 
  } // namespace control
} // namespace fccl
#endif // FCCL_CONTROL_GAINS_H
