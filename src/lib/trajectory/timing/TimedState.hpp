//
// Created by alexweiss on 7/9/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMING_TIMEDSTATE_HPP_
#define INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMING_TIMEDSTATE_HPP_

#include "../../geometry/interfaces/State.hpp"
#include <type_traits>

namespace trajectory {
  template<class S>
  class TimedState : public geometry::State<TimedState<S>> {
    static_assert(std::is_base_of<geometry::State<S>, S>::value, "S must derive from State");

   protected:
    S state_;
    double t_; // time at state_
    double velocity_;
    double acceleration_;

   public:
    TimedState(S state);
    TimedState(S state, double t, double velocity, double acceleration);
    S state();
    void set_t(double t);
    double t();
    void set_velocity(double v);
    double velocity();
    void set_acceleration(double a);
    double acceleration();
    std::string toCSV();
    std::string toString();
    TimedState<S> interpolate(TimedState<S> other, double x);
    double distance(TimedState<S> other);
    bool operator==(TimedState other);

  };
}

#endif //INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMING_TIMEDSTATE_HPP_
