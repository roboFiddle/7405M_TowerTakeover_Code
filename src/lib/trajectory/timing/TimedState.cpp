//
// Created by alexweiss on 7/9/19.
//

#include "TimedState.hpp"
#include "../../utility/Utility.hpp"
#include <string>
#include <sstream>

namespace trajectory {
  template <class S> TimedState<S>::TimedState(S state) {
    state_ = state;
    t_ = 0;
    velocity_ = 0;
    acceleration_ = 0;
  }
  template <class S> TimedState<S>::TimedState(S state, double t, double velocity, double acceleration) {
    state_ = state;
    t = t;
    velocity_ = velocity;
    acceleration_ = acceleration;
  }
  template <class S> S TimedState<S>::state() {
    return state_;
  }
  template <class S> void TimedState<S>::set_t(double t) {
    t_ = t;
  }
  template <class S> double TimedState<S>::t() {
    return t_;
  }
  template <class S> void TimedState<S>::set_velocity(double v) {
    velocity_ = v;
  }
  template <class S> double TimedState<S>::velocity() {
    return velocity;
  }
  template <class S> void TimedState<S>::set_acceleration(double a) {
    acceleration_ = a;
  }
  template <class S> double TimedState<S>::acceleration() {
    return acceleration_;
  }
  template <class S> std::string TimedState<S>::toCSV() {
    std::ostringstream stringStream;
    stringStream << "TimedState, {" << state_.toCSV() << "}, " << std::to_string(t_) << ", ";
    stringStream << std::to_string(velocity_) << ", " << std::to_string(acceleration_);
    return stringStream.str();
  }
  template <class S> std::string TimedState<S>::toString() {
    return toCSV();
  }
  template <class S> TimedState<S> TimedState<S>::interpolate(TimedState<S> other, double x) {
    double new_t = INTERPOLATE(t(), other.t(), x);
    double delta_t = new_t - t();
    if (delta_t < 0.0)
      return other.interpolate(this, 1.0 - x);

    bool reversing = velocity() < 0.0 || (FEQUALS(velocity(), 0.0) && acceleration() < 0.0);
    double new_v = velocity() + acceleration() * delta_t;
    double new_s = (reversing ? -1.0 : 1.0) * (velocity() * delta_t + .5 * acceleration() * delta_t * delta_t);

    return new TimedState<S>(state().interpolate(other.state(), new_s / state().distance(other.state())),
                             new_t,
                             new_v,
                             acceleration());
  }
  template <class S> double TimedState<S>::distance(TimedState<S> other) {
    return state_.distance(other.state());
  }
  template <class S> bool TimedState<S>::operator==(TimedState other) {
    return state_ == other.state();
  }
}