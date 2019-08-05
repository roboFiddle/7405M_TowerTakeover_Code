//
// Created by alexweiss on 8/2/19.
//

#include <cmath>
#include <stdio.h>
#include "TimingUtil.hpp"

namespace trajectory {
  template<class S>
  Trajectory<TimedState<S>> TimingUtil::timeParameterizeTrajectory(
    bool reverse,
    DistanceView<S> distance_view,
    double step_size,
    std::vector<TimingConstraint<S>> constraints,
    double start_velocity,
    double end_velocity,
    double max_velocity,
    double max_abs_acceleration) {

    int num_states = (int) std::ceil(distance_view.last_interpolant() / step_size + 1);
    std::vector<S> states;
    for (int i = 0; i < num_states; ++i) {
      states.push_back(distance_view.sample(MIN(i * step_size, distance_view.last_interpolant())).state());
    }
    return timeParameterizeTrajectory(reverse, states, constraints, start_velocity, end_velocity,
                                      max_velocity, max_abs_acceleration);
  }

  template<class S>
  Trajectory<TimedState<S>> TimingUtil::timeParameterizeTrajectory(
    bool reverse,
    std::vector<S> states,
    std::vector<TimingConstraint<S>> constraints,
    double start_velocity,
    double end_velocity,
    double max_velocity,
    double max_abs_acceleration) {

    std::vector<ConstrainedState<S>> constraint_states;

    // Forward pass. We look at pairs of consecutive states, where the start state has already been velocity
    // parameterized (though we may adjust the velocity downwards during the backwards pass). We wish to find an
    // acceleration that is admissible at both the start and end state, as well as an admissible end velocity. If
    // there is no admissible end velocity or acceleration, we set the end velocity to the state's maximum allowed
    // velocity and will repair the acceleration during the backward pass (by slowing down the predecessor).
    ConstrainedState<S> predecessor;
    predecessor.state = states.get(0);
    predecessor.distance = 0.0;
    predecessor.max_velocity = start_velocity;
    predecessor.min_acceleration = -max_abs_acceleration;
    predecessor.max_acceleration = max_abs_acceleration;

    for (int i = 0; i < states.size(); ++i) {
      // Add the new state.
      constraint_states.add(ConstrainedState<S>());
      ConstrainedState<S> constraint_state = constraint_states.get(i);
      constraint_state.state = states.get(i);
      double ds = constraint_state.state.distance(predecessor.state);
      constraint_state.distance = ds + predecessor.distance;


      // We may need to iterate to find the maximum end velocity and common acceleration, since acceleration
      // limits may be a function of velocity.
      while (true) {
        // Enforce global max velocity and max reachable velocity by global acceleration limit.
        // vf = sqrt(vi^2 + 2*a*d)
        constraint_state.max_velocity = MIN(max_velocity,
                                            std::sqrt(predecessor.max_velocity * predecessor.max_velocity
                                                          + 2.0 * predecessor.max_acceleration * ds));
        if (std::isnan(constraint_state.max_velocity)) {
          // uhh fuck
          printf("you might have an issue - TimingUtil::73");
        }

        // Enforce global max absolute acceleration.
        constraint_state.min_acceleration = -max_abs_acceleration;
        constraint_state.max_acceleration = max_abs_acceleration;

        // At this point, the state is full constructed, but no constraints have been applied aside from
        // predecessor
        // state max accel.

        // Enforce all velocity constraints.
        for (TimingConstraint<S> constraint : constraints) {
          constraint_state.max_velocity = MIN(constraint_state.max_velocity,
                                              constraint.getMaxVelocity(constraint_state.state));
        }
        if (constraint_state.max_velocity < 0.0) {
          // This should never happen if constraints are well-behaved.
          // uhh fuck
          printf("you might have an really big issue - TimingUtil::92");
        }

        // Now enforce all acceleration constraints.
        for (TimingConstraint<S> constraint : constraints) {
          physics::DifferentialDrive::MinMaxAcceleration min_max_accel = constraint.getMinMaxAcceleration(
              constraint_state.state,
              (reverse ? -1.0 : 1.0) * constraint_state.max_velocity);
          if (!min_max_accel.valid()) {
            // This should never happen if constraints are well-behaved.
            printf("you might have an really big issue - TimingUtil::102");
          }
          constraint_state.min_acceleration = MAX(constraint_state.min_acceleration,
                                                  reverse ? -1 * min_max_accel.max_acceleration()
                                                          : min_max_accel.min_acceleration());
          constraint_state.max_acceleration = MIN(constraint_state.max_acceleration,
                                                  reverse ? -1 * min_max_accel.min_acceleration()
                                                          : min_max_accel.max_acceleration());
        }
        if (constraint_state.min_acceleration > constraint_state.max_acceleration) {
          // This should never happen if constraints are well-behaved.
          printf("you might have an really big issue - TimingUtil::111");
        }

        if (ds < EPSILON) {
          break;
        }

        // If the max acceleration for this constraint state is more conservative than what we had applied, we
        // need to reduce the max accel at the predecessor state and try again.
        // TODO: Simply using the new max acceleration is guaranteed to be valid, but may be too conservative.
        // Doing a search would be better.
        double actual_acceleration = (constraint_state.max_velocity * constraint_state.max_velocity
            - predecessor.max_velocity * predecessor.max_velocity) / (2.0 * ds);
        if (constraint_state.max_acceleration < actual_acceleration - EPSILON) {
          predecessor.max_acceleration = constraint_state.max_acceleration;
        } else {
          if (actual_acceleration > predecessor.min_acceleration + EPSILON) {
            predecessor.max_acceleration = actual_acceleration;
          }
          // If actual acceleration is less than predecessor min accel, we will repair during the backward
          // pass.
          break;
        }
      }
      predecessor = constraint_state;
    }

    // Backward pass.
    ConstrainedState<S> successor;
    successor.state = states.get(states.size() - 1);
    successor.distance = constraint_states.get(states.size() - 1).distance;
    successor.max_velocity = end_velocity;
    successor.min_acceleration = -max_abs_acceleration;
    successor.max_acceleration = max_abs_acceleration;
    for (int i = states.size() - 1; i >= 0; --i) {
      ConstrainedState<S> constraint_state = constraint_states.get(i);
      double ds = constraint_state.distance - successor.distance; // will be negative.

      while (true) {
        // Enforce reverse max reachable velocity limit.
        // vf = sqrt(vi^2 + 2*a*d), where vi = successor.
        double new_max_velocity = std::sqrt(successor.max_velocity * successor.max_velocity
                                                      + 2.0 * successor.min_acceleration * ds);
        if (new_max_velocity >= constraint_state.max_velocity) {
          // No new limits to impose.
          break;
        }
        constraint_state.max_velocity = new_max_velocity;
        if (std::isnan(constraint_state.max_velocity)) {
          printf("you might have an really big issue - TimingUtil::162");
        }

        // Now check all acceleration constraints with the lower max velocity.
        for (TimingConstraint<S> constraint : constraints) {
          physics::DifferentialDrive::MinMaxAcceleration min_max_accel = constraint.getMinMaxAcceleration(
              constraint_state.state,
              (reverse ? -1.0 : 1.0) * constraint_state.max_velocity);
          if (!min_max_accel.valid()) {
            printf("you might have an really big issue - TimingUtil::102");
          }
          constraint_state.min_acceleration = MAX(constraint_state.min_acceleration,
                                                       reverse ? -1 * min_max_accel.max_acceleration() : min_max_accel.min_acceleration());
          constraint_state.max_acceleration = MIN(constraint_state.max_acceleration,
                                                       reverse ? -1 * min_max_accel.min_acceleration() : min_max_accel.max_acceleration());
        }
        if (constraint_state.min_acceleration > constraint_state.max_acceleration) {
          printf("you might have an really big issue - TimingUtil::179");
        }

        if (ds > EPSILON) {
          break;
        }
        // If the min acceleration for this constraint state is more conservative than what we have applied, we
        // need to reduce the min accel and try again.
        // TODO: Simply using the new min acceleration is guaranteed to be valid, but may be too conservative.
        // Doing a search would be better.
        double actual_acceleration = (constraint_state.max_velocity * constraint_state.max_velocity
            - successor.max_velocity * successor.max_velocity) / (2.0 * ds);
        if (constraint_state.min_acceleration > actual_acceleration + EPSILON) {
          successor.min_acceleration = constraint_state.min_acceleration;
        } else {
          successor.min_acceleration = actual_acceleration;
          break;
        }
      }
      successor = constraint_state;
    }

    std::vector<TimedState<S>> timed_states;
    double t = 0.0;
    double s = 0.0;
    double v = 0.0;
    for (int i = 0; i < states.size(); ++i) {
      ConstrainedState<S> constrained_state = constraint_states.get(i);
      // Advance t.
      double ds = constrained_state.distance - s;
      double accel = (constrained_state.max_velocity * constrained_state.max_velocity - v * v) / (2.0 * ds);
      double dt = 0.0;
      if (i > 0) {
        timed_states.at(i - 1).set_acceleration(reverse ? -accel : accel);
        if (std::fabs(accel) > EPSILON) {
          dt = (constrained_state.max_velocity - v) / accel;
        } else if (std::fabs(v) > EPSILON) {
          dt = ds / v;
        } else {
          printf("you might have an really big issue - TimingUtil::218");
        }
      }
      t += dt;
      if (std::isnan(t) || std::isinf(t)) {
        printf("you might have an really big issue - TimingUtil::223");
      }

      v = constrained_state.max_velocity;
      s = constrained_state.distance;
      timed_states.push_back(TimedState<S>(constrained_state.state, t, reverse ? -v : v, reverse ? -accel : accel));
    }
    return new Trajectory<TimedState<S>>(timed_states);
  }

}