//
// Created by alexweiss on 8/2/19.
//

#include <cmath>
#include "Trajectory.hpp"

namespace trajectory {
  template<class S>
  Trajectory<S>::Trajectory() {
    points_ = new std::vector<TrajectoryPoint<S>>();
  }

  template<class S>
  Trajectory<S>::Trajectory(std::vector<S> states) {
    points_ = new std::vector<TrajectoryPoint<S>>();
    for (int i = 0; i < states.size(); ++i) {
      points_->push_back(TrajectoryPoint<S>(states.at(i), i));
    }
  }

  template<class S>
  bool Trajectory<S>::isEmpty() {
    return length() == 0;
  }

  template<class S>
  int Trajectory<S>::length() {
    return points_->size();
  }

  template<class S>
  TrajectoryPoint<S> Trajectory<S>::getPoint(int index) {
    return points_->at(index);
  }

  template<class S>
  S Trajectory<S>::getState(int index) {
    return points_->at(index).state();
  }

  template<class S>
  S Trajectory<S>::getFirstState() {
    return points_->at(0);
  }

  template<class S>
  S Trajectory<S>::getLastState() {
    return points_->at(length() - 1);
  }

  template<class S>
  TrajectorySamplePoint<S> Trajectory<S>::getInterpolated(double index) {
    if (isEmpty()) {
      return NULL;
    } else if (index <= 0.0) {
      return TrajectorySamplePoint<S>(getPoint(0));
    } else if (index >= length() - 1) {
      return TrajectorySamplePoint<S>(getPoint(length() - 1));
    }
    int i = (int) std::floor(index);
    double frac = index - i;
    if (frac <= EPSILON) {
      return TrajectorySamplePoint<S>(getPoint(i));
    } else if (frac >= 1.0 - EPSILON) {
      return TrajectorySamplePoint<S>(getPoint(i + 1));
    } else {
      return TrajectorySamplePoint<S>(getState(i).interpolate(getState(i + 1), frac), i, i + 1);
    }
  }

  template<class S>
  IndexView<S> Trajectory<S>::getIndexView() {
    return *index_view_;
  }

  template<class S>
  std::string Trajectory<S>::toString() {
    return "Trajectory<>";
  }

  template<class S>
  std::string Trajectory<S>::toCSV() {
    return "Trajectory<>";
  }
}