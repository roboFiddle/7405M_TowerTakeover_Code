//
// Created by alexweiss on 8/2/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORY_HPP_
#define INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORY_HPP_

#include <type_traits>
#include <vector>
#include <string>
#include "../geometry/interfaces/State.hpp"
#include "../utility/Utility.hpp"
#include "TrajectoryPoint.hpp"
#include "TrajectorySamplePoint.hpp"
#include "TrajectoryView.hpp"
#include "IndexView.hpp"

namespace trajectory {
  template<class S>
  class Trajectory {
    static_assert(std::is_base_of<geometry::State<S>, S>::value, "S is not derived from State");

    protected:
      std::vector<TrajectoryPoint<S>>* points_;
      IndexView<S>* index_view_ = new IndexView<S>(this);
    public:
      Trajectory();
      Trajectory(std::vector<S> states);
      bool isEmpty();
      int length();
      TrajectoryPoint<S> getPoint(int index);
      S getState(int index);
      S getFirstState();
      S getLastState();
      TrajectorySamplePoint<S> getInterpolated(double index);
      IndexView<S> getIndexView();
      std::string toString();
      std::string toCSV();

  };
}
#endif //INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORY_HPP_
