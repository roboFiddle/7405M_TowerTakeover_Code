//
// Created by alexweiss on 8/2/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORY_HPP_
#define INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORY_HPP_

#include <type_traits>
#include <list>
#include <string>
#include "../geometry/interfaces/State.hpp"
#include "../utility/Utility.hpp"
#include "TrajectoryPoint.hpp"
#include "TrajectorySamplePoint.hpp"
#include "TrajectoryView.hpp"

namespace trajectory {
  template<class S>
  class Trajectory {
    static_assert(std::is_base_of<geometry::State<S>, S>::value, "S is not derived from State");
    protected:
      std::list<TrajectoryPoint<S>> points_;
      IndexView* index_view_ = new IndexView(this);
    public:
      bool isEmpty();
      int length();
      TrajectoryPoint<S> getPoint(int index);
      S getState(int index);
      S getFirstState();
      S getLastState();
      TrajectorySamplePoint<S> getInterpolated(double index);
      IndexView getIndexView();
      std::string toString();
      std::string toCSV();

    class IndexView : public TrajectoryView<S> {
     public:
      Trajectory* pointer_to_;
      IndexView(Trajectory* x) {
        pointer_to_ = x // TODO: try removing reference
      }
      TrajectorySamplePoint<S> sample(double index) {
        return pointer_to_->getInterpolated(index);
      }

      double last_interpolant() {
        return MAX(0.0, pointer_to_->length() - 1);
      }

      public double first_interpolant() {
        return 0.0;
      }

      public Trajectory<S> trajectory() {
        return *pointer_to_;
      }
    }


  };
}
#endif //INC_7405M_CODE_SRC_LIB_TRAJECTORY_TRAJECTORY_HPP_
