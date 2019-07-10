//
// Created by alexweiss on 7/9/19.
//

#ifndef INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMING_DIFFERENTIALDRIVEDYNAMICSCONSTRAINT_HPP_
#define INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMING_DIFFERENTIALDRIVEDYNAMICSCONSTRAINT_HPP_

#include "TimingConstraint.hpp"
#include "../../physics/DifferentialDrive.hpp"
#include "../../geometry/interfaces/IPose2d.hpp"
#include "../../geometry/interfaces/ICurvature.hpp"

namespace trajectory {

  template<class S>
  class DifferentialDriveDynamicsConstraint : TimingConstraint<S> {
    static_assert(std::is_base_of<geometry::IPose2d < S > , S > ::value, "S must derive from IPose2d<S>");
    static_assert(std::is_base_of<geometry::ICurvature < S > , S > ::value, "S must derive from ICurvature<S>");

   protected:
    physics::DifferentialDrive* drive_;
    double abs_voltage_limit_;
   public:
    DifferentialDriveDynamicsConstraint(physics::DifferentialDrive* drive, double abs_voltage_limit);
    double getMaxVelocity(S* state);
    physics::DifferentialDrive::MinMaxAcceleration getMinMaxAcceleration(S state, double velocity);


  };
}

#endif //INC_7405M_CODE_SRC_LIB_TRAJECTORY_TIMING_DIFFERENTIALDRIVEDYNAMICSCONSTRAINT_HPP_
