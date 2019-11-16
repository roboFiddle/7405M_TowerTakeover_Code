//
// Created by alexweiss on 8/14/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_SUBSYSTEMS_LIFT_HPP_
#define INC_7405M_CODE_SRC_ROBOT_SUBSYSTEMS_LIFT_HPP_

#include "Subsystem.hpp"
#include "../../lib/meecan_lib.hpp"
#include "main.h"

namespace subsystems {
  class Lift : Subsystem {
    private:
      pros::Motor* motor;
      pros::ADIAnalogIn* pot;
      units::Number demand;
      ControlState state;
      double lastTray;

    public:
      Lift();
      void setOpenLoop(units::Number control);
      void setPosition(units::Number control);
      units::Number get_demand();
      double get_position();
      double getTrayForDemand();
      ControlState getState();
      void updateOutputs();
      void stop();
      void tare();
      void registerEnabledLoops(loops::Looper* enabledLooper);

      struct LiftManager : util::Singleton<Lift, LiftManager> {};
      static LiftManager instance;
  };
}

#endif
