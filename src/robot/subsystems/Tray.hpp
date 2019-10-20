//
// Created by alexweiss on 8/14/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_SUBSYSTEMS_TRAY_HPP_
#define INC_7405M_CODE_SRC_ROBOT_SUBSYSTEMS_TRAY_HPP_

#include "Subsystem.hpp"
#include "../../lib/meecan_lib.hpp"
#include "main.h"

namespace subsystems {
  class Tray : Subsystem {
    private:
      pros::Motor* motor;
      units::Number demand;
      ControlState current_state;
    public:
      Tray();
      ~Tray();
      void setOpenLoop(units::Number control);
      void setPosition(units::Number control);
      units::Number get_demand();
      double get_position();
      ControlState getState();
      void updateOutputs();
      void stop();
      void registerEnabledLoops(loops::Looper* enabledLooper);

      struct TrayManager : util::Singleton<Tray, TrayManager> {};
      static TrayManager instance;
  };
}

#endif