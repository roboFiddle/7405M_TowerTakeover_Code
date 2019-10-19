//
// Created by alexweiss on 8/14/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_SUBSYSTEMS_INTAKE_HPP_
#define INC_7405M_CODE_SRC_ROBOT_SUBSYSTEMS_INTAKE_HPP_

#include "Subsystem.hpp"
#include "../../lib/meecan_lib.hpp"
#include "main.h"

namespace subsystems {
  class Intake : Subsystem {
    private:
      pros::Motor* left;
      pros::Motor* right;
      units::Number demand;
    public:
      Intake();
      void setOpenLoop(units::Number control);
      void updateOutputs();
      void stop();
      void registerEnabledLoops(loops::Looper* enabledLooper);

      struct IntakeManager : util::Singleton<Intake, IntakeManager> {};
      static IntakeManager instance;
  };
}

#endif
