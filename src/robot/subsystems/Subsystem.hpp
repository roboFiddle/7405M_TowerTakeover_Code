//
// Created by alexweiss on 8/12/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_SUBSYSTEMS_SUBSYSTEM_HPP_
#define INC_7405M_CODE_SRC_ROBOT_SUBSYSTEMS_SUBSYSTEM_HPP_

#include "../loops/Looper.hpp"

namespace subsystems {
  class Subsystem {

   public:
    virtual void writeToLog() {};
    virtual void initializeSystem() {};
    virtual bool checkSystem() {return true;};
    virtual void outputTelemetry() {};
    virtual void stop() = 0;
    virtual void zeroSensors() {}
    virtual void registerEnabledLoops(loops::Looper* enabledLooper) = 0;

  };
}


#endif //INC_7405M_CODE_SRC_ROBOT_SUBSYSTEMS_SUBSYSTEM_HPP_
