//
// Created by alexweiss on 8/12/19.
//

#ifndef INC_7405M_CODE_SRC_ROBOT_SUBSYSTEMS_SUBSYSTEM_HPP_
#define INC_7405M_CODE_SRC_ROBOT_SUBSYSTEMS_SUBSYSTEM_HPP_

#include "../loops/Looper.hpp"

namespace subsystems {
  class Subsystem {

   public:
    virtual void writeToLog() = 0;
    virtual bool validateConfig() = 0;
    virtual void initializeSystem() = 0;
    virtual bool checkSystem() = 0;
    virtual Subsystem* getInstance() = 0;
    virtual void outputTelemetry() {};
    virtual void stop() = 0;
    virtual void zeroSensors() {}
    virtual void registerEnabledLoops(loops::Looper* enabledLooper) {};

  };
}


#endif //INC_7405M_CODE_SRC_ROBOT_SUBSYSTEMS_SUBSYSTEM_HPP_
