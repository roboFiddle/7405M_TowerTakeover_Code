//
// Created by alexweiss on 8/12/19.
//

#include "Looper.hpp"
#include "../Constants.hpp"
#include <stdexcept>


namespace loops {
    Looper::Looper() : Looper(0, "") {}
    Looper::Looper(int priority_offset) : Looper(priority_offset, "Generic Looper") {}
    Looper::Looper(const char *name) : Looper(0, name) {}
    Looper::Looper(int priority_offset, const char *name) : control_(), runner_(new pros::Task(Looper::loop, (void*)this, TASK_PRIORITY_DEFAULT+priority_offset, TASK_STACK_DEPTH_DEFAULT, name)) {}

    void Looper::add(std::shared_ptr<Loop> loop) {
      control_.take(constants::LooperConstants::mutexDelay);
      if(loop->onStart && loop->onLoop && loop->onDisable) {
        loops_.push_back(loop);
      } else {
        throw std::invalid_argument("ALL METHODS MUST BE INITALIZED");
      }

      control_.give();
    }
    void Looper::enable() {
      control_.take(constants::LooperConstants::mutexDelay);
      for(std::shared_ptr<Loop> loop : loops_) {
        loop->onStart();
      }
      runner_->resume();
      control_.give();
    }
    void Looper::loop(void* param) {
      Looper* instance = static_cast<Looper*>(param);
      while(1) {
        instance->control_.take(constants::LooperConstants::mutexDelay);
        for (std::shared_ptr<Loop> loop : instance->loops_) {
          loop->onLoop();
        }
        instance->control_.give();
        pros::Task::delay(constants::LooperConstants::loopDt);
      }
    }
    void Looper::disable() {
      //control_.take(constants::LooperConstants::mutexDelay);
      /*for(std::shared_ptr<Loop> loop : loops_) {
        loop->onDisable();
      }*/
      //runner_->suspend();
      //control_.give();
    }
}