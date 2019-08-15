//
// Created by alexweiss on 8/14/19.
//

#include "SeriesAction.hpp"

namespace auton {
  namespace actions {
    SeriesAction::SeriesAction(std::list<Action*> actions) {
      actions_ = actions;
      current_ = NULL;
    }
    bool SeriesAction::isFinished() {
      return actions_.size() == 0 && current_ == NULL;
    }
    void SeriesAction::start() {

    }
    void SeriesAction::update() {
      if(current_ == NULL) {
        if(actions_.size() == 0)
          return;
        current_ = actions_.front();
        actions_.pop_front();
        current_->start();
      }
      current_->update();
      if(current_->isFinished()) {
        current_->done();
        current_ = NULL;
      }
    }
    void SeriesAction::done() {

    }
  }
}