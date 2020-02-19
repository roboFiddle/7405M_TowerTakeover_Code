#include "ProgrammingSkillsMode.hpp"
#include "AutoStacking.hpp"
#include "../actions/DriveTrajectory.hpp"
#include "../actions/WaitAction.hpp"
#include "../actions/OpenLoopDriveAction.hpp"
#include "../actions/OpenLoopIntakeAction.hpp"
#include "../actions/ParallelAction.hpp"
#include "../actions/SeriesAction.hpp"
#include "../actions/TrayEnableStackAction.hpp"
#include "../../paths/TrajectorySet.hpp"
#include "../actions/TrayPosition.hpp"
#include "../actions/OpenLoopLiftAction.hpp"
#include "../actions/OpenLoopTrayAction.hpp"
#include "../actions/DriveTurnAction.hpp"
#include "../actions/DriveTurnWheelAction.hpp"
#include "../actions/ResetLiftTrayPosition.hpp"
#include "../actions/LiftPosition.hpp"
#include "../../Constants.hpp"

namespace auton{
  void AutoStacker::routine() {
      //stacking the cubes
      runAction(new actions::OpenLoopIntakeAction(-40, 1));
      std::list<actions::Action*> score;
      score.push_back(new actions::TrayEnableStackAction(1));
      //score.push_back(new actions::OpenLoopIntakeAction(-40, 0));
      runAction(new actions::ParallelAction(score));

      //pulling back after stacking
      std::list<actions::Action*> pullBackFromStack;
      pullBackFromStack.push_back(new actions::OpenLoopIntakeAction(-125, 0));
      pullBackFromStack.push_back(new actions::DriveTrajectory(path_planning::TrajectorySet::instance->get("stackPullBack").get(false)));
      runAction(new actions::ParallelAction(pullBackFromStack));
      runAction(new actions::WaitAction(1.0));
  }
}
