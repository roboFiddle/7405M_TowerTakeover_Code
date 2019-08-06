//
// Created by alexweiss on 8/6/19.
//

#include "testTrajectory.hpp"
#include "../../../lib/trajectory/TrajectoryIterator.hpp"
#include "../../../lib/trajectory/Trajectory.hpp"
#include "../../../lib/trajectory/IndexView.hpp"
#include "../../../lib/geometry/Translation2d.hpp"
#include <vector>
#include <memory>
#include <stdio.h>

namespace test {
  void testTrajectory::test() {
    std::vector<geometry::Translation2d> kWaypoints;
    kWaypoints.push_back(geometry::Translation2d(0.0, 0.0));
    kWaypoints.push_back(geometry::Translation2d(24.0, 0.0));
    kWaypoints.push_back(geometry::Translation2d(36.0, 12.0));
    kWaypoints.push_back(geometry::Translation2d(60.0, 12.0));

    trajectory::Trajectory<geometry::Translation2d> traj;
    std::shared_ptr<trajectory::IndexView<geometry::Translation2d>> idxView = traj.createIndexView();

    assertTrue(traj.isEmpty());
    assertEquals(0.0, idxView->first_interpolant(), EPSILON);
    assertEquals(0.0, idxView->last_interpolant(), EPSILON);
    assertEquals(0, traj.length());

    // Set states at construction time.

    traj = trajectory::Trajectory<geometry::Translation2d>(kWaypoints);
    assertFalse(traj.isEmpty());
    assertEquals(0.0, idxView->first_interpolant(), EPSILON);
    assertEquals(3.0, idxView->last_interpolant(), EPSILON);
    assertEquals(4, traj.length());



  }
}