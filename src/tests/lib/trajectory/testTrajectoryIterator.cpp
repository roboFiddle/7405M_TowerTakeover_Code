//
// Created by alexweiss on 8/5/19.
//

#include "testTrajectoryIterator.hpp"
#include "../../../lib/trajectory/TrajectoryIterator.hpp"
#include "../../../lib/trajectory/Trajectory.hpp"
#include "../../../lib/trajectory/IndexView.hpp"
#include "../../../lib/geometry/Translation2d.hpp"
#include <vector>
#include <memory>
#include <iostream>

namespace test {

  void testTrajectoryIterator::test() {
    std::vector<geometry::Translation2d> kWaypoints;
    kWaypoints.push_back(geometry::Translation2d(0.0, 0.0));
    kWaypoints.push_back(geometry::Translation2d(24.0, 0.0));
    kWaypoints.push_back(geometry::Translation2d(36.0, 12.0));
    kWaypoints.push_back(geometry::Translation2d(60.0, 12.0));

    trajectory::Trajectory<geometry::Translation2d> traj(kWaypoints);

    std::shared_ptr<trajectory::TrajectoryView<geometry::Translation2d>> indexView = traj.getIndexView();
    trajectory::TrajectoryIterator<geometry::Translation2d> iterator(indexView);
    iterator.setup();

    // Initial conditions.
    assertEquals(0.0, iterator.getProgress(), EPSILON);
    assertEquals(3.0, iterator.getRemainingProgress(), EPSILON);
    assertEquals(kWaypoints.at(0), iterator.getState());
    assertFalse(iterator.isDone());

    // Advance forward.
    assertEquals(kWaypoints.at(0).interpolate(kWaypoints.at(1), 0.5), iterator.preview(0.5).state());
    assertEquals(kWaypoints.at(0).interpolate(kWaypoints.at(1), 0.5), iterator.advance(0.5).state());
    assertEquals(0.5, iterator.getProgress(), EPSILON);
    assertEquals(2.5, iterator.getRemainingProgress(), EPSILON);
    assertFalse(iterator.isDone());

    // Advance backwards.
    assertEquals(kWaypoints.at(0).interpolate(kWaypoints.at(1), 0.25), iterator.preview(-0.25).state());
    assertEquals(kWaypoints.at(0).interpolate(kWaypoints.at(1), 0.25), iterator.advance(-0.25).state());
    assertEquals(0.25, iterator.getProgress(), EPSILON);
    assertEquals(2.75, iterator.getRemainingProgress(), EPSILON);
    assertFalse(iterator.isDone());

    // Advance past end.
    assertEquals(kWaypoints.at(3), iterator.preview(5.0).state());
    assertEquals(kWaypoints.at(3), iterator.advance(5.0).state());
    assertEquals(3.0, iterator.getProgress(), EPSILON);
    assertEquals(0.0, iterator.getRemainingProgress(), EPSILON);
    assertTrue(iterator.isDone());

    // Advance past beginning.
    assertEquals(kWaypoints.at(0), iterator.preview(-5.0).state());
    assertEquals(kWaypoints.at(0), iterator.advance(-5.0).state());
    assertEquals(0.0, iterator.getProgress(), EPSILON);
    assertEquals(3.0, iterator.getRemainingProgress(), EPSILON);
    assertFalse(iterator.isDone());
  }
}