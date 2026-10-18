// Pure Pursuit geometry. See pure_pursuit_core.hpp and Lesson 11.

#include "pure_pursuit/pure_pursuit_core.hpp"
#include "robonav_training_common/angles.hpp"

#include <cmath>
#include <cstddef>
#include <limits>
#include <algorithm>

namespace robonav_training
{

PursuitCommand computePursuitCommand(
  double robot_x, double robot_y, double robot_yaw,
  const std::vector<std::array<double, 2>> & path,
  double lookahead_distance, double desired_linear_speed,
  double max_angular_speed, double goal_tolerance)
{
  if (path.empty()) {
    return PursuitCommand{0.0, 0.0, false};
  }

  const auto & goal = path.back();
  const double distance_to_goal =
    std::hypot(goal[0] - robot_x, goal[1] - robot_y);
  if (distance_to_goal <= goal_tolerance) {
    return PursuitCommand{0.0, 0.0, true};
  }

  std::size_t nearest_index = 0;
  double nearest_distance_squared = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < path.size(); ++index) {
    const double dx = path[index][0] - robot_x;
    const double dy = path[index][1] - robot_y;
    const double distance_squared = dx * dx + dy * dy;
    if (distance_squared < nearest_distance_squared) {
      nearest_distance_squared = distance_squared;
      nearest_index = index;
    }
  }

  std::size_t lookahead_index = path.size() - 1;

  for (std::size_t i = nearest_index; i < path.size(); i++) {
    const double distance =
      std::hypot(path[i][0] - robot_x, path[i][1] - robot_y);
    if (distance >= lookahead_distance) {
      lookahead_index = i;
      break;
    }
  }

  const double dx = path[lookahead_index][0] - robot_x;
  const double dy = path[lookahead_index][1] - robot_y;

  const double x_r =
    std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
  const double y_r =
    -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;

  PursuitCommand command{0.0, 0.0, false};

  if (x_r < 0.0) {
    command.linear = 0.05;
    const double turn_angle = normalizeAngle(std::atan2(y_r, x_r));
    command.angular = (turn_angle > 0.0) ? max_angular_speed : -max_angular_speed;
  } else {
    const double distance = std::hypot(x_r, y_r);
    const double curvature =
      (distance > 1e-6) ? (2.0 * y_r) / (distance * distance) : 0.0;
    
    if (distance_to_goal < 1.5 * lookahead_distance) {
      command.linear = desired_linear_speed * (distance_to_goal / (1.5 * lookahead_distance));
    } else {
      command.linear = desired_linear_speed;
    }

    command.angular = std::clamp(command.linear * curvature, -max_angular_speed, max_angular_speed);
  }
  return command;
}

}  // namespace robonav_training
