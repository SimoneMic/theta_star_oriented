# nav2_theta_star_oriented_planner

A NAV2 global planner plugin implementing the Theta\* any-angle path planning algorithm with full pose orientation support.

## Overview

Theta\* is an extension of A\* that produces shorter, smoother paths by allowing movement at any angle — not just along grid edges. The key difference is the **line-of-sight check**: instead of always inheriting the direct parent node, Theta\* checks whether the current node can "see" its grandparent directly, and if so, skips the intermediate node entirely. The result is paths that cut corners naturally rather than zigzagging along the grid.

This package extends the standard Theta\* implementation with:

- **Orientation-aware poses** — each waypoint carries a meaningful heading, not just a position
- **Obstacle escape behavior** — an escape waypoint is injected at planning start when the robot is near an obstacle
- **Dynamic parameter reconfiguration** — all parameters can be changed at runtime without restarting

## Algorithm Details

### Core Theta\*

The planner uses an 8-directional (or optionally 4-directional) grid search. For each expanded node, `resetParent()` performs a Bresenham line-of-sight check to the grandparent. If the grandparent is visible and the resulting path cost is lower, the current node's parent is updated to skip the intermediate node, yielding any-angle movement.

### Cost Model

Each cell's traversal cost combines three weighted components:

| Component | Formula | Parameter |
|-----------|---------|-----------|
| Traversal cost | `w_traversal_cost × (cell_cost / LETHAL_COST)²` | `w_traversal_cost` |
| Euclidean step cost | `w_euc_cost × hypot(Δx, Δy)` | `w_euc_cost` |
| Heuristic | `w_heuristic_cost × hypot(dx_to_goal, dy_to_goal)` | auto-set to `min(w_euc_cost, 1.0)` |

The heuristic weight is automatically clamped to maintain admissibility.

### Orientation Assignment

After path generation, each pose heading is assigned by one of three strategies (evaluated in order):

1. **Average orientation** — used when start and goal are close (`< proximity_threshold`) and their headings differ by less than `orientation_delta`. All poses receive the average of start and goal headings.
2. **Goal orientation** — used when start and goal headings differ significantly. Poses use the goal heading.
3. **Sequential heading** — default. Each pose points toward the next waypoint. If `use_final_approach_orientation` is true, the last pose uses the robot's approach direction instead of the goal heading.

### Obstacle Escape

When `computeEscapePoint()` detects a lethal cell within `escape_distance` meters ahead of the robot (inside a corridor of half-width `escape_lateral_range`), it computes a repulsion vector away from the obstacle centroid and inserts an escape waypoint as the first pose in the plan. This helps controllers avoid getting immediately stuck at planning start.

### Path Interpolation

The raw Theta\* output has variable spacing between waypoints. After the algorithm completes, the planner linearly interpolates between waypoints at costmap-resolution intervals so downstream controllers receive a dense, evenly spaced path.

## Parameters

All parameters support dynamic reconfiguration via the ROS2 parameter API.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `how_many_corners` | int | `8` | Neighbor connectivity: `4` (cardinal only) or `8` (cardinal + diagonal) |
| `allow_unknown` | bool | `true` | Allow planning through unknown (`255`) costmap cells |
| `w_euc_cost` | double | `1.0` | Weight for the Euclidean step cost |
| `w_traversal_cost` | double | `2.0` | Weight for the obstacle traversal cost |
| `use_final_approach_orientation` | bool | `false` | Set the last pose heading to the robot's approach direction |
| `proximity_threshold` | double | `1.0` | Distance (m) threshold for average-orientation mode |
| `orientation_delta` | double | `0.4` | Max heading difference (rad) for average-orientation mode |
| `escape_distance` | double | `0.4` | Radius (m) within which to detect obstacles for escape behavior |
| `escape_lateral_range` | double | `0.1` | Half-width (m) of the forward corridor used in escape detection |

**Tuning guidance:**
- Increase `w_traversal_cost` to push paths further from obstacles.
- Increase `w_euc_cost` to prefer shorter straight-line paths.
- Use `how_many_corners: 4` in narrow corridors to avoid diagonal cuts through walls.

## Integration with NAV2

Register the plugin in your planner server configuration:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_theta_star_oriented_planner::ThetaStarOrientedPlanner"
      how_many_corners: 8
      allow_unknown: true
      w_euc_cost: 1.0
      w_traversal_cost: 2.0
      use_final_approach_orientation: false
      proximity_threshold: 1.0
      orientation_delta: 0.4
      escape_distance: 0.4
      escape_lateral_range: 0.1
```

The plugin class is `nav2_theta_star_oriented_planner::ThetaStarOrientedPlanner` and it implements the `nav2_core::GlobalPlanner` interface.

## Building

```bash
# Build the package
colcon build --packages-select nav2_theta_star_oriented_planner

# Build and run tests
colcon build --packages-select nav2_theta_star_oriented_planner --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select nav2_theta_star_oriented_planner
```

## Dependencies

| Dependency | Role |
|-----------|------|
| `nav2_core` | GlobalPlanner interface |
| `nav2_costmap_2d` | Costmap access and coordinate conversion |
| `nav2_util` | Lifecycle and utility helpers |
| `pluginlib` | Runtime plugin loading |
| `rclcpp` / `rclcpp_lifecycle` | ROS2 node and lifecycle management |
| `tf2_ros` | Transform buffer |
| `geometry_msgs` / `nav_msgs` | Pose and Path message types |

## Package Info

- **Package name:** `nav2_theta_star_oriented_planner`
- **Version:** 1.3.1
- **License:** Apache License, Version 2.0
- **ROS distro:** Jazzy
- **Maintainer:** [SimoneMic](https://github.com/SimoneMic/theta_star_oriented)
