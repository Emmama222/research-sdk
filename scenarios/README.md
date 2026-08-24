# Planner scenarios

The research console stores named scenario files in this folder as JSON.

Coordinates and obstacle radii are stored in millimetres. Robot orientations
are radians. Obstacle velocity is included in the schema for future dynamic
obstacle experiments, but the initial UI treats obstacles as static.

Each obstacle may include a `planner_keys` list containing planner class import
paths. Such an obstacle is visible only to those algorithms. If the list is
missing or empty, the obstacle is shared by every algorithm for compatibility
with existing scenario files. In the UI, choose an active planner before using
the `add_obstacle` course tool.

Scenario files are reproducible experiment inputs and should be committed when
they form part of a reported result.
