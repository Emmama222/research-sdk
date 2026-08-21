# Deferred planner result metrics

The research console currently exports CSV headings only. It intentionally does
not calculate timing, collision, or resource metrics yet.

## Format A — individual planning events

- `input_latency_ms`: elapsed time from network input receipt to planner entry.
- `mapping_time_ms`: time used by the selected planner to generate its map.
- `planning_time_ms`: time used to produce one plan from an existing map.
- `number_of_fails`: failed planning attempts, replans caused by failure, or collisions.

## Format B — completed simulation summary

- `input_latency_ms`: same input-stage measurement as format A.
- `average_planner_execution_time_ms`: mean planner execution time for the run.
- `robot_arrival_time_ms`: scenario start to target arrival.
- `total_plans_made`: number of planner invocations for the robot.
- `number_of_collisions`: confirmed robot/obstacle collisions.
- `resources_used`: resource measurements are undecided. Candidate fields are CPU
  time, peak resident memory, and average process CPU percentage.

## Measurement requirements

Use one monotonic clock for all in-process durations. Define arrival tolerance,
collision attribution, warm-up runs, timeout, sampling frequency, and process
boundaries before collecting comparable results. Rendering and CSV writing must
not be included in planner timings.
