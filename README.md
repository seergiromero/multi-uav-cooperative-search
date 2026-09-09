# Multi-UAV Cooperative Search

A C++ research-oriented simulator for cooperative multi-UAV area search and
target detection.

The first version focuses on a deliberately simplified 2D problem: multiple
agents must cover a bounded area, detect hidden targets, and coordinate their
tasks without repeatedly searching the same regions.

This is a personal robotics project focused on learning multi-agent planning,
task allocation, simulation, and reproducible experimentation.

## Project Status

Early development. The repository currently defines the scope and initial
architecture; the simulator and algorithms are being implemented incrementally.

## Problem Statement

Given a bounded search area containing unknown target locations, a fleet of
UAV agents must:

- Explore the area.
- Detect and localize targets.
- Divide work between agents.
- Minimize redundant coverage.
- Complete the mission efficiently.
- Continue operating when an agent becomes unavailable.

The first research question is:

> How does dynamic task allocation affect search coverage and target detection
> compared with static area partitioning?

## Version 1 Scope

The initial simulator will use:

- A discrete 2D grid.
- Two or three simulated UAV agents.
- Hidden targets generated from reproducible random seeds.
- Discrete-time agent motion.
- Static area partitioning as a baseline.
- Greedy dynamic task allocation as a second baseline.
- Target detection when an agent reaches an observable cell.
- Coverage, detection, overlap, distance, and mission-time metrics.
- Deterministic experiment configuration.

The following are intentionally out of scope for Version 1:

- PX4 or flight dynamics.
- ROS 2 integration.
- 3D simulation.
- Real cameras or deep-learning detectors.
- Physical hardware.
- Rescue or physical interaction with targets.

These may be considered in later versions once the planning and coordination
model has been validated.

## Planned Algorithms

### Static partitioning

The map is divided into fixed regions and each UAV receives one region. This
provides a simple, reproducible baseline.

### Greedy allocation

Pending regions are assigned dynamically according to a cost that may include:

- Distance to the region.
- Region size.
- Current UAV availability.
- Estimated battery cost.
- Expected information gain.

More advanced allocation methods, such as auction-based allocation, may be
added after the baseline experiments are complete.

## Planned Metrics

The simulator will record:

- Percentage of area covered.
- Time to reach 50%, 80%, and 95% coverage.
- Time to first target detection.
- Number of detected targets.
- Target localization error.
- Total distance travelled.
- Coverage overlap between UAVs.
- Number of task reallocations.
- Mission completion time.
- Performance after an agent failure.

Experiments should be repeated with multiple random seeds rather than relying
on a single visually successful run.

## Planned Architecture

```text
Environment
  - Grid and obstacles
  - Hidden targets
  - Explored cells

UAV agents
  - Position and motion
  - Current task
  - Availability
  - Battery model (optional in the first milestone)

Planning
  - Coverage planner
  - Task allocator

Simulation
  - Discrete-time update loop
  - Target detection
  - Failure injection

Evaluation
  - Metrics collection
  - CSV export
  - Experiment aggregation
```

## Planned Repository Structure

```text
.
├── CMakeLists.txt
├── README.md
├── include/
│   └── cooperative_search/
├── src/
├── tests/
├── configs/
├── scripts/
├── experiments/
├── results/
├── docs/
└── notebooks/
```

## Development Roadmap

- [ ] Define the grid, agent, target, and mission models.
- [ ] Implement the single-UAV simulator.
- [ ] Add lawnmower coverage planning.
- [ ] Add static multi-UAV partitioning.
- [ ] Add greedy dynamic allocation.
- [ ] Implement metrics and CSV export.
- [ ] Add unit tests for planning, allocation, and metrics.
- [ ] Add reproducible multi-seed experiments.
- [ ] Add visualization and result plots.
- [ ] Simulate UAV unavailability and task reassignment.
- [ ] Document and compare experimental results.
- [ ] Evaluate a future ROS 2 integration.
- [ ] Evaluate a future PX4/Gazebo integration.

## Research Direction

The initial literature review will focus on:

- Cooperative multi-robot search.
- Multi-UAV area coverage.
- Informative path planning.
- Multi-agent exploration.
- Multi-robot task allocation.
- Search under uncertainty.

The goal is not to reproduce a complete research system. It is to select a
small number of clearly defined algorithms, implement them transparently, and
compare them under controlled conditions.

## Future Extensions

Potential future milestones include:

- Probabilistic target distributions.
- Information-aware search planning.
- Auction-based task allocation.
- Communication latency and packet loss.
- Battery-aware allocation.
- 3D trajectories.
- ROS 2 nodes and namespaces for each UAV.
- PX4 SITL and Gazebo integration.
- Visual target detection.

These extensions will only be added after the 2D simulator produces reliable
and interpretable baseline results.
