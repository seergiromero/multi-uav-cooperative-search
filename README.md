# Multi-UAV Cooperative Search

A C++ research-oriented simulator for cooperative multi-UAV area search, target detection, and decentralized task allocation.

The project studies how multiple UAV agents can coordinate the exploration of a bounded environment, detect hidden targets, reduce redundant coverage, and adapt their task allocation when the situation changes.

This is a personal robotics project focused on multi-agent planning, task allocation, simulation, and reproducible experimentation.

## Project Status

**Early development**

The simulator and algorithms are being implemented incrementally, starting from simple baselines and progressing toward decentralized and information-aware task allocation.

### Current ROS 2 Status

The ROS 2 simulation bringup currently supports three namespaced TurtleBot3
robots in Gazebo. A selected subset can be launched with the `robot_names`
argument, for example:

```bash
ros2 launch search_bringup simulation.launch.py robot_names:=robot_1,robot_3
```

The basic robot interfaces have been validated in simulation:

* Per-robot odometry, LiDAR, IMU, joint states, and velocity topics.
* Unique TF frames using the `robot_1/`, `robot_2/`, and `robot_3/` prefixes.
* Robot motion through the namespaced `cmd_vel` topics.

SLAM, frontier exploration, target detection, and multi-robot coordination are
not complete yet.

## Problem Statement

Given a bounded search area containing unknown target locations, a team of UAV agents must:

* Explore the environment.
* Detect hidden targets.
* Divide search tasks between agents.
* Minimize redundant exploration.
* Complete the mission efficiently.
* Adapt when agents become unavailable.

The main research question is:

> **How do decentralized and information-aware task allocation strategies affect cooperative search efficiency and robustness compared with static and greedy approaches?**

## Version 1 Scope

The initial simulator uses:

* A discrete 2D grid.
* Two or three simulated UAVs.
* Hidden targets generated from reproducible random seeds.
* Discrete-time agent motion.
* Known UAV positions.
* Static area partitioning as a baseline.
* Greedy dynamic task allocation.
* CBBA-based decentralized task allocation.
* Target detection using a simplified sensor model.
* Coverage, detection, overlap, distance, and mission-time metrics.
* Deterministic experiment configuration.

The following are intentionally out of scope for Version 1:

* PX4 and realistic flight dynamics.
* 3D simulation.
* ROS 2 integration.
* Real cameras and deep-learning perception.
* Physical UAV hardware.
* Realistic RF propagation.
* Adversarial or pursuit-evasion scenarios.
* Complex battery modelling.

These may be considered in later versions after the core search and coordination algorithms have been validated.

## Planned Algorithms

### Static Partitioning

The environment is divided into fixed regions and each UAV is assigned one region.

This provides a simple baseline with no dynamic task coordination.

### Greedy Dynamic Allocation

UAVs dynamically select their next search task according to a simple utility based on factors such as:

* Travel cost.
* Coverage gain.
* Local search utility.

This provides a baseline for dynamic task allocation without consensus.

### CBBA

Consensus-Based Bundle Algorithm (CBBA) is used as the main decentralized task allocation framework.

UAVs independently generate bids for tasks and use consensus to resolve conflicts and agree on task ownership.

### Information-Aware CBBA

The CBBA utility will be extended to consider the information gained by visiting a region.

This allows the UAVs to prioritize areas based not only on distance, but also on their expected contribution to the search.

### Redundancy-Aware Allocation

A further extension will penalize tasks that provide information already covered or planned by other UAVs.

The goal is to reduce duplicated sensing and improve effective cooperative coverage.

## Research Foundations

The project is guided by literature in:

* **Cooperative exploration** — Burgard et al.
* **Decentralized task allocation** — Choi et al. / CBBA.
* **Distributed greedy exploration and redundancy reduction** — Corah & Michael.
* **Value-of-information task assignment** — Mu et al.
* **Uncertainty-aware active search** — Banerjee et al.
* **Asynchronous and communication-limited allocation** — Johnson et al., Otte et al.
* **Local replanning and dynamic task reassignment** — Su et al.

Detailed paper summaries and design decisions are documented separately.

## Planned Metrics

The simulator will record:

* Area coverage over time.
* Time to 50%, 80%, and 95% coverage.
* Time to first target detection.
* Time to detect all targets.
* Number of detected targets.
* Total distance travelled.
* Coverage overlap between UAVs.
* Information gain, where applicable.
* Mission completion time.
* Task reallocations.
* Recovery time after an agent failure.

Experiments will be repeated using multiple independent random seeds rather than relying on individual successful runs.

## Planned Architecture

```text
Environment
├── Grid
├── Hidden targets
├── Obstacles
└── Observation / belief state

UAV Agents
├── Position
├── Motion
├── Current tasks
├── Availability
└── Local information

Task Allocation
├── Static partitioning
├── Greedy
├── CBBA
└── Information-aware extensions

Planning
└── Grid-based movement

Simulation
├── Discrete-time loop
├── Sensing
├── Target detection
└── Failure injection

Evaluation
├── Metrics
├── CSV export
├── Multi-seed experiments
└── Result analysis
```

## ROS 2 Cooperative Exploration Architecture

The ROS 2 implementation follows a layered architecture. Each robot first
builds a local representation of the unknown environment and explores it while
mapping. SLAM and exploration run concurrently rather than as separate stages.

```text
Robot i
├── Odometry + LiDAR
├── Local SLAM
│   ├── Local map
│   ├── Local pose graph
│   └── Keyframes / submaps
├── Frontier exploration
│   └── Selects the next unknown frontier
├── Nav2
│   └── Navigates to the selected frontier
└── Target detection

Swarm-SLAM
├── Inter-robot place recognition
├── Inter-robot loop closures
├── Distributed pose-graph optimization
└── Global map and pose consistency

Cooperative exploration
├── Shared robot state
├── Shared detections and explored regions
├── Frontier assignment
├── Redundancy reduction
└── Greedy / CBBA task allocation
```

The intended runtime flow is:

```text
Local SLAM + frontier exploration + Nav2
                    │
                    └── run continuously on each robot
                    │
              Swarm-SLAM in parallel
                    │
       align maps when robots share observations
                    │
       cooperative frontier assignment and planning
```

### Local Frames and Global Frames

Each robot maintains an isolated local TF tree:

```text
robot_i/odom
└── robot_i/base_footprint
    └── robot_i/base_link
        └── robot_i/base_scan
```

Initially, robots may have unknown global poses and independent map frames.
Swarm-SLAM must estimate the relative transform between local maps when robots
observe overlapping areas or exchange compatible keyframes. A completely
unknown deployment with no common observations, communication, or external
positioning cannot determine the relative placement of disconnected maps.

For simulation, random spawn poses are generated by Gazebo. They may be known
to the simulator for ground-truth evaluation while remaining unknown to the
robot autonomy stack.

### Runtime Layers

| Layer | Responsibility |
|---|---|
| Local SLAM | Estimate each robot pose and build its local map |
| Frontier exploration | Select useful unexplored regions |
| Nav2 | Plan and execute collision-free motion |
| Swarm-SLAM | Detect inter-robot overlap and optimize relative poses |
| Cooperative coordinator | Assign frontiers and reduce redundant exploration |
| Target detector | Detect targets and publish frame-aware observations |
| RViz2 | Visualize maps, robots, TF, scans, frontiers, and detections |

### Incremental Implementation Plan

1. Validate the per-robot ROS 2 interfaces and TF tree.
2. Integrate local SLAM for the three robots.
3. Integrate frontier exploration for one robot.
4. Run independent frontier exploration on all three robots.
5. Integrate Swarm-SLAM while the robots are mapping and exploring.
6. Detect and validate inter-robot map alignment.
7. Add cooperative frontier assignment, initially with a greedy strategy.
8. Add target detections and shared target beliefs.
9. Extend coordination with CBBA, information gain, and redundancy penalties.
10. Evaluate communication loss, robot failure, map consistency, and mission metrics.

Swarm-SLAM is the intended collaborative SLAM backend because it is open
source, ROS 2 based, decentralized, and supports inter-robot loop closures.
Its source packages and dependencies must be validated for the selected ROS 2
distribution before integration; the standard `slam_toolbox` package installed
on the development system is not itself the decentralized Swarm-SLAM backend.

## Repository Structure

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

* [ ] Define environment, UAV, target, and mission models.
* [ ] Implement single-UAV simulation.
* [ ] Implement grid-based coverage planning.
* [ ] Add static multi-UAV partitioning.
* [ ] Add greedy dynamic allocation.
* [ ] Implement metrics and data export.
* [ ] Add unit and integration tests.
* [ ] Add reproducible multi-seed experiments.
* [ ] Implement CBBA.
* [ ] Add probabilistic target belief.
* [ ] Add information-aware task allocation.
* [ ] Add redundancy-aware allocation.
* [ ] Add visualization and result plots.
* [ ] Evaluate communication limitations.
* [ ] Simulate UAV failure and task reassignment.
* [ ] Analyze and document results.

## Experimental Direction

The evaluation will compare progressively more capable strategies:

```text
Static Partitioning
        ↓
Greedy Dynamic Allocation
        ↓
CBBA
        ↓
CBBA + Information Gain
        ↓
CBBA + Information Gain + Redundancy
```

Later experiments will investigate:

```text
Communication Loss
        ↓
Agent Failure
        ↓
Task Reallocation / Recovery
```

The goal is not to reproduce a complete research system, but to build a transparent and reproducible simulator in which the contribution of each coordination strategy can be measured independently.

## Future Extensions

Potential future directions include:

* Communication latency and packet loss.
* Asynchronous task allocation.
* UAV failure recovery.
* Probabilistic target distributions.
* Battery-aware allocation.
* 3D trajectories.
* ROS 2 integration.
* PX4 SITL and Gazebo.
* Visual target detection.
* Moving targets.

These extensions will only be considered after the 2D simulator produces reliable and interpretable results.
