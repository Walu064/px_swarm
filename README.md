# PX-Swarm

Distributed Swarm Control for PX4 SITL (C++ / MAVSDK / simulator backend)

## English

### Overview

PX-Swarm is a distributed swarm control framework for multirotor UAVs running on top of PX4 SITL and a compatible simulator stack.

The simulator backend is not fixed. It can be, for example:

- Isaac Sim with Pegasus
- Gazebo
- another simulator integrated with PX4 SITL

The control layer is implemented in C++ using MAVSDK.

The core runtime model is simple:

- one `px_swarm_node` process controls exactly one UAV,
- each node connects to one PX4 SITL instance over UDP,
- multiple nodes running in parallel form a swarm,
- swarm members exchange state through a lightweight peer-to-peer UDP channel.

For a full architecture walkthrough with runtime diagrams, see [documentation](docs/PX_SWARM_SYSTEM_SCHEMATICS.md). Both files can be viewed side by side.

This means the swarm is distributed by design. There is no single central process responsible for commanding every vehicle. Each node makes its own decisions based on local state, PX4 telemetry, and messages received from the rest of the swarm.

### Roles

Each `px_swarm_node` runs in one of two roles.

### Leader

The leader is the operator-driven vehicle.

Responsibilities:

- receives keyboard input,
- converts keyboard commands into PX4 Offboard body-velocity setpoints,
- flies as the reference vehicle for the formation,
- publishes its current state to the swarm.

In practice, the leader is the anchor for the rest of the formation. Followers use the leader's reported position as the basis for their own target selection.

### Follower

The follower is the autonomous vehicle.

Responsibilities:

- receives swarm state over UDP,
- reconstructs the shared swarm view,
- computes an optimal target position using Particle Swarm Optimization (PSO),
- flies toward the selected target in PX4 Offboard mode.

The current follower controller uses a simple control loop:

- periodically solve for a target position,
- keep that target as the local navigation objective,
- command velocity toward that target using a proportional controller.

### System Architecture

At runtime, the setup looks like this:

```text
+------------------------------------------------------------------+
|                    Simulator Backend + PX4 SITL                  |
|                                                                  |
|   Example backends: Isaac Sim + Pegasus, Gazebo, others          |
|                                                                  |
|   PX4 SITL #0  <--udp:14540-->  px_swarm_node (leader)           |
|   PX4 SITL #1  <--udp:14541-->  px_swarm_node (follower)         |
|   PX4 SITL #2  <--udp:14542-->  px_swarm_node (follower)         |
|   ...                                                            |
|                                                                  |
+------------------------------------------------------------------+
```

Each PX4 instance is independent. Each `px_swarm_node` binds to one MAVSDK UDP endpoint and controls only its assigned vehicle.

Separately, all nodes communicate over a shared swarm bus:

- transport: UDP
- default address: `127.0.0.1:9000`

This second channel is used only for swarm coordination, not for PX4 control.

### Communication Layers

There are two distinct communication layers in the system.

### 1. PX4 <-> `px_swarm_node` (MAVLink via MAVSDK)

This is the flight-control channel.

- transport: UDP
- protocol stack: `px_swarm_node` -> MAVSDK -> MAVLink -> PX4 SITL

Example PX4 ports:

- `14540` -> vehicle 0
- `14541` -> vehicle 1
- `14542` -> vehicle 2

Each process connects to one PX4 instance using:

```text
udp://:<PORT>
```

For example, a node controlling the first vehicle uses port `14540`.

This channel is responsible for:

- connection establishment,
- telemetry reception,
- arming,
- takeoff,
- entering Offboard mode,
- streaming control setpoints,
- landing and disarming.

### 2. `px_swarm_node` <-> `px_swarm_node` (Swarm coordination via UDP)

This is the swarm-state channel.

- transport: UDP
- default endpoint: `127.0.0.1:9000`
- topology: local peer-to-peer broadcast-style message exchange on the same host

This channel is responsible for:

- publishing each vehicle's current pose estimate,
- receiving states from other agents,
- maintaining a shared swarm snapshot,
- enabling follower-side decision making.

Each node continuously:

1. reads its own current pose from PX4 telemetry,
2. publishes that pose to the swarm bus,
3. receives incoming messages from other nodes,
4. updates a local `SwarmState`,
5. executes role-specific control logic.

### Runtime Flow

The executable `px_swarm_node` performs the following sequence:

1. Parse command-line arguments (`role`, vehicle ID, PX4 UDP port, swarm IP/port, controller tuning).
2. Connect to PX4 via MAVSDK.
3. Arm and take off to a default altitude.
4. Send an initial zero-velocity setpoint.
5. Start PX4 Offboard mode.
6. Open swarm UDP sender and receiver.
7. Enter the main control loop.

Inside the main loop:

- the node publishes its own pose to the swarm bus,
- ingests incoming swarm messages,
- updates local swarm state,
- runs either the leader or follower controller,
- sleeps to maintain the configured control rate.

On shutdown, the node:

- stops keyboard input (leader),
- exits Offboard mode,
- lands,
- attempts to disarm cleanly.

### Control Logic

### Leader Control

The leader maps keyboard input directly to body-frame velocity commands.

Supported behavior:

- hold a movement key to move,
- release all keys to hover,
- yaw can be controlled separately,
- `SPACE` commands hover,
- `Ctrl+C` exits the process.

The leader controller is intentionally simple: it acts as the human-driven reference vehicle for the rest of the swarm.

### Follower Control

The follower operates in two layers:

1. target selection,
2. target tracking.

Target selection:

- uses the current swarm state,
- derives a desired formation anchor from the leader,
- runs PSO in a bounded search region,
- chooses a target point that respects spacing and safety constraints.

Target tracking:

- compares the current vehicle position with the selected target,
- computes a velocity command toward the target,
- sends bounded Offboard velocity setpoints to PX4.

This architecture keeps the optimization logic separate from the low-level flight command logic.

### Project Structure

```text
include/
  app/
  control/
  swarm/
  utils/

src/
  app/
  control/
  swarm/
  utils/

configs/
  leader.yaml
  follower.yaml

scripts/
  run_demo.sh
```

Main components:

- `src/app/px_swarm_node_main.cpp`: application entry point and main runtime loop
- `src/app/cli.cpp`: command-line parsing
- `src/control/px4_interface.cpp`: MAVSDK/PX4 integration
- `src/control/leader_controller.cpp`: keyboard-driven leader control
- `src/control/follower_controller.cpp`: follower autonomy logic
- `src/swarm/swarm_bus.cpp`: UDP messaging between nodes
- `src/swarm/swarm_state.cpp`: local shared-state model
- `src/swarm/pso.cpp`: PSO optimizer
- `src/utils/keyboard.cpp`: terminal keyboard input

## Polski

### Opis

PX-Swarm to rozproszony framework sterowania rojem wielowirnikowców działający nad PX4 SITL i zgodnym backendem symulacyjnym.

Backend symulacji nie jest narzucony. Może to być na przykład:

- Isaac Sim z Pegasus
- Gazebo
- inny symulator zintegrowany z PX4 SITL

Warstwa sterowania jest zaimplementowana w C++ z użyciem MAVSDK.

Model działania jest prosty:

- jeden proces `px_swarm_node` steruje dokładnie jednym UAV,
- każdy node łączy się z jedną instancją PX4 SITL przez UDP,
- wiele node'ów uruchomionych równolegle tworzy rój,
- członkowie roju wymieniają stan przez lekki kanał UDP peer-to-peer.

Pełny opis architektury z diagramami znajduje się w [dokumentacji](docs/PX_SWARM_SYSTEM_SCHEMATICS.md). Oba pliki można wygodnie otworzyć obok siebie.

To oznacza, że rój jest rozproszony z założenia. Nie ma jednego centralnego procesu sterującego wszystkimi pojazdami. Każdy node podejmuje decyzje na podstawie własnego stanu, telemetrii PX4 i wiadomości odebranych od reszty roju.

## Requirements

### Simulation stack

- Ubuntu
- PX4-Autopilot built for SITL
- any simulator backend compatible with PX4 SITL and MAVLink UDP

Examples:

- Isaac Sim with Pegasus
- Gazebo

Expected PX4 binary path:

```text
~/PX4-Autopilot/build/px4_sitl_default/bin/px4
```

### C++ toolchain

- CMake `>= 3.16`
- C++17-compatible compiler (`g++` or `clang`)
- standard build tools (`build-essential` on Ubuntu)
- MAVSDK installed system-wide so `find_package(MAVSDK REQUIRED)` works

Optional:

- QGroundControl for monitoring and debugging

## Build

From the project root:

```bash
mkdir -p build
cd build
cmake ..
make -j
```

This produces:

```text
build/px_swarm_node
```

## Running the System

### Manual launch

Start one process per vehicle.

Leader example:

```bash
./build/px_swarm_node --port 14540 --role leader --id 0 --swarm-ip 127.0.0.1 --swarm-port 9000
```

Follower examples:

```bash
./build/px_swarm_node --port 14541 --role follower --id 1 --swarm-ip 127.0.0.1 --swarm-port 9000
./build/px_swarm_node --port 14542 --role follower --id 2 --swarm-ip 127.0.0.1 --swarm-port 9000
```

### Demo launcher

The repository includes a helper script that starts:

- 1 leader
- `N-1` followers

Example:

```bash
./scripts/launch_px_swarm_instance.sh --count 5 --base-port 14540
```

Assumptions:

- PX4 SITL instances are already running,
- PX4 instances are listening on consecutive UDP ports,
- the leader is started in a real terminal (keyboard input is required).

The script:

- launches the leader in the foreground,
- launches followers in the background,
- writes follower logs to `/tmp/px_swarm_follower_<id>.log`,
- stops all processes on `Ctrl+C`.

## Command-Line Parameters

Core parameters:

- `--port` : PX4 UDP port for this vehicle
- `--role` : `leader` or `follower`
- `--id` : vehicle ID inside the swarm
- `--swarm-ip` : swarm bus destination IP
- `--swarm-port` : swarm bus UDP port

Leader tuning:

- `--leader-vxy`
- `--leader-vz`
- `--leader-yaw-rate`

Follower tuning:

- `--follow-update-hz`
- `--control-hz`
- `--safe-radius`
- `--spacing`

These allow you to tune responsiveness, formation spacing, and safety margins without changing code.

## Current Assumptions and Limits

This implementation currently assumes:

- all nodes run on the same machine,
- swarm communication uses localhost by default,
- vehicle `0` is the leader,
- PX4 SITL instances are already launched outside this project,
- followers depend on fresh swarm messages and may degrade if state becomes stale.

This repository focuses on the swarm control node, not on fully automating the simulator stack startup.

## Summary

PX-Swarm provides a practical distributed architecture for testing cooperative multirotor swarm behavior in simulation:

- PX4 SITL handles flight dynamics and autopilot behavior,
- the simulation environment can be provided by Isaac Sim + Pegasus, Gazebo, or another PX4-compatible backend,
- each `px_swarm_node` controls one UAV,
- the leader is operator-driven,
- followers use PSO-based target selection,
- swarm coordination happens over a lightweight UDP state-sharing layer.

The result is a modular foundation for experimenting with formation control, decentralized coordination, and autonomous swarm behaviors in a realistic simulation stack.
