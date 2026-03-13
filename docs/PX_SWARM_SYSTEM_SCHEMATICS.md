# PX-Swarm System Schematics

Reference overview: see the [README](../README.md) for the project summary. These two documents are intended to be easy to view side by side.

## EN: How the whole environment works

### 1. Layered view

```mermaid
flowchart TD
    U[Operator / mission] --> L[Leader node<br/>px_swarm_node --role leader]
    M[Mission YAML<br/>configs/mission_*.yaml] --> L

    subgraph SIM[Simulation layer]
        ISAAC[Isaac Sim / Pegasus<br/>or another simulator backend]
        PX0[PX4 SITL #0]
        PX1[PX4 SITL #1]
        PXN[PX4 SITL #N]
        ISAAC --> PX0
        ISAAC --> PX1
        ISAAC --> PXN
    end

    L <-- MAVSDK / MAVLink UDP --> PX0
    F1[Follower node #1<br/>px_swarm_node --role follower] <-- MAVSDK / MAVLink UDP --> PX1
    FN[Follower node #N<br/>px_swarm_node --role follower] <-- MAVSDK / MAVLink UDP --> PXN

    L <--> B[Swarm Bus UDP<br/>shared state + landing signal]
    F1 <--> B
    FN <--> B

    L --> LOG[CSV flight logs]
    F1 --> LOG
    FN --> LOG
    LOG --> PLOT[Post-run analysis<br/>scripts/plot_metrics.py]
```

### 2. What each layer does

- `px_swarm_node` is a single process that controls exactly one drone.
- The leader flies manually from the keyboard or executes a waypoint mission loaded from YAML.
- A follower does not receive direct operator commands; it receives swarm state and computes its own target.
- PX4 SITL provides autopilot behavior and flight dynamics.
- The world simulator provides the environment and vehicle models; the PX-Swarm control logic talks directly to PX4, so the controller is decoupled from the exact simulator backend.
- The Swarm Bus is a separate UDP channel used only for inter-agent coordination, not PX4 control.

### 3. Runtime topology

```mermaid
flowchart LR
    subgraph D0[Drone 0]
        PX4_0[PX4 SITL :14540]
        N0[Leader node id=0]
        PX4_0 <--> N0
    end

    subgraph D1[Drone 1]
        PX4_1[PX4 SITL :14541]
        N1[Follower node id=1]
        PX4_1 <--> N1
    end

    subgraph D2[Drone 2]
        PX4_2[PX4 SITL :14542]
        N2[Follower node id=2]
        PX4_2 <--> N2
    end

    N0 <--> BUS[UDP swarm bus<br/>for example 239.255.0.1:9000]
    N1 <--> BUS
    N2 <--> BUS
```

### 4. Startup sequence

```mermaid
sequenceDiagram
    participant User as User / launcher
    participant Sim as Gazebo / Isaac / Pegasus
    participant PX4 as PX4 SITL
    participant Node as px_swarm_node
    participant Bus as Swarm Bus

    User->>Sim: Start world and vehicle models
    User->>PX4: Start SITL instances
    User->>Node: Start leader and followers
    Node->>PX4: connect_udp(port)
    Node->>PX4: arm + takeoff
    Node->>PX4: initial zero setpoints
    Node->>PX4: start_offboard()
    Node->>Bus: open_sender/open_receiver
    Node->>Node: enter control loop
```

### 5. Main loop inside one `px_swarm_node`

```mermaid
flowchart TD
    A[Iteration starts] --> B[Read local pose from PX4 telemetry]
    B --> C[Publish own state to Swarm Bus]
    C --> D[Receive other agents states]
    D --> E[Update SwarmState and prune stale entries]
    E --> F{Role?}
    F -->|Leader| G[Keyboard or Mission Controller]
    F -->|Follower| H[Desired formation point]
    H --> I[PSO target selection]
    I --> J[Position setpoint to PX4]
    G --> J
    J --> K[Write CSV diagnostics]
    K --> A
```

### 6. Leader control flow

```mermaid
flowchart TD
    A[Leader tick] --> B{Key pressed?}
    B -->|Yes| C[Manual body velocity control]
    B -->|No, mission loaded| D[Wait until whole swarm stabilizes at takeoff altitude]
    D --> E[Read active waypoint]
    E --> F{Mission speed limit enabled?}
    F -->|Yes| G[Compute NED velocity toward waypoint]
    F -->|No| H[Send position NED waypoint setpoint]
    G --> I{Waypoint reached?}
    H --> I
    I -->|No| J[Continue flight]
    I -->|Yes| K[Hold for hold_s]
    K --> L{Last waypoint?}
    L -->|No| M[Advance to next waypoint]
    L -->|Yes| N[Set mission landing flag]
    C --> O[Offboard commands to PX4]
    J --> O
    M --> O
    N --> P[Leader lands and followers receive landing propagation]
```

### 7. Follower control flow

```mermaid
flowchart TD
    A[Follower tick] --> B[Is leader visible in SwarmState?]
    B -->|No| C[Hold current position]
    B -->|Yes| D[Build desired position relative to leader]
    D --> E[Collect neighbors]
    E --> F{Time to solve PSO again?}
    F -->|Yes| G[Build PsoProblem]
    G --> H[Run PSO inside bounded search region]
    F -->|No| I[Reuse cached target]
    H --> J[Position target]
    I --> J
    J --> K[set_position_ned to PX4]
```

### 8. Inputs, outputs, and diagnostics

```mermaid
flowchart LR
    CFG[CLI args] --> NODE[px_swarm_node]
    MIS[Mission YAML] --> NODE
    KEY[Keyboard] --> NODE
    TEL[PX4 telemetry] --> NODE
    BUSIN[UDP messages from other drones] --> NODE

    NODE --> PX4OUT[Offboard setpoints to PX4]
    NODE --> BUSOUT[State publication to Swarm Bus]
    NODE --> CSV[Per-drone / per-flight CSV logs]
    CSV --> MET[plot_metrics.py]
    MET --> FIG[Plots and quality metrics]
```

### 9. Module map

| Module | Responsibility |
|---|---|
| `src/app/px_swarm_node_main.cpp` | process startup, main loop, mission handling, logging, shutdown |
| `src/app/cli.cpp` | CLI argument parsing |
| `src/app/mission.cpp` | waypoint mission loading from YAML |
| `src/control/px4_interface.cpp` | MAVSDK/PX4 connection, telemetry, Offboard commands |
| `src/control/leader_controller.cpp` | manual leader control |
| `src/control/follower_controller.cpp` | follower logic: desired point + PSO + target tracking |
| `src/swarm/swarm_bus.cpp` | UDP messaging between agents |
| `src/swarm/swarm_state.cpp` | local swarm snapshot and stale-data pruning |
| `src/swarm/pso.cpp` | follower target optimization |
| `scripts/launch_gazebo_px4_instance.sh` | starts Gazebo world and models |
| `scripts/launch_px_swarm_instance.sh` | starts multiple PX4 and `px_swarm_node` instances |
| `scripts/plot_metrics.py` | post-flight log analysis |

### 10. Short end-to-end summary

1. The simulator starts the world and the drone models.
2. PX4 SITL starts a separate autopilot instance for each drone.
3. Each `px_swarm_node` connects to exactly one PX4 instance via MAVSDK.
4. The leader receives keyboard control or waypoint mission data.
5. All nodes publish their current state over UDP to the shared swarm bus.
6. Followers reconstruct a local swarm view and compute an optimal target using PSO.
7. Offboard setpoints are sent back to PX4, while each flight is logged to CSV.
8. At mission completion, the leader broadcasts a landing flag so the whole swarm lands coherently.

## PL: Schemat działania całego środowiska

### 1. Widok warstwowy

```mermaid
flowchart TD
    U[Operator / misja] --> L[Leader node<br/>px_swarm_node --role leader]
    M[Mission YAML<br/>configs/mission_*.yaml] --> L

    subgraph SIM[Warstwa symulacji]
        ISAAC[Isaac Sim / Pegasus<br/>lub inny backend symulacji]
        PX0[PX4 SITL #0]
        PX1[PX4 SITL #1]
        PXN[PX4 SITL #N]
        ISAAC --> PX0
        ISAAC --> PX1
        ISAAC --> PXN
    end

    L <-- MAVSDK / MAVLink UDP --> PX0
    F1[Follower node #1<br/>px_swarm_node --role follower] <-- MAVSDK / MAVLink UDP --> PX1
    FN[Follower node #N<br/>px_swarm_node --role follower] <-- MAVSDK / MAVLink UDP --> PXN

    L <--> B[Swarm Bus UDP<br/>stan roju + sygnał lądowania]
    F1 <--> B
    FN <--> B

    L --> LOG[CSV logi lotu]
    F1 --> LOG
    FN --> LOG
    LOG --> PLOT[Analiza / wykresy<br/>scripts/plot_metrics.py]
```

### 2. Co jest czym

- `px_swarm_node` to pojedynczy proces sterujący dokładnie jednym dronem.
- Lider steruje lotem ręcznie z klawiatury albo wykonuje misję waypointową z pliku YAML.
- Follower nie dostaje bezpośrednich komend od operatora; odbiera stan roju i sam liczy cel lotu.
- PX4 SITL odpowiada za autopilota i dynamikę lotu.
- Symulator świata dostarcza środowisko i modele; logika PX-Swarm komunikuje się jednak bezpośrednio z PX4, więc sam kontroler jest oddzielony od konkretnego backendu symulacji.
- Swarm Bus to osobny kanał UDP używany tylko do koordynacji agentów, nie do sterowania PX4.

### 3. Topologia runtime

```mermaid
flowchart LR
    subgraph D0[Drone 0]
        PX4_0[PX4 SITL :14540]
        N0[Leader node id=0]
        PX4_0 <--> N0
    end

    subgraph D1[Drone 1]
        PX4_1[PX4 SITL :14541]
        N1[Follower node id=1]
        PX4_1 <--> N1
    end

    subgraph D2[Drone 2]
        PX4_2[PX4 SITL :14542]
        N2[Follower node id=2]
        PX4_2 <--> N2
    end

    N0 <--> BUS[UDP swarm bus<br/>np. 239.255.0.1:9000]
    N1 <--> BUS
    N2 <--> BUS
```

### 4. Sekwencja uruchomienia

```mermaid
sequenceDiagram
    participant User as Uzytkownik / skrypt
    participant Sim as Gazebo / Isaac / Pegasus
    participant PX4 as PX4 SITL
    participant Node as px_swarm_node
    participant Bus as Swarm Bus

    User->>Sim: Start swiata i modeli
    User->>PX4: Start instancji SITL
    User->>Node: Start leadera i followerow
    Node->>PX4: connect_udp(port)
    Node->>PX4: arm + takeoff
    Node->>PX4: initial zero setpoints
    Node->>PX4: start_offboard()
    Node->>Bus: open_sender/open_receiver
    Node->>Node: start petli sterowania
```

### 5. Pętla główna jednego `px_swarm_node`

```mermaid
flowchart TD
    A[Start iteracji] --> B[Odczyt lokalnej pozycji z PX4 telemetry]
    B --> C[Publikacja wlasnego stanu na Swarm Bus]
    C --> D[Odbior stanow innych agentow]
    D --> E[Aktualizacja SwarmState i pruning starych wpisow]
    E --> F{Rola?}
    F -->|Leader| G[Keyboard lub Mission Controller]
    F -->|Follower| H[Desired formation point]
    H --> I[PSO target selection]
    I --> J[Position setpoint do PX4]
    G --> J
    J --> K[Zapis diagnostyki CSV]
    K --> A
```

### 6. Przepływ sterowania lidera

```mermaid
flowchart TD
    A[Leader tick] --> B{Nacisnieto klawisz?}
    B -->|Tak| C[Manual body velocity control]
    B -->|Nie, jest misja| D[Czekaj az caly roj ustabilizuje sie na wysokosci startu]
    D --> E[Pobierz aktywny waypoint]
    E --> F{Limit predkosci misji wlaczony?}
    F -->|Tak| G[Wylicz velocity NED do waypointu]
    F -->|Nie| H[Wyslij position NED do waypointu]
    G --> I{Waypoint osiagniety?}
    H --> I
    I -->|Nie| J[Kontynuuj lot]
    I -->|Tak| K[Hold przez hold_s]
    K --> L{Ostatni waypoint?}
    L -->|Nie| M[Nastepny waypoint]
    L -->|Tak| N[Ustaw mission landing flag]
    C --> O[Komendy Offboard do PX4]
    J --> O
    M --> O
    N --> P[Ladowanie lidera + propagacja sygnalu do followerow]
```

### 7. Przepływ sterowania followera

```mermaid
flowchart TD
    A[Follower tick] --> B[Czy leader widoczny w SwarmState?]
    B -->|Nie| C[Utrzymaj biezaca pozycje]
    B -->|Tak| D[Wyznacz desired position wzgledem leadera]
    D --> E[Zbierz sasiadow]
    E --> F{Czas na nowe rozwiazanie PSO?}
    F -->|Tak| G[Zbuduj PsoProblem]
    G --> H[Uruchom PSO w ograniczonym obszarze]
    F -->|Nie| I[Uzyj ostatniego targetu]
    H --> J[Target pozycji]
    I --> J
    J --> K[set_position_ned do PX4]
```

### 8. Dane wejściowe, wyjściowe i diagnostyka

```mermaid
flowchart LR
    CFG[CLI args] --> NODE[px_swarm_node]
    MIS[Mission YAML] --> NODE
    KEY[Klawiatura] --> NODE
    TEL[Telemetry z PX4] --> NODE
    BUSIN[Wiadomosci UDP od innych dronow] --> NODE

    NODE --> PX4OUT[Setpointy Offboard do PX4]
    NODE --> BUSOUT[Publikacja stanu na Swarm Bus]
    NODE --> CSV[CSV per dron / per lot]
    CSV --> MET[plot_metrics.py]
    MET --> FIG[Wykresy i metryki jakosci]
```

### 9. Mapa modułów w repo

| Moduł | Odpowiedzialność |
|---|---|
| `src/app/px_swarm_node_main.cpp` | start procesu, główna pętla, misja, logowanie, shutdown |
| `src/app/cli.cpp` | parsowanie parametrów CLI |
| `src/app/mission.cpp` | ładowanie misji waypointowej z YAML |
| `src/control/px4_interface.cpp` | połączenie MAVSDK z PX4, telemetry, Offboard |
| `src/control/leader_controller.cpp` | sterowanie ręczne lidera |
| `src/control/follower_controller.cpp` | logika followera: desired point + PSO + target tracking |
| `src/swarm/swarm_bus.cpp` | wymiana komunikatów UDP między agentami |
| `src/swarm/swarm_state.cpp` | lokalny obraz roju i wygaszanie starych danych |
| `src/swarm/pso.cpp` | optymalizacja pozycji followera |
| `scripts/launch_gazebo_px4_instance.sh` | start świata i modeli Gazebo |
| `scripts/launch_px_swarm_instance.sh` | start wielu instancji PX4 i `px_swarm_node` |
| `scripts/plot_metrics.py` | analiza logów po locie |

### 10. Najkrótszy opis end-to-end

1. Symulator uruchamia świat i modele dronów.
2. PX4 SITL uruchamia osobny autopilot dla każdego drona.
3. Każdy `px_swarm_node` łączy się z jednym PX4 przez MAVSDK.
4. Leader dostaje sterowanie z klawiatury albo waypointy z misji.
5. Wszystkie nody publikują swój stan przez UDP do wspólnego busa.
6. Followery odbudowują lokalny obraz roju i wyznaczają optymalny target przez PSO.
7. Setpointy Offboard wracają do PX4, a lot jest logowany do CSV.
8. Po końcu misji leader rozsyła flagę lądowania, a cały roj kończy lot spójnie.

## Notes

- The control layer depends on PX4/MAVSDK and the swarm UDP bus, so it can work with different simulator backends as long as PX4 exposes MAVLink UDP.
- These schematics were derived directly from the current implementation in `src/app`, `src/control`, `src/swarm`, `configs`, and `scripts`.
