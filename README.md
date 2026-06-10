# Robot Navigation & Control Simulator

An interactive 2D robot navigation simulator built in C++17 using **SFML 3.0**. It combines A* global path planning, Bezier corner smoothing, and a potential-field-guided Pure Pursuit controller for smooth, collision-free path tracking.

---

## Features

- **Global Path Planning (A*)**: Computes optimal paths using a soft-cost grid to encourage clearance from obstacles.
- **Bezier Curve Smoothing**: Dynamically fits safety-bounded quadratic Bezier curves around path corners to prevent sharp, abrupt turns.
- **Pure Pursuit Controller**: Features a decoupled lookahead tracking algorithm with a low-pass filter (input shaper) on target velocities for smooth transitions.
- **Reactive Potential & Vortex Fields**: Implements potential fields and rotational vortex fields to actively push the robot away from obstacle boundaries if it deviates from the path.
- **Target projection**: Real-time geometric projection (`ProjectTargetToSafe`) that automatically slides targets placed inside obstacles to the closest safe boundary.
- **Interactive SFML GUI**:
  - Drag and drop obstacles to see the path adapt in real-time.
  - Drag the goal point to watch target projection and navigation update dynamically.

---

## Directory Structure

```
├── common.hpp        # Shared structures (Robot, Obstacle) and math utilities (distance, projection)
├── pathfinder.hpp/cpp # A* search, path pruning, and Bezier curve fitting
├── controller.hpp/cpp # Pure Pursuit tracker, potential fields, and velocity input shaping
├── main.cpp          # SFML window orchestration, event handling, and rendering
└── .gitignore        # Standard ignore patterns for builds and IDEs
```

---

## Dependencies

- **Compiler**: GCC supporting C++17 (or newer).
- **Library**: [SFML 3.0](https://www.sfml-dev.org/) (Graphics, Window, System modules).

---

## Build and Run

To compile the application using `g++` on Windows/Mingw-w64 (assuming SFML is in your compiler's library search path):

```bash
g++ -std=c++17 -O2 main.cpp pathfinder.cpp controller.cpp -o robot.exe -lsfml-graphics -lsfml-window -lsfml-system
```

Run the executable:
```bash
./robot.exe
```
