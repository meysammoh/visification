> **⚠️ Archived Project (2015)** — This is a historical portfolio piece, not actively maintained. Not recommended for forking or production use.

# Visification - Multi-Robot Visibility-Based Path Planning (2015)

A computational geometry research project implementing visibility-based path planning algorithms for multiple robots navigating in a polygonal environment.

## Overview

Visification is a C++ application that visualizes and solves the multi-robot motion planning problem using visibility polygons and dual graph techniques. The system computes all possible states where robots can move within a polygon while maintaining visibility constraints through "windows" (visibility regions).

**Development Period:** November 2015
**Type:** Research Implementation / Algorithm Visualization
**Domain:** Computational Geometry, Robotics, Motion Planning

## Problem Description

### Multi-Robot Visibility Planning
Given:
- A simple polygon P (the environment)
- Multiple robots, each constrained to move along a specific edge of P
- Each robot has a visibility region (visibility polygon)

Goal:
- Find all valid configurations where robots can move
- Robots move through "windows" (intersection of visibility regions)
- Visualize the state space and possible transitions

### Key Concepts

**Visibility Polygon**: The region visible from a robot's position, bounded by polygon edges and obstacles.

**Window**: A line segment through which a robot can pass while maintaining visibility with other robots or the environment.

**Dual Graph**: A graph representation where each node corresponds to a subdivision cell, connected if cells share an edge.

**Event Points**: Critical points where robot visibility or configuration changes significantly.

**Reflective Points**: Points where robot paths reflect off polygon edges.

## Technical Architecture

### Core Components

#### Visifier.cpp/h (~50KB, 2.6KB)
**Purpose:** Main algorithm implementation
**Features:**
- Computes visibility polygons using CGAL
- Constructs windowed environment (subdivided polygon)
- Builds dual graph of subdivisions
- Manages robot movements and state transitions
- Multi-threaded state exploration

#### Robot.cpp/h (~7KB, 1.5KB)
**Purpose:** Robot entity and behavior
**Attributes:**
- Position on polygon edge
- Visibility area (arrangement)
- Window segments
- Movement constraints
- Event point handling

#### Utility.cpp/h (~7KB, 1.4KB)
**Purpose:** Geometric utility functions
**Functions:**
- Point-in-polygon tests
- Line segment intersections
- Visibility calculations
- Arrangement manipulations

#### Graph.cpp/h (~2.7KB, 506B)
**Purpose:** Dual graph representation
**Structure:**
- Adjacency list for cells
- Edge connectivity information
- Graph traversal utilities

### Visualization (OpenGL/GLUT)

**main.cpp:**
- Real-time rendering of:
  - Polygon environment
  - Robot positions
  - Visibility regions
  - Windows and paths
  - State transitions
- Interactive controls:
  - `n`: Next state
  - `z`: Zoom in
  - `x`: Zoom out

### Data Structures

**Event Points** (`EventPoint.hpp`):
- Critical geometric events
- Visibility changes
- Robot transition points

**Reflective Points** (`ReflectivePoint.hpp`):
- Path reflection on edges
- Angle calculations

**Robot Event Points** (`RobotEventPoint.hpp`):
- Robot-specific transition events
- Movement triggers

## Algorithm Overview

### State Computation Process

1. **Initialize Environment**
   - Load polygon vertices
   - Parse robot positions and constraints
   - Compute bounding box

2. **Compute Visibility Regions**
   - For each robot, calculate visibility polygon
   - Use CGAL visibility arrangements
   - Identify non-edge segments as "windows"

3. **Construct Windowed Environment**
   - Intersect all robot windows
   - Create subdivision of polygon
   - Build planar arrangement

4. **Build Dual Graph**
   - Create node for each subdivision cell
   - Connect nodes if cells share edge
   - Enables graph-based path planning

5. **Find Event Points**
   - Identify critical transition points
   - Compute reflective points
   - Determine robot movement triggers

6. **State Exploration**
   - Multi-threaded search
   - Generate successor states
   - Prune invalid configurations
   - Track all reachable states

## Technology Stack

### Computational Geometry
- **CGAL (Computational Geometry Algorithms Library)**
  - Arrangement_2 (planar arrangements)
  - Visibility computations
  - Polygon operations
  - Kernel primitives (Point_2, Segment_2, Polygon_2)

### Graphics & Visualization
- **OpenGL:** 3D graphics rendering
- **GLUT (FreeGLUT):** Windowing and input handling
- **2D Projection:** Top-down polygon view

### Concurrency
- **C++11 Threads:** Parallel state exploration
- **Mutex:** Thread-safe state management
- **Condition Variables:** Synchronization

### Build System
- **CMake 2.8.11+:** Cross-platform build
- **C++11:** Modern C++ features

## Project Structure

```
visification/
├── CMakeLists.txt              # Root CMake configuration
├── lib/                        # Core library
│   ├── CMakeLists.txt
│   ├── Visifier.cpp            # Main algorithm (50KB)
│   ├── Visifier.h
│   ├── Robot.cpp               # Robot entity
│   ├── Robot.h
│   ├── Utility.cpp             # Geometric utilities
│   ├── Utility.h
│   ├── Graph.cpp               # Dual graph
│   ├── Graph.h
│   ├── EventPoint.hpp          # Event structures
│   ├── EventPointBase.cpp
│   ├── EventPointBase.h
│   ├── ReflectivePoint.hpp
│   ├── RobotEventPoint.hpp
│   ├── ReflectiveToPathMapPoint.hpp
│   └── CGALHelper.hpp          # CGAL type definitions
├── apps/
│   └── Interactive/            # Visualization app
│       └── main.cpp            # OpenGL viewer
├── testcases/                  # Test environments
│   ├── polygon_a.dat
│   ├── polygon_b.dat
│   ├── polygon_b_3robot.dat
│   ├── polygon_b_extended.dat
│   ├── polygon_c.dat
│   ├── polygon_d.dat
│   └── test/
├── documents/
│   └── initial_algorithm.txt   # Algorithm description
├── Config.cpp/h                # Configuration handling
├── Engine.cpp/h                # Legacy engine (replaced by Visifier)
├── Robot.cpp/h                 # Legacy robot (kept for compatibility)
├── State.cpp/h                 # State management
├── Graph.cpp/h                 # Graph utilities
├── EventPoint.cpp/h            # Event handling
├── helper.cpp/h                # Helper functions
└── main.cpp                    # Standalone executable
```

## Input Format

### Environment Definition (.dat files)
```
Environment
0  0        # First vertex (x, y)
2  0        # Second vertex
1.5 1       # Third vertex
...         # Additional vertices
0  2        # Last vertex (closes polygon)

Robots
2 1 3 1     # Robot 1: edge start (2,1), edge end (3,1)
.5 1.5 1.5 1.8  # Robot 2: edge coordinates
3.8 1.8 4.7 .8  # Robot 3: edge coordinates
```

**Format:**
- **Environment section:** Polygon vertices in counter-clockwise order
- **Robots section:** Each line defines one robot's constraining edge (x1, y1, x2, y2)

## Building the Project

### Prerequisites
- **CGAL:** Version 4.x or 5.x
  - Requires Boost, GMP, MPFR
  - Install via package manager or from source
- **OpenGL:** Graphics library (usually pre-installed)
- **GLUT/FreeGLUT:** Windowing toolkit
- **CMake:** 2.8.11 or higher
- **C++11 Compiler:** GCC 4.8+, Clang 3.4+, MSVC 2013+

### Ubuntu/Linux Installation
```bash
# Install dependencies
sudo apt-get install libcgal-dev libcgal-qt5-dev
sudo apt-get install freeglut3-dev libglu1-mesa-dev
sudo apt-get install cmake g++

# Build project
mkdir build
cd build
cmake ..
make

# Run with test case
./visification ../testcases/polygon_b.dat
```

### macOS Installation
```bash
# Install dependencies via Homebrew
brew install cgal
brew install freeglut

# Build project
mkdir build && cd build
cmake ..
make

# Run
./visification ../testcases/polygon_b.dat
```

### Windows (Visual Studio)
```bash
# Install CGAL and dependencies (use vcpkg)
vcpkg install cgal freeglut

# Generate Visual Studio project
mkdir build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=[vcpkg]/scripts/buildsystems/vcpkg.cmake

# Open .sln file in Visual Studio and build
```

## Running the Application

### Command Line
```bash
# Run with specific test case
./visification testcases/polygon_b_3robot.dat

# Run without argument (uses default or prompts)
./visification
```

### Interactive Controls
- **`n`**: Advance to next state in sequence
- **`z`**: Zoom in (increase view scale)
- **`x`**: Zoom out (decrease view scale)

### Output
- **Real-time status:**
  - Number of new states found
  - Total states explored
  - Elapsed time in seconds
- **File output:** State summaries written periodically

## Test Cases

### Provided Environments

**polygon_a.dat**: Simple rectangular environment with 2 robots

**polygon_b.dat**: Non-convex polygon with 4 robots

**polygon_b_3robot.dat**: Same environment, 3 robots

**polygon_b_extended.dat**: Larger version with more complexity

**polygon_c.dat**: Different topology

**polygon_d.dat**: Alternative configuration

**polygon_d_3robot.dat**: 3-robot variant

## Algorithm Complexity

### Time Complexity
- **Visibility Polygon:** O(n) per robot (n = polygon vertices)
- **Arrangement Construction:** O(w²) (w = number of windows)
- **Dual Graph:** O(c + e) (c = cells, e = edges)
- **State Exploration:** Exponential in robots (state space explosion)

### Space Complexity
- **Arrangements:** O(w²) per robot
- **Dual Graph:** O(c + e)
- **State Storage:** O(states × robots × coordinates)

## Research Context

This project implements concepts from computational geometry research on:
- **Art Gallery Problems:** Visibility coverage
- **Multi-Robot Path Planning:** Coordination and collision avoidance
- **Visibility-Based Decomposition:** Environment subdivision
- **Configuration Space:** Robot state representation

### Related Work
- Visibility polygons (Ghosh, Mount)
- Multi-robot motion planning (LaValle)
- Art gallery theorems (Chvátal)
- Computational geometry (de Berg et al.)

## Known Limitations

### Scalability
- **State Space Explosion:** Grows exponentially with robot count
- **Memory Usage:** Large arrangements consume significant RAM
- **Computation Time:** Complex polygons take longer to process

### Assumptions
- Robots are point objects (no physical size)
- Simple polygons (no holes, self-intersections)
- Static environment (no dynamic obstacles)
- Perfect information (complete polygon knowledge)

## Version History

**2015-11-25:** CMake configuration finalized
**2015-11-24:** Visifier algorithm completed
**2015-11-23:** Interactive visualization implemented
**2015-11-22:** Robot behavior refined
**2015-11-20:** Library structure established
**2015-11-17:** Initial test cases created

## Skills Demonstrated

| Category | Skills |
|----------|--------|
| **Computational Geometry** | CGAL, visibility polygons, planar arrangements |
| **Algorithm Design** | Dual graphs, state space exploration, event-based computation |
| **C++ Development** | Templates, CGAL kernel types, modern C++11 |
| **Concurrency** | Multi-threaded search, mutex synchronization |
| **Visualization** | OpenGL, GLUT, real-time rendering |
| **Research Implementation** | Academic algorithm translation to working code |

## License

MIT License (see LICENSE file)

## Credits

**Development:** November 2015
**Domain:** Computational Geometry, Robotics
**Libraries:** CGAL, OpenGL, GLUT
**Algorithm:** Visibility-based multi-robot planning

---

*Archived 2015 research project demonstrating computational geometry with CGAL, visibility polygon algorithms, multi-robot path planning, and real-time OpenGL visualization.*
