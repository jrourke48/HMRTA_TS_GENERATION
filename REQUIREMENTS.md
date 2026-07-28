# Project Requirements

This document outlines the dependencies required for the HMRTA with Multi-Capability Robots project.

## Spot Library
- **Version**: 2.14.4.dev
- **Location**: `C:/Users/johnn/Spot Library/spot-2.14.4.dev`
- **Libraries**: libspot, libbddx
- **Windows Build Dependencies**:
  - g++ (C++20 or later)
  - MinGW-w64 compiler
  - BDD library (buddy)

### Building Spot on Linux/Ubuntu (for reference):
```bash
# Install build dependencies
sudo apt update
sudo apt install -y \
  build-essential git pkg-config \
  autoconf automake libtool \
  flex bison \
  libbdd-dev libgmp-dev libboost-all-dev \
  python3-dev swig

# Clone and build
git clone --recurse-submodules https://gitlab.lre.epita.fr/spot/spot.git
cd spot
autoreconf -vfi
./configure --prefix=/usr/local
make -j$(nproc)
sudo make install
sudo ldconfig
```

**Note**: The development version has assertions and debugging code enabled. Use `--disable-devel` flag if benchmarking.

## dstar (D* Lite Algorithm)
- **Location**: `dstar/`
- **Build Tool**: Make
- **C++ Compiler**: g++
- **Graphics Dependencies**:
  - **macOS**: OpenGL framework, GLUT framework
  - **Linux**: libGL, libGLU, libglut
  - **Windows**: OpenGL (typically included), GLUT libraries

## Robotics_Research
- **Location**: `Robotics_Research/`
- **Dependencies**: Inherits Spot library dependencies
- **Visualization**: graphviz, xdot (for LTL visualization)
  - Install on Ubuntu: `sudo apt install graphviz xdot`

## Compiler & Tools
- **C++ Standard**: C++20
- **Compiler**: g++ (MinGW on Windows)
- **Build System**: Make/CMake
- **Compiler Flags**: `-std=c++20 -Wall -Wextra -g3`

## Optional Visualization
- graphviz (for automata and formula visualization)
- xdot (for interactive dot file viewer)
