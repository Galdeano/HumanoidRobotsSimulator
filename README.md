# Humanoid Robot Simulator

A C-based multi-body humanoid robot simulator utilizing **OpenGL** and **SDL** for 3D visualization, and the **GNU Scientific Library (GSL)** for matrix computations and kinematics/dynamics algorithms.

This simulator was developed as part of a Master's/PhD thesis project (circa 2011–2015) by David Galdeano. It has been cleaned up and modernized to run on modern C/C++ environments using **CMake**.

---

## Features
*   **3D OpenGL Visualization**: Render ground planes, coordinate markers, force vectors, and humanoid robot meshes.
*   **Kinematics & Dynamics Engines**:
    *   Forward Kinematics (FK) and Inverse Kinematics (IK) for humanoid models.
    *   Forward Dynamics (FD) using Euler integration and contact force resolvers.
*   **Zero Moment Point (ZMP) & Balance**:
    *   Real-time ZMP computation and filtering.
    *   Linear and non-linear spring-damper contact models for foot-ground interactions.
*   **Trajectory & Scenario Generation**: Articular limits, spline interpolation, and pre-programmed motion trajectories (squats, arm movements, walking gaits).
*   **Multi-Model Support**: Configurations for HOAP-3 humanoid robot, Sherpa, and generic biped platforms.

---

## Build Instructions (Windows - MSYS2 / MinGW-w64)

### 1. Prerequisites
Install **MSYS2** (if not already installed) and set up the compiler toolchain. Open an MSYS2 shell and run:
```bash
pacman -S --noconfirm mingw-w64-x86_64-gcc mingw-w64-x86_64-gsl mingw-w64-x86_64-SDL mingw-w64-x86_64-make mingw-w64-x86_64-cmake
```

Ensure `C:\msys64\mingw64\bin` (or your MSYS2 install path) is added to your system environment `PATH`.

### 2. Configure and Compile
In your terminal (e.g. PowerShell), run:
```powershell
# Generate build files
cmake -B build -G "MinGW Makefiles"

# Compile the simulator
cmake --build build
```
This will compile the project and generate the executable at `build/SimuSherpaConsole.exe`.

---

## Build Instructions (Linux)
Install the dependencies using your package manager:
```bash
# Ubuntu / Debian
sudo apt update
sudo apt install build-essential cmake libgsl-dev libsdl1.2-dev libgl1-mesa-dev libglu1-mesa-dev
```

Then configure and build the project:
```bash
cmake -B build
cmake --build build
```

---

## Running the Simulator

Launch the executable from the **root folder of the project** so that it can locate the robot model description files (contained in `./Robots/`):

```bash
# Windows
.\build\SimuSherpaConsole.exe

# Linux
./build/SimuSherpaConsole
```

### Controls
*   **Interactive Menu**: The simulator defaults to loading the HOAP-3 humanoid robot configuration (`./Robots/HOAP3v7.xml`).
*   **Mouse Navigation**: Use mouse movement to drag and change the OpenGL camera angle.

---

## Repository Cleanup Details
Before publishing, the repository was cleaned of legacy and unnecessary files:
*   Removed conflicted layout/project backups (`*.layout`, sync folders).
*   Removed large pre-built installers (`codeblocks-17.12mingw-setup.exe` and `gsl-1.8.exe`).
*   Replaced the hardcoded absolute library paths in Code::Blocks with a portable `CMakeLists.txt` build system.
*   Disabled network hardware loops (`#define reseau 0` in `Setup.h`) to run purely locally without requiring physical HOAP-3 robot hardware.
