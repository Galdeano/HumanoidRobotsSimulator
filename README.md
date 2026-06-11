# Humanoid Robot Simulator

A C++ multi-body humanoid robot simulator utilizing **Modern OpenGL 3.3 (Core Profile)** and **SDL2** for 3D visualization, and the **GNU Scientific Library (GSL)** for matrix computations and kinematics/dynamics algorithms.

This simulator has been upgraded to modern rendering standards, replacing all legacy fixed-function OpenGL calls with programmable shaders and custom vertex buffer matrices.

---

## Features
*   **Modern OpenGL 3.3 Visualization**: Flat and shaded rendering models, custom materials, and stenciled shadow volumes.
*   **Kinematics & Dynamics Engines**:
    *   Forward Kinematics (FK) and Inverse Kinematics (IK) for humanoid models.
    *   Forward Dynamics (FD) using Euler integration and contact force resolvers.
*   **Zero Moment Point (ZMP) & Balance**:
    *   Real-time ZMP computation and filtering.
    *   Linear and non-linear spring-damper contact models for foot-ground interactions.
*   **Interactive Camera Controls**: Supports mouse translation, drag rotations, scroll zooming, and keyboard-driven yaw.
*   **Visual Indicators**: Real-time overlays for Center of Mass (CoM, turquoise cross), Zero Moment Point (ZMP, ruby cross), foot centers, and contact force vectors.
*   **Multi-Model Support**: Configurations for the HOAP-3 humanoid robot, Sherpa, and generic biped platforms.

---

## Build Instructions (Windows - MSYS2 / MinGW-w64)

### 1. Prerequisites
Install **MSYS2** (if not already installed) and open an MSYS2 MinGW-w64 Shell. Run the following command to install the compiler toolchain and dependencies:
```bash
pacman -S --noconfirm mingw-w64-x86_64-gcc \
                      mingw-w64-x86_64-gsl \
                      mingw-w64-x86_64-sdl2 \
                      mingw-w64-x86_64-glew \
                      mingw-w64-x86_64-glm \
                      mingw-w64-x86_64-pugixml \
                      mingw-w64-x86_64-cmake \
                      mingw-w64-x86_64-make
```

Ensure the path to your MinGW binaries (default: `C:\msys64\mingw64\bin`) is added to your system environment `PATH` variable.

### 2. Configure and Compile
In your terminal (e.g. PowerShell or Command Prompt), run:
```powershell
# Generate CMake build files
cmake -B build -G "MinGW Makefiles"

# Compile the simulator
cmake --build build
```
This generates the executable at `build/SimuSherpaConsole.exe`.

---

## Build Instructions (Linux)

### 1. Prerequisites
Install the dependencies using your distribution's package manager:
```bash
# Ubuntu / Debian
sudo apt update
sudo apt install build-essential cmake libgsl-dev libsdl2-dev libglew-dev libglm-dev libpugixml-dev libgl1-mesa-dev
```

### 2. Configure and Compile
```bash
# Generate build files
cmake -B build

# Compile the simulator
cmake --build build
```
This generates the executable at `build/SimuSherpaConsole`.

---

## Launching the Simulator

### Windows
You can launch the simulator using the wrapper batch script or Python script from the project root folder. These scripts ensure that the MSYS2 dll paths are in your environment and that the required `Simu_data` directories exist:

*   **Option A (Batch Script - Recommended)**: Double-click or run:
    ```powershell
    .\launch.bat
    ```
*   **Option B (Python Launcher)**: Run:
    ```powershell
    python launch_simulator.py
    ```
*   **Option C (Direct Executable)**: Ensure the compiler binary directory is on your path and directory structures exist, then run:
    ```powershell
    build\SimuSherpaConsole.exe
    ```

### Linux
Launch the executable directly from the root of the project:
```bash
./build/SimuSherpaConsole
```

---

## Controls & Customization

### Interactive Controls
*   **Rotate Camera**: Click and drag with the Left Mouse Button (LMB), or use the **Left** and **Right** arrow keys.
*   **Pan Camera**: Click and drag with the Right Mouse Button (RMB).
*   **Zoom Camera**: Scroll the mouse wheel.
*   **Toggle Ground**: Press **G** to toggle drawing the ground plane.
*   **Toggle Visualisation**: Settings in `src/Setup.h` configure whether to compute dynamic constraints (`#define StaticCOM`) or save simulation trajectories.

### Simulator Constants
The simulator's parameters can be modified inside `src/Setup.h`. For example:
*   `reseau`: Toggle physical hardware connection vs local simulation (default `0`).
*   `file_walk`: Load offline trajectories (default `1`).
*   `PD`, `Dynamic`, `Task`: Toggle control strategies (default `Task 1` for task-space operational controller).
