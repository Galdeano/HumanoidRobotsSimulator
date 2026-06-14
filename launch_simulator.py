import subprocess
import time
import sys
import os

def main():
    # 1. Set MSYS2 bin path and SDL video driver variables
    msys_bin = r"C:\Users\User\msys64\mingw64\bin"
    env = os.environ.copy()
    if os.path.exists(msys_bin):
        env["PATH"] = msys_bin + os.pathsep + env.get("PATH", "")
    # env["SDL_VIDEODRIVER"] = "windib"
    env["SDL_VIDEO_CENTRED"] = "1"

    # 2. Ensure Simu_data directories exist to prevent crashes when files are opened
    # The codebase on master opens files in relative paths like "./../Simu_data/..." and "./../../Simu_data/..."
    # If the process cwd is the project root (C:\Users\User\Drive\HumanoidRobotsSimulator):
    #   - "./../Simu_data" resolves to "C:\Users\User\Drive\Simu_data"
    #   - "./../../Simu_data" resolves to "C:\Users\User\Simu_data"
    # We will create both directories (plus one under the project root to be safe).
    project_root = os.path.dirname(os.path.abspath(__file__))
    simu_data_paths = [
        os.path.join(project_root, "Simu_data"),
        os.path.abspath(os.path.join(project_root, "..", "Simu_data")),
        os.path.abspath(os.path.join(project_root, "..", "..", "Simu_data"))
    ]
    
    for path in simu_data_paths:
        try:
            if not os.path.exists(path):
                os.makedirs(path, exist_ok=True)
                print(f"Created required directory: {path}")
        except Exception as e:
            print(f"Warning: could not create directory {path}: {e}")

    # 3. Check for the executable
    exe_path = os.path.join(project_root, "build", "SimuSherpaConsole.exe")
    if not os.path.exists(exe_path):
        print(f"Error: Executable not found at {exe_path}. Please build the project first.")
        sys.exit(1)
        
    print(f"Starting simulator: {exe_path}")
    print(f"Working Directory: {project_root}")
    
    # 4. Start the process without redirecting stdout/stderr so that 
    # the console/graphics output is visible directly.
    process = subprocess.Popen([exe_path], cwd=project_root, env=env)
    
    # Allow custom run duration via command line argument (default to 5.0 seconds)
    run_duration = 5.0
    if len(sys.argv) > 1:
        try:
            run_duration = float(sys.argv[1])
        except ValueError:
            print(f"Invalid duration '{sys.argv[1]}', using default of 5.0 seconds.")
            
    print(f"Running simulator for {run_duration} seconds...")
    
    time.sleep(run_duration)
    
    # 5. Check if the process is still running
    ret_code = process.poll()
    if ret_code is None:
        print("Simulator is running successfully. Terminating it now...")
        process.terminate()
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            process.kill()
        print("Simulator successfully launched and terminated without crashes.")
        sys.exit(0)
    else:
        print(f"Simulator exited prematurely with exit code: {ret_code}")
        print("Note: The simulator requires a desktop environment with a valid display.")
        sys.exit(1)

if __name__ == "__main__":
    main()
