import subprocess
import time
import sys
import os

def main():
    # Set MSYS2 bin path to environment to ensure DLLs are resolved correctly
    msys_bin = r"C:\Users\User\msys64\mingw64\bin"
    env = os.environ.copy()
    if os.path.exists(msys_bin):
        env["PATH"] = msys_bin + os.pathsep + env.get("PATH", "")

    exe_path = os.path.join("build", "SimuSherpaConsole.exe")
    if not os.path.exists(exe_path):
        print(f"Error: Executable not found at {exe_path}. Please build the project first.")
        sys.exit(1)
        
    print(f"Starting simulator: {exe_path}")
    
    # Start the process without redirecting stdout/stderr so that 
    # the console output is visible directly in the user's terminal.
    # We set cwd to the project root directory so assets are found correctly.
    project_root = os.path.dirname(os.path.abspath(__file__))
    process = subprocess.Popen([exe_path], cwd=project_root, env=env)
    
    run_duration = 5.0  # seconds
    print(f"Running simulator for {run_duration} seconds...")
    
    time.sleep(run_duration)
    
    # Check if the process is still running
    ret_code = process.poll()
    if ret_code is None:
        print("Simulator is running successfully. Terminating it now...")
        process.terminate()
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            process.kill()
        print("Simulator successfully validated! No crashes detected.")
        sys.exit(0)
    else:
        print(f"Simulator exited prematurely with exit code: {ret_code}")
        print("This could be due to running in a headless terminal where SDL cannot open a window.")
        sys.exit(1)

if __name__ == "__main__":
    main()
