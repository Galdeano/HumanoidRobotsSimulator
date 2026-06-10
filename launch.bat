@echo off
:: Set PATH to include MSYS2 MinGW-w64 DLLs
set PATH=C:\Users\User\msys64\mingw64\bin;%PATH%

:: Force SDL2 to use the default windows driver (windib is not supported in SDL2)
set SDL_VIDEO_CENTRED=1

:: Navigate to the repository root directory
cd /d "c:\Users\User\Drive\HumanoidRobotsSimulator"

:: Ensure Simu_data directories exist to prevent file write crashes
if not exist "Simu_data" mkdir "Simu_data"
if not exist "..\Simu_data" mkdir "..\Simu_data"
if not exist "..\..\Simu_data" mkdir "..\..\Simu_data"

:: Start the simulator
echo Starting SimuSherpaConsole...
build\SimuSherpaConsole.exe

if %ERRORLEVEL% neq 0 (
    echo.
    echo Simulator exited with error code %ERRORLEVEL%.
    pause
)
