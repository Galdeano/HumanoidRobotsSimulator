@echo off
:: Set PATH to include MSYS2 MinGW-w64 DLLs
set PATH=C:\Users\User\msys64\mingw64\bin;%PATH%

:: Force SDL 1.2 to use the GDI windowing driver (windib) instead of DirectX.
:: This fixes "error 3" (SDL video init failure) on modern Windows 10/11.
set SDL_VIDEODRIVER=windib
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
