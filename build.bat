@echo off
setlocal
set BUILD_DIR=%~dp0build
set SRC_DIR=%~dp0cpp

if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"

cmake -S "%SRC_DIR%" -B "%BUILD_DIR%" -A x64
if errorlevel 1 exit /b 1

cmake --build "%BUILD_DIR%" --config Release
if errorlevel 1 exit /b 1

echo.
echo Build complete. DLL staged at:
echo   %~dp0moddingtool\bin\moddingtool.dll
endlocal
