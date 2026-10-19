@echo off
setlocal
if not defined VSCMD_VER (
  if exist "C:\BuildTools\Common7\Tools\VsDevCmd.bat" (
    call "C:\BuildTools\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64
  ) else (
    echo Run this script from a Visual Studio x64 developer command prompt.
    exit /b 1
  )
)
cmake -S "%~dp0." -B "%~dp0build" -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Release
if errorlevel 1 exit /b %errorlevel%
cmake --build "%~dp0build"
if errorlevel 1 exit /b %errorlevel%
ctest --test-dir "%~dp0build" --output-on-failure
if errorlevel 1 exit /b %errorlevel%
cmake --install "%~dp0build" --prefix "%~dp0dist"
exit /b %errorlevel%
