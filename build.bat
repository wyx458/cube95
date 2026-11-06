@echo off
rem ===================================================================
rem  Rebuild rubiks.exe from rubiks.c (pure C, no third-party deps)
rem  Requires MinGW-w64 GCC. Will find it in PATH, in a local
rem  .toolchain\w64devkit folder, or in the temp folder used by the
rem  original build.
rem ===================================================================
setlocal
set "HERE=%~dp0"
set "GCC="

where gcc >nul 2>nul
if %errorlevel%==0 set "GCC=gcc"

if not defined GCC if exist "%HERE%.toolchain\w64devkit\bin\gcc.exe" set "GCC=%HERE%.toolchain\w64devkit\bin\gcc.exe"

if not defined GCC if exist "%TEMP%\rubiks-build\w64devkit\w64devkit\bin\gcc.exe" set "GCC=%TEMP%\rubiks-build\w64devkit\w64devkit\bin\gcc.exe"

if not defined GCC (
  echo [ERROR] MinGW-w64 GCC not found.
  echo Get w64devkit from https://github.com/skeeto/w64devkit/releases
  echo and extract it into:  %HERE%.toolchain\w64devkit\
  exit /b 1
)

echo Using: %GCC%
"%GCC%" -O2 -Wall -Wextra -finput-charset=UTF-8 -fexec-charset=UTF-8 -static -mwindows -o "%HERE%rubiks.exe" "%HERE%rubiks.c" -lm -lgdi32 -luser32
if errorlevel 1 (
  echo [ERROR] GUI build failed.
  exit /b 1
)
echo [OK] rubiks.exe built.

echo --- running logic self-test ---
"%GCC%" -O2 -Wall -Wextra -DSELFTEST -o "%HERE%rubiks_test.exe" "%HERE%rubiks.c" -lm
if errorlevel 1 (
  echo [ERROR] self-test build failed.
  exit /b 1
)
"%HERE%rubiks_test.exe"
if errorlevel 1 (
  echo [ERROR] self-test failed.
  exit /b 1
)
echo [OK] self-test passed.

endlocal
