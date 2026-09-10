@echo off
setlocal

set "GCC=C:\msys64\mingw32\bin\gcc.exe"
set "WINDRES=C:\msys64\mingw32\bin\windres.exe"
set "PATH=C:\msys64\mingw32\bin;%PATH%"
set "PROJECT_DIR=%~dp0"
set "BUILD_DIR=%PROJECT_DIR%build"

if not exist "%GCC%" (
  echo MinGW GCC was not found at "%GCC%".
  exit /b 1
)

if not exist "%WINDRES%" (
  echo MinGW windres was not found at "%WINDRES%".
  exit /b 1
)

if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"

"%WINDRES%" ^
  --target=pe-i386 ^
  -i "%PROJECT_DIR%physx_resources.rc" ^
  -o "%BUILD_DIR%\physx_resources.o"

if errorlevel 1 (
  echo Resource compilation failed.
  exit /b 1
)

"%GCC%" ^
  -m32 ^
  -shared ^
  -O2 ^
  -s ^
  -static-libgcc ^
  -o "%BUILD_DIR%\NC-TK17-PhysX.dll" ^
  "%PROJECT_DIR%NC-TK17-PhysX.c" ^
  "%BUILD_DIR%\physx_resources.o" ^
  -ld3d8 ^
  -lgdi32 ^
  -lopengl32

if errorlevel 1 (
  echo Compilation failed.
  exit /b 1
)

del /q "%BUILD_DIR%\physx_resources.o" >nul 2>nul

echo Built "%BUILD_DIR%\NC-TK17-PhysX.dll".
