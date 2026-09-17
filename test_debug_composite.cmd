@echo off
setlocal
set "PATH=C:\msys64\mingw32\bin;%PATH%"
pushd "%~dp0" || exit /b 1
gcc -m32 -O2 -static-libgcc -o build/physx_debug_composite_test.exe physx_debug_composite_test.c -ld3d11 -lgdi32
if errorlevel 1 goto done
build\physx_debug_composite_test.exe
if errorlevel 1 goto done
gcc -m32 -O2 -static-libgcc -o build/physx_wire_shapes_test.exe physx_wire_shapes_test.c -ld3d8 -lgdi32 -lopengl32
if errorlevel 1 goto done
build\physx_wire_shapes_test.exe
:done
set "test_exit=%errorlevel%"
popd
exit /b %test_exit%
