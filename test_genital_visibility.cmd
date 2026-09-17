@echo off
setlocal
set "PATH=C:\msys64\mingw32\bin;%PATH%"
pushd "%~dp0" || exit /b 1
gcc -m32 -O2 -static-libgcc -o build/genital_visibility_test.exe genital_visibility_test.c -ld3d8 -lgdi32 -lopengl32
if errorlevel 1 goto done
build\genital_visibility_test.exe
:done
set "test_exit=%errorlevel%"
popd
exit /b %test_exit%
