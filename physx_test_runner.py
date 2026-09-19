"""Compile a production test and run it in an isolated temporary directory."""
from pathlib import Path
import os
import subprocess
import tempfile


def run_production_test(source_name):
    root = Path(__file__).resolve().parent
    source = root / source_name
    build = root / "build"
    build.mkdir(exist_ok=True)
    gcc = Path(r"C:\msys64\mingw32\bin\gcc.exe")
    env = os.environ.copy()
    env["PATH"] = str(gcc.parent) + os.pathsep + env["PATH"]
    with tempfile.TemporaryDirectory(prefix=source.stem + "-", dir=build) as temp:
        exe = Path(temp) / (source.stem + ".exe")
        subprocess.run([str(gcc), "-m32", "-O2", "-static-libgcc", "-o", str(exe),
                        str(source), "-ld3d8", "-lgdi32", "-lopengl32"],
                       env=env, check=True)
        subprocess.run([str(exe)], cwd=temp, env=env, check=True, timeout=60)
