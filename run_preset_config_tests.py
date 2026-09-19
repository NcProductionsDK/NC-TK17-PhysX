"""Run the production config loader in an isolated miniature game directory."""
from pathlib import Path
import os
import subprocess
import tempfile

root = Path(__file__).resolve().parent
build = root / "build"
build.mkdir(exist_ok=True)
env = os.environ.copy()
env["PATH"] = r"C:\msys64\mingw32\bin" + os.pathsep + env["PATH"]
with tempfile.TemporaryDirectory(prefix="preset-tests-", dir=build) as temp:
    binaries = Path(temp) / "Binaries"
    binaries.mkdir()
    exe = binaries / "preset_config_test.exe"
    subprocess.run([
        r"C:\msys64\mingw32\bin\gcc.exe", "-m32", "-O2", "-static-libgcc",
        "-o", str(exe), str(root / "preset_config_test.c"),
        "-ld3d8", "-lgdi32", "-lopengl32",
    ], env=env, check=True)
    subprocess.run([str(exe)], cwd=temp, env=env, check=True)
