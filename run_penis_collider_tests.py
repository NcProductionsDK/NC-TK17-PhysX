"""Production config, collision geometry and joint-space solver regressions."""
import os
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parent
env = os.environ.copy()
env["PATH"] = r"C:\msys64\mingw32\bin" + os.pathsep + env["PATH"]
with tempfile.TemporaryDirectory(prefix="penis-collider-", dir=root / "build") as temp:
    temp = Path(temp)
    exe = temp / "test.exe"
    subprocess.run([
        r"C:\msys64\mingw32\bin\gcc.exe", "-m32", "-O2", "-static-libgcc",
        "-o", str(exe), str(root / "penis_collider_test.c"),
        "-ld3d8", "-lgdi32", "-lopengl32",
    ], env=env, check=True)
    subprocess.run([str(exe), str(temp / "global.ini"), str(temp / "body.ini")],
                   cwd=root, env=env, check=True)
