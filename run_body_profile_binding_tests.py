"""Exercise production body profile events using temporary add-ons, without TK17."""
from pathlib import Path
import os
import subprocess
import tempfile

root = Path(__file__).resolve().parent
build = root / "build"
build.mkdir(exist_ok=True)
env = os.environ.copy()
env["PATH"] = r"C:\msys64\mingw32\bin" + os.pathsep + env["PATH"]
exe = build / "body_profile_binding_test.exe"
subprocess.run([
    r"C:\msys64\mingw32\bin\gcc.exe", "-m32", "-O2", "-static-libgcc",
    "-o", str(exe), str(root / "body_profile_binding_test.c"),
    "-ld3d8", "-lgdi32", "-lopengl32",
], env=env, check=True)
with tempfile.TemporaryDirectory(prefix="body-profile-", dir=build) as temp:
    addons = Path(temp) / "Addons"
    for name, contents in [("Body.A", b""), ("Body.B", b"\x00\xffopaque"), ("Body.Plain", b"")]:
        directory = addons / name / "Scenes" / "Shared" / "Body"
        directory.mkdir(parents=True)
        (directory / "body02.bs").write_bytes(contents)
        if name != "Body.Plain":
            (directory / "body02.physx.ini").write_text(
                "[breasts_physics]\nenabled=" + ("false" if name == "Body.A" else "true") + "\n"
            )
    subprocess.run([str(exe), str(addons)], cwd=root, env=env, check=True)
