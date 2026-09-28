"""Real Windows loader regression: never load a game DLL into the fixture."""
from pathlib import Path
import os
import subprocess
import tempfile

root=Path(__file__).resolve().parent
env=dict(os.environ)
env['PATH']=r'C:\msys64\mingw32\bin'+os.pathsep+env['PATH']
gcc=r'C:\msys64\mingw32\bin\gcc.exe'
flags=['-m32','-O2','-Wall','-Wextra','-Werror','-static-libgcc']
with tempfile.TemporaryDirectory(prefix='module-lifetime-',dir=root/'build') as temp:
    folder=Path(temp)
    dll=folder/'module-lifetime-fixture.dll'
    exe=folder/'module-lifetime-test.exe'
    subprocess.run([gcc,*flags,'-shared','-o',str(dll),str(root/'module_lifetime_fixture.c')],env=env,check=True)
    subprocess.run([gcc,*flags,'-o',str(exe),str(root/'module_lifetime_test.c')],env=env,check=True)
    for pin in ('0','1'):
        subprocess.run([str(exe),str(dll),pin],cwd=folder,env=env,check=True,timeout=15)

# Both extension-loader entry points must protect lifetime before any hook work.
source=(root/'NC-TK17-PhysX.c').read_text()
for entry in ('loadextension','on_create'):
    body=source.split(f'__declspec(dllexport) int {entry}(void)',1)[1].split('}',1)[0]
    assert body.split('{',1)[1].lstrip().startswith('if (!physx_require_hook_lifetime()) return 0;')
print('PASS: both production initialization entry points gate hook installation')
