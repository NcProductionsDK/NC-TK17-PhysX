"""Package the existing build without rebuilding, installing, or publishing it."""
from hashlib import sha256
from pathlib import Path
import struct
import zipfile

root = Path(__file__).resolve().parent
files = {
    'Binaries/NC-TK17-PhysX.dll': root / 'build/NC-TK17-PhysX.dll',
    'README.md': root / 'RELEASE.md',
    'CHANGELOG.md': root / 'CHANGELOG.md',
    'Config.default.ini': root / 'Config.default.ini',
    'docs/GRAVITY-SAMPLING.md': root / 'docs/GRAVITY-SAMPLING.md',
    'docs/SINGLE-BONE-CONTACT.md': root / 'docs/SINGLE-BONE-CONTACT.md',
    'docs/BODY-COLLISION-CAMERA.md': root / 'docs/BODY-COLLISION-CAMERA.md',
}
payload = {name: path.read_bytes() for name, path in files.items()}
dll = payload['Binaries/NC-TK17-PhysX.dll']
if len(dll) < 64 or dll[:2] != b'MZ':
    raise SystemExit('Build is not a Windows executable')
pe = struct.unpack_from('<I', dll, 0x3C)[0]
if pe + 24 > len(dll) or dll[pe:pe + 4] != b'PE\0\0':
    raise SystemExit('Build has an invalid PE header')
machine = struct.unpack_from('<H', dll, pe + 4)[0]
characteristics = struct.unpack_from('<H', dll, pe + 22)[0]
if machine != 0x14C or not characteristics & 0x2000:
    raise SystemExit('Build must be a 32-bit x86 DLL')
manifest = ''.join(f'{sha256(data).hexdigest()}  {name}\n'
                   for name, data in payload.items())
payload['SHA256SUMS.txt'] = manifest.encode('utf-8')
archive = root / 'build/NC-TK17-PhysX-collision-checkpoint.zip'
with zipfile.ZipFile(archive, 'w', compression=zipfile.ZIP_DEFLATED) as package:
    for name, data in payload.items():
        # Fixed archive metadata makes identical inputs produce identical ZIPs.
        info = zipfile.ZipInfo(name, date_time=(2026, 9, 8, 0, 0, 0))
        info.compress_type = zipfile.ZIP_DEFLATED
        info.external_attr = 0o100644 << 16
        package.writestr(info, data)
with zipfile.ZipFile(archive) as package:
    if package.testzip() is not None or set(package.namelist()) != set(payload):
        raise SystemExit('Archive verification failed')
    for name, data in payload.items():
        if package.read(name) != data:
            raise SystemExit(f'Archive contents differ: {name}')
checksum = sha256(archive.read_bytes()).hexdigest()
archive.with_suffix('.zip.sha256').write_text(
    f'{checksum}  {archive.name}\n', encoding='ascii')
print(f'Verified package: {archive}')
print(f'DLL SHA256: {sha256(dll).hexdigest()}')
print(f'ZIP SHA256: {checksum}')
