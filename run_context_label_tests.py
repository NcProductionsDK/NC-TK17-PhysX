"""Verify the installed engine's Label ID and production string dispatch."""
from pathlib import Path
import struct
from physx_test_runner import run_production_test

if __name__ == "__main__":
    binary = (Path(__file__).resolve().parents[2] /
              "The Klub 17/Binaries/ThriXXX010278-SYS.dll").read_bytes()
    pe = struct.unpack_from("<I", binary, 0x3c)[0]
    optional = pe + 24
    base = struct.unpack_from("<I", binary, optional + 28)[0]
    sections = optional + struct.unpack_from("<H", binary, pe + 20)[0]
    verified = False
    for i in range(struct.unpack_from("<H", binary, pe + 6)[0]):
        _, rva, size, offset = struct.unpack_from("<IIII", binary, sections+i*40+8)
        pos = binary.find(b"\0Label\0", offset, offset+size)
        if pos >= 0:
            address = base + rva + pos + 1 - offset
            registration = b"\x68" + struct.pack("<I", address) + b"\x68" + struct.pack("<I", 0x03fff0eb)
            verified |= registration in binary
    assert verified, "installed engine's Label property registration changed"
    print("PASS: shipped engine registers Label as 0x03fff0eb", flush=True)
    run_production_test("context_label_test.c")
