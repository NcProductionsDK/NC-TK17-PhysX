"""Exercise the production Customizer fade and retained-command lifecycle."""
from physx_test_runner import run_production_test
from pathlib import Path
import struct


def verify_native_layout():
    data = (Path(__file__).resolve().parents[2] /
            "The Klub 17/Binaries/ThriXXX010278-SYS.dll").read_bytes()
    pe = struct.unpack_from("<I", data, 0x3c)[0]
    optional = pe + 24
    base = struct.unpack_from("<I", data, optional + 28)[0]
    sections = optional + struct.unpack_from("<H", data, pe + 20)[0]

    def read(rva, size):
        for i in range(struct.unpack_from("<H", data, pe + 6)[0]):
            _, start, length, offset = struct.unpack_from("<IIII", data, sections + i * 40 + 8)
            if start <= rva and rva + size <= start + length:
                return data[offset + rva - start:offset + rva - start + size]
        raise AssertionError(hex(rva))

    assert read(0x4550, 9) == bytes.fromhex("c7 01 00 00 00 00 8b c1 c3")
    assert read(0x4590, 16) == bytes.fromhex("55 8b ec 56 57 8b 7d 08 8b f1 8b 07 85 c0 74 0a")
    assert read(0x45a0, 16) == bytes.fromhex("83 c0 f0 50 ff 15 80 30 15 10 8b 07 89 06 5f 8b")
    assert read(0x4585, 2) == bytes.fromhex("5e c3")
    assert read(0x16a310, 4) == struct.pack("<I", base + 0xdac70)
    assert read(0xdac76, 8) == bytes.fromhex("8b 46 5c 83 f8 01 75 08")
    assert read(0xdac8a, 9) == bytes.fromhex("8b 46 08 8d 4e 08 ff 50 04")
    assert read(0xdac99, 3) == bytes.fromhex("8d 46 18")
    assert read(0xe93d0, 7) == bytes.fromhex("8b 81 fc 00 00 00 c3")
    assert read(0xe94b0, 12) == bytes.fromhex("89 9f fc 00 00 00 5f 5b 5d c2 04 00")
    print("PASS: native NameHash ownership and STransform clean/local-matrix layout match the shipped engine", flush=True)

if __name__ == "__main__":
    verify_native_layout()
    run_production_test("customizer_transition_test.c")
