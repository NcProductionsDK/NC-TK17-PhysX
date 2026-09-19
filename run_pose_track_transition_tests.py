"""Test production ownership handoff when PoseEditor replaces track data."""
from physx_test_runner import run_production_test
from pathlib import Path
import struct


def verify_native_reset_boundary():
    """Check the shipped executable, not a simulation of its command timing."""
    game = Path(__file__).resolve().parents[2] / "The Klub 17/Binaries/TK17-158.001.exe"
    data = game.read_bytes()
    pe = struct.unpack_from("<I", data, 0x3C)[0]
    optional = pe + 24
    image_base = struct.unpack_from("<I", data, optional + 28)[0]
    count = struct.unpack_from("<H", data, pe + 6)[0]
    sections = optional + struct.unpack_from("<H", data, pe + 20)[0]

    def read(va, size):
        rva = va - image_base
        for i in range(count):
            _, start, length, offset = struct.unpack_from("<IIII", data, sections + i * 40 + 8)
            if start <= rva and rva + size <= start + length:
                return data[offset + rva - start:offset + rva - start + size]
        raise AssertionError(f"Native address {va:#x} is outside the executable")

    def call_at(source, target):
        instruction = read(source, 5)
        assert instruction[0] == 0xE8, f"No native call at {source:#x}"
        assert source + 5 + struct.unpack_from("<i", instruction, 1)[0] == target

    assert read(0x4E7287, 10) == bytes.fromhex("c7 81 b8 01 00 00 01 00 00 00"), \
        "New-pose handler no longer queues PoseEdit+0x1b8"
    assert read(0x4DFF1F, 9) == bytes.fromhex("83 bf b8 01 00 00 00 74 1b"), \
        "Deferred update no longer tests the reset flag"
    call_at(0x4DFF2A, 0x4C8090)
    assert read(0x4DFF2F, 10) == bytes.fromhex("c7 87 b8 01 00 00 00 00 00 00"), \
        "Native update no longer clears the pending flag after reset"
    assert read(0x4E6E90, 6) == bytes.fromhex("55 8b ec 83 ec 30"), \
        "Native pose queue prologue changed"
    call_at(0x4EF499, 0x4E6E90)  # Pose-file loader uses the same queue as New.
    assert read(0x4C8090, 6) == bytes.fromhex("55 8b ec 83 ec 0c"), \
        "Reset hook's complete-instruction prologue changed"
    call_at(0x4C80AF, 0x4CE550)
    call_at(0x4C80C4, 0x4CC760)
    assert read(0x5FB200, 7) == bytes.fromhex("55 8b ec 51 8b 41 08"), \
        "Native situation-fade readiness prologue changed"
    assert read(0x6956E4, 8) == struct.pack("<II", 0x5FB200, 0x5FB1C0), \
        "Situation-change vtable no longer uses the verified fade readiness predicates"
    assert read(0x5FB240, 5) == bytes.fromhex("83 f8 01 75 5b"), \
        "Native fade predicate no longer checks BlendTo=1 before waiting for BlendCurrent"
    assert read(0x5FB28E, 11) == bytes.fromhex("da e9 df e0 f6 c4 44 7b 09 32 c0"), \
        "Native fade readiness comparison changed"
    assert read(0x5FB2A0, 9) == bytes.fromhex("b0 01 5e 8b e5 5d c2 08 00"), \
        "Native fade callback return/signature changed"
    assert read(0x4CE5E6, 7) == bytes.fromhex("89 84 37 98 00 00 00"), \
        "Native reset no longer clears the track key-array field"
    print("PASS: shipped executable queues New, resets in a later update, clears keys and evaluates tracks inside that reset", flush=True)

if __name__ == "__main__":
    verify_native_reset_boundary()
    run_production_test("pose_track_transition_test.c")
