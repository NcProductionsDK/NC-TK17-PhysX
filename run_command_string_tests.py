"""Test command field decoding against the installed TK17 native string routines."""
from physx_test_runner import run_production_test

if __name__ == "__main__":
    run_production_test("command_string_test.c")
