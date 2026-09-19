"""Exercise wind/room retirement and shutdown without live engine objects."""
from physx_test_runner import run_production_test

if __name__ == "__main__":
    run_production_test("scene_lifecycle_test.c")
