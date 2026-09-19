"""Exercise Customizer isolation against populated colliders left by a room."""
from physx_test_runner import run_production_test

if __name__ == "__main__":
    run_production_test("customizer_collision_test.c")
