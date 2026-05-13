"""Example: read fused orientation as quaternion, Euler angles, and matrix."""

import time

from bmi088 import BMI088

with BMI088(i2c_bus=1, filter_type="madgwick") as imu:
    last = time.monotonic()
    while True:
        now = time.monotonic()
        dt = now - last
        last = now

        roll, pitch, yaw = imu.get_euler(dt, degrees=True)
        print(f"Roll={roll:7.2f}  Pitch={pitch:7.2f}  Yaw={yaw:7.2f}  (deg)")

        time.sleep(0.05)
