"""Example: poll the BMI088 in a background thread for low-latency access."""

import time

from bmi088 import BMI088, BMI088Streamer

with BMI088(i2c_bus=1) as imu:
    with BMI088Streamer(imu, rate_hz=200) as stream:
        for _ in range(100):
            acc = stream.acceleration
            quat = stream.quaternion
            print(f"acc={acc}  quat={quat}")
            time.sleep(0.1)
