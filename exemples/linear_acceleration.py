"""Example: calibrate gyro bias then read gravity-compensated linear acceleration."""

import time

from bmi088 import BMI088

with BMI088(i2c_bus=1) as imu:
    print("Hold the sensor still — calibrating gyro bias...")
    bias = imu.calibrate_gyro(samples=200)
    print(f"Gyro bias (rad/s): {bias}")

    last = time.monotonic()
    while True:
        now = time.monotonic()
        dt = now - last
        last = now

        lin = imu.read_linear_acceleration(dt, m_per_s2=True)
        print(f"Linear acc (m/s²): X={lin[0]:7.3f} Y={lin[1]:7.3f} Z={lin[2]:7.3f}")

        if imu.is_free_fall():
            print("  >> free fall detected")

        time.sleep(0.05)
