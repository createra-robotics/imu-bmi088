"""Example script to read acceleration data from BMI088 sensor."""

import time

from bmi088 import BMI088

imu = BMI088(i2c_bus=1)

while True:
    acceleration = imu.read_accelerometer()
    print(
        f"Acceleration: X={acceleration[0]}, Y={acceleration[1]}, Z={acceleration[2]}"
    )
    time.sleep(0.1)
