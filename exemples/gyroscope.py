"""Example script to read gyroscope data from BMI088 sensor."""

import time

from bmi088 import BMI088

imu = BMI088(i2c_bus=1)

while True:
    gyroscope = imu.read_gyroscope()
    print(f"Gyroscope: X={gyroscope[0]}, Y={gyroscope[1]}, Z={gyroscope[2]}")
    time.sleep(0.1)
