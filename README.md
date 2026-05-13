# BMI088 IMU library

A Python library for the Bosch BMI088 IMU over I2C, with built-in AHRS sensor
fusion and ready-to-use helpers for robotics development.

## Installation

From PyPI:

```bash
pip install imu-bmi088
```

From source:

```bash
pip install -e .
```

## Quick start

```python
from bmi088 import BMI088

with BMI088(i2c_bus=1) as imu:
    acc  = imu.read_accelerometer()        # ndarray (3,) in g
    gyro = imu.read_gyroscope()            # ndarray (3,) in rad/s
    quat = imu.get_quat(dt=0.01)           # ndarray (4,) (w, x, y, z)
    roll, pitch, yaw = imu.get_euler(dt=0.01, degrees=True)
```

## Features

- Raw accelerometer / gyroscope / temperature reads
- Orientation fusion via Madgwick, Mahony, EKF, or Tilt (selectable)
- Output as quaternion, ZYX Euler angles, or 3×3 rotation matrix
- Gyroscope bias auto-calibration; accelerometer offset/scale calibration
- Gravity-compensated linear acceleration
- Tilt (roll/pitch) directly from accelerometer
- Free-fall detection
- Background-thread streamer for low-latency control loops
- Context-manager support (`with BMI088(...) as imu:`)

## Configuration

```python
from bmi088 import BMI088, AccRange, AccBandwidth, GyroRange, GyroBandwidth

imu = BMI088(
    i2c_bus=1,
    acc_range=AccRange.RANGE_6G,
    acc_bandwidth=AccBandwidth.BANDWIDTH_100,
    gyro_range=GyroRange.RANGE_500_DPS,
    gyro_bandwidth=GyroBandwidth.BANDWIDTH_100,
    filter_type="madgwick",   # "madgwick" | "mahony" | "ekf" | "tilt"
)
```

## Reading raw data

```python
acc  = imu.read_accelerometer(m_per_s2=False)   # default: g
gyro = imu.read_gyroscope(deg_per_s=False)      # default: rad/s
temp = imu.read_temperature()                   # °C
```

Returned arrays are NumPy `ndarray` of shape `(3,)` and are indexable like
tuples (`acc[0]`, `acc[1]`, `acc[2]`).

## Orientation

`dt` is the time step in seconds since the previous update.

```python
quat   = imu.get_quat(dt)                       # (w, x, y, z)
euler  = imu.get_euler(dt, degrees=True)        # (roll, pitch, yaw)
matrix = imu.get_rotation_matrix(dt)            # 3x3 body-to-world
imu.reset_orientation()                         # reset fused quaternion to identity
```

For a quick gravity-only estimate (no gyro, no yaw):

```python
roll, pitch = imu.get_tilt(degrees=True)
```

## Calibration

Gyroscope bias — keep the sensor still and call:

```python
bias = imu.calibrate_gyro(samples=200)          # rad/s, stored on the instance
```

Bias is subtracted automatically on every `read_gyroscope()`.

To apply pre-computed values:

```python
imu.set_gyro_bias([bx, by, bz])                 # rad/s
imu.set_acc_calibration(offset=[ox, oy, oz],    # g
                        scale=[sx, sy, sz])
```

Calibrated accelerometer reading is `(raw_g - offset) * scale`.

## Robotics derivatives

```python
gravity_body = imu.get_gravity(dt)              # gravity unit vector in body frame
linear_acc   = imu.read_linear_acceleration(dt, m_per_s2=True)  # gravity-removed
falling      = imu.is_free_fall(threshold_g=0.3)
```

## Background streamer

Poll the IMU in a daemon thread and read the latest cached sample without
blocking your control loop:

```python
from bmi088 import BMI088, BMI088Streamer

with BMI088(i2c_bus=1) as imu, BMI088Streamer(imu, rate_hz=200) as stream:
    while running:
        acc  = stream.acceleration              # latest sample, thread-safe
        gyro = stream.gyroscope
        quat = stream.quaternion
        ...
```

Do not call read methods directly on the `BMI088` instance while the streamer
is running.

## Quaternion utilities

Pure-math helpers (no hardware needed). All quaternions use scalar-first
convention `q = (w, x, y, z)`.

```python
from bmi088 import (
    quat_to_euler,
    quat_to_matrix,
    quat_normalize,
    quat_conjugate,
    rotate_vector,
)
```

## Examples

See the `exemples/` directory:

- `accelerometer.py` — raw acceleration
- `gyroscope.py` — raw angular velocity
- `quaternion.py` — fused orientation as a quaternion
- `orientation.py` — fused orientation as Euler angles
- `linear_acceleration.py` — gyro calibration + gravity-compensated acceleration + free-fall detection
- `streamer.py` — background-thread streaming

## API reference

### `BMI088`

- `read_accelerometer(m_per_s2=False)` — calibrated acceleration, g or m/s²
- `read_gyroscope(deg_per_s=False)` — bias-corrected angular velocity, rad/s or deg/s
- `read_temperature()` — temperature in °C
- `get_quat(dt, acc=None, gyro=None)` — update and return fused quaternion
- `get_euler(dt, degrees=False)` — roll, pitch, yaw
- `get_rotation_matrix(dt)` — 3×3 body-to-world matrix
- `get_tilt(degrees=False)` — roll, pitch from accelerometer alone
- `get_gravity(dt)` — gravity unit vector in body frame
- `read_linear_acceleration(dt, m_per_s2=False)` — gravity-compensated acceleration
- `is_free_fall(threshold_g=0.3)` — free-fall detector
- `calibrate_gyro(samples=200, delay=0.005)` — estimate and store gyro bias
- `set_gyro_bias(bias)` — apply external gyro bias
- `set_acc_calibration(offset, scale)` — apply external accelerometer calibration
- `reset_orientation()` — reset fused quaternion to identity
- `close()` / `__exit__` — release the I2C bus

### `BMI088Streamer`

- `start()` / `stop(timeout=1.0)` — manage the polling thread
- `acceleration` — latest acceleration (g)
- `gyroscope` — latest angular velocity (rad/s)
- `quaternion` — latest fused quaternion (if enabled)
- `timestamp` — `time.monotonic()` of the latest sample
