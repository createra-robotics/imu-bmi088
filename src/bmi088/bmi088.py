"""BMI088 sensor interface module."""

import time
from typing import Literal

import numpy as np
from ahrs.filters import EKF, Madgwick, Mahony, Tilt
from smbus3 import SMBus

from bmi088.params import (
    ACC_ADDRESS,
    ACC_ENABLE_ADDRESS,
    ACC_ON,
    ACC_POWER_CONF,
    ACC_SENSITIVITY_MAP,
    ACC_X_LSB,
    GYRO_ADDRESS,
    GYRO_RATE_X_LSB,
    GYRO_SENSITIVITY_MAP,
    TEMP_LSB,
    TEMP_MSB,
    AccBandwidth,
    AccRange,
    GyroBandwidth,
    GyroRange,
)
from bmi088.utils import quat_to_euler, quat_to_matrix

FilterType = Literal["madgwick", "mahony", "ekf", "tilt"]

G_STANDARD = 9.80665
WORLD_GRAVITY_UNIT = np.array([0.0, 0.0, 1.0])


class BMI088:
    """BMI088 sensor interface class."""

    def __init__(
        self,
        i2c_bus: int,
        acc_range: AccRange = AccRange.RANGE_3G,
        acc_bandwidth: AccBandwidth = AccBandwidth.BANDWIDTH_100,
        gyro_range: GyroRange = GyroRange.RANGE_500_DPS,
        gyro_bandwidth: GyroBandwidth = GyroBandwidth.BANDWIDTH_100,
        filter_type: FilterType = "madgwick",
    ):
        """Initialize the BMI088 sensor interface.

        Args:
            i2c_bus: I2C bus number.
            acc_range: Accelerometer range.
            acc_bandwidth: Accelerometer filter bandwidth.
            gyro_range: Gyroscope range.
            gyro_bandwidth: Gyroscope filter bandwidth.
            filter_type: AHRS algorithm for orientation fusion. One of
                "madgwick", "mahony", "ekf", "tilt".

        """
        self.bus = SMBus(i2c_bus)
        self.acc_range = acc_range
        self.acc_bandwidth = acc_bandwidth
        self.gyro_range = gyro_range
        self.gyro_bandwidth = gyro_bandwidth

        self.gyro_bias = np.zeros(3)
        self.acc_offset = np.zeros(3)
        self.acc_scale = np.ones(3)

        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.filter_type: FilterType = filter_type
        self._filter = self._make_filter(filter_type)

        self._start()

    @staticmethod
    def _make_filter(name: FilterType):
        if name == "madgwick":
            return Madgwick()
        if name == "mahony":
            return Mahony()
        if name == "ekf":
            return EKF()
        if name == "tilt":
            return Tilt()
        raise ValueError(
            f"Unknown filter_type: {name!r}. "
            "Expected one of: madgwick, mahony, ekf, tilt."
        )

    def _start(self) -> None:
        """Start the BMI088 sensor."""
        self.bus.write_byte_data(ACC_ADDRESS, 0x40, self.acc_bandwidth.value)
        self.bus.write_byte_data(ACC_ADDRESS, 0x41, self.acc_range.value)

        self.bus.write_byte_data(GYRO_ADDRESS, 0x0F, self.gyro_range.value)
        self.bus.write_byte_data(GYRO_ADDRESS, 0x10, self.gyro_bandwidth.value)

        self.bus.write_byte_data(ACC_ADDRESS, ACC_POWER_CONF, 0x00)
        self.bus.write_byte_data(ACC_ADDRESS, ACC_ENABLE_ADDRESS, ACC_ON)

    def _read_acc_g_raw(self) -> np.ndarray:
        """Read raw accelerometer data in g (no calibration applied)."""
        block = self.bus.read_i2c_block_data(ACC_ADDRESS, ACC_X_LSB, 6)
        raw = np.array(
            [
                np.int16((block[1] << 8) | block[0]),
                np.int16((block[3] << 8) | block[2]),
                np.int16((block[5] << 8) | block[4]),
            ],
            dtype=float,
        )
        return raw / ACC_SENSITIVITY_MAP[self.acc_range]

    def _read_gyro_rads_raw(self) -> np.ndarray:
        """Read raw gyroscope data in rad/s (no bias correction applied)."""
        block = self.bus.read_i2c_block_data(GYRO_ADDRESS, GYRO_RATE_X_LSB, 6)
        raw = np.array(
            [
                np.int16((block[1] << 8) | block[0]),
                np.int16((block[3] << 8) | block[2]),
                np.int16((block[5] << 8) | block[4]),
            ],
            dtype=float,
        )
        raw /= GYRO_SENSITIVITY_MAP[self.gyro_range]
        return np.deg2rad(raw)

    def read_accelerometer(self, m_per_s2: bool = False) -> np.ndarray:
        """Read calibrated acceleration data.

        Applies the configured offset/scale calibration. Output is in g by
        default.

        Args:
            m_per_s2: If True, returns acceleration in meters per second squared.

        Returns:
            ndarray of shape (3,) for X, Y, Z.

        """
        acc = (self._read_acc_g_raw() - self.acc_offset) * self.acc_scale
        if m_per_s2:
            acc = acc * G_STANDARD
        return acc

    def read_gyroscope(self, deg_per_s: bool = False) -> np.ndarray:
        """Read bias-corrected gyroscope data.

        Output is in rad/s by default.

        Args:
            deg_per_s: If True, returns angular velocity in degrees per second.

        Returns:
            ndarray of shape (3,) for X, Y, Z.

        """
        gyr = self._read_gyro_rads_raw() - self.gyro_bias
        if deg_per_s:
            gyr = np.rad2deg(gyr)
        return gyr

    def read_temperature(self) -> float:
        """Read temperature in degrees Celsius."""
        temp_lsb = self.bus.read_byte_data(ACC_ADDRESS, TEMP_LSB)
        temp_msb = self.bus.read_byte_data(ACC_ADDRESS, TEMP_MSB)

        _temp = (temp_msb << 3) | (temp_lsb >> 5)
        return _temp * 0.125 + 23

    def get_quat(
        self,
        dt: float | None = None,
        acc: np.ndarray | None = None,
        gyro: np.ndarray | None = None,
    ) -> np.ndarray:
        """Update and return the orientation quaternion (scalar-first).

        Args:
            dt: Time step in seconds since the previous update. Required for
                gyro-based filters (madgwick, mahony, ekf); ignored for tilt.
            acc: Optional pre-read accelerometer sample in g. Avoids an extra
                I2C read when caller already has it.
            gyro: Optional pre-read gyroscope sample in rad/s.

        Returns:
            ndarray of shape (4,) with components (w, x, y, z).

        """
        if acc is None:
            acc = self.read_accelerometer()

        if self.filter_type == "tilt":
            self.q = np.asarray(self._filter.estimate(acc), dtype=float)
            return self.q

        if gyro is None:
            gyro = self.read_gyroscope()

        if self.filter_type == "ekf":
            self.q = np.asarray(
                self._filter.update(self.q, gyr=gyro, acc=acc, dt=dt),
                dtype=float,
            )
        else:
            self.q = np.asarray(
                self._filter.updateIMU(self.q, gyr=gyro, acc=acc, dt=dt),
                dtype=float,
            )
        return self.q

    def get_euler(
        self,
        dt: float | None = None,
        degrees: bool = False,
    ) -> np.ndarray:
        """Return current orientation as ZYX Euler angles (roll, pitch, yaw).

        Args:
            dt: Time step for filter update.
            degrees: If True, returns degrees instead of radians.

        Returns:
            ndarray of shape (3,): roll (X), pitch (Y), yaw (Z).

        """
        return quat_to_euler(self.get_quat(dt), degrees=degrees)

    def get_rotation_matrix(self, dt: float | None = None) -> np.ndarray:
        """Return the body-to-world 3x3 rotation matrix from current orientation."""
        return quat_to_matrix(self.get_quat(dt))

    def get_tilt(self, degrees: bool = False) -> np.ndarray:
        """Estimate roll/pitch from accelerometer alone (no fusion).

        Assumes the sensor is approximately static. Yaw cannot be recovered
        without a magnetometer.

        Returns:
            ndarray of shape (2,): roll (X), pitch (Y).

        """
        ax, ay, az = self.read_accelerometer()
        roll = np.arctan2(ay, az)
        pitch = np.arctan2(-ax, np.sqrt(ay * ay + az * az))
        out = np.array([roll, pitch], dtype=float)
        return np.rad2deg(out) if degrees else out

    def get_gravity(self, dt: float | None = None) -> np.ndarray:
        """Return the gravity vector expressed in the body frame (unit g).

        Uses the current fused orientation to rotate world gravity into
        body coordinates.
        """
        rot = quat_to_matrix(self.get_quat(dt))
        return rot.T @ WORLD_GRAVITY_UNIT

    def read_linear_acceleration(
        self,
        dt: float | None = None,
        m_per_s2: bool = False,
    ) -> np.ndarray:
        """Read accelerometer with gravity subtracted, in body frame.

        Args:
            dt: Time step for orientation update.
            m_per_s2: If True, returns m/s² instead of g.

        Returns:
            ndarray of shape (3,): linear acceleration on X, Y, Z.

        """
        acc = self.read_accelerometer()
        gyro = self.read_gyroscope()
        q = self.get_quat(dt, acc=acc, gyro=gyro)
        gravity_body = quat_to_matrix(q).T @ WORLD_GRAVITY_UNIT
        linear = acc - gravity_body
        if m_per_s2:
            linear = linear * G_STANDARD
        return linear

    def is_free_fall(self, threshold_g: float = 0.3) -> bool:
        """Return True when the accelerometer magnitude indicates free fall.

        Args:
            threshold_g: Magnitude (in g) below which free fall is declared.
                Typical values are 0.2-0.4 g.

        """
        return float(np.linalg.norm(self.read_accelerometer())) < threshold_g

    def calibrate_gyro(
        self,
        samples: int = 200,
        delay: float = 0.005,
    ) -> np.ndarray:
        """Estimate and store gyroscope bias by averaging while still.

        The sensor MUST be stationary throughout. The returned bias (rad/s)
        is also stored on the instance and applied to subsequent reads.

        Args:
            samples: Number of samples to average.
            delay: Sleep between samples (seconds).

        Returns:
            ndarray of shape (3,): estimated bias in rad/s.

        """
        accum = np.zeros(3)
        for _ in range(samples):
            accum += self._read_gyro_rads_raw()
            time.sleep(delay)
        self.gyro_bias = accum / samples
        return self.gyro_bias

    def set_gyro_bias(self, bias: np.ndarray) -> None:
        """Set the gyroscope bias (rad/s) to subtract from future reads."""
        self.gyro_bias = np.asarray(bias, dtype=float).reshape(3)

    def set_acc_calibration(
        self,
        offset: np.ndarray,
        scale: np.ndarray,
    ) -> None:
        """Configure accelerometer offset and scale calibration.

        The calibrated reading is ``(raw_g - offset) * scale``.

        Args:
            offset: Per-axis offset in g (shape (3,)).
            scale: Per-axis scale factor (shape (3,)).

        """
        self.acc_offset = np.asarray(offset, dtype=float).reshape(3)
        self.acc_scale = np.asarray(scale, dtype=float).reshape(3)

    def reset_orientation(self) -> None:
        """Reset the fused orientation quaternion to identity."""
        self.q = np.array([1.0, 0.0, 0.0, 0.0])

    def close(self) -> None:
        """Close the I2C bus."""
        self.bus.close()

    def __enter__(self) -> "BMI088":
        """Enter context manager."""
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        """Close the bus on context exit."""
        self.close()
