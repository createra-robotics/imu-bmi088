"""Background thread that polls a BMI088 and caches latest values."""

import threading
import time

import numpy as np

from bmi088.bmi088 import BMI088


class BMI088Streamer:
    """Continuously read a BMI088 in a daemon thread.

    Use in control loops where you need the latest sample without paying
    the I2C latency on every call. The streamer takes ownership of the
    sensor during ``start()``..``stop()``; do not call read methods on the
    underlying ``BMI088`` while the thread is active.

    Example:
        >>> with BMI088(i2c_bus=4) as imu:
        ...     streamer = BMI088Streamer(imu, rate_hz=200)
        ...     streamer.start()
        ...     try:
        ...         while running:
        ...             acc = streamer.acceleration
        ...             quat = streamer.quaternion
        ...             ...
        ...     finally:
        ...         streamer.stop()

    """

    def __init__(
        self,
        imu: BMI088,
        rate_hz: float = 100.0,
        compute_orientation: bool = True,
    ):
        """Initialize the streamer.

        Args:
            imu: A configured ``BMI088`` instance.
            rate_hz: Target sampling rate in Hz.
            compute_orientation: If True, runs the AHRS filter each cycle and
                caches the latest quaternion.

        """
        if rate_hz <= 0:
            raise ValueError("rate_hz must be positive")
        self.imu = imu
        self._period = 1.0 / rate_hz
        self._compute_orientation = compute_orientation

        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()

        self._acc = np.zeros(3)
        self._gyro = np.zeros(3)
        self._quat = np.array([1.0, 0.0, 0.0, 0.0])
        self._timestamp = 0.0

    def start(self) -> None:
        """Start the background polling thread."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self, timeout: float | None = 1.0) -> None:
        """Signal the thread to exit and wait for it to join."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)
            self._thread = None

    def _loop(self) -> None:
        last = time.monotonic()
        while not self._stop_event.is_set():
            cycle_start = time.monotonic()
            dt = cycle_start - last
            last = cycle_start

            acc = self.imu.read_accelerometer()
            gyro = self.imu.read_gyroscope()
            quat = (
                self.imu.get_quat(dt, acc=acc, gyro=gyro)
                if self._compute_orientation
                else self._quat
            )

            with self._lock:
                self._acc = acc
                self._gyro = gyro
                self._quat = quat
                self._timestamp = cycle_start

            elapsed = time.monotonic() - cycle_start
            remaining = self._period - elapsed
            if remaining > 0:
                self._stop_event.wait(remaining)

    @property
    def acceleration(self) -> np.ndarray:
        """Latest accelerometer reading in g."""
        with self._lock:
            return self._acc.copy()

    @property
    def gyroscope(self) -> np.ndarray:
        """Latest gyroscope reading in rad/s."""
        with self._lock:
            return self._gyro.copy()

    @property
    def quaternion(self) -> np.ndarray:
        """Latest fused orientation quaternion (w, x, y, z)."""
        with self._lock:
            return self._quat.copy()

    @property
    def timestamp(self) -> float:
        """``time.monotonic()`` timestamp of the most recent sample."""
        with self._lock:
            return self._timestamp

    def __enter__(self) -> "BMI088Streamer":
        """Start the thread on context entry."""
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        """Stop the thread on context exit."""
        self.stop()
