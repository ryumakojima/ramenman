"""Minimal ICM-20948 driver (accelerometer + gyroscope) over I2C for Raspberry Pi.

Outputs SI units, which is what ROS / OpenVINS / Kalibr expect:
  accel -> m/s^2
  gyro  -> rad/s
"""
import math
import time

from smbus2 import SMBus

# --- User bank 0 registers ---
WHO_AM_I = 0x00
PWR_MGMT_1 = 0x06
PWR_MGMT_2 = 0x07
ACCEL_XOUT_H = 0x2D  # 12 bytes from here: accel x,y,z then gyro x,y,z (big-endian int16)
REG_BANK_SEL = 0x7F

# --- User bank 2 registers ---
GYRO_SMPLRT_DIV = 0x00
GYRO_CONFIG_1 = 0x01
ACCEL_SMPLRT_DIV_1 = 0x10
ACCEL_SMPLRT_DIV_2 = 0x11
ACCEL_CONFIG = 0x14

WHO_AM_I_VALUE = 0xEA
GRAVITY = 9.80665

# range -> (register setting, LSB per unit)
ACCEL_RANGES = {2: (0, 16384.0), 4: (1, 8192.0), 8: (2, 4096.0), 16: (3, 2048.0)}  # LSB/g
GYRO_RANGES = {250: (0, 131.0), 500: (1, 65.5), 1000: (2, 32.8), 2000: (3, 16.4)}  # LSB/(deg/s)


class ICM20948:
    def __init__(self, bus=1, address=0x68, accel_range_g=4, gyro_range_dps=500):
        if accel_range_g not in ACCEL_RANGES:
            raise ValueError(f"accel_range_g must be one of {list(ACCEL_RANGES)}")
        if gyro_range_dps not in GYRO_RANGES:
            raise ValueError(f"gyro_range_dps must be one of {list(GYRO_RANGES)}")

        self.bus = SMBus(bus)
        self.addr = address
        self._bank = None

        self._select_bank(0)
        who = self.bus.read_byte_data(self.addr, WHO_AM_I)
        if who != WHO_AM_I_VALUE:
            raise RuntimeError(
                f"ICM-20948 not found at 0x{address:02X} (WHO_AM_I=0x{who:02X}, expected 0xEA). "
                "Check wiring, run `i2cdetect -y 1`, or try --imu-addr 0x69."
            )

        # Reset, then wake up with auto clock source
        self._write(PWR_MGMT_1, 0x80)
        time.sleep(0.1)
        self._bank = None  # reset puts the chip back in bank 0
        self._select_bank(0)
        self._write(PWR_MGMT_1, 0x01)
        time.sleep(0.05)
        self._write(PWR_MGMT_2, 0x00)  # enable all accel + gyro axes

        a_sel, self.accel_lsb_per_g = ACCEL_RANGES[accel_range_g]
        g_sel, self.gyro_lsb_per_dps = GYRO_RANGES[gyro_range_dps]

        self._select_bank(2)
        self._write(GYRO_SMPLRT_DIV, 0)  # 1125 Hz internal rate
        self._write(GYRO_CONFIG_1, (1 << 3) | (g_sel << 1) | 1)  # range + low-pass filter on
        self._write(ACCEL_SMPLRT_DIV_1, 0)
        self._write(ACCEL_SMPLRT_DIV_2, 0)  # 1125 Hz internal rate
        self._write(ACCEL_CONFIG, (1 << 3) | (a_sel << 1) | 1)  # range + low-pass filter on
        self._select_bank(0)
        time.sleep(0.05)

    def read(self):
        """Return (accel_xyz in m/s^2, gyro_xyz in rad/s)."""
        d = self.bus.read_i2c_block_data(self.addr, ACCEL_XOUT_H, 12)
        raw = [self._int16(d[i], d[i + 1]) for i in range(0, 12, 2)]
        accel = tuple(v / self.accel_lsb_per_g * GRAVITY for v in raw[:3])
        gyro = tuple(math.radians(v / self.gyro_lsb_per_dps) for v in raw[3:])
        return accel, gyro

    def close(self):
        self.bus.close()

    # --- helpers ---
    def _select_bank(self, bank):
        if self._bank != bank:
            self.bus.write_byte_data(self.addr, REG_BANK_SEL, bank << 4)
            self._bank = bank

    def _write(self, reg, value):
        self.bus.write_byte_data(self.addr, reg, value)

    @staticmethod
    def _int16(hi, lo):
        v = (hi << 8) | lo
        return v - 65536 if v & 0x8000 else v
