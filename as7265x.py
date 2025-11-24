# AS7265X Python Driver for Raspberry Pi
# Full 18-channel SparkFun AS7265x Spectral Triad support
# Works with AS72651 (master), AS72652, AS72653
# Author: ChatGPT (based on SparkFun Arduino driver spec)
# License: MIT

import time
import smbus

# Main I2C address for AS72651 master
AS7265X_ADDR = 0x49

# Register addresses (from SparkFun datasheet)
AS726x_HW_VERSION_HIGH = 0x00
AS726x_HW_VERSION_LOW  = 0x01

AS726x_FW_VERSION_HIGH = 0x02
AS726x_FW_VERSION_LOW  = 0x03

AS726x_CONTROL_SETUP   = 0x04
AS726x_INT_T           = 0x05

AS726x_DEVICE_TEMP     = 0x06
AS726x_LED_CONTROL     = 0x07

# Virtual register interface
AS726x_SLAVE_STATUS_REG    = 0x00
AS726x_SLAVE_WRITE_REG     = 0x01
AS726x_SLAVE_READ_REG      = 0x02

AS726x_TX_VALID = 0x02
AS726x_RX_VALID = 0x01

# Virtual registers for sensors A/B/C
AS7265x_DEV_SELECT_CONTROL = 0x4F  # Device select (A=0,B=1,C=2)
AS7265x_DEVREG = [0, 1, 2]         # convenience enum

# 18-channel data registers (6 per sensor)
AS7265x_RAW_REGISTERS = [
    0x08, 0x0A, 0x0C, 0x0E, 0x10, 0x12,   # A (410-535 nm)
    0x08, 0x0A, 0x0C, 0x0E, 0x10, 0x12,   # B (560-705 nm)
    0x08, 0x0A, 0x0C, 0x0E, 0x10, 0x12    # C (730-940 nm)
]

AS7265x_CAL_REGISTERS = [
    0x14, 0x18, 0x1C, 0x20, 0x24, 0x28,   # A
    0x14, 0x18, 0x1C, 0x20, 0x24, 0x28,   # B
    0x14, 0x18, 0x1C, 0x20, 0x24, 0x28    # C
]

# Wavelength mapping
AS7265x_WAVELENGTHS = [
    410, 435, 460, 485, 510, 535,
    560, 585, 610, 645, 680, 705,
    730, 760, 810, 860, 900, 940
]

class AS7265X:
    """Low-level SparkFun-compatible AS7265x driver."""

    def __init__(self, i2c_bus=1):
        self.bus = smbus.SMBus(i2c_bus)
        time.sleep(0.1)

        # Check hardware version quickly
        try:
            hx = self._read_hw_version()
        except:
            raise RuntimeError("AS7265x not responding on I2C. Check wiring.")

        print(f"[AS7265X] Sensor detected. HW version: {hx}")
  # ----------------------------------------------------
    # -------- LOW-LEVEL VIRTUAL REGISTER ACCESS ---------
    # ----------------------------------------------------

    def _virtual_write(self, reg, value):
        """Write to AS7265x virtual register"""
        # Wait for TX to be free
        while True:
            status = self.bus.read_byte_data(AS7265X_ADDR, AS726x_SLAVE_STATUS_REG)
            if (status & AS726x_TX_VALID) == 0:
                break
            time.sleep(0.001)

        # Write register address
        self.bus.write_byte_data(AS7265X_ADDR, AS726x_SLAVE_WRITE_REG, reg)
        time.sleep(0.001)

        # Wait for TX to be free
        while True:
            status = self.bus.read_byte_data(AS7265X_ADDR, AS726x_SLAVE_STATUS_REG)
            if (status & AS726x_TX_VALID) == 0:
                break
            time.sleep(0.001)

        # Write the value
        self.bus.write_byte_data(AS7265X_ADDR, AS726x_SLAVE_WRITE_REG, value)
        time.sleep(0.001)

    def _virtual_read(self, reg):
        """Read from AS7265x virtual register"""
        # Wait for TX valid to clear
        while True:
            status = self.bus.read_byte_data(AS7265X_ADDR, AS726x_SLAVE_STATUS_REG)
            if (status & AS726x_TX_VALID) == 0:
                break
            time.sleep(0.001)

        # Write register address
        self.bus.write_byte_data(AS7265X_ADDR, AS726x_SLAVE_WRITE_REG, reg)
        time.sleep(0.001)

        # Wait for RX valid
        while True:
            status = self.bus.read_byte_data(AS7265X_ADDR, AS726x_SLAVE_STATUS_REG)
            if (status & AS726x_RX_VALID) != 0:
                break
            time.sleep(0.001)

        # Read value
        val = self.bus.read_byte_data(AS7265X_ADDR, AS726x_SLAVE_READ_REG)
        return val
   # ----------------------------------------------------
    # -------- BASIC DEVICE INFORMATION HELPERS ----------
    # ----------------------------------------------------

    def _read_hw_version(self):
        high = self._virtual_read(AS726x_HW_VERSION_HIGH)
        low  = self._virtual_read(AS726x_HW_VERSION_LOW)
        return (high << 8) | low
 # ----------------------------------------------------
    # -------- DEVICE SELECT (A = 0, B = 1, C = 2) --------
    # ----------------------------------------------------

    def _select_device(self, dev):
        """
        Select which sensor:
            0 = AS72651 (Visible)
            1 = AS72652 (NIR)
            2 = AS72653 (UV/extra channels)
        """
        if dev not in [0, 1, 2]:
            raise ValueError("Device must be 0, 1, or 2")

        self._virtual_write(AS7265x_DEV_SELECT_CONTROL, dev)
        time.sleep(0.002)

    # ----------------------------------------------------
    # ----------- MEASUREMENT / CONVERSION MODE ----------
    # ----------------------------------------------------

    def set_measurement_mode(self, mode):
        """
        mode:
            0 = MODE_0: bank A only
            1 = MODE_1: bank B only
            2 = MODE_2: bank C only
            3 = ONE_SHOT: measure all 3 sensors once
        For full triad readings, you normally use ONE_SHOT (3).
        """
        if mode not in [0, 1, 2, 3]:
            raise ValueError("Mode must be 0?3")

        ctrl = self._virtual_read(AS726x_CONTROL_SETUP)
        ctrl = (ctrl & 0b11001111) | (mode << 4)
        self._virtual_write(AS726x_CONTROL_SETUP, ctrl)
        time.sleep(0.002)

    # ----------------------------------------------------
    # ---------------- INTEGRATION TIME ------------------
    # ----------------------------------------------------

    def set_integration_time(self, ms):
        """
        Integration time in milliseconds.
        Valid range: 0?255 (each count = 2.8ms)
        """
        val = int(ms / 2.8)
        if val < 0: val = 0
        if val > 255: val = 255
        self._virtual_write(AS726x_INT_T, val)
        time.sleep(0.002)

    # ----------------------------------------------------
    # ----------------------- GAIN ------------------------
    # ----------------------------------------------------

    def set_gain(self, gain):
        """
        gain:
            0 = 1x
            1 = 3.7x
            2 = 16x
            3 = 64x
        """
        if gain not in [0, 1, 2, 3]:
            raise ValueError("Gain must be 0?3")

        ctrl = self._virtual_read(AS726x_CONTROL_SETUP)
        ctrl = (ctrl & 0b11110011) | (gain << 2)
        self._virtual_write(AS726x_CONTROL_SETUP, ctrl)
        time.sleep(0.002)
  # ----------------------------------------------------
    # ------------------- LED CONTROL --------------------
    # ----------------------------------------------------

    def enable_indicator_led(self, enable=True):
        val = self._virtual_read(AS726x_LED_CONTROL)
        if enable:
            val |= 0b00000001
        else:
            val &= ~0b00000001
        self._virtual_write(AS726x_LED_CONTROL, val)

    def enable_driver_led(self, enable=True):
        val = self._virtual_read(AS726x_LED_CONTROL)
        if enable:
            val |= 0b00010000
        else:
            val &= ~0b00010000
        self._virtual_write(AS726x_LED_CONTROL, val)

    def set_indicator_current(self, level):
        """
        0 = 1mA
        1 = 2mA
        2 = 4mA
        3 = 8mA
        """
        if level not in [0,1,2,3]:
            raise ValueError("Indicator current 0?3")
        val = self._virtual_read(AS726x_LED_CONTROL)
        val = (val & 0b11111100) | level
        self._virtual_write(AS726x_LED_CONTROL, val)

    def set_driver_current(self, level):
        """
        0 = 12.5mA
        1 = 25mA
        2 = 50mA
        3 = 100mA
        """
        if level not in [0,1,2,3]:
            raise ValueError("Driver current 0?3")
        val = self._virtual_read(AS726x_LED_CONTROL)
        val = (val & 0b11001111) | (level << 4)
        self._virtual_write(AS726x_LED_CONTROL, val)

    # ----------------------------------------------------
    # ---------------- DEVICE TEMPERATURE ----------------
    # ----------------------------------------------------

    def get_temperature(self):
        return self._virtual_read(AS726x_DEVICE_TEMP)
 # ----------------------------------------------------
    # ----------- INTERNAL RAW/CAL VALUE READERS ----------
    # ----------------------------------------------------

    def _read_raw_value(self, reg):
        """Read 16-bit raw value from virtual reg pair."""
        high = self._virtual_read(reg)
        low  = self._virtual_read(reg + 1)
        return (high << 8) | low

    def _read_calibrated_value(self, reg):
        """Read 32-bit IEEE float from 4 virtual registers."""
        b0 = self._virtual_read(reg)
        b1 = self._virtual_read(reg + 1)
        b2 = self._virtual_read(reg + 2)
        b3 = self._virtual_read(reg + 3)
        # Convert bytes ? float32
        import struct
        return struct.unpack('>f', bytes([b0, b1, b2, b3]))[0]

    # ----------------------------------------------------
    # ----------- READ 6 CHANNELS PER DEVICE -------------
    # ----------------------------------------------------

    def _read_device_raw(self, dev):
        """
        Read 6 raw channels for device:
          dev = 0 ? AS72651 (410?535nm)
          dev = 1 ? AS72652 (560?705nm)
          dev = 2 ? AS72653 (730?940nm)
        Returns: list of 6 ints
        """
        self._select_device(dev)
        out = []
        offsets = [0, 6, 12]  # offset per device
        base = offsets[dev]

        for i in range(6):
            reg = AS7265x_RAW_REGISTERS[base + i]
            out.append(self._read_raw_value(reg))

        return out

    def _read_device_calibrated(self, dev):
        """
        Read 6 calibrated float channels for device.
        Returns: list of 6 floats.
        """
        self._select_device(dev)
        out = []
        offsets = [0, 6, 12]
        base = offsets[dev]

        for i in range(6):
            reg = AS7265x_CAL_REGISTERS[base + i]
            out.append(self._read_calibrated_value(reg))

        return out
  # ----------------------------------------------------
    # ----------- ONE-SHOT 18-CHANNEL MEASUREMENT --------
    # ----------------------------------------------------

    def take_one_shot(self):
        """
        Performs a full AS7265x triad measurement.
        Returns a dict:
        {
            "raw": [18 raw ints],
            "cal": [18 floats],
            "wavelengths": [...],
            "temperature": x
        }
        """
        # Set ONE_SHOT mode
        self.set_measurement_mode(3)

        # Trigger via setting data_ready flag
        # Write ONE_SHOT bit
        ctrl = self._virtual_read(AS726x_CONTROL_SETUP)
        ctrl |= 0b00001000  # start measurement
        self._virtual_write(AS726x_CONTROL_SETUP, ctrl)

        # Wait until data ready (bit clears)
        timeout = time.time() + 3.0
        while True:
            status = self._virtual_read(AS726x_CONTROL_SETUP)
            if (status & 0b00001000) == 0:
                break
            if time.time() > timeout:
                raise RuntimeError("AS7265x measurement timeout")
            time.sleep(0.005)

        # Read raw and calibrated from all 3 sensors
        raw = []
        cal = []

        for dev in [0, 1, 2]:
            raw.extend(self._read_device_raw(dev))
            cal.extend(self._read_device_calibrated(dev))

        return {
            "raw": raw,
            "cal": cal,
            "wavelengths": AS7265x_WAVELENGTHS,
            "temperature": self.get_temperature()
        }
    # ----------------------------------------------------
    # ----------- HIGH-LEVEL CONVENIENCE WRAPPERS --------
    # ----------------------------------------------------

    def get_calibrated_spectrum(self):
        """Return list of 18 calibrated float values."""
        data = self.take_one_shot()
        return data["cal"]

    def get_raw_spectrum(self):
        """Return list of 18 raw ADC integers."""
        data = self.take_one_shot()
        return data["raw"]

    def get_full_spectrum_dict(self):
        """Return a nicely structured dictionary."""
        d = self.take_one_shot()
        return {
            "wavelengths": d["wavelengths"],
            "calibrated": d["cal"],
            "raw": d["raw"],
            "temperature": d["temperature"]
        }

    # ----------------------------------------------------
    # ---------------- PRETTY PRINT HELPER ---------------
    # ----------------------------------------------------

    def pretty_print(self, data=None):
        """
        Prints calibrated spectrum in a readable form.
        """
        if data is None:
            data = self.get_full_spectrum_dict()

        waves = data["wavelengths"]
        vals  = data["calibrated"]

        print("\n==== AS7265X Spectral Triad ====")
        for wl, v in zip(waves, vals):
            print(f"{wl:4d} nm : {v:10.6f}")
        print(f"Temperature: {data['temperature']} C")
        print("================================\n")
