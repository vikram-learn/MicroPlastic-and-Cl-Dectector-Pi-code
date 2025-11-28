#!/usr/bin/env python3
# mpu6050_read_resilient.py ? resilient MPU6050 reader with retry + soft-reset
import time
from smbus2 import SMBus
import math, statistics, sys

BUS = 1
ADDR = 0x68

# registers
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B

ACCEL_SCALE = 16384.0
GYRO_SCALE  = 131.0

# robustness params
READ_RETRIES = 3       # retries per read
ERROR_RETRY_SLEEP = 0.05
SOFT_RESET_TRIES = 2
BUS_REOPEN_SLEEP = 0.2

def twos_complement(high, low):
    val = (high << 8) | low
    if val & 0x8000:
        return -((65535 - val) + 1)
    return val

def read_regs(bus, reg, length):
    """Read block with safe exception propagation (caller handles exceptions)."""
    return bus.read_i2c_block_data(ADDR, reg, length)

def safe_read_block(bus, reg, length):
    """Try read with retries. On repeated failure raise OSError."""
    for attempt in range(READ_RETRIES):
        try:
            return read_regs(bus, reg, length)
        except OSError as e:
            # short backoff and retry
            time.sleep(ERROR_RETRY_SLEEP)
    # after retries, raise the error
    raise OSError("i2c read failed after retries")

def read_accel_gyro_temp(bus):
    b = safe_read_block(bus, ACCEL_XOUT_H, 14)
    ax = twos_complement(b[0], b[1]) / ACCEL_SCALE
    ay = twos_complement(b[2], b[3]) / ACCEL_SCALE
    az = twos_complement(b[4], b[5]) / ACCEL_SCALE
    temp_raw = twos_complement(b[6], b[7])
    gx = twos_complement(b[8], b[9]) / GYRO_SCALE
    gy = twos_complement(b[10], b[11]) / GYRO_SCALE
    gz = twos_complement(b[12], b[13]) / GYRO_SCALE
    temp_c = (temp_raw / 340.0) + 36.53
    return (ax, ay, az, gx, gy, gz, temp_c)

def init_mpu(bus):
    bus.write_byte_data(ADDR, PWR_MGMT_1, 0x00)  # wake
    bus.write_byte_data(ADDR, SMPLRT_DIV, 0x07)
    bus.write_byte_data(ADDR, CONFIG, 0x03)
    bus.write_byte_data(ADDR, GYRO_CONFIG, 0x00)
    bus.write_byte_data(ADDR, ACCEL_CONFIG, 0x00)

def soft_reset(bus):
    try:
        # set DEVICE_RESET bit (0x80), then wait and wake again
        bus.write_byte_data(ADDR, PWR_MGMT_1, 0x80)
        time.sleep(0.1)
        init_mpu(bus)
        time.sleep(0.05)
        return True
    except Exception:
        return False

def calibrate(bus, samples=200, delay=0.01):
    print("Calibrating... keep IMU steady")
    ax_list = []; ay_list = []; az_list = []
    gx_list = []; gy_list = []; gz_list = []
    for i in range(samples):
        ax, ay, az, gx, gy, gz, _ = read_accel_gyro_temp(bus)
        ax_list.append(ax); ay_list.append(ay); az_list.append(az)
        gx_list.append(gx); gy_list.append(gy); gz_list.append(gz)
        time.sleep(delay)
    offsets = {
        'ax_off': statistics.mean(ax_list),
        'ay_off': statistics.mean(ay_list),
        'az_off': statistics.mean(az_list) - 1.0,
        'gx_off': statistics.mean(gx_list),
        'gy_off': statistics.mean(gy_list),
        'gz_off': statistics.mean(gz_list),
    }
    print("Calibration done:", offsets)
    return offsets

def open_bus():
    return SMBus(BUS)
def main():
    bus = None
    try:
        bus = open_bus()
    except Exception as e:
        print("Failed opening I2C bus:", e)
        sys.exit(1)

    try:
        init_mpu(bus)
    except Exception as e:
        print("Init failed:", e)

    # try calibrate; if it fails, try soft resets a couple times
    for attempt in range(3):
        try:
            offsets = calibrate(bus, samples=150, delay=0.01)
            break
        except OSError as e:
            print("Calibration read error:", e, "? trying soft reset")
            for _ in range(SOFT_RESET_TRIES):
                if soft_reset(bus):
                    break
            time.sleep(BUS_REOPEN_SLEEP)
    else:
        print("Calibration failed repeatedly; continuing with zero offsets")
        offsets = {'ax_off':0,'ay_off':0,'az_off':0,'gx_off':0,'gy_off':0,'gz_off':0}

    # complementary filter state
    alpha = 0.98
    last = time.time()
    pitch = 0.0; roll = 0.0

    print("time,ax,ay,az,gx,gy,gz,temp,pitch,roll")
    while True:
        try:
            now = time.time()
            dt = now - last if last else 0.01
            last = now

            ax, ay, az, gx, gy, gz, temp_c = read_accel_gyro_temp(bus)

            ax -= offsets['ax_off']; ay -= offsets['ay_off']; az -= offsets['az_off']
            gx -= offsets['gx_off']; gy -= offsets['gy_off']; gz -= offsets['gz_off']

            # accel angles
            acc_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))
            acc_roll  = math.degrees(math.atan2( ay, az if abs(az) > 1e-6 else 1e-6))

            pitch_gyro = pitch + gx * dt
            roll_gyro  = roll  + gy * dt

            pitch = alpha * pitch_gyro + (1 - alpha) * acc_pitch
            roll  = alpha * roll_gyro  + (1 - alpha) * acc_roll

            print(f"{time.strftime('%Y-%m-%d %H:%M:%S')},{ax:.4f},{ay:.4f},{az:.4f},{gx:.3f},{gy:.3f},{gz:.3f},{temp_c:.2f},{pitch:.2f},{roll:.2f}")

            time.sleep(0.01)  # tune sampling
        except OSError as e:
            # recoverable I/O error: try soft-reset, reopen bus
            print("I/O error:", e, "- attempting recovery")
            try:
                bus.close()
            except Exception:
                pass
            time.sleep(BUS_REOPEN_SLEEP)
            recovered = False
            try:
                bus = open_bus()
                init_mpu(bus)
                recovered = True
                print("Reopened bus and reinitialized MPU.")
            except Exception as e2:
                print("Reopen failed:", e2)
            # attempt soft-reset if reopen not enough
            if not recovered:
                for _ in range(SOFT_RESET_TRIES):
                    try:
                        bus = open_bus()
                        if soft_reset(bus):
                            print("Soft reset succeeded.")
                            recovered = True
                            break
                    except Exception:
                        time.sleep(BUS_REOPEN_SLEEP)
            if not recovered:
                print("Recovery failed ? sleeping 1s then retrying main loop.")
                time.sleep(1.0)
        except KeyboardInterrupt:
            print("Stopped by user")
            break
        except Exception as ex:
            print("Unexpected error:", ex)
            time.sleep(0.2)

    try:
        bus.close()
    except Exception:
        pass

if __name__ == "__main__":
    main()