#!/usr/bin/env python3
# mpu6050_read.py ? simple reader + small calibration + complementary filter
import time
from smbus2 import SMBus
import math
import statistics

# I2C
BUS = 1
ADDR = 0x68

# MPU6050 registers
PWR_MGMT_1   = 0x6B
ACCEL_XOUT_H = 0x3B
TEMP_OUT_H   = 0x41
GYRO_XOUT_H  = 0x43
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C

# scale factors (defaults for �2g, �250�/s)
ACCEL_SCALE = 16384.0   # LSB/g
GYRO_SCALE  = 131.0     # LSB/(deg/s)

def write_reg(bus, reg, val):
    bus.write_byte_data(ADDR, reg, val)

def read_regs(bus, reg, length):
    return bus.read_i2c_block_data(ADDR, reg, length)

def twos_complement(high, low):
    val = (high << 8) | low
    if val & 0x8000:
        return -((65535 - val) + 1)
    return val

def read_accel_gyro_temp(bus):
    b = read_regs(bus, ACCEL_XOUT_H, 14)
    ax = twos_complement(b[0], b[1]) / ACCEL_SCALE
    ay = twos_complement(b[2], b[3]) / ACCEL_SCALE
    az = twos_complement(b[4], b[5]) / ACCEL_SCALE
    temp_raw = twos_complement(b[6], b[7])
    gx = twos_complement(b[8], b[9]) / GYRO_SCALE
    gy = twos_complement(b[10], b[11]) / GYRO_SCALE
    gz = twos_complement(b[12], b[13]) / GYRO_SCALE
    # temperature (degC) per datasheet
    temp_c = (temp_raw / 340.0) + 36.53
    return (ax, ay, az, gx, gy, gz, temp_c)

def calibrate(bus, samples=200, delay=0.01):
    print("Calibrating... keep IMU steady")
    ax_list, ay_list, az_list = [], [], []
    gx_list, gy_list, gz_list = [], [], []
    for i in range(samples):
        ax, ay, az, gx, gy, gz, _ = read_accel_gyro_temp(bus)
        ax_list.append(ax); ay_list.append(ay); az_list.append(az)
        gx_list.append(gx); gy_list.append(gy); gz_list.append(gz)
        time.sleep(delay)
    # compute offsets (gyro offset in deg/s; accel offset in g)
    offsets = {
        'ax_off': statistics.mean(ax_list),
        'ay_off': statistics.mean(ay_list),
        'az_off': statistics.mean(az_list) - 1.0,  # gravity bias
        'gx_off': statistics.mean(gx_list),
        'gy_off': statistics.mean(gy_list),
        'gz_off': statistics.mean(gz_list),
    }
    print("Calibration done:", offsets)
    return offsets

def acc_to_angles(ax, ay, az):
    # pitch: rotation about X (deg), roll: rotation about Y (deg)
    # avoid division by zero by clamping
    pitch = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))
    roll  = math.degrees(math.atan2( ay, az if abs(az) > 1e-6 else 1e-6))
    return pitch, roll
def main():
    with SMBus(BUS) as bus:
        # wake up
        write_reg(bus, PWR_MGMT_1, 0x00)
        # set sample rate divider (optional)
        write_reg(bus, SMPLRT_DIV, 0x07)  # sample rate = GyroRate / (1 + 7)
        # set low-pass filter (optional)
        write_reg(bus, CONFIG, 0x03)
        # set gyro range to �250deg/s (0)
        write_reg(bus, GYRO_CONFIG, 0x00)
        # set accel range to �2g (0)
        write_reg(bus, ACCEL_CONFIG, 0x00)

        offsets = calibrate(bus, samples=200, delay=0.01)

        # complementary filter variables
        alpha = 0.98   # filter coefficient (0.98 uses gyro more)
        dt = 0.01      # initial dt estimate
        pitch = 0.0
        roll = 0.0
        last = time.time()

        print("time,ax,ay,az,gx,gy,gz,temp,pitch,roll")
        while True:
            now = time.time()
            dt = now - last if last else 0.01
            last = now

            ax, ay, az, gx, gy, gz, temp_c = read_accel_gyro_temp(bus)
            # apply offsets
            ax -= offsets['ax_off']
            ay -= offsets['ay_off']
            az -= offsets['az_off']
            gx -= offsets['gx_off']
            gy -= offsets['gy_off']
            gz -= offsets['gz_off']

            # acc angles
            acc_pitch, acc_roll = acc_to_angles(ax, ay, az)

            # integrate gyro (gyro is in deg/s)
            pitch_gyro = pitch + gx * dt
            roll_gyro  = roll  + gy * dt

            # complementary filter
            pitch = alpha * pitch_gyro + (1 - alpha) * acc_pitch
            roll  = alpha * roll_gyro  + (1 - alpha) * acc_roll

            # print CSV line
            print(f"{time.strftime('%Y-%m-%d %H:%M:%S')},{ax:.4f},{ay:.4f},{az:.4f},{gx:.3f},{gy:.3f},{gz:.3f},{temp_c:.2f},{pitch:.2f},{roll:.2f}")
            time.sleep(0.01)  # ~100 Hz target (adjust as needed)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Stopped")