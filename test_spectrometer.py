from as7265x import AS7265X
import time

sensor = AS7265X(i2c_bus=1)

# Configure recommended settings
sensor.set_integration_time(280)  # ~280ms
sensor.set_gain(3)                 # 64x gain (best for low light)
sensor.enable_indicator_led(True)
sensor.enable_driver_led(False)    # keep main LED off unless needed

print("Reading AS7265x 18-channel spectrum...\n")

while True:
    data = sensor.get_full_spectrum_dict()

    # Pretty print
    sensor.pretty_print(data)

    time.sleep(1)
