# test_as7265x.py -- robust probe for AS7265x API variants
import time
import sys
import board
import busio

# try import library
try:
    import adafruit_as726x
except Exception as e:
    print("ERROR: cannot import adafruit_as726x:", e)
    sys.exit(1)

# helper ------------------------------------------------------------
def pick_trigger(sensor):
    """Return a callable(trigger) and description or (None, msg)."""
    if hasattr(sensor, "start_measurement"):
        return (lambda: sensor.start_measurement(), "start_measurement()")
    if hasattr(sensor, "take_measurements"):
        return (lambda: sensor.take_measurements(), "take_measurements()")
    if hasattr(sensor, "measure_once"):
        return (lambda: sensor.measure_once(), "measure_once()")
    # no explicit trigger; return a no-op
    return (lambda: None, "no trigger (assume instant-read)")

def pick_reader(sensor):
    """
    Return a reader function that yields a list of 18 floats (or raise).
    Tries several common APIs in order.
    """
    # 1) channels attribute (list-like)
    if hasattr(sensor, "channels"):
        return (lambda: list(sensor.channels), "sensor.channels")

    # 2) channel_0 .. channel_17 attributes
    attrs = ["channel_" + str(i) for i in range(18)]
    if all(hasattr(sensor, a) for a in attrs):
        return (lambda: [getattr(sensor, a) for a in attrs], "sensor.channel_0..channel_17")

    # 3) get_calibrated_values() or get_raw_values()
    if hasattr(sensor, "get_calibrated_values"):
        return (lambda: list(sensor.get_calibrated_values()), "get_calibrated_values()")
    if hasattr(sensor, "get_raw_values"):
        return (lambda: list(sensor.get_raw_values()), "get_raw_values()")
    if hasattr(sensor, "read_all"):
        return (lambda: list(sensor.read_all()), "read_all()")

    # 4) older libs sometimes expose read and raw buffer
    if hasattr(sensor, "read_calibrated_values"):
        return (lambda: list(sensor.read_calibrated_values()), "read_calibrated_values()")

    # none found
    raise RuntimeError("No known read API found on adafruit_as726x object. Available attributes: " +
                       ", ".join(sorted(attr for attr in dir(sensor) if not attr.startswith("_"))))

# main --------------------------------------------------------------
def main():
    # init i2c and sensor
    i2c = busio.I2C(board.SCL, board.SDA)
    try:
        sensor = adafruit_as726x.AS726x_I2C(i2c)
    except Exception:
        # some versions use AS726x class directly
        try:
            sensor = adafruit_as726x.AS726x()
            # try to attach i2c device if the library requires it
            if hasattr(adafruit_as726x, "I2CDevice"):
                try:
                    sensor.i2c_device = adafruit_as726x.I2CDevice(i2c, 0x49)
                except Exception:
                    pass
        except Exception as e:
            print("ERROR: could not construct sensor object:", e)
            raise

    # set extended mode & integration time if attributes exist
    if hasattr(sensor, "conversion_mode"):
        try:
            sensor.conversion_mode = getattr(sensor, "MODE_2", sensor.conversion_mode)
        except Exception:
            pass
    try:
        # many libs accept integration_time (ms)
        if hasattr(sensor, "integration_time"):
            sensor.integration_time = 100
    except Exception:
        pass

    # pick trigger + reader
    trigger, trig_name = pick_trigger(sensor)
    try:
        reader, read_name = pick_reader(sensor)
    except Exception as e:
        print("Reader probe failed:", e)
        raise

    print("Probe summary:")
    print("  Trigger chosen:", trig_name)
    print("  Reader chosen:", read_name)
    print("  Integration time attempted:", getattr(sensor, "integration_time", "n/a"))
    print("  Conversion mode:", getattr(sensor, "conversion_mode", "n/a"))
    print("Starting measurements (Ctrl+C to stop)...\n")
 # wavelengths for printing
    wavelengths = [
        410, 435, 460, 485, 510, 535,
        560, 585, 610, 645, 680, 705,
        730, 760, 810, 860, 900, 940
    ]

    try:
        while True:
            # trigger a measurement if needed
            try:
                trigger()
            except Exception as e:
                # ignore trigger exceptions but warn
                print("Warning: trigger call raised:", e)

            # wait briefly for integration / measurement
            # some implementations require polling; we simply wait a short time
            time.sleep(0.2)

            # attempt to read values
            try:
                vals = reader()
                # ensure length 18
                if len(vals) != 18:
                    print("Warning: reader returned length", len(vals), "expected 18. Values:", vals)
                # print one tidy block
                print("====== SPECTRUM ======")
                for wl, v in zip(wavelengths, vals[:18]):
                    # print floats with sensible precision; handle None or non-numeric
                    try:
                        print(f"{wl:3d} nm: {float(v):.6f}")
                    except Exception:
                        print(f"{wl:3d} nm: {v!r}")
                print("-----------------------\n")
            except Exception as e:
                print("Read error:", e)
                # if reading fails, show available attributes for debugging
                print("Sensor dir (sample):", sorted(attr for attr in dir(sensor) if not attr.startswith("_"))[:40])
                raise

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("Stopped by user")

if __name__ == "__main__":
    main()