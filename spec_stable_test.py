# spec_stable_test_fixed.py
import time, sys, json
from as7265x import AS7265X

LOG="/home/pi/spec_stable_log.txt"

def make_sensor():
    try:
        s = AS7265X()
    except Exception as e:
        print("Sensor construction error:", e)
        return None
    # safe defaults
    try:
        s.driver_led_current = 0
        s.indicator_led = False
    except Exception:
        pass
    try:
        s.gain = 1
        s.integration_time = 30
    except Exception:
        pass
    return s

def summarize_spectrum(spec):
    # spec may be a list or a dict
    if spec is None:
        return "NA"
    if isinstance(spec, (list, tuple)):
        # return first 3 numeric entries
        return ", ".join(str(spec[i]) if i < len(spec) else "NA" for i in range(3))
    if isinstance(spec, dict):
        # pick first 3 items sorted by key (if numeric) otherwise insertion order
        try:
            keys = sorted(spec.keys(), key=lambda k: float(k))  # if keys numeric strings
        except Exception:
            keys = list(spec.keys())
        items = []
        for k in keys[:3]:
            items.append(f"{k}:{spec[k]}")
        return ", ".join(items) if items else "NA"
    return str(spec)

def safe_read(sensor, attempts=3, wait_between=0.5):
    for a in range(attempts):
        try:
            d = sensor.get_full_spectrum_dict()
            return d
        except RuntimeError as e:
            print(f"Read timeout (attempt {a+1}/{attempts}):", e)
            time.sleep(wait_between)
        except Exception as e:
            print("Other read error:", e)
            time.sleep(wait_between)
    return None

def log_line(msg):
    with open(LOG, "a") as f:
        f.write(msg + "\n")

def main():
    sensor = make_sensor()
    if sensor is None:
        print("Couldn't init sensor ? exiting.")
        sys.exit(1)
    print("Starting stable read loop. Ctrl-C to stop.")
    i = 0
    while True:
        i += 1
        data = safe_read(sensor, attempts=3, wait_between=0.4)
        ts = time.strftime("%Y-%m-%d %H:%M:%S")
        if data:
            # data might be a dict containing 'spectrum' key or already the spectrum
            if isinstance(data, dict) and "spectrum" in data:
                spec = data["spectrum"]
            else:
                spec = data
            summary = summarize_spectrum(spec)
            temp = data.get("temperature", "NA") if isinstance(data, dict) else "NA"
            out = f"[{i}] {ts} OK temp={temp} => {summary}"
            print(out)
            log_line(out)
        else:
            out = f"[{i}] {ts} FAILED after retries ? attempting re-init"
            print(out)
            log_line(out)
            try:
                del sensor
            except:
                pass
            time.sleep(0.5)
            sensor = make_sensor()
            if sensor is None:
                err = f"{ts} Re-init failed, waiting 5s..."
                print(err)
                log_line(err)
                time.sleep(5)
        time.sleep(0.6)

if __name__ == "__main__":
    main()
