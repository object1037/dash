import serial
import time
import csv
import threading
import queue
import os
from datetime import datetime
from dotenv import load_dotenv

# --- CONFIGURATION ---
script_dir = os.path.dirname(os.path.abspath(__file__))
load_dotenv(os.path.join(script_dir, ".env"))

SERIAL_PORT = os.getenv("SERIAL_PORT", "COM3")
BAUD_RATE = int(os.getenv("BAUD_RATE", 115200))
LOG_DIRECTORY = os.getenv("LOG_DIRECTORY", "//home/data")
HEADER = ["Timestamp", "Temperature", "Humidity", "CO2"]

data_queue = queue.Queue()
stop_event = threading.Event()


def get_monthly_filename():
    if not os.path.exists(LOG_DIRECTORY):
        try:
            os.makedirs(LOG_DIRECTORY, exist_ok=True)
        except OSError:
            return f"{datetime.now().strftime('%Y%m')}.csv"
    return os.path.join(LOG_DIRECTORY, f"{datetime.now().strftime('%Y%m')}.csv")


def read_serial_data():
    """Thread 1: Completely blocking read (0% Idle CPU)"""
    while not stop_event.is_set():
        try:
            # OPTIMIZATION: timeout=None blocks forever until data arrives
            with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=None) as ser:
                print(f"✅ Connected to {SERIAL_PORT}")

                while not stop_event.is_set():
                    # This line will now sleep indefinitely until ESP32 talks
                    line = ser.readline()
                    if line:
                        decoded = line.decode("utf-8").strip()
                        if decoded:
                            data_queue.put(decoded)

        except Exception as e:
            print(f"❌ Serial Error: {e}")
            # If error, wait 5s before retrying to avoid rapid looping
            time.sleep(5)


def write_to_csv():
    """Thread 2: Consumer"""
    print(f"📂 Logger active. Saving to: {LOG_DIRECTORY}")
    while not stop_event.is_set() or not data_queue.empty():
        try:
            # timeout=60 saves CPU compared to timeout=1
            raw_data = data_queue.get(timeout=60)

            target_file = get_monthly_filename()
            file_exists = os.path.isfile(target_file)

            try:
                with open(target_file, mode="a", newline="") as f:
                    writer = csv.writer(f)
                    if not file_exists:
                        writer.writerow(HEADER)

                    parts = raw_data.split(",")
                    if len(parts) == 3:
                        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        row = [timestamp] + [p.strip() for p in parts]
                        writer.writerow(row)
                        print(f"saved: {row}")
            except PermissionError:
                pass
            except Exception:
                pass

            data_queue.task_done()
        except queue.Empty:
            continue


if __name__ == "__main__":
    t1 = threading.Thread(target=read_serial_data, daemon=True)
    t2 = threading.Thread(target=write_to_csv, daemon=True)

    t1.start()
    t2.start()

    try:
        # OPTIMIZATION: Wait on the event instead of looping
        while not stop_event.is_set():
            stop_event.wait(timeout=60)

    except KeyboardInterrupt:
        print("\n🛑 Stopping...")
        stop_event.set()

    # Note: We do NOT join t1 here because it might be stuck in a blocking read.
    # Since it is a daemon thread, it will die automatically when we exit.
    print("Exiting.")
