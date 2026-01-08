import serial
import time
import csv
import threading
import queue
import os
from datetime import datetime
from dotenv import load_dotenv

# --- LOAD .ENV CONFIGURATION ---

# 1. Force Python to look for .env in the script's actual folder
script_dir = os.path.dirname(os.path.abspath(__file__))
env_path = os.path.join(script_dir, ".env")
load_dotenv(env_path)

# 2. Load variables (with defaults if missing)
SERIAL_PORT = os.getenv("SERIAL_PORT", "COM3")
BAUD_RATE = int(os.getenv("BAUD_RATE", 115200))  # Convert string to int!
LOG_DIRECTORY = os.getenv("LOG_DIRECTORY", "//home/data")

HEADER = ["Timestamp", "Temperature", "Humidity", "CO2"]

data_queue = queue.Queue()
stop_event = threading.Event()


def get_monthly_filename():
    """Returns the full path for the current month's CSV in //home/data"""

    # 1. Ensure the directory exists (Create it if missing)
    if not os.path.exists(LOG_DIRECTORY):
        try:
            os.makedirs(LOG_DIRECTORY, exist_ok=True)
            print(f"📂 Created directory: {LOG_DIRECTORY}")
        except OSError as e:
            print(f"❌ Error creating directory {LOG_DIRECTORY}: {e}")
            # Fallback to script folder if we can't write to //home/data
            return f"{datetime.now().strftime('%Y%m')}.csv"

    # 2. Build path: //home/data/202601.csv
    current_month = datetime.now().strftime("%Y%m")
    filename = f"{current_month}.csv"
    return os.path.join(LOG_DIRECTORY, filename)


def read_serial_data():
    """Thread 1: Reads data from USB"""
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            print(f"✅ Serial connected: {SERIAL_PORT}")
            while not stop_event.is_set():
                try:
                    line = ser.readline()
                    if line:
                        decoded = line.decode("utf-8").strip()
                        if decoded:
                            data_queue.put(decoded)
                except serial.SerialException as e:
                    print(f"❌ Serial Error: {e}")
                    stop_event.set()
                    break
    except Exception as e:
        print(f"❌ Connection Failed: {e}")
        stop_event.set()


def write_to_csv():
    """Thread 2: Writes data to disk"""
    print(f"📂 Logger active. Saving to: {LOG_DIRECTORY}")

    while not stop_event.is_set() or not data_queue.empty():
        try:
            raw_data = data_queue.get(timeout=1)
            target_file = get_monthly_filename()
            file_exists = os.path.isfile(target_file)

            try:
                # Open, Write, Close (Excel-safe method)
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
                print(f"⚠️ File locked by Excel. Skipping this point.")
            except Exception as e:
                print(f"❌ Write Error: {e}")

            data_queue.task_done()

        except queue.Empty:
            continue


# --- MAIN EXECUTION ---
if __name__ == "__main__":
    t1 = threading.Thread(target=read_serial_data, daemon=True)
    t2 = threading.Thread(target=write_to_csv, daemon=True)

    t1.start()
    t2.start()

    try:
        while not stop_event.is_set():
            time.sleep(1)
    except KeyboardInterrupt:
        stop_event.set()

    t1.join()
    t2.join()
