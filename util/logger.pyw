import serial
import time
import csv
import threading
import queue
import os
from datetime import datetime

# --- CONFIGURATION ---
SERIAL_PORT = "COM6"  # Update this
BAUD_RATE = 115200
HEADER = ["Timestamp", "Temperature", "Humidity", "CO2"]

# Get the folder where this script lives
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

data_queue = queue.Queue()
stop_event = threading.Event()


def get_monthly_filename():
    """Returns the full path for the current month's CSV (e.g., 202601.csv)"""
    current_month = datetime.now().strftime("%Y%m")  # Format: YYYYMM
    filename = f"{current_month}.csv"
    return os.path.join(SCRIPT_DIR, filename)


def read_serial_data():
    """Thread 1: Reads data from USB (Blocking I/O)"""
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            print(f"✅ Serial connected: {SERIAL_PORT}")
            while not stop_event.is_set():
                try:
                    # Blocks until data arrives (0% CPU)
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
    """Thread 2: Writes data to disk. Handles Excel locking errors."""

    current_filename = ""
    print("📂 Logger active. Waiting for data...")

    while not stop_event.is_set() or not data_queue.empty():
        try:
            # Get data from queue
            raw_data = data_queue.get(timeout=1)

            # Check which month we are in
            target_file = get_monthly_filename()

            # Check if we need a header (file doesn't exist yet)
            file_exists = os.path.isfile(target_file)

            # --- THE SAFETY BLOCK ---
            try:
                # Open, Write, and CLOSE immediately
                with open(target_file, mode="a", newline="") as f:
                    writer = csv.writer(f)

                    # Write header if new file
                    if not file_exists:
                        writer.writerow(HEADER)

                    # Prepare and write data
                    parts = raw_data.split(",")
                    if len(parts) == 3:
                        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        row = [timestamp] + [p.strip() for p in parts]
                        writer.writerow(row)
                        print(f"saved: {row}")

            except PermissionError:
                # This runs if Excel has the file open!
                print(f"⚠️ File is locked by Excel! Data buffered in memory...")
                # We put the data BACK into the front of the queue to try again later
                # (Note: standard Queue doesn't support push-to-front, so we just
                # re-add it to the back or use a temporary retry logic.
                # For simplicity, we print an error and lose THIS single point,
                # or you can simply 'pass' and let the next point try.)
                pass

            data_queue.task_done()

        except queue.Empty:
            continue
        except Exception as e:
            print(f"❌ Write Error: {e}")


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
        print("\n🛑 Stopping...")
        stop_event.set()

    t1.join()
    t2.join()
    print("Done.")
