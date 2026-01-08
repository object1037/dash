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
    """Thread 1: Reader. Handles the 15-minute silence efficiently."""
    while not stop_event.is_set():
        try:
            # timeout=None blocks efficiently until data arrives
            with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=None) as ser:
                while not stop_event.is_set():
                    try:
                        line = ser.readline()

                        if line:
                            decoded = line.decode("utf-8").strip()
                            if decoded:
                                data_queue.put(decoded)
                        else:
                            # CRITICAL FIX: If driver returns empty bytes (EOF glitch),
                            # sleep briefly to prevent 100% CPU spin.
                            time.sleep(0.1)

                    except Exception:
                        break  # Break inner loop to reconnect

        except Exception:
            # If port fails to open, wait 60s before retrying
            # (Since data is slow, fast retries aren't needed)
            time.sleep(60)


def write_to_csv():
    """Thread 2: Writer. Sleeps for minutes at a time."""

    # Since data is rare (every 15m), we don't need complex buffering.
    # We can just write immediately when data arrives.

    while not stop_event.is_set():
        try:
            # OPTIMIZATION: Wait up to 5 minutes (300s) for data.
            # This allows the thread to be completely inactive (0% CPU).
            # Because daemon=True, we don't care if it gets stuck here on exit.
            raw_data = data_queue.get(timeout=300)

            # --- If we wake up, it means we have data! ---
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
                        # print(f"saved: {row}") # Keep silent

            except PermissionError:
                # If Excel is open, we lose this 1 point.
                # With 15min intervals, buffering is risky (data stays in RAM too long).
                pass
            except Exception:
                pass

            data_queue.task_done()

        except queue.Empty:
            # Timeout reached (5 mins passed with no data).
            # Loop again to check stop_event.
            continue


if __name__ == "__main__":
    # daemon=True is CRITICAL. It ensures that when you close the main script,
    # these "sleeping" threads are killed instantly by the OS.
    t1 = threading.Thread(target=read_serial_data, daemon=True)
    t2 = threading.Thread(target=write_to_csv, daemon=True)

    t1.start()
    t2.start()

    try:
        # Main thread does nothing but wait for Ctrl+C
        # Using a long wait prevents the main loop from eating CPU
        while not stop_event.is_set():
            stop_event.wait(timeout=3600)  # Check every hour
    except KeyboardInterrupt:
        stop_event.set()
