"""On-board check: ID2/ID3 online at zero voltage, no growing CAN losses.

Run with ESP-IDF's Python: python tools/check_can.py COM30
Only sends stop/status; does not reset the board or command motion.
"""
import re
import sys
import time

import serial


def open_port(port):
    connection = serial.Serial()
    connection.port = port
    connection.baudrate = 115200
    connection.timeout = 0.1
    connection.dtr = connection.rts = False
    connection.open()
    return connection


def read_for(connection, seconds):
    deadline = time.monotonic() + seconds
    data = bytearray()
    while time.monotonic() < deadline:
        data.extend(connection.read(4096))
    return data.decode("utf-8", errors="replace")


def snapshot(connection):
    connection.reset_input_buffer()
    connection.write(b"status\n")
    log = read_for(connection, 1.5)
    status = re.search(r"CAN state=running[^\r\n]+", log)
    assert status, log
    print(status.group(), flush=True)
    for motor_id in (2, 3):
        motor = re.search(rf"ID={motor_id} target=0\.000 V[^\r\n]*feedback=online", log)
        assert motor, log
        print(motor.group(), flush=True)
    return {key: int(value) for key, value in re.findall(r"(\w+)=(\d+)", status.group())}


if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else "COM30"
    with open_port(port) as connection:
        connection.write(b"stop\n")
        read_for(connection, 1)
        before = snapshot(connection)
        read_for(connection, 15)
        during = snapshot(connection)
    # Exercise the original USB-print stall condition with no serial reader.
    time.sleep(10)
    with open_port(port) as connection:
        after = snapshot(connection)
    for key in ("rx_missed", "rx_overrun", "tx_failed", "bus_errors"):
        assert before[key] == during[key] == after[key], (key, before, during, after)
    assert after["tx_err"] == after["rx_err"] == 0, after
    print("PASS: zero targets, both motors online, no new CAN losses with USB open or closed.")
