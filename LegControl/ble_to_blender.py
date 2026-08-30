"""
Bridge: BLE UpperLeg+LowerLeg -> Virtual Serial Port -> Blender.

Replicates the Arduino MPU6050 firmware output so UARTTable and
legcontrol_dual.py work unchanged:
    {"key": "/UpperLeg", "value": [w, x, y, z]}\n
    {"key": "/LowerLeg", "value": [w, x, y, z]}\n

The WT901 quaternion is used as-is: sensors are mounted +X right / +Y down /
+Z forward and the Blender bones (env_leg.blend) are aligned to the same
axes, so no coordinate transform is applied. The parent-child decomposition
(q0_inv * q1) happens on the Blender side in legcontrol_dual.py, exactly
like armcontrol.py.

Usage:
    python ble_to_blender.py --port COM1
    # In Blender: open env_leg.blend -> Scripting -> run legcontrol_dual.py
    # (Blender side opens the paired virtual port, e.g. COM2)
"""
import argparse
import asyncio
import json
import threading
import time

import numpy as np
import serial

from bleak import BleakClient, BleakScanner

MACS = {
    "UpperLeg": "DB:92:C8:74:13:3A",
    "LowerLeg": "FF:6F:55:03:8C:64",
}

NOTIFY = "0000ffe4-0000-1000-8000-00805f9a34fb"
WRITE = "0000ffe9-0000-1000-8000-00805f9a34fb"
READ_QUAT = bytes([0xFF, 0xAA, 0x27, 0x51, 0x00])
POLL_INTERVAL = 0.04  # 25 Hz

lock = threading.Lock()
latest = {role: np.array([1.0, 0.0, 0.0, 0.0]) for role in MACS}
initial = {}
buffers = {role: bytearray() for role in MACS}
status = {
    role: {"found": False, "connected": False, "quaternion_frames": 0, "error": ""}
    for role in MACS
}


def signed(v):
    return v - 65536 if v >= 32768 else v


def normalize(q):
    q = np.asarray(q, dtype=float)
    return q / (np.linalg.norm(q) or 1.0)


def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return normalize(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


def qinv(q):
    q = normalize(q)
    return np.array([q[0], -q[1], -q[2], -q[3]])


def qmat(q):
    w, x, y, z = normalize(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


# Mount matrix: sensor axes -> bone axes.
# Sensors are mounted +X right / +Y down / +Z forward and the Blender bones
# (env_leg.blend) are aligned to the same axes, so NO rotation is needed.
MOUNT = {
    "UpperLeg": np.eye(3),
    "LowerLeg": np.eye(3),
}


def local_rotation(role, q):
    with lock:
        base = initial.get(role, q)
    relative = qmul(qinv(base), q)
    p = MOUNT[role]
    return p @ qmat(relative) @ p.T


def mat_to_quat(R):
    """3x3 rotation matrix -> quaternion [w, x, y, z]."""
    trace = float(np.trace(R))
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = float((R[2, 1] - R[1, 2]) * s)
        y = float((R[0, 2] - R[2, 0]) * s)
        z = float((R[1, 0] - R[0, 1]) * s)
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = float((R[2, 1] - R[1, 2]) / s)
        x = float(0.25 * s)
        y = float((R[0, 1] + R[1, 0]) / s)
        z = float((R[0, 2] + R[2, 0]) / s)
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = float((R[0, 2] - R[2, 0]) / s)
        x = float((R[0, 1] + R[1, 0]) / s)
        y = float(0.25 * s)
        z = float((R[1, 2] + R[2, 1]) / s)
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = float((R[1, 0] - R[0, 1]) / s)
        x = float((R[0, 2] + R[2, 0]) / s)
        y = float((R[1, 2] + R[2, 1]) / s)
        z = float(0.25 * s)
    return normalize([w, x, y, z])


def parse(role):
    data = buffers[role]
    while len(data) >= 20:
        if data[0] != 0x55 or data[1] not in (0x61, 0x71):
            del data[0]
            continue
        frame = data[:20]
        del data[:20]
        if frame[1] == 0x71 and frame[2] == 0x51:
            q = [
                signed(frame[i] | frame[i + 1] << 8) / 32768
                for i in (4, 6, 8, 10)
            ]
            with lock:
                latest[role] = normalize(q)
                status[role]["quaternion_frames"] += 1
                if role not in initial:
                    initial[role] = latest[role].copy()


async def connect(role, device, stop):
    try:
        async with BleakClient(device, timeout=15) as client:
            notify = client.services.get_characteristic(NOTIFY)
            writer = client.services.get_characteristic(WRITE)
            if notify is None:
                raise RuntimeError("WT notify FFE4 not found")

            def callback(_, data):
                buffers[role].extend(data)
                parse(role)

            await client.start_notify(notify, callback)
            with lock:
                status[role]["connected"] = True
            while not stop.is_set():
                if writer:
                    await client.write_gatt_char(writer, READ_QUAT)
                await asyncio.sleep(POLL_INTERVAL)
            await client.stop_notify(notify)
    except Exception as exc:
        with lock:
            status[role]["error"] = str(exc) or type(exc).__name__
    finally:
        with lock:
            status[role]["connected"] = False


async def ble_main(stop):
    tasks = {}
    while not stop.is_set():
        devices = await BleakScanner.discover(timeout=5)
        by_address = {device.address.upper(): device for device in devices}
        for role, mac in MACS.items():
            device = by_address.get(mac.upper())
            with lock:
                status[role]["found"] = device is not None
            task = tasks.get(role)
            if device is not None and (task is None or task.done()):
                tasks[role] = asyncio.create_task(connect(role, device, stop))
        await asyncio.sleep(1)
    if tasks:
        await asyncio.gather(*tasks.values(), return_exceptions=True)


def calibrate_available_sensors():
    with lock:
        for role in MACS:
            if status[role]["quaternion_frames"] > 0:
                initial[role] = latest[role].copy()
    print("Calibration captured for available sensors.", flush=True)


def print_status():
    with lock:
        snapshot = {role: v.copy() for role, v in status.items()}
    for role, v in snapshot.items():
        state = "STREAMING" if v["connected"] else "FOUND" if v["found"] else "NOT FOUND"
        print(
            f"{role:<6} {state:<10} frames={v['quaternion_frames']:<4} {v['error']}",
            flush=True,
        )


def bridge_loop(ser, stop):
    last_status = 0.0
    while not stop.is_set():
        with lock:
            qs = {role: q.copy() for role, q in latest.items()}
        R_UpperLeg = local_rotation("UpperLeg", qs["UpperLeg"])
        R_LowerLeg = local_rotation("LowerLeg", qs["LowerLeg"])
        q0 = mat_to_quat(R_UpperLeg)
        q1 = mat_to_quat(R_LowerLeg)
        line0 = json.dumps({"key": "/UpperLeg", "value": q0.tolist()}) + "\n"
        line1 = json.dumps({"key": "/LowerLeg", "value": q1.tolist()}) + "\n"
        try:
            ser.write(line0.encode("utf-8"))
            ser.write(line1.encode("utf-8"))
            ser.flush()
        except serial.SerialException as exc:
            print(f"[Serial] {exc}", flush=True)
            time.sleep(0.5)
            continue
        now = time.time()
        if now - last_status >= 5.0:
            print_status()
            last_status = now
        time.sleep(POLL_INTERVAL)


def open_serial(port, baud=115200):
    ser = serial.Serial(port, baudrate=baud, timeout=1)
    print(f"Serial port {port} opened.", flush=True)
    return ser


def main():
    parser = argparse.ArgumentParser(description="BLE UpperLeg+LowerLeg -> virtual serial -> Blender")
    parser.add_argument("--port", default="COM1", help="Virtual serial port for Blender")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    stop = threading.Event()
    threading.Thread(target=lambda: asyncio.run(ble_main(stop)), daemon=True).start()

    # Let sensors connect, then capture the first valid pose as calibration.
    time.sleep(3)
    calibrate_available_sensors()

    ser = open_serial(args.port, args.baud)
    try:
        bridge_loop(ser, stop)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        ser.close()


if __name__ == "__main__":
    main()
