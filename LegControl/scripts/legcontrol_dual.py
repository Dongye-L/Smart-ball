"""
Dual-sensor leg control for Blender (UpperLeg + LowerLeg).

Reads /UpperLeg and /LowerLeg absolute quaternions from the
virtual serial port (UARTTable), applies the parent-child decomposition from
armcontrol.py, and drives J_Bip_R_UpperLeg / J_Bip_R_LowerLeg:

    upper leg = absolute quaternion q0
    lower leg = q0_inv * q1   (lower relative to upper -> real knee angle)

Run inside Blender: open env_leg.blend -> Scripting -> run this script.
Press ESC in the 3D view to stop.
"""
import sys
import os
import json
import logging
import threading
import time

import bpy
import numpy as np
import serial


# ---------------------------------------------------------------------------
# UARTTable (inlined from uartcomm.py so this file is fully self-contained:
# no import / sys.path needed when running inside Blender)
# ---------------------------------------------------------------------------
class UARTTable:
    def __init__(self, port, baudrate=115200, logging_level=logging.INFO):
        self.port = port
        self.baudrate = 115200
        self.ser = None
        self.stop_sig = threading.Event()
        self.stop_sig.clear()

        self.data_table = {}

        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.setLevel(logging_level)

        if not self.logger.handlers:
            ch = logging.StreamHandler()
            ch.setStream(sys.stdout)
            ch.setLevel(logging.DEBUG)
            ch.setFormatter(logging.Formatter("%(asctime)s %(levelname)s [%(name)s]: %(message)s", datefmt="%Y-%m-%d %H:%M:%S"))
            self.logger.addHandler(ch)

    def connect(self):
        self.logger.info("Connecting to serial port {0}...".format(self.port))
        # close any existing connection
        if self.ser:
            self.ser.close()
        while 1:
            # if main want to exit, we no longer try connect to port
            if self.stop_sig.is_set():
                return
            try:
                self.ser = serial.Serial(self.port, baudrate=self.baudrate)
            except serial.serialutil.SerialException as e:
                self.logger.error(str(e))
                self.logger.debug("Still trying...")
                time.sleep(0.5)
                continue

            # we only reach here if serial connection is established
            if self.ser.is_open:
                break

        self.logger.info("Connected.")

    def stop(self):
        self.stop_sig.set()
        if self.ser:
            self.ser.close()
        self.logger.info("Stopped.")

    def start(self):
        self.logger.debug("Starting...")
        self.recvPacket()

    def startThreaded(self):
        self.logger.debug("Starting thread...")
        self.t = threading.Thread(target=self.recvPacket)
        self.t.start()

    def get(self, key):
        return self.data_table.get(key)

    def recvPacket(self):
        while 1:
            if self.stop_sig.is_set():
                return

            c = b""
            buf = b""
            while c != b"\n":
                if self.stop_sig.is_set():
                    return
                buf += c
                # atomically handle serial object exceptions
                try:
                    c = self.ser.read()
                except (AttributeError, TypeError, serial.serialutil.SerialException) as e:
                    print(e)
                    self.connect()
                    continue

            data = buf.decode()
            try:
                data = json.loads(data)
            except json.decoder.JSONDecodeError:
                self.logger.warning("packet format error.")
                continue
            key = data.get("key")
            if not key:
                self.logger.warning("packet format error.")
                continue

            self.data_table[key] = data.get("value")

MCU_COM_PORT = "COM2"   # must match the paired virtual port of ble_to_blender.py
#FPS = 30               # use 30 if system performance is poor
FPS = 60

logger = logging.getLogger("BPY")
logger.setLevel(logging.DEBUG)
if not logger.handlers:
    ch = logging.StreamHandler()
    ch.setStream(sys.stdout)
    ch.setLevel(logging.DEBUG)
    ch.setFormatter(logging.Formatter("%(asctime)s %(levelname)s [%(name)s]: %(message)s", datefmt="%Y-%m-%d %H:%M:%S"))
    logger.addHandler(ch)


# get scene armature object
armature = bpy.data.objects["Armature"]

# get the bones we need
bone_upper_leg_R = armature.pose.bones.get("J_Bip_R_UpperLeg")
bone_lower_leg_R = armature.pose.bones.get("J_Bip_R_LowerLeg")

uart_table = UARTTable(MCU_COM_PORT, logging_level=logging.DEBUG)


def setBoneRotation(bone, rotation):
    if bone:
        w, x, y, z = rotation
        bone.rotation_mode = 'QUATERNION'
        bone.rotation_quaternion[0] = w
        bone.rotation_quaternion[1] = x
        bone.rotation_quaternion[2] = y
        bone.rotation_quaternion[3] = z


def multiplyQuaternion(q1, q0):
    """Hamilton product q1 * q0; both as [w, x, y, z].
    Copied from armcontrol.py (StackOverflow)."""
    w0, x0, y0, z0 = q0
    w1, x1, y1, z1 = q1
    return np.array([
        w1*w0 - x1*x0 - y1*y0 - z1*z0,
        w1*x0 + x1*w0 + y1*z0 - z1*y0,
        w1*y0 - x1*z0 + y1*w0 + z1*x0,
        w1*z0 + x1*y0 - y1*x0 + z1*w0
    ], dtype=np.float64)


class ModalTimerOperator(bpy.types.Operator):
    bl_idname = "wm.modal_timer_operator"
    bl_label = "Modal Timer Operator"
    _timer = None

    def modal(self, context, event):
        if event.type == "ESC":
            logger.info("BlenderTimer received ESC.")
            return self.cancel(context)

        if event.type == "TIMER":
            q0 = uart_table.get("/UpperLeg")   # upper leg absolute
            q1 = uart_table.get("/LowerLeg")   # lower leg absolute

            if not q0 or not q1:
                logger.warning("Invalid joint data: q0=%s q1=%s", q0, q1)
                return {"PASS_THROUGH"}

            q0 = np.array(q0, dtype=np.float64)
            q1 = np.array(q1, dtype=np.float64)

            logger.debug(f"q0={q0} q1={q1}")

            # Parent-child decomposition (same as armcontrol.py):
            # parent (upper leg) takes its absolute rotation;
            # child (lower leg) takes its absolute rotation relative to parent.
            q0_inv = q0 * np.array([1, -1, -1, -1])
            q1_rel = multiplyQuaternion(q0_inv, q1)

            setBoneRotation(bone_upper_leg_R, q0)
            setBoneRotation(bone_lower_leg_R, q1_rel)

            # Force 3D viewport redraw
            for window in context.window_manager.windows:
                for area in window.screen.areas:
                    if area.type == 'VIEW_3D':
                        area.tag_redraw()

        return {"PASS_THROUGH"}

    def execute(self, context):
        self._timer = context.window_manager.event_timer_add(1./FPS, window=context.window)
        context.window_manager.modal_handler_add(self)
        return {"RUNNING_MODAL"}

    def cancel(self, context):
        uart_table.stop()
        # reset joint position
        setBoneRotation(bone_upper_leg_R, [1, 0, 0, 0])
        setBoneRotation(bone_lower_leg_R, [1, 0, 0, 0])
        context.window_manager.event_timer_remove(self._timer)
        logger.info("BlenderTimer Stopped.")
        return {"CANCELLED"}


if __name__ == "__main__":
    try:
        logger.info("Starting dual-sensor leg mode (UpperLeg + LowerLeg)...")
        logger.info("Using /UpperLeg, /LowerLeg.")
        bpy.utils.register_class(ModalTimerOperator)
        uart_table.startThreaded()
        bpy.ops.wm.modal_timer_operator()
        logger.info("All started.")
    except KeyboardInterrupt:
        uart_table.stop()
        logger.info("Received KeyboardInterrupt, stopped.")
