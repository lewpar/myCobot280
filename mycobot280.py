"""
mycobot280 — clean Python API for the myCobot280 robotic arm.

Controls Feetech servos and the ATOM ESP32 over a shared half-duplex
UART bus using the Feetech SCS protocol.

    from mycobot280 import MyCobot280

    arm = MyCobot280("/dev/ttyAMA0")
    arm.servo(1).move(2048)
    arm.atom.color = (255, 0, 0)

    # coordinated multi-joint motion (all joints move at once):
    arm.move_many({1: 2048, 2: 1500, 3: 2600})

All serial access is thread-safe.
"""

import threading
import time

try:
    import serial
except ImportError:
    serial = None


# ---------------------------------------------------------------------------
# Feetech protocol helpers
# ---------------------------------------------------------------------------

# Servo registers
_ADDR_MIN_ANGLE_LIMIT     = 9
_ADDR_MAX_ANGLE_LIMIT     = 11
_ADDR_TORQUE_ENABLE       = 40
_ADDR_ACCELERATION        = 41
_ADDR_GOAL_POSITION       = 42
_ADDR_GOAL_SPEED          = 46
_ADDR_PRESENT_POSITION    = 56
_ADDR_LOCK                = 55
_ADDR_POSITION_CORRECTION = 31

# ATOM command addresses (Feetech WRITE to ID 7)
_ATOM_ID                = 7
_ATOM_ADDR_PING         = 0x00
_ATOM_ADDR_SET_COLOR    = 0x01
_ATOM_ADDR_SET_PIXEL    = 0x02
_ATOM_ADDR_SET_BRIGHTNESS = 0x03
_ATOM_ADDR_GET_STATE    = 0x04

_RANGE_MIN    = 0
_RANGE_MAX    = 4095
_SAFETY_BUFFER = 50

_MOVE_SETTLE_TIMEOUT = 15.0
_MOVE_SETTLE_POLL    = 0.1
_MOVE_TOLERANCE      = 10


def _checksum(data: bytes) -> int:
    return (~sum(data)) & 0xFF


def _build_ping(servo_id: int) -> bytes:
    body = bytes([servo_id, 0x02, 0x01])
    return bytes([0xFF, 0xFF]) + body + bytes([_checksum(body)])


def _build_write(servo_id: int, address: int, data: bytes) -> bytes:
    body = bytes([servo_id, 3 + len(data), 0x03, address]) + data
    return bytes([0xFF, 0xFF]) + body + bytes([_checksum(body)])


def _build_read(servo_id: int, address: int, length: int) -> bytes:
    body = bytes([servo_id, 0x04, 0x02, address, length])
    return bytes([0xFF, 0xFF]) + body + bytes([_checksum(body)])


def _parse_status(resp: bytes) -> bytes | None:
    if len(resp) >= 6 and resp[0] == 0xFF and resp[1] == 0xFF:
        length = resp[3]
        return resp[5:5 + (length - 2)]
    return None


def _encode_signed_11bit(value: int) -> bytes:
    """Feetech sign-magnitude 16-bit field: bit 11 = sign, bits 0-10 = magnitude."""
    value = max(-2047, min(2047, value))
    if value < 0:
        word = 0x0800 | (-value)
    else:
        word = value & 0x07FF
    return bytes([word & 0xFF, (word >> 8) & 0xFF])


def _decode_signed_11bit(word: int) -> int:
    magnitude = word & 0x07FF
    if word & 0x0800:
        return -magnitude
    return magnitude


# ---------------------------------------------------------------------------
# Servo — represents a single servo joint
# ---------------------------------------------------------------------------

class Servo:
    """A single servo on the bus.

    Don't create this directly — use ``arm.servo(id)``."""

    def __init__(self, bus: "_Bus", servo_id: int):
        self._bus = bus
        self.id = servo_id

    # -- position ------------------------------------------------------------

    @property
    def position(self) -> int | None:
        """Current position (0–4095)."""
        with self._bus._lock:
            return self._bus._read_u16(self.id, _ADDR_PRESENT_POSITION)

    # -- limits --------------------------------------------------------------

    @property
    def limits(self) -> tuple[int, int]:
        """Safe min/max position with safety buffer applied."""
        with self._bus._lock:
            return self._bus._safe_limits(self.id)

    @property
    def raw_limits(self) -> tuple[int | None, int | None]:
        """Raw min/max angle limits from EEPROM."""
        with self._bus._lock:
            return (self._bus._read_u16(self.id, _ADDR_MIN_ANGLE_LIMIT),
                    self._bus._read_u16(self.id, _ADDR_MAX_ANGLE_LIMIT))

    # -- move ----------------------------------------------------------------

    def move(self, target: int, speed: int = 600, accel: int = 20) -> tuple[bool, int | None]:
        """Move to an absolute position and block until settled (or timeout).

        Returns (ok, final_position). For moving several servos at once,
        use ``arm.move_many(...)`` instead — calling ``move()`` on several
        servos in a row runs them strictly one-after-another.
        """
        return self._bus._move(self.id, target, speed, accel)

    def move_async(self, target: int, speed: int = 600, accel: int = 20) -> int:
        """Send the goal position and return immediately (no settle-wait).

        Returns the clamped target that was actually sent. Combine with
        ``arm.wait_until_settled(servo_id)`` or ``arm.move_many(...)`` for
        coordinated multi-joint motion.
        """
        return self._bus._move_async(self.id, target, speed, accel)

    def move_rel(self, delta: int, speed: int = 600, accel: int = 20) -> tuple[bool, int | None]:
        """Move relative to current position (atomic read+move). Returns (ok, final_position)."""
        return self._bus._move_rel(self.id, delta, speed, accel)

    def center(self, position: int = 2048, speed: int = 600, accel: int = 20) -> tuple[bool, int | None]:
        """Move to a center position (default 2048)."""
        return self.move(position, speed, accel)

    # -- center register ------------------------------------------------------

    def set_center_register(self, center: int = 2048) -> tuple[bool, int | None]:
        """Write the servo's Position Correction EEPROM register so the
        current physical position reports as ``center`` (default 2048).

        This is Feetech's "set center position" / CalibrationOfs operation.
        It does NOT move the servo — it calibrates the servo's zero so the
        current physical position becomes the new reported center.

        Returns (ok, new_correction).
        """
        return self._bus._set_center_register(self.id, center)

    def get_center_register(self) -> int | None:
        """Read the Position Correction (center) register value.

        Returns the signed correction offset, or None if unreadable.
        """
        return self._bus._read_correction(self.id)

    def wait_until_settled(self, target: int | None = None,
                            timeout: float = _MOVE_SETTLE_TIMEOUT) -> tuple[bool, int | None]:
        """Poll until this servo reaches ``target`` (or its last commanded
        target if omitted) or the timeout elapses."""
        if target is None:
            target = self._bus._read_u16(self.id, _ADDR_PRESENT_POSITION)
            if target is None:
                return False, None
        results = self._bus._wait_settled({self.id: target}, timeout=timeout)
        return results[self.id]

    # -- torque --------------------------------------------------------------

    @property
    def torque(self) -> bool:
        """Is torque enabled?"""
        with self._bus._lock:
            return self._bus._read_u8(self.id, _ADDR_TORQUE_ENABLE) == 1

    @torque.setter
    def torque(self, enable: bool):
        """Enable or disable torque."""
        with self._bus._lock:
            self._bus._write_raw(self.id, _ADDR_TORQUE_ENABLE, bytes([1 if enable else 0]))

    def torque_on(self):
        self.torque = True

    def torque_off(self):
        self.torque = False

    # -- ping ----------------------------------------------------------------

    def ping(self) -> bool:
        """Check if the servo responds."""
        with self._bus._lock:
            return self._bus._ping(self.id)

    def __repr__(self):
        pos = self.position
        return f"Servo(id={self.id}, pos={pos})"


# ---------------------------------------------------------------------------
# Atom — represents the ATOM ESP32 (LED matrix)
# ---------------------------------------------------------------------------

class _Atom:
    """Control the ATOM ESP32 at the end of the servo chain.

    Don't create this directly — use ``arm.atom``."""

    def __init__(self, bus: "_Bus"):
        self._bus = bus

    def ping(self) -> bool:
        """Check if the ATOM is reachable."""
        with self._bus._lock:
            return self._bus._write_raw(_ATOM_ID, _ATOM_ADDR_PING, b"") is not None

    @property
    def color(self) -> None:
        """Write-only: set all 25 LEDs to an RGB colour.

        ``arm.atom.color = (255, 0, 0)``. For confirmation of success, use
        ``set_color()`` instead, which returns a bool.
        """
        return None  # write-only, reading returns nothing useful

    @color.setter
    def color(self, rgb: tuple[int, int, int]):
        self.set_color(*rgb)

    def set_color(self, r: int = 0, g: int = 0, b: int = 0) -> bool:
        """Set all 25 LEDs to the given colour. Returns True if the ATOM acked."""
        with self._bus._lock:
            resp = self._bus._write_raw(_ATOM_ID, _ATOM_ADDR_SET_COLOR, bytes([r, g, b]))
        return resp is not None

    def pixel(self, x: int, y: int, r: int = 255, g: int = 0, b: int = 0) -> bool:
        """Set a single pixel on the 5×5 matrix (x, y = 0–4). Returns True if acked."""
        with self._bus._lock:
            resp = self._bus._write_raw(_ATOM_ID, _ATOM_ADDR_SET_PIXEL,
                                        bytes([x, y, r, g, b]))
        return resp is not None

    def set_brightness(self, percent: int) -> bool:
        """Set LED brightness as a percentage (1–100). Returns True if acked.

        Mapped to 0–128 on the hardware (0–50% of the NeoPixel range) to
        prevent ESP32 regulator burnout.

        ``arm.atom.set_brightness(50)``"""
        percent = max(1, min(100, percent))
        raw = int(percent * 128 / 100)
        with self._bus._lock:
            resp = self._bus._write_raw(_ATOM_ID, _ATOM_ADDR_SET_BRIGHTNESS,
                                        bytes([raw]))
        return resp is not None

    @property
    def brightness(self) -> None:
        """Write-only: set LED brightness as a percentage (1–100).

        ``arm.atom.brightness = 50``. For confirmation of success, use
        ``set_brightness()`` instead, which returns a bool.
        """
        return None

    @brightness.setter
    def brightness(self, percent: int):
        self.set_brightness(percent)

    def get_led_state(self) -> dict | None:
        """Read the current LED state from the ATOM.

        Returns dict with ``brightness`` (0-128 raw), global ``r``/``g``/``b``,
        and a ``pixels`` list of 25 [r,g,b] entries (row-major, 5x5),
        or None if unresponsive."""
        with self._bus._lock:
            resp = self._bus._write_raw(_ATOM_ID, _ATOM_ADDR_GET_STATE, b"")
            if resp and len(resp) == 79:
                pixels = []
                for i in range(25):
                    off = 4 + i * 3
                    pixels.append([resp[off], resp[off + 1], resp[off + 2]])
                return {
                    "brightness": resp[0],
                    "r": resp[1],
                    "g": resp[2],
                    "b": resp[3],
                    "pixels": pixels,
                }
        return None

    def __repr__(self):
        alive = self.ping()
        return f"Atom(alive={alive})"


# ---------------------------------------------------------------------------
# _Bus — internal low-level serial transport
# ---------------------------------------------------------------------------

class _Bus:
    """Thread-safe serial transport for the half-duplex bus."""

    def __init__(self, port: str, baud: int = 1_000_000, timeout: float = 0.2):
        if serial is None:
            raise ImportError("pyserial is required: pip install pyserial")
        self._lock = threading.Lock()
        self._ser = serial.Serial(port, baud, timeout=timeout)
        self._limit_cache: dict[int, tuple[int, int]] = {}

    # -- raw I/O -------------------------------------------------------------

    def _read_resp(self, wait: float = 0.05) -> bytes:
        time.sleep(wait)
        n = self._ser.in_waiting
        return self._ser.read(n) if n else b""

    def _write_raw(self, servo_id: int, address: int, data: bytes) -> bytes | None:
        pkt = _build_write(servo_id, address, data)
        self._ser.reset_input_buffer()
        self._ser.write(pkt)
        return _parse_status(self._read_resp(0.05))

    def _read_u16(self, servo_id: int, address: int) -> int | None:
        pkt = _build_read(servo_id, address, 2)
        self._ser.reset_input_buffer()
        self._ser.write(pkt)
        params = _parse_status(self._read_resp(0.05))
        if params and len(params) >= 2:
            return params[0] | (params[1] << 8)
        return None

    def _read_u8(self, servo_id: int, address: int) -> int | None:
        pkt = _build_read(servo_id, address, 1)
        self._ser.reset_input_buffer()
        self._ser.write(pkt)
        params = _parse_status(self._read_resp(0.05))
        return params[0] if params else None

    def _ping(self, servo_id: int) -> bool:
        pkt = _build_ping(servo_id)
        self._ser.reset_input_buffer()
        self._ser.write(pkt)
        resp = self._read_resp()
        return len(resp) >= 6 and resp[0] == 0xFF and resp[1] == 0xFF

    # -- center register -----------------------------------------------------

    def _read_correction(self, servo_id: int) -> int | None:
        """Read the signed 11-bit Position Correction register (EEPROM addr 31)."""
        with self._lock:
            word = self._read_u16(servo_id, _ADDR_POSITION_CORRECTION)
        return _decode_signed_11bit(word) if word is not None else None

    def _write_correction(self, servo_id: int, value: int) -> bool:
        """Unlock EEPROM, write the Position Correction register, re-lock."""
        with self._lock:
            self._write_raw(servo_id, _ADDR_LOCK, bytes([0]))
            ok = self._write_raw(servo_id, _ADDR_POSITION_CORRECTION,
                                 _encode_signed_11bit(value)) is not None
            self._write_raw(servo_id, _ADDR_LOCK, bytes([1]))
        return ok

    def _set_center_register(self, servo_id: int, center: int) -> tuple[bool, int | None]:
        """Calibrate the servo's center: rewrite Position Correction so the
        current physical position reports as ``center``. Does not move."""
        with self._lock:
            current = self._read_u16(servo_id, _ADDR_PRESENT_POSITION)
            old_word = self._read_u16(servo_id, _ADDR_POSITION_CORRECTION)
            if current is None or old_word is None:
                return False, None
            old = _decode_signed_11bit(old_word)
            new = max(-2047, min(2047, old + (current - center)))
            self._write_raw(servo_id, _ADDR_LOCK, bytes([0]))
            ok = self._write_raw(servo_id, _ADDR_POSITION_CORRECTION,
                                 _encode_signed_11bit(new)) is not None
            self._write_raw(servo_id, _ADDR_LOCK, bytes([1]))
        return (ok, new if ok else None)

    # -- limits --------------------------------------------------------------

    def _safe_limits(self, servo_id: int) -> tuple[int, int]:
        if servo_id not in self._limit_cache:
            lo = self._read_u16(servo_id, _ADDR_MIN_ANGLE_LIMIT)
            hi = self._read_u16(servo_id, _ADDR_MAX_ANGLE_LIMIT)
            if lo is None or hi is None or (lo == 0 and hi == 0):
                lo_b, hi_b = _RANGE_MIN + _SAFETY_BUFFER, _RANGE_MAX - _SAFETY_BUFFER
            else:
                lo_b, hi_b = lo + _SAFETY_BUFFER, hi - _SAFETY_BUFFER
                # If the safety buffer is wider than the servo's own raw
                # range, the buffered bounds invert (lo_b > hi_b), which
                # makes _clamp() collapse every target to lo_b regardless
                # of what was asked for. Fall back to the raw (unbuffered)
                # limits for narrow-range joints rather than freezing them.
                if lo_b >= hi_b:
                    lo_b, hi_b = lo, hi
            self._limit_cache[servo_id] = (lo_b, hi_b)
        return self._limit_cache[servo_id]

    def _clamp(self, servo_id: int, target: int) -> int:
        lo, hi = self._safe_limits(servo_id)
        return max(lo, min(hi, target))

    # -- move ----------------------------------------------------------------

    def _send_goal(self, servo_id: int, target: int, speed: int, accel: int) -> int:
        """Write goal position/speed/accel. Caller must hold self._lock."""
        target = self._clamp(servo_id, target)
        self._write_raw(servo_id, _ADDR_TORQUE_ENABLE, bytes([1]))
        self._write_raw(servo_id, _ADDR_ACCELERATION,  bytes([accel & 0xFF]))
        self._write_raw(servo_id, _ADDR_GOAL_SPEED,
                        bytes([speed & 0xFF, (speed >> 8) & 0xFF]))
        self._write_raw(servo_id, _ADDR_GOAL_POSITION,
                        bytes([target & 0xFF, (target >> 8) & 0xFF]))
        return target

    def _move_async(self, servo_id: int, target: int, speed: int, accel: int) -> int:
        """Send the goal position without waiting for it to settle."""
        with self._lock:
            return self._send_goal(servo_id, target, speed, accel)

    def _wait_settled(self, targets: dict[int, int],
                       timeout: float = _MOVE_SETTLE_TIMEOUT,
                       tolerance: int = _MOVE_TOLERANCE) -> dict[int, tuple[bool, int | None]]:
        """Poll one or more servos until each reaches its target or times out.

        Polling multiple servos in the same wait loop is what makes
        coordinated multi-joint motion possible — the lock is only held for
        the brief duration of each individual read, not for the whole wait.
        """
        deadline = time.time() + timeout
        positions: dict[int, int | None] = {sid: None for sid in targets}
        pending = set(targets)

        while pending and time.time() < deadline:
            time.sleep(_MOVE_SETTLE_POLL)
            for sid in list(pending):
                with self._lock:
                    p = self._read_u16(sid, _ADDR_PRESENT_POSITION)
                if p is not None:
                    positions[sid] = p
                    if abs(p - targets[sid]) <= tolerance:
                        pending.discard(sid)

        results = {}
        for sid, target in targets.items():
            p = positions[sid]
            ok = p is not None and abs(p - target) <= tolerance
            results[sid] = (ok, p)
        return results

    def _move(self, servo_id: int, target: int, speed: int, accel: int) -> tuple[bool, int | None]:
        target = self._move_async(servo_id, target, speed, accel)
        return self._wait_settled({servo_id: target})[servo_id]

    def _move_rel(self, servo_id: int, delta: int, speed: int, accel: int) -> tuple[bool, int | None]:
        """Atomic read-current-position-then-move, so a concurrent move on
        the same servo can't change the base position mid-calculation."""
        with self._lock:
            cur = self._read_u16(servo_id, _ADDR_PRESENT_POSITION)
            if cur is None:
                return False, None
            target = self._send_goal(servo_id, cur + delta, speed, accel)
        return self._wait_settled({servo_id: target})[servo_id]

    def close(self):
        self._ser.close()


# ---------------------------------------------------------------------------
# MyCobot280 — the public API
# ---------------------------------------------------------------------------

class MyCobot280:
    """Top-level interface for the myCobot280 robotic arm.

    >>> arm = MyCobot280("/dev/ttyAMA0")
    >>> arm.servo(1).move(2048)
    >>> arm.atom.color = (255, 0, 0)
    >>> arm.move_many({1: 2048, 2: 1500})   # coordinated multi-joint move
    """

    def __init__(self, port: str, baud: int = 1_000_000):
        self._bus = _Bus(port, baud)
        self._servo_ids: list[int] = []
        self._atom = _Atom(self._bus)
        self.scan()

    # -- scanning ------------------------------------------------------------

    def scan(self) -> list[int]:
        """Scan the bus for servos (IDs 1–50). Returns the list of found IDs.

        ID 7 is reserved for the ATOM ESP32 and is skipped even though it
        would not normally answer a real PING (only WRITE frames).
        """
        with self._bus._lock:
            ids = []
            for sid in range(1, 51):
                if sid == _ATOM_ID:
                    continue
                if self._bus._ping(sid):
                    ids.append(sid)
                time.sleep(0.015)
            self._servo_ids = ids
            self._bus._limit_cache.clear()
            for sid in ids:
                _ = self._bus._safe_limits(sid)  # populate cache
        return self._servo_ids

    @property
    def servo_ids(self) -> list[int]:
        """List of detected servo IDs (cached from last scan)."""
        return list(self._servo_ids)

    @property
    def servo_count(self) -> int:
        """Number of detected servos."""
        return len(self._servo_ids)

    # -- servo access ---------------------------------------------------------

    def servo(self, servo_id: int) -> Servo:
        """Get a ``Servo`` object for a specific ID.

        >>> s1 = arm.servo(1)
        >>> s1.position
        2036
        >>> s1.move(2048)
        (True, 2048)
        """
        return Servo(self._bus, servo_id)

    # -- convenience: direct servo operations ----------------------------------

    def move(self, servo_id: int, target: int, speed: int = 600, accel: int = 20):
        """Move a single servo to an absolute position and block until settled.

        For several servos at once, use ``move_many()`` — calling this in a
        loop runs the joints strictly one-after-another.
        """
        return self.servo(servo_id).move(target, speed, accel)

    def move_many(self, targets: dict[int, int], speed: int = 600, accel: int = 20,
                   timeout: float = _MOVE_SETTLE_TIMEOUT) -> dict[int, tuple[bool, int | None]]:
        """Move several servos at once, coordinated.

        Sends all goal positions first, then waits for every servo to settle
        together, instead of moving them one at a time.

        >>> arm.move_many({1: 2048, 2: 1500, 3: 2600})
        {1: (True, 2048), 2: (True, 1500), 3: (True, 2601)}
        """
        clamped = {}
        for sid, target in targets.items():
            clamped[sid] = self._bus._move_async(sid, target, speed, accel)
        return self._bus._wait_settled(clamped, timeout=timeout)

    def move_rel(self, servo_id: int, delta: int, speed: int = 600, accel: int = 20):
        """Move a servo relative to its current position."""
        return self.servo(servo_id).move_rel(delta, speed, accel)

    def center(self, servo_id: int, position: int = 2048, speed: int = 600, accel: int = 20):
        """Center a servo."""
        return self.servo(servo_id).center(position, speed, accel)

    def set_center_register(self, servo_id: int, center: int = 2048):
        """Write the servo's Position Correction register so its current
        physical position reports as ``center``. Does not move the servo."""
        return self.servo(servo_id).set_center_register(center)

    def get_center_register(self, servo_id: int) -> int | None:
        """Read the servo's Position Correction (center) register."""
        return self.servo(servo_id).get_center_register()

    def wait_until_settled(self, *servo_ids: int, timeout: float = _MOVE_SETTLE_TIMEOUT):
        """Block until the given servos (already in motion) reach their last
        commanded position, or the timeout elapses. Useful after
        ``move_async()``/``move_many()`` if you want to wait later rather
        than immediately."""
        targets = {}
        for sid in servo_ids:
            p = self.get_position(sid)
            if p is not None:
                targets[sid] = p
        return self._bus._wait_settled(targets, timeout=timeout)

    def get_position(self, servo_id: int) -> int | None:
        """Read a servo's current position."""
        return self.servo(servo_id).position

    def get_limits(self, servo_id: int) -> tuple[int, int]:
        """Read a servo's safe (buffered) position limits."""
        return self.servo(servo_id).limits

    def set_torque(self, servo_id: int, enable: bool):
        """Enable or disable torque on a servo."""
        self.servo(servo_id).torque = enable

    def servo_ping(self, servo_id: int) -> bool:
        """Check if a servo responds."""
        return self.servo(servo_id).ping()

    # -- ATOM ----------------------------------------------------------------

    @property
    def atom(self) -> _Atom:
        """Access the ATOM ESP32 (LED matrix, I/O).

        >>> arm.atom.color = (0, 255, 0)
        >>> arm.atom.pixel(2, 2, 255, 0, 0)
        """
        return self._atom

    # -- shutdown -------------------------------------------------------------

    def close(self):
        """Close the serial port."""
        self._bus.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def __repr__(self):
        return f"MyCobot280(servos={self._servo_ids})"