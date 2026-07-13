"""
mycobot280 — clean Python API for the myCobot280 robotic arm.

Controls Feetech servos and the ATOM ESP32 over a shared half-duplex
UART bus using the Feetech SCS protocol.

    from mycobot280 import MyCobot280

    arm = MyCobot280("/dev/ttyAMA0")
    arm.servo(1).move(2048)
    arm.atom.color = (255, 0, 0)

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
        return self._bus._safe_limits(self.id)

    @property
    def raw_limits(self) -> tuple[int | None, int | None]:
        """Raw min/max angle limits from EEPROM."""
        with self._bus._lock:
            return (self._bus._read_u16(self.id, _ADDR_MIN_ANGLE_LIMIT),
                    self._bus._read_u16(self.id, _ADDR_MAX_ANGLE_LIMIT))

    # -- move ----------------------------------------------------------------

    def move(self, target: int, speed: int = 600, accel: int = 20) -> tuple[bool, int | None]:
        """Move to an absolute position. Returns (ok, final_position)."""
        return self._bus._move(self.id, target, speed, accel)

    def move_rel(self, delta: int, speed: int = 600, accel: int = 20) -> tuple[bool, int | None]:
        """Move relative to current position. Returns (ok, final_position)."""
        with self._bus._lock:
            cur = self._bus._read_u16(self.id, _ADDR_PRESENT_POSITION)
            if cur is None:
                return False, None
        return self._bus._move(self.id, cur + delta, speed, accel)

    def center(self, position: int = 2048, speed: int = 600, accel: int = 20) -> tuple[bool, int | None]:
        """Move to a center position (default 2048)."""
        return self.move(position, speed, accel)

    # -- torque --------------------------------------------------------------

    @property
    def torque(self) -> bool:
        """Is torque enabled?"""
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

        ``arm.atom.color = (255, 0, 0)``"""
        return None  # write-only, reading returns nothing useful

    @color.setter
    def color(self, rgb: tuple[int, int, int]):
        r, g, b = rgb
        with self._bus._lock:
            self._bus._write_raw(_ATOM_ID, _ATOM_ADDR_SET_COLOR, bytes([r, g, b]))

    def set_color(self, r: int = 0, g: int = 0, b: int = 0):
        """Set all 25 LEDs to the given colour."""
        self.color = (r, g, b)

    def pixel(self, x: int, y: int, r: int = 255, g: int = 0, b: int = 0):
        """Set a single pixel on the 5×5 matrix (x, y = 0–4)."""
        with self._bus._lock:
            self._bus._write_raw(_ATOM_ID, _ATOM_ADDR_SET_PIXEL,
                                 bytes([x, y, r, g, b]))

    def set_brightness(self, percent: int):
        """Set LED brightness as a percentage (1–100).

        Mapped to 0–128 on the hardware (0–50% of the NeoPixel range) to
        prevent ESP32 regulator burnout.

        ``arm.atom.set_brightness(50)``"""
        percent = max(1, min(100, percent))
        raw = int(percent * 128 / 100)
        with self._bus._lock:
            self._bus._write_raw(_ATOM_ID, _ATOM_ADDR_SET_BRIGHTNESS,
                                 bytes([raw]))

    @property
    def brightness(self) -> None:
        """Write-only: set LED brightness as a percentage (1–100).

        ``arm.atom.brightness = 50``"""
        return None

    @brightness.setter
    def brightness(self, percent: int):
        self.set_brightness(percent)

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

    # -- limits --------------------------------------------------------------

    def _safe_limits(self, servo_id: int) -> tuple[int, int]:
        if servo_id not in self._limit_cache:
            lo = self._read_u16(servo_id, _ADDR_MIN_ANGLE_LIMIT)
            hi = self._read_u16(servo_id, _ADDR_MAX_ANGLE_LIMIT)
            if lo is None or hi is None or (lo == 0 and hi == 0):
                self._limit_cache[servo_id] = (_RANGE_MIN + _SAFETY_BUFFER,
                                                _RANGE_MAX - _SAFETY_BUFFER)
            else:
                self._limit_cache[servo_id] = (lo + _SAFETY_BUFFER,
                                                hi - _SAFETY_BUFFER)
        return self._limit_cache[servo_id]

    def _clamp(self, servo_id: int, target: int) -> int:
        lo, hi = self._safe_limits(servo_id)
        return max(lo, min(hi, target))

    # -- move ----------------------------------------------------------------

    def _move(self, servo_id: int, target: int, speed: int, accel: int):
        with self._lock:
            target = self._clamp(servo_id, target)
            self._write_raw(servo_id, _ADDR_TORQUE_ENABLE, bytes([1]))
            self._write_raw(servo_id, _ADDR_ACCELERATION,  bytes([accel & 0xFF]))
            self._write_raw(servo_id, _ADDR_GOAL_SPEED,
                            bytes([speed & 0xFF, (speed >> 8) & 0xFF]))
            self._write_raw(servo_id, _ADDR_GOAL_POSITION,
                            bytes([target & 0xFF, (target >> 8) & 0xFF]))

            deadline = time.time() + _MOVE_SETTLE_TIMEOUT
            pos = None
            while time.time() < deadline:
                time.sleep(_MOVE_SETTLE_POLL)
                p = self._read_u16(servo_id, _ADDR_PRESENT_POSITION)
                if p is not None:
                    pos = p
                    if abs(p - target) <= _MOVE_TOLERANCE:
                        break

        ok = pos is not None and abs(pos - target) <= _MOVE_TOLERANCE
        return ok, pos

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
    """

    def __init__(self, port: str, baud: int = 1_000_000):
        self._bus = _Bus(port, baud)
        self._servo_ids: list[int] = []
        self._atom = _Atom(self._bus)
        self.scan()

    # -- scanning ------------------------------------------------------------

    def scan(self) -> list[int]:
        """Scan the bus for servos (IDs 1–50). Returns the list of found IDs."""
        with self._bus._lock:
            ids = []
            for sid in range(1, 51):
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
        """Move a servo to an absolute position."""
        return self.servo(servo_id).move(target, speed, accel)

    def move_rel(self, servo_id: int, delta: int, speed: int = 600, accel: int = 20):
        """Move a servo relative to its current position."""
        return self.servo(servo_id).move_rel(delta, speed, accel)

    def center(self, servo_id: int, position: int = 2048, speed: int = 600, accel: int = 20):
        """Center a servo."""
        return self.servo(servo_id).center(position, speed, accel)

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
