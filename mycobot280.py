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
_ATOM_ADDR_GET_STATE    = 0x04

_RANGE_MIN    = 0
_RANGE_MAX    = 4095
_SAFETY_BUFFER = 50

# Valid register ranges. Speed 0 and acceleration 0 both mean "no limit" on STS servos,
# so they are never sent; values outside these ranges are clamped rather than wrapped.
SPEED_MIN, SPEED_MAX = 1, 4000      # goal speed, steps/s
ACCEL_MIN, ACCEL_MAX = 1, 254       # acceleration, 100 steps/s^2 per unit

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


def _build_sync_write(address: int, per_id: dict[int, bytes]) -> bytes:
    """SYNC WRITE (0x83) to the broadcast ID: one packet, every listed servo, no reply."""
    n = len(next(iter(per_id.values())))
    params = bytes([address, n]) + b"".join(bytes([i]) + d for i, d in per_id.items())
    body = bytes([0xFE, len(params) + 2, 0x83]) + params
    return bytes([0xFF, 0xFF]) + body + bytes([_checksum(body)])


def _find_status(buf: bytes, servo_id: int, nparams: int | None = None) -> tuple[bytes | None, bool]:
    """Look for a complete, checksum-valid status packet from ``servo_id`` in ``buf``.

    Returns (params, complete). ``params`` is the data after the error byte.
    ``nparams=None`` accepts any length. Anything else on the wire (our own TX echo,
    another device's traffic, line noise) is skipped rather than misread."""
    i = 0
    while True:
        i = buf.find(b"\xff\xff", i)
        if i < 0 or len(buf) < i + 4:
            return None, False
        sid, ln = buf[i + 2], buf[i + 3]
        if sid == servo_id and ln >= 2 and (nparams is None or ln == nparams + 2):
            end = i + 4 + ln
            if len(buf) < end:
                return None, False           # wait for more bytes
            body = buf[i + 2:end - 1]
            if _checksum(body) == buf[end - 1]:
                return bytes(buf[i + 5:end - 1]), True
        i += 1


def _u16(b: bytes) -> int:
    return b[0] | (b[1] << 8)


def _s16(b: bytes) -> int:
    """STS words use bit 15 as a sign bit (multi-turn / negative positions)."""
    v = _u16(b)
    return -(v & 0x7FFF) if v & 0x8000 else v


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
            return self._bus._read_s16(self.id, _ADDR_PRESENT_POSITION)

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

    def move(self, target: int, speed: int = 600, accel: int = 20,
             should_abort=None) -> tuple[bool, int | None]:
        """Move to an absolute position and wait for it. Returns (ok, final_position).
        ``should_abort`` is polled while waiting; returning True stops waiting early."""
        return self._bus._move(self.id, target, speed, accel, should_abort)

    def move_rel(self, delta: int, speed: int = 600, accel: int = 20) -> tuple[bool, int | None]:
        """Move relative to current position. Returns (ok, final_position)."""
        with self._bus._lock:
            cur = self._bus._read_s16(self.id, _ADDR_PRESENT_POSITION)
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

        ``arm.atom.color = (255, 0, 0)``"""
        return None  # write-only, reading returns nothing useful

    @color.setter
    def color(self, rgb: tuple[int, int, int]):
        self.set_color(*rgb)

    def set_color(self, r: int = 0, g: int = 0, b: int = 0) -> bool:
        """Set all 25 LEDs to the given colour. Returns True if the ATOM acknowledged it."""
        with self._bus._lock:
            return self._bus._write_raw(_ATOM_ID, _ATOM_ADDR_SET_COLOR, bytes([r, g, b])) is not None

    def pixel(self, x: int, y: int, r: int = 255, g: int = 0, b: int = 0) -> bool:
        """Set a single pixel on the 5×5 matrix (x, y = 0–4). Returns True if acknowledged."""
        with self._bus._lock:
            return self._bus._write_raw(_ATOM_ID, _ATOM_ADDR_SET_PIXEL,
                                        bytes([x, y, r, g, b])) is not None

    def set_brightness(self, percent: int) -> bool:
        """Set LED brightness as a percentage (1–100). Returns True if acknowledged.

        Mapped to 0–128 on the hardware (0–50% of the NeoPixel range) to
        prevent ESP32 regulator burnout.

        ``arm.atom.set_brightness(50)``"""
        percent = max(1, min(100, percent))
        raw = int(percent * 128 / 100)
        with self._bus._lock:
            return self._bus._write_raw(_ATOM_ID, _ATOM_ADDR_SET_BRIGHTNESS,
                                        bytes([raw])) is not None

    @property
    def brightness(self) -> None:
        """Write-only: set LED brightness as a percentage (1–100).

        ``arm.atom.brightness = 50``"""
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
    """Thread-safe serial transport for the half-duplex bus.

    Every method starting with ``_`` expects the caller to hold ``_lock``."""

    def __init__(self, port: str, baud: int = 1_000_000, timeout: float = 0.02):
        if serial is None:
            raise ImportError("pyserial is required: pip install pyserial")
        self._lock = threading.Lock()
        # short read timeout: replies arrive within ~1 ms at 1 Mbaud, so we poll instead of sleeping.
        # exclusive=True takes an advisory lock on the port, so a second program using this library
        # (a tool while the backend runs, say) fails to open it instead of garbling the bus.
        try:
            self._ser = serial.Serial(port, baud, timeout=0.002, exclusive=True)
        except serial.SerialException as e:
            if "lock" in str(e).lower() or "busy" in str(e).lower() or "resource temporarily" in str(e).lower():
                raise RuntimeError(f"{port} is already in use by another program (is the backend "
                                   f"already running?)") from e
            raise
        self._reply_timeout = timeout
        self._limit_cache: dict[int, tuple[int, int]] = {}

    # -- raw I/O -------------------------------------------------------------

    def _transact(self, pkt: bytes, servo_id: int, nparams: int | None = None,
                  timeout: float | None = None) -> bytes | None:
        """Send ``pkt`` and wait for the matching status packet. Returns its params or None."""
        self._ser.reset_input_buffer()
        self._ser.write(pkt)
        self._ser.flush()
        deadline = time.monotonic() + (timeout or self._reply_timeout)
        buf = b""
        while time.monotonic() < deadline:
            chunk = self._ser.read(max(1, self._ser.in_waiting))
            if not chunk:
                continue
            buf += chunk
            if buf.startswith(pkt):          # adapter echoed our own request back: drop it
                buf = buf[len(pkt):]
            params, done = _find_status(buf, servo_id, nparams)
            if done:
                return params
        return None

    def _write_raw(self, servo_id: int, address: int, data: bytes) -> bytes | None:
        # the ATOM redraws its LEDs before answering, so give it longer than a servo
        timeout = 0.06 if servo_id == _ATOM_ID else None
        return self._transact(_build_write(servo_id, address, data), servo_id, None, timeout)

    def _read_u16(self, servo_id: int, address: int) -> int | None:
        p = self._transact(_build_read(servo_id, address, 2), servo_id, 2)
        return _u16(p) if p is not None else None

    def _read_s16(self, servo_id: int, address: int) -> int | None:
        p = self._transact(_build_read(servo_id, address, 2), servo_id, 2)
        return _s16(p) if p is not None else None

    def _read_u8(self, servo_id: int, address: int) -> int | None:
        p = self._transact(_build_read(servo_id, address, 1), servo_id, 1)
        return p[0] if p else None

    def _ping(self, servo_id: int, timeout: float | None = None) -> bool:
        return self._transact(_build_ping(servo_id), servo_id, 0, timeout) is not None

    def _sync_write(self, address: int, per_id: dict[int, bytes]):
        if not per_id:
            return
        self._ser.reset_input_buffer()
        self._ser.write(_build_sync_write(address, per_id))
        self._ser.flush()

    # -- limits --------------------------------------------------------------

    def _safe_limits(self, servo_id: int) -> tuple[int, int]:
        if servo_id not in self._limit_cache:
            lo = self._read_u16(servo_id, _ADDR_MIN_ANGLE_LIMIT)
            hi = self._read_u16(servo_id, _ADDR_MAX_ANGLE_LIMIT)
            if lo is None or hi is None:
                # don't cache a failed read; fall back for this call only
                return (_RANGE_MIN + _SAFETY_BUFFER, _RANGE_MAX - _SAFETY_BUFFER)
            if lo == 0 and hi == 0:
                self._limit_cache[servo_id] = (_RANGE_MIN + _SAFETY_BUFFER,
                                                _RANGE_MAX - _SAFETY_BUFFER)
            else:
                self._limit_cache[servo_id] = (lo + _SAFETY_BUFFER,
                                                hi - _SAFETY_BUFFER)
        return self._limit_cache[servo_id]

    def _clamp(self, servo_id: int, target: int) -> int:
        lo, hi = self._safe_limits(servo_id)
        return max(lo, min(hi, target))

    @staticmethod
    def _motion_block(target: int, speed: int, accel: int) -> bytes:
        """Registers 41..47 in one go: accel, goal position, goal time (0), goal speed."""
        speed = max(SPEED_MIN, min(SPEED_MAX, int(speed)))
        accel = max(ACCEL_MIN, min(ACCEL_MAX, int(accel)))
        target = max(_RANGE_MIN, min(_RANGE_MAX, int(target)))
        return bytes([accel & 0xFF, target & 0xFF, (target >> 8) & 0xFF, 0, 0,
                      speed & 0xFF, (speed >> 8) & 0xFF])

    # -- move ----------------------------------------------------------------

    def _move(self, servo_id: int, target: int, speed: int, accel: int, should_abort=None):
        # hold the lock only for each bus transaction, never for the whole move,
        # so other requests (and the IK stream) keep working while we wait
        with self._lock:
            target = self._clamp(servo_id, target)
            self._write_raw(servo_id, _ADDR_TORQUE_ENABLE, bytes([1]))
            self._write_raw(servo_id, _ADDR_ACCELERATION, self._motion_block(target, speed, accel))

        deadline = time.time() + _MOVE_SETTLE_TIMEOUT
        pos = None
        while time.time() < deadline:
            time.sleep(_MOVE_SETTLE_POLL)
            if should_abort and should_abort():
                break
            with self._lock:
                p = self._read_s16(servo_id, _ADDR_PRESENT_POSITION)
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
                if sid == _ATOM_ID:
                    continue                      # the ATOM answers WRITE-pings, not PING
                if self._bus._ping(sid, timeout=0.006):
                    ids.append(sid)
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

    def move(self, servo_id: int, target: int, speed: int = 600, accel: int = 20, should_abort=None):
        """Move a servo to an absolute position."""
        return self.servo(servo_id).move(target, speed, accel, should_abort)

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

    # -- streaming (used by the IK link) --------------------------------------

    def read_positions(self, ids: list[int]) -> list[int | None]:
        """Present position of each servo (signed ticks), None where a servo didn't answer.
        Takes the lock per servo so other callers can interleave."""
        out = []
        for sid in ids:
            with self._bus._lock:
                out.append(self._bus._read_s16(sid, _ADDR_PRESENT_POSITION))
        return out

    def sync_move(self, targets: dict[int, int], speed: int = 600, accel: int = 20):
        """Send goal position + speed + accel to several servos in ONE sync-write packet.
        Non-blocking (doesn't wait for arrival); every target is clamped to the safe limits."""
        with self._bus._lock:
            block = {sid: self._bus._motion_block(self._bus._clamp(sid, t), speed, accel)
                     for sid, t in targets.items()}
            self._bus._sync_write(_ADDR_ACCELERATION, block)

    def sync_torque(self, ids: list[int], enable: bool):
        """Torque on/off for several servos in one packet."""
        with self._bus._lock:
            self._bus._sync_write(_ADDR_TORQUE_ENABLE, {sid: bytes([1 if enable else 0]) for sid in ids})

    def hold(self, ids: list[int]) -> list[int | None]:
        """Stop where the arm is: set every goal to the present position (torque stays as it is).
        Returns the positions it held at."""
        now = self.read_positions(ids)
        with self._bus._lock:
            block = {sid: bytes([p & 0xFF, (p >> 8) & 0xFF])
                     for sid, p in zip(ids, now) if p is not None and 0 <= p <= _RANGE_MAX}
            self._bus._sync_write(_ADDR_GOAL_POSITION, block)
        return now

    def move_all(self, targets: dict[int, int], speed: int = 600, accel: int = 20,
                 wait: bool = True, timeout: float = _MOVE_SETTLE_TIMEOUT,
                 should_abort=None) -> dict[int, tuple[bool, int | None]]:
        """Move several servos together (one sync-write) and optionally wait until all arrive.

        ``should_abort`` is polled while waiting; returning True stops waiting early.
        Returns {id: (arrived, final_position)}."""
        with self._bus._lock:
            clamped = {sid: self._bus._clamp(sid, t) for sid, t in targets.items()}
            self._bus._sync_write(_ADDR_TORQUE_ENABLE, {sid: b"\x01" for sid in clamped})
            self._bus._sync_write(_ADDR_ACCELERATION,
                                  {sid: self._bus._motion_block(t, speed, accel) for sid, t in clamped.items()})
        ids = list(clamped)
        pos = dict.fromkeys(ids)
        deadline = time.time() + (timeout if wait else 0)
        while True:
            for sid, p in zip(ids, self.read_positions(ids)):
                if p is not None:
                    pos[sid] = p
            done = all(pos[s] is not None and abs(pos[s] - clamped[s]) <= _MOVE_TOLERANCE for s in ids)
            if done or not wait or time.time() > deadline or (should_abort and should_abort()):
                break
            time.sleep(_MOVE_SETTLE_POLL)
        return {s: (pos[s] is not None and abs(pos[s] - clamped[s]) <= _MOVE_TOLERANCE, pos[s]) for s in ids}

    def safe_limits(self, servo_id: int) -> tuple[int, int]:
        with self._bus._lock:
            return self._bus._safe_limits(servo_id)

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
