"""A fake Feetech bus: six STS servos (IDs 1-6) and the reflashed ATOM (ID 7).

Stands in for ``serial.Serial`` so the library, the backend and the tools run without the arm.
It answers PING / READ / WRITE, applies SYNC WRITE, and moves each servo toward its goal with a
trapezoidal profile from its speed (steps/s) and acceleration (100 steps/s^2) registers, in real time.

Test hooks:
    bus.servos[sid].pos           present position (ticks); set it to move a limp joint "by hand"
    bus.servos[sid].stop_at       a tick the servo can't move past (an obstruction), or None
    bus.echo = True               the adapter echoes every request back before the reply
    bus.atom_log                  ATOM writes as (address, bytes)
    bus.alive                     set of IDs that answer
"""
import threading
import time

ADDR_MIN, ADDR_MAX, ADDR_TORQUE, ADDR_ACC, ADDR_GOAL, ADDR_SPEED, ADDR_POS = 9, 11, 40, 41, 42, 46, 56


def _chk(body):
    return (~sum(body)) & 0xFF


def _status(sid, params=b""):
    body = bytes([sid, len(params) + 2, 0]) + params
    return b"\xff\xff" + body + bytes([_chk(body)])


class FakeServo:
    def __init__(self, sid, lo, hi, pos=2048):
        self.id = sid
        self.regs = bytearray(256)
        self.regs[ADDR_MIN:ADDR_MIN + 2] = lo.to_bytes(2, "little")
        self.regs[ADDR_MAX:ADDR_MAX + 2] = hi.to_bytes(2, "little")
        self.regs[ADDR_TORQUE] = 1
        self.regs[ADDR_ACC] = 20
        self.regs[ADDR_SPEED:ADDR_SPEED + 2] = (600).to_bytes(2, "little")
        self.pos = float(pos)
        self.vel = 0.0
        self.goal = pos
        self.stop_at = None
        self.regs[ADDR_GOAL:ADDR_GOAL + 2] = pos.to_bytes(2, "little")

    def write(self, addr, data):
        self.regs[addr:addr + len(data)] = data
        if addr <= ADDR_GOAL + 1 and addr + len(data) > ADDR_GOAL:
            self.goal = int.from_bytes(self.regs[ADDR_GOAL:ADDR_GOAL + 2], "little")
        if addr <= ADDR_TORQUE < addr + len(data) and not self.regs[ADDR_TORQUE]:
            self.vel = 0.0

    def step(self, dt):
        if not self.regs[ADDR_TORQUE]:
            return
        speed = int.from_bytes(self.regs[ADDR_SPEED:ADDR_SPEED + 2], "little")
        acc = self.regs[ADDR_ACC] * 100
        assert speed > 0 and acc > 0, "speed/accel 0 means unlimited: never sent"
        err = self.goal - self.pos
        vdes = (1 if err > 0 else -1) * min(speed, (2 * acc * abs(err)) ** 0.5)
        dv = max(-acc * dt, min(acc * dt, vdes - self.vel))
        self.vel += dv
        step = self.vel * dt
        if abs(step) >= abs(err):
            step, self.vel = err, 0.0
        new = self.pos + step
        if self.stop_at is not None and (self.pos <= self.stop_at < new or new < self.stop_at <= self.pos):
            new, self.vel = float(self.stop_at), 0.0
        self.pos = new

    def read(self, addr, n):
        p = int(round(self.pos))
        self.regs[ADDR_POS:ADDR_POS + 2] = (abs(p) | (0x8000 if p < 0 else 0)).to_bytes(2, "little")
        return bytes(self.regs[addr:addr + n])


class FakeAtom:
    def __init__(self):
        self.pixels = [[0, 0, 0] for _ in range(25)]
        self.brightness = 64
        self.rgb = [0, 0, 0]

    def write(self, addr, data):
        if addr == 1 and len(data) == 3:
            self.rgb = list(data)
            self.pixels = [list(data) for _ in range(25)]
        elif addr == 2 and len(data) == 5 and data[0] < 5 and data[1] < 5:
            self.pixels[data[1] * 5 + data[0]] = list(data[2:])
        elif addr == 3 and len(data) == 1:
            self.brightness = data[0]
        elif addr == 4:
            return bytes([self.brightness] + self.rgb + [v for p in self.pixels for v in p])
        return b""


class FakeBus:
    """Drop-in for serial.Serial."""

    def __init__(self, echo=False):
        # J1-J5 with EEPROM limits, J6 reporting 0,0 like the real one
        self.servos = {sid: FakeServo(sid, 100, 3996) for sid in range(1, 6)}
        self.servos[6] = FakeServo(6, 0, 0)
        self.atom = FakeAtom()
        self.atom_log = []
        self.alive = set(range(1, 8))
        self.echo = echo
        self.is_open = True
        self._out = bytearray()
        self._lock = threading.Lock()
        self._t = time.monotonic()
        self.packets = 0

    # -- simulation -------------------------------------------------------------
    def _advance(self):
        now = time.monotonic()
        dt, self._t = now - self._t, now
        while dt > 0:
            h = min(dt, 0.005)
            for s in self.servos.values():
                s.step(h)
            dt -= h

    def settle(self, seconds):
        """Let the servos run for a while (real time)."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            with self._lock:
                self._advance()
            time.sleep(0.005)

    # -- serial.Serial interface ------------------------------------------------
    @property
    def in_waiting(self):
        return len(self._out)

    def read(self, n=1):
        with self._lock:
            data, self._out = bytes(self._out[:n]), self._out[n:]
        if not data:
            time.sleep(0.0002)
        return data

    def reset_input_buffer(self):
        with self._lock:
            self._out.clear()

    def flush(self):
        pass

    def close(self):
        self.is_open = False

    def write(self, pkt):
        pkt = bytes(pkt)
        with self._lock:
            self._advance()
            self.packets += 1
            if self.echo:
                self._out += pkt
            reply = self._handle(pkt)
            if reply:
                self._out += reply
        return len(pkt)

    def _handle(self, pkt):
        if len(pkt) < 6 or pkt[:2] != b"\xff\xff" or _chk(pkt[2:-1]) != pkt[-1]:
            return None
        sid, ln, ins, params = pkt[2], pkt[3], pkt[4], pkt[5:-1]
        if ln != len(params) + 2:
            return None
        if ins == 0x83 and sid == 0xFE:
            addr, n = params[0], params[1]
            body = params[2:]
            for k in range(0, len(body), n + 1):
                s = self.servos.get(body[k])
                if s and s.id in self.alive:
                    s.write(addr, body[k + 1:k + 1 + n])
            return None
        if sid not in self.alive:
            return None
        if sid == 7:
            if ins != 0x03:
                return None   # the ATOM only answers WRITE-style packets
            self.atom_log.append((params[0], bytes(params[1:])))
            return _status(7, self.atom.write(params[0], bytes(params[1:])))
        s = self.servos.get(sid)
        if s is None:
            return None
        if ins == 0x01:
            return _status(sid)
        if ins == 0x02:
            return _status(sid, s.read(params[0], params[1]))
        if ins == 0x03:
            s.write(params[0], bytes(params[1:]))
            return _status(sid)
        return None
