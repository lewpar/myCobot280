"""A /ws/arm client for the tests: logs in, keeps the latest config and state, fills in goal and target epochs."""
from conftest import PASSWORD


class ArmWS:
    def __init__(self, client, password=PASSWORD):
        self._cm = client.websocket_connect("/ws/arm")
        self.w = self._cm.__enter__()
        self.w.send_json({"type": "auth", "password": password})
        self.hello = self.w.receive_json()
        self.config = self.last = None
        self.errors = []

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return self._cm.__exit__(*exc)

    def close(self):
        self._cm.__exit__(None, None, None)

    def recv(self):
        m = self.w.receive_json()
        kind = m.get("type")
        if kind == "config":
            self.config = m
        elif kind == "state":
            self.last = m
        elif kind == "error":
            self.errors.append(m)
        return m

    def state(self, cond=lambda m: True, n=60):
        """The next state matching ``cond``."""
        for _ in range(n):
            m = self.recv()
            if m["type"] == "state" and cond(m):
                return m
        raise AssertionError(f"no matching state (last: {self.last})")

    def ready(self):
        """Wait until every servo reads back."""
        return self.state(lambda m: all(a is not None for a in m["angles"]))

    def wait_config(self, cond, n=60):
        for _ in range(n):
            if self.config and cond(self.config):
                return self.config
            self.recv()
        raise AssertionError(f"no matching config (last: {self.config})")

    def error(self, n=60):
        """The next error message."""
        for _ in range(n):
            m = self.recv()
            if m["type"] == "error":
                return m
        raise AssertionError("no error came back")

    def send(self, **m):
        self.w.send_json(m)

    def goal(self, angles, speed=120, acc=1000, epoch=None):
        if epoch is None:
            epoch = (self.last or self.state())["epoch"]
        self.send(type="goal", angles=list(angles), speed=speed, acc=acc, epoch=epoch)

    def target(self, xyz=None, angles=None, down=False, speed=120, acc=1000, epoch=None):
        if epoch is None:
            epoch = (self.last or self.state())["epoch"]
        m = {"xyz": list(xyz), "down": down} if xyz is not None else {"angles": list(angles)}
        self.send(type="target", speed=speed, acc=acc, epoch=epoch, **m)
