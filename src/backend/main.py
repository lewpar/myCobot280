import asyncio
import hmac
import json
import os
import secrets
import sys
import time
from collections import defaultdict, deque
from contextlib import asynccontextmanager

from dotenv import load_dotenv
load_dotenv(os.path.join(os.path.dirname(__file__), ".env"))

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse
from pydantic import BaseModel, Field

import arm_model as model
from mycobot280 import MyCobot280, SPEED_MIN, SPEED_MAX, ACCEL_MIN, ACCEL_MAX
from ik_link import IKLink

SERIAL_PORT = os.environ.get("MYCOBOT_PORT", "/dev/ttyAMA0")
SERIAL_BAUD = int(os.environ.get("MYCOBOT_BAUD", "1000000"))
CORS_ORIGINS = os.environ.get("MYCOBOT_CORS_ORIGINS", "*")

# ---------------------------------------------------------------------------
# Password
# ---------------------------------------------------------------------------
# Every /api request must carry the password in the X-Arm-Password header. Browsers can't set
# headers on a WebSocket, so /ws/arm expects {"type": "auth", "password": ...} as its first message.
# Note this is plain HTTP: the password protects against casual use on the network, not against
# someone capturing traffic. Put the backend behind HTTPS if that matters.
PASSWORD = os.environ.get("MYCOBOT_PASSWORD", "")
if not PASSWORD:
    PASSWORD = secrets.token_urlsafe(9)
    print("\n" + "=" * 64)
    print(f"  No MYCOBOT_PASSWORD set. Password for this run:  {PASSWORD}")
    print("  Set MYCOBOT_PASSWORD in src/backend/.env to choose your own.")
    print("=" * 64 + "\n", flush=True)

AUTH_HEADER = "X-Arm-Password"
FAIL_WINDOW, FAIL_LIMIT = 60.0, 5
_failures: dict[str, deque] = defaultdict(deque)


def _password_ok(given: str | None) -> bool:
    return given is not None and hmac.compare_digest(given.encode(), PASSWORD.encode())


def _locked_out(ip: str) -> bool:
    q = _failures[ip]
    while q and time.monotonic() - q[0] > FAIL_WINDOW:
        q.popleft()
    return len(q) >= FAIL_LIMIT


async def _record_failure(ip: str):
    _failures[ip].append(time.monotonic())
    await asyncio.sleep(0.5)   # slows down guessing


# ---------------------------------------------------------------------------
# App
# ---------------------------------------------------------------------------

arm: MyCobot280 | None = None
link: IKLink | None = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    global arm, link
    try:
        arm = MyCobot280(SERIAL_PORT, SERIAL_BAUD)
        link = IKLink(arm)
    except Exception as e:
        print(f"WARNING: Could not open serial port {SERIAL_PORT}: {e}")
    yield
    if arm:
        arm.close()


app = FastAPI(title="MyCobot280 API", lifespan=lifespan)


@app.middleware("http")
async def require_password(request: Request, call_next):
    if request.url.path.startswith("/api") and request.method != "OPTIONS":
        ip = request.client.host if request.client else "?"
        if _locked_out(ip):
            return JSONResponse({"detail": "Too many wrong passwords. Wait a minute and try again."},
                                status_code=429)
        if not _password_ok(request.headers.get(AUTH_HEADER)):
            await _record_failure(ip)
            return JSONResponse({"detail": "Password required"}, status_code=401)
    return await call_next(request)


# Added after the password check so it wraps it: refusals (401/429) still carry CORS headers and a
# page on another origin sees the real error instead of a network failure.
origins = CORS_ORIGINS.split(",") if CORS_ORIGINS != "*" else ["*"]
app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
)


def _get_arm() -> MyCobot280:
    if arm is None:
        raise HTTPException(503, "Robot not connected — check serial port")
    return arm


def _guard_motion(new_ticks: dict[int, int]):
    """Refuse a move while stopped or if it would collide. ``new_ticks`` maps servo id -> target."""
    if link.stopped:
        raise HTTPException(423, "The arm is stopped. Resume it before moving.")
    targets = [new_ticks.get(sid) for sid in model.JOINT_IDS]
    if any(sid not in model.JOINT_IDS for sid in new_ticks):
        return   # not an arm joint: nothing to check
    why = link.check_ticks(targets)
    if why:
        raise HTTPException(409, f"Move refused: {why}.")


def _aborted():
    return link is not None and link.stopped


# ---------------------------------------------------------------------------
# Pydantic models (ranges match the servo registers, so bad values are rejected, not wrapped)
# ---------------------------------------------------------------------------

Speed = Field(600, ge=SPEED_MIN, le=SPEED_MAX, description="steps/s")
Accel = Field(20, ge=ACCEL_MIN, le=ACCEL_MAX, description="100 steps/s² per unit")


class MoveRequest(BaseModel):
    position: int = Field(ge=0, le=4095)
    speed: int = Speed
    accel: int = Accel


class MoveRelRequest(BaseModel):
    delta: int = Field(ge=-4095, le=4095)
    speed: int = Speed
    accel: int = Accel


class TorqueRequest(BaseModel):
    enabled: bool


class CenterRequest(BaseModel):
    position: int = Field(2048, ge=0, le=4095)
    speed: int = Speed
    accel: int = Accel


class ColorRequest(BaseModel):
    r: int = Field(0, ge=0, le=255)
    g: int = Field(0, ge=0, le=255)
    b: int = Field(0, ge=0, le=255)


class PixelRequest(BaseModel):
    x: int = Field(0, ge=0, le=4)
    y: int = Field(0, ge=0, le=4)
    r: int = Field(255, ge=0, le=255)
    g: int = Field(0, ge=0, le=255)
    b: int = Field(0, ge=0, le=255)


class BrightnessRequest(BaseModel):
    percent: int = Field(ge=1, le=100)


# ---------------------------------------------------------------------------
# Health, auth, safety
# ---------------------------------------------------------------------------

@app.get("/api/health")
def health():
    connected = arm is not None
    servos = arm.servo_ids if connected else []
    return {
        "connected": connected,
        "servo_count": len(servos),
        "servo_ids": servos,
        "serial_port": SERIAL_PORT,
    }


@app.get("/api/auth")
def auth_check():
    """200 if the X-Arm-Password header is right (the middleware already checked it)."""
    return {"ok": True}


@app.get("/api/safety")
def safety_state():
    return {"stopped": bool(link and link.stopped),
            "calibrated": bool(link and link.calib["calibrated"])}


@app.post("/api/stop")
def stop():
    """Hold every joint where it is and refuse motion until /api/resume."""
    _get_arm()
    link.stop()
    return {"success": True, "stopped": True}


@app.post("/api/resume")
def resume():
    _get_arm()
    link.resume()
    return {"success": True, "stopped": False}


# ---------------------------------------------------------------------------
# Servos
# ---------------------------------------------------------------------------

@app.get("/api/servos")
def list_servos(rescan: bool = False):
    # clients may poll this often; re-scanning the whole bus each time kept the bus
    # locked for seconds, so use the cached IDs unless a rescan is asked for
    a = _get_arm()
    ids = a.scan() if rescan or not a.servo_ids else a.servo_ids
    result = []
    for sid in ids:
        s = a.servo(sid)
        pos = s.position
        limits = s.limits
        result.append({
            "id": sid,
            "position": pos,
            "limits_min": limits[0],
            "limits_max": limits[1],
        })
    return result


@app.get("/api/servo/{servo_id}")
def get_servo(servo_id: int):
    a = _get_arm()
    s = a.servo(servo_id)
    pos = s.position
    if pos is None:
        raise HTTPException(502, f"Servo {servo_id} did not respond")
    limits = s.limits
    return {
        "id": servo_id,
        "position": pos,
        "limits_min": limits[0],
        "limits_max": limits[1],
    }


@app.post("/api/servo/{servo_id}/move")
def servo_move(servo_id: int, req: MoveRequest):
    a = _get_arm()
    lo, hi = a.safe_limits(servo_id)
    target = max(lo, min(hi, req.position))
    _guard_motion({servo_id: target})
    ok, pos = a.move(servo_id, target, req.speed, req.accel, should_abort=_aborted)
    return {"success": ok, "id": servo_id, "position": pos, "target": req.position}


@app.post("/api/servo/{servo_id}/move_rel")
def servo_move_rel(servo_id: int, req: MoveRelRequest):
    a = _get_arm()
    cur = a.get_position(servo_id)
    if cur is None:
        raise HTTPException(502, f"Servo {servo_id} did not respond")
    lo, hi = a.safe_limits(servo_id)
    target = max(lo, min(hi, cur + req.delta))
    _guard_motion({servo_id: target})
    ok, pos = a.move(servo_id, target, req.speed, req.accel, should_abort=_aborted)
    return {"success": ok, "id": servo_id, "position": pos}


@app.post("/api/servo/{servo_id}/torque")
def servo_torque(servo_id: int, req: TorqueRequest):
    a = _get_arm()
    a.set_torque(servo_id, req.enabled)
    return {"success": True, "id": servo_id, "torque_enabled": req.enabled}


@app.post("/api/servo/{servo_id}/center")
def servo_center(servo_id: int, req: CenterRequest):
    return servo_move(servo_id, MoveRequest(position=req.position, speed=req.speed, accel=req.accel))


@app.post("/api/servo/{servo_id}/ping")
def servo_ping(servo_id: int):
    a = _get_arm()
    ok = a.servo_ping(servo_id)
    return {"success": ok, "id": servo_id, "alive": ok}


# ---------------------------------------------------------------------------
# ATOM
# ---------------------------------------------------------------------------

@app.post("/api/atom/color")
def atom_color(req: ColorRequest):
    a = _get_arm()
    # acked=False: sent, but the ATOM didn't reply (it may still have changed its LEDs)
    return {"success": True, "acked": a.atom.set_color(req.r, req.g, req.b)}


@app.post("/api/atom/pixel")
def atom_pixel(req: PixelRequest):
    a = _get_arm()
    return {"success": True, "acked": a.atom.pixel(req.x, req.y, req.r, req.g, req.b)}


@app.post("/api/atom/ping")
def atom_ping():
    a = _get_arm()
    ok = a.atom.ping()
    return {"success": ok, "alive": ok}


@app.post("/api/atom/brightness")
def atom_brightness(req: BrightnessRequest):
    a = _get_arm()
    return {"success": True, "acked": a.atom.set_brightness(req.percent), "percent": req.percent}


@app.get("/api/atom/state")
def atom_get_state():
    a = _get_arm()
    state = a.atom.get_led_state()
    if state is None:
        raise HTTPException(502, "ATOM did not respond")
    return {"success": True, **state}


# ---------------------------------------------------------------------------
# All servos
# ---------------------------------------------------------------------------

@app.get("/api/servos/status")
def servos_status():
    a = _get_arm()
    ids = a.servo_ids
    return [{"id": sid, "position": pos} for sid, pos in zip(ids, a.read_positions(ids))]


# Home positions (raw ticks) live in center_positions.json.
@app.get("/api/servos/home")
def get_home_positions():
    return {"home": model.load_centers()}


@app.post("/api/servos/home")
def set_home_positions():
    a = _get_arm()
    ids = sorted(a.servo_ids)
    home = {sid: pos for sid, pos in zip(ids, a.read_positions(ids)) if pos is not None}
    centers = model.load_centers()
    centers.update(home)
    model.save_centers(centers)
    return {"success": True, "home": centers}


@app.post("/api/servos/center_all")
def center_all_servos():
    """Move every joint to its home position together, after checking the path is clear."""
    a = _get_arm()
    centers = model.load_centers()
    targets = {}
    for sid in sorted(a.servo_ids):
        lo, hi = a.safe_limits(sid)
        targets[sid] = max(lo, min(hi, centers.get(sid, 2048)))
    _guard_motion({sid: t for sid, t in targets.items() if sid in model.JOINT_IDS})
    results = a.move_all(targets, should_abort=_aborted)
    return {"success": all(ok for ok, _ in results.values()),
            "servos": [{"id": sid, "success": ok, "position": pos, "target": targets[sid]}
                       for sid, (ok, pos) in results.items()]}


@app.post("/api/servos/torque_all")
def torque_all_servos(req: TorqueRequest):
    a = _get_arm()
    a.sync_torque(sorted(a.servo_ids), req.enabled)
    return {"success": True, "enabled": req.enabled}


# ---------------------------------------------------------------------------
# IK simulator page, served from here so it can reach /ws/arm on the same host.
# The page itself holds no secrets; it asks for the password before connecting.
# ---------------------------------------------------------------------------

@app.get("/sim")
def ik_sim_page():
    return FileResponse(os.path.join(os.path.dirname(__file__), "static", "ik_sim.html"))


# ---------------------------------------------------------------------------
# Live IK link (the simulator page connects here)
# ---------------------------------------------------------------------------

@app.websocket("/ws/arm")
async def ws_arm(ws: WebSocket):
    await ws.accept()
    ip = ws.client.host if ws.client else "?"
    if _locked_out(ip):
        await ws.send_json({"type": "error", "code": "locked",
                            "message": "Too many wrong passwords. Wait a minute and try again."})
        await ws.close(code=4429)
        return
    try:
        first = json.loads(await asyncio.wait_for(ws.receive_text(), timeout=5))
    except (asyncio.TimeoutError, ValueError, WebSocketDisconnect):
        first = {}
    if not (isinstance(first, dict) and first.get("type") == "auth" and _password_ok(first.get("password"))):
        await _record_failure(ip)
        try:
            await ws.send_json({"type": "error", "code": "auth", "message": "Wrong or missing password."})
            await ws.close(code=4401)
        except Exception:
            pass
        return
    if link is None:
        await ws.send_json({"type": "error", "code": "no_arm", "message": f"Robot not connected on {SERIAL_PORT}"})
        await ws.close()
        return
    link.add_client()

    async def sender():
        while True:
            await asyncio.sleep(0.1)
            state = await asyncio.to_thread(link.state)
            await ws.send_json(state)

    task = asyncio.create_task(sender())
    try:
        while True:
            raw = await ws.receive_text()
            try:
                msg = json.loads(raw)
            except ValueError:
                continue
            if isinstance(msg, dict):
                await asyncio.to_thread(link.handle, msg)
    except WebSocketDisconnect:
        pass
    finally:
        task.cancel()
        link.remove_client()


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
