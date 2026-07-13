import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from contextlib import asynccontextmanager

from mycobot280 import MyCobot280

SERIAL_PORT = os.environ.get("MYCOBOT_PORT", "/dev/ttyAMA0")
SERIAL_BAUD = int(os.environ.get("MYCOBOT_BAUD", "1000000"))

arm: MyCobot280 | None = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    global arm
    try:
        arm = MyCobot280(SERIAL_PORT, SERIAL_BAUD)
    except Exception as e:
        print(f"WARNING: Could not open serial port {SERIAL_PORT}: {e}")
    yield
    if arm:
        arm.close()


app = FastAPI(title="MyCobot280 API", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


def _get_arm() -> MyCobot280:
    if arm is None:
        raise HTTPException(503, "Robot not connected — check serial port")
    return arm


# ---------------------------------------------------------------------------
# Pydantic models
# ---------------------------------------------------------------------------

class MoveRequest(BaseModel):
    position: int
    speed: int = 600
    accel: int = 20


class MoveRelRequest(BaseModel):
    delta: int
    speed: int = 600
    accel: int = 20


class TorqueRequest(BaseModel):
    enabled: bool


class CenterRequest(BaseModel):
    position: int = 2048
    speed: int = 600
    accel: int = 20


class ColorRequest(BaseModel):
    r: int = 0
    g: int = 0
    b: int = 0


class PixelRequest(BaseModel):
    x: int = 0
    y: int = 0
    r: int = 255
    g: int = 0
    b: int = 0


# ---------------------------------------------------------------------------
# Health
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


# ---------------------------------------------------------------------------
# Servos
# ---------------------------------------------------------------------------

@app.get("/api/servos")
def list_servos():
    a = _get_arm()
    ids = a.scan()
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
    ok, pos = a.move(servo_id, req.position, req.speed, req.accel)
    return {"success": ok, "id": servo_id, "position": pos, "target": req.position}


@app.post("/api/servo/{servo_id}/move_rel")
def servo_move_rel(servo_id: int, req: MoveRelRequest):
    a = _get_arm()
    ok, pos = a.move_rel(servo_id, req.delta, req.speed, req.accel)
    return {"success": ok, "id": servo_id, "position": pos}


@app.post("/api/servo/{servo_id}/torque")
def servo_torque(servo_id: int, req: TorqueRequest):
    a = _get_arm()
    a.set_torque(servo_id, req.enabled)
    return {"success": True, "id": servo_id, "torque_enabled": req.enabled}


@app.post("/api/servo/{servo_id}/center")
def servo_center(servo_id: int, req: CenterRequest):
    a = _get_arm()
    ok, pos = a.center(servo_id, req.position, req.speed, req.accel)
    return {"success": ok, "id": servo_id, "position": pos}


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
    a.atom.set_color(req.r, req.g, req.b)
    return {"success": True}


@app.post("/api/atom/pixel")
def atom_pixel(req: PixelRequest):
    a = _get_arm()
    a.atom.pixel(req.x, req.y, req.r, req.g, req.b)
    return {"success": True}


@app.post("/api/atom/ping")
def atom_ping():
    a = _get_arm()
    ok = a.atom.ping()
    return {"success": ok, "alive": ok}


# ---------------------------------------------------------------------------
# All servos quick scan (no limits, faster)
# ---------------------------------------------------------------------------

@app.get("/api/servos/status")
def servos_status():
    a = _get_arm()
    ids = a.servo_ids
    result = []
    for sid in ids:
        pos = a.get_position(sid)
        result.append({"id": sid, "position": pos})
    return result


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
