const BASE = '/api'

async function request(url, options = {}) {
  const res = await fetch(`${BASE}${url}`, {
    headers: { 'Content-Type': 'application/json' },
    ...options,
  })
  if (!res.ok) {
    const text = await res.text()
    throw new Error(text || res.statusText)
  }
  return res.json()
}

export function getHealth() {
  return request('/health')
}

export function listServos() {
  return request('/servos')
}

export function getServoStatus() {
  return request('/servos/status')
}

export function getServo(id) {
  return request(`/servo/${id}`)
}

export function moveServo(id, position, speed = 600, accel = 20) {
  return request(`/servo/${id}/move`, {
    method: 'POST',
    body: JSON.stringify({ position, speed, accel }),
  })
}

export function moveServoRel(id, delta, speed = 600, accel = 20) {
  return request(`/servo/${id}/move_rel`, {
    method: 'POST',
    body: JSON.stringify({ delta, speed, accel }),
  })
}

export function setTorque(id, enabled) {
  return request(`/servo/${id}/torque`, {
    method: 'POST',
    body: JSON.stringify({ enabled }),
  })
}

export function centerServo(id, position = 2048, speed = 600, accel = 20) {
  return request(`/servo/${id}/center`, {
    method: 'POST',
    body: JSON.stringify({ position, speed, accel }),
  })
}

export function pingServo(id) {
  return request(`/servo/${id}/ping`, { method: 'POST' })
}

export function setAtomColor(r, g, b) {
  return request('/atom/color', {
    method: 'POST',
    body: JSON.stringify({ r, g, b }),
  })
}

export function setAtomPixel(x, y, r, g, b) {
  return request('/atom/pixel', {
    method: 'POST',
    body: JSON.stringify({ x, y, r, g, b }),
  })
}

export function pingAtom() {
  return request('/atom/ping', { method: 'POST' })
}

export function setAtomBrightness(percent) {
  return request('/atom/brightness', {
    method: 'POST',
    body: JSON.stringify({ percent }),
  })
}
