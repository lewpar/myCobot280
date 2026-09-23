const BASE = import.meta.env.VITE_API_URL || '/api'
const KEY = 'mycobot-password'

// The backend wants the password in this header on every request.
let password = readStored()
let onAuthFailure = () => {}

function readStored() {
  try {
    return sessionStorage.getItem(KEY) || localStorage.getItem(KEY) || ''
  } catch {
    return ''
  }
}

export function setPassword(value, remember = false) {
  password = value
  try {
    sessionStorage.setItem(KEY, value)
    if (remember) localStorage.setItem(KEY, value)
    else localStorage.removeItem(KEY)
  } catch {
    // storage unavailable: keep it in memory for this page only
  }
}

export function clearPassword() {
  password = ''
  try {
    sessionStorage.removeItem(KEY)
    localStorage.removeItem(KEY)
  } catch {
    // ignore
  }
}

export function hasPassword() {
  return password !== ''
}

/** Called whenever the backend rejects the password, e.g. to show the sign-in screen again. */
export function setAuthFailureHandler(fn) {
  onAuthFailure = fn
}

export class AuthError extends Error {}

async function request(url, options = {}) {
  const res = await fetch(`${BASE}${url}`, {
    ...options,
    headers: { 'Content-Type': 'application/json', 'X-Arm-Password': password, ...(options.headers || {}) },
  })
  if (res.status === 401) {
    clearPassword()
    onAuthFailure()
    throw new AuthError('Wrong or missing password')
  }
  if (!res.ok) {
    const text = await res.text()
    let message = text || res.statusText
    try {
      const body = JSON.parse(text)
      if (typeof body.detail === 'string') message = body.detail
      else if (Array.isArray(body.detail)) {
        // FastAPI validation errors: "speed: Input should be less than or equal to 4000"
        message = body.detail.map(d => `${(d.loc || []).slice(-1)[0]}: ${d.msg}`).join('; ')
      }
    } catch {
      // not JSON: keep the raw text
    }
    throw new Error(message)
  }
  return res.json()
}

export function checkAuth() {
  return request('/auth')
}

export function getSafety() {
  return request('/safety')
}

export function stopArm() {
  return request('/stop', { method: 'POST' })
}

export function resumeArm() {
  return request('/resume', { method: 'POST' })
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

export function centerAllServos() {
  return request('/servos/center_all', { method: 'POST' })
}

export function getHomePositions() {
  return request('/servos/home')
}

export function setHomeAll() {
  return request('/servos/home', { method: 'POST' })
}

export function torqueAllServos(enabled) {
  return request('/servos/torque_all', { method: 'POST', body: JSON.stringify({ enabled }) })
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

export function getAtomState() {
  return request('/atom/state')
}
