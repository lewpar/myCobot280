import { useState, useEffect, useCallback } from 'react'
import * as api from './api'
import './App.css'

const SERVO_IDS = [1, 2, 3, 4, 5, 6]
const DEFAULT_SPEED = 600
const JOG_STEP = 100

function App() {
  const [connected, setConnected] = useState(false)
  const [servos, setServos] = useState({})
  const [error, setError] = useState('')
  const [loading, setLoading] = useState({})
  const [color, setColor] = useState('#ff0000')
  const [selectedPixel, setSelectedPixel] = useState(null)
  const [brightness, setBrightness] = useState(50)

  const fetchServos = useCallback(async () => {
    try {
      const status = await api.getServoStatus()
      const map = {}
      for (const s of status) {
        map[s.id] = { position: s.position }
      }
      setServos(prev => {
        const next = { ...prev }
        for (const id of SERVO_IDS) {
          next[id] = { ...next[id], ...map[id] }
        }
        return next
      })
    } catch (e) {
      // silent poll error
    }
  }, [])

  useEffect(() => {
    let running = true
    const check = async () => {
      try {
        const h = await api.getHealth()
        if (!running) return
        setConnected(h.connected)
        if (h.connected) {
          const status = await api.getServoStatus()
          if (!running) return
          const map = {}
          for (const s of status) {
            map[s.id] = { position: s.position }
          }
          const full = {}
          for (const id of SERVO_IDS) {
            full[id] = { ...map[id] }
          }
          setServos(full)
        }
      } catch (e) {
        if (running) setConnected(false)
      }
    }
    check()
    const interval = setInterval(fetchServos, 1000)
    return () => { running = false; clearInterval(interval) }
  }, [fetchServos])

  const doAction = async (label, fn) => {
    setLoading(prev => ({ ...prev, [label]: true }))
    setError('')
    try {
      await fn()
      await fetchServos()
    } catch (e) {
      setError(e.message)
    } finally {
      setLoading(prev => ({ ...prev, [label]: false }))
    }
  }

  // --- servo helpers ---

  const handleMove = (id, pos) => {
    doAction(`move-${id}`, () => api.moveServo(id, pos))
  }

  const handleJog = (id, delta) => {
    doAction(`jog-${id}`, () => api.moveServoRel(id, delta))
  }

  const handleCenter = id => {
    doAction(`center-${id}`, () => api.centerServo(id))
  }

  const handleTorque = (id, enabled) => {
    doAction(`torque-${id}`, () => api.setTorque(id, enabled))
    setServos(prev => ({
      ...prev,
      [id]: { ...prev[id], torque: enabled },
    }))
  }

  const handleScan = () => {
    doAction('scan', async () => {
      const list = await api.listServos()
      const map = {}
      for (const s of list) {
        map[s.id] = { position: s.position }
      }
      setServos(map)
    })
  }

  // --- atom helpers ---

  const handleColorSet = () => {
    const r = parseInt(color.slice(1, 3), 16)
    const g = parseInt(color.slice(3, 5), 16)
    const b = parseInt(color.slice(5, 7), 16)
    doAction('atom-color', () => api.setAtomColor(r, g, b))
  }

  const handlePixelSet = (x, y) => {
    const r = parseInt(color.slice(1, 3), 16)
    const g = parseInt(color.slice(3, 5), 16)
    const b = parseInt(color.slice(5, 7), 16)
    doAction(`pixel-${x}-${y}`, () => api.setAtomPixel(x, y, r, g, b))
  }

  const handleBrightness = (pct) => {
    setBrightness(pct)
    doAction('brightness', () => api.setAtomBrightness(pct))
  }

  return (
    <div className="app">
      {/* Header */}
      <header className="header">
        <h1>MyCobot280</h1>
        <div className="status-row">
          <span className={`status-dot ${connected ? 'on' : 'off'}`} />
          <span>{connected ? 'Connected' : 'Disconnected'}</span>
          <button className="btn btn-sm" onClick={handleScan} disabled={loading['scan']}>
            {loading['scan'] ? 'Scanning...' : 'Scan'}
          </button>
        </div>
        {error && <div className="error">{error}</div>}
      </header>

      {/* Servo grid */}
      <section className="section">
        <h2>Servos</h2>
        <div className="servo-grid">
          {SERVO_IDS.map(id => {
            const s = servos[id] || {}
            const pos = s.position
            return (
              <div key={id} className="servo-card">
                <div className="servo-header">
                  <span className="servo-label">Joint {id}</span>
                  <span className="servo-pos">
                    {pos != null ? pos : '---'}
                  </span>
                </div>

                <input
                  type="range"
                  min={50}
                  max={4045}
                  value={pos ?? 2048}
                  onChange={e => {
                    const v = parseInt(e.target.value, 10)
                    setServos(prev => ({
                      ...prev,
                      [id]: { ...prev[id], position: v },
                    }))
                  }}
                  onMouseUp={() => {
                    if (pos != null) handleMove(id, pos)
                  }}
                  onTouchEnd={() => {
                    if (pos != null) handleMove(id, pos)
                  }}
                  className="slider"
                  disabled={!connected}
                />

                <div className="servo-actions">
                  <button
                    className="btn"
                    onClick={() => handleJog(id, -JOG_STEP)}
                    disabled={!connected || loading[`jog-${id}`]}
                  >
                    −{JOG_STEP}
                  </button>
                  <button
                    className="btn"
                    onClick={() => handleJog(id, JOG_STEP)}
                    disabled={!connected || loading[`jog-${id}`]}
                  >
                    +{JOG_STEP}
                  </button>
                  <button
                    className="btn btn-accent"
                    onClick={() => handleMove(id, pos ?? 2048)}
                    disabled={!connected || loading[`move-${id}`]}
                  >
                    Go
                  </button>
                  <button
                    className="btn"
                    onClick={() => handleCenter(id)}
                    disabled={!connected || loading[`center-${id}`]}
                  >
                    Center
                  </button>
                  <button
                    className={`btn ${s.torque !== false ? 'btn-on' : 'btn-off'}`}
                    onClick={() => handleTorque(id, s.torque === false)}
                    disabled={!connected || loading[`torque-${id}`]}
                  >
                    {s.torque === false ? 'Torque On' : 'Torque Off'}
                  </button>
                </div>
              </div>
            )
          })}
        </div>
      </section>

      {/* ATOM LED controls */}
      <section className="section">
        <h2>ATOM LED Matrix</h2>
        <div className="atom-controls">
          <div className="color-picker-group">
            <input
              type="color"
              value={color}
              onChange={e => setColor(e.target.value)}
              className="color-input"
            />
            <button
              className="btn btn-accent"
              onClick={handleColorSet}
              disabled={!connected || loading['atom-color']}
            >
              Set All LEDs
            </button>
          </div>

          <div className="brightness-group">
            <label className="brightness-label">
              Brightness: {brightness}%
            </label>
            <input
              type="range"
              min={1}
              max={100}
              value={brightness}
              onChange={e => {
                const v = parseInt(e.target.value, 10)
                setBrightness(v)
              }}
              onMouseUp={() => handleBrightness(brightness)}
              onTouchEnd={() => handleBrightness(brightness)}
              className="slider"
              disabled={!connected}
            />
            <button
              className="btn btn-sm"
              onClick={() => handleBrightness(brightness)}
              disabled={!connected || loading['brightness']}
            >
              Set
            </button>
          </div>

          <div className="pixel-grid-section">
            <p className="hint">Click a pixel to set it to the selected colour</p>
            <div className="pixel-grid">
              {Array.from({ length: 5 }, (_, y) => (
                <div key={y} className="pixel-row">
                  {Array.from({ length: 5 }, (_, x) => (
                    <button
                      key={x}
                      className={`pixel ${selectedPixel?.x === x && selectedPixel?.y === y ? 'selected' : ''}`}
                      style={{ backgroundColor: color }}
                      onClick={() => {
                        setSelectedPixel({ x, y })
                        handlePixelSet(x, y)
                      }}
                      title={`(${x},${y})`}
                    />
                  ))}
                </div>
              ))}
            </div>
          </div>
        </div>
      </section>
    </div>
  )
}

export default App
