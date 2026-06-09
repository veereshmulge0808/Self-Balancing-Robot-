import { useEffect, useMemo, useRef, useState } from 'react'
import { Line } from 'react-chartjs-2'
import {
  Chart as ChartJS,
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Tooltip,
  Legend,
  Title,
} from 'chart.js'
import './App.css'

ChartJS.register(CategoryScale, LinearScale, PointElement, LineElement, Tooltip, Legend, Title)

const COMMANDS = {
  FORWARD: 'DRIVE_FORWARD',
  BACK: 'DRIVE_BACKWARD',
  LEFT: 'TURN_LEFT',
  RIGHT: 'TURN_RIGHT',
  STOP: 'STOP',
}

const initialTelemetry = {
  pitch: 0,
  velocity: 0,
  height: 0,
  uptime: 0,
  last_cmd: 'NONE',
  last_intent: 'NONE',
  last_confidence: 0,
  last_raw_text: '',
  last_vel_extracted: 0,
  last_height_extracted: 0,
  ble_connected: false,
  voice_active: false,
  enc_left: 0,
  enc_right: 0,
}

function App() {
  const [telemetry, setTelemetry] = useState(initialTelemetry)
  const [pitchHistory, setPitchHistory] = useState(Array(60).fill(0))
  const [velocityHistory, setVelocityHistory] = useState(Array(60).fill(0))
  const [speed, setSpeed] = useState(0.5)
  const [connected, setConnected] = useState(false)
  const [logs, setLogs] = useState([])
  const wsRef = useRef(null)
  const reconnectsRef = useRef(0)

  const chartData = useMemo(
    () => ({
      labels: pitchHistory.map((_, index) => index + 1),
      datasets: [
        {
          label: 'Pitch (°)',
          data: pitchHistory,
          fill: true,
          borderColor: '#48b5ff',
          backgroundColor: 'rgba(72, 181, 255, 0.15)',
          tension: 0.25,
          pointRadius: 0,
        },
      ],
    }),
    [pitchHistory],
  )

  const velocityData = useMemo(
    () => ({
      labels: velocityHistory.map((_, index) => index + 1),
      datasets: [
        {
          label: 'Velocity (m/s)',
          data: velocityHistory,
          fill: true,
          borderColor: '#f7b500',
          backgroundColor: 'rgba(247, 181, 0, 0.14)',
          tension: 0.25,
          pointRadius: 0,
        },
      ],
    }),
    [velocityHistory],
  )

  useEffect(() => {
    let active = true
    let ws

    const connect = () => {
      ws = new WebSocket('ws://localhost:8765')
      wsRef.current = ws

      ws.onopen = () => {
        if (!active) return
        setConnected(true)
      }

      ws.onmessage = (event) => {
        if (!active) return
        const data = JSON.parse(event.data)
        setTelemetry(data)

        setPitchHistory((prev) => {
          const next = [...prev, parseFloat(data.pitch ?? 0)]
          if (next.length > 60) next.shift()
          return next
        })
        setVelocityHistory((prev) => {
          const next = [...prev, parseFloat(data.velocity ?? 0)]
          if (next.length > 60) next.shift()
          return next
        })

        setLogs((prev) => {
          const entry = `${new Date().toLocaleTimeString()} · ${data.last_intent} → ${data.last_cmd}`
          return [entry, ...prev].slice(0, 30)
        })
      }

      ws.onclose = () => {
        if (!active) return
        setConnected(false)
        reconnectsRef.current += 1
        setTimeout(connect, 2000)
      }

      ws.onerror = () => {
        if (!active) return
        setConnected(false)
      }
    }

    connect()
    return () => {
      active = false
      wsRef.current?.close()
    }
  }, [])

  const sendCommand = (payload) => {
    const ws = wsRef.current
    if (ws?.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify(payload))
    }
  }

  const doMove = (command) => sendCommand({ cmd: COMMANDS[command] })

  const updateSpeed = (value) => {
    const newSpeed = parseFloat(value)
    setSpeed(newSpeed)
    sendCommand({ cmd: 'SPEED', val: newSpeed })
  }

  const toggleVoice = () => sendCommand({ cmd: 'VOICE_TOGGLE' })

  return (
    <div className="app-shell">
      <header className="topbar">
        <div className="brand">VoxBot BLE Control</div>
        <div className="status-row">
          <span className={`status-pill ${connected ? 'ok' : 'warn'}`}>{connected ? 'WS CONNECTED' : 'WS DISCONNECTED'}</span>
          <span className={`status-pill ${telemetry.ble_connected ? 'ok' : 'warn'}`}>{telemetry.ble_connected ? 'BLE CONNECTED' : 'BLE DISCONNECTED'}</span>
          <span className="status-mini">Uptime: {Math.floor(telemetry.uptime || 0)}s</span>
        </div>
      </header>

      <section className="grid-row">
        <div className="card">
          <p className="card-title">Pitch</p>
          <p className="card-value">{telemetry.pitch.toFixed(2)}°</p>
        </div>
        <div className="card">
          <p className="card-title">Velocity</p>
          <p className="card-value">{telemetry.velocity.toFixed(2)} m/s</p>
        </div>
        <div className="card">
          <p className="card-title">Height</p>
          <p className="card-value">{telemetry.height.toFixed(0)} mm</p>
        </div>
        <div className="card">
          <p className="card-title">Last command</p>
          <p className="card-value small">{telemetry.last_cmd}</p>
        </div>
      </section>

      <section className="chart-row">
        <div className="chart-card">
          <Line data={chartData} options={{ responsive: true, plugins: { legend: { display: false } }, scales: { x: { display: false }, y: { min: -20, max: 20 } } }} />
        </div>
        <div className="chart-card">
          <Line data={velocityData} options={{ responsive: true, plugins: { legend: { display: false } }, scales: { x: { display: false }, y: { min: -1, max: 1 } } }} />
        </div>
      </section>

      <section className="control-panel">
        <div className="dpad">
          <button className="btn" onClick={() => doMove('FORWARD')}>FORWARD</button>
          <button className="btn" onClick={() => doMove('LEFT')}>LEFT</button>
          <button className="btn stop" onClick={() => doMove('STOP')}>STOP</button>
          <button className="btn" onClick={() => doMove('RIGHT')}>RIGHT</button>
          <button className="btn" onClick={() => doMove('BACK')}>BACK</button>
        </div>

        <div className="slider-row">
          <label className="slider-label">
            Speed
            <input type="range" min="0.1" max="1.0" step="0.05" value={speed} onChange={(event) => updateSpeed(event.target.value)} />
          </label>
          <span>{speed.toFixed(2)}</span>
        </div>

        <button className={`btn voice-btn ${telemetry.voice_active ? 'voice-active' : ''}`} onClick={toggleVoice}>
          {telemetry.voice_active ? 'VOICE ACTIVE' : 'VOICE OFF'}
        </button>
      </section>

      <section className="log-panel">
        <div className="log-header">Recent Activity</div>
        <div className="log-list">
          {logs.map((entry, index) => (
            <div key={index} className="log-entry">{entry}</div>
          ))}
        </div>
      </section>
    </div>
  )
}

export default App
