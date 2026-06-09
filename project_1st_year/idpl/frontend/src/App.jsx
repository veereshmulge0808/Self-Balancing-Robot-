import { useEffect, useMemo, useRef, useState, useCallback } from 'react'
import { Line } from 'react-chartjs-2'
import {
  Chart as ChartJS,
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Filler,
  Tooltip,
} from 'chart.js'
import './App.css'

ChartJS.register(CategoryScale, LinearScale, PointElement, LineElement, Filler, Tooltip)

/* ── Firmware command set (matches parseCommand in Voxbot.ino) ─── */
const COMMANDS = {
  FORWARD:  'DRIVE_FORWARD',
  BACK:     'DRIVE_BACKWARD',
  LEFT:     'TURN_LEFT',
  RIGHT:    'TURN_RIGHT',
  STOP:     'STOP',
}

/* ── Initial telemetry — mirrors ws_server.py send_loop fields ─── */
const initialTelemetry = {
  pitch:              0,
  pwm:                0,
  uptime:             0,
  last_cmd:           'NONE',
  last_intent:        'NONE',
  last_confidence:    0,
  last_raw_text:      '',
  last_vel_extracted: 0.5,
  ble_connected:      false,
  voice_active:       false,
}

/* ── Arrow icons (inline SVGs, no deps) ──────────────────────── */
const ArrowUp    = () => <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5" strokeLinecap="round" strokeLinejoin="round"><path d="M12 5v14M5 12l7-7 7 7"/></svg>
const ArrowDown  = () => <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5" strokeLinecap="round" strokeLinejoin="round"><path d="M12 19V5M5 12l7 7 7-7"/></svg>
const ArrowLeft  = () => <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5" strokeLinecap="round" strokeLinejoin="round"><path d="M5 12h14M12 5l-7 7 7 7"/></svg>
const ArrowRight = () => <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2.5" strokeLinecap="round" strokeLinejoin="round"><path d="M19 12H5M12 5l7 7-7 7"/></svg>
const StopIcon   = () => <svg viewBox="0 0 24 24" fill="currentColor"><rect x="6" y="6" width="12" height="12" rx="2"/></svg>

/* ── Helpers ──────────────────────────────────────────────────── */
function formatUptime(seconds) {
  const s = Math.floor(seconds || 0)
  const m = Math.floor(s / 60)
  const h = Math.floor(m / 60)
  if (h > 0) return `${h}h ${m % 60}m`
  if (m > 0) return `${m}m ${s % 60}s`
  return `${s}s`
}

const HISTORY_LEN = 80

function App() {
  const [telemetry, setTelemetry]       = useState(initialTelemetry)
  const [pitchHistory, setPitchHistory] = useState(Array(HISTORY_LEN).fill(0))
  const [pwmHistory, setPwmHistory]     = useState(Array(HISTORY_LEN).fill(0))
  const [speed, setSpeed]               = useState(0.5)
  const [wsConnected, setWsConnected]   = useState(false)
  const [logs, setLogs]                 = useState([])
  const [nlpText, setNlpText]           = useState('')
  const wsRef = useRef(null)

  /* ── Chart configs ────────────────────────────────────────────── */
  const chartOpts = useCallback((label, min, max, color) => ({
    responsive: true,
    maintainAspectRatio: false,
    animation: false,
    plugins: {
      tooltip: { enabled: true },
      legend: { display: false },
    },
    scales: {
      x: { display: false },
      y: {
        min, max,
        ticks: {
          font: { family: "'Space Mono', monospace", size: 10 },
          color: '#9a9a9a',
          maxTicksLimit: 5,
        },
        grid: { color: 'rgba(0,0,0,0.04)', drawBorder: false },
        border: { display: false },
      },
    },
    elements: {
      line: { borderWidth: 2, borderColor: color, tension: 0.3, fill: true },
      point: { radius: 0 },
    },
  }), [])

  const pitchChartData = useMemo(() => ({
    labels: pitchHistory.map((_, i) => i),
    datasets: [{
      data: pitchHistory,
      borderColor: '#6c47ff',
      backgroundColor: 'rgba(108, 71, 255, 0.06)',
      fill: true,
    }],
  }), [pitchHistory])

  const pwmChartData = useMemo(() => ({
    labels: pwmHistory.map((_, i) => i),
    datasets: [{
      data: pwmHistory,
      borderColor: '#3b82f6',
      backgroundColor: 'rgba(59, 130, 246, 0.06)',
      fill: true,
    }],
  }), [pwmHistory])

  /* ── WebSocket ────────────────────────────────────────────────── */
  useEffect(() => {
    let active = true
    let ws

    const connect = () => {
      ws = new WebSocket('ws://localhost:8765')
      wsRef.current = ws

      ws.onopen = () => {
        if (!active) return
        setWsConnected(true)
      }

      ws.onmessage = (event) => {
        if (!active) return
        const data = JSON.parse(event.data)
        setTelemetry(data)

        setPitchHistory(prev => {
          const next = [...prev, parseFloat(data.pitch ?? 0)]
          if (next.length > HISTORY_LEN) next.shift()
          return next
        })
        setPwmHistory(prev => {
          const next = [...prev, parseFloat(data.pwm ?? 0)]
          if (next.length > HISTORY_LEN) next.shift()
          return next
        })

        if (data.last_cmd && data.last_cmd !== 'NONE') {
          setLogs(prev => {
            const entry = `${new Date().toLocaleTimeString()} — ${data.last_intent} → ${data.last_cmd}`
            const next = [entry, ...prev]
            if (next.length > 30) next.pop()
            return next
          })
        }
      }

      ws.onclose = () => {
        if (!active) return
        setWsConnected(false)
        setTimeout(connect, 2000)
      }

      ws.onerror = () => {
        if (!active) return
        setWsConnected(false)
      }
    }

    connect()
    return () => { active = false; wsRef.current?.close() }
  }, [])

  /* ── Command senders ─────────────────────────────────────────── */
  const send = (payload) => {
    const ws = wsRef.current
    if (ws?.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify(payload))
    }
  }

  const doMove       = (dir)  => send({ cmd: COMMANDS[dir] })
  const toggleVoice  = ()     => send({ cmd: 'VOICE_TOGGLE' })
  const sendNlpText  = ()     => {
    if (!nlpText.trim()) return
    send({ cmd: 'NLP_TEXT', text: nlpText.trim() })
    setNlpText('')
  }
  const updateSpeed  = (val)  => {
    const v = parseFloat(val)
    setSpeed(v)
    send({ cmd: 'SPEED', val: v })
  }

  const handleNlpKey = (e) => {
    if (e.key === 'Enter') sendNlpText()
  }

  /* ── Derived state ───────────────────────────────────────────── */
  const bleConnected = telemetry.ble_connected
  const voiceActive  = telemetry.voice_active

  return (
    <div className="app-shell">
      {/* ── Header ──────────────────────────────────────────────── */}
      <header className="topbar">
        <div className="brand">
          <img className="brand-logo" src="/favicon.svg" alt="VoxBot" />
          <span className="brand-name">VoxBot</span>
          <span className="brand-tag">v1.0</span>
        </div>
        <div className="status-row">
          <span className={`status-pill ${wsConnected ? 'ok' : 'off'}`}>
            <span className="dot" />
            {wsConnected ? 'Server Online' : 'Server Offline'}
          </span>
          <span className={`status-pill ${bleConnected ? 'ok' : 'warn'}`}>
            <span className="dot" />
            {bleConnected ? 'Robot Connected' : 'Robot Not Connected'}
          </span>
          <span className="uptime-label">{formatUptime(telemetry.uptime)}</span>
        </div>
      </header>

      {/* ── Disconnected banner ─────────────────────────────────── */}
      {!bleConnected && (
        <div className="disconnected-banner">
          <span className="banner-icon">📡</span>
          <div>
            <div className="banner-text">Robot not connected via Bluetooth</div>
            <div className="banner-sub">Ensure VoxBot is powered on and advertising. The system will auto-connect when in range.</div>
          </div>
        </div>
      )}

      {/* ── Telemetry metrics ───────────────────────────────────── */}
      <div className="section-heading">Live Telemetry</div>
      <section className="metrics-grid">
        <div className="metric-card">
          <p className="metric-label">Pitch Angle</p>
          <p className="metric-value">
            {(telemetry.pitch ?? 0).toFixed(1)}<span className="metric-unit">°</span>
          </p>
        </div>
        <div className="metric-card">
          <p className="metric-label">Motor PWM</p>
          <p className="metric-value">
            {Math.round(telemetry.pwm ?? 0)}<span className="metric-unit"> / 200</span>
          </p>
        </div>
        <div className="metric-card">
          <p className="metric-label">Last Command</p>
          <p className="metric-value" style={{ fontSize: '1rem', wordBreak: 'break-word' }}>
            {telemetry.last_cmd}
          </p>
        </div>
      </section>

      {/* ── Charts ──────────────────────────────────────────────── */}
      <section className="chart-section">
        <div className="section-heading">Sensor History</div>
        <div className="two-col">
          <div className="chart-container">
            <div className="section-heading" style={{ marginBottom: 8 }}>Pitch (°)</div>
            <div style={{ height: 180 }}>
              <Line data={pitchChartData} options={chartOpts('Pitch', -25, 25, '#6c47ff')} />
            </div>
          </div>
          <div className="chart-container">
            <div className="section-heading" style={{ marginBottom: 8 }}>Motor PWM</div>
            <div style={{ height: 180 }}>
              <Line data={pwmChartData} options={chartOpts('PWM', -200, 200, '#3b82f6')} />
            </div>
          </div>
        </div>
      </section>

      {/* ── Controls + NLP ──────────────────────────────────────── */}
      <div className="two-col">
        {/* Left: movement controls */}
        <div className="controls-card">
          <div className="section-heading">Movement Controls</div>
          <div className="dpad">
            <button id="btn-forward" className="dpad-btn up"    onClick={() => doMove('FORWARD')}><ArrowUp /></button>
            <button id="btn-left"    className="dpad-btn left"  onClick={() => doMove('LEFT')}><ArrowLeft /></button>
            <button id="btn-stop"    className="dpad-btn stop"  onClick={() => doMove('STOP')}><StopIcon /></button>
            <button id="btn-right"   className="dpad-btn right" onClick={() => doMove('RIGHT')}><ArrowRight /></button>
            <button id="btn-back"    className="dpad-btn down"  onClick={() => doMove('BACK')}><ArrowDown /></button>
          </div>
          <div className="speed-control">
            <span className="speed-label">Speed</span>
            <input
              id="speed-slider"
              className="speed-slider"
              type="range"
              min="0.1"
              max="1.0"
              step="0.05"
              value={speed}
              onChange={e => updateSpeed(e.target.value)}
            />
            <span className="speed-val">{speed.toFixed(2)}</span>
          </div>
        </div>

        {/* Right: NLP + Voice */}
        <div className="nlp-card">
          <div className="section-heading">Voice & NLP</div>

          <div className="nlp-input-row">
            <input
              id="nlp-text-input"
              className="nlp-input"
              type="text"
              placeholder='Type a command… e.g. "go forward" or "turn left"'
              value={nlpText}
              onChange={e => setNlpText(e.target.value)}
              onKeyDown={handleNlpKey}
            />
            <button id="nlp-send-btn" className="send-btn" onClick={sendNlpText} disabled={!nlpText.trim()}>
              Send
            </button>
          </div>

          <button
            id="voice-toggle-btn"
            className={`voice-btn ${voiceActive ? 'active' : ''}`}
            onClick={toggleVoice}
          >
            <span className="voice-icon" />
            {voiceActive ? 'Voice Active — Listening…' : 'Enable Voice Control'}
          </button>

          {/* NLP result readout */}
          <div className="nlp-result">
            <div className="nlp-row">
              <span className="nlp-row-label">Transcript</span>
              <span className="nlp-row-value">{telemetry.last_raw_text || '—'}</span>
            </div>
            <div className="nlp-row">
              <span className="nlp-row-label">Intent</span>
              <span className="nlp-row-value">{telemetry.last_intent}</span>
            </div>
            <div className="nlp-row">
              <span className="nlp-row-label">Confidence</span>
              <span style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
                <span className="nlp-row-value">{((telemetry.last_confidence ?? 0) * 100).toFixed(0)}%</span>
                <span className="confidence-bar-bg">
                  <span className="confidence-bar-fill" style={{ width: `${(telemetry.last_confidence ?? 0) * 100}%` }} />
                </span>
              </span>
            </div>
            <div className="nlp-row">
              <span className="nlp-row-label">Extracted Speed</span>
              <span className="nlp-row-value">{(telemetry.last_vel_extracted ?? 0.5).toFixed(2)}</span>
            </div>
          </div>
        </div>
      </div>

      {/* ── Activity log ────────────────────────────────────────── */}
      <section className="log-section">
        <div className="section-heading">Activity Log</div>
        <div className="log-card">
          <div className="log-list">
            {logs.length === 0 && <div className="log-empty">No commands sent yet</div>}
            {logs.map((entry, i) => (
              <div key={i} className="log-entry">{entry}</div>
            ))}
          </div>
        </div>
      </section>
    </div>
  )
}

export default App
