import { memo, useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { createRoot } from 'react-dom/client';
import './styles.css';

const telemetry = [
  ['Roll', '0.42°'], ['Pitch', '-1.08°'], ['Yaw', '92.16°'],
  ['Accel X', '0.012 m/s²'], ['Accel Y', '-0.031 m/s²'], ['Accel Z', '9.806 m/s²'],
  ['Gyro X', '0.003 rad/s'], ['Gyro Y', '-0.007 rad/s'], ['Gyro Z', '0.011 rad/s'],
];

function SectionTitle({ eyebrow, title, note }) {
  return <div className="section-title"><div><span className="eyebrow">{eyebrow}</span><h2>{title}</h2></div>{note && <span className="section-note">{note}</span>}</div>;
}

const StatusChip = memo(function StatusChip({ label, value, tone = 'neutral' }) {
  return <div className={`status-chip ${tone}`}><span className="chip-dot" />{label}<strong>{value}</strong></div>;
});

function Sidebar({ connected, onToggle }) {
  const [linear, setLinear] = useState(0.15);
  const [angular, setAngular] = useState(0.5);
  const [mode, setMode] = useState('goal');
  const [pressed, setPressed] = useState(null);
  const speed = useMemo(() => `${linear.toFixed(2)} m/s`, [linear]);
  const turn = useMemo(() => `${angular.toFixed(2)} rad/s`, [angular]);
  const press = useCallback((direction) => setPressed(direction), []);
  const release = useCallback(() => setPressed(null), []);

  useEffect(() => {
    const handleKey = (event) => {
      if (['INPUT', 'SELECT', 'TEXTAREA'].includes(event.target.tagName)) return;
      if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', ' '].includes(event.key)) event.preventDefault();
      const map = { ArrowUp: 'forward', ArrowDown: 'backward', ArrowLeft: 'left', ArrowRight: 'right', ' ': 'stop' };
      if (map[event.key]) press(map[event.key]);
    };
    const releaseKey = () => release();
    window.addEventListener('keydown', handleKey);
    window.addEventListener('keyup', releaseKey);
    window.addEventListener('blur', releaseKey);
    return () => { window.removeEventListener('keydown', handleKey); window.removeEventListener('keyup', releaseKey); window.removeEventListener('blur', releaseKey); };
  }, [press, release]);

  return <aside className="sidebar">
    <div className="brand"><span className="brand-mark">RC</span><div><strong>Robot Center</strong><small>ROS2 OPERATIONS</small></div></div>
    <button className={`connection ${connected ? 'is-connected' : ''}`} onClick={onToggle}><span className="connection-dot" /><span>{connected ? 'Connected to rosbridge' : 'Connect to rosbridge'}</span><b>{connected ? 'LIVE' : 'OFFLINE'}</b></button>
    <div className="side-group"><span className="group-label">Connection</span><label>ROSBridge URL<input defaultValue="ws://localhost:9090" /></label><button className="button primary" onClick={onToggle}>{connected ? 'Disconnect' : 'Connect'}</button></div>
    <div className="side-group"><span className="group-label">Point cloud</span><label>Fixed frame<input defaultValue="odom" /></label><label>Topic<select defaultValue="/registered_scan"><option>/registered_scan</option><option>/cloud_registered</option></select></label></div>
    <div className="side-group"><span className="group-label">Manual control</span><label className="range-label">Linear speed <strong>{speed}</strong><input type="range" min="0" max="0.26" step="0.01" value={linear} onChange={(e) => setLinear(Number(e.target.value))} /></label><label className="range-label">Angular speed <strong>{turn}</strong><input type="range" min="0" max="1" step="0.05" value={angular} onChange={(e) => setAngular(Number(e.target.value))} /></label><div className="dpad">{[['↖','forward-left'],['↑','forward'],['↗','forward-right'],['←','left'],['■','stop'],['→','right'],['↙','backward-left'],['↓','backward'],['↘','backward-right']].map(([icon, key]) => <button key={key} className={key === 'stop' ? 'stop' : ''} onMouseDown={() => press(key)} onMouseUp={release} onMouseLeave={release} onTouchStart={() => press(key)} onTouchEnd={release} aria-label={key}>{icon}</button>)}</div><div className="velocity"><span>Command velocity</span><strong className={pressed ? 'active-value' : ''}>{pressed || 'idle'}</strong></div></div>
    <div className="side-group"><span className="group-label">Navigation mode</span><div className="mode-tabs">{[['goal','Goal'],['waypoint','Waypoints'],['pose','Initial pose']].map(([key, label]) => <button key={key} className={mode === key ? 'active' : ''} onClick={() => setMode(key)}>{label}</button>)}</div></div>
    <footer><span className="footer-pulse" />MID360 · FAST-LIO · Nav2</footer>
  </aside>;
}

function PointCloud() {
  const canvasRef = useRef(null);
  useEffect(() => {
    const canvas = canvasRef.current; const ctx = canvas.getContext('2d'); let frame;
    const resize = () => { canvas.width = canvas.clientWidth * devicePixelRatio; canvas.height = canvas.clientHeight * devicePixelRatio; };
    const draw = (time) => { const { width, height } = canvas; ctx.clearRect(0, 0, width, height); ctx.save(); ctx.scale(devicePixelRatio, devicePixelRatio); const w = canvas.clientWidth; const h = canvas.clientHeight;
      ctx.strokeStyle = 'rgba(101,230,192,.08)'; ctx.lineWidth = 1; for (let x = 0; x < w; x += 48) { ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke(); } for (let y = 0; y < h; y += 48) { ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke(); }
      const cx = w * .51, cy = h * .54; for (let i = 0; i < 420; i++) { const angle = i * 2.399 + time * .00012; const radius = (i % 36) * 5 + 8 + Math.sin(i * .7) * 10; const px = cx + Math.cos(angle) * radius * 1.8; const py = cy + Math.sin(angle) * radius * .62; ctx.fillStyle = `hsla(${155 + (i % 40)}, 72%, ${48 + (i % 30)}%, .72)`; ctx.fillRect(px, py, 2, 2); }
      ctx.fillStyle = '#65e6c0'; ctx.beginPath(); ctx.arc(cx, cy, 5, 0, Math.PI * 2); ctx.fill(); ctx.restore(); frame = requestAnimationFrame(draw); };
    resize(); window.addEventListener('resize', resize); frame = requestAnimationFrame(draw); return () => { cancelAnimationFrame(frame); window.removeEventListener('resize', resize); };
  }, []);
  return <div className="point-cloud"><canvas ref={canvasRef} /><div className="viewer-overlay"><span>LIVE POINT CLOUD</span><b>/registered_scan</b><small>Frame: odom · 2 Hz</small></div><div className="axis"><span>X</span><span>Y</span><span>Z</span></div></div>;
}

function Telemetry() { return <section className="telemetry panel"><SectionTitle eyebrow="Sensor stream" title="IMU telemetry" note="/livox/imu · sensor_msgs/Imu" /><div className="telemetry-grid">{telemetry.map(([label, value]) => <div className="telemetry-cell" key={label}><span>{label}</span><strong>{value}</strong></div>)}</div></section>; }

function App() { const [connected, setConnected] = useState(false); const toggle = useCallback(() => setConnected((value) => !value), []); return <div className="app-shell"><Sidebar connected={connected} onToggle={toggle} /><main className="main"><header className="topbar"><div><span className="eyebrow">Fleet console / local session</span><h1>One view for every signal.</h1></div><div className="topbar-status"><StatusChip label="ROSBridge" value={connected ? 'Connected' : 'Offline'} tone={connected ? 'success' : 'neutral'} /><StatusChip label="Frame" value="odom" /><StatusChip label="Point cloud" value="48k max" /></div></header><div className="hero-grid"><section className="panel viewer-panel"><SectionTitle eyebrow="Spatial view" title="3D point cloud" note="FAST-LIO registered scan" /><PointCloud /></section><section className="panel command-panel"><SectionTitle eyebrow="Command center" title="Ready state" note="Manual drive" /><div className="command-state"><span className={connected ? 'state-light live' : 'state-light'} /><strong>{connected ? 'Vehicle link active' : 'Connect to begin'}</strong><p>Use the directional pad or arrow keys to send velocity commands.</p></div><div className="command-list"><div><span>Linear limit</span><b>0.26 m/s</b></div><div><span>Angular limit</span><b>1.00 rad/s</b></div><div><span>Safety stop</span><b>Spacebar</b></div></div></section></div><Telemetry /></main></div>; }

createRoot(document.getElementById('root')).render(<App />);
