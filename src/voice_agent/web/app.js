/**
 * MiMo Rover 前端：麦克风采集、SSE 流式对话、TTS 播放、小车状态面板。
 */

// 浏览器禁止在 0.0.0.0 上使用麦克风，自动跳到 localhost
if (window.location.hostname === '0.0.0.0') {
  window.location.replace(window.location.href.replace('0.0.0.0', 'localhost'));
}

const btnMic = document.getElementById('btn-mic');
const micHint = document.getElementById('mic-hint');
const inputText = document.getElementById('input-text');
const btnSend = document.getElementById('btn-send');
const btnClear = document.getElementById('btn-clear');
const selectVoice = document.getElementById('select-voice');
const selectMode = document.getElementById('select-mode');
const inputToken = document.getElementById('input-token');
const messagesContainer = document.getElementById('messages-container');
const aiOrb = document.getElementById('ai-orb');
const stateCaption = document.getElementById('state-caption');
const statusPill = document.getElementById('status-pill');
const statusText = document.getElementById('status-text');
const canvas = document.getElementById('waveform-canvas');
const ctx = canvas.getContext('2d');

const robotMode = document.getElementById('robot-mode');
const robotDetail = document.getElementById('robot-detail');
const chipOdom = document.getElementById('chip-odom');
const chipScan = document.getElementById('chip-scan');
const chipCmd = document.getElementById('chip-cmd');
const robotWarning = document.getElementById('robot-warning');
const btnEstop = document.getElementById('btn-estop');
const btnReleaseEstop = document.getElementById('btn-release-estop');

let currentState = 'idle'; // idle | listening | thinking | speaking
let mediaStream = null;
let audioContext = null;
let analyser = null;
let scriptProcessor = null;
let recordedBuffers = [];
let recordingSampleRate = 16000;
let animationId = null;
let currentAudio = null;
let conversationHistory = [];
let chatBusy = false;
let latestRobotStatus = null;

// ==============================================================================
// 1. 令牌与 API 请求
// ==============================================================================
function getToken() {
  return inputToken ? inputToken.value.trim() : '';
}

if (inputToken) {
  inputToken.value = localStorage.getItem('mimo_voice_token') || '';
  inputToken.addEventListener('change', () => {
    localStorage.setItem('mimo_voice_token', inputToken.value.trim());
  });
}

function apiFetch(url, options = {}) {
  const opts = { ...options, headers: { ...(options.headers || {}) } };
  const token = getToken();
  if (token) opts.headers['X-API-Token'] = token;
  return fetch(url, opts);
}

// ==============================================================================
// 2. 状态管理与可视化
// ==============================================================================
function setState(state, customMessage) {
  currentState = state;
  statusPill.className = 'status-pill ' + state;
  aiOrb.className = 'ai-orb ' + state;

  switch (state) {
    case 'idle':
      statusText.textContent = latestRobotStatus && latestRobotStatus.estop ? '急停锁存' : '待命';
      stateCaption.textContent = customMessage || '点击麦克风说话，或直接输入“前进一米”“去1号点位”';
      btnMic.classList.remove('recording');
      micHint.textContent = selectMode.value === 'hold' ? '按住麦克风说话' : '点击开始录音';
      break;
    case 'listening':
      statusText.textContent = '正在倾听';
      stateCaption.textContent = '正在聆听，说完请再次点击...';
      btnMic.classList.add('recording');
      micHint.textContent = '点击结束并发送';
      break;
    case 'thinking':
      statusText.textContent = '思考中';
      stateCaption.textContent = 'MiMo 正在处理...';
      btnMic.classList.remove('recording');
      micHint.textContent = '请稍候';
      break;
    case 'speaking':
      statusText.textContent = '正在讲话';
      stateCaption.textContent = 'MiMo 正在回复语音...';
      btnMic.classList.remove('recording');
      micHint.textContent = '播放中';
      break;
  }
}

function initVisualizer() {
  function resize() {
    canvas.width = canvas.parentElement.clientWidth;
    canvas.height = canvas.parentElement.clientHeight;
  }
  window.addEventListener('resize', resize);
  resize();

  function draw() {
    animationId = requestAnimationFrame(draw);
    const width = canvas.width;
    const height = canvas.height;
    ctx.clearRect(0, 0, width, height);

    const dataArray = new Uint8Array(64);
    if (analyser && (currentState === 'listening' || currentState === 'speaking')) {
      analyser.getByteFrequencyData(dataArray);
    } else {
      const t = Date.now() * 0.002;
      for (let i = 0; i < 64; i++) {
        dataArray[i] = Math.sin(t + i * 0.15) * 12 + 15;
      }
    }

    const centerY = height / 2;
    const barWidth = (width / 64) * 0.8;
    const gradient = ctx.createLinearGradient(0, 0, width, 0);

    if (currentState === 'listening') {
      gradient.addColorStop(0, 'rgba(255, 105, 0, 0.2)');
      gradient.addColorStop(0.5, 'rgba(255, 105, 0, 0.8)');
      gradient.addColorStop(1, 'rgba(255, 105, 0, 0.2)');
    } else if (currentState === 'thinking') {
      gradient.addColorStop(0, 'rgba(0, 210, 255, 0.2)');
      gradient.addColorStop(0.5, 'rgba(0, 210, 255, 0.8)');
      gradient.addColorStop(1, 'rgba(0, 210, 255, 0.2)');
    } else if (currentState === 'speaking') {
      gradient.addColorStop(0, 'rgba(168, 85, 247, 0.2)');
      gradient.addColorStop(0.5, 'rgba(168, 85, 247, 0.8)');
      gradient.addColorStop(1, 'rgba(168, 85, 247, 0.2)');
    } else {
      gradient.addColorStop(0, 'rgba(255, 255, 255, 0.05)');
      gradient.addColorStop(0.5, 'rgba(255, 105, 0, 0.3)');
      gradient.addColorStop(1, 'rgba(255, 255, 255, 0.05)');
    }

    ctx.fillStyle = gradient;
    for (let i = 0; i < 64; i++) {
      const barHeight = (dataArray[i] / 255) * (height * 0.45);
      const x = i * (width / 64);
      ctx.fillRect(x, centerY - barHeight / 2, barWidth, barHeight);
    }
  }
  draw();
}

// ==============================================================================
// 3. 麦克风录音与 WAV 编码
// ==============================================================================
async function startRecording() {
  try {
    if (currentAudio) {
      currentAudio.pause();
      currentAudio = null;
    }
    if (!window.isSecureContext) {
      alert('浏览器只允许在 localhost 或 HTTPS 页面使用麦克风。\n\n请在小车上打开 http://localhost:8000，或给服务配置 HTTPS 证书。');
      return;
    }
    if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
      throw new Error('浏览器未提供麦克风 API');
    }

    mediaStream = await navigator.mediaDevices.getUserMedia({
      audio: {
        channelCount: 1,
        sampleRate: 16000,
        echoCancellation: true,
        noiseSuppression: true,
      },
    });

    audioContext = new (window.AudioContext || window.webkitAudioContext)();
    analyser = audioContext.createAnalyser();
    analyser.fftSize = 128;

    const source = audioContext.createMediaStreamSource(mediaStream);
    source.connect(analyser);

    recordedBuffers = [];
    scriptProcessor = audioContext.createScriptProcessor(4096, 1, 1);
    recordingSampleRate = audioContext.sampleRate;

    scriptProcessor.onaudioprocess = (event) => {
      if (currentState === 'listening') {
        recordedBuffers.push(new Float32Array(event.inputBuffer.getChannelData(0)));
      }
    };

    source.connect(scriptProcessor);
    scriptProcessor.connect(audioContext.destination);
    setState('listening');
  } catch (err) {
    console.error('麦克风权限错误:', err);
    alert('无法获取麦克风权限！\n\n请检查浏览器地址栏左侧的麦克风权限，以及系统设置中是否允许浏览器使用麦克风。');
    setState('idle');
  }
}

async function stopRecording() {
  if (currentState !== 'listening') return;
  setState('thinking', '正在识别语音...');

  if (mediaStream) {
    mediaStream.getTracks().forEach((track) => track.stop());
    mediaStream = null;
  }
  if (scriptProcessor) {
    scriptProcessor.disconnect();
    scriptProcessor = null;
  }

  const wavBlob = encodeWAV(recordedBuffers, recordingSampleRate);
  recordedBuffers = [];
  if (!wavBlob || wavBlob.size < 1000) {
    setState('idle', '录音时间太短，请重试');
    return;
  }

  try {
    const formData = new FormData();
    formData.append('file', wavBlob, 'recording.wav');
    const res = await apiFetch('/api/asr', { method: 'POST', body: formData });
    if (!res.ok) {
      const detail = await res.text();
      throw new Error(detail || 'ASR 服务响应失败');
    }
    const data = await res.json();
    const transcript = (data.text || '').trim();
    if (!transcript) {
      setState('idle', '没有识别到清晰的语音，请重试');
      return;
    }
    await processUserMessage(transcript);
  } catch (err) {
    console.error('ASR 请求失败:', err);
    setState('idle', '语音识别出错：' + err.message);
  }
}

function encodeWAV(buffers, sampleRate) {
  let totalLength = 0;
  for (const buffer of buffers) totalLength += buffer.length;
  if (totalLength === 0) return null;

  const merged = new Float32Array(totalLength);
  let offset = 0;
  for (const buffer of buffers) {
    merged.set(buffer, offset);
    offset += buffer.length;
  }

  const arrayBuffer = new ArrayBuffer(44 + merged.length * 2);
  const view = new DataView(arrayBuffer);

  function writeString(targetView, targetOffset, string) {
    for (let i = 0; i < string.length; i++) {
      targetView.setUint8(targetOffset + i, string.charCodeAt(i));
    }
  }

  writeString(view, 0, 'RIFF');
  view.setUint32(4, 36 + merged.length * 2, true);
  writeString(view, 8, 'WAVE');
  writeString(view, 12, 'fmt ');
  view.setUint32(16, 16, true);
  view.setUint16(20, 1, true);
  view.setUint16(22, 1, true);
  view.setUint32(24, sampleRate, true);
  view.setUint32(28, sampleRate * 2, true);
  view.setUint16(32, 2, true);
  view.setUint16(34, 16, true);
  writeString(view, 36, 'data');
  view.setUint32(40, merged.length * 2, true);

  let index = 44;
  for (let i = 0; i < merged.length; i++) {
    const sample = Math.max(-1, Math.min(1, merged[i]));
    view.setInt16(index, sample < 0 ? sample * 0x8000 : sample * 0x7fff, true);
    index += 2;
  }
  return new Blob([view], { type: 'audio/wav' });
}

// ==============================================================================
// 4. 对话流程
// ==============================================================================
async function processUserMessage(text) {
  text = (text || '').trim();
  if (!text) return;
  if (chatBusy) {
    console.warn('上一条消息还在处理中，忽略:', text);
    return;
  }
  chatBusy = true;

  appendMessage('user', text);
  conversationHistory.push({ role: 'user', content: text });
  setState('thinking');

  const assistantMsgEl = appendMessage('assistant', '');
  const contentEl = assistantMsgEl.querySelector('.msg-content');
  let fullReply = '';

  try {
    const res = await apiFetch('/api/chat', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        messages: conversationHistory,
        robot: true,
      }),
    });

    if (!res.ok) {
      const detail = await res.text();
      throw new Error(detail || '对话服务响应异常');
    }

    const reader = res.body.getReader();
    const decoder = new TextDecoder('utf-8');
    let buffer = '';

    while (true) {
      const { done, value } = await reader.read();
      if (done) break;
      buffer += decoder.decode(value, { stream: true });
      const blocks = buffer.split('\n\n');
      buffer = blocks.pop();

      for (const block of blocks) {
        if (!block.startsWith('data: ')) continue;
        const dataStr = block.slice(6).trim();
        if (dataStr === '[DONE]') continue;
        let parsed;
        try {
          parsed = JSON.parse(dataStr);
        } catch (err) {
          continue;
        }

        if (parsed.delta) {
          fullReply += parsed.delta;
          contentEl.textContent = fullReply;
          messagesContainer.scrollTop = messagesContainer.scrollHeight;
        }
        if (parsed.robot) {
          updateRobotStatus(parsed.robot);
        }
        if (parsed.tool_call) {
          console.log('工具调用:', parsed.tool_call);
        }
        if (parsed.tool_result) {
          console.log('工具结果:', parsed.tool_result);
        }
        if (parsed.error) {
          fullReply += (fullReply ? '\n' : '') + '⚠️ ' + parsed.error;
          contentEl.textContent = fullReply;
        }
      }
    }

    if (!fullReply) {
      fullReply = '（没有收到回复）';
      contentEl.textContent = fullReply;
    }
    conversationHistory.push({ role: 'assistant', content: fullReply });

    if (fullReply && !fullReply.startsWith('⚠️')) {
      await speakText(fullReply);
    } else {
      setState('idle');
    }
  } catch (err) {
    console.error('对话流程异常:', err);
    contentEl.textContent = '❌ 请求异常：' + err.message;
    setState('idle');
  } finally {
    chatBusy = false;
  }
}

function appendMessage(role, content) {
  const msgEl = document.createElement('div');
  msgEl.className = `message-bubble ${role}`;

  const isUser = role === 'user';
  const avatar = isUser ? '👤' : '🤖';
  const author = isUser ? '你' : 'MiMo Rover';
  const timeStr = new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });

  msgEl.innerHTML = `
    <div class="msg-avatar">${avatar}</div>
    <div class="msg-body">
      <div class="msg-header">
        <span class="msg-author">${author}</span>
        <span class="msg-time">${timeStr}</span>
      </div>
      <div class="msg-content">${escapeHTML(content)}</div>
    </div>
  `;

  messagesContainer.appendChild(msgEl);
  messagesContainer.scrollTop = messagesContainer.scrollHeight;
  return msgEl;
}

function escapeHTML(str) {
  return String(str).replace(/[&<>'"]/g, (tag) => (
    { '&': '&amp;', '<': '&lt;', '>': '&gt;', "'": '&#39;', '"': '&quot;' }[tag] || tag
  ));
}

// ==============================================================================
// 5. TTS 播放（失败时回退浏览器语音）
// ==============================================================================
async function speakText(text) {
  if (!text || !text.trim()) {
    setState('idle');
    return;
  }
  setState('speaking');

  try {
    const res = await apiFetch('/api/tts', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ text, voice: selectVoice.value }),
    });
    if (!res.ok) throw new Error('TTS 接口返回 ' + res.status);
    const data = await res.json();
    if (!data.audio_base64) throw new Error('未获取到音频数据');

    currentAudio = new Audio(`data:audio/wav;base64,${data.audio_base64}`);
    try {
      if (!audioContext) {
        audioContext = new (window.AudioContext || window.webkitAudioContext)();
      }
      if (audioContext.state === 'suspended') audioContext.resume();
      const source = audioContext.createMediaElementSource(currentAudio);
      analyser = audioContext.createAnalyser();
      analyser.fftSize = 128;
      source.connect(analyser);
      analyser.connect(audioContext.destination);
    } catch (err) {
      // 某些浏览器会拒绝重复创建 media element source，忽略即可
    }

    currentAudio.onended = () => setState('idle');
    currentAudio.onerror = () => setState('idle');
    await currentAudio.play();
  } catch (err) {
    console.warn('MiMo TTS 失败，回退浏览器语音:', err);
    fallbackSpeak(text);
  }
}

function fallbackSpeak(text) {
  if (!('speechSynthesis' in window)) {
    setState('idle');
    return;
  }
  const utterance = new SpeechSynthesisUtterance(text);
  utterance.lang = 'zh-CN';
  utterance.onend = () => setState('idle');
  utterance.onerror = () => setState('idle');
  window.speechSynthesis.cancel();
  window.speechSynthesis.speak(utterance);
}

// ==============================================================================
// 6. 小车状态面板
// ==============================================================================
const MODE_LABELS = {
  idle: '待命',
  moving: '运动中',
  navigating: '导航中',
};

function updateRobotStatus(status) {
  if (!status) return;
  latestRobotStatus = status;

  let modeClass = status.mode || 'idle';
  if (status.estop) modeClass = 'estop';
  robotMode.className = 'robot-mode ' + modeClass;

  let modeLabel = MODE_LABELS[status.mode] || status.mode || '未知';
  if (status.estop) modeLabel = '急停锁存';
  if (status.mode === 'navigating' && status.point) {
    modeLabel = `导航 → ${status.point.id}号点位`;
  }
  robotMode.textContent = modeLabel;

  let detail = status.detail || '待命';
  if (status.pose) {
    detail += ` · x=${status.pose.x.toFixed(2)}, y=${status.pose.y.toFixed(2)}`;
  }
  if (status.nav_feedback && status.nav_feedback.distance_remaining != null) {
    detail += ` · 剩余 ${status.nav_feedback.distance_remaining.toFixed(1)} m`;
  }
  robotDetail.textContent = detail;

  setChip(chipOdom, status.odom_ok ? '里程计 OK' : '里程计无数据', status.odom_ok);
  setChip(chipScan, status.scan_ok ? '扫描 OK' : '扫描无数据', status.scan_ok);
  setChip(chipCmd, '输出 ' + (status.cmd_vel_topic || '—'), true);

  const warnings = [];
  if (status.estop) warnings.push('软件急停已锁存，运动指令被屏蔽。说“解除急停”或点击下方按钮恢复。');
  if (status.autonomy_blocked) warnings.push('安全层锁住了自动驾驶（可能是定位未就绪），导航目标可能不会移动。');
  if (status.waypoint_error) warnings.push(status.waypoint_error);
  if (warnings.length) {
    robotWarning.hidden = false;
    robotWarning.textContent = warnings.join(' ');
  } else {
    robotWarning.hidden = true;
    robotWarning.textContent = '';
  }

  btnEstop.textContent = status.estop ? '🛑 急停已锁存' : '🛑 急停';
  document.querySelectorAll('.quick-btn[data-cmd]').forEach((button) => {
    button.disabled = !!status.estop;
  });
  btnReleaseEstop.disabled = !status.estop;

  if (status.estop && currentState === 'idle') {
    statusText.textContent = '急停锁存';
  }
}

function setChip(element, text, ok) {
  element.textContent = text;
  element.className = 'chip ' + (ok ? 'ok' : 'bad');
}

async function refreshRobotStatus() {
  try {
    const res = await apiFetch('/api/robot/status');
    if (!res.ok) {
      if (res.status === 401) {
        robotDetail.textContent = '令牌无效：请在右下角“令牌”框填入 VOICE_AGENT_TOKEN。';
      }
      return;
    }
    updateRobotStatus(await res.json());
  } catch (err) {
    robotDetail.textContent = '无法连接小车状态服务';
  }
}

async function directStop() {
  try {
    const res = await apiFetch('/api/robot/stop', { method: 'POST' });
    const data = await res.json();
    appendMessage('user', '停止');
    appendMessage('assistant', data.reply || '已停止。');
    updateRobotStatus(data.status);
  } catch (err) {
    console.error('停止失败:', err);
  }
}

async function setEstop(engage) {
  try {
    const res = await apiFetch('/api/robot/estop', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ engage }),
    });
    const data = await res.json();
    appendMessage('user', engage ? '急停' : '解除急停');
    appendMessage('assistant', data.reply || '');
    updateRobotStatus(data.status);
  } catch (err) {
    console.error('急停操作失败:', err);
    alert('急停操作失败：' + err.message);
  }
}

// ==============================================================================
// 7. 事件绑定
// ==============================================================================
btnMic.addEventListener('click', () => {
  if (selectMode.value === 'hold') return;
  if (currentState === 'listening') stopRecording();
  else if (currentState === 'idle') startRecording();
});

btnMic.addEventListener('mousedown', () => {
  if (selectMode.value === 'hold' && currentState === 'idle') startRecording();
});

window.addEventListener('mouseup', () => {
  if (selectMode.value === 'hold' && currentState === 'listening') stopRecording();
});

btnMic.addEventListener('touchstart', (event) => {
  if (selectMode.value === 'hold' && currentState === 'idle') {
    event.preventDefault();
    startRecording();
  }
});

window.addEventListener('touchend', () => {
  if (selectMode.value === 'hold' && currentState === 'listening') stopRecording();
});

btnSend.addEventListener('click', () => {
  const value = inputText.value.trim();
  if (value) {
    inputText.value = '';
    processUserMessage(value);
  }
});

inputText.addEventListener('keydown', (event) => {
  if (event.key === 'Enter') btnSend.click();
});

btnClear.addEventListener('click', () => {
  conversationHistory = [];
  messagesContainer.innerHTML = '';
  appendMessage('assistant', '对话已清空。随时点击麦克风或输入指令。');
});

document.getElementById('orb-wrapper').addEventListener('click', () => btnMic.click());

document.querySelectorAll('.quick-btn[data-cmd]').forEach((button) => {
  button.addEventListener('click', () => {
    const command = button.dataset.cmd;
    if (command === '停止') {
      directStop();
    } else {
      processUserMessage(command);
    }
  });
});

btnEstop.addEventListener('click', () => {
  const already = latestRobotStatus && latestRobotStatus.estop;
  if (already) {
    if (confirm('急停已锁存。要解除急停并恢复运动能力吗？')) setEstop(false);
    return;
  }
  if (confirm('确认触发软件急停？小车会立即停止并锁存，直到解除。')) setEstop(true);
});

btnReleaseEstop.addEventListener('click', () => setEstop(false));

window.addEventListener('DOMContentLoaded', async () => {
  initVisualizer();
  setState('idle');
  refreshRobotStatus();
  setInterval(refreshRobotStatus, 1000);

  try {
    const res = await apiFetch('/api/info');
    if (res.ok) {
      const info = await res.json();
      const badge = document.getElementById('badge-model');
      if (badge && info.model) {
        badge.innerHTML = `<span class="badge-icon">⚡</span><span>${escapeHTML(info.model)}</span>`;
      }
      if (!info.api_configured) {
        appendMessage('assistant', '提示：服务端还没有配置 MIMO_API_KEY，语音识别/对话/语音合成不可用，但手动运动指令仍然可用。');
      }
      updateRobotStatus(info.robot);
    }
  } catch (err) {
    console.warn('读取 /api/info 失败:', err);
  }
});
