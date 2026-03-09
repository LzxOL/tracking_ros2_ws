/**
 * Swiss Minimalist Tracking System Logic
 */

const API_BASE = '/api';
let keypointClicks = [];
let coarsePoint = null;  // 粗定位点（完成后会消失）
let coarseDone = false;  // 粗定位是否已完成

// Pose formatting functions
function formatPose(pose) {
  if (!pose) return { xyz: '--', quat: '--' };
  const xyz = pose.slice(0, 3).map(v => v.toFixed(3)).join(', ');
  const quat = pose.slice(3, 7).map(v => v.toFixed(3)).join(', ');
  return { xyz, quat };
}

async function updatePoseDisplay() {
  try {
    const res = await fetch(`${API_BASE}/pose`);
    if (!res.ok) return;
    const data = await res.json();
    console.log('Pose data received:', data);  // 调试日志
    const pose = data.pose;
    const { xyz, quat } = formatPose(pose);
    const xyzEl = document.getElementById('pose-xyz');
    const quatEl = document.getElementById('pose-quat');
    if (xyzEl) xyzEl.textContent = xyz;
    if (quatEl) quatEl.textContent = quat;
  } catch (e) { console.error('Pose update error:', e); }
}

const videoStream = document.getElementById('video-stream');
const videoWrapper = document.getElementById('video-wrapper');
const clickLayer = document.getElementById('click-layer');

function getImageDisplayInfo() {
  const rect = videoStream.getBoundingClientRect();
  const naturalW = videoStream.naturalWidth || 640;
  const naturalH = videoStream.naturalHeight || 480;
  if (rect.width === 0 || rect.height === 0) {
    return { rect, naturalW, naturalH, scale: 1, displayW: 0, displayH: 0, offsetX: 0, offsetY: 0 };
  }
  // object-fit: contain logic
  const scale = Math.min(rect.width / naturalW, rect.height / naturalH);
  const displayW = naturalW * scale;
  const displayH = naturalH * scale;
  const offsetX = (rect.width - displayW) / 2;
  const offsetY = (rect.height - displayH) / 2;
  return { rect, naturalW, naturalH, scale, displayW, displayH, offsetX, offsetY };
}

function getImageCoords(clientX, clientY) {
  const { rect, naturalW, naturalH, scale, displayW, displayH, offsetX, offsetY } = getImageDisplayInfo();
  if (displayW === 0 || displayH === 0) return { x: 0, y: 0, inBounds: false };
  const px = clientX - rect.left - offsetX;
  const py = clientY - rect.top - offsetY;
  const inBounds = px >= 0 && py >= 0 && px <= displayW && py <= displayH;
  const x = px / scale;
  const y = py / scale;
  return {
    x: Math.max(0, Math.min(x, naturalW)),
    y: Math.max(0, Math.min(y, naturalH)),
    inBounds,
  };
}

function syncCanvas() {
  if (!clickLayer || !videoStream) return;
  const { rect, naturalW, naturalH, scale, displayW, displayH, offsetX, offsetY } = getImageDisplayInfo();
  if (rect.width === 0 || rect.height === 0) return;

  clickLayer.width = rect.width;
  clickLayer.height = rect.height;
  clickLayer.style.width = rect.width + 'px';
  clickLayer.style.height = rect.height + 'px';

  const ctx = clickLayer.getContext('2d');
  ctx.clearRect(0, 0, clickLayer.width, clickLayer.height);

  if (displayW === 0 || displayH === 0) return;

  const scaleX = scale;
  const scaleY = scale;

  // 1. 绘制粗定位点（菱形样式，完成后消失）
  if (coarsePoint) {
    const sx = offsetX + coarsePoint.x * scaleX;
    const sy = offsetY + coarsePoint.y * scaleY;
    
    // 菱形外框
    ctx.beginPath();
    ctx.moveTo(sx, sy - 12);
    ctx.lineTo(sx + 12, sy);
    ctx.lineTo(sx, sy + 12);
    ctx.lineTo(sx - 12, sy);
    ctx.closePath();
    ctx.strokeStyle = '#F59E0B';  // Orange for coarse
    ctx.lineWidth = 2;
    ctx.stroke();
    
    // 中心点
    ctx.beginPath();
    ctx.arc(sx, sy, 3, 0, Math.PI * 2);
    ctx.fillStyle = '#F59E0B';
    ctx.fill();
    
    // 标签
    ctx.font = '600 11px "Inter"';
    ctx.fillStyle = '#F59E0B';
    ctx.fillText('COARSE', sx + 15, sy + 4);
  }

  // 2. 绘制关键点（圆形样式）
  keypointClicks.forEach((p, i) => {
    const sx = offsetX + p.x * scaleX;
    const sy = offsetY + p.y * scaleY;

    // Outer Ring
    ctx.beginPath();
    ctx.arc(sx, sy, 8, 0, Math.PI * 2);
    ctx.strokeStyle = '#2563EB'; // Accent Blue
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // Center Dot
    ctx.beginPath();
    ctx.arc(sx, sy, 2, 0, Math.PI * 2);
    ctx.fillStyle = '#FFFFFF';
    ctx.fill();

    // Dotted Line Connection
    if (i > 0) {
      const prev = keypointClicks[i - 1];
      const prevSx = offsetX + prev.x * scaleX;
      const prevSy = offsetY + prev.y * scaleY;
      ctx.beginPath();
      ctx.moveTo(prevSx, prevSy);
      ctx.lineTo(sx, sy);
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.4)';
      ctx.lineWidth = 1;
      ctx.setLineDash([3, 3]);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // Number Label
    ctx.font = '500 10px "Inter"';
    ctx.fillStyle = '#FFFFFF';
    ctx.fillText(i + 1, sx + 10, sy + 3);
  });
}

function log(msg, type = 'info') {
  const box = document.getElementById('sys-log');
  if (!box) return;

  const time = new Date().toLocaleTimeString('zh-CN', { hour12: false, hour: '2-digit', minute: '2-digit', second: '2-digit' });
  
  // Minimalist Log Colors
  let colorClass = 'text-primary';
  if (type === 'error') colorClass = 'text-red-500'; // Handled via inline style usually, or class
  let colorStyle = type === 'error' ? 'color:#EF4444' : (type === 'warn' ? 'color:#F59E0B' : '');

  const entry = document.createElement('div');
  entry.className = 'log-line';
  entry.innerHTML = `<span class="ts">${time}</span><span class="msg" style="${colorStyle}">${msg}</span>`;
  box.appendChild(entry);
  box.scrollTop = box.scrollHeight;
}

function clearLog() {
  const box = document.getElementById('sys-log');
  if (box) box.innerHTML = '';
}

async function postJson(endpoint, data) {
  const res = await fetch(`${API_BASE}${endpoint}`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(data),
  });
  const json = await res.json();
  if (!res.ok) throw new Error(json.detail || 'Request failed');
  return json;
}

async function sendCommand(cmd) {
  try {
    if (cmd === 'set_keypoints') {
      if (keypointClicks.length === 0) {
        log('No keypoints selected.', 'warn');
        return;
      }
      const body = { x: keypointClicks.map((p) => p.x), y: keypointClicks.map((p) => p.y) };
      const res = await postJson('/set_keypoints', body);
      log('Keypoints synchronized: ' + res.message, 'success');
      return;
    }
    const res = await postJson('/control', { command: cmd });
    log(`CMD [${cmd}]: ${res.message}`, res.success ? 'success' : 'warn');
    if (cmd === 'start' && res.success) {
      keypointClicks = [];
      syncCanvas();
    }
  } catch (e) {
    log(e.message, 'error');
  }
}

async function sendReset() {
  if (!confirm('Reset system state?')) return;
  try {
    const res = await postJson('/reset', {});
    log('系统重置: ' + res.message, res.success ? 'success' : 'warn');
    if (res.success) {
      keypointClicks = [];
      coarsePoint = null;
      coarseDone = false;
      syncCanvas();
      document.getElementById('status-coarse').innerText = 'PENDING';
      document.getElementById('status-coarse').style.color = 'var(--text-primary)';
      log('状态: 等待设置粗定位点', 'info');
    }
  } catch (e) {
    log(e.message, 'error');
  }
}

async function toggleArmMode() {
  try {
    const res = await postJson('/toggle_arm_mode', {});
    log(res.message, res.success ? 'success' : 'error');
  } catch (e) {
    log('Toggle arm mode failed: ' + e.message, 'error');
  }
}

async function returnHome() {
  if (!confirm('Return arm to initial position?')) return;
  try {
    const res = await postJson('/return_home', {});
    log('Returning to Initial Position: ' + res.message, 'success');
  } catch (e) { log(e.message, 'error'); }
}

videoWrapper.addEventListener('click', async (e) => {
  if (!videoStream.complete || videoStream.naturalWidth === 0) return;

  const { x, y, inBounds } = getImageCoords(e.clientX, e.clientY);
  if (!inBounds) return;

  if (!coarseDone) {
    // 粗定位阶段：设置粗定位点
    coarsePoint = { x, y };
    log(`粗定位点设置: (${Math.round(x)}, ${Math.round(y)})`, 'info');
    log(`状态: 粗定位中...`, 'info');
    syncCanvas();

    try {
      await postJson('/set_coarse_point', { x, y });
    } catch (e) {
      log('粗定位点发送失败', 'error');
    }
  } else {
    // 关键点跟踪阶段：添加关键点
    keypointClicks.push({ x, y });
    log(`关键点设置: (${Math.round(x)}, ${Math.round(y)})，共 ${keypointClicks.length} 个关键点`, 'info');
    syncCanvas();
  }
});

// Event Listeners
document.getElementById('btn-set-kp')?.addEventListener('click', () => sendCommand('set_keypoints'));
document.getElementById('btn-start')?.addEventListener('click', () => sendCommand('start'));
document.getElementById('btn-stop')?.addEventListener('click', () => sendCommand('stop'));

document.getElementById('btn-toggle-arm-mode')?.addEventListener('click', toggleArmMode);

document.getElementById('btn-return-home')?.addEventListener('click', returnHome);
document.getElementById('btn-reset')?.addEventListener('click', sendReset);
document.getElementById('btn-clear-log')?.addEventListener('click', clearLog);

videoStream?.addEventListener('load', syncCanvas);
window.addEventListener('resize', syncCanvas);

// Polling Loop
setInterval(async () => {
  const dot = document.getElementById('ros-status-dot');
  const connText = document.getElementById('connection-text');
  
  try {
    const res = await fetch(`${API_BASE}/status`);
    if (res.ok) {
      const data = await res.json();
      
      if (dot) {
        dot.className = 'status-dot ok';
        connText.innerText = "ONLINE";
        connText.style.color = "var(--text-secondary)";
      }

      const coarseText = document.getElementById('status-coarse');
      if (coarseText) {
        if (data.coarse_done) {
          coarseText.innerText = 'LOCKED';
          coarseText.style.color = 'var(--status-green)';
          // 粗定位完成后隐藏粗定位点
          if (coarsePoint) {
            coarsePoint = null;
            coarseDone = true;  // 标记粗定位已完成
            syncCanvas();
            log('粗定位完成，等待设置关键点...', 'info');
          }
        } else {
          coarseText.innerText = 'PENDING';
          coarseText.style.color = 'var(--text-primary)';
          coarseDone = false;  // 重置状态
        }
      }

      const kpCount = data.keypoints ? data.keypoints.length : 0;
      const kpCountEl = document.getElementById('kp-count');
      const kpProgress = document.getElementById('kp-progress');
      if (kpCountEl) kpCountEl.innerText = kpCount;
      if (kpProgress) kpProgress.style.width = Math.min((kpCount / 20) * 100, 100) + '%';
    } else throw new Error();
  } catch (e) {
    if (dot) {
      dot.className = 'status-dot err';
      connText.innerText = "OFFLINE";
      connText.style.color = "var(--status-red)";
    }
  }
}, 1000);

log('System ready.');
setInterval(updatePoseDisplay, 500);