#include <WiFi.h>
#include <driver/gpio.h>
#include <esp_http_server.h>
#include <esp_log.h>
#include <esp_rom_gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <soc/uart_periph.h>
#include <string.h>
#include <unistd.h>

// =========================================================================
// WiFi SoftAP Configuration
// =========================================================================
static const char *AP_SSID = "XE_CAN_BANG_WIFI";
static const char *AP_PASS = "12345678";

// =========================================================================
// UART & Pin Configuration (ESP32-C6 SuperMini <-> STM32L476RG)
//   MAC DINH: ESP32 RX (GPIO17) <-> STM32 TX (A4/PC1 hoac PA2)
//             ESP32 TX (GPIO18 va dong bo ra chan in "TX" GPIO16) <-> STM32 RX (D6/PB10 hoac PA3)
//             GND <-> GND
// =========================================================================
static const int RX_PIN = 17;
static const int TX_PIN = 18;
static const int TX0_ALT_PIN = 16;
static const int LED_BUILTIN_C6 = 15;
static const unsigned long CMD_WATCHDOG_MS = 450UL;

HardwareSerial stm32Serial(1);
static httpd_handle_t httpServer = nullptr;
static SemaphoreHandle_t stateMutex = nullptr;

// Driving & diagnostic state (protected by stateMutex)
static bool driveFB = false;
static bool driveTurn = false;
static unsigned long lastHeartbeatMs = 0;
static int controlClientFd = -1;
static uint32_t txCmdCount = 0;
static uint32_t rxByteCount = 0;
static char lastTxCmd = 'X';
static bool hasRealStat = false;
static bool diagPending = false;
static unsigned long lastDiagMs = 0;

// Latest telemetry line from STM32 (protected by stateMutex)
static char uartRxBuf[160];
static size_t uartRxLen = 0;
static char latestStat[160] = "STAT:0.0,0.0,0,X,INIT,0,0.0,0,0,0,WAIT";
static bool broadcastPending = false;

// =========================================================================
// Embedded Web Control UI (WebSocket + Hold-once + Keep-alive 'k')
// =========================================================================
static const char HTML_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>Xe Can Bang - WebSocket</title>
<style>
  * { box-sizing:border-box; user-select:none; -webkit-user-select:none; -webkit-touch-callout:none; }
  body {
    margin:0; padding:12px; font-family:system-ui,-apple-system,sans-serif;
    background:#0f172a; color:#f8fafc; text-align:center;
    display:flex; flex-direction:column; align-items:center;
  }
  h2 { margin:4px 0 8px; font-size:18px; color:#38bdf8; }
  .badges { display:flex; gap:8px; justify-content:center; flex-wrap:wrap; margin-bottom:10px; }
  .badge {
    font-size:12px; font-weight:700; padding:5px 10px; border-radius:999px;
    background:#1e293b; border:1px solid #334155; color:#cbd5e1;
  }
  .badge.ok { background:#064e3b; border-color:#10b981; color:#a7f3d0; }
  .badge.warn { background:#78350f; border-color:#f59e0b; color:#fde68a; }
  .badge.err { background:#7f1d1d; border-color:#ef4444; color:#fecaca; }
  .panel {
    width:100%; max-width:390px; background:#1e293b; border-radius:12px;
    padding:10px 12px; margin-bottom:12px; border:1px solid #334155;
  }
  .grid-stat { display:grid; grid-template-columns:1fr 1fr 1fr; gap:8px; }
  .stat-item { background:#0f172a; border-radius:8px; padding:6px 4px; }
  .stat-lbl { font-size:11px; color:#94a3b8; }
  .stat-val { font-size:14px; font-weight:700; color:#e2e8f0; margin-top:2px; }
  .row-btns { display:flex; gap:8px; justify-content:center; margin-bottom:10px; width:100%; max-width:390px; }
  .tier-btn {
    flex:1; padding:10px 6px; border-radius:10px; border:1px solid #334155;
    background:#1e293b; color:#cbd5e1; font-weight:700; font-size:13px; cursor:pointer;
  }
  .tier-btn.sel { background:#0284c7; border-color:#38bdf8; color:#fff; }
  .dpad {
    display:grid; grid-template-columns:repeat(3,88px); grid-template-rows:repeat(3,76px);
    gap:10px; margin:6px 0 12px;
  }
  .btn {
    background:#1e293b; border:2px solid #334155; border-radius:14px; color:#f8fafc;
    font-size:16px; font-weight:700; display:flex; flex-direction:column;
    align-items:center; justify-content:center; cursor:pointer; touch-action:none;
    transition:transform .06s, background .08s, border-color .08s;
  }
  .btn small { font-size:10px; color:#94a3b8; margin-top:2px; font-weight:500; }
  .btn.active, .btn:active { background:#0284c7; border-color:#38bdf8; transform:scale(0.95); }
  .btn.stop { background:#991b1b; border-color:#ef4444; }
  .btn.stop.active { background:#dc2626; }
  .act-btn {
    flex:1; padding:11px 8px; border-radius:10px; border:none;
    color:#fff; font-weight:700; font-size:13px; cursor:pointer; touch-action:manipulation;
  }
  .act-btn:disabled { opacity:0.4; cursor:not-allowed; }
  .c-rearm { background:#059669; }
  .c-estop { background:#dc2626; }
  .c-imu { background:#475569; }
  .c-zero { background:#7c3aed; }
  .log-panel {
    width:100%; max-width:390px; background:#1e293b; border-radius:12px;
    padding:8px 10px; margin-top:10px; border:1px solid #334155; text-align:left;
  }
  .log-hdr { display:flex; align-items:center; justify-content:space-between; gap:6px; margin-bottom:6px; }
  .log-hdr span { font-size:11px; font-weight:700; color:#38bdf8; }
  .log-btns { display:flex; gap:6px; }
  .log-btn {
    padding:4px 8px; border-radius:6px; border:1px solid #475569;
    background:#0f172a; color:#e2e8f0; font-size:11px; font-weight:600; cursor:pointer;
  }
  .log-btn.paused { background:#78350f; border-color:#f59e0b; color:#fde68a; }
  #logBox {
    width:100%; height:112px; background:#0f172a; color:#cbd5e1;
    border:1px solid #334155; border-radius:8px; padding:6px;
    font-family:ui-monospace,Consolas,monospace; font-size:10px; line-height:1.35;
    resize:none; user-select:text; -webkit-user-select:text;
  }
  .hint { font-size:11px; color:#94a3b8; max-width:390px; line-height:1.4; }
</style>
</head>
<body oncontextmenu="return false">
  <h2>XE CAN BANG - WEBSOCKET</h2>
  <div class="badges">
    <span id="wsBadge" class="badge err">WS: DANG KET NOI...</span>
    <span id="stateBadge" class="badge">TRANG THAI: --</span>
  </div>
  <div id="diagBar" style="font-size:11px;color:#94a3b8;text-align:center;margin-bottom:8px;">TX: 0 cmd (--) &bull; RX UART: 0 B &bull; WS: 0</div>

  <div class="panel">
    <div class="grid-stat">
      <div class="stat-item"><div class="stat-lbl">Goc Pitch</div><div class="stat-val" id="vPitch">--</div></div>
      <div class="stat-item"><div class="stat-lbl">Theta Ref</div><div class="stat-val" id="vRef">--</div></div>
      <div class="stat-item"><div class="stat-lbl">Zero Goc</div><div class="stat-val" id="vZero">--</div></div>
      <div class="stat-item"><div class="stat-lbl">V Lenh</div><div class="stat-val" id="vCmd">--</div></div>
      <div class="stat-item"><div class="stat-lbl">V Thuc</div><div class="stat-val" id="vReal">--</div></div>
      <div class="stat-item"><div class="stat-lbl">Lenh</div><div class="stat-val" id="vAct">--</div></div>
      <div class="stat-item"><div class="stat-lbl">PWM Out</div><div class="stat-val" id="vOut">--</div></div>
      <div class="stat-item"><div class="stat-lbl">Enc L / R</div><div class="stat-val" id="vEnc">--</div></div>
      <div class="stat-item"><div class="stat-lbl">Enc State</div><div class="stat-val" id="vEncSt">--</div></div>
    </div>
  </div>

  <div class="row-btns">
    <button class="tier-btn" id="t1" onclick="setTier('1')">Cham<br><small>0.14 m/s</small></button>
    <button class="tier-btn sel" id="t2" onclick="setTier('2')">Vua<br><small>0.22 m/s</small></button>
    <button class="tier-btn" id="t3" onclick="setTier('3')">Nhanh<br><small>0.30 m/s</small></button>
  </div>

  <div class="dpad">
    <div></div>
    <button class="btn" id="btnW">TIEN<small>W</small></button>
    <div></div>
    <button class="btn" id="btnA">TRAI<small>A</small></button>
    <button class="btn stop" id="btnX" onclick="stopAll()">DUNG<small>X / Space</small></button>
    <button class="btn" id="btnD">PHAI<small>D</small></button>
    <div></div>
    <button class="btn" id="btnS">LUI<small>S</small></button>
    <div></div>
  </div>

  <div class="row-btns">
    <button class="act-btn c-rearm" onclick="sendCmd('r')">MO KHOA / ARM (R)</button>
    <button class="act-btn c-estop" onclick="sendCmd('!')">CAT KHAN CAP (!)</button>
  </div>

  <div class="row-btns">
    <button class="act-btn c-imu" id="btnCalib" onclick="safeCalib()">Calib IMU (I)</button>
    <button class="act-btn c-zero" id="btnZero" onclick="safeZero()">Set Zero (Z)</button>
  </div>

  <div class="hint" id="hintBox">
    Giu nut de chay, tha nut de dung ngay. Khi giu nut, Web tu gui lai chinh lenh w/s/a/d moi 150ms de mat 1 byte van tu khoi phuc. Calib IMU / Set Zero chi dung khi xe da dung / nam nghi.
  </div>

  <div class="log-panel">
    <div class="log-hdr">
      <span>LOG TELEMETRY (T1-T4)</span>
      <div class="log-btns">
        <button class="log-btn" id="btnLogPause" onclick="toggleLogPause()">Tam dung</button>
        <button class="log-btn" id="btnLogCopy" onclick="copyLog()">Copy</button>
        <button class="log-btn" onclick="clearLog()">Xoa</button>
      </div>
    </div>
    <textarea id="logBox" readonly placeholder="t_ms,pitch,thRef,vCmd,cmd,state,vReal,zero,out,encL,encR,encSt"></textarea>
  </div>

<script>
  let ws = null;
  let reconnectTimer = null;
  let activeFB = null;
  let activeTurn = null;
  let hbTimer = null;
  let robotReason = "--";
  let logPaused = false;
  let logStartMs = 0;
  const logLines = [];

  function renderLogBox() {
    const box = document.getElementById("logBox");
    if (!box) return;
    box.value = "t_ms,pitch,thRef,vCmd,cmd,state,vReal,zero,out,encL,encR,encSt\n" + logLines.join("\n");
    box.scrollTop = box.scrollHeight;
  }

  function toggleLogPause() {
    logPaused = !logPaused;
    const btn = document.getElementById("btnLogPause");
    if (btn) {
      btn.innerText = logPaused ? "Tiep tuc" : "Tam dung";
      btn.classList.toggle("paused", logPaused);
    }
  }

  function clearLog() {
    logLines.length = 0;
    logStartMs = 0;
    renderLogBox();
  }

  function copyLog() {
    const box = document.getElementById("logBox");
    const btn = document.getElementById("btnLogCopy");
    if (!box) return;
    const text = box.value;
    const markCopied = () => {
      if (!btn) return;
      btn.innerText = "Da copy";
      setTimeout(() => { btn.innerText = "Copy"; }, 1200);
    };
    if (navigator.clipboard && navigator.clipboard.writeText) {
      navigator.clipboard.writeText(text).then(markCopied).catch(() => {
        box.select();
        try { document.execCommand("copy"); markCopied(); } catch (_) {}
      });
    } else {
      box.select();
      try { document.execCommand("copy"); markCopied(); } catch (_) {}
    }
  }

  function handleIncomingLine(text) {
    if (text.startsWith("DIAG:")) {
      const d = text.substring(5).split(",");
      if (d.length >= 4) {
        document.getElementById("diagBar").innerText =
          "TX: " + d[0] + " cmd (" + d[1] + ") \u2022 RX UART: " + d[2] + " B \u2022 WS: " + d[3];
      }
      return;
    }
    if (!text.startsWith("STAT:")) return;
    const raw = text.substring(5);
    const p = raw.split(",");
    if (p.length < 6) return;
    document.getElementById("vPitch").innerText = p[0] + "°";
    document.getElementById("vRef").innerText = p[1] + "°";
    document.getElementById("vCmd").innerText = p[2] + " cm/s";
    document.getElementById("vAct").innerText = p[3];
    document.getElementById("vReal").innerText = p[5] + " cm/s";
    document.getElementById("vZero").innerText = (p.length >= 7 ? p[6] : "--") + "°";
    document.getElementById("vOut").innerText = (p.length >= 8 ? p[7] : "--");
    document.getElementById("vEnc").innerText = (p.length >= 10 ? (p[8] + " / " + p[9]) : "--");
    document.getElementById("vEncSt").innerText = (p.length >= 11 ? p[10] : "--");

    if (!logPaused) {
      const now = Date.now();
      if (logStartMs === 0) logStartMs = now;
      logLines.push((now - logStartMs) + "," + raw);
      if (logLines.length > 35) logLines.shift();
      renderLogBox();
    }

    robotReason = p[4];
    const sb = document.getElementById("stateBadge");
    const isRun = (robotReason === "BALANCE" || robotReason === "OK");
    sb.innerText = "TRANG THAI: " + robotReason;
    sb.className = isRun ? "badge ok" : (robotReason === "FALL_WAIT" ? "badge warn" : "badge err");

    document.getElementById("btnCalib").disabled = isRun;
    document.getElementById("btnZero").disabled = isRun;
  }

  function connectWS() {
    if (ws && (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING)) return;
    const proto = (location.protocol === "https:") ? "wss://" : "ws://";
    ws = new WebSocket(proto + location.host + "/ws");

    ws.onopen = () => {
      const b = document.getElementById("wsBadge");
      b.innerText = "WS: DA KET NOI";
      b.className = "badge ok";
    };

    ws.onclose = () => {
      const b = document.getElementById("wsBadge");
      b.innerText = "WS: MAT KET NOI (HTTP FALLBACK)";
      b.className = "badge warn";
      clearLocalHolds(false);
      if (!reconnectTimer) {
        reconnectTimer = setTimeout(() => { reconnectTimer = null; connectWS(); }, 800);
      }
    };

    ws.onerror = () => {
      if (ws) ws.close();
    };

    ws.onmessage = (ev) => {
      const lines = (ev.data || "").split("\n");
      for (let i = 0; i < lines.length; i++) {
        const line = lines[i].trim();
        if (line) handleIncomingLine(line);
      }
    };
  }

  function sendCmd(c) {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(c);
    } else {
      fetch("/cmd?c=" + encodeURIComponent(c), { cache: "no-store" })
        .then(r => r.text())
        .then(t => {
          const lines = t.split("\n");
          for (let i = 0; i < lines.length; i++) {
            const line = lines[i].trim();
            if (line) handleIncomingLine(line);
          }
        })
        .catch(() => {});
    }
  }

  function ensureHeartbeat() {
    if (activeFB || activeTurn) {
      if (!hbTimer) {
        hbTimer = setInterval(() => {
          if (activeFB) sendCmd(activeFB);
          if (activeTurn) sendCmd(activeTurn);
        }, 150);
      }
    } else if (hbTimer) {
      clearInterval(hbTimer);
      hbTimer = null;
    }
  }

  function clearLocalHolds(sendStop) {
    activeFB = null;
    activeTurn = null;
    ensureHeartbeat();
    ["btnW","btnS","btnA","btnD"].forEach(id => {
      const el = document.getElementById(id);
      if (el) el.classList.remove("active");
    });
    if (sendStop) sendCmd("x");
  }

  function stopAll() {
    clearLocalHolds(true);
  }

  function setTier(t) {
    ["1","2","3"].forEach(x => document.getElementById("t" + x).classList.toggle("sel", x === t));
    sendCmd(t);
  }

  function safeCalib() {
    if (robotReason === "BALANCE" || robotReason === "OK") return;
    if (confirm("Dat xe nam yen / dung han truoc khi Calib IMU. Tiep tuc?")) sendCmd("i");
  }

  function safeZero() {
    if (robotReason === "BALANCE" || robotReason === "OK") return;
    if (confirm("Giu xe o goc can bang tinh (<= 15 do) de luu Zero? Tiep tuc?")) sendCmd("z");
  }

  function bindHoldBtn(id, pressCmd, releaseCmd, isTurn) {
    const el = document.getElementById(id);
    let hasCapture = false;
    const startHold = (e) => {
      if (e) e.preventDefault();
      hasCapture = false;
      if (el.setPointerCapture && e && e.pointerId !== undefined) {
        try { el.setPointerCapture(e.pointerId); hasCapture = true; } catch (_) {}
      }
      el.classList.add("active");
      if (isTurn) {
        if (activeTurn !== pressCmd) { activeTurn = pressCmd; sendCmd(pressCmd); }
      } else {
        if (activeFB !== pressCmd) { activeFB = pressCmd; sendCmd(pressCmd); }
      }
      ensureHeartbeat();
    };
    const endHold = (e) => {
      if (e) e.preventDefault();
      el.classList.remove("active");
      if (isTurn) {
        if (activeTurn === pressCmd) { activeTurn = null; sendCmd(releaseCmd); }
      } else {
        if (activeFB === pressCmd) { activeFB = null; sendCmd(releaseCmd); }
      }
      ensureHeartbeat();
    };
    if (window.PointerEvent) {
      el.addEventListener("pointerdown", startHold);
      el.addEventListener("pointerup", endHold);
      el.addEventListener("pointercancel", endHold);
      el.addEventListener("lostpointercapture", endHold);
      el.addEventListener("pointerleave", (e) => { if (!hasCapture) endHold(e); });
    } else {
      el.addEventListener("touchstart", startHold, { passive: false });
      el.addEventListener("touchend", endHold, { passive: false });
      el.addEventListener("touchcancel", endHold, { passive: false });
      el.addEventListener("mousedown", startHold);
      el.addEventListener("mouseup", endHold);
      el.addEventListener("mouseleave", endHold);
    }
  }

  bindHoldBtn("btnW", "w", "v", false);
  bindHoldBtn("btnS", "s", "v", false);
  bindHoldBtn("btnA", "a", "c", true);
  bindHoldBtn("btnD", "d", "c", true);

  document.addEventListener("keydown", (e) => {
    if (e.repeat) return;
    const k = e.key.toLowerCase();
    if (k === "w" && activeFB !== "w") { activeFB = "w"; sendCmd("w"); document.getElementById("btnW").classList.add("active"); ensureHeartbeat(); }
    else if (k === "s" && activeFB !== "s") { activeFB = "s"; sendCmd("s"); document.getElementById("btnS").classList.add("active"); ensureHeartbeat(); }
    else if (k === "a" && activeTurn !== "a") { activeTurn = "a"; sendCmd("a"); document.getElementById("btnA").classList.add("active"); ensureHeartbeat(); }
    else if (k === "d" && activeTurn !== "d") { activeTurn = "d"; sendCmd("d"); document.getElementById("btnD").classList.add("active"); ensureHeartbeat(); }
    else if (k === " " || k === "x") { e.preventDefault(); stopAll(); }
    else if (k === "1" || k === "2" || k === "3") { setTier(k); }
    else if (k === "r") { sendCmd("r"); }
    else if (k === "escape") { sendCmd("!"); }
  });

  document.addEventListener("keyup", (e) => {
    const k = e.key.toLowerCase();
    if (k === "w" && activeFB === "w") { activeFB = null; sendCmd("v"); document.getElementById("btnW").classList.remove("active"); ensureHeartbeat(); }
    else if (k === "s" && activeFB === "s") { activeFB = null; sendCmd("v"); document.getElementById("btnS").classList.remove("active"); ensureHeartbeat(); }
    else if (k === "a" && activeTurn === "a") { activeTurn = null; sendCmd("c"); document.getElementById("btnA").classList.remove("active"); ensureHeartbeat(); }
    else if (k === "d" && activeTurn === "d") { activeTurn = null; sendCmd("c"); document.getElementById("btnD").classList.remove("active"); ensureHeartbeat(); }
  });

  window.addEventListener("blur", () => clearLocalHolds(true));
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) clearLocalHolds(true);
  });

  connectWS();
  setInterval(() => {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
      fetch("/status", { cache: "no-store" })
        .then(r => r.text())
        .then(t => {
          const lines = t.split("\n");
          for (let i = 0; i < lines.length; i++) {
            const line = lines[i].trim();
            if (line) handleIncomingLine(line);
          }
        })
        .catch(() => {});
    }
  }, 600);
</script>
</body>
</html>
)rawliteral";

// =========================================================================
// Helpers
// =========================================================================
static bool isValidCommandChar(char c) {
  switch (c) {
    case 'w':
    case 's':
    case 'v':
    case 'a':
    case 'd':
    case 'c':
    case 'k':
    case 'x':
    case '1':
    case '2':
    case '3':
    case 'i':
    case 'z':
    case 'r':
    case '!':
      return true;
    default:
      return false;
  }
}

// Goi lenh UART qua UART1 (dong thoi xuat ra ca GPIO18 va GPIO16 qua GPIO Matrix phan cung).
// Ham nay phai duoc goi khi da giu stateMutex.
static void sendStm32Byte(uint8_t b) {
  stm32Serial.write(b);
  if (b != 'k') {
    txCmdCount++;
    lastTxCmd = static_cast<char>(b);
    diagPending = true;
  }
}

static void processCommandChar(char rawChar, int clientFd) {
  char c = (rawChar == 'K') ? 'k' : rawChar;
  if (!isValidCommandChar(c)) {
    return;
  }

  unsigned long now = millis();
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    switch (c) {
      case 'w':
      case 's':
        driveFB = true;
        controlClientFd = clientFd;
        lastHeartbeatMs = now;
        break;
      case 'v':
        driveFB = false;
        lastHeartbeatMs = now;
        break;
      case 'a':
      case 'd':
        driveTurn = true;
        controlClientFd = clientFd;
        lastHeartbeatMs = now;
        break;
      case 'c':
        driveTurn = false;
        lastHeartbeatMs = now;
        break;
      case 'k':
        lastHeartbeatMs = now;
        break;
      case 'x':
      case '!':
        driveFB = false;
        driveTurn = false;
        break;
      default:
        break;
    }
    sendStm32Byte(static_cast<uint8_t>(c));
    xSemaphoreGive(stateMutex);
  }

  // Dao trang thai LED_BUILTIN_C6 (GPIO15) moi khi nhan lenh hoac heartbeat 'k'
  // de den nhay sang ro rang tren ca mach Active-LOW lan Active-HIGH.
  static bool ledToggle = false;
  if (c == 'v' || c == 'c' || c == 'x' || c == '!') {
    ledToggle = false;
    digitalWrite(LED_BUILTIN_C6, HIGH);
  } else {
    ledToggle = !ledToggle;
    digitalWrite(LED_BUILTIN_C6, ledToggle ? LOW : HIGH);
  }
}

// Broadcast latest STAT and DIAG lines strictly within the httpd task context.
static void wsBroadcastWork(void *arg) {
  (void)arg;
  if (httpServer == nullptr) {
    return;
  }

  char statPayload[160];
  size_t statLen = 0;
  bool sendStat = false;
  uint32_t txSnap = 0;
  uint32_t rxSnap = 0;
  char cmdSnap = 'X';

  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    if (hasRealStat) {
      statLen = strnlen(latestStat, sizeof(latestStat) - 1);
      memcpy(statPayload, latestStat, statLen);
      statPayload[statLen] = '\0';
      sendStat = true;
    }
    txSnap = txCmdCount;
    rxSnap = rxByteCount;
    cmdSnap = lastTxCmd;
    broadcastPending = false;
    diagPending = false;
    xSemaphoreGive(stateMutex);
  } else {
    return;
  }

  int clientFds[7];
  size_t fds = sizeof(clientFds) / sizeof(clientFds[0]);
  if (httpd_get_client_list(httpServer, &fds, clientFds) != ESP_OK) {
    return;
  }

  int wsClients = 0;
  for (size_t i = 0; i < fds; ++i) {
    if (httpd_ws_get_fd_info(httpServer, clientFds[i]) == HTTPD_WS_CLIENT_WEBSOCKET) {
      wsClients++;
    }
  }
  if (wsClients == 0) {
    return;
  }

  char combinedPayload[256];
  int combinedLen = 0;
  if (sendStat && statLen > 0) {
    combinedLen = snprintf(combinedPayload, sizeof(combinedPayload), "%s\nDIAG:%lu,%c,%lu,%d",
                           statPayload,
                           static_cast<unsigned long>(txSnap), cmdSnap,
                           static_cast<unsigned long>(rxSnap), wsClients);
  } else {
    combinedLen = snprintf(combinedPayload, sizeof(combinedPayload), "DIAG:%lu,%c,%lu,%d",
                           static_cast<unsigned long>(txSnap), cmdSnap,
                           static_cast<unsigned long>(rxSnap), wsClients);
  }
  if (combinedLen <= 0) {
    return;
  }

  httpd_ws_frame_t wsPkt;
  memset(&wsPkt, 0, sizeof(wsPkt));
  wsPkt.type = HTTPD_WS_TYPE_TEXT;
  wsPkt.payload = reinterpret_cast<uint8_t *>(combinedPayload);
  wsPkt.len = static_cast<size_t>(combinedLen);
  wsPkt.final = true;

  // QUAN TRONG: Trong callback httpd_queue_work (dang chay tren chinh task httpd),
  // PHAI dung httpd_ws_send_frame_async() thay vi httpd_ws_send_data() de tranh tu khoa (deadlock) task httpd!
  for (size_t i = 0; i < fds; ++i) {
    if (httpd_ws_get_fd_info(httpServer, clientFds[i]) == HTTPD_WS_CLIENT_WEBSOCKET) {
      httpd_ws_send_frame_async(httpServer, clientFds[i], &wsPkt);
    }
  }
}

// =========================================================================
// ESP-IDF HTTP + WebSocket Handlers
// =========================================================================
static void onSocketClose(httpd_handle_t hd, int sockfd) {
  (void)hd;
  bool shouldStop = false;
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    if (sockfd == controlClientFd && (driveFB || driveTurn)) {
      driveFB = false;
      driveTurn = false;
      controlClientFd = -1;
      shouldStop = true;
    } else if (sockfd == controlClientFd) {
      controlClientFd = -1;
    }
    if (shouldStop) {
      sendStm32Byte('x');
    }
    xSemaphoreGive(stateMutex);
  }
  if (sockfd >= 0) {
    close(sockfd);
  }
}

static esp_err_t indexHandler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, HTML_PAGE, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t statusHandler(httpd_req_t *req) {
  char payload[256];
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    snprintf(payload, sizeof(payload), "%s\nDIAG:%lu,%c,%lu,0",
             latestStat,
             static_cast<unsigned long>(txCmdCount),
             lastTxCmd,
             static_cast<unsigned long>(rxByteCount));
    xSemaphoreGive(stateMutex);
  } else {
    strncpy(payload, "BUSY", sizeof(payload));
  }
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, payload, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t cmdHandler(httpd_req_t *req) {
  char query[32];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
    char val[8];
    if (httpd_query_key_value(query, "c", val, sizeof(val)) == ESP_OK && val[0] != '\0') {
      processCommandChar(val[0], -1);
    }
  }
  return statusHandler(req);
}

static esp_err_t wsHandler(httpd_req_t *req) {
  if (req->method == HTTP_GET) {
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      diagPending = true;
      xSemaphoreGive(stateMutex);
    }
    return ESP_OK;
  }

  httpd_ws_frame_t wsPkt;
  memset(&wsPkt, 0, sizeof(wsPkt));
  wsPkt.type = HTTPD_WS_TYPE_TEXT;

  esp_err_t ret = httpd_ws_recv_frame(req, &wsPkt, 0);
  if (ret != ESP_OK) {
    return ret;
  }
  if (wsPkt.len == 0) {
    return ESP_OK;
  }

  uint8_t buf[65];
  if (wsPkt.len >= sizeof(buf)) {
    return ESP_ERR_INVALID_SIZE;
  }

  wsPkt.payload = buf;
  ret = httpd_ws_recv_frame(req, &wsPkt, wsPkt.len);
  if (ret != ESP_OK) {
    return ret;
  }
  buf[wsPkt.len] = '\0';

  if (wsPkt.type == HTTPD_WS_TYPE_TEXT) {
    int fd = httpd_req_to_sockfd(req);
    for (size_t i = 0; i < wsPkt.len; ++i) {
      processCommandChar(static_cast<char>(buf[i]), fd);
    }
  }
  return ESP_OK;
}

static void startWebServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.stack_size = 8192;
  config.server_port = 80;
  config.max_open_sockets = 7;
  config.lru_purge_enable = true;
  config.recv_wait_timeout = 5;
  config.send_wait_timeout = 5;
  config.close_fn = onSocketClose;

  if (httpd_start(&httpServer, &config) != ESP_OK) {
    return;
  }

  httpd_uri_t rootUri = {};
  rootUri.uri = "/";
  rootUri.method = HTTP_GET;
  rootUri.handler = indexHandler;
  httpd_register_uri_handler(httpServer, &rootUri);

  httpd_uri_t statusUri = {};
  statusUri.uri = "/status";
  statusUri.method = HTTP_GET;
  statusUri.handler = statusHandler;
  httpd_register_uri_handler(httpServer, &statusUri);

  httpd_uri_t cmdUri = {};
  cmdUri.uri = "/cmd";
  cmdUri.method = HTTP_GET;
  cmdUri.handler = cmdHandler;
  httpd_register_uri_handler(httpServer, &cmdUri);

  httpd_uri_t wsUri = {};
  wsUri.uri = "/ws";
  wsUri.method = HTTP_GET;
  wsUri.handler = wsHandler;
  wsUri.is_websocket = true;
  httpd_register_uri_handler(httpServer, &wsUri);
}

// =========================================================================
// Arduino Entry Points
// =========================================================================
void setup() {
  pinMode(LED_BUILTIN_C6, OUTPUT);
  digitalWrite(LED_BUILTIN_C6, HIGH);

  stateMutex = xSemaphoreCreateMutex();

  // Chi khoi tao DUY NHAT 1 bo UART1 (stm32Serial) tren RX=17, TX=18 de tranh xung dot driver/ngat voi Serial0.
  // Chuyen chan GPIO17 sang che do GPIO Matrix Input + Pullup va phan chieu tin hieu UART1_TX sang ca GPIO16 (TX) bang phan cung.
  esp_log_level_set("*", ESP_LOG_NONE);
  stm32Serial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  esp_rom_gpio_pad_select_gpio(RX_PIN);
  gpio_input_enable(static_cast<gpio_num_t>(RX_PIN));
  gpio_pullup_en(static_cast<gpio_num_t>(RX_PIN));
  esp_rom_gpio_connect_in_signal(RX_PIN, UART_PERIPH_SIGNAL(1, SOC_UART_RX_PIN_IDX), false);

  esp_rom_gpio_pad_select_gpio(TX0_ALT_PIN);
  gpio_set_direction(static_cast<gpio_num_t>(TX0_ALT_PIN), GPIO_MODE_OUTPUT);
  esp_rom_gpio_connect_out_signal(TX0_ALT_PIN, UART_PERIPH_SIGNAL(1, SOC_UART_TX_PIN_IDX), false, false);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  startWebServer();
}

static void consumeUartRxByte(int ch, uint32_t &rxNewBytes, bool &statUpdated, char *completedStat, size_t completedCap) {
  if (ch < 0) {
    return;
  }
  rxNewBytes++;
  if (ch == '\r') {
    return;
  }
  if (ch == '\n') {
    uartRxBuf[uartRxLen] = '\0';
    if (strncmp(uartRxBuf, "STAT:", 5) == 0) {
      strncpy(completedStat, uartRxBuf, completedCap - 1);
      completedStat[completedCap - 1] = '\0';
      statUpdated = true;
    }
    uartRxLen = 0;
  } else if (uartRxLen < sizeof(uartRxBuf) - 1) {
    uartRxBuf[uartRxLen++] = static_cast<char>(ch);
  } else {
    uartRxLen = 0;
  }
}

void loop() {
  unsigned long now = millis();

  // 1. Read incoming telemetry lines from STM32 on UART1 (GPIO17)
  uint32_t rxNewBytes = 0;
  bool statUpdated = false;
  char completedStat[160];
  while (stm32Serial.available() > 0) {
    consumeUartRxByte(stm32Serial.read(), rxNewBytes, statUpdated, completedStat, sizeof(completedStat));
  }

  bool triggerBroadcast = false;
  bool periodicDiagDue = (now - lastDiagMs >= 300UL);
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    rxByteCount += rxNewBytes;
    if (statUpdated) {
      strncpy(latestStat, completedStat, sizeof(latestStat) - 1);
      latestStat[sizeof(latestStat) - 1] = '\0';
      hasRealStat = true;
    }
    if (!broadcastPending && httpServer != nullptr &&
        (statUpdated || diagPending || periodicDiagDue)) {
      broadcastPending = true;
      triggerBroadcast = true;
      lastDiagMs = now;
    }
    xSemaphoreGive(stateMutex);
  }

  if (triggerBroadcast && httpServer != nullptr) {
    if (httpd_queue_work(httpServer, wsBroadcastWork, nullptr) != ESP_OK) {
      if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        broadcastPending = false;
        xSemaphoreGive(stateMutex);
      }
    }
  }

  // 2. Heartbeat watchdog: stop motion if hold commands stop refreshing within 450 ms
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    if ((driveFB || driveTurn) && (now - lastHeartbeatMs > CMD_WATCHDOG_MS)) {
      driveFB = false;
      driveTurn = false;
      sendStm32Byte('x');
      digitalWrite(LED_BUILTIN_C6, HIGH);
    }
    xSemaphoreGive(stateMutex);
  }

  delay(2);
}
