const state = {
  imageMode: "overlay",
  previewMode: "overlay",
  selectedSession: "",
  selectedSample: null,
  status: null,
  sessions: [],
};

const els = {
  liveImage: document.getElementById("live-image"),
  qualityMetrics: document.getElementById("quality-metrics"),
  cameraChip: document.getElementById("camera-chip"),
  nodeChip: document.getElementById("node-chip"),
  motionChip: document.getElementById("motion-chip"),
  sessionChip: document.getElementById("session-chip"),
  configSummary: document.getElementById("config-summary"),
  lastStatus: document.getElementById("last-status"),
  currentWaypoint: document.getElementById("current-waypoint"),
  tcpPose: document.getElementById("tcp-pose"),
  cameraBoard: document.getElementById("camera-board"),
  commandResult: document.getElementById("command-result"),
  sessionSelect: document.getElementById("session-select"),
  reportPath: document.getElementById("report-path"),
  sampleCount: document.getElementById("sample-count"),
  skippedCount: document.getElementById("skipped-count"),
  transMean: document.getElementById("trans-mean"),
  rotMean: document.getElementById("rot-mean"),
  residualBars: document.getElementById("residual-bars"),
  trajectoryPath: document.getElementById("trajectory-path"),
  waypointBody: document.getElementById("waypoint-body"),
  previewTitle: document.getElementById("preview-title"),
  samplePreview: document.getElementById("sample-preview"),
  eventLog: document.getElementById("event-log"),
};

function fmt(value, digits = 2, suffix = "") {
  if (value === null || value === undefined || Number.isNaN(Number(value))) {
    return "--";
  }
  return `${Number(value).toFixed(digits)}${suffix}`;
}

function basename(path) {
  if (!path) return "";
  const parts = String(path).split(/[\\/]/);
  return parts[parts.length - 1] || "";
}

function setChip(el, text, tone = "neutral") {
  el.textContent = text;
  el.className = `chip ${tone}`;
}

function setNotice(message, tone = "") {
  els.commandResult.textContent = message || "等待操作";
  els.commandResult.className = `notice ${tone}`.trim();
}

async function getJson(url, fallback) {
  try {
    const response = await fetch(url, { cache: "no-store" });
    if (!response.ok) throw new Error(`${response.status} ${response.statusText}`);
    return await response.json();
  } catch (error) {
    addEvent("ui_error", `${url}: ${error.message}`);
    return fallback;
  }
}

async function postJson(url, body = {}) {
  const response = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  if (!response.ok) throw new Error(`${response.status} ${response.statusText}`);
  return response.json();
}

function refreshImage() {
  els.liveImage.src = `/api/image/latest.jpg?mode=${state.imageMode}&t=${Date.now()}`;
}

async function refreshStatus() {
  const status = await getJson("/api/status", null);
  if (!status) return;
  state.status = status;
  const camera = status.camera || {};
  const handeye = status.handeye || {};
  const config = status.config || {};
  const last = handeye.last_status || {};
  setChip(els.cameraChip, camera.connected ? `Camera ${camera.image_sequence || 0}` : "Camera offline", camera.connected ? "good" : "bad");
  setChip(els.nodeChip, handeye.calibration_node_connected ? "Node online" : "Node waiting", handeye.calibration_node_connected ? "good" : "warn");
  setChip(els.motionChip, handeye.execute_motion ? "Motion enabled" : "Dry-run", handeye.execute_motion ? "bad" : "good");
  const sessionId = basename(last.session_dir);
  setChip(els.sessionChip, sessionId ? `Session ${sessionId}` : "Session --", sessionId ? "good" : "neutral");
  els.configSummary.textContent = `${config.board_rows || "--"}x${config.board_cols || "--"}  square=${fmt(config.square_size_m, 3, "m")}  max_px=${fmt(handeye.max_reprojection_error_px, 1)}`;
  els.lastStatus.textContent = last.status ? `${last.status}: ${last.message || ""}` : "--";
  els.currentWaypoint.textContent = last.waypoint_name || last.waypoint?.name || "--";
  els.tcpPose.textContent = (last.stable_tcp_pose_mmdeg || last.tcp_pose_mmdeg || []).map((v) => fmt(v, 2)).join(", ") || "--";
  if (last.session_dir && !state.selectedSession) {
    state.selectedSession = basename(last.session_dir);
  }
  const result = handeye.last_command_result;
  if (result) {
    setNotice(result.message, result.success ? "good" : "bad");
  }
}

async function refreshQuality() {
  const quality = await getJson("/api/handeye/quality", {});
  const t = quality.camera_to_board_translation_m || [];
  const cells = [
    ["检测", quality.detected ? "ok" : "no"],
    ["重投影 px", fmt(quality.reprojection_error_px, 3)],
    ["边距 px", fmt(quality.board_margin_px, 1)],
    ["Z m", fmt(t[2], 3)],
    ["棋盘角度", fmt(quality.board_angle_deg, 2, " deg")],
  ];
  els.qualityMetrics.innerHTML = cells
    .map((item) => `<div class="metric"><span>${item[0]}</span><strong>${item[1]}</strong></div>`)
    .join("");
  els.cameraBoard.textContent = t.length ? `x=${fmt(t[0], 3)} y=${fmt(t[1], 3)} z=${fmt(t[2], 3)} m` : "--";
}

async function refreshSessions() {
  const sessions = await getJson("/api/sessions", []);
  state.sessions = sessions;
  if (!state.selectedSession && sessions.length) {
    state.selectedSession = sessions[0].id;
  }
  const options = sessions.map((session) => `<option value="${session.id}">${session.id} (${session.sample_count || 0})</option>`);
  if (!options.length) options.push('<option value="">无 session</option>');
  els.sessionSelect.innerHTML = options.join("");
  els.sessionSelect.value = state.selectedSession || "";
}

async function refreshReport() {
  if (!state.selectedSession) {
    els.reportPath.textContent = "尚未选择 session";
    els.sampleCount.textContent = "--";
    els.skippedCount.textContent = "--";
    els.transMean.textContent = "--";
    els.rotMean.textContent = "--";
    els.residualBars.innerHTML = "";
    return;
  }
  const report = await getJson(`/api/sessions/${encodeURIComponent(state.selectedSession)}/report`, {});
  const residuals = report.residuals || {};
  els.reportPath.textContent = report.report_path || `calibration_sessions/${state.selectedSession}/report.yaml`;
  els.sampleCount.textContent = report.sample_count ?? "--";
  els.transMean.textContent = fmt(residuals.translation_mean_mm, 2, " mm");
  els.rotMean.textContent = fmt(residuals.rotation_mean_deg, 2, " deg");
  els.residualBars.innerHTML = [
    ["translation_rms", residuals.translation_rms_mm, 80, " mm"],
    ["translation_max", residuals.translation_max_mm, 160, " mm"],
    ["rotation_rms", residuals.rotation_rms_deg, 20, " deg"],
    ["rotation_max", residuals.rotation_max_deg, 45, " deg"],
  ]
    .map(([name, value, maxValue, suffix]) => {
      const pct = Math.max(0, Math.min(100, Number(value || 0) / maxValue * 100));
      return `<div class="bar-row"><span>${name}</span><div class="bar-track"><div class="bar-fill" style="width:${pct}%"></div></div><strong>${fmt(value, 2, suffix)}</strong></div>`;
    })
    .join("");
}

async function refreshWaypoints() {
  let payload;
  if (state.selectedSession) {
    payload = await getJson(`/api/sessions/${encodeURIComponent(state.selectedSession)}/waypoints`, []);
  } else {
    const trajectory = await getJson("/api/handeye/waypoints", { waypoints: [] });
    payload = trajectory.waypoints.map((waypoint) => ({ waypoint, name: waypoint.name, status: "pending", capture: waypoint.capture }));
    els.trajectoryPath.textContent = trajectory.trajectory_path || "trajectory YAML";
  }
  const waypoints = Array.isArray(payload) ? payload : [];
  const skipped = waypoints.filter((item) => item.status === "skipped").length;
  els.skippedCount.textContent = skipped || "0";
  els.waypointBody.innerHTML = waypoints
    .map((item, index) => {
      const waypoint = item.waypoint || item;
      const quality = waypoint.record_quality || {};
      const status = item.status || "pending";
      const sampleIndex = item.sample_index;
      const tcb = item.camera_to_board_translation_m || quality.camera_to_board_translation_m || [];
      const thumb = sampleIndex
        ? `<button class="thumb-button" type="button" data-sample="${sampleIndex}"><img src="/api/sessions/${encodeURIComponent(state.selectedSession)}/sample-image/${sampleIndex}.jpg?mode=overlay&thumb=${Date.now()}" alt="sample ${sampleIndex}" /></button>`
        : "";
      return `
        <tr data-sample="${sampleIndex || ""}">
          <td>${index + 1}</td>
          <td title="${waypoint.name || item.name || ""}">${waypoint.name || item.name || "--"}</td>
          <td><span class="status-pill ${status}">${status}</span></td>
          <td>${fmt(item.reprojection_error_px ?? quality.reprojection_error_px, 3)}</td>
          <td>${fmt(item.board_margin_px ?? quality.board_margin_px, 1)}</td>
          <td>${fmt(tcb[2], 3)}</td>
          <td>${waypoint.capture === false ? "no" : "yes"}</td>
          <td title="${item.reason || ""}">${item.reason || ""}</td>
          <td>${thumb}</td>
        </tr>`;
    })
    .join("");
  const trajectory = await getJson("/api/handeye/waypoints", {});
  if (trajectory.trajectory_path) els.trajectoryPath.textContent = trajectory.trajectory_path;
}

async function refreshAll() {
  await refreshStatus();
  await refreshSessions();
  await refreshReport();
  await refreshWaypoints();
}

function openSamplePreview(sampleIndex) {
  if (!state.selectedSession || !sampleIndex) return;
  state.selectedSample = sampleIndex;
  els.previewTitle.textContent = `${state.selectedSession} / sample_${String(sampleIndex).padStart(3, "0")}`;
  els.samplePreview.src = `/api/sessions/${encodeURIComponent(state.selectedSession)}/sample-image/${sampleIndex}.jpg?mode=${state.previewMode}&t=${Date.now()}`;
}

async function runCommand(label, url, body = {}) {
  try {
    setNotice(`${label} 请求已发送`);
    const result = await postJson(url, body);
    if (result.requires_confirmation) {
      setNotice(result.message, "bad");
      return result;
    }
    setNotice(result.message, result.success ? "good" : "bad");
    await refreshAll();
    return result;
  } catch (error) {
    setNotice(`${label} 失败: ${error.message}`, "bad");
    return { success: false, message: error.message };
  }
}

function addEvent(type, message) {
  const item = document.createElement("div");
  item.className = "event-item";
  item.innerHTML = `<strong>${new Date().toLocaleTimeString()}</strong> ${type}: ${message}`;
  els.eventLog.prepend(item);
  while (els.eventLog.children.length > 80) {
    els.eventLog.removeChild(els.eventLog.lastChild);
  }
}

function connectEvents() {
  const proto = window.location.protocol === "https:" ? "wss:" : "ws:";
  const socket = new WebSocket(`${proto}//${window.location.host}/ws/events`);
  socket.addEventListener("message", (event) => {
    const payload = JSON.parse(event.data);
    const status = payload.payload || payload.result || {};
    addEvent(payload.type || "event", status.message || status.status || payload.message || "");
    if (payload.type === "eye_to_hand_status") {
      refreshStatus();
      refreshWaypoints();
      refreshReport();
    }
  });
  socket.addEventListener("close", () => {
    addEvent("ws", "连接关闭，3 秒后重连");
    window.setTimeout(connectEvents, 3000);
  });
}

function bindUi() {
  document.querySelectorAll("[data-image-mode]").forEach((button) => {
    button.addEventListener("click", () => {
      state.imageMode = button.dataset.imageMode;
      document.querySelectorAll("[data-image-mode]").forEach((item) => item.classList.toggle("active", item === button));
      refreshImage();
    });
  });
  document.querySelectorAll("[data-preview-mode]").forEach((button) => {
    button.addEventListener("click", () => {
      state.previewMode = button.dataset.previewMode;
      document.querySelectorAll("[data-preview-mode]").forEach((item) => item.classList.toggle("active", item === button));
      openSamplePreview(state.selectedSample);
    });
  });
  document.getElementById("record-btn").addEventListener("click", () => runCommand("记录当前点", "/api/handeye/record_waypoint"));
  document.getElementById("delete-btn").addEventListener("click", () => runCommand("删除上一个", "/api/handeye/delete_last_waypoint"));
  document.getElementById("dry-run-btn").addEventListener("click", async () => {
    if (state.status?.handeye?.execute_motion) {
      setNotice("当前 launch 为 execute_motion=true；dry-run 请重新以 execute_motion:=false 启动。", "bad");
      return;
    }
    await runCommand("Dry-run", "/api/handeye/run", { confirmed: true });
  });
  document.getElementById("run-btn").addEventListener("click", async () => {
    const motionEnabled = Boolean(state.status?.handeye?.execute_motion);
    if (motionEnabled) {
      const ok = window.confirm("execute_motion=true 将真实驱动机械臂。确认标定板、线缆和工作空间安全后再继续。");
      if (!ok) return;
    }
    await runCommand("开始标定", "/api/handeye/run", { confirmed: motionEnabled });
  });
  document.getElementById("stop-btn").addEventListener("click", () => runCommand("停止", "/api/handeye/stop"));
  document.getElementById("refresh-btn").addEventListener("click", refreshAll);
  els.sessionSelect.addEventListener("change", () => {
    state.selectedSession = els.sessionSelect.value;
    refreshReport();
    refreshWaypoints();
  });
  els.waypointBody.addEventListener("click", (event) => {
    const button = event.target.closest("[data-sample]");
    if (button) openSamplePreview(button.dataset.sample);
  });
}

bindUi();
connectEvents();
refreshImage();
refreshAll();
refreshQuality();
window.setInterval(refreshImage, 1000);
window.setInterval(refreshStatus, 1500);
window.setInterval(refreshQuality, 2200);
window.setInterval(refreshSessions, 5000);
