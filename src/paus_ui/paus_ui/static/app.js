const state = {
  previewMode: "raw",
  selectedSession: "",
  selectedWaypointName: "",
  selectedSampleRow: null,
  selectionVersion: 0,
  status: null,
  quality: null,
  sessions: [],
  waypoints: [],
  waypointsDirty: false,
  trajectoryLoadError: "",
  userSelectedSession: false,
  currentTrajectorySelected: false,
  autoSelectedSession: false,
  lastEventId: 0,
  wsConnected: false,
  imageWsConnected: false,
  liveImageObjectUrl: "",
  liveImageFallbackTimer: null,
  liveImageWatchdogTimer: null,
  liveImageReconnectTimer: null,
  liveImageReconnectDelay: 1500,
  liveImageFrameCount: 0,
  liveImageLastFrameAt: 0,
  thumbObserver: null,
  refreshInFlight: null,
  fullRefreshTimer: null,
  eventReconnectTimer: null,
  eventReconnectDelay: 5000,
  eventDedupe: new Map(),
};

const els = {
  liveImage: document.getElementById("live-image"),
  liveEmpty: document.getElementById("live-empty"),
  qualityMetrics: document.getElementById("quality-metrics"),
  cameraChip: document.getElementById("camera-chip"),
  nodeChip: document.getElementById("node-chip"),
  extrinsicsChip: document.getElementById("extrinsics-chip"),
  motionChip: document.getElementById("motion-chip"),
  imageChip: document.getElementById("image-chip"),
  currentWaypoint: document.getElementById("current-waypoint"),
  workflowFlow: document.getElementById("workflow-flow"),
  tcpPoseGrid: document.getElementById("tcp-pose-grid"),
  cameraBoardGrid: document.getElementById("camera-board-grid"),
  boardAngle: document.getElementById("board-angle"),
  commandResult: document.getElementById("command-result"),
  sessionSelect: document.getElementById("session-select"),
  reportPath: document.getElementById("report-path"),
  sampleCount: document.getElementById("sample-count"),
  acceptedCount: document.getElementById("accepted-count"),
  skippedCount: document.getElementById("skipped-count"),
  pendingCount: document.getElementById("pending-count"),
  residualGrid: document.getElementById("residual-grid"),
  residualBars: document.getElementById("residual-bars"),
  trajectoryPath: document.getElementById("trajectory-path"),
  waypointStats: document.getElementById("waypoint-stats"),
  waypointBody: document.getElementById("waypoint-body"),
  tableFooter: document.getElementById("table-footer"),
  previewTitle: document.getElementById("preview-title"),
  previewSubtitle: document.getElementById("preview-subtitle"),
  samplePreview: document.getElementById("sample-preview"),
  previewEmpty: document.getElementById("preview-empty"),
  previewDetails: document.getElementById("preview-details"),
  eventLog: document.getElementById("event-log"),
  eventMode: document.getElementById("event-mode"),
  deleteBtn: document.getElementById("delete-btn"),
  loadSessionTrajectoryBtn: document.getElementById("load-session-trajectory-btn"),
};

function fmt(value, digits = 2, suffix = "") {
  if (value === null || value === undefined || value === "" || Number.isNaN(Number(value))) {
    return "--";
  }
  return `${Number(value).toFixed(digits)}${suffix}`;
}

function text(value, fallback = "--") {
  return value === null || value === undefined || value === "" ? fallback : String(value);
}

function escapeHtml(value) {
  return text(value, "").replace(/[&<>"']/g, (char) => ({
    "&": "&amp;",
    "<": "&lt;",
    ">": "&gt;",
    '"': "&quot;",
    "'": "&#39;",
  })[char]);
}

function basename(path) {
  if (!path) return "";
  const parts = String(path).split(/[\\/]/);
  return parts[parts.length - 1] || "";
}

function setChip(el, label, tone = "neutral") {
  el.textContent = label;
  el.className = `chip ${tone}`;
}

function setNotice(message, tone = "") {
  els.commandResult.textContent = message || "等待操作";
  els.commandResult.className = `notice ${tone}`.trim();
}

function commandTone(result) {
  if (result?.success) return "good";
  if (result?.accepted || result?.queued) return "";
  return "bad";
}

async function getJson(url, fallback) {
  try {
    const response = await fetch(url, { cache: "no-store" });
    if (!response.ok) throw new Error(`${response.status} ${response.statusText}`);
    return await response.json();
  } catch (error) {
    addEvent({ type: "ui_error", operator_message: `${url}: ${error.message}`, tone: "bad" });
    return fallback;
  }
}

async function postJson(url, body = {}) {
  const response = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body || {}),
  });
  if (!response.ok) throw new Error(`${response.status} ${response.statusText}`);
  return response.json();
}

function setLiveImageStatus(label, tone = "neutral") {
  if (els.imageChip) {
    setChip(els.imageChip, label, tone);
  }
}

function clearLiveImageObjectUrl() {
  if (state.liveImageObjectUrl) {
    URL.revokeObjectURL(state.liveImageObjectUrl);
    state.liveImageObjectUrl = "";
  }
}

function showLivePlaceholder(message) {
  els.liveEmpty.textContent = message || "等待相机图像";
  els.liveEmpty.classList.remove("hidden");
}

function hideLivePlaceholder() {
  els.liveEmpty.classList.add("hidden");
}

function markLiveImageFrameReceived() {
  state.liveImageLastFrameAt = Date.now();
  state.liveImageFrameCount += 1;
  hideLivePlaceholder();
}

function refreshLiveImageHttp(force = false) {
  if (state.imageWsConnected && !force) return;
  els.liveImage.src = `/api/image/latest.jpg?mode=raw&axes=false&t=${Date.now()}`;
}

function startLiveImageFallback() {
  if (state.liveImageFallbackTimer) return;
  refreshLiveImageHttp();
  state.liveImageFallbackTimer = window.setInterval(refreshLiveImageHttp, 1800);
  setLiveImageStatus("图像 HTTP fallback", "warn");
}

function stopLiveImageFallback() {
  if (state.liveImageFallbackTimer) {
    window.clearInterval(state.liveImageFallbackTimer);
    state.liveImageFallbackTimer = null;
  }
}

function startLiveImageWatchdog() {
  if (state.liveImageWatchdogTimer) return;
  state.liveImageLastFrameAt = Date.now();
  state.liveImageWatchdogTimer = window.setInterval(() => {
    if (!state.imageWsConnected) return;
    if (!state.liveImageLastFrameAt) return;
    const staleForMs = Date.now() - state.liveImageLastFrameAt;
    if (staleForMs >= 2500) {
      setLiveImageStatus("图像 WS stale", "warn");
    }
  }, 1000);
}

function stopLiveImageWatchdog() {
  if (state.liveImageWatchdogTimer) {
    window.clearInterval(state.liveImageWatchdogTimer);
    state.liveImageWatchdogTimer = null;
  }
}

function scheduleLiveImageReconnect() {
  if (state.liveImageReconnectTimer) return;
  const delay = state.liveImageReconnectDelay;
  state.liveImageReconnectTimer = window.setTimeout(() => {
    state.liveImageReconnectTimer = null;
    connectLiveImageStream();
  }, delay);
  state.liveImageReconnectDelay = Math.min(delay * 2, 30000);
}

function scheduleEventReconnect() {
  if (state.eventReconnectTimer) return;
  const delay = state.eventReconnectDelay;
  state.eventReconnectTimer = window.setTimeout(() => {
    state.eventReconnectTimer = null;
    connectEvents();
  }, delay);
  state.eventReconnectDelay = Math.min(delay * 2, 60000);
}

function connectLiveImageStream() {
  const proto = window.location.protocol === "https:" ? "wss:" : "ws:";
  const wsUrl = `${proto}//${window.location.host}/ws/image`;
  let socket;
  try {
    socket = new WebSocket(wsUrl);
  } catch (error) {
    state.imageWsConnected = false;
    setLiveImageStatus("图像 HTTP fallback", "warn");
    startLiveImageFallback();
    scheduleLiveImageReconnect();
    return;
  }
  socket.binaryType = "arraybuffer";
  socket.addEventListener("open", () => {
    state.imageWsConnected = true;
    state.liveImageReconnectDelay = 1500;
    stopLiveImageFallback();
    startLiveImageWatchdog();
    setLiveImageStatus("图像 WebSocket", "good");
  });
  socket.addEventListener("message", (event) => {
    const payload = event.data;
    if (!(payload instanceof ArrayBuffer)) return;
    const blob = new Blob([payload], { type: "image/jpeg" });
    const nextUrl = URL.createObjectURL(blob);
    const previousUrl = state.liveImageObjectUrl;
    state.liveImageObjectUrl = nextUrl;
    els.liveImage.src = nextUrl;
    markLiveImageFrameReceived();
    if (previousUrl) {
      window.setTimeout(() => URL.revokeObjectURL(previousUrl), 0);
    }
  });
  socket.addEventListener("error", () => {
    // error is followed by close; handled there.
  });
  socket.addEventListener("close", () => {
    state.imageWsConnected = false;
    stopLiveImageFallback();
    stopLiveImageWatchdog();
    startLiveImageFallback();
    scheduleLiveImageReconnect();
  });
}

async function refreshStatus() {
  const status = await getJson("/api/status", null);
  if (!status) return;
  state.status = status;
  const camera = status.camera || {};
  const handeye = status.handeye || {};
  const workflow = handeye.workflow || {};
  const current = handeye.current_waypoint || {};
  const session = handeye.session || {};
  const motion = handeye.motion || {};
  if (handeye.trajectory_dirty !== null && handeye.trajectory_dirty !== undefined) {
    state.waypointsDirty = Boolean(handeye.trajectory_dirty);
  }
  setChip(els.cameraChip, camera.connected ? `相机已连接 ${camera.image_sequence || 0}` : "相机等待中", camera.connected ? "good" : "warn");
  setChip(els.nodeChip, handeye.calibration_node_connected ? "机器人已连接" : "节点等待中", handeye.calibration_node_connected ? "good" : "warn");
  setChip(els.extrinsicsChip, session.session_id ? `session ${session.session_id}` : "外参 --", session.session_id ? "good" : "neutral");
  const motionLabel = handeye.execute_motion ? "真机模式 (execute_motion=true)" : "Dry-run (execute_motion=false)";
  const motionTone = handeye.execute_motion ? "bad" : "good";
  setChip(els.motionChip, motionLabel, motionTone);
  const runBtn = document.getElementById("run-btn");
  const dryRunBtn = document.getElementById("dry-run-btn");
  if (handeye.execute_motion) {
    runBtn.textContent = "▶ 开始标定（真机）";
    runBtn.title = "机械臂将真实运动！确认工作空间安全后操作。";
    dryRunBtn.style.display = "none";
  } else {
    runBtn.textContent = "▶ 开始标定";
    runBtn.title = "当前为 dry-run 模式，机械臂不会真实运动。";
    dryRunBtn.style.display = "";
  }
  if (!handeye.motion_state_known && handeye.calibration_node_connected) {
    setChip(els.motionChip, "运动状态未知 — 请确认", "warn");
  }
  els.currentWaypoint.textContent = current.name || "--";
  renderFlow(workflow.stage);
  const tcp = current.stable_tcp_pose_mmdeg || current.tcp_pose_mmdeg || [];
  renderPoseGrid(els.tcpPoseGrid, ["x", "y", "z", "rx", "ry", "rz"], tcp, ["mm", "mm", "mm", "deg", "deg", "deg"]);
  const boardTranslation = current.camera_to_board_translation_m || [];
  const boardRotation = current.camera_to_board_rotation_rpy_deg || [];
  if (boardTranslation.length || boardRotation.length) {
    renderPoseGrid(els.cameraBoardGrid, ["x", "y", "z", "rx", "ry", "rz"], [
      boardTranslation[0],
      boardTranslation[1],
      boardTranslation[2],
      ...boardRotation,
    ], ["m", "m", "m", "deg", "deg", "deg"]);
  } else {
    renderPoseGrid(els.cameraBoardGrid, ["x", "y", "z", "rx", "ry", "rz"], [], ["m", "m", "m", "deg", "deg", "deg"]);
  }
  if (current.board_angle_deg !== null && current.board_angle_deg !== undefined) {
    els.boardAngle.textContent = fmt(current.board_angle_deg, 2, " deg");
  } else if (current.empty_reason) {
    els.boardAngle.textContent = current.empty_reason;
  } else {
    els.boardAngle.textContent = "--";
  }
  renderQualityMetrics(current);
  if (session.session_id && handeye.run_active && !state.userSelectedSession && state.selectedSession !== session.session_id) {
    state.selectedSession = session.session_id;
    state.autoSelectedSession = true;
    state.currentTrajectorySelected = false;
  } else if (!handeye.run_active && state.autoSelectedSession) {
    state.autoSelectedSession = false;
  }
  const result = handeye.last_command_result;
  if (result) {
    setNotice(result.operator_message || result.message, commandTone(result));
  }
  if (motion.trajectory_path && (!state.selectedSession || state.selectedSession === session.session_id || state.currentTrajectorySelected)) {
    els.trajectoryPath.textContent = motion.trajectory_path;
  }
  const currentStage = workflow.stage || "";
  if ((currentStage === "finished" || currentStage === "error") && !completionShown) {
    const sessionId = session.session_id || state.selectedSession;
    refreshReport().then(() => {
      if (!sessionId) {
        showCompletionDialog(handeye, null);
        return;
      }
      getJson(`/api/sessions/${encodeURIComponent(sessionId)}/report`, {}).then((report) => {
        showCompletionDialog(handeye, report);
      });
    });
  }
  if (currentStage === "movej" || currentStage === "dry_run") {
    completionShown = false;
  }
}

function renderQualityMetrics(quality = {}) {
  const t = quality.camera_to_board_translation_m || [];
  const thresholds = quality.thresholds || {};
  const detected = quality.quality_detected ?? quality.detected;
  const metrics = [
    { label: "检测", value: detected === true ? "ok" : (detected === false ? "no" : "--"), tone: detected === false ? "warn" : "" },
    { label: "重投影 px", value: fmt(quality.reprojection_error_px, 3), tone: thresholds.reprojection_ok === false ? "bad" : "" },
    { label: "边距 px", value: fmt(quality.board_margin_px, 1), tone: thresholds.margin_ok === false ? "bad" : "" },
    { label: "Z m", value: fmt(t[2], 3), tone: "" },
    { label: "棋盘角度", value: fmt(quality.board_angle_deg, 2, " deg"), tone: "" },
  ];
  els.qualityMetrics.innerHTML = metrics.map((item) => `<div class="metric ${item.tone}"><span>${item.label}</span><strong>${item.value}</strong></div>`).join("");
}

async function refreshSessions() {
  const sessions = await getJson("/api/sessions", []);
  state.sessions = sessions;
  const selectedStillExists = sessions.some((session) => session.id === state.selectedSession);
  if (state.selectedSession && !selectedStillExists) {
    state.selectedSession = "";
    state.autoSelectedSession = false;
    state.userSelectedSession = false;
    state.currentTrajectorySelected = true;
    state.selectionVersion += 1;
  }
  const handeye = (state.status && state.status.handeye) || {};
  const latestReportSession = sessions.find((session) => session.has_solution || (session.has_report && !session.is_invalid));
  if (!handeye.run_active && !state.userSelectedSession && !state.currentTrajectorySelected && !state.selectedSession && latestReportSession) {
    state.selectedSession = latestReportSession.id;
    state.autoSelectedSession = true;
    state.selectedWaypointName = "";
    state.selectedSampleRow = null;
    state.selectionVersion += 1;
  }
  const options = ['<option value="">\u5f53\u524d\u5c06\u8fd0\u884c\u7684\u8f68\u8ff9</option>'];
  options.push(...sessions.map((session) => {
    const suffix = session.has_solution ? "" : session.has_report ? " / unsolved" : " / no report";
    const label = `\u5386\u53f2 session ${session.id} (${session.sample_count || 0})${suffix}`;
    return `<option value="${escapeHtml(session.id)}">${escapeHtml(label)}</option>`;
  }));
  els.sessionSelect.innerHTML = options.join("");
  els.sessionSelect.value = state.selectedSession || "";
  updateLoadSessionTrajectoryButton();
}
async function refreshReport() {
  const sessionId = state.selectedSession || "";
  const selectionVersion = state.selectionVersion;
  if (!sessionId) {
    renderEmptyReport("\u6682\u65e0 session");
    return;
  }
  const report = await getJson(`/api/sessions/${encodeURIComponent(sessionId)}/report`, {});
  if (state.selectionVersion !== selectionVersion || state.selectedSession !== sessionId) return;
  if (!report.has_report) {
    renderEmptyReport(report.empty_reason || "\u8be5 session \u6682\u65e0 report.yaml\u3002");
    return;
  }
  els.reportPath.textContent = report.report_path || `calibration_sessions/${sessionId}/report.yaml`;
  els.sampleCount.textContent = report.sample_count ?? "--";
  els.acceptedCount.textContent = report.accepted_count ?? "--";
  els.skippedCount.textContent = report.skipped_count ?? "--";
  els.pendingCount.textContent = report.pending_count ?? "--";
  if (!report.has_solution) {
    const message = report.is_invalid
      ? (report.empty_reason || report.report_error || "\u8be5 session \u65e0\u6548\u3002")
      : "report.yaml exists, but this session has not produced a solved calibration result yet.";
    els.residualGrid.innerHTML = `<div class="empty-card">${escapeHtml(message)}</div>`;
    els.residualBars.innerHTML = "";
    return;
  }
  const residuals = report.residuals || {};
  els.residualGrid.innerHTML = [
    ["translation_rms_mm", "trans_rms", "mm"],
    ["translation_mean_mm", "trans_mean", "mm"],
    ["translation_max_mm", "trans_max", "mm"],
    ["rotation_rms_deg", "rot_rms", "deg"],
    ["rotation_mean_deg", "rot_mean", "deg"],
    ["rotation_max_deg", "rot_max", "deg"],
  ].map(([key, label, unit]) => `<div class="kv"><span>${label}</span><strong>${fmt(residuals[key], 2, ` ${unit}`)}</strong></div>`).join("");
  els.residualBars.innerHTML = (report.residual_comparison || []).map((row) => {
    const pct = Math.max(0, Math.min(100, Number(row.ratio || 0) * 50));
    const tone = row.ok === false ? "bad" : row.ok === true ? "" : "warn";
    return `<div class="bar-row" title="${escapeHtml(row.target_note || "")}"><span>${escapeHtml(row.label)}</span><div class="bar-track"><div class="bar-fill ${tone}" style="width:${pct}%"></div></div><strong>${fmt(row.value, 2, ` ${row.unit}`)}</strong></div>`;
  }).join("");
}
function renderEmptyReport(message) {
  els.reportPath.textContent = message;
  els.sampleCount.textContent = "--";
  els.acceptedCount.textContent = "--";
  els.skippedCount.textContent = "--";
  els.pendingCount.textContent = "--";
  els.residualGrid.innerHTML = "";
  els.residualBars.innerHTML = "";
}

function renderWaypointLoadError(message) {
  const error = message || "当前示教轨迹加载失败。";
  els.waypointStats.textContent = "轨迹加载失败";
  els.tableFooter.textContent = error;
  els.waypointBody.innerHTML = `<tr class="empty-row"><td colspan="12">${escapeHtml(error)}</td></tr>`;
  state.selectedWaypointName = "";
  state.selectedSampleRow = null;
  updateDeleteButtonState();
  renderPreview(null);
  els.previewEmpty.textContent = error;
}

async function refreshWaypoints() {
  const sessionId = state.selectedSession || "";
  const selectionVersion = state.selectionVersion;
  let waypoints = [];
  let trajectoryLoadError = "";
  if (sessionId) {
    waypoints = await getJson(`/api/sessions/${encodeURIComponent(sessionId)}/waypoints`, []);
    if (state.selectionVersion !== selectionVersion || state.selectedSession !== sessionId) return;
    state.waypointsDirty = false;
    state.trajectoryLoadError = "";
    const sessionRoot = text(state.status?.handeye?.session_root_path, "calibration_sessions").replace(/[\\/]+$/, "");
    els.trajectoryPath.textContent = `${sessionRoot}/${sessionId}/trajectory_used.yaml`;
  } else {
    const trajectory = await getJson("/api/handeye/waypoints", { waypoints: [] });
    if (state.selectionVersion !== selectionVersion || state.selectedSession !== sessionId) return;
    state.waypointsDirty = Boolean(trajectory.dirty);
    if (trajectory.error) {
      trajectoryLoadError = trajectory.error;
      if (trajectory.trajectory_path) els.trajectoryPath.textContent = trajectory.trajectory_path;
      waypoints = [];
    } else {
      waypoints = (trajectory.waypoints || []).map((waypoint, index) => ({
        waypoint,
        index: index + 1,
        name: waypoint.name,
        status: waypoint.status || "pending",
        result: waypoint.result || "-",
        capture: waypoint.capture,
        thresholds: waypoint.thresholds || {},
        reprojection_error_px: waypoint.reprojection_error_px,
        board_margin_px: waypoint.board_margin_px,
        camera_to_board_translation_m: waypoint.camera_to_board_translation_m,
        camera_to_board_rotation_rpy_deg: waypoint.camera_to_board_rotation_rpy_deg,
        board_angle_deg: waypoint.board_angle_deg,
        reason: waypoint.reason,
        reason_display: waypoint.reason_display,
        has_image: waypoint.has_image || false,
        sample_row_index: waypoint.sample_row_index || null,
        thumbnail_url: waypoint.thumbnail_url || null,
      }));
      if (trajectory.trajectory_path) els.trajectoryPath.textContent = trajectory.trajectory_path;
    }
    state.trajectoryLoadError = trajectoryLoadError;
  }
  state.waypoints = Array.isArray(waypoints) ? waypoints : [];
  if (state.trajectoryLoadError && !sessionId) {
    renderWaypointLoadError(state.trajectoryLoadError);
    return;
  }
  const accepted = state.waypoints.filter((item) => item.status === "accepted").length;
  const skipped = state.waypoints.filter((item) => item.status === "skipped").length;
  const pending = state.waypoints.filter((item) => item.status === "pending").length;
  els.waypointStats.textContent = `\u5171 ${state.waypoints.length} \u4e2a\u70b9`;
  els.tableFooter.textContent = `\u5df2\u63a5\u53d7 ${accepted}\u3000\u8df3\u8fc7 ${skipped}\u3000\u5f85\u91c7\u96c6 ${pending}`;
  els.waypointBody.innerHTML = state.waypoints.map(renderWaypointRow).join("");
  hydrateWaypointThumbnails();
  if (!state.waypoints.length) {
    state.selectedWaypointName = "";
    state.selectedSampleRow = null;
    renderPreview(null);
  } else if (!state.waypoints.some((item) => item.name === state.selectedWaypointName)) {
    selectWaypoint(state.waypoints.find((item) => item.has_image) || state.waypoints[0]);
  } else {
    markSelectedRow();
    updateDeleteButtonState();
  }
}
function renderWaypointRow(item, index) {
  const waypoint = item.waypoint || item;
  const status = item.status || "pending";
  const thresholds = item.thresholds || {};
  const t = item.camera_to_board_translation_m || [];
  const displayIndex = item.index ?? item.display_index ?? index + 1;
  const reprojClass = thresholds.reprojection_ok === false ? "quality-bad" : "";
  const marginClass = thresholds.margin_ok === false ? "quality-bad" : "";
  const transResid = item.translation_residual_mm;
  const rotResid = item.rotation_residual_deg;
  const transResidClass = transResid !== null && transResid !== undefined && transResid > 10 ? "quality-bad" : (transResid !== null && transResid !== undefined && transResid > 5 ? "quality-warn" : "");
  const rotResidClass = rotResid !== null && rotResid !== undefined && rotResid > 2 ? "quality-bad" : (rotResid !== null && rotResid !== undefined && rotResid > 1 ? "quality-warn" : "");
  const thumb = item.has_image && item.sample_row_index
    ? `<button class="thumb-button" type="button" data-row="${item.sample_row_index}"><img class="thumb-image" data-src="/api/sessions/${encodeURIComponent(state.selectedSession)}/sample-image/${item.sample_row_index}.jpg?mode=raw&axes=false" alt="sample ${item.sample_row_index}" loading="lazy" decoding="async" /></button>`
    : "-";
  return `
    <tr data-name="${escapeHtml(item.name || waypoint.name || "")}" data-row="${item.sample_row_index || ""}">
      <td>${escapeHtml(displayIndex)}</td>
      <td title="${escapeHtml(item.name || waypoint.name || "")}">${escapeHtml(item.name || waypoint.name || "--")}</td>
      <td><span class="status-pill ${escapeHtml(status)}">${escapeHtml(status)}</span></td>
      <td><span class="result-pill ${escapeHtml(String(item.result || "-").toLowerCase())}">${escapeHtml(item.result || "-")}</span></td>
      <td class="${reprojClass}">${fmt(item.reprojection_error_px, 3)}</td>
      <td class="${marginClass}">${fmt(item.board_margin_px, 1)}</td>
      <td>${fmt(t[2], 3)}</td>
      <td class="${transResidClass}">${fmt(transResid, 3)}</td>
      <td class="${rotResidClass}">${fmt(rotResid, 4)}</td>
      <td>${item.capture === false ? "no" : "OK"}</td>
      <td title="${escapeHtml(item.reason_display || item.reason || "")}">${escapeHtml(item.reason_display || item.reason || "-")}</td>
      <td>${thumb}</td>
    </tr>`;
}

function clearThumbnailObserver() {
  if (state.thumbObserver) {
    state.thumbObserver.disconnect();
    state.thumbObserver = null;
  }
}

function hydrateWaypointThumbnails() {
  clearThumbnailObserver();
  const images = [...els.waypointBody.querySelectorAll("img.thumb-image[data-src]")];
  if (!images.length) return;
  const loadThumb = (img) => {
    const src = img.dataset.src;
    if (!src || img.dataset.loaded === "true") return;
    img.src = src;
    img.dataset.loaded = "true";
    delete img.dataset.src;
  };
  if ("IntersectionObserver" in window) {
    const tableWrap = els.waypointBody.closest(".table-wrap");
    state.thumbObserver = new IntersectionObserver((entries) => {
      entries.forEach((entry) => {
        if (!entry.isIntersecting) return;
        loadThumb(entry.target);
        state.thumbObserver?.unobserve(entry.target);
      });
    }, {
      root: tableWrap,
      rootMargin: "180px 0px",
      threshold: 0.01,
    });
    images.forEach((img) => state.thumbObserver.observe(img));
    return;
  }
  images.slice(0, 8).forEach(loadThumb);
}

function selectWaypoint(item) {
  if (!item) return;
  state.selectedWaypointName = item.name || item.waypoint?.name || "";
  state.selectedSampleRow = item.sample_row_index || null;
  markSelectedRow();
  updateDeleteButtonState();
  renderPreview(item);
}

function markSelectedRow() {
  [...els.waypointBody.querySelectorAll("tr")].forEach((row) => {
    row.classList.toggle("selected", row.dataset.name === state.selectedWaypointName);
  });
  updateDeleteButtonState();
}

function updateDeleteButtonState() {
  if (!els.deleteBtn) return;
  const selected = state.waypoints.find((item) => item.name === state.selectedWaypointName);
  const canDelete = Boolean(!state.selectedSession && selected?.name);
  els.deleteBtn.disabled = !canDelete;
  els.deleteBtn.textContent = canDelete ? `\u5220\u9664\u9009\u4e2d ${selected.name}` : "\u5220\u9664\u9009\u4e2d";
  els.deleteBtn.title = state.selectedSession ? "\u5386\u53f2 session \u4e0d\u5141\u8bb8\u5220\u9664 waypoint" : (canDelete ? `\u5220\u9664 ${selected.name}` : "\u5148\u5728\u5f53\u524d\u8f68\u8ff9\u5217\u8868\u4e2d\u9009\u62e9 waypoint");
}

function updateLoadSessionTrajectoryButton() {
  if (!els.loadSessionTrajectoryBtn) return;
  const sessionId = state.selectedSession || "";
  const selected = state.sessions.find((session) => session.id === sessionId);
  const canLoad = Boolean(sessionId && selected && !selected.trajectory_error);
  els.loadSessionTrajectoryBtn.disabled = !canLoad;
  els.loadSessionTrajectoryBtn.title = canLoad
    ? `\u5c06 session ${sessionId} \u7684 trajectory_used.yaml \u8f7d\u5165\u4e3a\u5f53\u524d\u5c06\u8fd0\u884c\u7684\u8f68\u8ff9`
    : "\u5148\u9009\u62e9\u4e00\u4e2a\u5305\u542b trajectory_used.yaml \u7684\u5386\u53f2 session";
}


function renderPreview(item) {
  if (!item) {
    els.previewTitle.textContent = "样本预览";
    els.previewSubtitle.textContent = "选择 waypoint 查看图像和质量指标";
    els.samplePreview.removeAttribute("src");
    els.samplePreview.style.display = "none";
    els.previewEmpty.classList.remove("hidden");
    els.previewEmpty.textContent = "当前没有 waypoint";
    els.previewDetails.innerHTML = "";
    return;
  }
  const name = item.name || item.waypoint?.name || "样本预览";
  els.previewTitle.textContent = `${name} 预览`;
  const rowIndex = item.sample_row_index;
  els.previewSubtitle.textContent = item.status ? `${item.status} / ${item.result || "-"}` : "选择 waypoint 查看图像和质量指标";
  const t = item.camera_to_board_translation_m || [];
  const r = item.camera_to_board_rotation_rpy_deg || [];
  if (state.previewMode === "raw") {
    els.previewDetails.innerHTML = "";
    if (state.selectedSession && rowIndex && item.has_image) {
      els.previewEmpty.classList.add("hidden");
      els.samplePreview.style.display = "block";
      els.samplePreview.src = `/api/sessions/${encodeURIComponent(state.selectedSession)}/sample-image/${rowIndex}.jpg?mode=raw&axes=false`;
    } else {
      els.samplePreview.removeAttribute("src");
      els.samplePreview.style.display = "none";
      els.previewEmpty.classList.remove("hidden");
      els.previewEmpty.textContent = item.reason_display || "该 waypoint 暂无样本图像";
    }
    return;
  }
  els.samplePreview.removeAttribute("src");
  els.samplePreview.style.display = "none";
  els.previewEmpty.classList.add("hidden");
  els.previewDetails.innerHTML = [
    ["reprojection_error_px", fmt(item.reprojection_error_px, 3)],
    ["board_margin_px", fmt(item.board_margin_px, 1)],
    ["board_angle_deg", fmt(item.board_angle_deg, 2, " deg")],
    ["T_camera_board x", fmt(t[0], 3, " m")],
    ["T_camera_board y", fmt(t[1], 3, " m")],
    ["T_camera_board z", fmt(t[2], 3, " m")],
    ["board rx", fmt(r[0], 2, " deg")],
    ["board ry", fmt(r[1], 2, " deg")],
    ["board rz", fmt(r[2], 2, " deg")],
    ["image_sequence", text(item.image_sequence)],
    ["capture_time", text(item.capture_time_s)],
    ["translation_residual_mm", fmt(item.translation_residual_mm, 3, " mm")],
    ["rotation_residual_deg", fmt(item.rotation_residual_deg, 4, " deg")],
  ].map(([label, value]) => `<div class="kv"><span>${label}</span><strong>${escapeHtml(value)}</strong></div>`).join("");
}

function initPreviewErrorHandler() {
  els.samplePreview.addEventListener("error", () => {
    els.samplePreview.removeAttribute("src");
    els.samplePreview.style.display = "none";
    els.previewEmpty.classList.remove("hidden");
    els.previewEmpty.textContent = "样本图像加载失败，文件可能不存在。";
  });
}

function renderPoseGrid(el, labels, values, units) {
  el.innerHTML = labels.map((label, index) => `<div class="kv"><span>${label} (${units[index]})</span><strong>${fmt(values[index], units[index] === "deg" ? 2 : 3)}</strong></div>`).join("");
}

function renderFlow(activeStage) {
  const baseSteps = [
    { key: "movej", label: "MoveJ" },
  ];
  const normalizedStage = normalizeWorkflowStage(activeStage);
  const steps = normalizedStage === "dry_run"
    ? [
        { key: "movej", label: "MoveJ" },
        { key: "dry_run", label: "Dry-run" },
        { key: "wait_stable", label: "等待稳定" },
        { key: "capture", label: "拍照" },
        { key: "detect", label: "detect" },
        { key: "outcome", label: "结果" },
        { key: "solve", label: "solve" },
        { key: "finished", label: "完成" },
      ]
    : [
        ...baseSteps,
        { key: "wait_stable", label: "等待稳定" },
        { key: "capture", label: "拍照" },
        { key: "detect", label: "detect" },
        { key: "outcome", label: "结果" },
        { key: "solve", label: "solve" },
        { key: "finished", label: "完成" },
      ];
  const order = {
    movej: 0,
    dry_run: 1,
    wait_stable: 2,
    capture: 3,
    detect: 4,
    accepted: 5,
    skipped: 5,
    error: 5,
    solve: 6,
    finished: 7,
  };
  const activeIndex = order[normalizedStage] ?? -1;
  els.workflowFlow.innerHTML = steps.map((step, index) => {
    const isOutcome = step.key === "outcome";
    const isOutcomeState = normalizedStage === "accepted" || normalizedStage === "skipped" || normalizedStage === "error";
    const isActive = isOutcome ? isOutcomeState : step.key === normalizedStage;
    const isDone = index < activeIndex;
    const label = isOutcome && isActive ? (normalizedStage === "error" ? "错误" : normalizedStage) : step.label;
    const branchClass = isOutcome && isActive ? normalizedStage : "";
    const cls = [isActive ? "active" : "", isDone ? "done" : "", branchClass].filter(Boolean).join(" ");
    return `<span class="flow-step ${cls}">${label}</span>`;
  }).join("");
}

function normalizeWorkflowStage(stage) {
  const aliases = {
    idle: "",
    recorded: "",
    reached: "wait_stable",
    detecting: "detect",
    detected: "detect",
    solved: "solve",
    error: "error",
  };
  return aliases[stage] !== undefined ? aliases[stage] : stage;
}

async function refreshAll() {
  if (state.refreshInFlight) return state.refreshInFlight;
  state.refreshInFlight = (async () => {
    await refreshStatus();
    await refreshSessions();
    await refreshReport();
    await refreshWaypoints();
  })();
  try {
    await state.refreshInFlight;
  } finally {
    state.refreshInFlight = null;
  }
}

function scheduleFullRefresh(reason = "refresh") {
  if (state.fullRefreshTimer) window.clearTimeout(state.fullRefreshTimer);
  state.fullRefreshTimer = window.setTimeout(() => {
    state.fullRefreshTimer = null;
    refreshAll();
  }, 80);
}

function isTerminalStatusEvent(event) {
  const payload = event?.payload || {};
  const status = payload.status || event?.status || event?.result?.status || "";
  return [
    "solved",
    "semi_auto_finished",
    "semi_auto_finished_with_skips",
    "semi_auto_stopped",
    "semi_auto_failed",
    "semi_auto_insufficient_samples",
  ].includes(String(status));
}


async function runCommand(label, url, body = {}) {
  try {
    setNotice(`${label} 请求已发送`);
    const result = await postJson(url, body);
    if (result.requires_confirmation) {
      setNotice(result.operator_message || result.message, "bad");
      return result;
    }
    setNotice(result.operator_message || result.message, commandTone(result));
    await refreshAll();
    return result;
  } catch (error) {
    setNotice(`${label} 失败: ${error.message}`, "bad");
    return { success: false, message: error.message };
  }
}

async function deleteSelectedWaypoint() {
  const selected = state.waypoints.find((item) => item.name === state.selectedWaypointName);
  if (state.selectedSession) {
    setNotice("历史 session 不允许删除 waypoint。", "bad");
    return;
  }
  if (!selected?.name) {
    setNotice("请先选择要删除的 waypoint。", "bad");
    return;
  }
  await runCommand("删除选中", "/api/handeye/delete_waypoint", { waypoint_name: selected.name });
}

async function saveTrajectoryIfDirty() {
  const dirty = Boolean(state.waypointsDirty || state.status?.handeye?.trajectory_dirty);
  if (!dirty) return true;
  setNotice("检测到未保存的示教轨迹，正在先保存轨迹...");
  const result = await runCommand("保存轨迹", "/api/handeye/save_trajectory");
  return Boolean(result?.success);
}

async function loadSelectedSessionTrajectory() {
  const sessionId = state.selectedSession || "";
  if (!sessionId) {
    setNotice("\u8bf7\u5148\u9009\u62e9\u4e00\u4e2a\u5386\u53f2 session\u3002", "bad");
    return;
  }
  const confirmed = window.confirm(
    [
      `\u5c06\u628a session ${sessionId} \u7684 waypoints \u8f7d\u5165\u4e3a\u5f53\u524d\u5c06\u8fd0\u884c\u7684\u8f68\u8ff9\u3002`,
      "\u4e0b\u4e00\u6b21\u5f00\u59cb\u6807\u5b9a\u4f1a\u6267\u884c\u8fd9\u7ec4\u70b9\u3002",
      "\u5386\u53f2 session \u672c\u8eab\u4e0d\u4f1a\u88ab\u4fee\u6539\u3002",
    ].join("\n")
  );
  if (!confirmed) return;
  const result = await runCommand("\u8f7d\u5165\u4e3a\u5f53\u524d\u8f68\u8ff9", `/api/sessions/${encodeURIComponent(sessionId)}/load_trajectory`);
  if (!result?.success) return;
  state.selectedSession = "";
  state.userSelectedSession = false;
  state.currentTrajectorySelected = true;
  state.autoSelectedSession = false;
  state.selectedWaypointName = "";
  state.selectedSampleRow = null;
  state.selectionVersion += 1;
  setNotice(result.operator_message || `\u5df2\u8f7d\u5165 session ${sessionId} \u7684 ${result.waypoint_count || ""} \u4e2a waypoints\u3002`, "good");
  await refreshAll();
}


function addEvent(event) {
  const key = event.dedupe_key || `${event.type}:${event.operator_message || event.message || ""}`;
  const now = Date.now();
  const last = state.eventDedupe.get(key) || 0;
  if (now - last < 1400) return;
  state.eventDedupe.set(key, now);
  const message = event.operator_message || event.message || event.result?.operator_message || event.payload?.message || "";
  const item = document.createElement("div");
  const tone = event.tone || (event.type && String(event.type).includes("error") ? "bad" : "");
  item.className = `event-item ${tone}`.trim();
  item.innerHTML = `<strong>${new Date((event.time_s || Date.now() / 1000) * 1000).toLocaleTimeString()}</strong> ${escapeHtml(event.type || "event")}: ${escapeHtml(message)}`;
  els.eventLog.prepend(item);
  while (els.eventLog.children.length > 70) {
    els.eventLog.removeChild(els.eventLog.lastChild);
  }
}

let completionShown = false;

function showCompletionDialog(handeye, report) {
  if (completionShown) return;
  const workflow = handeye.workflow || {};
  const stage = workflow.stage || "";
  if (stage !== "finished" && stage !== "error") return;
  completionShown = true;
  const session = handeye.session || {};
  const reportPath = report?.report_path || session.report_path || "";
  const sampleCount = report?.sample_count ?? workflow.sample_count ?? "--";
  const accepted = report?.accepted_count ?? "--";
  const skipped = report?.skipped_count ?? "--";
  const isSuccess = stage === "finished";
  const title = isSuccess ? "标定完成" : "标定异常";
  const body = [
    `<div class="kv"><span>状态</span><strong>${escapeHtml(workflow.label || stage)}</strong></div>`,
    `<div class="kv"><span>采集样本数</span><strong>${sampleCount}</strong></div>`,
    `<div class="kv"><span>已接受 / 已跳过</span><strong>${accepted} / ${skipped}</strong></div>`,
    `<div class="kv"><span>报告路径</span><strong>${escapeHtml(reportPath || "暂无")}</strong></div>`,
    `<div class="kv"><span>extrinsics 写入</span><strong>${report?.has_solution ? "已写入" : "未写入"}</strong></div>`,
    `<div class="kv"><span>session ID</span><strong>${escapeHtml(session.session_id || "--")}</strong></div>`,
  ].join("");
  document.getElementById("modal-title").textContent = title;
  document.getElementById("modal-body").innerHTML = body;
  document.getElementById("complete-modal").classList.remove("hidden");
}

function hideCompletionDialog() {
  document.getElementById("complete-modal").classList.add("hidden");
}

async function pollEvents() {
  const payload = await getJson(`/api/events?since=${state.lastEventId}`, { events: [], last_id: state.lastEventId });
  state.lastEventId = payload.last_id || state.lastEventId;
  const events = payload.events || [];
  events.forEach(addEvent);
  if (events.some((event) => event.type === "eye_to_hand_status" || event.type === "ui_command_result")) {
    if (events.some(isTerminalStatusEvent)) {
      scheduleFullRefresh("terminal-poll-event");
    } else {
      refreshStatus();
      refreshWaypoints();
      refreshReport();
    }
  }
}

function connectEvents() {
  const proto = window.location.protocol === "https:" ? "wss:" : "ws:";
  const wsUrl = `${proto}//${window.location.host}/ws/events`;
  let socket;
  try {
    socket = new WebSocket(wsUrl);
  } catch (error) {
    state.wsConnected = false;
    els.eventMode.textContent = "polling fallback";
    scheduleEventReconnect();
    return;
  }
  socket.addEventListener("open", () => {
    state.wsConnected = true;
    state.eventReconnectDelay = 5000;
    els.eventMode.textContent = "WebSocket connected";
    scheduleFullRefresh("events-ws-open");
  });
  socket.addEventListener("message", (event) => {
    try {
      const payload = JSON.parse(event.data);
      state.lastEventId = Math.max(state.lastEventId, Number(payload.id || 0));
      addEvent(payload);
      if (payload.type === "eye_to_hand_status" || payload.type === "ui_command_result") {
        if (isTerminalStatusEvent(payload)) {
          scheduleFullRefresh("terminal-ws-event");
        } else {
          refreshStatus();
          refreshWaypoints();
          refreshReport();
        }
      }
    } catch (parseError) {
      // ignore malformed messages
    }
  });
  socket.addEventListener("error", () => {
    // error will be followed by close; no need to change state here
  });
  socket.addEventListener("close", (event) => {
    state.wsConnected = false;
    els.eventMode.textContent = event.code === 1006 ? "ws closed, polling fallback" : "polling fallback";
    scheduleEventReconnect();
  });
}

function bindUi() {
  els.liveImage.addEventListener("load", hideLivePlaceholder);
  els.liveImage.addEventListener("error", () => {
    showLivePlaceholder(state.imageWsConnected ? "图像加载失败" : "等待相机图像");
  });
  document.querySelectorAll("[data-preview-mode]").forEach((button) => {
    button.addEventListener("click", () => {
      state.previewMode = button.dataset.previewMode;
      document.querySelectorAll("[data-preview-mode]").forEach((item) => item.classList.toggle("active", item === button));
      const selected = state.waypoints.find((item) => item.name === state.selectedWaypointName);
      if (selected) renderPreview(selected);
    });
  });
  document.getElementById("record-btn").addEventListener("click", () => runCommand("记录当前点", "/api/handeye/record_waypoint"));
  els.deleteBtn.addEventListener("click", deleteSelectedWaypoint);
  els.loadSessionTrajectoryBtn?.addEventListener("click", loadSelectedSessionTrajectory);
  updateDeleteButtonState();
  updateLoadSessionTrajectoryButton();
  document.getElementById("save-trajectory-btn").addEventListener("click", () => runCommand("保存轨迹", "/api/handeye/save_trajectory"));
  document.getElementById("dry-run-btn").addEventListener("click", async () => {
    const handeye = state.status?.handeye || {};
    if (handeye.execute_motion) {
      setNotice("当前为真机模式，无法执行 dry-run。请将 execute_motion 设为 false 后重试。", "bad");
      return;
    }
    if (handeye.backend_config_source !== "backend_status" && handeye.backend_config_source !== "backend_service_ready") {
      setNotice("无法确认后端是 dry-run；请确认标定节点状态已连接且 execute_motion=false。", "bad");
      return;
    }
    if (!await saveTrajectoryIfDirty()) return;
    await runCommand("Dry-run", "/api/handeye/run", { confirmed: false });
  });
  document.getElementById("run-btn").addEventListener("click", async () => {
    const handeye = state.status?.handeye || {};
    const motion = handeye.motion || {};
    if (handeye.requires_motion_confirmation) {
      const modeWarning = handeye.execute_motion
        ? "⚠ 真机模式：将真实驱动机械臂执行标定！请确认工作空间安全。"
        : "当前为 dry-run 模式，机械臂不会运动。";
      const message = [
        modeWarning,
        `waypoint 数量：${motion.waypoint_count ?? "--"}`,
        `运动模式：${motion.motion || "movej"}`,
        `速度/加速度：${motion.vel ?? "--"} / ${motion.acc ?? "--"}`,
        `轨迹文件：${motion.trajectory_path || handeye.trajectory_path || "--"}`,
        "",
        handeye.execute_motion ? "输入 'YES' 确认开始真机标定：" : "确认标定板、线缆和工作空间安全后继续。",
      ].join("\n");
      if (handeye.execute_motion) {
        if (window.prompt(message) !== "YES") {
          setNotice("真机标定已取消。", "warn");
          return;
        }
      } else {
        if (!window.confirm(message)) return;
      }
    }
    if (!await saveTrajectoryIfDirty()) return;
    await runCommand("开始标定", "/api/handeye/run", { confirmed: Boolean(handeye.requires_motion_confirmation) });
  });
  document.getElementById("stop-btn").addEventListener("click", () => runCommand("停止", "/api/handeye/stop"));
  document.getElementById("refresh-btn").addEventListener("click", refreshAll);
  els.sessionSelect.addEventListener("change", () => {
    const selectedSession = els.sessionSelect.value;
    state.userSelectedSession = Boolean(selectedSession);
    state.currentTrajectorySelected = !selectedSession;
    state.autoSelectedSession = false;
    state.selectedSession = selectedSession;
    state.selectedWaypointName = "";
    state.selectedSampleRow = null;
    state.selectionVersion += 1;
    updateDeleteButtonState();
    updateLoadSessionTrajectoryButton();
    refreshReport();
    refreshWaypoints();
  });
  document.addEventListener("visibilitychange", () => {
    if (!document.hidden) scheduleFullRefresh("visible");
  });
  window.addEventListener("focus", () => scheduleFullRefresh("focus"));
  document.getElementById("modal-close").addEventListener("click", hideCompletionDialog);
  els.waypointBody.addEventListener("click", (event) => {
    const row = event.target.closest("tr");
    if (!row) return;
    const item = state.waypoints.find((waypoint) => waypoint.name === row.dataset.name);
    selectWaypoint(item);
  });
}

bindUi();
initPreviewErrorHandler();
showLivePlaceholder("等待相机图像");
refreshLiveImageHttp();
connectLiveImageStream();
startLiveImageWatchdog();
connectEvents();
refreshAll();
pollEvents();
window.setInterval(() => {
  if (!state.wsConnected) pollEvents();
}, 2000);
