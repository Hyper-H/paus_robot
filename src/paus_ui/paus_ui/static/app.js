const state = {
  imageMode: "overlay",
  previewMode: "overlay",
  showAxes: true,
  selectedSession: "",
  selectedWaypointName: "",
  selectedSampleRow: null,
  status: null,
  quality: null,
  sessions: [],
  waypoints: [],
  waypointsDirty: false,
  userSelectedSession: false,
  autoSelectedSession: false,
  lastEventId: 0,
  wsConnected: false,
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

function refreshImage() {
  els.liveImage.src = `/api/image/latest.jpg?mode=${state.imageMode}&axes=${state.showAxes ? "true" : "false"}&t=${Date.now()}`;
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
  setChip(els.motionChip, `execute_motion: ${handeye.execute_motion ? "true" : "false"}`, handeye.execute_motion ? "bad" : "good");
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
  if (session.session_id && handeye.run_active && !state.userSelectedSession && state.selectedSession !== session.session_id) {
    state.selectedSession = session.session_id;
    state.autoSelectedSession = true;
  } else if (!handeye.run_active && state.autoSelectedSession) {
    state.autoSelectedSession = false;
  }
  const result = handeye.last_command_result;
  if (result) {
    setNotice(result.operator_message || result.message, commandTone(result));
  }
  if (motion.trajectory_path) {
    els.trajectoryPath.textContent = motion.trajectory_path;
  }
}

async function refreshQuality() {
  const quality = await getJson("/api/handeye/quality", {});
  state.quality = quality;
  const t = quality.camera_to_board_translation_m || [];
  const thresholds = quality.thresholds || {};
  const metrics = [
    { label: "检测", value: quality.detected ? "ok" : "no", tone: quality.detected ? "" : "warn" },
    { label: "重投影 px", value: fmt(quality.reprojection_error_px, 3), tone: thresholds.reprojection_ok === false ? "bad" : "" },
    { label: "边距 px", value: fmt(quality.board_margin_px, 1), tone: thresholds.margin_ok === false ? "bad" : "" },
    { label: "Z m", value: fmt(t[2], 3), tone: "" },
    { label: "棋盘角度", value: fmt(quality.board_angle_deg, 2, " deg"), tone: "" },
  ];
  els.qualityMetrics.innerHTML = metrics.map((item) => `<div class="metric ${item.tone}"><span>${item.label}</span><strong>${item.value}</strong></div>`).join("");
  renderPoseGrid(els.cameraBoardGrid, ["x", "y", "z", "rx", "ry", "rz"], [
    t[0],
    t[1],
    t[2],
    ...(quality.camera_to_board_rotation_rpy_deg || []),
  ], ["m", "m", "m", "deg", "deg", "deg"]);
  els.boardAngle.textContent = quality.detected ? fmt(quality.board_angle_deg, 2, " deg") : (quality.operator_message || "--");
  els.liveEmpty.classList.toggle("hidden", Boolean(quality.image_sequence));
}

async function refreshSessions() {
  const sessions = await getJson("/api/sessions", []);
  state.sessions = sessions;
  const selectedStillExists = sessions.some((session) => session.id === state.selectedSession);
  if (state.selectedSession && !selectedStillExists) {
    state.selectedSession = "";
    state.autoSelectedSession = false;
    state.userSelectedSession = false;
  }
  const options = ['<option value="">当前示教轨迹</option>'];
  options.push(...sessions.map((session) => {
    const label = `${session.id} (${session.sample_count || 0})${session.has_solution ? "" : session.has_report ? " unsolved" : " no report"}`;
    return `<option value="${escapeHtml(session.id)}">${escapeHtml(label)}</option>`;
  }));
  els.sessionSelect.innerHTML = options.join("");
  els.sessionSelect.value = state.selectedSession || "";
}

async function refreshReport() {
  if (!state.selectedSession) {
    renderEmptyReport("暂无 session");
    return;
  }
  const report = await getJson(`/api/sessions/${encodeURIComponent(state.selectedSession)}/report`, {});
  if (!report.has_report) {
    renderEmptyReport(report.empty_reason || "该 session 暂无 report.yaml。");
    return;
  }
  els.reportPath.textContent = report.report_path || `calibration_sessions/${state.selectedSession}/report.yaml`;
  els.sampleCount.textContent = report.sample_count ?? "--";
  els.acceptedCount.textContent = report.accepted_count ?? "--";
  els.skippedCount.textContent = report.skipped_count ?? "--";
  els.pendingCount.textContent = report.pending_count ?? "--";
  if (!report.has_solution) {
    els.residualGrid.innerHTML = `<div class="empty-card">report.yaml exists, but this session has not produced a solved calibration result yet.</div>`;
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

async function refreshWaypoints() {
  let waypoints = [];
  state.waypointsDirty = false;
  if (state.selectedSession) {
    waypoints = await getJson(`/api/sessions/${encodeURIComponent(state.selectedSession)}/waypoints`, []);
  } else {
    const trajectory = await getJson("/api/handeye/waypoints", { waypoints: [] });
    state.waypointsDirty = Boolean(trajectory.dirty);
    waypoints = (trajectory.waypoints || []).map((waypoint, index) => ({
      waypoint,
      index: index + 1,
      name: waypoint.name,
      status: "pending",
      result: "-",
      capture: waypoint.capture,
      thresholds: {},
    }));
    if (trajectory.trajectory_path) els.trajectoryPath.textContent = trajectory.trajectory_path;
  }
  state.waypoints = Array.isArray(waypoints) ? waypoints : [];
  const accepted = state.waypoints.filter((item) => item.status === "accepted").length;
  const skipped = state.waypoints.filter((item) => item.status === "skipped").length;
  const pending = state.waypoints.filter((item) => item.status === "pending").length;
  els.waypointStats.textContent = `共 ${state.waypoints.length} 个点`;
  els.tableFooter.textContent = `已接受 ${accepted}　跳过 ${skipped}　待采集 ${pending}`;
  els.waypointBody.innerHTML = state.waypoints.map(renderWaypointRow).join("");
  if (!state.selectedWaypointName && state.waypoints.length) {
    selectWaypoint(state.waypoints.find((item) => item.has_image) || state.waypoints[0]);
  } else {
    markSelectedRow();
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
  const thumb = item.has_image && item.sample_row_index
    ? `<button class="thumb-button" type="button" data-row="${item.sample_row_index}"><img src="/api/sessions/${encodeURIComponent(state.selectedSession)}/sample-image/${item.sample_row_index}.jpg?mode=overlay&axes=true&t=${Date.now()}" alt="sample ${item.sample_row_index}" /></button>`
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
      <td>${item.capture === false ? "no" : "OK"}</td>
      <td title="${escapeHtml(item.reason_display || item.reason || "")}">${escapeHtml(item.reason_display || item.reason || "-")}</td>
      <td>${thumb}</td>
    </tr>`;
}

function selectWaypoint(item) {
  if (!item) return;
  state.selectedWaypointName = item.name || item.waypoint?.name || "";
  state.selectedSampleRow = item.sample_row_index || null;
  markSelectedRow();
  renderPreview(item);
}

function markSelectedRow() {
  [...els.waypointBody.querySelectorAll("tr")].forEach((row) => {
    row.classList.toggle("selected", row.dataset.name === state.selectedWaypointName);
  });
}

function renderPreview(item) {
  const name = item.name || item.waypoint?.name || "样本预览";
  els.previewTitle.textContent = `${name} 预览`;
  const rowIndex = item.sample_row_index;
  if (state.selectedSession && rowIndex && item.has_image) {
    els.previewEmpty.classList.add("hidden");
    els.samplePreview.style.display = "block";
    els.samplePreview.src = `/api/sessions/${encodeURIComponent(state.selectedSession)}/sample-image/${rowIndex}.jpg?mode=${state.previewMode}&axes=${state.showAxes ? "true" : "false"}&t=${Date.now()}`;
  } else {
    els.samplePreview.removeAttribute("src");
    els.samplePreview.style.display = "none";
    els.previewEmpty.classList.remove("hidden");
    els.previewEmpty.textContent = item.reason_display || "该 waypoint 暂无样本图像";
  }
  els.previewSubtitle.textContent = item.status ? `${item.status} / ${item.result || "-"}` : "选择 waypoint 查看图像和质量指标";
  const t = item.camera_to_board_translation_m || [];
  const r = item.camera_to_board_rotation_rpy_deg || [];
  els.previewDetails.innerHTML = [
    ["reprojection_error_px", fmt(item.reprojection_error_px, 3)],
    ["board_margin_px", fmt(item.board_margin_px, 1)],
    ["T_camera_board x", fmt(t[0], 3, " m")],
    ["T_camera_board y", fmt(t[1], 3, " m")],
    ["T_camera_board z", fmt(t[2], 3, " m")],
    ["board rx", fmt(r[0], 2, " deg")],
    ["board ry", fmt(r[1], 2, " deg")],
    ["board rz", fmt(r[2], 2, " deg")],
    ["image_sequence", text(item.image_sequence)],
    ["capture_time", text(item.capture_time_s)],
  ].map(([label, value]) => `<div class="kv"><span>${label}</span><strong>${escapeHtml(value)}</strong></div>`).join("");
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
  await refreshStatus();
  await refreshSessions();
  await refreshReport();
  await refreshWaypoints();
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

async function saveTrajectoryIfDirty() {
  const dirty = Boolean(state.waypointsDirty || state.status?.handeye?.trajectory_dirty);
  if (!dirty) return true;
  setNotice("检测到未保存的示教轨迹，正在先保存轨迹...");
  const result = await runCommand("保存轨迹", "/api/handeye/save_trajectory");
  return Boolean(result?.success);
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

async function pollEvents() {
  const payload = await getJson(`/api/events?since=${state.lastEventId}`, { events: [], last_id: state.lastEventId });
  state.lastEventId = payload.last_id || state.lastEventId;
  const events = payload.events || [];
  events.forEach(addEvent);
  if (events.some((event) => event.type === "eye_to_hand_status" || event.type === "ui_command_result")) {
    refreshStatus();
    refreshWaypoints();
    refreshReport();
  }
}

function connectEvents() {
  const proto = window.location.protocol === "https:" ? "wss:" : "ws:";
  let socket;
  try {
    socket = new WebSocket(`${proto}//${window.location.host}/ws/events`);
  } catch (error) {
    state.wsConnected = false;
    els.eventMode.textContent = "polling fallback";
    return;
  }
  socket.addEventListener("open", () => {
    state.wsConnected = true;
    els.eventMode.textContent = "WebSocket connected";
  });
  socket.addEventListener("message", (event) => {
    const payload = JSON.parse(event.data);
    state.lastEventId = Math.max(state.lastEventId, Number(payload.id || 0));
    addEvent(payload);
    if (payload.type === "eye_to_hand_status" || payload.type === "ui_command_result") {
      refreshStatus();
      refreshWaypoints();
      refreshReport();
    }
  });
  socket.addEventListener("close", () => {
    state.wsConnected = false;
    els.eventMode.textContent = "polling fallback";
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
      const selected = state.waypoints.find((item) => item.name === state.selectedWaypointName);
      if (selected) renderPreview(selected);
    });
  });
  document.getElementById("axis-toggle").addEventListener("change", (event) => {
    state.showAxes = Boolean(event.target.checked);
    refreshImage();
    const selected = state.waypoints.find((item) => item.name === state.selectedWaypointName);
    if (selected) renderPreview(selected);
  });
  document.getElementById("record-btn").addEventListener("click", () => runCommand("记录当前点", "/api/handeye/record_waypoint"));
  document.getElementById("delete-btn").addEventListener("click", () => runCommand("删除上一个", "/api/handeye/delete_last_waypoint"));
  document.getElementById("save-trajectory-btn").addEventListener("click", () => runCommand("保存轨迹", "/api/handeye/save_trajectory"));
  document.getElementById("dry-run-btn").addEventListener("click", async () => {
    const handeye = state.status?.handeye || {};
    if (handeye.execute_motion || handeye.backend_config_source !== "backend_status") {
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
      const message = [
        handeye.execute_motion ? "将真实驱动机械臂执行半自动手眼标定。" : "暂未确认标定节点运动状态，继续前需要人工确认。",
        `waypoint 数量：${motion.waypoint_count ?? "--"}`,
        `运动模式：${motion.motion || "movej"}`,
        `速度/加速度：${motion.vel ?? "--"} / ${motion.acc ?? "--"}`,
        `轨迹文件：${motion.trajectory_path || handeye.trajectory_path || "--"}`,
        "确认标定板、线缆和工作空间安全后继续。",
      ].join("\n");
      if (!window.confirm(message)) return;
    }
    if (!await saveTrajectoryIfDirty()) return;
    await runCommand("开始标定", "/api/handeye/run", { confirmed: Boolean(handeye.requires_motion_confirmation) });
  });
  document.getElementById("stop-btn").addEventListener("click", () => runCommand("停止", "/api/handeye/stop"));
  document.getElementById("refresh-btn").addEventListener("click", refreshAll);
  els.sessionSelect.addEventListener("change", () => {
    state.userSelectedSession = true;
    state.autoSelectedSession = false;
    state.selectedSession = els.sessionSelect.value;
    state.selectedWaypointName = "";
    state.selectedSampleRow = null;
    refreshReport();
    refreshWaypoints();
  });
  els.waypointBody.addEventListener("click", (event) => {
    const row = event.target.closest("tr");
    if (!row) return;
    const item = state.waypoints.find((waypoint) => waypoint.name === row.dataset.name);
    selectWaypoint(item);
  });
}

bindUi();
connectEvents();
refreshImage();
refreshAll();
refreshQuality();
pollEvents();
window.setInterval(refreshImage, 1000);
window.setInterval(refreshStatus, 1500);
window.setInterval(refreshQuality, 2000);
window.setInterval(refreshSessions, 5000);
window.setInterval(() => {
  if (!state.wsConnected) pollEvents();
}, 2000);
window.setInterval(() => {
  if (!state.wsConnected) {
    refreshWaypoints();
    refreshReport();
  }
}, 5000);
