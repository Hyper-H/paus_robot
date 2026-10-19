(() => {
  const panel = document.getElementById("ultrasound-panel");
  const image = document.getElementById("ultrasound-image");
  const empty = document.getElementById("ultrasound-empty");
  let busy = false;
  let objectUrl = null;
  const set = (name, value) => { document.getElementById(`ultrasound-${name}`).textContent = value; };
  const labels = { WAITING: "等待接收端", LISTENING: "等待 Windows", CONNECTED: "等待首帧", STREAMING: "图像流可用", STALE: "图像已超时", ERROR: "接收错误", STOPPED: "接收已停止" };

  function clearImage(label) {
    image.hidden = true;
    image.removeAttribute("src");
    if (objectUrl) URL.revokeObjectURL(objectUrl);
    objectUrl = null;
    empty.hidden = false;
    empty.textContent = label;
  }

  async function refresh() {
    if (busy || document.hidden) return;
    busy = true;
    const abort = new AbortController();
    const timeout = setTimeout(() => abort.abort(), 2000);
    try {
      const response = await fetch("/api/ultrasound/status", { cache: "no-store", signal: abort.signal });
      if (!response.ok) throw new Error("status unavailable");
      const status = await response.json();
      const label = labels[status.state] || "状态未知";
      set("state", label);
      document.getElementById("ultrasound-state").className = `chip ${status.stream_available ? "good" : "neutral"}`;
      set("received", String(status.received ?? 0));
      set("fps", `${Number(status.fps_2s || 0).toFixed(1)} Hz`);
      set("age", status.latest_age_s == null ? "--" : `${(status.latest_age_s * 1000).toFixed(0)} ms`);
      set("roi", status.roi_confirmed ? "已确认" : "未确认");
      set("size", status.width ? `${status.width} × ${status.height} ${status.encoding}` : "--");
      const scale = status.display_scale_mm_per_pixel || {};
      const axis = (name, value) => value == null ? `${name}未知` : `${name} ${Number(value).toFixed(5)} mm/px`;
      set("scale", `${axis("横向", scale.horizontal)} / ${axis("纵向", scale.vertical)}`);
      set("context", status.imaging_context || "未记录");
      set("revision", status.profile_revision || "--");
      if (!status.stream_available) {
        clearImage(label);
      } else if (panel.open) {
        const frame = await fetch("/api/ultrasound/image.jpg", { cache: "no-store", signal: abort.signal });
        if (frame.status === 204) {
          clearImage("等待新图像");
        } else {
          if (!frame.ok) throw new Error("preview unavailable");
          const next = URL.createObjectURL(await frame.blob());
          const staging = new Image();
          staging.src = next;
          try {
            await staging.decode();
            image.src = next;
            image.hidden = false;
            empty.hidden = true;
            if (objectUrl) URL.revokeObjectURL(objectUrl);
            objectUrl = next;
          } catch (error) {
            URL.revokeObjectURL(next);
            throw error;
          }
        }
      }
    } catch (_) {
      set("state", "主控连接中断");
      document.getElementById("ultrasound-state").className = "chip neutral";
      clearImage("等待主控连接");
    } finally {
      clearTimeout(timeout);
      busy = false;
    }
  }

  panel.addEventListener("toggle", refresh);
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) clearImage("等待新图像");
    else refresh();
  });
  setInterval(refresh, 250);
  refresh();
})();
