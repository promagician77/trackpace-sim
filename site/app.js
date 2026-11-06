/* Track pacing review site. All figures come from data/*.json (exported from the PX4 flight logs). */
"use strict";

const TRK = { kerb: 36.5, L: 84.39, lw: 1.22, lanes: 8 };
TRK.R1 = TRK.kerb + 0.30;
TRK.P = 2 * Math.PI * TRK.R1 + 2 * TRK.L;
const COL = { c1: "#2E63A8", c2: "#C66C22", c3: "#1F8A70", c4: "#8A55B5", ink: "#1C1F24", muted: "#7A7F87", rule: "#DDD7CA", rule2: "#EAE5DA", tartan: "#B8432F" };
const $ = (s, el = document) => el.querySelector(s);
const $$ = (s, el = document) => [...el.querySelectorAll(s)];
const fmt = (v, d = 2) => (v == null || Number.isNaN(v) ? "—" : Number(v).toFixed(d));
const sgn = (v, d = 0) => (v == null ? "—" : (v > 0 ? "+" : v < 0 ? "−" : "±") + Math.abs(v).toFixed(d));
const clockStr = (t) => { if (t == null || t < 0) return "0:00.0"; const m = Math.floor(t / 60); const s = t - 60 * m; return `${m}:${s.toFixed(1).padStart(4, "0")}`; };
const DATA = { summary: [], byId: {}, replay: {} };
const RENDER = {}; let CUR_TAB = null, TM_RUN = "gz59";
const msOrS = (v) => (Math.abs(v) >= 2000 ? `${sgn(v / 1000, 2)} s` : `${sgn(v, 0)} ms`);
const absMsOrS = (v) => (Math.abs(v) >= 2000 ? `${(Math.abs(v) / 1000).toFixed(1)} s` : `${Math.round(Math.abs(v))} ms`);

async function loadJSON(p) { const r = await fetch(p); if (!r.ok) throw new Error(p); return r.json(); }
async function replayOf(id) { if (!DATA.replay[id]) DATA.replay[id] = await loadJSON(`data/replay_${id}.json`); return DATA.replay[id]; }

/* ---------- track geometry (same as trackpace/track.py) ---------- */
function trackPoint(s, R = TRK.R1) {
  const L = TRK.L, P = 2 * Math.PI * R + 2 * L, b = Math.PI * R;
  s = ((s % P) + P) % P;
  if (s < b) { const th = -Math.PI / 2 + s / R; return [L / 2 + R * Math.cos(th), R * Math.sin(th)]; }
  if (s < b + L) return [L / 2 - (s - b), R];
  if (s < 2 * b + L) { const th = Math.PI / 2 + (s - b - L) / R; return [-L / 2 + R * Math.cos(th), R * Math.sin(th)]; }
  return [-L / 2 + (s - 2 * b - L), -R];
}

/* ---------- tabs ---------- */
function initTabs() {
  const btns = $$("nav.tabs button");
  const show = (id) => {
    btns.forEach((b) => b.setAttribute("aria-selected", b.dataset.tab === id));
    $$("section.tab").forEach((s) => s.classList.toggle("on", s.id === id));
    CUR_TAB = id; requestAnimationFrame(() => { try { RENDER[id] && RENDER[id](); } catch (e) { console.error(e); } });
    try { history.replaceState(null, "", "#" + id); } catch (e) {}
    window.dispatchEvent(new Event("resize"));
  };
  btns.forEach((b) => b.addEventListener("click", () => show(b.dataset.tab)));
  const fromHash = () => { const h = location.hash.slice(1); show(btns.some((b) => b.dataset.tab === h) ? h : btns[0].dataset.tab); };
  window.addEventListener("hashchange", fromHash);
  fromHash();
}

/* ---------- SVG chart helpers ---------- */
const NS = "http://www.w3.org/2000/svg";
function el(tag, attrs = {}, parent) { const e = document.createElementNS(NS, tag); for (const k in attrs) e.setAttribute(k, attrs[k]); if (parent) parent.appendChild(e); return e; }
function ticks(a, b, n = 5) {
  const span = b - a, step0 = span / n, mag = Math.pow(10, Math.floor(Math.log10(step0)));
  const step = [1, 2, 2.5, 5, 10].map((m) => m * mag).find((s) => s >= step0) || mag * 10;
  const out = []; for (let v = Math.ceil(a / step) * step; v <= b + 1e-9; v += step) out.push(+v.toFixed(10)); return out;
}
function lineChart(host, o) {
  host.innerHTML = ""; host.classList.add("chart");
  const W = Math.max(320, host.clientWidth || o.width || 900), H = o.height || 300, m = { l: 54, r: 18, t: 14, b: 38, ...(o.margin || {}) };
  const svg = el("svg", { viewBox: `0 0 ${W} ${H}`, role: "img", "aria-label": o.aria || "" }, host);
  const xs = o.series.flatMap((s) => s.x), ys = o.series.flatMap((s) => s.y).filter((v) => v != null && isFinite(v));
  const [x0, x1] = o.xDomain || [Math.min(...xs), Math.max(...xs)];
  let [y0, y1] = o.yDomain || [Math.min(...ys), Math.max(...ys)];
  if (!o.yDomain) { const pad = (y1 - y0) * 0.08 || 1; y0 -= pad; y1 += pad; }
  const X = (v) => m.l + ((v - x0) / (x1 - x0)) * (W - m.l - m.r), Y = (v) => H - m.b - ((v - y0) / (y1 - y0)) * (H - m.t - m.b);
  (o.bands || []).forEach((b) => el("rect", { x: X(b[0]), y: m.t, width: Math.max(0, X(b[1]) - X(b[0])), height: H - m.t - m.b, fill: b[2] || "#EFE9DC" }, svg));
  (o.bandLabels || []).forEach((b) => { const t = el("text", { x: (X(b[0]) + X(b[1])) / 2, y: m.t + 12, "text-anchor": "middle", class: "lbl", style: "font-size:11px;fill:#8A857A" }, svg); t.textContent = b[2]; });
  const g = el("g", { class: "grid" }, svg), ax = el("g", { class: "ax" }, svg);
  ticks(y0, y1, o.yTicks || 5).forEach((v) => {
    el("line", { x1: m.l, x2: W - m.r, y1: Y(v), y2: Y(v) }, g);
    const t = el("text", { x: m.l - 8, y: Y(v) + 4, "text-anchor": "end" }, ax); t.textContent = o.yFmt ? o.yFmt(v) : v;
  });
  ticks(x0, x1, o.xTicks || 8).forEach((v) => { const t = el("text", { x: X(v), y: H - m.b + 18, "text-anchor": "middle" }, ax); t.textContent = o.xFmt ? o.xFmt(v) : v; });
  el("line", { x1: m.l, x2: W - m.r, y1: H - m.b, y2: H - m.b }, ax);
  if (y0 < 0 && y1 > 0) el("line", { x1: m.l, x2: W - m.r, y1: Y(0), y2: Y(0), stroke: "#A9A396", "stroke-width": 1 }, svg);
  (o.refLines || []).forEach((r) => { el("line", { x1: m.l, x2: W - m.r, y1: Y(r.y), y2: Y(r.y), stroke: r.color || "#A9A396", "stroke-dasharray": "4 4" }, svg); if (r.label) { const t = el("text", { x: m.l + 6, y: Y(r.y) - 5, "text-anchor": "start", class: "lbl", style: "font-size:11.5px;paint-order:stroke;stroke:#FFFDF8;stroke-width:3px" }, svg); t.textContent = r.label; } });
  if (o.xLabel) { const t = el("text", { x: W - m.r, y: H - 4, "text-anchor": "end", class: "lbl", style: "fill:#7A7F87;font-size:11.5px" }, svg); t.textContent = o.xLabel; }
  if (o.yLabel) { const t = el("text", { x: m.l, y: m.t - 2, class: "lbl", style: "fill:#7A7F87;font-size:11.5px" }, svg); t.textContent = o.yLabel; }
  o.series.forEach((s) => {
    let d = "", pen = false;
    s.x.forEach((xv, i) => { const yv = s.y[i]; if (yv == null || !isFinite(yv)) { pen = false; return; } d += (pen ? "L" : "M") + X(xv).toFixed(1) + "," + Y(yv).toFixed(1); pen = true; });
    el("path", { d, fill: "none", stroke: s.color, "stroke-width": s.width || 2, "stroke-opacity": s.opacity ?? 1, "stroke-linejoin": "round", "stroke-linecap": "round" }, svg);
    if (s.label) { const i = s.labelAt ?? s.x.length - 1; const t = el("text", { x: X(s.x[i]) + 6, y: Y(s.y[i]) + 4, class: "lbl" }, svg); t.textContent = s.label; }
  });
  // hover: crosshair + tooltip on the first "hover" series set
  const hs = o.series.filter((s) => s.hover !== false);
  if (hs.length) {
    const tip = document.createElement("div"); tip.className = "tip"; host.appendChild(tip);
    const vline = el("line", { y1: m.t, y2: H - m.b, stroke: "#1C1F24", "stroke-opacity": 0.35, visibility: "hidden" }, svg);
    const dots = hs.map((s) => el("circle", { r: 4.5, fill: s.color, stroke: "#FFFDF8", "stroke-width": 2, visibility: "hidden" }, svg));
    const hit = el("rect", { x: m.l, y: m.t, width: W - m.l - m.r, height: H - m.t - m.b, fill: "transparent" }, svg);
    const move = (ev) => {
      const r = svg.getBoundingClientRect(), px = ((ev.clientX - r.left) / r.width) * W;
      const xv = x0 + ((px - m.l) / (W - m.l - m.r)) * (x1 - x0);
      const s0 = hs[0]; let i = 0, best = 1e18; s0.x.forEach((v, k) => { const d = Math.abs(v - xv); if (d < best) { best = d; i = k; } });
      vline.setAttribute("x1", X(s0.x[i])); vline.setAttribute("x2", X(s0.x[i])); vline.setAttribute("visibility", "visible");
      const lines = [o.xFmtTip ? o.xFmtTip(s0.x[i]) : s0.x[i]];
      hs.forEach((s, k) => { const yv = s.y[i]; if (yv == null) { dots[k].setAttribute("visibility", "hidden"); return; } dots[k].setAttribute("cx", X(s.x[i])); dots[k].setAttribute("cy", Y(yv)); dots[k].setAttribute("visibility", "visible"); lines.push(`${s.name}: ${o.yFmtTip ? o.yFmtTip(yv) : yv}`); });
      tip.innerHTML = lines.join("<br>"); tip.classList.add("on");
      tip.style.left = (X(s0.x[i]) / W) * r.width + "px"; tip.style.top = (Y(hs[0].y[i] ?? 0) / H) * r.height + "px";
    };
    hit.addEventListener("pointermove", move); hit.addEventListener("pointerleave", () => { tip.classList.remove("on"); vline.setAttribute("visibility", "hidden"); dots.forEach((d) => d.setAttribute("visibility", "hidden")); });
  }
  return svg;
}
function barChart(host, o) {
  host.innerHTML = ""; host.classList.add("chart");
  const W = Math.max(320, host.clientWidth || o.width || 900), rowH = o.rowH || 44, m = { l: o.labelW || 260, r: o.marginR ?? 90, t: 8, b: 26 };
  const H = m.t + m.b + rowH * o.rows.length;
  const svg = el("svg", { viewBox: `0 0 ${W} ${H}`, role: "img", "aria-label": o.aria || "" }, host);
  const vmax = o.max || Math.max(...o.rows.map((r) => r.v)) * 1.05;
  const X = (v) => m.l + (v / vmax) * (W - m.l - m.r);
  const ax = el("g", { class: "ax" }, svg), g = el("g", { class: "grid" }, svg);
  ticks(0, vmax, o.ticks || 5).forEach((v) => { el("line", { x1: X(v), x2: X(v), y1: m.t, y2: H - m.b }, g); const t = el("text", { x: X(v), y: H - 8, "text-anchor": "middle" }, ax); t.textContent = o.fmt ? o.fmt(v) : v; });
  const tip = document.createElement("div"); tip.className = "tip"; host.appendChild(tip);
  o.rows.forEach((r, i) => {
    const y = m.t + i * rowH + 8, h = rowH - 16;
    const t = el("text", { x: m.l - 12, y: y + h / 2 + 4, "text-anchor": "end", class: "lbl", style: "font-size:13px;fill:#1C1F24" }, svg); t.textContent = r.label;
    const w = Math.max(3, X(r.v) - m.l);
    const bar = el("path", { d: `M${m.l},${y}h${w - 4}a4,4 0 0 1 4,4v${h - 8}a4,4 0 0 1 -4,4h${-(w - 4)}z`, fill: r.color }, svg);
    const v = el("text", { x: m.l + w + 8, y: y + h / 2 + 4, class: "lbl", style: "font-family:JetBrains Mono,monospace;font-size:12.5px;fill:#1C1F24" }, svg); v.textContent = r.text;
    const hit = el("rect", { x: 0, y: y - 6, width: W, height: rowH, fill: "transparent" }, svg);
    hit.addEventListener("pointermove", (ev) => { const b = svg.getBoundingClientRect(); tip.innerHTML = r.tip || `${r.label}: ${r.text}`; tip.style.left = ((m.l + w) / W) * b.width + "px"; tip.style.top = (y / H) * b.height + "px"; tip.classList.add("on"); bar.setAttribute("fill-opacity", 0.85); });
    hit.addEventListener("pointerleave", () => { tip.classList.remove("on"); bar.setAttribute("fill-opacity", 1); });
  });
}

/* ---------- replay ---------- */
const RP = { id: null, d: null, run: null, i: 0, t: 0, playing: false, speed: 4, follow: false, last: 0 };
function drawTrack(ctx, k) {
  const L = TRK.L, Ro = TRK.kerb + TRK.lanes * TRK.lw;
  const stadium = (R) => { ctx.beginPath(); ctx.moveTo(-L / 2, -R); ctx.lineTo(L / 2, -R); ctx.arc(L / 2, 0, R, -Math.PI / 2, Math.PI / 2); ctx.lineTo(-L / 2, R); ctx.arc(-L / 2, 0, R, Math.PI / 2, 1.5 * Math.PI); ctx.closePath(); };
  ctx.fillStyle = "#6E8F5E"; ctx.fillRect(-200, -120, 400, 240);
  stadium(Ro + 1.5); ctx.fillStyle = "#C9C1B0"; ctx.fill();
  stadium(Ro); ctx.fillStyle = "#B8432F"; ctx.fill();
  stadium(TRK.kerb); ctx.fillStyle = "#5C8A4E"; ctx.fill();
  ctx.strokeStyle = "rgba(255,255,255,.95)";
  for (let j = 0; j <= TRK.lanes; j++) { stadium(TRK.kerb + j * TRK.lw); ctx.lineWidth = Math.max(0.05, (j === 0 ? 1.6 : 1) / k); ctx.stroke(); }
  // finish line
  ctx.lineWidth = Math.max(0.1, 2.2 / k); ctx.beginPath(); ctx.moveTo(L / 2, -TRK.kerb); ctx.lineTo(L / 2, -Ro); ctx.stroke();
  // measurement line of lane 1 (pace line)
  ctx.setLineDash([6 / k, 5 / k]); ctx.strokeStyle = "rgba(255,240,170,.85)"; ctx.lineWidth = 1.2 / k; stadium(TRK.R1); ctx.stroke(); ctx.setLineDash([]);
  // labels (screen-scaled)
  ctx.save(); ctx.fillStyle = "rgba(255,255,255,.88)"; ctx.font = `600 ${13 / k}px Barlow Condensed, sans-serif`; ctx.textAlign = "center";
  const lab = (txt, x, y) => { ctx.save(); ctx.translate(x, y); ctx.scale(1, -1); ctx.fillText(txt, 0, 0); ctx.restore(); };
  lab("HOME STRAIGHT  →", 0, -TRK.kerb + 4.5); lab("←  BACK STRAIGHT", 0, TRK.kerb - 7.5); lab("BEND 1", L / 2 + 16, 0); lab("BEND 2", -L / 2 - 16, 0);
  lab("FINISH / 400 m START", L / 2, -Ro - 5);
  ctx.fillStyle = "rgba(255,255,255,.8)"; ctx.font = `500 ${11 / k}px JetBrains Mono, monospace`;
  [100, 200, 300].forEach((s) => { const [x, y] = trackPoint(s, TRK.kerb - 3.2); lab(s + " m", x, y); });
  ctx.restore();
}
function drawDrone(ctx, x, y, hdg, k, color) {
  ctx.save(); ctx.translate(x, y); ctx.rotate(hdg); const sz = 9 / k;
  ctx.strokeStyle = "#111"; ctx.lineWidth = 2.2 / k;
  ctx.beginPath(); ctx.moveTo(-sz, -sz); ctx.lineTo(sz, sz); ctx.moveTo(-sz, sz); ctx.lineTo(sz, -sz); ctx.stroke();
  [[sz, sz], [sz, -sz], [-sz, sz], [-sz, -sz]].forEach(([a, b]) => { ctx.beginPath(); ctx.arc(a, b, sz * 0.62, 0, 2 * Math.PI); ctx.fillStyle = "rgba(255,255,255,.75)"; ctx.fill(); ctx.lineWidth = 1.4 / k; ctx.stroke(); });
  ctx.beginPath(); ctx.arc(0, 0, sz * 0.45, 0, 2 * Math.PI); ctx.fillStyle = color; ctx.fill(); ctx.lineWidth = 1.5 / k; ctx.stroke();
  ctx.beginPath(); ctx.moveTo(sz * 1.25, 0); ctx.lineTo(sz * 2.1, 0); ctx.strokeStyle = color; ctx.lineWidth = 2.4 / k; ctx.stroke();
  ctx.restore();
}
function renderReplay() {
  const cv = $("#rp-canvas"); if (!cv || !RP.d) return;
  const box = cv.parentElement.getBoundingClientRect(); const dpr = Math.min(2, window.devicePixelRatio || 1);
  const W = Math.max(300, box.width), H = Math.round(W * 0.56);
  if (cv.width !== Math.round(W * dpr)) { cv.width = Math.round(W * dpr); cv.height = Math.round(H * dpr); cv.style.height = H + "px"; }
  const ctx = cv.getContext("2d"); ctx.setTransform(1, 0, 0, 1, 0, 0); ctx.clearRect(0, 0, cv.width, cv.height);
  const d = RP.d, i = RP.i;
  const ext = TRK.L / 2 + TRK.kerb + TRK.lanes * TRK.lw + 6;
  let k, cx = 0, cy = 0;
  if (RP.follow) { k = (cv.width / dpr) / 34; cx = d.x[i]; cy = d.y[i]; } else { k = Math.min((cv.width / dpr) / (2 * ext), (cv.height / dpr) / (2 * (TRK.kerb + TRK.lanes * TRK.lw + 9))); }
  ctx.setTransform(k * dpr, 0, 0, -k * dpr, cv.width / 2 - cx * k * dpr, cv.height / 2 + cy * k * dpr);
  drawTrack(ctx, k);
  // trail: last 25 s
  const t0 = d.t[i] - 25; let j = i; while (j > 0 && d.t[j] > t0) j--;
  for (let q = j + 1; q <= i; q++) { const a = (d.t[q] - t0) / 25; ctx.strokeStyle = `rgba(28,31,36,${(0.15 + 0.75 * a).toFixed(3)})`; ctx.lineWidth = 2.2 / k; ctx.beginPath(); ctx.moveTo(d.x[q - 1], d.y[q - 1]); ctx.lineTo(d.x[q], d.y[q]); ctx.stroke(); }
  // virtual runner
  if (d.athlete_x && d.athlete_x[i] != null) {
    ctx.beginPath(); ctx.arc(d.athlete_x[i], d.athlete_y[i], (RP.follow ? 0.32 : 6.5 / k), 0, 2 * Math.PI); ctx.fillStyle = "#F2C230"; ctx.fill(); ctx.strokeStyle = "#111"; ctx.lineWidth = 1.6 / k; ctx.stroke();
  }
  const i2 = Math.min(d.x.length - 1, i + 1), i1 = Math.max(0, i - 1);
  const hdg = Math.atan2(d.y[i2] - d.y[i1], d.x[i2] - d.x[i1]);
  drawDrone(ctx, d.x[i], d.y[i], hdg, RP.follow ? Math.max(k, 22) : k, COL.c1);
  if (RP.follow) { // scale bar
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0); const px = 2 * k; ctx.fillStyle = "#1C1F24"; ctx.fillRect(16, cv.height / dpr - 22, px, 3); ctx.font = "500 12px JetBrains Mono, monospace"; ctx.fillText("2 m", 16 + px + 6, cv.height / dpr - 18);
  }
  updateReadouts();
}
function updateReadouts() {
  const d = RP.d, i = RP.i, r = RP.run; if (!d || !r) return;
  const t = d.t[i], cross = r.crossings_s || [], N = r.laps_target_s.length;
  let lap = 0; while (lap < cross.length && t >= cross[lap]) lap++;
  const inRace = t >= 0 && lap <= N && (cross.length < 2 || t <= cross[cross.length - 1] + 0.01);
  $("#ro-clock").textContent = clockStr(Math.max(0, Math.min(t, cross[cross.length - 1] ?? t)));
  $("#ro-lap").textContent = t < 0 ? "run-up" : lap > N ? "finished" : `lap ${Math.max(1, lap)} of ${N}`;
  $("#ro-target").textContent = t < 0 || lap > N ? "" : `target ${fmt(r.laps_target_s[Math.max(0, lap - 1)], 1)} s`;
  const v = d.v_plan ? d.v_plan[i] : null;
  $("#ro-gap").textContent = d.along && d.along[i] != null && inRace ? sgn(d.along[i], 2) : "—";
  $("#ro-gap-s").textContent = d.along && v > 0.5 && inRace ? `${sgn(d.along[i] / v, 3)} s` : r.mode === "mission" ? "no pace clock" : "";
  $("#ro-xt").textContent = sgn(d.xtrack[i], 2);
  $("#ro-spd").textContent = fmt(d.speed[i], 2);
  $("#ro-alt").textContent = fmt(d.alt[i], 2);
  const tb = $("#rp-splits tbody"); tb.innerHTML = "";
  for (let k = 0; k < N; k++) {
    const done = cross.length > k + 1 && t >= cross[k + 1];
    const meas = done ? r.lap_times_s[k] : null, err = done ? r.lap_error_ms[k] : null;
    const ce = r.crossing_error_ms && done ? r.crossing_error_ms[k + 1] : null;
    tb.insertAdjacentHTML("beforeend", `<tr class="${done ? "" : "pending"}"><td>${k + 1}</td><td>${fmt(r.laps_target_s[k], 2)}</td><td>${done ? fmt(meas, 3) : "—"}</td><td class="${err < 0 ? "neg" : "pos"}">${done ? sgn(err, 0) : "—"}</td><td>${ce != null ? sgn(ce, 0) : "—"}</td></tr>`);
  }
  $("#rp-time").textContent = `${t < 0 ? "−" : ""}${clockStr(Math.abs(t))}`;
  $("#rp-scrub").value = i;
}
function tick(now) {
  if (RP.playing && RP.d) {
    const dt = Math.min(0.1, (now - (RP.last || now)) / 1000); RP.t += dt * RP.speed;
    const d = RP.d; while (RP.i < d.t.length - 1 && d.t[RP.i + 1] <= RP.t) RP.i++;
    if (RP.i >= d.t.length - 1) { RP.i = 0; RP.t = d.t[0]; }
    renderReplay();
  }
  RP.last = now; requestAnimationFrame(tick);
}
async function selectRun(id) {
  RP.id = id; RP.run = DATA.byId[id]; RP.d = await replayOf(id);
  $$("#rp-runs button").forEach((b) => b.setAttribute("aria-pressed", b.dataset.id === id));
  const d = RP.d; let i = 0; const start = RP.run.mode === "mission" ? -12 : (RP.run.plan.segments?.[0]?.t0 ?? 0) - 2; while (i < d.t.length - 1 && d.t[i] < start) i++;
  RP.i = i; RP.t = d.t[i]; $("#rp-scrub").max = d.t.length - 1;
  $("#rp-note").innerHTML = RP.run.note || "";
  renderReplay();
}
function initReplay() {
  const runs = DATA.summary;
  const host = $("#rp-runs");
  runs.forEach((r) => { const b = document.createElement("button"); b.dataset.id = r.id; b.innerHTML = `${r.label}<small>${r.sim}${r.group === "baseline" ? " · baseline" : ""}</small>`; b.onclick = () => selectRun(r.id); host.appendChild(b); });
  $("#rp-play").onclick = () => { RP.playing = !RP.playing; $("#rp-play").textContent = RP.playing ? "❚❚ pause" : "▶ play"; $("#rp-play").classList.toggle("on", RP.playing); };
  $$("#rp-speeds button").forEach((b) => b.onclick = () => { RP.speed = +b.dataset.s; $$("#rp-speeds button").forEach((x) => x.classList.toggle("on", x === b)); });
  $("#rp-follow").onclick = () => { RP.follow = !RP.follow; $("#rp-follow").classList.toggle("on", RP.follow); $("#rp-follow").textContent = RP.follow ? "whole track" : "follow (34 m view)"; renderReplay(); };
  $("#rp-scrub").oninput = (e) => { RP.i = +e.target.value; RP.t = RP.d.t[RP.i]; renderReplay(); };
  window.addEventListener("resize", renderReplay);
  selectRun(runs[0].id); requestAnimationFrame(tick);
}

/* ---------- timing tab ---------- */
function crossingErrs(r) {
  if (r.crossing_error_ms) return r.crossing_error_ms;
  const c = r.crossings_s || []; return c.map((t, k) => 1000 * (t - c[0] - r.laps_target_s.slice(0, k).reduce((a, b) => a + b, 0)));
}
async function renderTiming(id = TM_RUN) {
  TM_RUN = id; const r = DATA.byId[id]; const d = await replayOf(id);
  $$("#tm-runs button").forEach((b) => b.setAttribute("aria-pressed", b.dataset.id === id));
  const ce = r.crossing_error_ms || [];
  $("#tm-table tbody").innerHTML = r.lap_times_s.map((lt, k) => `<tr><td>Lap ${k + 1}</td><td class="n">${fmt(r.laps_target_s[k], 3)}</td><td class="n">${fmt(lt, 3)}</td><td class="n">${sgn(r.lap_error_ms[k], 0)} ms</td><td class="n">${ce[k + 1] != null ? sgn(ce[k + 1], 0) + " ms" : "—"}</td><td class="n">${fmt(r.per_lap[k]?.along_track_rms_m, 2)} m</td><td class="n">${fmt(r.per_lap[k]?.speed_mean, 2)}</td></tr>`).join("")
    + `<tr class="hl"><td>Start</td><td class="n" colspan="3">pace clock zero, drone at ${fmt(r.laps_target_s[0] ? r.lap_length_m / r.laps_target_s[0] : 0, 2)} m/s</td><td class="n">${ce[0] != null ? sgn(ce[0], 0) + " ms" : "—"}</td><td class="n" colspan="2"></td></tr>`;
  const T = r.laps_target_s.reduce((a, b) => a + b, 0);
  const idx = d.t.map((t, i) => (t >= 0 && t <= T ? i : -1)).filter((i) => i >= 0);
  lineChart($("#tm-gap"), {
    height: 280, xDomain: [0, T], yDomain: [-1.6, 1.6], yTicks: 6, xLabel: "race time (s)", yLabel: "metres ahead (+) / behind (−) of the virtual runner",
    bands: r.crossings_s.slice(1, -1).map((c) => [c - 0.25, c + 0.25, "#E4DDCD"]),
    series: [{ name: "gap", color: COL.c1, x: idx.map((i) => d.t[i]), y: idx.map((i) => d.along[i]), width: 1.6 }],
    xFmtTip: (v) => `t = ${v.toFixed(1)} s`, yFmtTip: (v) => `${sgn(v, 2)} m`, aria: "Gap between drone and virtual runner over the race",
  });
  $("#tm-gap-cap").textContent = `RMS ${fmt(r.along_track_m.rms, 2)} m · worst ${fmt(r.along_track_m.max_abs, 2)} m · mean ${sgn(r.along_track_m.mean, 2)} m. Shaded marks are line crossings.`;
}
function renderMethods() {
  const ids = ["ff59", "nodrag59", "pos59", "mission59"].filter((i) => DATA.byId[i]);
  const name = { ff59: "Feed-forward + identified drag", nodrag59: "Feed-forward, no drag term", pos59: "Position-only setpoints", mission59: "Waypoint mission" };
  const color = { ff59: COL.c1, nodrag59: COL.c3, pos59: COL.c2, mission59: COL.c4 };
  const rows = ids.map((id) => { const r = DATA.byId[id]; const ce = crossingErrs(r).slice(1); const w = Math.max(...ce.map(Math.abs)); return { id, r, w, mean: ce.reduce((a, b) => a + b, 0) / ce.length }; });
  barChart($("#mt-bars"), { labelW: 250, rows: rows.filter((x) => x.id !== "mission59").map((x) => ({ label: name[x.id], v: x.w, color: color[x.id], text: `${x.w >= 1000 ? (x.w / 1000).toFixed(2) + " s" : Math.round(x.w) + " ms"}`, tip: `${name[x.id]}<br>worst line crossing vs pace clock: ${Math.round(x.w)} ms<br>mean: ${sgn(x.mean, 0)} ms` })), fmt: (v) => (v >= 1000 ? v / 1000 + " s" : v + " ms"), aria: "Worst line-crossing error by method" });
  $("#mt-table tbody").innerHTML = rows.map((x) => { const r = x.r; return `<tr${x.id === "ff59" ? ' class="hl"' : ""}><td><span class="sw" style="background:${color[x.id]}"></span>${name[x.id]}</td><td class="n">${r.lap_times_s.length}</td><td class="n">${absMsOrS(Math.max(...r.lap_error_ms.map(Math.abs)))}</td><td class="n">${msOrS(x.mean)}</td><td class="n">${r.along_track_m ? fmt(r.along_track_m.rms, 2) + " m" : "—"}</td><td class="n">${fmt(r.cross_track_m.all.rms, 2)} m</td><td class="n">${sgn(-r.cross_track_m.bend_mean_inward_m, 2)} m</td></tr>`; }).join("");
}

/* ---------- path tab ---------- */
let PT_RUN = "gz59";
async function renderPath(id = PT_RUN) {
  if (!DATA.byId[id]) id = "ff59";
  PT_RUN = id; const r = DATA.byId[id]; const p = r.profile; const b1 = Math.PI * TRK.R1;
  $$("#pt-runs button").forEach((b) => b.setAttribute("aria-pressed", b.dataset.id === id));
  const n = r.lap_times_s.length, ct = r.cross_track_m;
  $("#pt-tiles").innerHTML = [
    [`${fmt(ct.all.rms, 2)} m`, `RMS off the lane-1 line over ${n} laps (a lane is 1.22 m wide)`],
    [`${fmt(ct.straights.rms, 2)} / ${fmt(ct.bends.rms, 2)} m`, `straights / bends RMS; ${fmt(ct.junction_5m.rms, 2)} m within 5 m of a straight–bend join`],
    [`${fmt(r.repeatability.path_std_mean_m, 2)} m`, `spread of the path between laps at the same spot (mean std; widest ${fmt(r.repeatability.path_spread_max_m, 2)} m)`],
    [`${fmt(r.speed_m_s.mean, 2)} ± ${fmt(r.speed_m_s.std, 2)}`, "ground speed m/s during the laps (runner pace 6.78)"],
  ].map(([b, s]) => `<div class="tile"><b>${b}</b><span>${s}</span></div>`).join("");
  $("#pt-title").textContent = `Distance off the lane-1 line, all ${n} laps · ${r.label} · ${r.sim}`;
  $("#pt-note").textContent = id === "gz59"
    ? `Gazebo's x500 with its estimator matched to the simulated GPS: laps lie on top of each other (${fmt(r.repeatability.path_std_mean_m, 2)} m mean spread). Within 5 m of the joins, where curvature jumps from 0 to 1/36.8 m, the error stays at ${fmt(ct.junction_5m.rms, 2)} m RMS, because the turn acceleration is fed forward from the exact point the bend starts.`
    : `Within 5 m of the joins the error is no larger than elsewhere (${fmt(ct.junction_5m.rms, 2)} m RMS against ${fmt(ct.all.rms, 2)} m overall). What's left is slow wander (±0.5–1 m over 20–30 s), and much of it is the estimator: SIH adds 0.2 m GPS noise and baro drift, and the controller tracks the estimate, not the truth (estimate vs truth ${fmt(r.estimate_vs_truth?.horizontal_rms_m, 2)} m RMS here, against ${fmt(DATA.byId.gz59?.estimate_vs_truth?.horizontal_rms_m, 2)} m in Gazebo).`;
  const bands = [[0, b1, "#EFE9DC"], [b1 + TRK.L + b1 - b1, b1 + TRK.L + b1, "#EFE9DC"]];
  const series = p.xtrack_by_lap.map((ys, k) => ({ name: `lap ${k + 1}`, color: "#9C968A", x: p.s, y: ys, width: 1.1, opacity: 0.8, hover: false }));
  series.push({ name: "mean of laps", color: COL.c1, x: p.s, y: p.xtrack_mean, width: 2.4 });
  lineChart($("#pt-xt"), { height: 300, xDomain: [0, 400], yDomain: [-1.5, 1.5], yTicks: 6, bands: [[0, b1], [b1 + TRK.L, 2 * b1 + TRK.L]], bandLabels: [[0, b1, "BEND 1"], [b1, b1 + TRK.L, "BACK STRAIGHT"], [b1 + TRK.L, 2 * b1 + TRK.L, "BEND 2"], [2 * b1 + TRK.L, 400, "HOME STRAIGHT"]], xLabel: "distance into the lap (m)", yLabel: "metres outside (+) / inside (−) the lane-1 line", series, xFmtTip: (v) => `${v} m into the lap`, yFmtTip: (v) => `${sgn(v, 2)} m` });
  // speed by lap distance: lane 1 vs flying outside
  const prof = async (id) => { const d = await replayOf(id); const n = 200, acc = new Array(n).fill(0), cnt = new Array(n).fill(0); const T = DATA.byId[id].laps_target_s.reduce((a, b) => a + b, 0); d.t.forEach((t, i) => { if (t < 0 || t > T) return; const s = ((d.s[i] % TRK.P) + TRK.P) % TRK.P; const k = Math.min(n - 1, Math.floor(s / 2)); acc[k] += d.speed[i]; cnt[k]++; }); return acc.map((a, k) => (cnt[k] ? a / cnt[k] : null)); };
  if (DATA.byId.lane2) {
    const [a, c] = await Promise.all([prof("ff59"), prof("lane2")]); const xs = a.map((_, k) => k * 2 + 1);
    lineChart($("#pt-spd"), { height: 260, xDomain: [0, 400], yDomain: [6.2, 7.4], yTicks: 6, bands: [[0, b1], [b1 + TRK.L, 2 * b1 + TRK.L]], xLabel: "distance into the lap (m)", yLabel: "drone ground speed (m/s), mean of laps", series: [{ name: "over the lane-1 line", color: COL.c1, x: xs, y: a }, { name: "1.2 m outside lane 1", color: COL.c2, x: xs, y: c }], refLines: [{ y: TRK.P / 59, label: "runner 6.78 m/s" }, { y: (TRK.P / 59) * (1 + 1.2 / TRK.R1), label: "×(R+1.2)/R = 7.00 m/s" }], xFmtTip: (v) => `${v} m into the lap`, yFmtTip: (v) => `${fmt(v, 2)} m/s` });
  }
}

async function renderMissionFinding() {
  if (!DATA.byId.mission59) return;
  const b1 = Math.PI * TRK.R1;
  const prof = async (id) => { const d = await replayOf(id); const r = DATA.byId[id]; const n = 100, acc = new Array(n).fill(0), cnt = new Array(n).fill(0); const end = r.crossings_s[r.crossings_s.length - 1]; d.t.forEach((t, i) => { if (t < 0 || t > end) return; const s = ((d.s[i] % TRK.P) + TRK.P) % TRK.P; const k = Math.min(n - 1, Math.floor(s / 4)); acc[k] += d.speed[i]; cnt[k]++; }); return acc.map((a, k) => (cnt[k] ? a / cnt[k] : null)); };
  const [a, m] = await Promise.all([prof("ff59"), prof("mission59")]); const xs = a.map((_, k) => k * 4 + 2);
  lineChart($("#fd-mission"), { width: 560, height: 250, xDomain: [0, 400], yDomain: [0, 8], yTicks: 4, bands: [[0, b1], [b1 + TRK.L, 2 * b1 + TRK.L]], xLabel: "distance into the lap (m)", yLabel: "ground speed (m/s)", series: [{ name: "offboard pacer", color: COL.c1, x: xs, y: a }, { name: "waypoint mission", color: COL.c4, x: xs, y: m }], xFmtTip: (v) => `${v} m into the lap`, yFmtTip: (v) => `${fmt(v, 2)} m/s` });
}

/* ---------- pace planner (port of trackpace/schedule.py) ---------- */
function pacePlan(laps, P, o) {
  let v = P / laps[0]; const aRun = Math.max(o.runupAccel, (v * v) / (2 * o.runupMax));
  const segs = [{ kind: "runup", lap: 0, t0: -v / aRun, s0: -(v * v) / (2 * aRun), v0: 0, a: aRun, dur: v / aRun }];
  let t = 0, s = 0; const lapInfo = [];
  for (let k = 0; k < laps.length; k++) {
    const T = laps[k]; let vp = P / T, tr = 0, rest = T;
    if (Math.abs(vp - v) > 1e-9) {
      for (let it = 0; it < 100; it++) { tr = Math.abs(vp - v) / o.rampAccel; if (tr >= T) return { error: `Lap ${k + 1}: the change ${v.toFixed(2)} → ${vp.toFixed(2)} m/s does not fit in ${T} s at ${o.rampAccel} m/s²` }; const n = (P - (tr * v) / 2) / (T - tr / 2); if (Math.abs(n - vp) < 1e-12) break; vp = n; }
      tr = Math.abs(vp - v) / o.rampAccel; const a = Math.sign(vp - v) * o.rampAccel;
      segs.push({ kind: "ramp", lap: k + 1, t0: t, s0: s, v0: v, a, dur: tr }); t += tr; s += v * tr + 0.5 * a * tr * tr; v = vp; rest = T - tr;
    }
    segs.push({ kind: "pace", lap: k + 1, t0: t, s0: s, v0: v, a: 0, dur: rest }); t += rest; s = (k + 1) * P;
    lapInfo.push({ lap: k + 1, target: T, avg: P / T, plateau: v, ramp: tr });
  }
  segs.push({ kind: "stop", lap: 0, t0: t, s0: s, v0: v, a: -o.stopDecel, dur: v / o.stopDecel });
  return { segs, lapInfo, runup: -segs[0].s0, end: t + v / o.stopDecel, vmax: Math.max(...segs.map((g) => Math.max(g.v0, g.v0 + g.a * g.dur))) };
}
function renderPlanner() {
  const val = (id) => $(id).value;
  const laps = val("#pl-laps").split(/[\s,;]+/).map(Number).filter((x) => x > 0);
  const lane = Math.max(1, Math.min(8, +val("#pl-lane") || 1)), off = +val("#pl-off") || 0, drag = +val("#pl-drag") || 0, vlim = +val("#pl-vmax") || 12, tilt = +val("#pl-tilt") || 45;
  const R = lane === 1 ? TRK.kerb + 0.3 : TRK.kerb + TRK.lw * (lane - 1) + 0.2, P = 2 * Math.PI * R + 2 * TRK.L;
  const out = $("#pl-out"), flags = $("#pl-flags");
  if (!laps.length) { flags.innerHTML = `<div class="flag bad">Enter at least one lap time.</div>`; return; }
  const plan = pacePlan(laps, P, { runupAccel: 0.5, runupMax: 80, rampAccel: +val("#pl-ramp") || 0.4, stopDecel: 1.0 });
  if (plan.error) { flags.innerHTML = `<div class="flag bad">${plan.error}</div>`; out.innerHTML = ""; return; }
  const g = 9.80665, k = 1 + off / R;
  const rows = plan.lapInfo.map((l) => { const w = l.plateau * k; const ac = Math.hypot(drag * w, (w * w) / (R + off)); const th = (Math.atan(ac / g) * 180) / Math.PI; return { ...l, w, ac, th }; });
  const wmax = plan.vmax * k, thmax = Math.max(...rows.map((r) => r.th));
  const F = [];
  F.push(wmax > 0.95 * vlim ? `<div class="flag bad">Fastest drone speed <b>${wmax.toFixed(2)} m/s</b> is above 95% of MPC_XY_VEL_MAX (${vlim}). Raise it or the plan is refused.</div>` : `<div class="flag ok">Fastest drone speed <b>${wmax.toFixed(2)} m/s</b> — inside MPC_XY_VEL_MAX ${vlim} m/s.</div>`);
  F.push(thmax > 0.85 * tilt ? `<div class="flag ${thmax > tilt ? "bad" : "warn"}">Bends need <b>${thmax.toFixed(1)}°</b> of tilt (drag + turn), against MPC_TILTMAX_AIR ${tilt}°. ${thmax > tilt ? "Not flyable at this pace with this drag." : "Little margin left for wind."}</div>` : `<div class="flag ok">Bends need <b>${thmax.toFixed(1)}°</b> of tilt (drag + turn) — limit ${tilt}°.</div>`);
  F.push(`<div class="flag ok">Run-up <b>${plan.runup.toFixed(1)} m</b> on the home straight (84.39 m long); the drone crosses the line at pace.</div>`);
  flags.innerHTML = F.join("");
  $("#pl-table tbody").innerHTML = rows.map((r) => `<tr><td>${r.lap}</td><td class="n">${r.target.toFixed(2)}</td><td class="n">${r.avg.toFixed(3)}</td><td class="n">${r.plateau.toFixed(3)}</td><td class="n">${r.ramp ? r.ramp.toFixed(1) + " s" : "—"}</td><td class="n">${r.w.toFixed(2)}</td><td class="n">${r.th.toFixed(1)}°</td></tr>`).join("");
  // speed vs time
  const xs = [], ys = []; for (let t = plan.segs[0].t0; t <= plan.end; t += 0.25) { const gi = plan.segs.findLast((s) => s.t0 <= t) || plan.segs[0]; const tau = t - gi.t0; xs.push(+t.toFixed(2)); ys.push(gi.v0 + gi.a * Math.min(tau, gi.dur)); }
  let cum = 0; const bands = laps.map((T, i) => { const b = [cum, cum + T, i % 2 ? "#F1ECE1" : "#E9E3D6"]; cum += T; return b; });
  lineChart($("#pl-chart"), { height: 260, xDomain: [xs[0], xs[xs.length - 1]], yDomain: [0, Math.ceil(wmax + 0.5)], bands, bandLabels: laps.map((T, i) => [bands[i][0], bands[i][1], `lap ${i + 1}`]), xLabel: "race time (s); 0 = start line", yLabel: "runner pace (m/s)", series: [{ name: "pace", color: COL.c1, x: xs, y: ys }], xFmtTip: (v) => `t = ${v.toFixed(1)} s`, yFmtTip: (v) => `${v.toFixed(2)} m/s` });
  const toml = `# generated by the pace planner\n[track]\nlane = ${lane}\noffset_m = ${off}\naltitude_m = 3.0\n\n[pace]\nlaps_s = [${laps.map((x) => x.toFixed(1)).join(", ")}]\nramp_accel = ${+val("#pl-ramp") || 0.4}\n\n[control]\nmode = "ff"\ndrag_per_s = ${drag}\n\n[px4]\nMPC_XY_VEL_MAX = ${vlim.toFixed(1)}`;
  $("#pl-toml").textContent = toml;
  $("#pl-total").textContent = `${laps.length} lap${laps.length > 1 ? "s" : ""} · ${(laps.length * P).toFixed(1)} m · ${clockStr(laps.reduce((a, b) => a + b, 0))}`;
}
function initPlanner() {
  $$("#planner input").forEach((i) => i.addEventListener("input", renderPlanner));
  $$("#pl-presets button").forEach((b) => b.onclick = () => { $("#pl-laps").value = b.dataset.v; renderPlanner(); });
  $("#pl-copy").onclick = () => { navigator.clipboard?.writeText($("#pl-toml").textContent); $("#pl-copy").textContent = "copied"; setTimeout(() => ($("#pl-copy").textContent = "copy"), 1200); };
  renderPlanner();
}

/* ---------- headline numbers ---------- */
function fillNumbers() {
  const r = DATA.byId.ff59, pos = DATA.byId.pos59, nd = DATA.byId.nodrag59, gz = DATA.byId.gz59, ms = DATA.byId.mission59, cal = DATA.byId.gz_cal;
  const maxAbs = (a) => Math.max(...a.map(Math.abs));
  const set = (k, v) => $$(`[data-n="${k}"]`).forEach((e) => (e.textContent = v));
  set("ff-laps", r.lap_times_s.length); set("ff-lap-max", Math.round(maxAbs(r.lap_error_ms))); set("ff-cross-max", Math.round(maxAbs(r.crossing_error_ms)));
  set("ff-gap-rms", fmt(r.along_track_m.rms, 2)); set("ff-xt-rms", fmt(r.cross_track_m.all.rms, 2)); set("ff-alt-rms", fmt(r.altitude_m.rms, 2));
  set("ff-stream", fmt(r.setpoint_stream.mean_interval_ms, 1)); set("ff-gapmax", Math.round(r.setpoint_stream.max_gap_ms)); set("ff-spread", fmt(r.repeatability.path_std_mean_m, 2));
  set("ff-xt-str", fmt(r.cross_track_m.straights.rms, 2)); set("ff-xt-bend", fmt(r.cross_track_m.bends.rms, 2)); set("ff-xt-junc", fmt(r.cross_track_m.junction_5m.rms, 2));
  set("ff-spd-std", fmt(r.speed_m_s.std, 2)); set("ff-spd-mean", fmt(r.speed_m_s.mean, 2));
  if (pos) { const ce = pos.crossing_error_ms.slice(1); set("pos-late", fmt(ce.reduce((a, b) => a + b, 0) / ce.length / 1000, 2)); set("pos-gap", fmt(-pos.along_track_m.mean, 1)); set("pos-lap-max", Math.round(maxAbs(pos.lap_error_ms))); }
  if (nd) { set("nd-wide", fmt(-nd.cross_track_m.bend_mean_inward_m, 2)); set("nd-behind", fmt(-nd.along_track_m.mean, 2)); set("nd-lap-max", Math.round(maxAbs(nd.lap_error_ms))); }
  if (ms) { set("ms-laps", ms.lap_times_s.map((x) => fmt(x, 2)).join(" s, ") + " s"); set("ms-xt", fmt(ms.cross_track_m.all.rms, 2)); set("ms-bend-in", fmt(ms.cross_track_m.bend_mean_inward_m, 2)); set("ms-spd-bend", fmt(ms.speed_m_s.bends_mean, 2)); set("ms-spd-str", fmt(ms.speed_m_s.straights_mean, 2)); set("ms-lap-max", fmt(maxAbs(ms.lap_error_ms) / 1000, 2)); }
  if (gz) { set("gz-lap-max", Math.round(maxAbs(gz.lap_error_ms))); set("gz-cross-max", Math.round(maxAbs(gz.crossing_error_ms))); set("gz-gap-rms", fmt(gz.along_track_m.rms, 2)); set("gz-xt-rms", fmt(gz.cross_track_m.all.rms, 2)); set("gz-drag", fmt(gz.drag_per_s, 3)); set("gz-spread", fmt(gz.repeatability.path_std_mean_m, 2)); set("gz-alt-rms", fmt(gz.altitude_m.rms, 2)); }
  if (cal) { set("cal-drag", fmt(cal.identified_drag_per_s, 3)); }
  const gzd = DATA.byId.gz59_gpsdelay, mean = (a) => a.reduce((x, y) => x + y, 0) / a.length;
  if (gzd) { set("gzd-lap-max", Math.round(maxAbs(gzd.lap_error_ms))); set("gzd-cross-mean", Math.round(mean(gzd.crossing_error_ms.slice(1)))); set("gzd-ekf", fmt(gzd.estimate_vs_truth.along_mean_m, 2)); set("gzd-ekf-ms", Math.round(gzd.estimate_vs_truth.along_mean_ms)); }
  if (gz) { set("gz-ekf-ms", Math.round(gz.estimate_vs_truth.along_mean_ms)); set("gz-cross-mean", sgn(mean(gz.crossing_error_ms.slice(1)), 0)); }
  const pl = DATA.byId.plan; if (pl) set("plan-lap-max", Math.round(maxAbs(pl.lap_error_ms)));
  const l2 = DATA.byId.lane2; if (l2) { set("l2-gap-rms", fmt(l2.along_track_m.rms, 2)); set("l2-bend", fmt(l2.speed_m_s.bends_mean, 2)); }
  $$("[data-if]").forEach((e) => { if (!DATA.byId[e.dataset.if]) e.remove(); });
}

async function main() {
  initTabs();
  DATA.summary = await loadJSON("data/summary.json");
  DATA.summary.forEach((r) => (DATA.byId[r.id] = r));
  fillNumbers();
  initReplay();
  const tmIds = ["ff59", "gz59", "plan", "lane2"].filter((i) => DATA.byId[i]);
  tmIds.forEach((id) => { const b = document.createElement("button"); b.dataset.id = id; b.innerHTML = `${DATA.byId[id].label}<small>${DATA.byId[id].sim}</small>`; b.onclick = () => renderTiming(id); $("#tm-runs").appendChild(b); });
  ["gz59", "ff59"].filter((i) => DATA.byId[i]).forEach((id) => { const b = document.createElement("button"); b.dataset.id = id; b.innerHTML = `${DATA.byId[id].label}<small>${DATA.byId[id].sim}</small>`; b.onclick = () => renderPath(id); $("#pt-runs").appendChild(b); });
  if (!DATA.byId[TM_RUN]) TM_RUN = "ff59";
  RENDER.timing = () => { renderTiming(); renderMethods(); };
  RENDER.path = renderPath; RENDER.findings = renderMissionFinding; RENDER.planner = renderPlanner;
  initPlanner();
  let rz; window.addEventListener("resize", () => { clearTimeout(rz); rz = setTimeout(() => RENDER[CUR_TAB] && RENDER[CUR_TAB](), 150); });
  RENDER[CUR_TAB] && RENDER[CUR_TAB]();
}
if (!document.body.dataset.print) main().catch((e) => { console.error(e); document.body.insertAdjacentHTML("afterbegin", `<p style="padding:12px;background:#F8E1DE">Data failed to load (${e.message}). Serve this folder over HTTP.</p>`); });
