/* 2-page PDF: same data and chart code as the review site. */
"use strict";
(async function () {
  const base = "../site/data/";
  DATA.summary = await loadJSON(base + "summary.json");
  DATA.summary.forEach((r) => (DATA.byId[r.id] = r));
  const rep = async (id) => (DATA.replay[id] = DATA.replay[id] || (await loadJSON(base + `replay_${id}.json`)));
  await Promise.all(DATA.summary.map((r) => rep(r.id)));
  fillNumbers();
  const ff = DATA.byId.ff59, gz = DATA.byId.gz59, hero = gz || ff, d = DATA.replay[hero.id];
  const maxAbs = (a) => Math.max(...a.map(Math.abs));

  // track figure: whole track + 5 laps of true path, inset zoom at the bend-1 exit
  const cv = $("#pf-track"), ctx = cv.getContext("2d");
  const T = hero.laps_target_s.reduce((a, b) => a + b, 0);
  const drawPath = (k, color, dd, Tend) => { ctx.strokeStyle = color; ctx.lineWidth = 1.6 / k; ctx.beginPath(); let pen = false; dd.t.forEach((t, i) => { if (t < 0 || t > Tend) { pen = false; return; } if (!pen) { ctx.moveTo(dd.x[i], dd.y[i]); pen = true; } else ctx.lineTo(dd.x[i], dd.y[i]); }); ctx.stroke(); };
  const ext = TRK.L / 2 + TRK.kerb + TRK.lanes * TRK.lw + 5;
  const k = Math.min(cv.width / (2 * ext), cv.height / (2 * (TRK.kerb + TRK.lanes * TRK.lw + 8)));
  ctx.setTransform(k, 0, 0, -k, cv.width / 2, cv.height / 2);
  drawTrack(ctx, k / 2.2);
  drawPath(k, "#1C1F24", d, T);
  // inset
  const [sx, sy] = trackPoint(Math.PI * TRK.R1 + 2, TRK.R1);   // just after bend 1 joins the back straight
  const iw = 520, ih = 300, ix = cv.width / 2 - iw / 2, iy = cv.height / 2 - ih / 2 + 6, kz = k * 9;
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.fillStyle = "#FFFDF8"; ctx.fillRect(ix - 4, iy - 4, iw + 8, ih + 8);
  ctx.save(); ctx.beginPath(); ctx.rect(ix, iy, iw, ih); ctx.clip();
  ctx.setTransform(kz, 0, 0, -kz, ix + iw / 2 - sx * kz, iy + ih / 2 + sy * kz);
  drawTrack(ctx, kz / 2.2);
  ctx.strokeStyle = "rgba(255,240,170,1)"; ctx.lineWidth = 3 / kz; ctx.setLineDash([]);
  drawPath(kz, "#1C1F24", d, T);
  ctx.restore();
  ctx.setTransform(1, 0, 0, 1, 0, 0); ctx.strokeStyle = "#1C1F24"; ctx.lineWidth = 2; ctx.strokeRect(ix, iy, iw, ih);
  ctx.fillStyle = "rgba(255,253,248,.92)"; ctx.fillRect(ix, iy, iw, 44); ctx.fillStyle = "#1C1F24"; ctx.font = "700 34px Barlow Condensed, sans-serif"; ctx.fillText(`ZOOM ×9 · ${hero.lap_times_s.length} LAPS · BEND 1 EXIT`, ix + 12, iy + 33);
  const px = 2 * kz; ctx.fillStyle = "rgba(255,253,248,.92)"; ctx.fillRect(ix, iy + ih - 44, px + 90, 44); ctx.fillStyle = "#1C1F24"; ctx.fillRect(ix + 12, iy + ih - 24, px, 6); ctx.font = "600 28px JetBrains Mono, monospace"; ctx.fillText("2 m", ix + 20 + px, iy + ih - 14);
  $("#pf-track-cap").textContent = `True path of all ${hero.lap_times_s.length} laps at 59.0 s (${hero.sim}) over the track. Dashed: lane-1 measurement line (0.30 m outside the kerb), where pace is defined. ${fmt(hero.cross_track_m.all.rms, 2)} m RMS off the line; lap-to-lap spread ${fmt(hero.repeatability.path_std_mean_m, 2)} m; lanes are 1.22 m wide.`;

  // KPIs
  const kp = [
    [`±${Math.round(maxAbs(hero.lap_error_ms))} ms`, `worst of ${hero.lap_times_s.length} laps at 59.0 s, ${hero.sim}`],
    [`±${Math.round(maxAbs(hero.crossing_error_ms))} ms`, "worst start/finish-line crossing vs the pace clock"],
    [`${fmt(hero.along_track_m.rms, 2)} m`, "RMS gap to the virtual runner, whole race"],
    [`${fmt(hero.cross_track_m.all.rms, 2)} m`, "RMS off the lane-1 line (a lane is 1.22 m)"],
  ];
  if (gz) kp.push([`±${Math.round(maxAbs(ff.lap_error_ms))} ms`, `worst of ${ff.lap_times_s.length} laps in PX4's SIH simulator, same code`]);
  $("#pf-kpis").innerHTML = kp.map(([b, s]) => `<div class="kpi"><b>${b}</b><span>${s}</span></div>`).join("");

  // splits
  const rows = [];
  [["gz59", "Gazebo"], ["ff59", "SIH"]].forEach(([id, nm]) => { const r = DATA.byId[id]; if (!r) return; r.lap_times_s.forEach((lt, i) => rows.push(`<tr><td>${i === 0 ? nm : ""}</td><td>${i + 1}</td><td class="n">${fmt(r.laps_target_s[i], 2)}</td><td class="n">${fmt(lt, 3)}</td><td class="n">${sgn(r.lap_error_ms[i], 0)} ms</td><td class="n">${sgn(r.crossing_error_ms[i + 1], 0)} ms</td></tr>`)); });
  $("#pf-splits tbody").innerHTML = rows.join("");

  const idx = d.t.map((t, i) => (t >= 0 && t <= T ? i : -1)).filter((i) => i >= 0);
  lineChart($("#pf-gap"), { height: 168, xDomain: [0, T], yDomain: [-1, 1], yTicks: 4, margin: { l: 38, b: 40 }, xLabel: "race time (s)", yLabel: "m ahead (+) / behind (−)", bands: hero.crossings_s.slice(1, -1).map((c) => [c - 0.3, c + 0.3, "#E4DDCD"]), series: [{ name: "gap", color: COL.c1, x: idx.map((i) => d.t[i]), y: idx.map((i) => d.along[i]), width: 1.3, hover: false }] });
  $("#pf-gap-cap").textContent = `RMS ${fmt(hero.along_track_m.rms, 2)} m, worst ${fmt(hero.along_track_m.max_abs, 2)} m. Shaded: line crossings.`;

  // methods bars (offboard variants)
  const nm = { ff59: "pos + vel + acc + drag", nodrag59: "pos + vel + acc", pos59: "position only" }, cl = { ff59: COL.c1, nodrag59: COL.c3, pos59: COL.c2 };
  barChart($("#pf-bars"), { labelW: 128, rowH: 30, marginR: 58, ticks: 2, rows: ["ff59", "nodrag59", "pos59"].filter((i) => DATA.byId[i]).map((id) => { const w = maxAbs(DATA.byId[id].crossing_error_ms.slice(1)); return { label: nm[id], v: w, color: cl[id], text: w >= 1000 ? (w / 1000).toFixed(2) + " s" : Math.round(w) + " ms" }; }), fmt: (v) => (v >= 1000 ? v / 1000 + " s" : v) });
  // mission speed profile
  const b1 = Math.PI * TRK.R1;
  const prof = (id) => { const dd = DATA.replay[id], r = DATA.byId[id]; const n = 100, acc = new Array(n).fill(0), cnt = new Array(n).fill(0); const end = r.crossings_s[r.crossings_s.length - 1]; dd.t.forEach((t, i) => { if (t < 0 || t > end) return; const s = ((dd.s[i] % TRK.P) + TRK.P) % TRK.P; const q = Math.min(n - 1, Math.floor(s / 4)); acc[q] += dd.speed[i]; cnt[q]++; }); return acc.map((a, q) => (cnt[q] ? a / cnt[q] : null)); };
  if (DATA.byId.mission59) { const xs = Array.from({ length: 100 }, (_, q) => q * 4 + 2); lineChart($("#pf-mission"), { height: 175, xDomain: [0, 400], yDomain: [0, 8], yTicks: 4, margin: { l: 30, b: 40 }, bands: [[0, b1], [b1 + TRK.L, 2 * b1 + TRK.L]], xLabel: "m into the lap", yLabel: "m/s", series: [{ name: "pacer", color: COL.c1, x: xs, y: prof("ff59"), hover: false }, { name: "mission", color: COL.c4, x: xs, y: prof("mission59"), hover: false }] }); }
  const gzRows = [["gz_cal", "calibration lap"], ["gz59", "paced laps"]].filter(([id]) => DATA.byId[id]).map(([id, nm]) => { const r = DATA.byId[id]; const ce = r.crossing_error_ms.slice(1); return `<tr><td>${nm}</td><td class="n">${fmt(r.drag_per_s, 3)}</td><td class="n">${r.lap_times_s.length}</td><td class="n">${sgn(r.lap_error_ms.reduce((m, v) => (Math.abs(v) > Math.abs(m) ? v : m), 0), 0)} ms</td><td class="n">${sgn(ce.reduce((a, b) => a + b, 0) / ce.length, 0)} ms</td><td class="n">${fmt(r.along_track_m.rms, 2)} m</td><td class="n">${fmt(r.cross_track_m.all.rms, 2)} m</td><td class="n">${fmt(r.altitude_m.rms, 2)} m</td></tr>`; });
  if ($("#pf-gz")) $("#pf-gz tbody").innerHTML = gzRows.join("");
  document.body.dataset.ready = "1";
})().catch((e) => { document.body.insertAdjacentHTML("afterbegin", `<pre>${e.stack}</pre>`); document.body.dataset.ready = "1"; });
