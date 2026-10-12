import { getState, getSystem, sendCmd } from "./api";
import type { State, System } from "./types";

export function mountDashboard(): void {
const POLL_MS = 2000;
const GEAR = { F: "Forward", R: "Reverse", N: "Neutral" } as const;
const SRC = ["CAN", "MQTT", "HTTP"] as const;
const HTTP_SRC = 2;          // CtrlSource::CTRL_HTTP
const HOLD_MS = 300;         // firmware drops a command after 500 ms without a refresh

const $ = <T extends HTMLElement>(id: string) => document.getElementById(id) as T;
const fmt = (n: number, d = 2) => n.toFixed(d);
const hms = (s: number) =>
  [Math.floor(s / 3600), Math.floor((s % 3600) / 60), s % 60].map((v) => String(v).padStart(2, "0")).join(":");

const rows = (items: [string, string, string?][]) =>
  items.map(([label, id, unit]) =>
    `<div class="row"><span>${label}</span><b id="${id}">–</b>${unit ? `<i>${unit}</i>` : "<i></i>"}</div>`).join("");

document.body.innerHTML = `
<header>
  <h1>HDV Simulator</h1>
  <span id="status" class="pill off">OFFLINE</span>
  <span class="meta">Uptime <b id="uptime">–</b></span>
  <nav><a href="/dash">Dashboard</a><a href="/ota">OTA</a><a href="/logout">Logout</a></nav>
</header>
<main>
  <section class="card span2">
    <h2>Vehicle motion</h2>
    <div class="kpis">
      <div><b id="k-speed">0.0</b><i>km/h</i><span>Speed</span></div>
      <div><b id="k-soc">–</b><i>%</i><span>State of charge</span><div class="bar"><u id="soc-bar"></u></div></div>
      <div><b id="k-yawr">0.0</b><i>°/s</i><span>Yaw rate</span></div>
    </div>
    <div class="cols">
      <div>${rows([["vx", "s-vx", "m/s"], ["vy", "s-vy", "m/s"], ["yaw", "s-yaw", "°"], ["x", "s-x", "m"], ["y", "s-y", "m"], ["steer", "s-steer", "°"]])}</div>
      <div>${rows([["ω FL", "s-ofl", "rad/s"], ["ω FR", "s-ofr", "rad/s"], ["ω RL", "s-orl", "rad/s"], ["ω RR", "s-orr", "rad/s"], ["surface μ", "s-mu"]])}</div>
    </div>
  </section>

  <section class="card">
    <h2>Actuator command</h2>
    ${rows([["Enable", "c-en"], ["Gear", "c-gear"], ["Torque", "c-torq", "Nm"], ["Brake", "c-brk", "%"], ["Steer", "c-steer", "°"]])}
    <h2 class="sub">CAN</h2>
    ${rows([["TX frames", "can-tx"], ["RX frames", "can-rx"], ["Timeouts", "can-to"], ["Last RX", "can-lrx", "s"]])}
  </section>

  <section class="card span2">
    <h2>Controls</h2>
    <div class="btns">
      <button class="danger" data-act="stop">■ Stop</button>
      <button class="ok" data-act="fwd">▶ Drive FWD</button>
      <button class="info" data-act="rev">◀ Drive REV</button>
      <button data-act="left">← −5°</button>
      <button data-act="right">5° →</button>
    </div>
    <form id="cmd" class="form">
      <label>Steer °<input name="steer" type="number" value="0" step="0.5" /></label>
      <label>Torque Nm<input name="torque" type="number" value="0" /></label>
      <label>Brake %<input name="brake" type="number" value="0" min="0" max="100" /></label>
      <label>Gear<select name="gear"><option value="F">Forward</option><option value="R">Reverse</option><option value="N">Neutral</option></select></label>
      <label class="chk"><input name="enable" type="checkbox" /> Enable</label>
      <button class="info" type="submit">Inject command →</button>
    </form>
    <p class="hint" id="cmd-msg">Select the HTTP source to send commands.</p>
  </section>

  <section class="card">
    <h2>Control source</h2>
    <div class="seg" id="src">${SRC.map((s, i) => `<button data-src="${i}">${s}</button>`).join("")}</div>
    <p class="hint">Active source writes g_cmd. Others decode but discard.</p>
    <h2 class="sub">MQTT broker</h2>
    <form id="mqtt" class="form">
      <label>IP<input name="broker_addr" value="192.168.1.106" /></label>
      <label>Port<input name="broker_port" type="number" value="8883" /></label>
      <button type="submit">Apply</button>
    </form>
    <div class="row"><span>Status</span><b id="mqtt-st">–</b><i></i></div>
    <div class="row"><span>MQTT RX</span><b id="mqtt-rx">–</b><i></i></div>
  </section>

  <section class="card span3">
    <h2>System</h2>
    <div class="row"><span>Heap</span><b id="res-hu">–</b><i></i></div>
    <div class="bar"><u id="heap-bar"></u></div>
    <div class="row"><span>Heap free</span><b id="res-hf">–</b><i></i></div>
    <div class="row"><span>Sim loop max</span><b id="res-loop">–</b><i>ms</i></div>
  </section>

  <section class="card span2">
    <h2>Kernel threads</h2>
    <table id="threads"><thead><tr><th>Name</th><th>Prio</th><th>Stack used / total</th><th></th></tr></thead><tbody></tbody></table>
  </section>

  <section class="card">
    <h2>Memory map</h2>
    <div class="row"><span>Flash (slot0)</span><b id="m-flash">–</b><i></i></div><div class="bar"><u id="m-flash-bar"></u></div>
    <div class="row"><span>RAM (AXI SRAM)</span><b id="m-ram">–</b><i></i></div><div class="bar"><u id="m-ram-bar"></u></div>
    <div class="row sm"><span>General heap pool</span><b id="m-heap">–</b><i></i></div>
    <div class="row sm"><span>mbedTLS heap pool</span><b id="m-tls">–</b><i></i></div>
    <div class="row sm"><span>Thread stacks</span><b id="m-stacks">–</b><i></i></div>
    <div class="row sm"><span>Kernel + globals</span><b id="m-other">–</b><i></i></div>
    <h2 class="sub">Vehicle</h2>
    ${rows([["Mass", "v-mass"], ["Battery", "v-batt"], ["Motor power", "v-pwr"], ["Motor torque", "v-trq"], ["Max speed", "v-vmax"], ["Gear ratio", "v-gr"]])}
  </section>
</main>`;

// Static vehicle spec (unchanged from the old dashboard).
const VEHICLE: Record<string, string> = {
  "v-mass": "218 000 kg", "v-batt": "1 650 kWh", "v-pwr": "2 013 kW",
  "v-trq": "145 000 Nm", "v-vmax": "17.8 m/s (64 km/h)", "v-gr": "28.0",
};
for (const [id, v] of Object.entries(VEHICLE)) document.getElementById(id)!.textContent = v;

const set = (id: string, v: string | number, cls = "") => {
  const e = $(id);
  e.textContent = String(v);
  e.className = cls;
};

let lastSrc = -1;

function render(d: State): void {
  lastSrc = d.src;
  const s = $("status");
  s.textContent = "ONLINE";
  s.className = "pill on";
  set("uptime", hms(d.uptime));
  $("k-speed").textContent = fmt(Math.abs(d.vx) * 3.6, 1);
  $("k-soc").textContent = fmt(d.soc, 1);
  $("k-yawr").textContent = fmt(d.yawr, 1);
  $("soc-bar").style.width = `${Math.max(0, Math.min(100, d.soc))}%`;
  set("s-vx", fmt(d.vx, 3)); set("s-vy", fmt(d.vy, 3)); set("s-yaw", fmt(d.yaw));
  set("s-x", fmt(d.x)); set("s-y", fmt(d.y)); set("s-steer", fmt(d.steer));
  set("s-ofl", fmt(d.ofl)); set("s-ofr", fmt(d.ofr)); set("s-orl", fmt(d.orl)); set("s-orr", fmt(d.orr));
  set("s-mu", fmt(d.mu));
  set("c-en", d.en ? "ON" : "OFF", d.en ? "hi" : "warn");
  set("c-gear", GEAR[d.gear] ?? d.gear);
  set("c-torq", fmt(d.torq, 1)); set("c-brk", fmt(d.brk)); set("c-steer", fmt(d.csteer));
  set("can-tx", d.cantx); set("can-rx", d.canrx);
  set("can-to", d.canto, d.canto ? "warn" : ""); set("can-lrx", fmt(d.lrx, 3));
  const hp = d.htot > 0 ? Math.round((100 * d.hu) / d.htot) : 0;
  set("res-hu", `${d.hu} B / ${d.htot} B (${hp}%)`, hp >= 80 ? "warn" : "hi");
  $("heap-bar").style.width = `${hp}%`;
  set("res-hf", `${d.hf} B`);
  set("res-loop", fmt(d.loop / 1000, 1), d.loop > 10000 ? "warn" : "hi");
  set("mqtt-st", d.mqc ? "connected" : "disconnected", d.mqc ? "hi" : "warn");
  set("mqtt-rx", d.mqrx);
  document.querySelectorAll<HTMLButtonElement>("#src button").forEach((b) =>
    b.classList.toggle("active", Number(b.dataset.src) === d.src));
}

const kb = (n: number) => `${Math.round(n / 1024)} KB`;
const pctCls = (p: number) => (p >= 80 ? "warn" : p >= 60 ? "" : "hi");

function renderSystem(s: System): void {
  const fp = Math.round((100 * s.flash_used) / s.flash_total);
  const rp = Math.round((100 * s.ram_image) / s.ram_total);
  set("m-flash", `${kb(s.flash_used)} / ${kb(s.flash_total)} (${fp}%)`, pctCls(fp));
  set("m-ram", `${kb(s.ram_image)} / ${kb(s.ram_total)} (${rp}%)`, pctCls(rp));
  $("m-flash-bar").style.width = `${fp}%`;
  $("m-ram-bar").style.width = `${rp}%`;
  set("m-heap", kb(s.heap_gen)); set("m-tls", kb(s.heap_tls)); set("m-stacks", kb(s.stacks));
  set("m-other", kb(Math.max(0, s.ram_image - s.heap_gen - s.heap_tls - s.stacks)));
  const body = document.querySelector<HTMLTableSectionElement>("#threads tbody")!;
  body.replaceChildren(...s.threads.map((t) => {
    const p = t.t ? Math.round((100 * t.u) / t.t) : 0;
    const tr = document.createElement("tr");
    for (const text of [t.n, String(t.p), `${t.u} / ${t.t} B`, `${p}%`]) {
      const td = document.createElement("td");
      td.textContent = text;
      tr.append(td);
    }
    tr.children[2].className = tr.children[3].className = pctCls(p);
    return tr;
  }));
  // Fill the broker form once; don't clobber what the user is typing.
  const f = $<HTMLFormElement>("mqtt");
  if (!f.dataset.init) {
    (f.elements.namedItem("broker_addr") as HTMLInputElement).value = s.broker_addr;
    (f.elements.namedItem("broker_port") as HTMLInputElement).value = String(s.broker_port);
    f.dataset.init = "1";
  }
}

async function pollSystem(): Promise<void> {
  try { renderSystem(await getSystem()); } catch { /* shown as offline by poll() */ }
  setTimeout(pollSystem, 5000);
}

async function poll(): Promise<void> {
  try {
    render(await getState());
  } catch {
    const s = $("status");
    s.textContent = "OFFLINE";
    s.className = "pill off";
  }
  setTimeout(poll, POLL_MS);
}

const act: Record<string, Cmd> = {
  stop: { enable: 0, torque: 0, brake: 100 },
  fwd: { enable: 1, gear: "F", torque: 20000, brake: 0 },
  rev: { enable: 1, gear: "R", torque: 20000, brake: 0 },
  left: { steer: -5 },
  right: { steer: 5 },
};

// The firmware (plant_thread.cpp) resets the actuator command if it isn't refreshed
// within 500 ms, so an injected command is held by resending it every HOLD_MS.
// Only one request is in flight at a time (each is a fresh TLS connection).
type Cmd = Record<string, string | number>;
let held: Cmd | null = null;
let inflight = false;

function cmdMsg(text: string, cls = ""): void {
  const m = $("cmd-msg");
  m.textContent = text;
  m.className = `hint ${cls}`;
}

function inject(c: Cmd): void {
  if (lastSrc !== HTTP_SRC) {
    cmdMsg("Control source is not HTTP — select HTTP first, otherwise the board discards this command.", "warn");
    return;
  }
  held = { ...held, ...c };
  cmdMsg(`Holding command, resent every ${HOLD_MS} ms: ${JSON.stringify(held)}`, "hi");
  void sendHeld();
}

async function sendHeld(): Promise<void> {
  if (!held || inflight) return;
  inflight = true;
  try { await sendCmd(held); } catch { /* next tick retries */ }
  inflight = false;
}

setInterval(() => {
  if (held && lastSrc !== HTTP_SRC) {
    held = null;
    cmdMsg("Source changed away from HTTP — command released.", "warn");
  } else void sendHeld();
}, HOLD_MS);

document.addEventListener("click", (e) => {
  const t = e.target as HTMLElement;
  const a = t.dataset.act;
  if (a === "stop") {
    // Brake once, then stop refreshing: the watchdog then resets the command.
    held = null;
    if (lastSrc === HTTP_SRC) void sendCmd(act.stop);
    cmdMsg("Stopped. Command released.", "");
  } else if (a) inject(act[a]);
  if (t.dataset.src) {
    held = null;
    void sendCmd({ ctrl: SRC[Number(t.dataset.src)].toLowerCase() });
    if (SRC[Number(t.dataset.src)] === "HTTP") cmdMsg("HTTP source selected. Inject a command to drive.", "");
    else cmdMsg("Select the HTTP source to send commands.", "");
  }
});
$<HTMLFormElement>("cmd").addEventListener("submit", (e) => {
  e.preventDefault();
  const f = new FormData(e.target as HTMLFormElement);
  inject({
    steer: String(f.get("steer")), torque: String(f.get("torque")), brake: String(f.get("brake")),
    gear: String(f.get("gear")), enable: f.get("enable") ? 1 : 0,
  });
});
$<HTMLFormElement>("mqtt").addEventListener("submit", (e) => {
  e.preventDefault();
  const f = new FormData(e.target as HTMLFormElement);
  void sendCmd({ broker_addr: String(f.get("broker_addr")), broker_port: String(f.get("broker_port")) });
});

void poll();
void pollSystem();
}
