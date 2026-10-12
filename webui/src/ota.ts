import { getState } from "./api";

export function mountOta(): void {
const SLOT_MAX = 640 * 1024; // slot1_partition (http_ota.cpp)

document.body.innerHTML = `
<header>
  <h1>OTA Firmware Update</h1>
  <nav><a href="/dash">← Dashboard</a><a href="/logout">Logout</a></nav>
</header>
<main class="narrow">
  <section class="card">
    <h2>Upload signed image</h2>
    <p class="hint">Select <code>zephyr.signed.bin</code>. MCUboot verifies the signature before booting
      and reverts to the current image if the new one fails. Max ${SLOT_MAX / 1024} KB.</p>
    <label class="drop" id="drop">
      <input id="file" type="file" accept=".bin" hidden />
      <span id="fname">Drop file here or click to browse</span>
    </label>
    <div class="bar tall"><u id="bar"></u></div>
    <p id="msg" class="hint"></p>
    <div class="btns">
      <button class="ok" id="up" disabled>Upload firmware</button>
      <button class="info" id="rb" hidden>Reboot &amp; apply</button>
    </div>
  </section>
</main>`;

const el = <T extends HTMLElement>(id: string) => document.getElementById(id) as T;
const file = el<HTMLInputElement>("file");
const up = el<HTMLButtonElement>("up");
const rb = el<HTMLButtonElement>("rb");
const bar = el("bar");
let picked: File | null = null;
let countdown = 0;

function msg(text: string, cls = ""): void {
  const m = el("msg");
  m.textContent = text;
  m.className = `hint ${cls}`;
}

function pick(f: File | undefined): void {
  picked = f ?? null;
  up.disabled = !picked;
  el("fname").textContent = picked ? `${picked.name} (${Math.round(picked.size / 1024)} KB)` : "Drop file here or click to browse";
  if (picked && picked.size > SLOT_MAX) {
    msg(`Image is larger than the ${SLOT_MAX / 1024} KB slot.`, "warn");
    up.disabled = true;
  } else msg("");
}

file.addEventListener("change", () => pick(file.files?.[0]));
const drop = el("drop");
drop.addEventListener("dragover", (e) => { e.preventDefault(); drop.classList.add("over"); });
drop.addEventListener("dragleave", () => drop.classList.remove("over"));
drop.addEventListener("drop", (e) => {
  e.preventDefault();
  drop.classList.remove("over");
  pick(e.dataTransfer?.files[0]);
});

up.addEventListener("click", () => {
  if (!picked) return;
  up.disabled = true;
  bar.style.background = "";
  bar.style.width = "0";
  msg(`Uploading ${picked.name}…`);
  const x = new XMLHttpRequest();
  x.open("POST", "/api/firmware");
  x.setRequestHeader("Content-Type", "application/octet-stream");
  x.upload.onprogress = (e) => {
    if (!e.lengthComputable) return;
    const pct = Math.round((e.loaded / e.total) * 100);
    bar.style.width = `${pct}%`;
    if (pct === 100) msg("File sent — writing to flash, please wait…", "warn");
  };
  x.onload = () => {
    up.disabled = false;
    if (x.status === 200) {
      bar.style.background = "var(--ok)";
      startCountdown(JSON.parse(x.responseText).bytes as number);
    } else if (x.status === 401) {
      bar.style.background = "var(--bad)";
      msg("Session expired — log in again, then retry.", "warn");
    } else {
      bar.style.background = "var(--bad)";
      msg(`Upload failed (HTTP ${x.status}): ${x.responseText}`, "warn");
    }
  };
  x.onerror = () => {
    up.disabled = false;
    msg("Network error — board may have rebooted or the session expired.", "warn");
  };
  x.send(picked);
});

function startCountdown(bytes: number): void {
  let n = 5;
  rb.hidden = false;
  rb.textContent = "Cancel reboot";
  const tick = (): void => {
    msg(`Upload OK — ${bytes} bytes written. Rebooting in ${n} s…`, "hi");
    if (n-- === 0) return void reboot();
    countdown = window.setTimeout(tick, 1000);
  };
  rb.onclick = () => {
    if (countdown) {
      clearTimeout(countdown);
      countdown = 0;
      rb.textContent = "Reboot & apply";
      msg("Reboot cancelled. Click “Reboot & apply” when ready.", "warn");
    } else void reboot();
  };
  tick();
}

async function reboot(): Promise<void> {
  countdown = 0;
  rb.disabled = true;
  msg("Rebooting… waiting for the board to come back (MCUboot swap can take ~30 s).", "warn");
  try { await fetch("/api/reboot", { method: "POST" }); } catch { /* connection drops — expected */ }
  await new Promise((r) => setTimeout(r, 8000));
  for (let i = 0; i < 40; i++) {
    try {
      await getState();
      msg("Board is back online. Redirecting…", "hi");
      setTimeout(() => (location.href = "/dash"), 1000);
      return;
    } catch { await new Promise((r) => setTimeout(r, 3000)); }
  }
  msg("Board did not come back within 2 minutes — check the device.", "warn");
}
}
