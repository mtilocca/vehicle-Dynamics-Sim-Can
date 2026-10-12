import type { State, System } from "./types";

export async function getState(): Promise<State> {
  const r = await fetch("/api/state", { credentials: "include" });
  if (r.status === 401) {
    location.href = "/";
    throw new Error("unauthorized");
  }
  if (!r.ok) throw new Error(`HTTP ${r.status}`);
  return r.json();
}

export async function getSystem(): Promise<System> {
  const r = await fetch("/api/system", { credentials: "include" });
  if (!r.ok) throw new Error(`HTTP ${r.status}`);
  return r.json();
}

// The firmware applies commands from the /dash query string (apply_web_cmd).
export async function sendCmd(params: Record<string, string | number>): Promise<void> {
  const qs = new URLSearchParams(Object.entries(params).map(([k, v]) => [k, String(v)]));
  await fetch(`/dash?${qs}`, { credentials: "include" });
}
