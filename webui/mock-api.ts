import type { Plugin } from "vite";

// Minimal stand-in for the board: same JSON shape as GET /api/state.
export function mockApi(): Plugin {
  let t = 0;
  const cmd = { en: 0, gear: "F", torq: 0, brk: 0, csteer: 0, src: 0 };
  return {
    name: "hdv-mock-api",
    configureServer(server) {
      server.middlewares.use((req, res, next) => {
        const url = new URL((req as { url?: string }).url ?? "/", "http://x");
        if (url.pathname === "/dash" && url.search) {
          const p = url.searchParams;
          if (p.has("torque")) cmd.torq = Number(p.get("torque"));
          if (p.has("brake")) cmd.brk = Number(p.get("brake"));
          if (p.has("steer")) cmd.csteer = Number(p.get("steer"));
          if (p.has("gear")) cmd.gear = p.get("gear")!;
          if (p.has("enable")) cmd.en = Number(p.get("enable"));
          if (p.has("ctrl")) cmd.src = Math.max(0, ["can", "mqtt", "http"].indexOf(p.get("ctrl")!));
          res.statusCode = 204;
          return res.end();
        }
        if (url.pathname === "/api/firmware" && req.method === "POST") {
          let n = 0;
          req.on("data", (c) => (n += c.length));
          req.on("end", () => {
            res.setHeader("Content-Type", "application/json");
            res.end(JSON.stringify({ status: "ok", bytes: n }));
          });
          return;
        }
        if (url.pathname === "/api/reboot" && req.method === "POST") {
          res.setHeader("Content-Type", "application/json");
          return res.end('{"status":"rebooting"}');
        }
        if (url.pathname === "/api/system") {
          const names = ["watchdog_tid", "stats_tid", "plant_tid", "mqtt_tid", "led_tid", "http_tid", "can_rx_tid", "main"];
          res.setHeader("Content-Type", "application/json");
          return res.end(JSON.stringify({
            broker_addr: "192.168.1.106", broker_port: 8883,
            flash_used: 387700, flash_total: 786432, ram_image: 391168, ram_total: 524288,
            heap_gen: 131072, heap_tls: 131072, stacks: 55296,
            threads: names.map((n, i) => ({ n, p: i + 2, u: 100 + i * 300, t: 1024 * (1 + (i % 4) * 4) })),
          }));
        }
        if (url.pathname !== "/api/state") return next();
        t += 1;
        const v = cmd.en ? Math.min(17.8, t * 0.05) : 0;
        const body = {
          uptime: t * 5, vx: v, vy: 0, yaw: 0, yawr: 0, x: v * t, y: 0, steer: cmd.csteer,
          soc: 50 - t * 0.01, ofl: v * 2, ofr: v * 2, orl: v * 2, orr: v * 2, mu: 0.72,
          ...cmd, cantx: 96102 + t * 100, canrx: 0, canto: 234, lrx: -1,
          hu: 0, hf: 130972, htot: 131072, loop: 600, mqc: 0, mqrx: 0,
        };
        res.setHeader("Content-Type", "application/json");
        res.end(JSON.stringify(body));
      });
    },
  };
}
