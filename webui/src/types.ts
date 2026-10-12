// Shape of GET /api/state (zephyr/src/http/http_page.cpp: send_api_state).
export interface State {
  uptime: number;
  vx: number; vy: number; yaw: number; yawr: number; x: number; y: number;
  steer: number; soc: number;
  ofl: number; ofr: number; orl: number; orr: number; mu: number;
  en: number; gear: "F" | "R" | "N"; torq: number; brk: number; csteer: number;
  cantx: number; canrx: number; canto: number; lrx: number;
  hu: number; hf: number; htot: number; loop: number;
  mqc: number; mqrx: number; src: number;
}

// Shape of GET /api/system (http_page.cpp: send_api_system).
export interface Thread { n: string; p: number; u: number; t: number }
export interface System {
  broker_addr: string; broker_port: number;
  flash_used: number; flash_total: number;
  ram_image: number; ram_total: number;
  heap_gen: number; heap_tls: number; stacks: number;
  threads: Thread[];
}
