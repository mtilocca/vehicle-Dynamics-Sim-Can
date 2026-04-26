// zephyr/src/http/page/http_page_resources.cpp
// Dashboard card: System Resources (heap usage + sim loop timing).

#include <zephyr/logging/log.h>
#include "http_page_internal.hpp"

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

void send_resources_card(int fd)
{
    hdv::SysStats st{};
    { hdv::MutexGuard g(g_health_mtx); st = g_health_bus.stats; }

    const size_t heap_total = CONFIG_HEAP_MEM_POOL_SIZE;
    int heap_pct = (heap_total > 0 && st.heap_used > 0)
                   ? (int)(100u * st.heap_used / heap_total) : 0;
    const char* heap_cls = (heap_pct >= 80) ? "val-warn"
                         : (heap_pct >= 60) ? ""
                         : "val-hi";

    int loop_tenths = (int)(st.plant_loop_us_max / 100);
    const char* loop_cls = (st.plant_loop_us_max > 10000) ? "val-warn" : "val-hi";

    char buf[512];
    snprintf(buf, sizeof(buf),
        "<div class='card'><h2>System Resources</h2><table>"
        "<tr><td>Heap used</td>"
        "<td id='res-hu'><span class='%s'>%zu&nbsp;B / %zu&nbsp;B&nbsp;(%d%%)</span></td></tr>"
        "<tr><td>Heap free</td><td id='res-hf'>%zu&nbsp;B</td></tr>"
        "<tr><td>Sim loop&nbsp;max</td>"
        "<td id='res-loop'><span class='%s'>%d.%d&nbsp;ms</span>"
        "&nbsp;<span style='color:#8b949e'>(budget&nbsp;10&nbsp;ms)</span></td></tr>"
        "</table></div>",
        heap_cls, st.heap_used, heap_total, heap_pct,
        st.heap_free,
        loop_cls, loop_tenths / 10, loop_tenths % 10);
    send_str(fd, buf);
}
