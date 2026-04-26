// zephyr/src/http/page/http_page_threads.cpp
// Dashboard card: Kernel Threads with stack usage.
// Defines s_threads[] and s_thread_count used by http_page_memory.cpp.

#include <zephyr/logging/log.h>
#include "http_page_internal.hpp"

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

ThreadInfo s_threads[20];
int        s_thread_count = 0;

static void collect_thread_cb(const struct k_thread* t, void*)
{
    if (s_thread_count >= 20) return;
    ThreadInfo& info = s_threads[s_thread_count++];

    const char* n = k_thread_name_get((k_tid_t)t);
    strncpy(info.name, (n && n[0]) ? n : "?", sizeof(info.name) - 1);
    info.name[sizeof(info.name) - 1] = '\0';

    info.priority    = k_thread_priority_get((k_tid_t)t);
    info.stack_total = t->stack_info.size;

    size_t unused = 0;
    k_thread_stack_space_get(t, &unused);
    info.stack_used = (info.stack_total > unused) ? (info.stack_total - unused) : 0;
}

void send_threads_card(int fd)
{
    char buf[512];

    s_thread_count = 0;
    k_thread_foreach(collect_thread_cb, nullptr);

    send_str(fd,
        "<div class='card'>"
        "<h2>Kernel Threads</h2>"
        "<table>"
        "<tr style='color:#8b949e;font-size:12px'>"
        "<td>Name</td><td>Prio</td><td style='width:40%'>Stack used / total</td><td>%</td>"
        "</tr>"
    );

    for (int i = 0; i < s_thread_count; ++i) {
        const ThreadInfo& ti = s_threads[i];
        int pct = (ti.stack_total > 0)
                  ? (int)(100u * ti.stack_used / ti.stack_total) : 0;
        const char* cls = (pct >= 80) ? "val-warn" : (pct >= 60) ? "" : "val-hi";

        snprintf(buf, sizeof(buf),
            "<tr><td>%.23s</td><td>%d</td>"
            "<td><span class='%.8s'>%zu</span> / %zu B</td>"
            "<td class='%.8s'>%d%%</td></tr>",
            ti.name, ti.priority,
            cls, ti.stack_used, ti.stack_total,
            cls, pct);
        send_str(fd, buf);
    }

    send_str(fd, "</table></div>");
}
