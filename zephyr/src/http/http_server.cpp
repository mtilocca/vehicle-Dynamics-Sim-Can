// zephyr/src/http/http_server.cpp
// HTTP server thread — socket lifecycle (open / bind / listen / accept loop).
// Delegates request parsing to http_cmd.cpp and page rendering to http_page.cpp.
//
// Priority 10: below plant sim (prio 5), above LED (prio 12) and shell (prio 14).

#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/logging/log.h>

#include "http_internal.hpp"

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

static void http_server_thread(void*, void*, void*)
{
    k_msleep(2000);

    int srv = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (srv < 0) {
        LOG_ERR("HTTP: socket() failed: %d", srv);
        return;
    }

    int opt = 1;
    zsock_setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(80);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (zsock_bind(srv, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        LOG_ERR("HTTP: bind() failed");
        zsock_close(srv);
        return;
    }

    if (zsock_listen(srv, 3) < 0) {
        LOG_ERR("HTTP: listen() failed");
        zsock_close(srv);
        return;
    }

    LOG_INF("HTTP server listening on 192.168.1.80:80");

    while (true) {
        struct sockaddr_in client_addr{};
        socklen_t client_len = sizeof(client_addr);
        int client = zsock_accept(srv, (struct sockaddr*)&client_addr, &client_len);
        if (client < 0) {
            k_msleep(10);
            continue;
        }

        char query[128] = {};
        read_request_line(client, query, sizeof(query));
        drain_headers(client);

        if (query[0] != '\0') {
            apply_web_cmd(query);
        }

        send_page(client);
        zsock_close(client);
    }
}

K_THREAD_DEFINE(http_tid, 8192, http_server_thread, NULL, NULL, NULL, 10, 0, 0);
