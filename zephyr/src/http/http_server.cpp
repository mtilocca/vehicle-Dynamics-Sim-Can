// zephyr/src/http/http_server.cpp
// HTTPS server thread (TLS 1.2, port 443) — request routing + auth.
//
// Security model:
//   - TLS 1.2 with ECDHE-RSA-AES128-GCM-SHA256 (self-signed cert, IP SAN)
//   - Bearer token or browser session cookie required for all control endpoints
//   - Token generated once per deployment by scripts/gen_http_token.sh (gitignored)
//   - Session cookie carries Secure flag — never sent over plain HTTP
//   - Input values clamped to physical limits before writing to g_cmd
//
// Routes:
//   GET  /            → login page
//   GET  /login       → login page
//   POST /login       → validate token → set session cookie → redirect /dash
//   GET  /dash        → dashboard (requires session cookie or Bearer token)
//   GET  /dash?...    → apply command + dashboard (requires auth)
//   GET  /logout      → invalidate session + redirect /
//   GET  /ota         → OTA upload page (requires auth)
//   GET  /api/state   → JSON telemetry snapshot for JS polling (requires auth)
//   POST /api/firmware → stream firmware binary to slot1 (requires auth)
//   POST /api/reboot  → cold-reboot into MCUboot swap (requires auth)
//   other             → redirect /

#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/tls_credentials.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdio.h>

#include "http_internal.hpp"
#include "tls/tls_creds.hpp"

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

using namespace http;

// ── Response helpers ──────────────────────────────────────────────────────────

namespace http {
void send_401(int fd)
{
    static const char resp[] =
        "HTTP/1.1 401 Unauthorized\r\n"
        "Content-Type: text/plain\r\n"
        "Content-Length: 13\r\n"
        "Connection: close\r\n"
        "\r\n"
        "Unauthorized\n";
    zsock_send(fd, resp, sizeof(resp) - 1, 0);
}
} // namespace http

static void send_redirect(int fd, const char* location)
{
    char buf[256];
    int n = snprintf(buf, sizeof(buf),
        "HTTP/1.1 302 Found\r\n"
        "Location: %s\r\n"
        "Connection: close\r\n"
        "Content-Length: 0\r\n"
        "\r\n",
        location);
    zsock_send(fd, buf, n, 0);
}

// Redirect to /dash after successful login; set session cookie.
static void send_login_success(int fd, const char* sid)
{
    char buf[384];
    int n = snprintf(buf, sizeof(buf),
        "HTTP/1.1 302 Found\r\n"
        "Location: /dash\r\n"
        "Set-Cookie: sid=%s; Path=/; HttpOnly; Secure; SameSite=Lax; Max-Age=3600\r\n"
        "Connection: close\r\n"
        "Content-Length: 0\r\n"
        "\r\n",
        sid);
    zsock_send(fd, buf, n, 0);
}

// Clear session cookie on logout.
static void send_logout_redirect(int fd)
{
    static const char resp[] =
        "HTTP/1.1 302 Found\r\n"
        "Location: /\r\n"
        "Set-Cookie: sid=; Path=/; HttpOnly; Max-Age=0\r\n"
        "Connection: close\r\n"
        "Content-Length: 0\r\n"
        "\r\n";
    zsock_send(fd, resp, sizeof(resp) - 1, 0);
}

// ── HTTP server thread ────────────────────────────────────────────────────────
static void http_server_thread(void*, void*, void*)
{
    k_msleep(2000);  // let Ethernet settle

    if (tls_creds_init() != 0) {
        LOG_ERR("HTTPS: TLS credential load failed — aborting"); return;
    }

    int srv = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TLS_1_2);
    if (srv < 0) { LOG_ERR("HTTPS: socket() failed: %d", srv); return; }

    // Bind server certificate credentials to the socket.
    // TLS role (server vs client) is implicit for TLS 1.2: accept() = server.
    sec_tag_t sec_tag = HDV_TLS_SERVER_TAG;
    zsock_setsockopt(srv, SOL_TLS, TLS_SEC_TAG_LIST, &sec_tag, sizeof(sec_tag));

    int opt = 1;
    zsock_setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(443);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (zsock_bind(srv, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        LOG_ERR("HTTPS: bind() failed"); zsock_close(srv); return;
    }
    if (zsock_listen(srv, 3) < 0) {
        LOG_ERR("HTTPS: listen() failed"); zsock_close(srv); return;
    }

    LOG_INF("HTTPS server listening on 192.168.1.80:443");
    LOG_INF("Open https://192.168.1.80/ in your browser");

    while (true) {
        struct sockaddr_in client_addr{};
        socklen_t client_len = sizeof(client_addr);
        int client = zsock_accept(srv, (struct sockaddr*)&client_addr, &client_len);
        if (client < 0) {
            int err = errno;
            if (err == ENOMEM) {
                // Heap exhausted — wait for current handshake to free memory.
                LOG_WRN("HTTPS: accept() ENOMEM — backing off 300 ms");
                k_msleep(300);
            } else {
                // Client aborted handshake (browser cancelled speculative conn).
                LOG_DBG("HTTPS: accept() abandoned by client (errno=%d)", err);
            }
            continue;
        }
        LOG_INF("HTTPS: client connected (TLS handshake OK), fd=%d", client);

        // ── Parse request ─────────────────────────────────────────────────────
        char method[8]   = {};
        char path[64]    = {};
        char query[128]  = {};
        char auth[128]   = {};
        char cookie[256] = {};
        int  clen        = 0;

        read_request_line(client,
                          method, sizeof(method),
                          path,   sizeof(path),
                          query,  sizeof(query));
        read_headers(client,
                     auth,   sizeof(auth),
                     cookie, sizeof(cookie),
                     &clen);

        if (method[0] == '\0') {
            zsock_close(client);
            continue;
        }

        bool is_get          = (method[0] == 'G');
        bool is_post         = (method[0] == 'P');
        bool is_root         = (path[0] == '\0' || strcmp(path, "/") == 0);
        bool is_login        = (strcmp(path, "/login")        == 0);
        bool is_dash         = (strcmp(path, "/dash")         == 0);
        bool is_logout       = (strcmp(path, "/logout")       == 0);
        bool is_ota          = (strcmp(path, "/ota")          == 0);
        bool is_api_state    = (strcmp(path, "/api/state")    == 0);
        bool is_api_firmware = (strcmp(path, "/api/firmware") == 0);
        bool is_api_reboot   = (strcmp(path, "/api/reboot")   == 0);

        bool has_bearer  = verify_bearer(auth);
        bool has_session = session_check(cookie);
        bool authed      = has_bearer || has_session;

        // ── Route ─────────────────────────────────────────────────────────────

        if (is_post && is_login) {
            // Read POST body (token=<hex>)
            char body[192] = {};
            int to_read = (clen > 0 && clen < (int)sizeof(body) - 1) ? clen : 0;
            int got = 0;
            while (got < to_read) {
                int r = zsock_recv(client, body + got, to_read - got, 0);
                if (r <= 0) break;
                got += r;
            }

            // Extract token value from "token=<value>"
            const char* tok = "";
            const char* eq  = strstr(body, "token=");
            if (eq) tok = eq + 6;
            char tok_clean[128] = {};
            int  ti = 0;
            while (ti < (int)sizeof(tok_clean) - 1 &&
                   tok[ti] && tok[ti] != '&' && tok[ti] != '\r' && tok[ti] != '\n') {
                tok_clean[ti] = tok[ti];
                ti++;
            }
            tok_clean[ti] = '\0';

if (verify_bearer(tok_clean)) {
                char sid[33] = {};
                session_create(sid, sizeof(sid));
                LOG_INF("HTTP: login OK → /dash");
                send_login_success(client, sid);
            } else {
                LOG_WRN("HTTP: login FAILED — bad token");
                send_login_page(client, true);
            }

        } else if (is_get && is_logout) {
            session_invalidate(cookie);
            LOG_INF("HTTP: logout");
            send_logout_redirect(client);

        } else if (is_get && (is_root || is_login)) {
            send_login_page(client, false);

        } else if (is_get && is_dash) {
            if (authed) {
                if (query[0]) apply_web_cmd(query);
                send_page(client);
            } else {
                send_redirect(client, "/");
            }

        } else if (is_get && is_ota) {
            if (authed) { handle_ota_page(client); }
            else        { send_redirect(client, "/"); }

        } else if (is_get && is_api_state) {
            if (authed) { send_api_state(client); }
            else        { send_401(client); }

        } else if (is_post && is_api_firmware) {
            if (authed) {
                handle_ota_upload(client, clen);
            } else {
                // Drain the upload body before sending 401.
                // The browser is already streaming the firmware binary. If we
                // close the socket immediately after the 401, the H7 TCP stack
                // sends a RST that races with the response — the browser sees
                // xhr.onerror ("Network error") instead of HTTP 401 status,
                // so the user gets no useful feedback.
                // Draining lets the 401 reach the browser cleanly.
                LOG_WRN("OTA: auth failed (session expired?) — draining %d B body before 401", clen);
                if (clen > 0) {
                    char drain[256];
                    int drained = 0;
                    while (drained < clen) {
                        int want = (clen - drained < (int)sizeof(drain))
                                   ? (clen - drained) : (int)sizeof(drain);
                        int r = zsock_recv(client, drain, want, 0);
                        if (r <= 0) break;
                        drained += r;
                    }
                }
                send_401(client);
            }

        } else if (is_post && is_api_reboot) {
            if (authed) { handle_api_reboot(client); /* never returns */ }
            else        { send_401(client); }

        } else if (is_get && authed) {
            send_redirect(client, "/dash");

        } else {
            send_redirect(client, "/");
        }

        zsock_close(client);
    }
}

// Stack 16 KB — TLS handshake needs ~12 KB headroom above plain HTTP overhead
K_THREAD_DEFINE(http_tid, 16384, http_server_thread, NULL, NULL, NULL, 10, 0, 0);
