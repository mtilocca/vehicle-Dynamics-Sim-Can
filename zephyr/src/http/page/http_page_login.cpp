// zephyr/src/http/page/http_page_login.cpp
// Login page — shown when session cookie is absent or token is wrong.

#include <zephyr/logging/log.h>
#include "http_page_internal.hpp"

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

namespace http {

void send_login_page(int fd, bool bad_token)
{
    static const char hdr[] =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html; charset=utf-8\r\n"
        "Connection: close\r\n"
        "\r\n"
        "<!DOCTYPE html><html><head>"
        "<meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>HDV Sim &mdash; Login</title>"
        "<style>"
        "body{background:#0d1117;color:#c9d1d9;font-family:monospace;"
        "display:flex;align-items:center;justify-content:center;"
        "min-height:100vh;margin:0;}"
        ".card{background:#161b22;border:1px solid #30363d;border-radius:8px;"
        "padding:32px 40px;width:420px;}"
        "h1{color:#58a6ff;font-size:18px;margin:0 0 4px;}"
        ".sub{color:#8b949e;font-size:12px;margin:0 0 24px;}"
        "label{display:block;color:#8b949e;font-size:12px;margin-bottom:6px;}"
        "input[type=text]{"
        "width:100%;box-sizing:border-box;"
        "background:#0d1117;border:1px solid #30363d;border-radius:6px;"
        "color:#c9d1d9;font-family:monospace;font-size:13px;"
        "padding:12px 14px;margin-bottom:20px;}"
        "input[type=text]:focus{outline:none;border-color:#58a6ff;}"
        "button{width:100%;background:#238636;border:none;border-radius:6px;"
        "color:#fff;font-family:monospace;font-size:14px;padding:10px;cursor:pointer;}"
        "button:hover{background:#2ea043;}"
        ".err{color:#f85149;font-size:13px;margin-bottom:16px;"
        "padding:8px;border:1px solid #f8514944;border-radius:6px;background:#f8514911;}"
        "</style></head><body>"
        "<div class='card'>"
        "<h1>&#9651;&nbsp;HDV Simulator</h1>"
        "<p class='sub'>Heavy-Duty Electric Vehicle &mdash; Secure Dashboard</p>";

    zsock_send(fd, hdr, sizeof(hdr) - 1, 0);

    if (bad_token) {
        static const char err[] =
            "<p class='err'>&#10007;&nbsp;Invalid token &mdash; try again.</p>";
        zsock_send(fd, err, sizeof(err) - 1, 0);
    }

    static const char form[] =
        "<form method='post' action='/login'>"
        "<label for='tok'>API Token</label>"
        "<input id='tok' type='text' name='token'"
        " placeholder='paste token here' autocomplete='off' autofocus spellcheck='false'>"
        "<button type='submit'>Sign In &rarr;</button>"
        "</form>"
        "</div></body></html>";
    zsock_send(fd, form, sizeof(form) - 1, 0);
}

} // namespace http
