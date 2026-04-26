// zephyr/src/http/http_internal.hpp
// Internal declarations shared across the HTTP module.
#pragma once

namespace http {

// ── Request parsing (http_cmd.cpp) ────────────────────────────────────────────
void read_request_line(int fd,
                       char* method_out, int mlen,
                       char* path_out,   int plen,
                       char* query_out,  int qlen);

void read_headers(int fd,
                  char* auth_out,    int alen,
                  char* cookie_out,  int clen,
                  int*  content_len);

bool verify_bearer(const char* token_value);
void apply_web_cmd(const char* qs);

// ── Session management (http_session.cpp) ─────────────────────────────────────
bool session_create(char* token_out, int len);
bool session_check(const char* cookie_hdr);
void session_invalidate(const char* cookie_hdr);

// ── Page builders (page/) ─────────────────────────────────────────────────────
void send_page(int fd);
void send_login_page(int fd, bool bad_token);
void send_api_state(int fd);

// ── Response helpers (http_server.cpp) ───────────────────────────────────────
void send_401(int fd);

// ── OTA firmware update (http_ota.cpp) ───────────────────────────────────────
void handle_ota_page(int fd);
void handle_ota_upload(int fd, int content_length);
void handle_api_reboot(int fd);

} // namespace http
