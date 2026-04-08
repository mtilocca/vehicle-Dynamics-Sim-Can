// zephyr/src/http/http_internal.hpp
// Internal declarations shared across the HTTP module.
#pragma once

// ── Request parsing (http_cmd.cpp) ────────────────────────────────────────────
// Reads the first request line; fills method ("GET"/"POST"/…), path ("/dash"),
// and query string ("steer=5&torque=0") separately.
void read_request_line(int fd,
                       char* method_out, int mlen,
                       char* path_out,   int plen,
                       char* query_out,  int qlen);

// Reads all headers; captures Authorization Bearer value, Cookie header,
// and Content-Length (set to 0 if not present).
void read_headers(int fd,
                  char* auth_out,    int alen,
                  char* cookie_out,  int clen,
                  int*  content_len);

bool verify_bearer(const char* token_value);
void apply_web_cmd(const char* qs);

// ── Session management (http_session.cpp) ─────────────────────────────────────
bool session_create(char* token_out, int len);   // generates token + stores it
bool session_check(const char* cookie_hdr);      // true if valid sid= cookie
void session_invalidate(const char* cookie_hdr); // removes session on logout

// ── Page builders (http_page.cpp) ────────────────────────────────────────────
void send_page(int fd);
void send_login_page(int fd, bool bad_token);

// ── Response helpers (http_server.cpp) ───────────────────────────────────────
void send_401(int fd);
