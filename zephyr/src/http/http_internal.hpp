// zephyr/src/http/http_internal.hpp
// Internal declarations shared across the HTTP module.
#pragma once

// Request parsing (http_cmd.cpp)
void read_request_line(int fd, char* query_out, int qlen);
void read_headers(int fd, char* auth_out, int alen);   // replaces drain_headers
bool verify_bearer(const char* token_value);
void apply_web_cmd(const char* qs);

// Page builder (http_page.cpp)
void send_page(int fd);
void send_401(int fd);
