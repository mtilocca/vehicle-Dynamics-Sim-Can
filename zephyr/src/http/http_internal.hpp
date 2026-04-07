// zephyr/src/http/http_internal.hpp
// Internal declarations shared across the HTTP module.
// Not part of the public interface — only include within http/*.cpp.
#pragma once

// Request parsing (http_cmd.cpp)
void read_request_line(int fd, char* query_out, int qlen);
void drain_headers(int fd);
void apply_web_cmd(const char* qs);

// Page builder (http_page.cpp)
void send_page(int fd);
