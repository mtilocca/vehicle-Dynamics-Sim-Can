// zephyr/src/http/http_session.cpp
// Browser session management — create / validate / invalidate short-lived
// session tokens stored in a small fixed-size table on the MCU.
//
// Flow:
//   POST /login  → verify Bearer token → session_create() → Set-Cookie: sid=<hex>
//   Every /dash  → session_check(cookie_header) → true if valid
//   GET /logout  → session_invalidate(cookie_header) → clear-cookie redirect

#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdio.h>

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

// ── Session table ─────────────────────────────────────────────────────────────

#define MAX_SESSIONS    4
#define SESSION_TTL_MS  3600000u    // 1 hour

struct Session {
    char     token[33]; // 32-char hex + null
    uint32_t expires_ms;
    bool     used;
};

static Session       s_sessions[MAX_SESSIONS];
K_MUTEX_DEFINE(g_session_mutex);

// ── Internal helpers ──────────────────────────────────────────────────────────

static void evict_expired(void)
{
    uint32_t now = k_uptime_get_32();
    for (int i = 0; i < MAX_SESSIONS; ++i) {
        if (s_sessions[i].used) {
            // Signed comparison handles wrap-around correctly
            if ((int32_t)(now - s_sessions[i].expires_ms) >= 0)
                s_sessions[i].used = false;
        }
    }
}

// Extract "sid=<32hex>" from a raw Cookie header value.
// Returns false if not found or malformed.
static bool extract_sid(const char* cookie_hdr, char* out, int olen)
{
    if (!cookie_hdr || !cookie_hdr[0]) return false;
    const char* p = strstr(cookie_hdr, "sid=");
    if (!p) return false;
    p += 4;
    int i = 0;
    while (i < 32 && i < olen - 1 && p[i] && p[i] != ';' && p[i] != ' ') {
        out[i] = p[i];
        i++;
    }
    out[i] = '\0';
    return (i == 32);
}

// ── Public API ────────────────────────────────────────────────────────────────

bool session_create(char* token_out, int len)
{
    if (len < 33) return false;

    // Generate 16 random bytes → 32-char hex token
    uint8_t rnd[16];
    sys_rand_get(rnd, sizeof(rnd));
    for (int i = 0; i < 16; ++i)
        snprintf(token_out + 2 * i, 3, "%02x", rnd[i]);
    token_out[32] = '\0';

    k_mutex_lock(&g_session_mutex, K_FOREVER);
    evict_expired();

    // Find a free slot; if all full, evict slot 0
    int slot = 0;
    for (int i = 0; i < MAX_SESSIONS; ++i) {
        if (!s_sessions[i].used) { slot = i; break; }
    }

    strncpy(s_sessions[slot].token, token_out, 32);
    s_sessions[slot].token[32]    = '\0';
    s_sessions[slot].expires_ms   = k_uptime_get_32() + SESSION_TTL_MS;
    s_sessions[slot].used         = true;
    k_mutex_unlock(&g_session_mutex);

    LOG_INF("Session created: slot=%d tok=%.8s...", slot, token_out);
    return true;
}

bool session_check(const char* cookie_hdr)
{
    char tok[33];
    if (!extract_sid(cookie_hdr, tok, sizeof(tok))) return false;

    uint32_t now   = k_uptime_get_32();
    bool     found = false;

    k_mutex_lock(&g_session_mutex, K_FOREVER);
    for (int i = 0; i < MAX_SESSIONS; ++i) {
        if (s_sessions[i].used &&
            strncmp(s_sessions[i].token, tok, 32) == 0 &&
            (int32_t)(now - s_sessions[i].expires_ms) < 0) {
            found = true;
            break;
        }
    }
    k_mutex_unlock(&g_session_mutex);
    return found;
}

void session_invalidate(const char* cookie_hdr)
{
    char tok[33];
    if (!extract_sid(cookie_hdr, tok, sizeof(tok))) return;

    k_mutex_lock(&g_session_mutex, K_FOREVER);
    for (int i = 0; i < MAX_SESSIONS; ++i) {
        if (s_sessions[i].used &&
            strncmp(s_sessions[i].token, tok, 32) == 0) {
            s_sessions[i].used = false;
            LOG_INF("Session invalidated: slot=%d", i);
            break;
        }
    }
    k_mutex_unlock(&g_session_mutex);
}
