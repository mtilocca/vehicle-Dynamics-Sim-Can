// zephyr/src/http/http_ota.cpp
// OTA firmware update over HTTP using MCUboot slot1.
//
// POST /api/firmware   — stream raw zephyr.signed.bin to slot1, arm upgrade
// POST /api/reboot     — cold-reboot into MCUboot to apply the swap
// GET  /ota            — browser upload page
//
// Safety:
//   g_ota_mutex prevents concurrent uploads.
//   Content-Length is validated against slot1 size before any flash write.
//   MCUboot uses BOOT_UPGRADE_TEST: new image must call boot_write_img_confirmed()
//   (done in main.cpp) or MCUboot rolls back automatically on the next reset.
//   ECDSA-P256 signature is verified by MCUboot before chainload — an unsigned
//   or tampered binary is rejected even if it uploads successfully.

#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/logging/log.h>
#include <zephyr/dfu/flash_img.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/reboot.h>
#include <string.h>
#include <stdio.h>

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

// slot1_partition: bank 2 sectors 0-4, 640 KB (0x08100000–0x081A0000)
#define SLOT1_MAX_BYTES (640u * 1024u)
#define RX_CHUNK_SIZE   4096

K_MUTEX_DEFINE(g_ota_mutex);

// Static receive buffer — keeps 4 KB off the HTTP thread stack.
static uint8_t s_rx_buf[RX_CHUNK_SIZE];

// ── JSON response helper ──────────────────────────────────────────────────────

static void send_json(int fd, int code, const char* phrase, const char* body)
{
    char hdr[192];
    int  blen = (int)strlen(body);
    int  n = snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 %d %s\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: %d\r\n"
        "Connection: close\r\n"
        "\r\n",
        code, phrase, blen);
    zsock_send(fd, hdr,  n,    0);
    zsock_send(fd, body, blen, 0);
}

// ── POST /api/firmware ────────────────────────────────────────────────────────

void handle_ota_upload(int fd, int content_length)
{
    if (content_length <= 0) {
        send_json(fd, 400, "Bad Request",
                  "{\"error\":\"Content-Length required\"}");
        return;
    }
    if ((unsigned)content_length > SLOT1_MAX_BYTES) {
        send_json(fd, 413, "Content Too Large",
                  "{\"error\":\"firmware exceeds slot1 size (640 KB)\"}");
        return;
    }
    if (k_mutex_lock(&g_ota_mutex, K_NO_WAIT) != 0) {
        send_json(fd, 503, "Service Unavailable",
                  "{\"error\":\"upload already in progress\"}");
        return;
    }

    LOG_INF("OTA: upload started — %d bytes → slot1 (0x08100000)", content_length);

    // flash_img_buffered_write + CONFIG_IMG_ERASE_PROGRESSIVELY=y erases
    // each sector on-demand as data arrives — no separate flash_area_erase needed.
    struct flash_img_context ctx;
    int rc = flash_img_init_id(&ctx, FIXED_PARTITION_ID(slot1_partition));
    if (rc != 0) {
        LOG_ERR("OTA: flash_img_init_id failed (%d)", rc);
        k_mutex_unlock(&g_ota_mutex);
        send_json(fd, 500, "Internal Server Error",
                  "{\"error\":\"flash init failed\"}");
        return;
    }

    int total = 0;
    while (total < content_length) {
        int want = MIN(RX_CHUNK_SIZE, content_length - total);
        int got  = zsock_recv(fd, s_rx_buf, want, 0);
        if (got <= 0) break;

        bool flush = (total + got >= content_length);
        rc = flash_img_buffered_write(&ctx, s_rx_buf, (size_t)got, flush);
        if (rc != 0) {
            LOG_ERR("OTA: write failed (%d) at offset %d", rc, total);
            k_mutex_unlock(&g_ota_mutex);
            send_json(fd, 500, "Internal Server Error",
                      "{\"error\":\"flash write failed\"}");
            return;
        }
        total += got;
    }

    if (total != content_length) {
        LOG_WRN("OTA: incomplete upload (%d / %d bytes)", total, content_length);
        k_mutex_unlock(&g_ota_mutex);
        send_json(fd, 400, "Bad Request", "{\"error\":\"incomplete upload\"}");
        return;
    }

    rc = boot_request_upgrade(BOOT_UPGRADE_TEST);
    k_mutex_unlock(&g_ota_mutex);

    if (rc != 0) {
        LOG_ERR("OTA: boot_request_upgrade failed (%d)", rc);
        send_json(fd, 500, "Internal Server Error",
                  "{\"error\":\"upgrade request failed\"}");
        return;
    }

    LOG_INF("OTA: upload complete (%d bytes) — upgrade armed, POST /api/reboot to apply",
            total);
    char resp[64];
    snprintf(resp, sizeof(resp), "{\"status\":\"ok\",\"bytes\":%d}", total);
    send_json(fd, 200, "OK", resp);
}

// ── POST /api/reboot ──────────────────────────────────────────────────────────

void handle_api_reboot(int fd)
{
    LOG_INF("OTA: cold reboot requested via HTTP");
    send_json(fd, 200, "OK", "{\"status\":\"rebooting\"}");
    zsock_close(fd);  // flush TCP before reboot
    k_msleep(200);
    sys_reboot(SYS_REBOOT_COLD);
}
