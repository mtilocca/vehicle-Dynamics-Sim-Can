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

namespace http {

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

// ── GET /ota ──────────────────────────────────────────────────────────────────

void handle_ota_page(int fd)
{
    // Inlined HTML — same dark theme as dashboard.css.
    // JavaScript uses XMLHttpRequest for upload progress, then fetch for reboot.
    static const char hdr[] =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html; charset=utf-8\r\n"
        "Connection: close\r\n"
        "\r\n";

    static const char body[] =
        "<!DOCTYPE html><html lang='en'><head>"
        "<meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>OTA Update \xe2\x80\x94 HDV Sim</title>"
        "<style>"
        "body{background:#161b22;color:#c9d1d9;font-family:monospace;"
            "margin:0;padding:24px}"
        "h1{color:#58a6ff;margin-bottom:4px}"
        "p,li{color:#8b949e;margin:4px 0}"
        "code{background:#30363d;padding:1px 4px;border-radius:3px}"
        ".card{background:#21262d;border-radius:8px;padding:24px;"
            "max-width:540px;margin:24px auto}"
        "input[type=file]{display:block;margin:16px 0;color:#c9d1d9}"
        "button{background:#238636;color:#fff;border:none;padding:8px 20px;"
            "border-radius:6px;cursor:pointer;font-family:monospace;margin-top:8px}"
        "button:disabled{background:#30363d;color:#6e7681;cursor:not-allowed}"
        "button.reboot{background:#da3633;margin-left:8px}"
        "#bar-wrap{background:#30363d;border-radius:4px;height:12px;"
            "margin:16px 0;display:none}"
        "#bar{background:#238636;border-radius:4px;height:12px;"
            "width:0%;transition:width .2s}"
        "#status{margin-top:12px;min-height:1.2em}"
        ".ok{color:#3fb950}.err{color:#da3633}.warn{color:#d29922}"
        "hr{border:none;border-top:1px solid #30363d;margin:16px 0}"
        "a{color:#58a6ff;text-decoration:none}"
        "</style></head><body>"
        "<div class='card'>"
        "<h1>OTA Firmware Update</h1>"
        "<p>Upload a signed <code>zephyr.signed.bin</code> to replace the running firmware.</p>"
        "<ul>"
        "<li>MCUboot verifies the ECDSA-P256 signature before booting.</li>"
        "<li>On failure the board automatically rolls back to the current image.</li>"
        "<li>Slot 1: 640 KB max (current image: ~286 KB).</li>"
        "</ul>"
        "<hr>"
        "<input type='file' id='f' accept='.bin'>"
        "<div id='bar-wrap'><div id='bar'></div></div>"
        "<div>"
        "<button id='up' onclick='upload()'>Upload Firmware</button>"
        "<button id='rb' class='reboot' style='display:none'"
            " onclick='reboot()'>Reboot &amp; Apply</button>"
        "</div>"
        "<div id='status'></div>"
        "<hr>"
        "<p><a href='/dash'>&larr; Back to Dashboard</a></p>"
        "</div>"
        "<script>"
        // Upload: send file, show progress, then "writing to flash" while server processes.
        "function upload(){"
          "var f=document.getElementById('f').files[0];"
          "if(!f){set('Select a .bin file first','warn');return;}"
          "set('Uploading '+f.name+' ('+Math.round(f.size/1024)+' KB)\u2026','');"
          "document.getElementById('up').disabled=true;"
          "document.getElementById('bar-wrap').style.display='block';"
          "var x=new XMLHttpRequest();"
          "x.open('POST','/api/firmware');"
          "x.setRequestHeader('Content-Type','application/octet-stream');"
          // Track browser-side send progress; at 100% the server is still writing flash.
          "x.upload.onprogress=function(e){"
            "if(e.lengthComputable){"
              "var pct=Math.round(e.loaded/e.total*100);"
              "document.getElementById('bar').style.width=pct+'%';"
              "if(pct===100)"
                "set('File sent \u2014 writing to flash, please wait\u2026','warn');"
            "}"
          "};"
          // onload fires when server responds (after flash write is complete).
          "x.onload=function(){"
            "document.getElementById('up').disabled=false;"
            "var bar=document.getElementById('bar');"
            "if(x.status===200){"
              "var d=JSON.parse(x.responseText);"
              "bar.style.background='#3fb950';"
              "startCountdown(d.bytes);"
            "}else if(x.status===401){"
              "bar.style.background='#da3633';"
              "set('Session expired \u2014 <a href=\"/\">log in again</a> then retry.','err');"
            "}else{"
              "set('Upload failed (HTTP '+x.status+'): '+x.responseText,'err');"
              "bar.style.background='#da3633';"
            "}"
          "};"
          "x.onerror=function(){"
            "set('Network error \u2014 board may have rebooted or session expired. "
                "<a href=\"/\">Log in again</a>.','err');"
            "document.getElementById('up').disabled=false;"
          "};"
          "x.send(f);"
        "}"
        // Auto-reboot countdown: ticks from 5 → 0, then reboots. Cancel link stops it.
        "function startCountdown(bytes){"
          "var n=5;"
          "document.getElementById('rb').style.display='inline-block';"
          "function tick(){"
            "set('Upload OK \u2014 '+bytes+' bytes written. "
                "Rebooting in '+n+' s\u2026 "
                "(<a href=\"#\" onclick=\"cancelReboot();return false\">cancel</a>)','ok');"
            "if(n===0){doReboot();return;}"
            "n--;"
            "window._rt=setTimeout(tick,1000);"
          "}"
          "tick();"
        "}"
        "function cancelReboot(){"
          "clearTimeout(window._rt);"
          "set('Reboot cancelled. Click \u2018Reboot \u0026 Apply\u2019 when ready.','warn');"
          "document.getElementById('rb').textContent='Reboot \u0026 Apply';"
        "}"
        "function doReboot(){reboot();}"
        "function reboot(){"
          "set('Rebooting\u2026 board back in ~5 s','warn');"
          "document.getElementById('rb').disabled=true;"
          "fetch('/api/reboot',{method:'POST'})"
          ".catch(function(){"
            "set('Rebooting (connection dropped \u2014 expected)','ok');"
          "});"
        "}"
        "function set(msg,cls){"
          "var e=document.getElementById('status');"
          "e.className=cls;e.innerHTML=msg;"  // innerHTML so the cancel <a> renders
        "}"
        "</script>"
        "</body></html>";

    zsock_send(fd, hdr,  sizeof(hdr)  - 1, 0);
    zsock_send(fd, body, sizeof(body) - 1, 0);
}

} // namespace http
