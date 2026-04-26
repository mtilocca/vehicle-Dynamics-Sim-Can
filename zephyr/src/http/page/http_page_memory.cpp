// zephyr/src/http/page/http_page_memory.cpp
// Dashboard card: Memory Map (flash + RAM breakdown).
// Reads s_threads[] — must be called after send_threads_card().

#include <zephyr/logging/log.h>
#include "http_page_internal.hpp"

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

void send_memory_card(int fd)
{
    // __rom_region_start → MCUboot slot0 base (0x08040000).
    // MCUboot image header at that address:
    //   offset 0x08: ih_hdr_size     uint16
    //   offset 0x0A: ih_protect_tlv_size uint16
    //   offset 0x0C: ih_img_size     uint32  (payload bytes)
    extern char __rom_region_start[];
    extern char _image_ram_start[];
    extern char _image_ram_end[];

    const uint8_t* hdr = (const uint8_t*)__rom_region_start;
    uint16_t ih_hdr_size, ih_tlv_size;
    uint32_t ih_img_size;
    memcpy(&ih_hdr_size,  hdr + 8,  sizeof(ih_hdr_size));
    memcpy(&ih_tlv_size,  hdr + 10, sizeof(ih_tlv_size));
    memcpy(&ih_img_size,  hdr + 12, sizeof(ih_img_size));
    size_t flash_used = (size_t)ih_hdr_size + ih_img_size + ih_tlv_size;

    size_t ram_image = (size_t)(_image_ram_end - _image_ram_start);

    static const size_t kHeapGen = CONFIG_HEAP_MEM_POOL_SIZE;
    static const size_t kHeapTLS = CONFIG_MBEDTLS_HEAP_SIZE;

    size_t stacks_sz = 0;
    for (int i = 0; i < s_thread_count; ++i) stacks_sz += s_threads[i].stack_total;

    size_t other = (ram_image > kHeapGen + kHeapTLS + stacks_sz)
                   ? ram_image - kHeapGen - kHeapTLS - stacks_sz : 0;

    int flash_pct = (int)(100u * flash_used / (768u * 1024u));
    int ram_pct   = (int)(100u * ram_image  / (512u * 1024u));

    char buf[400];

    snprintf(buf, sizeof(buf),
        "<div class='card'><h2>Memory Map</h2><table>"
        "<tr style='color:#8b949e;font-size:11px'>"
        "<td colspan='3'>FLASH &mdash; MCUboot slot0 (768&nbsp;KB)</td></tr>"
        "<tr><td>Image&nbsp;(text+rodata+.data)</td>"
        "<td class='%s'>%zu&nbsp;KB&nbsp;/&nbsp;768&nbsp;KB&nbsp;(%d%%)</td>"
        "<td></td></tr>",
        flash_pct >= 80 ? "val-warn" : "val-hi",
        flash_used / 1024, flash_pct);
    send_str(fd, buf);

    snprintf(buf, sizeof(buf),
        "<tr style='color:#8b949e;font-size:11px'>"
        "<td colspan='3' style='padding-top:8px'>"
        "RAM &mdash; AXI SRAM (512&nbsp;KB) &mdash; static image footprint</td></tr>"
        "<tr><td>Total image RAM</td>"
        "<td class='%s'>%zu&nbsp;KB&nbsp;/&nbsp;512&nbsp;KB&nbsp;(%d%%)</td>"
        "<td style='color:#8b949e;font-size:11px'>includes all rows below</td></tr>",
        ram_pct >= 80 ? "val-warn" : ram_pct >= 60 ? "" : "val-hi",
        ram_image / 1024, ram_pct);
    send_str(fd, buf);

    snprintf(buf, sizeof(buf),
        "<tr><td>&nbsp;&nbsp;General heap pool</td>"
        "<td>%zu&nbsp;KB</td>"
        "<td style='color:#8b949e;font-size:11px'>"
        "runtime:&nbsp;<span id='mm-hu'>?</span>&nbsp;used&nbsp;/&nbsp;"
        "<span id='mm-hf'>?</span>&nbsp;free</td></tr>"
        "<tr><td>&nbsp;&nbsp;mbedTLS heap pool</td>"
        "<td>%zu&nbsp;KB</td>"
        "<td style='color:#8b949e;font-size:11px'>peak ~60&nbsp;KB during TLS handshake</td></tr>",
        kHeapGen / 1024, kHeapTLS / 1024);
    send_str(fd, buf);

    snprintf(buf, sizeof(buf),
        "<tr><td>&nbsp;&nbsp;Thread stacks (%d)</td>"
        "<td>%zu&nbsp;KB</td><td></td></tr>"
        "<tr><td>&nbsp;&nbsp;Kernel&nbsp;+&nbsp;globals&nbsp;+&nbsp;buffers</td>"
        "<td>%zu&nbsp;KB</td><td></td></tr>"
        "</table></div>",
        s_thread_count, stacks_sz / 1024,
        other / 1024);
    send_str(fd, buf);
}
