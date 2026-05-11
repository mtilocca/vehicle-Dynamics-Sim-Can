/*
 * zephyr/src/tls/mbedtls_heap_sram1.c
 *
 * Redirects the mbedTLS memory-buffer allocator to a static buffer placed in
 * SRAM1 (0x30000000, 128 KB, D2 domain) instead of the default AXI SRAM.
 *
 * How it works
 * ────────────
 * CONFIG_MBEDTLS_ENABLE_HEAP=y (prj.conf) enables MBEDTLS_MEMORY_BUFFER_ALLOC_C
 * and causes Zephyr's modules/mbedtls/mbedtls_heap.c to:
 *   1. Allocate  static unsigned char heap_buf[CONFIG_MBEDTLS_HEAP_SIZE]  in AXI
 *      SRAM BSS.
 *   2. Call  mbedtls_memory_buffer_alloc_init(heap_buf, sizeof(heap_buf))  at
 *      POST_KERNEL priority.
 *
 * This file defines an identically-sized buffer in SRAM1 (annotated with
 * Z_GENERIC_SECTION(SRAM1), which maps to __attribute__((section("SRAM1")))).
 * A SYS_INIT hook at APPLICATION priority — guaranteed to run after POST_KERNEL
 * — calls mbedtls_memory_buffer_alloc_init() again with the SRAM1 buffer,
 * atomically redirecting the allocator.  All TLS handshakes and record
 * processing that happen after system init will draw memory from SRAM1.
 *
 * Residual cost
 * ─────────────
 * Zephyr's heap_buf (128 KB) still appears in the .bss section in AXI SRAM
 * because it is defined by a Zephyr module we do not modify.  It is never
 * used at runtime.  To remove it from the binary entirely, add a linker snippet
 * via zephyr_linker_sources(BSS_SECTIONS ...) that relocates the symbol:
 *
 *     *:mbedtls_heap*.o(.bss.heap_buf)  >SRAM1
 *
 * That approach moves the symbol at link time and eliminates the AXI SRAM
 * cost without any runtime overhead.
 *
 * SRAM1 is accessible by DMA1, DMA2, Ethernet DMA, and BDMA (D2 domain AHB
 * slave port), so placing the mbedTLS heap there is safe even if Ethernet or
 * other DMA masters are active.
 */

#include <zephyr/init.h>
#include <zephyr/linker/sections.h>
#include <mbedtls/memory_buffer_alloc.h>

/* 128 KB heap in SRAM1 (0x30000000–0x3001FFFF).
 * __noinit skips zero-initialisation at boot; the allocator's own init call
 * sets up the internal free-list, so zeroing is redundant and wastes time. */
static uint8_t __noinit Z_GENERIC_SECTION(SRAM1)
    s_mbedtls_sram1_heap[CONFIG_MBEDTLS_HEAP_SIZE];

static int mbedtls_heap_relocate_to_sram1(void)
{
	mbedtls_memory_buffer_alloc_init(s_mbedtls_sram1_heap,
					 sizeof(s_mbedtls_sram1_heap));
	return 0;
}

/*
 * APPLICATION priority ensures this runs after Zephyr's modules/mbedtls init
 * (POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT) so the redirect takes
 * effect before any TLS socket is opened.
 */
SYS_INIT(mbedtls_heap_relocate_to_sram1, APPLICATION,
	 CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
