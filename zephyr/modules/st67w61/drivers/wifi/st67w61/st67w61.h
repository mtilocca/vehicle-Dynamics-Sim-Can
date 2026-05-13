/* st67w61.h — internal driver header
 *
 * SPI frame protocol (stm32-hotspot/ST67W61-Bare-metal-implementation):
 *   Header  : 8 bytes — [0xAA][0x55][LEN_L][LEN_H][0x00][0x00][0x00][0x00]
 *   Payload : AT command string (with \r\n), padded to 4-byte boundary (0x88)
 *
 * CS-first: assert CS, wait RDY HIGH, then clock.  RDY is active-HIGH.
 */

#pragma once
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>

#define ST67W61_HDR_LEN        8
#define ST67W61_MAX_PAYLOAD    1024
#define ST67W61_BUF_LEN        (ST67W61_HDR_LEN + ST67W61_MAX_PAYLOAD + 4)  /* +4 align pad */

#define ST67W61_OK_STR         "OK"
#define ST67W61_ERROR_STR      "ERROR"

/* AT commands — ST67W61 proprietary set.
 * Reference: UM3475 / ST67W611M1 AT Command Guide.
 * Verify command strings against the actual firmware version on the module. */
#define ST67W61_AT_TEST        "AT"
#define ST67W61_AT_RESET       "AT+RST"
#define ST67W61_AT_WFJAP       "AT+WFJAP"   /* +WFJAP="ssid","psk" — join AP */
#define ST67W61_AT_WFDAP       "AT+WFDAP"   /* disconnect from AP */
#define ST67W61_AT_WFSTAT      "AT+WFSTAT"  /* Wi-Fi link status */
#define ST67W61_AT_NSTAT       "AT+NSTAT"   /* network (IP) status */
#define ST67W61_AT_GETMAC      "AT+GETMAC"  /* get MAC address */
#define ST67W61_AT_NSTCP       "AT+NSTCP"   /* open TCP connection: +NSTCP="host",port */
#define ST67W61_AT_NSEND       "AT+NSEND"   /* send on TCP: +NSEND=id,len */
#define ST67W61_AT_NCLOSE      "AT+NCLOSE"  /* close TCP: +NCLOSE=id */

struct st67w61_config {
    struct spi_dt_spec  spi;
    struct gpio_dt_spec chip_en;  /* CHIP_EN: D5 = PE11, active-low in DTS (LOW=off, HIGH=on) */
    struct gpio_dt_spec boot;     /* BOOT:    D6 = PE9,  drive LOW for normal SPI AT mode */
    struct gpio_dt_spec rdy;      /* SPI_RDY: D3 = PE13, active-high, input */
};

struct st67w61_data {
    struct net_if        *iface;
    uint8_t               mac[6];
    struct k_mutex        mutex;
    struct k_work         connect_work;
    struct k_work         hw_init_work;  /* deferred: reset + AT init + MAC read */
    bool                  hw_ready;      /* set when hw_init_work completes OK */
    /* RDY interrupt — semaphore given on rising edge; never misses a short pulse */
    struct k_sem          rdy_sem;
    struct gpio_callback  rdy_cb;
    /* copy of params from mgmt_connect — work handler reads these */
    char            ssid[WIFI_SSID_MAX_LEN + 1];
    char            psk[65];
    uint8_t         security;
    bool            connected;
    uint16_t        tx_seq;
    uint8_t         tx_buf[ST67W61_BUF_LEN];
    uint8_t         rx_buf[ST67W61_BUF_LEN];
    /* Cached no-CS SPI config — stable pointer prevents Zephyr driver from
     * reconfiguring CFG2 (and clearing IOSWP) on every TX spi_transceive call. */
    struct spi_config spi_no_cs_cfg;
};

/* SPI transport layer (st67w61_spi.c) */
int  st67w61_spi_init(const struct device *dev);      /* fast: configure only */
void st67w61_spi_hw_reset(const struct device *dev);  /* slow: must run in work ctx */
int st67w61_spi_transact(const struct device *dev,
                          const uint8_t *payload, uint16_t tx_len,
                          uint8_t *resp_buf, uint16_t resp_cap,
                          k_timeout_t timeout);

/* AT command layer (st67w61_at.c) */
int st67w61_at_init(const struct device *dev);
int st67w61_at_cmd(const struct device *dev, const char *cmd,
                    char *resp, size_t resp_cap, k_timeout_t timeout);
int st67w61_at_connect(const struct device *dev,
                        const char *ssid, const char *psk);
int st67w61_at_disconnect(const struct device *dev);
int st67w61_at_get_mac(const struct device *dev, uint8_t mac[6]);
