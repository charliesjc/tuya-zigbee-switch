#include "telink_size_t_hack.h"
#pragma pack(push, 1)
#include "tl_common.h"
#pragma pack(pop)

#include "hal/uart.h"

#include <string.h>

// Secondary-MCU UART for the ZTU (TLSR8258) module.
//
// The Tuya secondary MCU (e.g. Puya PY32F002A) is driven over a UART on the
// module's PB1 (TX to MCU) and PB7 (RX from MCU) pins. Both pins are part of
// the UART1 module's pinmux (PB1 = UART_TX_PB1, PB7 = UART_RX_PB7).
//
// RX uses the SDK's DMA receive path (drv_uart), which is wired into the
// main-build ISR via drv_uart_rx_irq_handler(). DMA moves incoming bytes into
// a RAM buffer as they arrive - the 8258's 4-byte hardware FIFO would
// otherwise overflow when the main loop is busy with the Zigbee stack. The
// ISR callback drains the DMA buffer into a software ring buffer, which
// hal_uart_read() consumes in the main loop.
//
// TX keeps the proven NDMA byte-push path (uart_ndma_send_byte).

#define MCU_UART_TX_PIN UART_TX_PB1
#define MCU_UART_RX_PIN UART_RX_PB7
#define MCU_UART_BAUDRATE 115200

/* DMA receive buffer used by drv_uart. The first 4 bytes hold the received
 * length (big-endian), the payload follows at +4, so the usable payload
 * capacity is (size - 4). Must be 4-byte aligned. */
#define UART_RX_DMA_BUF_SIZE 64
static uint8_t g_uart_rx_dma_buf[UART_RX_DMA_BUF_SIZE] __attribute__((aligned(4)));

/* Software ring buffer holding bytes drained from the DMA buffer in the ISR. */
#define UART_RX_RING_SIZE 256
static uint8_t g_uart_rx_ring[UART_RX_RING_SIZE];
static volatile uint16_t g_uart_rx_head = 0;
static volatile uint16_t g_uart_rx_tail = 0;

static uint16_t uart_ring_used(void) {
    uint16_t head = (uint16_t)g_uart_rx_head;
    uint16_t tail = (uint16_t)g_uart_rx_tail;
    return (uint16_t)(head - tail);
}

static void uart_ring_push(const uint8_t *data, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        /* Drop bytes if the ring is full rather than corrupting it. */
        if (uart_ring_used() >= (UART_RX_RING_SIZE - 1)) {
            return;
        }
        uint16_t idx = (uint16_t)(g_uart_rx_head % UART_RX_RING_SIZE);
        g_uart_rx_ring[idx] = data[i];
        g_uart_rx_head++;
    }
}

/* Called from the SDK ISR (drv_uart_rx_irq_handler) when DMA receives data. */
static void uart_dma_rx_callback(void) {
    uint32_t dataLen =
        ((uint32_t)g_uart_rx_dma_buf[0]) |
        ((uint32_t)g_uart_rx_dma_buf[1] << 8) |
        ((uint32_t)g_uart_rx_dma_buf[2] << 16) |
        ((uint32_t)g_uart_rx_dma_buf[3] << 24);

    if (dataLen && dataLen <= (UART_RX_DMA_BUF_SIZE - 4)) {
        uart_ring_push(&g_uart_rx_dma_buf[4], (uint16_t)dataLen);
    }
}

void hal_uart_init(const hal_uart_config_t *cfg) {
    (void)cfg;

    /* drv_uart_init sets up the DMA receive path and registers the ISR
     * callback. It enables both RX and TX DMA; we re-disable TX DMA below so
     * the NDMA TX path in hal_uart_write keeps working. */
    uart_gpio_set(MCU_UART_TX_PIN, MCU_UART_RX_PIN);
    if (drv_uart_init(MCU_UART_BAUDRATE, g_uart_rx_dma_buf,
                      UART_RX_DMA_BUF_SIZE, uart_dma_rx_callback) != 0) {
        /* Fall back to plain NDMA init so TX still works even if the DMA RX
         * setup failed. */
        uart_reset();
        uart_gpio_set(MCU_UART_TX_PIN, MCU_UART_RX_PIN);
        uart_init_baudrate(MCU_UART_BAUDRATE, CLOCK_SYS_CLOCK_HZ, PARITY_NONE,
                           STOP_BIT_ONE);
    }
    uart_dma_enable(1, 0); /* RX DMA on, TX stays NDMA */

    g_uart_rx_head = 0;
    g_uart_rx_tail = 0;
}

void hal_uart_deinit(void) {
    uart_reset();
    g_uart_rx_head = 0;
    g_uart_rx_tail = 0;
}

uint16_t hal_uart_rx_available(void) {
    return uart_ring_used();
}

void hal_uart_flush(void) {
    g_uart_rx_head = 0;
    g_uart_rx_tail = 0;
    uart_ndma_clear_tx_index();
}

hal_uart_status_t hal_uart_write(const uint8_t *data, uint16_t len,
                                 uint16_t *written) {
    if (!data && len != 0) {
        return HAL_UART_ERR_INVALID_ARG;
    }

    uint16_t actual = 0;
    for (uint16_t i = 0; i < len; i++) {
        // Wait for the previous byte to finish shifting out before writing the
        // next one. NDMA mode cycles through the four TX data registers.
        while (uart_tx_is_busy()) {
        }
        uart_ndma_send_byte(data[i]);
        actual++;
    }
    // Wait for the final byte to be fully transmitted before returning, so a
    // subsequent read/command isn't corrupted by an in-flight frame.
    while (uart_tx_is_busy()) {
    }

    if (written) {
        *written = actual;
    }
    return HAL_UART_OK;
}

hal_uart_status_t hal_uart_read(uint8_t *data, uint16_t len,
                                uint16_t *read_len) {
    if (!data && len != 0) {
        return HAL_UART_ERR_INVALID_ARG;
    }

    uint16_t actual = 0;
    while (actual < len && uart_ring_used() > 0) {
        uint16_t idx = (uint16_t)(g_uart_rx_tail % UART_RX_RING_SIZE);
        data[actual++] = g_uart_rx_ring[idx];
        g_uart_rx_tail++;
    }

    if (read_len) {
        *read_len = actual;
    }
    return HAL_UART_OK;
}

hal_uart_status_t hal_uart_write_byte(uint8_t byte) {
    uint16_t written = 0;
    return hal_uart_write(&byte, 1, &written);
}

hal_uart_status_t hal_uart_read_byte(uint8_t *byte) {
    if (!byte) {
        return HAL_UART_ERR_INVALID_ARG;
    }
    uint16_t read_len = 0;
    hal_uart_status_t st = hal_uart_read(byte, 1, &read_len);
    return read_len ? HAL_UART_OK : st;
}
