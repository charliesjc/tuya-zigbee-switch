#include "telink_size_t_hack.h"
#pragma pack(push, 1)
#include "tl_common.h"
#pragma pack(pop)

#include "hal/uart.h"
#include "hal/printf_selector.h"

// Secondary-MCU UART for the ZTU (TLSR8258) module.
//
// The Tuya secondary MCU (e.g. Puya PY32F002A) is driven over a UART on the
// module's PB1 (TX to MCU) and PB7 (RX from MCU) pins. Both pins are part of
// the UART1 module's pinmux (PB1 = UART_TX_PB1, PB7 = UART_RX_PB7). We use
// non-DMA (NDMA) polling mode: writes spin until the TX shift register is
// free, and reads drain the RX buffer count register.

#define MCU_UART_TX_PIN UART_TX_PB1
#define MCU_UART_RX_PIN UART_RX_PB7
#define MCU_UART_BAUDRATE 115200

void hal_uart_init(const hal_uart_config_t *cfg) {
    (void)cfg;

    uart_reset();
    uart_gpio_set(MCU_UART_TX_PIN, MCU_UART_RX_PIN);
    uart_init_baudrate(MCU_UART_BAUDRATE, CLOCK_SYS_CLOCK_HZ, PARITY_NONE,
                       STOP_BIT_ONE);
    uart_ndma_clear_tx_index();
}

void hal_uart_deinit(void) {
    uart_reset();
}

uint16_t hal_uart_rx_available(void) {
    return reg_uart_buf_cnt & FLD_UART_RX_BUF_CNT;
}

void hal_uart_flush(void) {
    // Clear the RX FIFO by draining whatever is pending.
    while (hal_uart_rx_available() > 0) {
        (void)reg_uart_data_buf0;
    }
    uart_ndma_clear_tx_index();
}

hal_uart_status_t hal_uart_write(const uint8_t *data, uint16_t len,
                                 uint16_t *written) {
    if (!data && len != 0) {
        return HAL_UART_ERR_INVALID_ARG;
    }

    printf("[UART] write %u bytes\r\n", (unsigned)len);
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
    while (actual < len && (reg_uart_buf_cnt & FLD_UART_RX_BUF_CNT)) {
        // The RX FIFO is spread across four hardware registers. The TX pointer
        // is unrelated to RX reads and was being used here, so we must read the
        // actual data buffer slots in sequence instead of reusing uart_TxIndex.
        data[actual++] = reg_uart_data_buf(actual % 4);
        // The hardware FIFO count is decremented by each successful read of the
        // data buffer; no extra state is needed beyond the read position.
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
