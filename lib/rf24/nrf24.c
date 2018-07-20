// Functions to manage the nRF24L01+ transceiver
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "nrf24.h"

#define nRF24_CSN_L() gpio_set_level(15, 0);
#define nRF24_CSN_H() gpio_set_level(15, 1);

#define CE_PIN          17

#ifndef CE_PIN
#error Pls define CE pin
#endif

#define nRF24_CE_H()    gpio_set_level(CE_PIN, 1);
#define nRF24_CE_L()    gpio_set_level(CE_PIN, 0);

static uint8_t dynamic_payloads_enabled = false;
static spi_device_handle_t g_spi = NULL;

void nRF24_CE_High(void)
{
    nRF24_CE_H();
}

void nRF24_CE_Low(void)
{
    nRF24_CE_L();
}


// Read a register
// input:
//   reg - number of register to read
// return: value of register
uint8_t nRF24_ReadReg(uint8_t reg)
{
    spi_transaction_t t;

    t.length = 16;
    t.rxlength = 0;
    t.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    t.tx_data[0] = reg & nRF24_MASK_REG_MAP;
    t.tx_data[1] = nRF24_CMD_NOP;

    nRF24_CSN_L();
    esp_err_t ret = spi_device_transmit(g_spi, &t);
    assert( ret == ESP_OK );
    nRF24_CSN_H();

    return t.rx_data[1];
}

// Write a new value to register
// input:
//   reg - number of register to write
//   value - value to write
static void nRF24_WriteReg(uint8_t reg, uint8_t value) {
    spi_transaction_t t;

    t.rxlength = 0;
    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;

    if (reg < nRF24_CMD_W_REGISTER) {
        // This is a register access
        t.length = 16;
        t.tx_data[0] = nRF24_CMD_W_REGISTER | (reg & nRF24_MASK_REG_MAP);
        t.tx_data[1] = value;
        // spiTransfer(nRF24_CMD_W_REGISTER | (reg & nRF24_MASK_REG_MAP));
        // spiTransfer(value);
    } else {
        // This is a single byte command or future command/register
        t.length = 8;
        t.tx_data[0] = reg;
        // spiTransfer(reg);
        if ((reg != nRF24_CMD_FLUSH_TX) && (reg != nRF24_CMD_FLUSH_RX) && \
            (reg != nRF24_CMD_REUSE_TX_PL) && (reg != nRF24_CMD_NOP)) {
                // Send register value
                t.length = 16;
                t.tx_data[1] = value;
                // spiTransfer(value);
        }
    }
    nRF24_CSN_L();
    esp_err_t ret = spi_device_transmit(g_spi, &t);
    assert( ret == ESP_OK );
    nRF24_CSN_H();
}

// Read a multi-byte register
// input:
//   reg - number of register to read
//   pBuf - pointer to the buffer for register data
//   count - number of bytes to read
static void nRF24_ReadMBReg(uint8_t reg, uint8_t *pBuf, uint8_t count) {
    esp_err_t ret = ESP_OK;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction

    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t.length = 8;
    t.rxlength = 0;
    t.tx_data[0] = reg;

    nRF24_CSN_L();
    ret = spi_device_transmit(g_spi, &t);
    assert( ret == ESP_OK );

    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length = count * 8;
    t.rx_buffer = (void*)pBuf;

    ret = spi_device_transmit(g_spi, &t);
    nRF24_CSN_H();
    assert( ret == ESP_OK );
    // spiTransfer(reg);
    // while (count--) {
    //     *pBuf++ = spiTransfer(nRF24_CMD_NOP);
    // }

}

// Write a multi-byte register
// input:
//   reg - number of register to write
//   pBuf - pointer to the buffer with data to write
//   count - number of bytes to write
static void nRF24_WriteMBReg(uint8_t reg, uint8_t *pBuf, uint8_t count) {
    esp_err_t ret = ESP_OK;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction

    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t.length = 8;
    t.rxlength = 0;
    t.tx_data[0] = reg;
    nRF24_CSN_L();
    ret = spi_device_transmit(g_spi, &t);
    assert( ret == ESP_OK );

    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length = count * 8;
    t.tx_buffer = (void*)pBuf;

    ret = spi_device_transmit(g_spi, &t);
    nRF24_CSN_H();
    assert( ret == ESP_OK );
    // spiTransfer(reg);
    // while (count--) {
    //     spiTransfer(*pBuf++);
    // }

}

// Set transceiver to it's initial state
// note: RX/TX pipe addresses remains untouched
void nRF24_Init(spi_device_handle_t _spi) {
    // hold the handle
    g_spi = _spi;

    // Write to registers their initial values
    nRF24_WriteReg(nRF24_REG_CONFIG, 0x08);
    // nRF24_CE_Low();
    //  nRF24_WriteReg(nRF24_REG_CONFIG, 0x0F);
    ESP_LOGI("test", "nRF24_REG_CONFIG: %x\n", nRF24_ReadReg(nRF24_REG_CONFIG));
    nRF24_WriteReg(nRF24_REG_EN_AA, 0x3F);
    ESP_LOGI("test", "nRF24_REG_EN_AA: %x\n", nRF24_ReadReg(nRF24_REG_EN_AA));
    nRF24_WriteReg(nRF24_REG_EN_RXADDR, 0x00);
    ESP_LOGI("test", "nRF24_REG_EN_RXADDR: %x\n", nRF24_ReadReg(nRF24_REG_EN_RXADDR));
    nRF24_WriteReg(nRF24_REG_SETUP_AW, 0x03);
    ESP_LOGI("test", "nRF24_REG_SETUP_AW: %x\n", nRF24_ReadReg(nRF24_REG_SETUP_AW));
    nRF24_WriteReg(nRF24_REG_SETUP_RETR, 0x03);
    ESP_LOGI("test", "nRF24_REG_SETUP_RETR: %x\n", nRF24_ReadReg(nRF24_REG_SETUP_RETR));
    nRF24_WriteReg(nRF24_REG_RF_CH, 0x01);
    ESP_LOGI("test", "nRF24_REG_RF_CH: %x\n", nRF24_ReadReg(nRF24_REG_RF_CH));
    nRF24_WriteReg(nRF24_REG_RF_SETUP, 0x0E);
    ESP_LOGI("test", "nRF24_REG_RF_SETUP: %x\n", nRF24_ReadReg(nRF24_REG_RF_SETUP));
    nRF24_WriteReg(nRF24_REG_STATUS, 0x00);
    nRF24_WriteReg(nRF24_REG_RX_PW_P0, 0x00);
    nRF24_WriteReg(nRF24_REG_RX_PW_P1, 0x00);
    nRF24_WriteReg(nRF24_REG_RX_PW_P2, 0x00);
    nRF24_WriteReg(nRF24_REG_RX_PW_P3, 0x00);
    nRF24_WriteReg(nRF24_REG_RX_PW_P4, 0x00);
    nRF24_WriteReg(nRF24_REG_RX_PW_P5, 0x00);
    nRF24_WriteReg(nRF24_REG_DYNPD, 0x00);
    nRF24_WriteReg(nRF24_REG_FEATURE, 0x00);

    // Clear the FIFO's
    nRF24_FlushRX();
    nRF24_FlushTX();

// Clear any pending interrupt flags
    nRF24_ClearIRQFlags();
    // nRF24_CE_High();

    // Deassert CSN pin (chip release)
    nRF24_CSN_H();
    dynamic_payloads_enabled = false;
}

// Check if the nRF24L01 present
// return:
//   1 - nRF24L01 is online and responding
//   0 - received sequence differs from original
uint8_t nRF24_Check(void) {
    uint8_t rxbuf[5];
    uint8_t i;
    uint8_t *ptr = (uint8_t *)nRF24_TEST_ADDR;

    // Write test TX address and read TX_ADDR register
    nRF24_WriteMBReg(nRF24_CMD_W_REGISTER | nRF24_REG_TX_ADDR, ptr, 5);

    nRF24_ReadMBReg(nRF24_CMD_R_REGISTER | nRF24_REG_TX_ADDR, rxbuf, 5);

    // Compare buffers, return error on first mismatch
    for (i = 0; i < 5; i++) {
        if (rxbuf[i] != *ptr++) return 0;
        // printf("%x %x\n", rxbuf[i] , *ptr++);
    }

    return 1;
}

// Control transceiver power mode
// input:
//   mode - new state of power mode, one of nRF24_PWR_xx values
void nRF24_SetPowerMode(uint8_t mode) {
    uint8_t reg;

    reg = nRF24_ReadReg(nRF24_REG_CONFIG);
    if (mode == nRF24_PWR_UP) {
        // Set the PWR_UP bit of CONFIG register to wake the transceiver
        // It goes into Stanby-I mode with consumption about 26uA
        reg |= nRF24_CONFIG_PWR_UP;
    } else {
        // Clear the PWR_UP bit of CONFIG register to put the transceiver
        // into power down mode with consumption about 900nA
        reg &= ~nRF24_CONFIG_PWR_UP;
    }
    nRF24_WriteReg(nRF24_REG_CONFIG, reg);
}

// Set transceiver operational mode
// input:
//   mode - operational mode, one of nRF24_MODE_xx values
void nRF24_SetOperationalMode(uint8_t mode) {
    uint8_t reg;

    // Configure PRIM_RX bit of the CONFIG register
    reg  = nRF24_ReadReg(nRF24_REG_CONFIG);
    reg &= ~nRF24_CONFIG_PRIM_RX;
    reg |= (mode & nRF24_CONFIG_PRIM_RX);
    nRF24_WriteReg(nRF24_REG_CONFIG, reg);
}

// Configure transceiver CRC scheme
// input:
//   scheme - CRC scheme, one of nRF24_CRC_xx values
// note: transceiver will forcibly turn on the CRC in case if auto acknowledgment
//       enabled for at least one RX pipe
void nRF24_SetCRCScheme(uint8_t scheme) {
    uint8_t reg;

    // Configure EN_CRC[3] and CRCO[2] bits of the CONFIG register
    reg  = nRF24_ReadReg(nRF24_REG_CONFIG);
    reg &= ~nRF24_MASK_CRC;
    reg |= (scheme & nRF24_MASK_CRC);
    nRF24_WriteReg(nRF24_REG_CONFIG, reg);
}

// Set frequency channel
// input:
//   channel - radio frequency channel, value from 0 to 127
// note: frequency will be (2400 + channel)MHz
// note: PLOS_CNT[7:4] bits of the OBSERVER_TX register will be reset
void nRF24_SetRFChannel(uint8_t channel) {
    nRF24_WriteReg(nRF24_REG_RF_CH, channel);
}

// Set automatic retransmission parameters
// input:
//   ard - auto retransmit delay, one of nRF24_ARD_xx values
//   arc - count of auto retransmits, value form 0 to 15
// note: zero arc value means that the automatic retransmission disabled
void nRF24_SetAutoRetr(uint8_t ard, uint8_t arc) {
    // Set auto retransmit settings (SETUP_RETR register)
    nRF24_WriteReg(nRF24_REG_SETUP_RETR, (uint8_t)((ard << 4) | (arc & nRF24_MASK_RETR_ARC)));
}

// Set of address widths
// input:
//   addr_width - RX/TX address field width, value from 3 to 5
// note: this setting is common for all pipes
void nRF24_SetAddrWidth(uint8_t addr_width) {
    nRF24_WriteReg(nRF24_REG_SETUP_AW, addr_width - 2);
}

// Set static RX address for a specified pipe
// input:
//   pipe - pipe to configure address, one of nRF24_PIPEx values
//   addr - pointer to the buffer with address
// note: pipe can be a number from 0 to 5 (RX pipes) and 6 (TX pipe)
// note: buffer length must be equal to current address width of transceiver
// note: for pipes[2..5] only first byte of address will be written because
//       other bytes of address equals to pipe1
// note: for pipes[2..5] only first byte of address will be written because
//       pipes 1-5 share the four most significant address bytes
void nRF24_SetAddr(uint8_t pipe, const uint8_t *addr) {
    uint8_t addr_width;

    // RX_ADDR_Px register
    switch (pipe) {
      case nRF24_PIPETX:
      case nRF24_PIPE0:
      case nRF24_PIPE1:
        // Get address width
        addr_width = nRF24_ReadReg(nRF24_REG_SETUP_AW) + 2;
        // Write address in reverse order (LSByte first)
        // addr += addr_width;
        nRF24_CSN_L();
        // spiTransfer(nRF24_CMD_W_REGISTER | nRF24_ADDR_REGS[pipe]);
        // do {
        //     spiTransfer(*addr--);
        // } while (addr_width--);

        spi_transaction_t t;
        t.length = 8;
        t.rxlength = 0;
        t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
        t.tx_data[0] = nRF24_CMD_W_REGISTER | nRF24_ADDR_REGS[pipe];
        esp_err_t ret = spi_device_transmit(g_spi, &t);
        assert( ret == ESP_OK );

        memset(&t, 0, sizeof(spi_transaction_t));
        t.length = 8 * addr_width;
        t.tx_buffer = (void*)addr; // FIX ME
        ret = spi_device_transmit(g_spi, &t);
        assert( ret == ESP_OK );
        nRF24_CSN_H();
        break;
      case nRF24_PIPE2:
      case nRF24_PIPE3:
      case nRF24_PIPE4:
      case nRF24_PIPE5:
        // Write address LSBbyte (only first byte from the addr buffer)
        nRF24_WriteReg(nRF24_ADDR_REGS[pipe], *addr);
        break;
      default:
        // Incorrect pipe number -> do nothing
        break;
    }
}

// Configure RF output power in TX mode
// input:
//   tx_pwr - RF output power, one of nRF24_TXPWR_xx values
void nRF24_SetTXPower(uint8_t tx_pwr) {
    uint8_t reg;

    // Configure RF_PWR[2:1] bits of the RF_SETUP register
    reg  = nRF24_ReadReg(nRF24_REG_RF_SETUP);
    reg &= ~nRF24_MASK_RF_PWR;
    reg |= tx_pwr;
    nRF24_WriteReg(nRF24_REG_RF_SETUP, reg);
}

// Configure transceiver data rate
// input:
//   data_rate - data rate, one of nRF24_DR_xx values
void nRF24_SetDataRate(uint8_t data_rate) {
    uint8_t reg;

    // Configure RF_DR_LOW[5] and RF_DR_HIGH[3] bits of the RF_SETUP register
    reg  = nRF24_ReadReg(nRF24_REG_RF_SETUP);
    reg &= ~nRF24_MASK_DATARATE;
    reg |= data_rate;
    nRF24_WriteReg(nRF24_REG_RF_SETUP, reg);
}

// Configure a specified RX pipe
// input:
//   pipe - number of the RX pipe, value from 0 to 5
//   aa_state - state of auto acknowledgment, one of nRF24_AA_xx values
//   payload_len - payload length in bytes
void nRF24_SetRXPipe(uint8_t pipe, uint8_t aa_state, uint8_t payload_len) {
    uint8_t reg;

    // Enable the specified pipe (EN_RXADDR register)
    reg = (nRF24_ReadReg(nRF24_REG_EN_RXADDR) | (1 << pipe)) & nRF24_MASK_EN_RX;
    nRF24_WriteReg(nRF24_REG_EN_RXADDR, reg);

    // Set RX payload length (RX_PW_Px register)
    nRF24_WriteReg(nRF24_RX_PW_PIPE[pipe], payload_len & nRF24_MASK_RX_PW);

    // Set auto acknowledgment for a specified pipe (EN_AA register)
    reg = nRF24_ReadReg(nRF24_REG_EN_AA);
    if (aa_state == nRF24_AA_ON) {
        reg |=  (1 << pipe);
    } else {
        reg &= ~(1 << pipe);
    }
    nRF24_WriteReg(nRF24_REG_EN_AA, reg);
}

// Disable specified RX pipe
// input:
//   PIPE - number of RX pipe, value from 0 to 5
void nRF24_ClosePipe(uint8_t pipe) {
    uint8_t reg;

    reg  = nRF24_ReadReg(nRF24_REG_EN_RXADDR);
    reg &= ~(1 << pipe);
    reg &= nRF24_MASK_EN_RX;
    nRF24_WriteReg(nRF24_REG_EN_RXADDR, reg);
}

// Enable the auto retransmit (a.k.a. enhanced ShockBurst) for the specified RX pipe
// input:
//   pipe - number of the RX pipe, value from 0 to 5
void nRF24_EnableAA(uint8_t pipe) {
    uint8_t reg;

    // Set bit in EN_AA register
    reg  = nRF24_ReadReg(nRF24_REG_EN_AA);
    reg |= (1 << pipe);
    nRF24_WriteReg(nRF24_REG_EN_AA, reg);
}

// Disable the auto retransmit (a.k.a. enhanced ShockBurst) for one or all RX pipes
// input:
//   pipe - number of the RX pipe, value from 0 to 5, any other value will disable AA for all RX pipes
void nRF24_DisableAA(uint8_t pipe) {
    uint8_t reg;

    if (pipe > 5) {
        // Disable Auto-ACK for ALL pipes
        nRF24_WriteReg(nRF24_REG_EN_AA, 0x00);
    } else {
        // Clear bit in the EN_AA register
        reg  = nRF24_ReadReg(nRF24_REG_EN_AA);
        reg &= ~(1 << pipe);
        nRF24_WriteReg(nRF24_REG_EN_AA, reg);
    }
}

// Get value of the STATUS register
// return: value of STATUS register
uint8_t nRF24_GetStatus(void) {
    return nRF24_ReadReg(nRF24_REG_STATUS);
}

// Get pending IRQ flags
// return: current status of RX_DR, TX_DS and MAX_RT bits of the STATUS register
uint8_t nRF24_GetIRQFlags(void) {
    return (nRF24_ReadReg(nRF24_REG_STATUS) & nRF24_MASK_STATUS_IRQ);
}

// Get status of the RX FIFO
// return: one of the nRF24_STATUS_RXFIFO_xx values
uint8_t nRF24_GetStatus_RXFIFO(void) {
    return (nRF24_ReadReg(nRF24_REG_FIFO_STATUS) & nRF24_MASK_RXFIFO);
}

// Get status of the TX FIFO
// return: one of the nRF24_STATUS_TXFIFO_xx values
// note: the TX_REUSE bit ignored
uint8_t nRF24_GetStatus_TXFIFO(void) {
    return ((nRF24_ReadReg(nRF24_REG_FIFO_STATUS) & nRF24_MASK_TXFIFO) >> 4);
}

// Get pipe number for the payload available for reading from RX FIFO
// return: pipe number or 0x07 if the RX FIFO is empty
uint8_t nRF24_GetRXSource(void) {
    return ((nRF24_ReadReg(nRF24_REG_STATUS) & nRF24_MASK_RX_P_NO) >> 1);
}

// Get auto retransmit statistic
// return: value of OBSERVE_TX register which contains two counters encoded in nibbles:
//   high - lost packets count (max value 15, can be reseted by write to RF_CH register)
//   low  - retransmitted packets count (max value 15, reseted when new transmission starts)
uint8_t nRF24_GetRetransmitCounters(void) {
    return (nRF24_ReadReg(nRF24_REG_OBSERVE_TX));
}

// Reset packet lost counter (PLOS_CNT bits in OBSERVER_TX register)
void nRF24_ResetPLOS(void) {
    uint8_t reg;

    // The PLOS counter is reset after write to RF_CH register
    reg = nRF24_ReadReg(nRF24_REG_RF_CH);
    nRF24_WriteReg(nRF24_REG_RF_CH, reg);
}

// Flush the TX FIFO
void nRF24_FlushTX(void) {
    nRF24_WriteReg(nRF24_CMD_FLUSH_TX, nRF24_CMD_NOP);
}

// Flush the RX FIFO
void nRF24_FlushRX(void) {
    nRF24_WriteReg(nRF24_CMD_FLUSH_RX, nRF24_CMD_NOP);
}

// Clear any pending IRQ flags
void nRF24_ClearIRQFlags(void) {
    uint8_t reg;

    // Clear RX_DR, TX_DS and MAX_RT bits of the STATUS register
    reg  = nRF24_ReadReg(nRF24_REG_STATUS);
    reg |= nRF24_MASK_STATUS_IRQ;
    nRF24_WriteReg(nRF24_REG_STATUS, reg);
}

// Write TX payload
// input:
//   pBuf - pointer to the buffer with payload data
//   length - payload length in bytes
void nRF24_WritePayload(uint8_t *pBuf, uint8_t length) {
    nRF24_WriteMBReg(nRF24_CMD_W_TX_PAYLOAD, pBuf, length);
}

// Read top level payload available in the RX FIFO
// input:
//   pBuf - pointer to the buffer to store a payload data
//   length - pointer to variable to store a payload length
// return: one of nRF24_RX_xx values
//   nRF24_RX_PIPEX - packet has been received from the pipe number X
//   nRF24_RX_EMPTY - the RX FIFO is empty
nRF24_RXResult nRF24_ReadPayload(uint8_t *pBuf, uint8_t *length) {
    uint8_t pipe;

    // Extract a payload pipe number from the STATUS register
    pipe = (nRF24_ReadReg(nRF24_REG_STATUS) & nRF24_MASK_RX_P_NO) >> 1;

    // RX FIFO empty?
    if (pipe < 6) {
        // Get payload length
        if(dynamic_payloads_enabled != true)
          *length = nRF24_ReadReg(nRF24_RX_PW_PIPE[pipe]);
        else
        {
            nRF24_CSN_L();
            // spiTransfer(nRF24_CMD_R_RX_PAYLOAD_WID);
            // *length = spiTransfer(nRF24_CMD_NOP);
            // nRF24_CSN_H();
           //*length = nRF24_ReadReg(nRF24_CMD_R_RX_PAYLOAD_WID);
            spi_transaction_t t;
            t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
            t.length = 16;
            t.rxlength = 0;
            t.tx_data[0] = nRF24_CMD_R_RX_PAYLOAD_WID;
            t.tx_data[1] = nRF24_CMD_NOP;
            esp_err_t ret = spi_device_transmit(g_spi, &t);
            assert( ret == ESP_OK );
            nRF24_CSN_H();
            *length = t.rx_data[1];
        }

        // Read a payload from the RX FIFO
        if (*length) {
            nRF24_ReadMBReg(nRF24_CMD_R_RX_PAYLOAD, pBuf, *length);
        }
        nRF24_FlushRX();

        return ((nRF24_RXResult)pipe);
    }

    // The RX FIFO is empty
    *length = 0;

    return nRF24_RX_EMPTY;
}


  /**
   * Enable custom payloads on the acknowledge packets
   *
   * Ack payloads are a handy way to return data back to senders without
   * manually changing the radio modes on both units.
   *
   * @note Ack payloads are dynamic payloads. This only works on pipes 0&1 by default. Call
   * enableDynamicPayloads() to enable on all pipes.
   */
void nRF24_EnableAckPayload(void)
{
    uint8_t reg;
    reg  = nRF24_ReadReg(nRF24_REG_FEATURE);
    reg |= (uint8_t)(1<<nRF24_EN_ACK_PAY);
    reg |= (uint8_t)(1<<nRF24_EN_DPL);

    nRF24_WriteReg(nRF24_REG_FEATURE,reg);

    // Enable dynamic payload on pipes 0 & 1
    reg  = nRF24_ReadReg(nRF24_REG_DYNPD);
    reg |= (uint8_t)(1<<nRF24_PIPE0);
    reg |= (uint8_t)(1<<nRF24_PIPE1);

    nRF24_WriteReg(nRF24_REG_DYNPD,reg);
    dynamic_payloads_enabled = true;
}

void nRF24_WriteAckPayload(uint8_t pipe, uint8_t *pBuf, uint8_t len)
{
    if(len >=32)
        len = 32;
    nRF24_WriteMBReg((nRF24_CMD_W_ACK_PAYLOAD | (pipe & 0x07)), pBuf, len);
}

// Function to transmit data packet
// input:
//   pBuf - pointer to the buffer with data to transmit
//   length - length of the data buffer in bytes
// return: one of nRF24_TX_xx values
nRF24_TXResult nRF24_TransmitPacket(uint8_t *pBuf, uint8_t length) {
    volatile uint32_t wait = 5;
    uint8_t status;
    // Deassert the CE pin (in case if it still high)
    nRF24_CE_L();
    // Transfer a data from the specified buffer to the TX FIFO
    nRF24_WritePayload(pBuf, length);
    // Start a transmission by asserting CE pin (must be held at least 10us)
    nRF24_CE_H();
    // Poll the transceiver status register until one of the following flags will be set:
    //   TX_DS  - means the packet has been transmitted
    //   MAX_RT - means the maximum number of TX retransmits happened
    // note: this solution is far from perfect, better to use IRQ instead of polling the status
    // Should put this function into IRQ

    do {
        status = nRF24_GetStatus();
        if (status & (nRF24_FLAG_TX_DS | nRF24_FLAG_MAX_RT)) {
            break;
        }
        vTaskDelay(10 / portTICK_RATE_MS);
    } while (wait--);

    // Deassert the CE pin (Standby-II --> Standby-I)
    nRF24_CE_L();

    if (!wait) {
        // Timeout
        return nRF24_TX_TIMEOUT;
    }

    // Clear pending IRQ flags
    nRF24_ClearIRQFlags();
    if (status & nRF24_FLAG_MAX_RT) {
        // Auto retransmit counter exceeds the programmed maximum limit (FIFO is not removed)
        return nRF24_TX_MAXRT;
    }
    if (status & nRF24_FLAG_TX_DS) {
        // Successful transmission
        return nRF24_TX_SUCCESS;
    }
    // Some banana happens, a payload remains in the TX FIFO, flush it
    nRF24_FlushTX();
    return nRF24_TX_ERROR;
}

/****************************************************************************/

void nRF24_enableDynamicPayloads(uint8_t pipe)
{
    uint8_t reg;
    reg  = nRF24_ReadReg(nRF24_REG_FEATURE);
    reg |= (uint8_t)(1<<nRF24_EN_DPL);
    // Enable dynamic payload throughout the system
    nRF24_WriteReg(nRF24_REG_FEATURE,reg);
    // Enable dynamic payload on all pipes
    //
    // Not sure the use case of only having dynamic payload on certain
    // pipes, so the library does not support it.
    reg  = nRF24_ReadReg(nRF24_REG_DYNPD);
    reg |= (uint8_t)(1<<pipe);
    nRF24_WriteReg(nRF24_REG_DYNPD,reg);

    dynamic_payloads_enabled = true;
}

/****************************************************************************/
void nRF24_disableDynamicPayloads(void)
{
    // Disables dynamic payload throughout the system.  Also disables Ack Payloads
    nRF24_WriteReg(nRF24_REG_FEATURE, 0);

    // Disable dynamic payload on all pipes
    //
    // Not sure the use case of only having dynamic payload on certain
    // pipes, so the library does not support it.
    nRF24_WriteReg(nRF24_REG_DYNPD,0);
    dynamic_payloads_enabled = false;
}

