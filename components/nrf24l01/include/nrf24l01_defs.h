#pragma once

#define NRF24_CMD_ACTIVATE_DATA 0x73
#define NRF24_MAX_PAYLOAD_SIZE  32
#define NRF24_PIPE_COUNT        6
#define NRF24_MAX_ADDRESS_WIDTH 5
#define NRF24_MIN_CHANNEL       0
#define NRF24_MAX_CHANNEL       125
#define NRF24_MAX_SPI_CLOCK_HZ  10000000
#define NRF24_POWER_ON_RESET_DELAY_MS 100U
#define NRF24_POWER_UP_DELAY_US       1500U
#define NRF24_STANDBY_TO_ACTIVE_US    130U
#define NRF24_CE_PULSE_US             15U

/* Memory Map */
#define NRF24_CONFIG 0x00
#define NRF24_EN_AA 0x01
#define NRF24_EN_RXADDR 0x02
#define NRF24_SETUP_AW 0x03
#define NRF24_SETUP_RETR 0x04
#define NRF24_RF_CH 0x05
#define NRF24_RF_SETUP 0x06
#define NRF24_STATUS 0x07
#define NRF24_OBSERVE_TX 0x08
#define NRF24_RPD 0x09
#define NRF24_RX_ADDR_P0 0x0A
#define NRF24_RX_ADDR_P1 0x0B
#define NRF24_RX_ADDR_P2 0x0C
#define NRF24_RX_ADDR_P3 0x0D
#define NRF24_RX_ADDR_P4 0x0E
#define NRF24_RX_ADDR_P5 0x0F
#define NRF24_TX_ADDR 0x10
#define NRF24_RX_PW_P0 0x11
#define NRF24_RX_PW_P1 0x12
#define NRF24_RX_PW_P2 0x13
#define NRF24_RX_PW_P3 0x14
#define NRF24_RX_PW_P4 0x15
#define NRF24_RX_PW_P5 0x16
#define NRF24_FIFO_STATUS 0x17
#define NRF24_DYNPD 0x1C
#define NRF24_FEATURE 0x1D

/* SPI Instruction Mnemonics */
#define NRF24_CMD_R_REGISTER 0x00 // Read command and status registers 5 bits Register MAp Address
#define NRF24_CMD_W_REGISTER 0x20 // Write command and status registers 5 bits Register MAp Address
#define NRF24_CMD_REGISTER_MASK 0x1F

#define NRF24_CMD_R_RX_PAYLOAD 0x61 // Read RX payliad
#define NRF24_CMD_W_TX_PAYLOAD 0xA0 // Write TX payload

#define NRF24_CMD_FLUSH_TX 0xE1    // Flush TX FIFO
#define NRF24_CMD_FLUSH_RX 0xE2    // Flush Rx FIFO
#define NRF24_CMD_REUSE_TX_PL 0xE3 // To transmit the same message

#define NRF24_CMD_ACTIVATE 0x50           // Followed by 0x73 to activate 3 registers
#define NRF24_CMD_R_RX_PL_WID 0x60        // Read RX-Payload width
#define NRF24_CMD_W_ACK_PAYLOAD 0xA8      // 1010 1PPP   PPP:Pipe
#define NRF24_CMD_W_ACK_PAYLOAD_PIPE_MASK 0x07
#define NRF24_CMD_W_TX_PAYLOAD_NOACK 0xB0 // Write TX payload and disable auto-ack

#define NRF24_CMD_NOP 0xFF // Nothing happens here, used to read the status

/* CONFIG register bits */
#define NRF24_CONFIG_PRIM_RX     (1U << 0)
#define NRF24_CONFIG_PWR_UP      (1U << 1)
#define NRF24_CONFIG_CRCO        (1U << 2)
#define NRF24_CONFIG_EN_CRC      (1U << 3)
#define NRF24_CONFIG_MASK_MAX_RT (1U << 4)
#define NRF24_CONFIG_MASK_TX_DS  (1U << 5)
#define NRF24_CONFIG_MASK_RX_DR  (1U << 6)

/* EN_AA, EN_RXADDR and DYNPD pipe bits */
#define NRF24_PIPE_BIT(pipe)     (1U << (pipe))
#define NRF24_ALL_PIPES_MASK     0x3FU

/* SETUP_AW register bits */
#define NRF24_SETUP_AW_MASK      0x03U

/* SETUP_RETR register bits */
#define NRF24_SETUP_RETR_ARC_MASK 0x0FU
#define NRF24_SETUP_RETR_ARD_POS  4U
#define NRF24_SETUP_RETR_ARD_MASK (0x0FU << NRF24_SETUP_RETR_ARD_POS)

/* RF_CH register bits */
#define NRF24_RF_CH_MASK          0x7FU

/* RF_SETUP register bits */
#define NRF24_RF_SETUP_RF_PWR_POS 1U
#define NRF24_RF_SETUP_RF_PWR_MASK (0x03U << NRF24_RF_SETUP_RF_PWR_POS)
#define NRF24_RF_SETUP_RF_DR_HIGH (1U << 3)
#define NRF24_RF_SETUP_PLL_LOCK   (1U << 4)
#define NRF24_RF_SETUP_RF_DR_LOW  (1U << 5)
#define NRF24_RF_SETUP_CONT_WAVE  (1U << 7)

/* STATUS register bits. Write one to clear an IRQ flag. */
#define NRF24_STATUS_TX_FULL     (1U << 0)
#define NRF24_STATUS_RX_P_NO_POS 1U
#define NRF24_STATUS_RX_P_NO_MASK (0x07U << NRF24_STATUS_RX_P_NO_POS)
#define NRF24_STATUS_MAX_RT      (1U << 4)
#define NRF24_STATUS_TX_DS       (1U << 5)
#define NRF24_STATUS_RX_DR       (1U << 6)
#define NRF24_STATUS_IRQ_MASK    (NRF24_STATUS_MAX_RT | NRF24_STATUS_TX_DS | NRF24_STATUS_RX_DR)

/* OBSERVE_TX register bits */
#define NRF24_OBSERVE_TX_ARC_CNT_MASK 0x0FU
#define NRF24_OBSERVE_TX_PLOS_CNT_POS 4U
#define NRF24_OBSERVE_TX_PLOS_CNT_MASK (0x0FU << NRF24_OBSERVE_TX_PLOS_CNT_POS)

/* RPD register bits */
#define NRF24_RPD_DETECTED       (1U << 0)

/* FIFO_STATUS register bits */
#define NRF24_FIFO_RX_EMPTY      (1U << 0)
#define NRF24_FIFO_RX_FULL       (1U << 1)
#define NRF24_FIFO_TX_EMPTY      (1U << 4)
#define NRF24_FIFO_TX_FULL       (1U << 5)
#define NRF24_FIFO_TX_REUSE      (1U << 6)

/* FEATURE register bits */
#define NRF24_FEATURE_EN_DYN_ACK (1U << 0)
#define NRF24_FEATURE_EN_ACK_PAY (1U << 1)
#define NRF24_FEATURE_EN_DPL     (1U << 2)

typedef enum
{
    NRF24_MODE_POWER_DOWN = 0,
    NRF24_MODE_STANDBY,
    NRF24_MODE_PTX,
    NRF24_MODE_PRX
} nrf24_mode_t;

typedef enum
{
    NRF24_PIPE_0 = 0,
    NRF24_PIPE_1,
    NRF24_PIPE_2,
    NRF24_PIPE_3,
    NRF24_PIPE_4,
    NRF24_PIPE_5
} nrf24_pipe_t;

// Register 3 SETUP_AW 
typedef enum
{
    NRF24_ADDRESS_WIDTH_3_BYTES = 1,
    NRF24_ADDRESS_WIDTH_4_BYTES,
    NRF24_ADDRESS_WIDTH_5_BYTES
} nrf24_address_width_t;

// Registro 4 SETUP_RETR //////////
typedef enum
{
    NRF24_RETRANSMIT_DELAY_250_US = 0,
    NRF24_RETRANSMIT_DELAY_500_US,
    NRF24_RETRANSMIT_DELAY_750_US,
    NRF24_RETRANSMIT_DELAY_1000_US,
    NRF24_RETRANSMIT_DELAY_1250_US,
    NRF24_RETRANSMIT_DELAY_1500_US,
    NRF24_RETRANSMIT_DELAY_1750_US,
    NRF24_RETRANSMIT_DELAY_2000_US,
    NRF24_RETRANSMIT_DELAY_2250_US,
    NRF24_RETRANSMIT_DELAY_2500_US,
    NRF24_RETRANSMIT_DELAY_2750_US,
    NRF24_RETRANSMIT_DELAY_3000_US,
    NRF24_RETRANSMIT_DELAY_3250_US,
    NRF24_RETRANSMIT_DELAY_3500_US,
    NRF24_RETRANSMIT_DELAY_3750_US,
    NRF24_RETRANSMIT_DELAY_4000_US
} nrf24_auto_retransmit_delay_t;

typedef enum
{
    NRF24_RETRANSMIT_DISABLED = 0,
    NRF24_RETRANSMIT_COUNT_1,
    NRF24_RETRANSMIT_COUNT_2,
    NRF24_RETRANSMIT_COUNT_3,
    NRF24_RETRANSMIT_COUNT_4,
    NRF24_RETRANSMIT_COUNT_5,
    NRF24_RETRANSMIT_COUNT_6,
    NRF24_RETRANSMIT_COUNT_7,
    NRF24_RETRANSMIT_COUNT_8,
    NRF24_RETRANSMIT_COUNT_9,
    NRF24_RETRANSMIT_COUNT_10,
    NRF24_RETRANSMIT_COUNT_11,
    NRF24_RETRANSMIT_COUNT_12,
    NRF24_RETRANSMIT_COUNT_13,
    NRF24_RETRANSMIT_COUNT_14,
    NRF24_RETRANSMIT_COUNT_15
} nrf24_auto_retransmit_count_t;


//Registro 6 RF_SETUP

typedef enum
{
    NRF24_DATA_RATE_1_MBPS = 0,
    NRF24_DATA_RATE_2_MBPS,
    NRF24_DATA_RATE_250_KBPS
} nrf24_air_data_rate_t;

typedef enum
{
    NRF24_OUTPUT_POWER_MINUS_18_DBM = 0,
    NRF24_OUTPUT_POWER_MINUS_12_DBM,
    NRF24_OUTPUT_POWER_MINUS_6_DBM,
    NRF24_OUTPUT_POWER_0_DBM,
} nrf24_output_power_t;


typedef enum
{
    NRF24_CRC_DISABLED = 0,
    NRF24_CRC_1_BYTE,
    NRF24_CRC_2_BYTES
} nrf24_crc_length_t;
