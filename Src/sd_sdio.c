#include "sd_sdio.h"
#include <stdint.h>
#include <string.h>
#include "stm32l4xx_hal.h"   // for HAL_Delay

/* System core / SDMMC kernel clock – provided by system_stm32l4xx.c */
extern uint32_t SystemCoreClock;

/* UART debug from main.c */
extern void UART_print(char *str);
extern void UART_printf(const char *fmt, ...);

/* --- Local silent wrappers to preserve timing but hide output --- */
static void sd_silent_print(const char *s)
{
    (void)s;
    // Approximate time UART printing used to take
    HAL_Delay(1);
}

static void sd_silent_printf(const char *fmt, ...)
{
    (void)fmt;
    // Approximate time UART printing used to take
    HAL_Delay(1);
}

/* Redirect all UART_print / UART_printf calls in THIS file only */
#undef UART_print
#undef UART_printf
#define UART_print  sd_silent_print
#define UART_printf sd_silent_printf

/* -------- SDMMC helper defines (from RM0351 + stm32l476xx.h) -------- */

/* Power control: PWRCTRL[1:0] = 0b11 => power ON */
#ifndef SDMMC_POWER_PWRCTRL
#define SDMMC_POWER_PWRCTRL      (0x3U)
#endif

/* Some handy response encodings */
#define SDMMC_WAIT_NO_RESP       (0U)
#define SDMMC_WAIT_SHORT_RESP    (SDMMC_CMD_WAITRESP_0)
#define SDMMC_WAIT_LONG_RESP     (SDMMC_CMD_WAITRESP_1)

/* Data control helper: block size is log2(512) = 9 => shift into DBLOCKSIZE bits [7:4] */
#define SDMMC_DCTRL_DBLOCKSIZE_512B   (9U << 4)

/* Command indices */
#define SD_CMD_GO_IDLE_STATE         0U   /* CMD0  */
#define SD_CMD_SEND_IF_COND          8U   /* CMD8  */
#define SD_CMD_APP_CMD               55U  /* CMD55 */
#define SD_ACMD_SD_SEND_OP_COND      41U  /* ACMD41 */
#define SD_CMD_ALL_SEND_CID          2U   /* CMD2  */
#define SD_CMD_SEND_REL_ADDR         3U   /* CMD3  */
#define SD_CMD_SELECT_CARD           7U   /* CMD7  */
#define SD_CMD_SET_BLOCKLEN          16U  /* CMD16 */
#define SD_CMD_READ_SINGLE_BLOCK     17U  /* CMD17 */
#define SD_CMD_WRITE_SINGLE_BLOCK    24U  /* CMD24 */
#define SD_CMD_SEND_STATUS           13U  /* CMD13 */

/* Timeouts (simple loop-based; tune if needed) */
#define SDMMC_CMD_TIMEOUT_LOOPS   (0x00FFFFFFUL)
#define SDMMC_DATA_TIMEOUT_LOOPS  (0x00FFFFFFUL)

/* Card type */
#define CARDTYPE_UNKNOWN 0
#define CARDTYPE_SDSC    1
#define CARDTYPE_SDHC    2

/* Internal state */
static uint16_t s_rca           = 0;   /* Relative Card Address */
static uint8_t  s_card_type     = CARDTYPE_UNKNOWN;
static uint8_t  s_initialized   = 0;

/* Single 512-byte aligned buffer for SDMMC transfers */
static uint8_t s_sd_buf[512] __attribute__((aligned(4)));

/* -------- small busy-wait delay (very rough) -------- */
static void sd_delay_loops(volatile uint32_t loops)
{
    while (loops--) {
        __NOP();
    }
}

/* -------- SDMMC1 GPIO / Clock init (1-bit mode) -------- */
/* PC8 = D0, PC12 = CK, PD2 = CMD */

static void SDMMC1_GPIO_Init_1bit(void)
{
    /* Enable GPIOC and GPIOD clocks */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN | RCC_AHB2ENR_GPIODEN;

    /* PC8: D0, AF12, very high speed, pull-up */
    GPIOC->MODER   &= ~(GPIO_MODER_MODE8_Msk);
    GPIOC->MODER   |=  (0x2U << GPIO_MODER_MODE8_Pos);   /* AF */
    GPIOC->OTYPER  &= ~(GPIO_OTYPER_OT8);
    GPIOC->OSPEEDR |=  (0x3U << GPIO_OSPEEDR_OSPEED8_Pos);
    GPIOC->PUPDR   &= ~(GPIO_PUPDR_PUPD8_Msk);
    GPIOC->PUPDR   |=  (0x1U << GPIO_PUPDR_PUPD8_Pos);   /* pull-up */
    GPIOC->AFR[1]  &= ~(0xFU << ((8U - 8U) * 4U));
    GPIOC->AFR[1]  |=  (12U  << ((8U - 8U) * 4U));       /* AF12 */

    /* PC12: CK, AF12, very high speed, no pull */
    GPIOC->MODER   &= ~(GPIO_MODER_MODE12_Msk);
    GPIOC->MODER   |=  (0x2U << GPIO_MODER_MODE12_Pos);  /* AF */
    GPIOC->OTYPER  &= ~(GPIO_OTYPER_OT12);
    GPIOC->OSPEEDR |=  (0x3U << GPIO_OSPEEDR_OSPEED12_Pos);
    GPIOC->PUPDR   &= ~(GPIO_PUPDR_PUPD12_Msk);
    GPIOC->AFR[1]  &= ~(0xFU << ((12U - 8U) * 4U));
    GPIOC->AFR[1]  |=  (12U  << ((12U - 8U) * 4U));      /* AF12 */

    /* PD2: CMD, AF12, very high speed, pull-up */
    GPIOD->MODER   &= ~(GPIO_MODER_MODE2_Msk);
    GPIOD->MODER   |=  (0x2U << GPIO_MODER_MODE2_Pos);   /* AF */
    GPIOD->OTYPER  &= ~(GPIO_OTYPER_OT2);
    GPIOD->OSPEEDR |=  (0x3U << GPIO_OSPEEDR_OSPEED2_Pos);
    GPIOD->PUPDR   &= ~(GPIO_PUPDR_PUPD2_Msk);
    GPIOD->PUPDR   |=  (0x1U << GPIO_PUPDR_PUPD2_Pos);   /* pull-up */
    GPIOD->AFR[0]  &= ~(0xFU << (2U * 4U));
    GPIOD->AFR[0]  |=  (12U  << (2U * 4U));              /* AF12 */
}

/* -------- SDMMC1 clock enable -------- */
static void SDMMC1_PeripheralClockEnable(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_SDMMC1EN;
}

/* -------- helper: clear all SDMMC status flags -------- */
static void sdmmc_clear_flags(void)
{
    SDMMC1->ICR = 0xFFFFFFFFUL;
}

/* -------- helper: send command and wait for completion --------
 * waitresp: SDMMC_WAIT_NO_RESP / _SHORT_RESP / _LONG_RESP
 * If resp_ptr != NULL and response is short (R1/R3/R6/R7) or OCR-like,
 * RESP1 is returned there.
 * Return 0 on success, non-zero on error.
 */
static int sdmmc_send_cmd(uint8_t cmd_idx, uint32_t arg, uint32_t waitresp, uint32_t *resp_ptr)
{
    uint32_t sta;
    uint32_t timeout = SDMMC_CMD_TIMEOUT_LOOPS;

    /* Clear previous command flags */
    SDMMC1->ICR = SDMMC_ICR_CCRCFAILC |
                  SDMMC_ICR_CTIMEOUTC |
                  SDMMC_ICR_CMDRENDC  |
                  SDMMC_ICR_CMDSENTC;

    /* Program argument */
    SDMMC1->ARG = arg;

    /* Program command index + CPSM enable + response type */
    SDMMC1->CMD = ((uint32_t)cmd_idx & 0x3FU) |
                  waitresp |
                  SDMMC_CMD_CPSMEN;

    while (timeout--) {
        sta = SDMMC1->STA;

        /* Real (hardware) timeout */
        if (sta & SDMMC_STA_CTIMEOUT) {
            SDMMC1->ICR = SDMMC_ICR_CTIMEOUTC;
            UART_printf("sdmmc_send_cmd: CMD%u CTIMEOUT, STA=0x%08lX\r\n",
                        (unsigned)cmd_idx, (unsigned long)sta);
            return -1;
        }

        /* CRC failure – for some commands (e.g. R3/R7) CRC is not meaningful.
           We treat CCRCFAIL as "command completed" in those cases. */
        if (sta & SDMMC_STA_CCRCFAIL) {
            SDMMC1->ICR = SDMMC_ICR_CCRCFAILC;
            UART_printf("sdmmc_send_cmd: CMD%u CRCFAIL (treating as complete), STA=0x%08lX\r\n",
                        (unsigned)cmd_idx, (unsigned long)sta);
            break;
        }

        if (sta & SDMMC_STA_CMDREND) {
            SDMMC1->ICR = SDMMC_ICR_CMDRENDC;
            break;
        }

        if (sta & SDMMC_STA_CMDSENT) {
            SDMMC1->ICR = SDMMC_ICR_CMDSENTC;
            break;
        }
    }

    if (!timeout) {
        UART_printf("sdmmc_send_cmd: CMD%u SW timeout, STA=0x%08lX\r\n",
                    (unsigned)cmd_idx, (unsigned long)SDMMC1->STA);
        SDMMC1->ICR = SDMMC_ICR_CTIMEOUTC |
                      SDMMC_ICR_CCRCFAILC |
                      SDMMC_ICR_CMDRENDC  |
                      SDMMC_ICR_CMDSENTC;
        return -2;
    }

    if (resp_ptr && (waitresp != SDMMC_WAIT_NO_RESP)) {
        *resp_ptr = SDMMC1->RESP1;
    }

    return 0;
}

/* -------- helper: application-specific commands (ACMDxx) -------- */
static int sdmmc_send_app_cmd(uint8_t acmd_idx, uint32_t arg, uint32_t *resp_ptr)
{
    int rc;
    uint32_t r1;

    rc = sdmmc_send_cmd(SD_CMD_APP_CMD,
                        ((uint32_t)s_rca) << 16,
                        SDMMC_WAIT_SHORT_RESP,
                        &r1);
    if (rc != 0) return rc;

    rc = sdmmc_send_cmd(acmd_idx, arg, SDMMC_WAIT_SHORT_RESP, resp_ptr);
    return rc;
}

/* -------- helper: configure SDMMC clock -------- */
/* Compute divider for identification phase (<= 400kHz) */
static uint8_t sdmmc_compute_init_clkdiv(void)
{
    uint32_t sdclk  = SystemCoreClock;
    uint32_t target = 400000UL;         /* 400 kHz max for identification */

    if (sdclk <= target) {
        return 0;
    }

    /* From RM: SDCLK = sdclk / (2 * (CLKDIV + 2)) */
    uint32_t div = sdclk / (2UL * target);
    if (div <= 2) {
        return 0;
    }

    div -= 2;
    if (div > 0xFFU) div = 0xFFU;

    return (uint8_t)div;
}

/* Conservative data phase divider */
static uint8_t sdmmc_data_clkdiv(void)
{
    /* You can tune this later; start slow for signal integrity margin */
    return 4;  /* SDCLK = sdclk / (2 * (4 + 2)) = sdclk / 12 */
}

static void sdmmc_set_clock_divider(uint8_t clkdiv)
{
    SDMMC1->CLKCR &= ~(SDMMC_CLKCR_CLKEN);

    SDMMC1->CLKCR = (SDMMC1->CLKCR & ~(SDMMC_CLKCR_CLKDIV |
                                       SDMMC_CLKCR_WIDBUS |
                                       SDMMC_CLKCR_BYPASS |
                                       SDMMC_CLKCR_NEGEDGE |
                                       SDMMC_CLKCR_HWFC_EN)) |
                    ((uint32_t)clkdiv & SDMMC_CLKCR_CLKDIV);

    SDMMC1->CLKCR |= SDMMC_CLKCR_CLKEN;
}

/* -------- helper: card power-on / power-off -------- */
static void sdmmc_power_on(void)
{
    SDMMC1->POWER = SDMMC_POWER_PWRCTRL;   /* power on */
    sd_delay_loops(1000000UL);
}

static void sdmmc_power_off(void)
{
    SDMMC1->POWER = 0;
    sd_delay_loops(100000UL);
}

/* -------- card initialization sequence -------- */
static int sdmmc_card_init(void)
{
    int rc;
    uint32_t resp;
    uint8_t init_div = sdmmc_compute_init_clkdiv();

    UART_print("sdmmc_card_init: selecting MSI as SDMMC1 clock source\r\n");
    RCC->CCIPR &= ~RCC_CCIPR_CLK48SEL;
    RCC->CCIPR |=  RCC_CCIPR_CLK48SEL;   /* 0b11: MSI as SDMMC1 clock */

    UART_print("sdmmc_card_init: enabling SDMMC1 APB2 clock\r\n");
    RCC->APB2ENR |= RCC_APB2ENR_SDMMC1EN;
    sd_delay_loops(1000);

    UART_print("sdmmc_card_init: resetting SDMMC1\r\n");
    RCC->APB2RSTR |= RCC_APB2RSTR_SDMMC1RST;
    sd_delay_loops(1000);
    RCC->APB2RSTR &= ~RCC_APB2RSTR_SDMMC1RST;

    UART_print("sdmmc_card_init: powering up SDMMC1\r\n");
    SDMMC1->POWER = SDMMC_POWER_PWRCTRL;
    sd_delay_loops(10000);

    UART_print("sdmmc_card_init: setting init clock divider\r\n");
    sdmmc_set_clock_divider(init_div);

    /* CMD0 */
    rc = sdmmc_send_cmd(SD_CMD_GO_IDLE_STATE, 0, SDMMC_WAIT_NO_RESP, NULL);
    if (rc != 0) return rc;

    /* CMD8 */
    rc = sdmmc_send_cmd(SD_CMD_SEND_IF_COND,
                        0x1AA,
                        SDMMC_WAIT_SHORT_RESP,
                        &resp);
    if (rc != 0) {
        UART_print("sdmmc_card_init: CMD8 failed, maybe v1.x card\r\n");
    } else {
        if ((resp & 0xFFF) != 0x1AA) {
            UART_print("sdmmc_card_init: CMD8 pattern mismatch\r\n");
        }
    }

    UART_print("sdmmc_card_init: sending ACMD41 until card is ready\r\n");
    uint32_t ocr;
    uint32_t timeout = SDMMC_CMD_TIMEOUT_LOOPS;

    s_rca       = 0;
    s_card_type = CARDTYPE_UNKNOWN;

    do {
        rc = sdmmc_send_app_cmd(SD_ACMD_SD_SEND_OP_COND,
                                0x40FF8000UL,
                                &ocr);
        if (rc != 0) {
            UART_print("sdmmc_card_init: ACMD41 error\r\n");
            return rc;
        }

        if (ocr & 0x80000000UL) {
            break;
        }
    } while (--timeout);

    if (!timeout) {
        UART_print("sdmmc_card_init: ACMD41 timeout\r\n");
        return -1;
    }

    if (ocr & (1UL << 30)) {
        s_card_type = CARDTYPE_SDHC;
    } else {
        s_card_type = CARDTYPE_SDSC;
    }

    UART_print("sdmmc_card_init: CMD2 (ALL_SEND_CID)\r\n");
    rc = sdmmc_send_cmd(SD_CMD_ALL_SEND_CID,
                        0,
                        SDMMC_WAIT_LONG_RESP,
                        NULL);
    if (rc != 0) return rc;

    UART_print("sdmmc_card_init: CMD3 (SEND_REL_ADDR)\r\n");
    rc = sdmmc_send_cmd(SD_CMD_SEND_REL_ADDR,
                        0,
                        SDMMC_WAIT_SHORT_RESP,
                        &resp);
    if (rc != 0) return rc;

    s_rca = (uint16_t)(resp >> 16);

    UART_print("sdmmc_card_init: CMD7 (SELECT_CARD)\r\n");
    rc = sdmmc_send_cmd(SD_CMD_SELECT_CARD,
                        ((uint32_t)s_rca) << 16,
                        SDMMC_WAIT_SHORT_RESP,
                        &resp);
    if (rc != 0) return rc;

    if (s_card_type == CARDTYPE_SDSC) {
        rc = sdmmc_send_cmd(SD_CMD_SET_BLOCKLEN, 512U, SDMMC_WAIT_SHORT_RESP, &resp);
        if (rc != 0) return rc;
    }

    sdmmc_set_clock_divider(sdmmc_data_clkdiv());

    return 0;
}

/* -------- helper: wait until card is READY_FOR_DATA and in TRAN state -------- */
static int sdmmc_wait_card_ready(void)
{
    uint32_t resp;
    uint32_t timeout = 0x00FFFFFFUL;

    while (timeout--) {
        int rc = sdmmc_send_cmd(SD_CMD_SEND_STATUS,
                                ((uint32_t)s_rca) << 16,
                                SDMMC_WAIT_SHORT_RESP,
                                &resp);
        if (rc != 0) {
            UART_printf("sdmmc_wait_card_ready: CMD13 rc=%d\r\n", rc);
            return rc;
        }

        uint32_t ready = (resp >> 8) & 1U;   /* READY_FOR_DATA */
        uint32_t state = (resp >> 9) & 0xFU; /* CURRENT_STATE */

        if (ready && (state == 4U)) {  /* 4 = TRAN state */
            return 0;
        }
    }

    UART_print("sdmmc_wait_card_ready: timeout\r\n");
    return -1;
}

/* -------- low-level block read -------- */
static int sdmmc_read_block_raw(uint8_t *buff, uint32_t sector)
{
    int rc;
    uint32_t addr;

    UART_printf("sdmmc_read_block_raw: sector=%lu\r\n",
                (unsigned long)sector);

    if (!buff) {
        UART_printf("sdmmc_read_block_raw: NULL buffer\r\n");
        return 0;
    }

    if (s_card_type == CARDTYPE_SDHC) {
        addr = sector;
    } else {
        addr = sector * 512UL;
    }
    UART_printf("sdmmc_read_block_raw: addr=0x%08lX (type=%u)\r\n",
                (unsigned long)addr, (unsigned)s_card_type);

    /* Clear data & error flags */
    SDMMC1->ICR = SDMMC_ICR_DCRCFAILC |
                  SDMMC_ICR_DTIMEOUTC |
                  SDMMC_ICR_DATAENDC  |
                  SDMMC_ICR_RXOVERRC  |
                  SDMMC_ICR_TXUNDERRC |
                  SDMMC_ICR_STBITERRC;

    /* Configure data path: one block of 512 bytes, from card to host */
    SDMMC1->DTIMER = 0xFFFFFFFFUL;
    SDMMC1->DLEN   = 512UL;
    SDMMC1->DCTRL  = SDMMC_DCTRL_DBLOCKSIZE_512B |
                     SDMMC_DCTRL_DTDIR |        /* from card to controller */
                     SDMMC_DCTRL_DTEN;

    UART_printf("sdmmc_read_block_raw: issuing CMD17\r\n");
    rc = sdmmc_send_cmd(SD_CMD_READ_SINGLE_BLOCK,
                        addr,
                        SDMMC_WAIT_SHORT_RESP,
                        NULL);
    if (rc != 0) {
        UART_print("sdmmc_read_block_raw: CMD17 failed\r\n");
        return 0;
    }

    uint32_t *p32            = (uint32_t *)s_sd_buf;
    uint32_t words_remaining = 512U / 4U;
    uint32_t timeout         = SDMMC_DATA_TIMEOUT_LOOPS;

    while (words_remaining && timeout) {
        uint32_t sta = SDMMC1->STA;

        if (sta & SDMMC_STA_DTIMEOUT) {
            SDMMC1->ICR = SDMMC_ICR_DTIMEOUTC;
            UART_print("sdmmc_read_block_raw: DTIMEOUT\r\n");
            return 0;
        }
        if (sta & SDMMC_STA_DCRCFAIL) {
            SDMMC1->ICR = SDMMC_ICR_DCRCFAILC;
            UART_print("sdmmc_read_block_raw: DCRCFAIL\r\n");
            return 0;
        }
        if (sta & SDMMC_STA_RXOVERR) {
            SDMMC1->ICR = SDMMC_ICR_RXOVERRC;
            UART_print("sdmmc_read_block_raw: RXOVERR\r\n");
            return 0;
        }
        if (sta & SDMMC_STA_STBITERR) {
            SDMMC1->ICR = SDMMC_ICR_STBITERRC;
            UART_print("sdmmc_read_block_raw: STBITERR\r\n");
            return 0;
        }

        if (sta & SDMMC_STA_RXDAVL) {  /* data available in FIFO */
            *p32++ = SDMMC1->FIFO;
            words_remaining--;
        } else {
            timeout--;
        }
    }

    if (!timeout || words_remaining) {
        UART_print("sdmmc_read_block_raw: timeout while receiving\r\n");
        return 0;
    }

    timeout = SDMMC_DATA_TIMEOUT_LOOPS;
    while (!(SDMMC1->STA & SDMMC_STA_DATAEND) && --timeout) {
        /* wait */
    }
    SDMMC1->ICR = SDMMC_ICR_DATAENDC;

    /* Copy from aligned internal buffer to user buffer */
    memcpy(buff, s_sd_buf, 512);

    UART_printf("sdmmc_read_block_raw: DONE\r\n");
    return 1;
}

/* -------- low-level block write -------- */
static int sdmmc_write_block_raw(const uint8_t *buff, uint32_t sector)
{
    int rc;
    uint32_t addr;

    if (!buff) {
        return 0;
    }

    if (s_card_type == CARDTYPE_SDHC) {
        addr = sector;
    } else {
        addr = sector * 512UL;
    }

    /* Ensure the card is ready before starting a new write */
    if (sdmmc_wait_card_ready() != 0) {
        UART_print("sdmmc_write_block_raw: card not ready before write\r\n");
        return 0;
    }

    /* Copy from user buffer to aligned internal buffer */
    memcpy(s_sd_buf, buff, 512);

    /* Clear previous data & error flags */
    SDMMC1->ICR = SDMMC_ICR_DCRCFAILC |
                  SDMMC_ICR_DTIMEOUTC |
                  SDMMC_ICR_DATAENDC  |
                  SDMMC_ICR_RXOVERRC  |
                  SDMMC_ICR_TXUNDERRC |
                  SDMMC_ICR_STBITERRC;

    /* Configure one 512-byte write transfer */
    SDMMC1->DTIMER = 0xFFFFFFFFUL;
    SDMMC1->DLEN   = 512UL;
    SDMMC1->DCTRL  = SDMMC_DCTRL_DBLOCKSIZE_512B |
                     SDMMC_DCTRL_DTEN;        /* write: controller -> card */

    /* CMD24: WRITE_SINGLE_BLOCK */
    rc = sdmmc_send_cmd(SD_CMD_WRITE_SINGLE_BLOCK,
                        addr,
                        SDMMC_WAIT_SHORT_RESP,
                        NULL);
    if (rc != 0) {
        UART_print("sdmmc_write_block_raw: CMD24 failed (cmd error)\r\n");
        return 0;
    }

    uint32_t *p32            = (uint32_t *)s_sd_buf;
    uint32_t timeout         = SDMMC_DATA_TIMEOUT_LOOPS;
    uint32_t words_remaining = 512U / 4U;

    while (words_remaining && timeout) {
        uint32_t sta = SDMMC1->STA;

        if (sta & SDMMC_STA_DTIMEOUT) {
            SDMMC1->ICR = SDMMC_ICR_DTIMEOUTC;
            UART_print("sdmmc_write_block_raw: DTIMEOUT\r\n");
            return 0;
        }
        if (sta & SDMMC_STA_DCRCFAIL) {
            SDMMC1->ICR = SDMMC_ICR_DCRCFAILC;
            UART_print("sdmmc_write_block_raw: DCRCFAIL\r\n");
            return 0;
        }
        if (sta & SDMMC_STA_TXUNDERR) {
            SDMMC1->ICR = SDMMC_ICR_TXUNDERRC;
            UART_print("sdmmc_write_block_raw: TXUNDERR\r\n");
            return 0;
        }
        if (sta & SDMMC_STA_STBITERR) {
            SDMMC1->ICR = SDMMC_ICR_STBITERRC;
            UART_print("sdmmc_write_block_raw: STBITERR\r\n");
            return 0;
        }

        if (sta & SDMMC_STA_TXFIFOHE) {
            SDMMC1->FIFO = *p32++;
            words_remaining--;
        } else {
            timeout--;
        }
    }

    if (!timeout || words_remaining) {
        UART_print("sdmmc_write_block_raw: timeout while sending\r\n");
        return 0;
    }

    /* Wait for DATAEND to indicate controller finished sending data */
    timeout = SDMMC_DATA_TIMEOUT_LOOPS;
    while (!(SDMMC1->STA & SDMMC_STA_DATAEND) && --timeout) {
        /* wait */
    }
    if (!timeout) {
        UART_print("sdmmc_write_block_raw: DATAEND timeout\r\n");
        return 0;
    }
    SDMMC1->ICR = SDMMC_ICR_DATAENDC;

    /* Now wait until the card has finished internal programming */
    if (sdmmc_wait_card_ready() != 0) {
        UART_print("sdmmc_write_block_raw: card not ready after write\r\n");
        return 0;
    }

    return 1;
}

/* ===================================================================== */
/*              PUBLIC API – used by user_diskio.c                       */
/* ===================================================================== */

BYTE SD_InitializeLowLevel(void)
{
    UART_print("SD_InitializeLowLevel: start\r\n");

    if (s_initialized) {
        UART_print("SD_InitializeLowLevel: already initialized\r\n");
        return 0;
    }

    UART_print("SD_InitializeLowLevel: GPIO/clock\r\n");
    SDMMC1_GPIO_Init_1bit();
    SDMMC1_PeripheralClockEnable();

    UART_print("SD_InitializeLowLevel: calling sdmmc_card_init\r\n");
    int rc = sdmmc_card_init();
    UART_printf("SD_InitializeLowLevel: sdmmc_card_init rc=%d\r\n", rc);

    if (rc == 0) {
        s_initialized = 1;
        UART_print("SD_InitializeLowLevel: success\r\n");
        return 0;
    } else {
        UART_print("SD_InitializeLowLevel: FAIL\r\n");
        sdmmc_power_off();
        return 1;
    }
}

void SD_DeinitializeLowLevel(void)
{
    sdmmc_power_off();
    s_initialized = 0;
}

int SD_ReadBlock(BYTE *buff, DWORD sector)
{
    if (!s_initialized) {
        BYTE res = SD_InitializeLowLevel();
        if (res != 0) {
            return 0;
        }
    }
    return sdmmc_read_block_raw((uint8_t *)buff, (uint32_t)sector);
}

int SD_WriteBlock(const BYTE *buff, DWORD sector)
{
    if (!s_initialized) {
        BYTE res = SD_InitializeLowLevel();
        if (res != 0) {
            return 0;
        }
    }
    return sdmmc_write_block_raw((const uint8_t *)buff, (uint32_t)sector);
}
