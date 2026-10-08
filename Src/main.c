#include "main.h"
#include "stm32l476xx.h"
#include "fatfs.h"
#include "diskio.h"
#include "sd_sdio.h"

#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <ctype.h>

/* ------------------- UART ------------------- */

#define BAUD 35          // ~115200 at default MSI (~4 MHz)
#define CMD  0x1B

void SystemClock_Config(void);

void UART_print_char(int c) {
    while ((USART2->ISR & USART_ISR_TXE) == 0) { }
    USART2->TDR = c;
    while ((USART2->ISR & USART_ISR_TC) == 0) { }
}

void UART_print(char *str) {
    while (*str != '\0') {
        if (*str == '\n') {
            UART_print_char('\r');   // CR for terminals
        }
        UART_print_char((int)*str);
        str++;
    }
}

void UART_ESC_Code(char *code) {
    UART_print_char(CMD);
    UART_print("[");
    UART_print(code);
}

void UART_printf(const char *fmt, ...) {
    char buffer[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    UART_print(buffer);
}

void UART_init(void) {
    // Clocks for USART2 and GPIOA
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN;

    // PA2 and PA3 as UART TX and RX (AF7)
    GPIOA->MODER &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3);
    GPIOA->MODER |=  (GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1);   // AF
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT2 | GPIO_OTYPER_OT3);
    GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED2 | GPIO_OSPEEDR_OSPEED3);
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD2 | GPIO_PUPDR_PUPD3);
    GPIOA->PUPDR |=  (GPIO_PUPDR_PUPD3_0); // RX pull-up for safety
    GPIOA->AFR[0] &= ~((0xF << GPIO_AFRL_AFSEL2_Pos) | (0xF << GPIO_AFRL_AFSEL3_Pos));
    GPIOA->AFR[0] |=  ((7U << GPIO_AFRL_AFSEL2_Pos) | (7U << GPIO_AFRL_AFSEL3_Pos));

    // Disable USART before config
    USART2->CR1 &= ~USART_CR1_UE;

    // 8N1, oversampling by 16, no parity
    USART2->CR1 &= ~(USART_CR1_M0 | USART_CR1_M1 | USART_CR1_PCE | USART_CR1_OVER8);
    USART2->CR2 &= ~USART_CR2_STOP;
    USART2->CR3 = 0;

    // Baud rate
    USART2->BRR = BAUD;

    // Enable TX and RX
    USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE);

    // Enable USART
    USART2->CR1 |= USART_CR1_UE;

    while ((USART2->ISR & USART_ISR_TEACK) == 0) { }
    while ((USART2->ISR & USART_ISR_REACK) == 0) { }

    // Clear screen & home cursor once
    UART_ESC_Code("2J");
    UART_ESC_Code("H");
}

/* Blocking character receive (polling) */
int UART_get_char(void) {
    while ((USART2->ISR & USART_ISR_RXNE) == 0) { }
    return (int)(USART2->RDR & 0xFF);
}

/* Line input with echo, backspace, CR/LF end. Returns length (no terminator). */
int UART_read_line(char *buf, int maxLen) {
    int idx = 0;
    int ch;

    if (maxLen <= 0) return 0;
    maxLen--; // reserve for '\0'

    while (1) {
        ch = UART_get_char();

        if (ch == '\r' || ch == '\n') {
            UART_print("\r\n");
            buf[idx] = '\0';
            return idx;
        } else if ((ch == 0x08 || ch == 0x7F) && idx > 0) {
            // backspace
            idx--;
            UART_print("\b \b");
        } else if (ch >= 0x20 && ch <= 0x7E) {
            // printable
            if (idx < maxLen) {
                buf[idx++] = (char)ch;
                UART_print_char(ch);
            } else {
                // buffer full -> bell
                UART_print_char(0x07);
            }
        }
    }
}

/* ------------------- FATFS + Shell Globals ------------------- */

static FATFS SDFatFs;
static char  cmdLine[128];
static char  textLine[256];
static char  cwd[128];      // "" = root, otherwise e.g. "dir", "dir/sub"

/* ------------------- Helpers ------------------- */

/* Build a path given current cwd and a name.
 * Root is represented as cwd="" (empty string).
 * Result is always relative to drive 0: (no "0:" prefix), e.g. "dir/file".
 */
static void build_path(const char *base, const char *name, char *out, size_t outSize) {
    if (!out || outSize == 0) return;

    if (!name || name[0] == '\0') {
        // Just base
        if (base && base[0] != '\0') {
            strncpy(out, base, outSize - 1);
            out[outSize - 1] = '\0';
        } else {
            out[0] = '\0';
        }
        return;
    }

    // If name starts with '/', treat as absolute from root (strip leading '/')
    if (name[0] == '/') {
        name++;
        strncpy(out, name, outSize - 1);
        out[outSize - 1] = '\0';
        return;
    }

    // base is root
    if (!base || base[0] == '\0') {
        strncpy(out, name, outSize - 1);
        out[outSize - 1] = '\0';
        return;
    }

    // base + "/" + name
    snprintf(out, outSize, "%s/%s", base, name);
}

/* Case-insensitive token comparison */
static int is_token_equal(const char *tok, const char *word) {
    if (!tok || !word) return 0;
    while (*tok && *word) {
        if (tolower((unsigned char)*tok) != tolower((unsigned char)*word)) {
            return 0;
        }
        tok++;
        word++;
    }
    return (*tok == '\0' && *word == '\0');
}

/* Safely append a single path component to tmp */
static void append_component(char *tmp, size_t tmpSize,
                             const char *comp, size_t compLen)
{
    if (!tmp || tmpSize == 0 || !comp || compLen == 0) return;

    size_t curLen = strlen(tmp);
    size_t extra = compLen + (curLen > 0 ? 1 : 0); // '/' + comp

    if (curLen + extra >= tmpSize) {
        // Truncate component if needed so we always keep NUL
        if (curLen >= tmpSize - 1) {
            // No room to append anything
            return;
        }
        // Adjust compLen so that curLen + (maybe '/') + compLen == tmpSize-1
        size_t avail = (tmpSize - 1) - curLen;
        if (curLen > 0) {
            if (avail == 0) return;
            avail--;  // for '/'
        }
        compLen = avail;
        if (compLen == 0) return;
    }

    if (curLen > 0) {
        tmp[curLen++] = '/';
    }

    memcpy(&tmp[curLen], comp, compLen);
    tmp[curLen + compLen] = '\0';
}

/* Normalize a path: base is current cwd ("" = root), rel is argument to cd.
 * Handles '.', '..', '../..', '/absolute', etc., without recursion.
 */
static void normalize_path(const char *base,
                           const char *rel,
                           char *out,
                           size_t outSize)
{
    char tmp[256];

    if (!out || outSize == 0) return;
    tmp[0] = '\0';

    if (!rel || rel[0] == '\0') {
        // No relative part: just use base
        if (base && base[0] != '\0') {
            strncpy(tmp, base, sizeof(tmp) - 1);
            tmp[sizeof(tmp) - 1] = '\0';
        } else {
            tmp[0] = '\0';
        }
    } else {
        const char *p = rel;

        // If rel starts with '/', start from root
        if (*p == '/') {
            tmp[0] = '\0';
            // skip leading '/'
            while (*p == '/') p++;
        } else {
            // start from base (current cwd)
            if (base && base[0] != '\0') {
                strncpy(tmp, base, sizeof(tmp) - 1);
                tmp[sizeof(tmp) - 1] = '\0';
            } else {
                tmp[0] = '\0';
            }
        }

        // Process each component
        while (*p) {
            // Skip extra '/'
            while (*p == '/') p++;
            if (!*p) break;

            const char *start = p;
            while (*p && *p != '/') p++;
            size_t compLen = (size_t)(p - start);
            if (compLen == 0) continue;

            if (compLen == 1 && start[0] == '.') {
                // '.' -> no-op
                continue;
            } else if (compLen == 2 && start[0] == '.' && start[1] == '.') {
                // '..' -> pop one component
                if (tmp[0] != '\0') {
                    char *slash = strrchr(tmp, '/');
                    if (slash) {
                        *slash = '\0';
                    } else {
                        tmp[0] = '\0';
                    }
                }
            } else {
                // normal component: append
                append_component(tmp, sizeof(tmp), start, compLen);
            }
        }
    }

    // Copy to output
    strncpy(out, tmp, outSize - 1);
    out[outSize - 1] = '\0';
}

/* ------------------- Command Implementations ------------------- */

static void cmd_help(void) {
    UART_print("Commands:\r\n");
    UART_print("  ls [path]      - list directory contents\r\n");
    UART_print("  cd [dir]       - change directory (cd .., cd ., cd /, cd ../.. supported)\r\n");
    UART_print("  mkdir <name>   - create directory\r\n");
    UART_print("  rm <name>      - remove file or empty directory\r\n");
    UART_print("  cat <file>     - display file contents\r\n");
    UART_print("  vim <file>     - simple line editor ('.' on its own line to finish)\r\n");
    UART_print("  help           - List of Commands\r\n");
}

/* ls [path] */
static void cmd_ls(const char *arg) {
    char path[128];
    DIR dir;
    FILINFO fno;
    FRESULT res;

    if (arg && arg[0] != '\0') {
        build_path(cwd, arg, path, sizeof(path));
    } else {
        // current directory
        if (cwd[0] == '\0') {
            path[0] = '\0';  // root
        } else {
            strncpy(path, cwd, sizeof(path) - 1);
            path[sizeof(path) - 1] = '\0';
        }
    }

    res = f_opendir(&dir, path[0] ? path : "");
    if (res != FR_OK) {
        UART_printf("ls: opendir failed (%d)\r\n", (int)res);
        return;
    }

    UART_print("Name              Size      Attr\r\n");
    UART_print("================================\r\n");

    for (;;) {
        res = f_readdir(&dir, &fno);
        if (res != FR_OK || fno.fname[0] == 0) break;

        char attr = (fno.fattrib & AM_DIR) ? 'D' : 'F';
        UART_printf("%-16s %8lu  %c\r\n",
                    fno.fname, (unsigned long)fno.fsize, attr);
    }

    f_closedir(&dir);
}

/* mkdir <name> */
static void cmd_mkdir(const char *arg) {
    if (!arg || arg[0] == '\0') {
        UART_print("mkdir: missing name\r\n");
        return;
    }

    char path[128];
    build_path(cwd, arg, path, sizeof(path));

    FRESULT res = f_mkdir(path);
    if (res != FR_OK) {
        UART_printf("mkdir: failed (%d)\r\n", (int)res);
    }
}

/* cd [dir] with multi-level ../ support */
static void cmd_cd(const char *arg) {
    if (!arg || arg[0] == '\0') {
        // cd with no args -> root
        cwd[0] = '\0';
        return;
    }

    char newCwd[128];
    normalize_path(cwd, arg, newCwd, sizeof(newCwd));

    // Verify directory exists
    DIR dir;
    FRESULT res = f_opendir(&dir, newCwd[0] ? newCwd : "");
    if (res != FR_OK) {
        UART_printf("cd: cannot access '%s' (%d)\r\n", arg, (int)res);
        return;
    }

    f_closedir(&dir);

    // Accept new cwd
    strncpy(cwd, newCwd, sizeof(cwd) - 1);
    cwd[sizeof(cwd) - 1] = '\0';
}

/* rm <name> */
static void cmd_rm(const char *arg) {
    if (!arg || arg[0] == '\0') {
        UART_print("rm: missing operand\r\n");
        return;
    }

    char path[128];
    build_path(cwd, arg, path, sizeof(path));

    FRESULT res = f_unlink(path);
    if (res != FR_OK) {
        UART_printf("rm: failed (%d)\r\n", (int)res);
    }
}

/* View file contents (text) */
static void view_file(const char *path) {
    FIL file;
    FRESULT res;
    UINT br;

    res = f_open(&file, path, FA_READ);
    if (res != FR_OK) {
        UART_printf("view: f_open failed (%d)\r\n", (int)res);
        return;
    }

    for (;;) {
        res = f_read(&file, textLine, sizeof(textLine) - 1, &br);
        if (res != FR_OK) {
            UART_printf("view: f_read error (%d)\r\n", (int)res);
            break;
        }
        if (br == 0) {
            break; // EOF
        }
        textLine[br] = '\0';
        UART_print(textLine);
    }

    f_close(&file);
}

/* cat <file> */
static void cmd_cat(const char *arg) {
    if (!arg || arg[0] == '\0') {
        UART_print("cat: missing filename\r\n");
        return;
    }

    char path[128];
    build_path(cwd, arg, path, sizeof(path));

    view_file(path);
    UART_print("\r\n");
}

/* Simple line editor: write lines to an already-open file (append or overwrite)
 * '.' on its own line finishes.
 */
static int edit_lines(FIL *file) {
    FRESULT res;
    UINT written;
    int len;

    UART_print("Enter text; each Enter becomes a new line in the file.\r\n");
    UART_print("Enter a single '.' on its own line to finish.\r\n\r\n");

    while (1) {
        UART_print("> ");
        len = UART_read_line(textLine, sizeof(textLine));

        if (len == 1 && textLine[0] == '.') {
            UART_print("End-of-file marker received.\r\n");
            break;
        }

        if (len > 0) {
            res = f_write(file, textLine, (UINT)len, &written);
            if (res != FR_OK || written != (UINT)len) {
                UART_printf("write error: %d (wrote %u / %u)\r\n",
                            (int)res, (unsigned)written, (unsigned)len);
                return -1;
            }
        }

        const char crlf[2] = {'\r', '\n'};
        res = f_write(file, crlf, 2, &written);
        if (res != FR_OK || written != 2) {
            UART_printf("write error (CRLF): %d\r\n", (int)res);
            return -1;
        }
    }

    return 0;
}

/* vim <file> */
static void cmd_vim(const char *arg) {
    if (!arg || arg[0] == '\0') {
        UART_print("vim: missing filename\r\n");
        return;
    }

    char path[128];
    build_path(cwd, arg, path, sizeof(path));

    FILINFO fno;
    FRESULT res = f_stat(path, &fno);

    if (res != FR_OK && res != FR_NO_FILE) {
        UART_printf("vim: f_stat failed (%d)\r\n", (int)res);
        return;
    }

    /* If file does not exist, create it and immediately enter editor.
       Afterwards, we drop into the submenu as well. */
    if (res == FR_NO_FILE) {
        FIL file;
        UART_printf("vim: '%s' does not exist, creating new.\r\n", arg);
        res = f_open(&file, path, FA_CREATE_ALWAYS | FA_WRITE);
        if (res != FR_OK) {
            UART_printf("vim: f_open failed (%d)\r\n", (int)res);
            return;
        }

        if (edit_lines(&file) == 0) {
            res = f_sync(&file);
            if (res != FR_OK) {
                UART_printf("vim: f_sync failed (%d)\r\n", (int)res);
            }
        }
        f_close(&file);
        /* Now treat it as an existing file and show the menu below. */
    }

    /* Sub-menu loop for existing file (including newly created one).
       After editing (end-of-file marker), we return to this menu instead
       of exiting the vim command. */
    while (1) {
        UART_printf("vim: file '%s' menu.\r\n", arg);
        UART_print("  [E]dit (overwrite), [A]ppend, [V]iew, [C]ancel: ");
        int len = UART_read_line(cmdLine, sizeof(cmdLine));
        char choice = (len > 0) ? (char)toupper((unsigned char)cmdLine[0]) : 'C';

        if (choice == 'C') {
            UART_print("vim: cancelled.\r\n");
            return;
        } else if (choice == 'V') {
            view_file(path);
            UART_print("\r\n");
            // show menu again
        } else if (choice == 'E') {
            FIL file;
            res = f_open(&file, path, FA_CREATE_ALWAYS | FA_WRITE);
            if (res != FR_OK) {
                UART_printf("vim: f_open (overwrite) failed (%d)\r\n", (int)res);
                return;
            }
            if (edit_lines(&file) == 0) {
                res = f_sync(&file);
                if (res != FR_OK) {
                    UART_printf("vim: f_sync failed (%d)\r\n", (int)res);
                }
            }
            f_close(&file);
            // After editing, go back to menu instead of exiting
        } else if (choice == 'A') {
            FIL file;
            res = f_open(&file, path, FA_OPEN_APPEND | FA_WRITE);
            if (res != FR_OK) {
                UART_printf("vim: f_open (append) failed (%d)\r\n", (int)res);
                return;
            }
            if (edit_lines(&file) == 0) {
                res = f_sync(&file);
                if (res != FR_OK) {
                    UART_printf("vim: f_sync failed (%d)\r\n", (int)res);
                }
            }
            f_close(&file);
            // After editing, go back to menu instead of exiting
        } else {
            UART_print("vim: unknown choice.\r\n");
        }
    }
}

/* ------------------- Command Parsing ------------------- */

static void process_command(char *line) {
    // Trim leading spaces
    char *p = line;
    while (*p == ' ' || *p == '\t') p++;

    if (*p == '\0') return; // empty line

    // Extract command token
    char cmd[16];
    int ci = 0;
    while (*p && !isspace((unsigned char)*p) && ci < (int)(sizeof(cmd) - 1)) {
        cmd[ci++] = *p++;
    }
    cmd[ci] = '\0';

    // Skip spaces before arg
    while (*p == ' ' || *p == '\t') p++;
    const char *arg = (*p) ? p : NULL;

    if (is_token_equal(cmd, "ls")) {
        cmd_ls(arg);
    } else if (is_token_equal(cmd, "mkdir")) {
        cmd_mkdir(arg);
    } else if (is_token_equal(cmd, "cd")) {
        cmd_cd(arg);
    } else if (is_token_equal(cmd, "rm")) {
        cmd_rm(arg);
    } else if (is_token_equal(cmd, "cat")) {
        cmd_cat(arg);
    } else if (is_token_equal(cmd, "vim")) {
        cmd_vim(arg);
    } else if (is_token_equal(cmd, "help")) {
        cmd_help();
    } else {
        UART_printf("Unknown command: %s\r\n", cmd);
    }
}

/* ------------------- Main ------------------- */

int main(void) {
    HAL_Init();
    SystemClock_Config();
    UART_init();

    MX_FATFS_Init();

    // Minimal startup messages; no low-level SD debug spam
    UART_print("=== UART SDMMC Line Shell ===\r\n");
    UART_print("Shell is case-insensitive. Type 'help' for a List of Commands.\r\n\r\n");

    // Initialize disk (FatFs calls into our SD driver via user_diskio.c)
    if (disk_initialize(0) & STA_NOINIT) {
        UART_print("Warning: disk initialize failed; commands may not work.\r\n");
    }

    // Mount filesystem
    FRESULT res = f_mount(&SDFatFs, "", 1);
    if (res != FR_OK) {
        UART_printf("Warning: f_mount failed (%d); commands may not work.\r\n", (int)res);
    }

    // Start at root
    cwd[0] = '\0';

    // Shell loop
    while (1) {
        if (cwd[0] == '\0') {
            UART_print("sd:/ $ ");
        } else {
            UART_printf("sd:/%s$ ", cwd);
        }

        int len = UART_read_line(cmdLine, sizeof(cmdLine));
        if (len <= 0) {
            continue;
        }

        process_command(cmdLine);
    }
}

/* ------------------- HAL Clock / Error ------------------- */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
