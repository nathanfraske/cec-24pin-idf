/*
 * Line-based command interface over USB CDC.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"   /* IDF 6.0: was esp_vfs_usb_serial_jtag.h */
#include "cec_cli.h"

static const char *TAG = "cec_cli";

#define CLI_LINE_BUF_SIZE  192
#define CLI_MAX_TOKENS     16
#define CLI_TASK_STACK     6144
#define CLI_TASK_PRIORITY  3

static const cec_cli_command_t *s_commands = NULL;
static size_t                   s_command_count = 0;
static bool                     s_inited = false;

static const cec_cli_command_t *find_command(const char *name)
{
    for (size_t i = 0; i < s_command_count; i++) {
        if (strcmp(s_commands[i].name, name) == 0) {
            return &s_commands[i];
        }
    }
    return NULL;
}

static int handle_help(int argc, char **argv)
{
    if (argc >= 2) {
        const cec_cli_command_t *c = find_command(argv[1]);
        if (c == NULL) {
            printf("error: unknown command '%s'\n", argv[1]);
            return 1;
        }
        printf("%s  %s\n", c->name, c->help);
        return 0;
    }
    printf("Available commands:\n");
    /* Compute max name width so the help column lines up. */
    size_t name_w = 4;  /* "help" */
    for (size_t i = 0; i < s_command_count; i++) {
        size_t n = strlen(s_commands[i].name);
        if (n > name_w) name_w = n;
    }
    printf("  %-*s  %s\n", (int)name_w, "help", "show this list, or 'help <command>' for one-line detail");
    for (size_t i = 0; i < s_command_count; i++) {
        printf("  %-*s  %s\n", (int)name_w, s_commands[i].name, s_commands[i].help);
    }
    return 0;
}

/* Tokenize in-place: split on whitespace, NUL-terminate each token,
 * store pointers in argv. Returns argc. */
static int tokenize(char *line, char **argv, int max_argv)
{
    int argc = 0;
    char *p = line;
    while (*p && argc < max_argv) {
        while (*p == ' ' || *p == '\t') p++;
        if (!*p) break;
        argv[argc++] = p;
        while (*p && *p != ' ' && *p != '\t' && *p != '\n' && *p != '\r') p++;
        if (*p) *p++ = '\0';
    }
    return argc;
}

static void dispatch(char *line)
{
    char *argv[CLI_MAX_TOKENS];
    int argc = tokenize(line, argv, CLI_MAX_TOKENS);
    if (argc == 0) return;

    if (strcmp(argv[0], "help") == 0 || strcmp(argv[0], "?") == 0) {
        handle_help(argc, argv);
        return;
    }

    const cec_cli_command_t *c = find_command(argv[0]);
    if (c == NULL) {
        printf("error: unknown command '%s' (try 'help')\n", argv[0]);
        return;
    }
    int rc = c->handler(argc, argv);
    if (rc != 0) {
        printf("error: %d\n", rc);
    }
}

static void cli_task(void *arg)
{
    (void)arg;
    char line[CLI_LINE_BUF_SIZE];
    printf("\n[cli] ready — type 'help' for commands\n");
    while (1) {
        if (fgets(line, sizeof(line), stdin) == NULL) {
            /* fgets returns NULL on EOF or read error. With the USB CDC
             * driver installed this only happens transiently; back off
             * briefly so we don't hot-spin. */
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        dispatch(line);
    }
}

esp_err_t cec_cli_init(const cec_cli_command_t *commands, size_t count)
{
    if (s_inited) return ESP_OK;
    if (commands == NULL && count > 0) return ESP_ERR_INVALID_ARG;

    /* Install the USB Serial-JTAG driver and rebind stdio to it so
     * fgets() on stdin blocks until a newline arrives. Without this
     * stdin reads come back immediately (non-blocking default) and
     * the reader task would burn CPU. */
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(usb_serial_jtag_driver_install(&cfg),
                        TAG, "usb_serial_jtag_driver_install");
    usb_serial_jtag_vfs_use_driver();   /* IDF 6.0 rename of esp_vfs_usb_serial_jtag_use_driver */
    /* Line-buffer stdin so reads complete on each '\n'. */
    setvbuf(stdin, NULL, _IOLBF, CLI_LINE_BUF_SIZE);

    s_commands = commands;
    s_command_count = count;

    if (xTaskCreate(cli_task, "cec_cli", CLI_TASK_STACK, NULL,
                    CLI_TASK_PRIORITY, NULL) != pdPASS) {
        return ESP_FAIL;
    }

    s_inited = true;
    ESP_LOGI(TAG, "registered %u commands", (unsigned)count);
    return ESP_OK;
}
