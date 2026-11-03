#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"

/* LED integrado habitual de las placas ESP32 DevKit clasicas. */
#define SERIAL_TEST_GPIO GPIO_NUM_2
#define SERIAL_TEST_PULSE_MS 500
#define SERIAL_TEST_COMMAND_SIZE 128
#define SERIAL_TEST_QUEUE_LENGTH 32

typedef struct
{
    char text[SERIAL_TEST_COMMAND_SIZE];
} serial_command_t;

static const char *TAG = "[SERIAL]";
static const uart_port_t s_serial_uart = (uart_port_t)CONFIG_ESP_CONSOLE_UART_NUM;
static QueueHandle_t s_commands;

/* Prototipos de tareas. */
static void vTaskReceiveSerial(void *argument);
static void vTaskPulse(void *argument);

/* Prototipos de funciones auxiliares. */
static esp_err_t init_uart(void);
static esp_err_t init_led(void);

void app_main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);

    ESP_ERROR_CHECK(init_led());
    ESP_ERROR_CHECK(init_uart());

    s_commands = xQueueCreate(SERIAL_TEST_QUEUE_LENGTH, sizeof(serial_command_t));
    ESP_ERROR_CHECK(s_commands != NULL ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(vTaskPulse, "serial_pulse", 3072, NULL, 2, NULL) == pdPASS
                        ? ESP_OK : ESP_ERR_NO_MEM);

    ESP_LOGI(TAG, "READY,UART%d,%d baud,GPIO%d,pulso=%d ms,terminador=LF o CRLF",
             (int)s_serial_uart, CONFIG_ESP_CONSOLE_UART_BAUDRATE,
             (int)SERIAL_TEST_GPIO, SERIAL_TEST_PULSE_MS);
    ESP_ERROR_CHECK(xTaskCreate(vTaskReceiveSerial, "serial_receive", 4096, NULL, 2, NULL) == pdPASS
                        ? ESP_OK : ESP_ERR_NO_MEM);
}

/* Tareas. */
static void vTaskReceiveSerial(void *argument)
{
    (void)argument;

    serial_command_t command = {0};
    size_t length = 0;
    bool discard_line = false;
    uint8_t received[128];

    while (true)
    {
        const int count = uart_read_bytes(s_serial_uart, received, sizeof(received),
                                          pdMS_TO_TICKS(20));
        for (int i = 0; i < count; ++i)
        {
            const uint8_t byte = received[i];
            if (byte == '\n')
            {
                if (!discard_line && length > 0)
                {
                    command.text[length] = '\0';
                    ESP_LOGI(TAG, "Comando recibido: %s", command.text);
                    if (xQueueSend(s_commands, &command, 0) != pdTRUE)
                    {
                        printf("[SERIAL] ERROR,COLA_LLENA,%s\n", command.text);
                    }
                }
                length = 0;
                discard_line = false;
            }
            else if (byte != '\r' && !discard_line)
            {
                if (byte < 32 || byte > 126)
                {
                    printf("[SERIAL] ERROR,SOLO_ASCII_IMPRIMIBLE\n");
                    discard_line = true;
                }
                else if (length >= sizeof(command.text) - 1)
                {
                    printf("[SERIAL] ERROR,INSTRUCCION_DEMASIADO_LARGA\n");
                    discard_line = true;
                }
                else
                {
                    command.text[length++] = (char)byte;
                }
            }
        }
    }
}

static void vTaskPulse(void *argument)
{
    (void)argument;
    serial_command_t command;

    while (true)
    {
        if (xQueueReceive(s_commands, &command, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        ESP_ERROR_CHECK(gpio_set_level(SERIAL_TEST_GPIO, 1));
        vTaskDelay(pdMS_TO_TICKS(SERIAL_TEST_PULSE_MS));
        ESP_ERROR_CHECK(gpio_set_level(SERIAL_TEST_GPIO, 0));
        printf("[SERIAL] LED_OFF: %s\n", command.text);

        /* Separar los pulsos si llegaron varias instrucciones juntas. */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* Definiciones de funciones auxiliares. */
static esp_err_t init_uart(void)
{
    /* Conservar los pines del UART de consola configurado por ESP-IDF. */
    esp_err_t err = uart_driver_install(s_serial_uart, 2048, 0, 0, NULL, 0);
    if (err != ESP_OK)
    {
        return err;
    }

    err = uart_set_baudrate(s_serial_uart, CONFIG_ESP_CONSOLE_UART_BAUDRATE);
    if (err != ESP_OK)
    {
        uart_driver_delete(s_serial_uart);
    }
    return err;
}

static esp_err_t init_led(void)
{
    const gpio_config_t pin_config = {
        .pin_bit_mask = UINT64_C(1) << SERIAL_TEST_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_set_level(SERIAL_TEST_GPIO, 0);
    if (err != ESP_OK)
    {
        return err;
    }
    return gpio_config(&pin_config);
}
