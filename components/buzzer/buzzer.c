#include "buzzer.h"

#define BUZZER_TIMER_RESOLUTION_HZ UINT32_C(1000000)

/* Se usa GPTimer para cubrir tambien 1 Hz en ESP32-S3. El callback solo cambia
 * el GPIO; no hay esperas, tareas adicionales ni logs dentro de la interrupcion.
 */
static bool buzzer_on_alarm(gptimer_handle_t timer,
                            const gptimer_alarm_event_data_t *event,
                            void *user_data)
{
    (void)timer;
    (void)event;
    buzzer_t *device = user_data;
    portENTER_CRITICAL_ISR(&device->lock);
    if (device->active)
    {
        device->level ^= 1U;
        gpio_set_level(device->config.pin, device->level);
    }
    portEXIT_CRITICAL_ISR(&device->lock);
    return false;
}

esp_err_t buzzer_init(buzzer_t *device, const buzzer_config_t *config)
{
    if (device == NULL || config == NULL || !GPIO_IS_VALID_OUTPUT_GPIO(config->pin))
        return ESP_ERR_INVALID_ARG;
    if (device->initialized)
        return ESP_ERR_INVALID_STATE;

    device->config = *config;
    device->lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
    device->timer = NULL;
    device->frequency_hz = 0U;
    device->level = 0U;
    device->active = false;
    device->running = false;
    device->enabled = false;

    const gpio_config_t pin_config = {
        .pin_bit_mask = UINT64_C(1) << device->config.pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t error = gpio_set_level(device->config.pin, 0);
    if (error == ESP_OK)
        error = gpio_config(&pin_config);
    if (error != ESP_OK)
        return error;

    const gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = BUZZER_TIMER_RESOLUTION_HZ,
    };
    error = gptimer_new_timer(&timer_config, &device->timer);
    if (error != ESP_OK)
        goto init_failed;

    const gptimer_event_callbacks_t callbacks = { .on_alarm = buzzer_on_alarm };
    error = gptimer_register_event_callbacks(device->timer, &callbacks, device);
    if (error == ESP_OK)
        error = gptimer_enable(device->timer);
    if (error != ESP_OK)
        goto init_failed;

    device->enabled = true;
    device->initialized = true;
    return ESP_OK;

init_failed:
    if (device->timer != NULL)
    {
        gptimer_del_timer(device->timer);
        device->timer = NULL;
    }
    gpio_reset_pin(device->config.pin);
    return error;
}

esp_err_t buzzer_off(buzzer_t *device)
{
    if (device == NULL)
        return ESP_ERR_INVALID_ARG;
    if (!device->initialized)
        return ESP_ERR_INVALID_STATE;

    /* La ISR no puede volver a subir la salida despues de apagar. */
    portENTER_CRITICAL(&device->lock);
    device->active = false;
    device->level = 0U;
    esp_err_t error = gpio_set_level(device->config.pin, 0);
    portEXIT_CRITICAL(&device->lock);
    device->frequency_hz = 0U;

    if (device->running)
    {
        const esp_err_t stop_error = gptimer_stop(device->timer);
        if (stop_error != ESP_OK)
            return stop_error;
        device->running = false;
    }
    return error;
}

esp_err_t buzzer_set_frequency(buzzer_t *device, uint32_t frequency_hz)
{
    if (device == NULL || frequency_hz > BUZZER_MAX_FREQUENCY_HZ)
        return ESP_ERR_INVALID_ARG;
    if (!device->initialized || !device->enabled)
        return ESP_ERR_INVALID_STATE;

    esp_err_t error = buzzer_off(device);
    if (error != ESP_OK || frequency_hz == 0U)
        return error;

    const uint32_t divisor = 2U * frequency_hz;
    const gptimer_alarm_config_t alarm = {
        .alarm_count = (BUZZER_TIMER_RESOLUTION_HZ + divisor / 2U) / divisor,
        .reload_count = 0U,
        .flags.auto_reload_on_alarm = true,
    };
    error = gptimer_set_raw_count(device->timer, 0U);
    if (error == ESP_OK)
        error = gptimer_set_alarm_action(device->timer, &alarm);
    if (error != ESP_OK)
        return error;

    portENTER_CRITICAL(&device->lock);
    device->active = true;
    portEXIT_CRITICAL(&device->lock);
    error = gptimer_start(device->timer);
    if (error != ESP_OK)
    {
        buzzer_off(device);
        return error;
    }
    device->running = true;
    device->frequency_hz = frequency_hz;
    return ESP_OK;
}

esp_err_t buzzer_deinit(buzzer_t *device)
{
    esp_err_t error = buzzer_off(device);
    if (error != ESP_OK)
        return error;
    if (device->enabled)
    {
        error = gptimer_disable(device->timer);
        if (error != ESP_OK)
            return error;
        device->enabled = false;
    }
    error = gptimer_del_timer(device->timer);
    if (error != ESP_OK)
        return error;
    device->timer = NULL;
    device->initialized = false;
    return gpio_reset_pin(device->config.pin);
}
