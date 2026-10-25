# Buzzer PWM

Buzzer pasivo para ESP32-S3 / ESP-IDF 5.4. Pin por defecto: `PIN_BUZZER`
en `shared.h` (GPIO8).

La frecuencia se expresa en **Hz**, no como duty: `0` apaga y `1..1024`
enciende una onda cuadrada al 50 %. Valores superiores devuelven
`ESP_ERR_INVALID_ARG` sin cambiar la salida actual.

```c
#include "buzzer.h"

static buzzer_t buzzer = {0};
buzzer_config_t config = BUZZER_DEFAULT_CONFIG();

ESP_ERROR_CHECK(buzzer_init(&buzzer, &config)); // Inicialmente apagado.
ESP_ERROR_CHECK(buzzer_set_frequency(&buzzer, 500));
ESP_ERROR_CHECK(buzzer_set_frequency(&buzzer, 1024));
ESP_ERROR_CHECK(buzzer_set_frequency(&buzzer, 0)); // Igual que buzzer_off().
ESP_ERROR_CHECK(buzzer_deinit(&buzzer));
```

La instancia debe permanecer viva hasta `buzzer_deinit()`. Realizar el control
desde una sola tarea, o proteger las llamadas externamente.

Se utiliza un GPTimer a 1 MHz con dos interrupciones por periodo para cubrir
tambien 1 Hz en el ESP32-S3. No crea tareas ni realiza esperas activas. La
frecuencia tiene redondeo a microsegundos y jitter de interrupcion; las
frecuencias muy bajas producen pulsaciones, no un tono continuo audible.
Con la configuracion actual las interrupciones pueden aplazarse durante
operaciones que deshabilitan la cache de flash.

Un buzzer activo genera su propio tono y no permite elegirlo mediante este PWM.
