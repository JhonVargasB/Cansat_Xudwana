# BMP180: presion original y EMA

`bmp180_read_data()` hace las conversiones existentes de temperatura y presion,
aplica la compensacion de Bosch y actualiza un EMA causal de primer orden:

```text
dt = (timestamp_us - ultimo_timestamp_valido_us) * 1e-6
alpha = 1 - expf(-2*pi*fc*dt)
presion_filtrada += alpha * (presion_original - presion_filtrada)
```

La configuracion `pressure_filter_cutoff_hz` vale `3.0f` por defecto; `0.0f`
permite comparar sin suavizado. No se cambia el OSS predeterminado (1), la
frecuencia de lectura ni las esperas necesarias para convertir el sensor.
El timestamp se toma con `esp_timer_get_time()` justo despues de obtener una
presion nueva del BMP180; no es un timestamp interno del ADC.

Los campos de `device.data` son:

| Campo | Significado |
| --- | --- |
| `pressure` | Presion original compensada, en Pa; no se modifica por el EMA |
| `pressure_filtered_pa` | Presion EMA, en Pa (`float`) |
| `altitude` | Altitud calculada con la presion EMA y `sea_level_pressure_pa` |
| `timestamp_us` | Instante de lectura de la presion original |
| `pressure_filter_dt_s` | Tiempo entre las dos ultimas lecturas validas |
| `pressure_filter_alpha` | Coeficiente usado en esta actualizacion |
| `valid` | Solo `true` tras una lectura completa correcta |

La primera muestra se copia directamente: P_f=P, dt=0 y alpha=1. El estado
es propio de cada instancia y se limpia al inicializar/desinicializar. Un
fallo I2C, una compensacion invalida o un timestamp no creciente no confirma
un estado nuevo; los valores previos se conservan con `valid=false`. Tras un
hueco, el dt incluye todo el tiempo desde la ultima muestra valida. No se
procesan otra vez datos anteriores. El EMA usa float/expf, memoria fija O(1)
y ninguna espera adicional. Mantener un solo propietario o serializar accesos.

Con 10 Hz nominales, alpha es aproximadamente 0.848164: este ajuste suaviza
moderadamente, priorizando la respuesta. Las pruebas a 10 Hz dan 84.82% de
respuesta a un escalon en 100 ms y 97.70% en 200 ms. Para una fluctuacion
alternante de 5 Hz, la relacion RMS salida/entrada es 0.7363. Aumentar la
frecuencia de adquisicion cambia alpha automaticamente. `fc` parametriza la
constante de tiempo mediante la ecuacion solicitada; cerca de Nyquist no
equivale exactamente al punto -3 dB de la respuesta digital discreta.

No elimina deriva lenta ni offset: la ganancia en continua es 1. Tampoco
corrige problemas de actitud, compuerta NIS o recuperacion del Kalman. La
presion suavizada introduce retardo y correlacion entre muestras; en esta
modificacion se conservan los parametros y la logica existentes del Kalman.

En `main2.c`, la altura relativa, la entrada barometrica del Kalman y la altura
de respaldo usan esta altitud EMA. La presion del payload de radio sigue
siendo la original y la trama sigue ocupando 32 bytes.

Cuando se recibe `REC_ON`, la tarea SD existente registra `telem_ema.csv` y
`ekf.csv`. La nueva ruta evita mezclar el esquema ampliado con un `telem.csv`
antiguo. Las columnas originales de telemetria se conservan y se agregan:

```text
baro_timestamp_us,pressure_raw_pa,pressure_filtered_pa,pressure_dt_s,pressure_alpha,baro_valid
```

`pressure_raw_pa` significa presion compensada sin EMA, no cuentas del ADC.
Si falla una lectura, esas columnas numericas contienen `nan`, el timestamp
barometrico es 0 y `baro_valid=0`; no se registra una muestra antigua como
nueva. En `Test_bmp180.c` tambien se imprimen timestamp, dt, alpha y ambas
presiones. El test conserva el periodo nominal de 100 ms.

Pruebas de software y comandos: `tests/bmp180_ema/README.md`. La cadencia real
y la respuesta del montaje fisico se verifican en placa con los timestamps
registrados; las pruebas de escritorio usan sensor y reloj simulados.
