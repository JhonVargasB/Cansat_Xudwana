# Cansat_Xudwana

Firmware del CanSat Xudwana para **ESP32-S3** con **ESP-IDF 5.4**.
Incluye sensores, EKF de actitud, Kalman de altura y velocidad vertical,
telemetria nRF24L01+, almacenamiento microSD y visualizacion en MATLAB R2021.

## Estructura

- `main/main.c`: firmware del CanSat transmisor.
- `main/main_Rx.c`: receptor nRF24 y salida serial para la estacion.
- `main/main_Tx.c`: ejemplo TX con datos simulados y comandos por ACK.
- `main/main_Rx2.c`: ejemplo alternativo de receptor.
- `main/Test_*.c` y `main/main_mpu.c`: pruebas individuales.
- `components/shared/include/shared.h`: pines, parametros, datos y formatos CSV.
- `components/`: BMP180, MPU6050, NEO-6M, nRF24L01+, LoRa E220, SD, buzzer,
  EKF de cuaterniones (`ekq_q`) y Kalman de altura.
- `matlab/`: captura y graficas en tiempo real del resumen EKF.

## Compilar y cargar

Instala ESP-IDF 5.4 y abre su terminal con el entorno activado:

```sh
git clone https://github.com/JhonVargasB/Cansat_Xudwana.git
cd Cansat_Xudwana
idf.py set-target esp32s3
idf.py build
idf.py -p COM9 flash monitor
```

Cambia `COM9` por el puerto de tu placa. En Linux puede ser
`/dev/ttyUSB0` o `/dev/ttyACM0`.
`sdkconfig.defaults` conserva la configuracion base; `sdkconfig` y `build/`
se generan en cada equipo. La configuracion base usa flash de 2 MB:
ajustala si tu placa y las particiones requieren otra capacidad.

### Seleccionar TX, RX o una prueba

Edita manualmente `main/CMakeLists.txt` y selecciona un solo archivo:

```cmake
idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS ".")
```

Para el receptor usa `"main_Rx.c"`. Cada ejemplo define su propio
`app_main()`; no deben compilarse todos juntos.

## Pines actuales

La fuente de verdad es `shared.h`.

| Periferico | Senales y GPIO |
| --- | --- |
| I2C: BMP180 y MPU6050 | SDA=1, SCL=2 |
| microSD SPI2 | MOSI=12, MISO=10, SCLK=11, CS=13 |
| nRF24 TX SPI3 | MOSI=4, MISO=5, SCLK=6, CSN=15, CE=7, IRQ=NC |
| nRF24 RX SPI3, segunda placa | MOSI=9, MISO=43, SCLK=5, CSN=7, CE=4, IRQ=NC |
| NEO-6M UART1 | TX ESP32=17, RX ESP32=18 |
| Buzzer pasivo | GPIO8 |

El bus I2C usa pull-ups externas a 3.3 V. TX y RX son placas distintas.
Los ejemplos LoRa tienen su propia configuracion de pines.

## Funcionamiento

Mantener el CanSat inmovil y nivelado durante el arranque: se calibra el
MPU6050 y se promedia la referencia barometrica.

- Actitud y filtro vertical: 100 Hz.
- Telemetria y lectura del BMP180: cada 100 ms por defecto.
- Altura relativa al arranque, positiva hacia arriba.
- Velocidad vertical positiva al subir y negativa al bajar.
- Roll y pitch estimados con acelerometro y giroscopio.
- Yaw relativo, sin magnetometro; puede acumular deriva.
- Montaje del MPU configurable mediante `MPU_TO_BODY_ROTATION`.
- Telemetria de 32 bytes por nRF24, con ACK y comandos en ACK payload.
- Radio predeterminada: canal 76, 250 kbps, CRC de dos bytes,
  direccion `01:C2:C2:C2:C2` (LSByte primero).
- Diagnostico al arrancar con pines y registros leidos de la radio.
- SD: `/sdcard/telem.csv` y `/sdcard/ekf.csv`, agregando registros sin borrar
  el contenido anterior.

## Serial y MATLAB

La consola trabaja a **115200 baudios**. El resumen del filtro usa:

```text
EKF,timestamp_us,sequence,roll_deg,pitch_deg,yaw_deg,linear_ad_m_s2,valid,height_m,vertical_velocity_m_s,vertical_valid
```

Los angulos estan en grados. La aceleracion D es positiva hacia abajo;
altura relativa y velocidad vertical son positivas hacia arriba.
Las banderas indican la validez de cada estimacion.

1. Cierra `idf.py monitor` y cualquier programa que ocupe el puerto.
2. Abre `matlab/visualizar_ekf.m` en MATLAB R2021a/R2021b.
3. Ajusta `puerto` y ejecuta el script; conserva `parsear_fila_ekf.m`
   en la misma carpeta.
4. Pulsa **Detener y guardar** para finalizar.

Las capturas CSV/MAT se guardan en `matlab/capturas/`, excluida de Git.
Se muestran actitud, aceleracion vertical, altura y velocidad.

El receptor imprime, despues del diagnostico inicial:

```text
pitch,roll,yaw,alt_rel_m,temp_c,pres_pa,ax,ay,az,lat,lon,sats
```

`ax`, `ay` y `az` son ceros de compatibilidad; la trama de radio actual
no contiene esas aceleraciones.

## Buzzer

`BUZZER_DEFAULT_CONFIG()`, `buzzer_init()` y `buzzer_set_frequency()`
permiten generar una onda cuadrada al 50 % entre 1 y 1024 Hz; 0 apaga.
Utiliza GPTimer y todavia no se incorpora a los mains.
Ejemplo: `components/buzzer/README.md`.

## Archivos locales

`build/`, `sdkconfig`, `sdkconfig.old`, `.vscode/` y las capturas se
conservan localmente, pero no se suben al repositorio.
