# Cansat_Xudwana

Firmware del CanSat Xudwana para **ESP32-S3** con **ESP-IDF 5.4**.
Incluye sensores, EKF de actitud, Kalman de altura y velocidad vertical,
telemetria nRF24L01+, almacenamiento microSD y visualizacion en MATLAB R2021.

## Estructura

- `main/main2.c`: transmisor con comandos SD, EMA y radio de tasa variable.
- `main/main.c`: version anterior del transmisor.
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

Para el transmisor actual usa `"main2.c"`; para el receptor, `"main_Rx.c"`. Cada ejemplo define su propio
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
- Adquisicion de telemetria y BMP180: cada 100 ms por defecto.
- En main2, radio a 1 Hz durante arranque/espera y a 10 Hz durante grabacion SD.
- Altura relativa al arranque, positiva hacia arriba.
- Velocidad vertical positiva al subir y negativa al bajar.
- Roll y pitch estimados con acelerometro y giroscopio.
- En `main2.c`, yaw corregido con el QMC5883P en el mismo EKF, a 20 Hz.
- Si el magnetometro falla o una muestra se rechaza, continua el yaw inercial
  (puede derivar); el log `MAG` indica la edad de la ultima correccion.
- Montaje del MPU configurable mediante `MPU_TO_BODY_ROTATION`.
- Telemetria de 32 bytes por nRF24, con ACK y comandos en ACK payload.
- Radio predeterminada: canal 30 (2430 MHz), 250 kbps, CRC de dos bytes,
  direccion `01:C2:C2:C2:C2` (LSByte primero).
- Diagnostico al arrancar con pines y registros leidos de la radio.
- SD en main2: `/sdcard/telem_ema.csv` y `/sdcard/ekf.csv`, agregando registros sin borrar
  el contenido anterior.

El BMP180 aplica un EMA de presion con corte inicial 3 Hz y dt real entre
lecturas validas. Conserva la presion original y calcula la altura con la
filtrada. Ambas presiones y los parametros de cada actualizacion se guardan
en `telem_ema.csv` al activar `REC_ON`; los `telem.csv` anteriores se conservan.
Configuracion y validacion: [BMP180 EMA](components/bmp180/README.md).

### Radio durante arranque, espera y grabacion en main2.c

La aplicacion crea las colas y los eventos antes de inicializar el nRF24.
Inmediatamente despues de configurar la radio crea su tarea, antes del I2C,
SD, GPS y calibracion del MPU. El primer envio no espera a que esten listos
los sensores: una trama de 32 bytes con secuencia y datos invalidos permite
comprobar el enlace mediante el ACK. No se transmite basura sin inicializar.
Los paquetes continuan durante la calibracion. Los comandos por ACK que
lleguen durante el arranque quedan en cola; la operacion se ejecuta al crear
las tareas correspondientes, y solo entonces se confirma su resultado.

| Estado | Intervalo RF nominal |
| --- | --- |
| Arranque, calibracion inicial y espera | 1000 ms |
| REC_ON completado y archivos SD preparados | 100 ms |
| REC_OFF, o fallo de escritura que detiene el registro | 1000 ms |

REC_ON fallido conserva la tasa lenta. REC_OFF conserva el cierre/sincronizado
y el buzzer existentes. Una recalibracion posterior no detiene la radio;
se invalidan los angulos/velocidad mientras se calibra. El estado de respuesta
mantiene sus bits: bit0=grabando, bit1=calibrando/preparando arranque.

Los periodos se configuran en shared.h con `NRF24_IDLE_TX_PERIOD_MS` y
`NRF24_RECORDING_TX_PERIOD_MS`. Son independientes del periodo de adquisicion
de 100 ms y del EKF de 10 ms. La radio toma la muestra mas reciente de una
cola de longitud 1; no acumula muestras para mandarlas despues en rafaga.
Muestras de mas de `NRF24_RADIO_SAMPLE_MAX_AGE_US` (500 ms) se sustituyen por
una trama sin datos validos. El bit SD sigue reflejando si se esta grabando.

La secuencia RF cuenta los intentos de envio, incluyendo los paquetes de
arranque; la secuencia de los registros SD sigue contando muestras adquiridas.
Al bajar a 1 Hz no se generan falsos saltos RF por las muestras no transmitidas.
Las respuestas a comandos son paquetes adicionales, fuera del periodo normal.

Tras leer el ACK payload, la tarea pone la radio en power_down. El driver
la despierta automaticamente en el proximo transmit(). La tarea espera con
notificaciones hasta el siguiente plazo: cambios de grabacion y resultados
de comandos la despiertan sin esperar el segundo completo. Los comandos
desde tierra siguen llegando con el ACK de un envio: en espera su latencia
puede incluir hasta aproximadamente un segundo, ademas del procesamiento.

La reduccion de consumo debe medirse en el equipo completo; este cambio no
garantiza por si solo pasar de tres a cuatro horas de bateria. El algoritmo
del Kalman se mantiene. Pruebas con reloj/RTOS/radio simulados:
[ciclo de radio](tests/radio_lifecycle/README.md).

### Yaw magnetico y montaje en main2.c

Seleccionar `main2.c` en `main/CMakeLists.txt`. El QMC5883P comparte I2C mediante
el bypass del MPU6050. La tarea de actitud actualiza el EKF con accel/gyro a
100 Hz y, cada 50 ms, intenta leer una muestra magnetica nueva sin esperar
DRDY. Calibra el vector con los coeficientes configurados, lo lleva a ejes
body y compensa la inclinacion con la rotacion del EKF antes de corregir el
rumbo con `ekf_q_correct_yaw()`. Los paquetes nRF24 mantienen sus 32 bytes;
el campo yaw publicado corresponde al cuaternion corregido.

En `components/shared/include/shared.h`:

- `MPU_TO_BODY_ROTATION`: montaje de toda la placa respecto al CanSat;
  se aplica por software a acelerometro, giroscopo y campo magnetico.
- `QMC_TO_MPU_ROTATION`: alineacion relativa del QMC respecto al MPU,
  despues de los signos configurados por el driver (`0x29=0x06`). Se deja
  identidad como suposicion para las pruebas: verificarla en esta GY-87.
- `QMC_HARD_IRON_OFFSET_UT` y `QMC_SOFT_IRON_MATRIX`: calibracion magnetica
  en ejes QMC, antes de rotarlos. Por defecto son cero e identidad; aun no
  representan una calibracion medida del montaje final.
- `YAW_MAGNETIC_DECLINATION_RAD`: por defecto cero, referencia al norte
  magnetico. Configurar la declinacion local para obtener norte geografico.
- `YAW_MEASUREMENT_NOISE_STD_RAD`: incertidumbre efectiva de la observacion,
  inicialmente 5 grados; no es una garantia de precision.

Las matrices de orientacion deben ser rotaciones propias (ortogonales y
determinante +1). No se cambian registros del MPU para redefinir los ejes.
Los registros de bypass solo permiten acceder al QMC en el bus auxiliar.
Si el montaje cambia, editar `MPU_TO_BODY_ROTATION`; la alineacion fija
entre los dos sensores permanece en `QMC_TO_MPU_ROTATION`.

Se rechazan saturacion, campo fuera de 10..100 uT, componente horizontal
menor de 5 uT y pitch mayor de 85 grados en valor absoluto (rumbo Euler casi
singular). Estas comprobaciones no detectan todas las perturbaciones
magneticas. No se repiten muestras sin DRDY ni se reinicia el EKF por un
rechazo magnetico. El diagnostico `MAG` sale cada 5 s.

`CALIBRAR` conserva su funcion: accel/gyro, gravedad y reinicio del EKF;
el QMC vuelve a corregir yaw despues. No calcula hard/soft iron automaticamente.
La calibracion del acelerometro en main2 deriva su referencia en los ejes del
sensor de `MPU_TO_BODY_ROTATION`: mantener el CanSat quieto y nivelado en su
montaje configurado. La API antigua del driver mantiene su referencia +Z
hacia arriba para los otros ejemplos. La compensacion
magnetica sigue el mismo enfoque de `cubesat-flight-software`, adaptado al QMC.

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
