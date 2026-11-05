# Prueba de navegacion de main3

`main/main3.c` conserva las tareas, el protocolo NRF24 de 32 bytes, el arranque
temprano de radio y los comandos de main2. Los ajustes experimentales estan en
`s_test_config`, dentro de main3; no cambian los otros mains ni las librerias.

Para probar en la placa, seleccionar manualmente `main3.c` en `main/CMakeLists.txt`:

```cmake
idf_component_register(SRCS "main3.c"
                       INCLUDE_DIRS ".")
```

Esta revision deja el CMake existente sin modificar. La prueba no cambia los
pines de shared.h ni la estructura que espera el receptor.

## Cambios de la prueba

- MPU a 100 Hz y DLPF de 44/42 Hz. La lectura y conversion de unidades conserva
  el comportamiento de main2, sin descartar muestras al acercarse al 98 % del rango.
- Compuerta NIS siempre activa: 9 en reposo, 25 en movimiento.
- Ruido inicial de altura de 1 m; aceleracion de 0.25/3 m/s2 en reposo/movimiento.
  Son valores de partida para caracterizar el BMP180; no son ruidos ya medidos.
- Aceleracion completa, sin multiplicarla por 0.2 ni limitarla a 5 m/s2.
- Reposo requiere IMU y evolucion barometrica compatibles durante un segundo.
  Al entrar, se restablece velocidad cero; el sesgo vertical se aprende en reposo.
- Tras diez rechazos y una serie de al menos cinco lecturas barometricas coherentes,
  se recupera el filtro manteniendo el origen relativo. La velocidad inicial de
  recuperacion proviene del barometro si hay movimiento.
- Tras un segundo sin correccion aceptada deja de publicar una prediccion valida.
  Se conserva el respaldo de altura barometrica y velocidad cero en la telemetria.
- La altura puede seguir estimandose con barometro y modelo de velocidad constante
  si la IMU no es valida; se eleva la incertidumbre de aceleracion y se registra
  la validez de actitud por separado.
- La supervision de gravedad es diagnostica: no caduca la actitud a los dos
  segundos ni la reinicializa por una correccion de acelerometro rechazada.
  Los pasos IMU y las correcciones magneticas usan la misma matematica de main2.
- La actualizacion IMU se confirma sobre una copia del EKF. Un error aislado no
  corrompe x/P ni desinicializa un filtro sano. Un hueco de tiempo se descarta
  y se resincroniza el timestamp, manteniendo la orientacion anterior.
- La telemetria mantiene los ultimos angulos cuando una muestra es invalida o
  antigua, con MPU_OK apagado. Con la siguiente muestra valida actualiza los
  valores. No representa una perdida puntual de IMU con un salto artificial a cero.
- La inicializacion y CALIBRAR siguen pudiendo establecer una nueva actitud.
  La recuperacion en reposo se reserva para una inicializacion/calibracion
  incompleta; no se ejecuta por movimiento ni por falta de barometro.

## Consola y SD

`NAV_HEADER` identifica los campos y unidades de las lineas `NAV`, a 10 Hz desde
data_task; el filtro no hace el printf en su ciclo de 100 Hz:

```
NAV,timestamp_us,sequence,baro_relative_m,height_m,vertical_velocity_m_s,a_up_m_s2,a_used_m_s2,vertical_bias_m_s2,nis,baro_accepted,rejections,recoveries,stationary,attitude_valid,vertical_valid,baro_age_ms,gravity_error_m_s2,attitude_resets
```

- `height_m` y velocidad son el estado del filtro; comprobar `vertical_valid`.
  Mientras no existe anclaje se imprimen `nan`. NRF conserva el respaldo.
- `a_up` es aceleracion vertical sin gravedad, positiva al subir; no es el
  modulo de la aceleracion neta. `a_used` es la entrada efectiva al Kalman.
- `baro_accepted` y NIS describen la ultima lectura procesada, no un pulso que
  pueda perderse al observar a 10 Hz. `rejections` es la racha consecutiva.
- `recoveries` cuenta reanclajes verticales despues del inicial; `stationary`
  confirma reposo; `gravity_error` mide el residuo sin ocultarlo con el bias del
  EKF. `attitude_resets` cuenta arranques recuperados de una inicializacion
  incompleta; un rechazo de gravedad no incrementa este contador.
- El paquete NRF se mantiene intacto; la interfaz Python conserva su calculo
  actual de velocidad. Comparar NAV y NRF_TX directamente para esta prueba.

REC_ON abre `TELEM3.CSV` (19 columnas) y **`EKF3.CSV`**
(25 columnas), ambos en append. No se mezclan esquemas con los archivos EKF
anteriores. REC_OFF sincroniza y cierra los archivos. El cero relativo no cambia
al recuperar actitud, perder un sensor ni ejecutar CALIBRAR.
Los dos nombres cumplen FAT 8.3: esta compilacion tiene LFN deshabilitado.
Si no se puede abrir/sincronizar la telemetria, REC_ON falla y la radio conserva
1 Hz. Con telemetria iniciada correctamente pasa a 10 Hz, incluso si el archivo
opcional del filtro no puede abrirse. El log esperado es `Radio: periodo=100 ms,
estado=GRABANDO`. Los archivos previos no se renombran ni se eliminan.

El filtro sigue adquiriendo a 100 Hz. El CSV compacto conserva las entradas
IMU y elimina Euler, normas, matrices e innovaciones XYZ del registro anterior:

```
timestamp_us,sequence,dt_s,ax_body_m_s2,ay_body_m_s2,az_body_m_s2,gx_body_rad_s,gy_body_rad_s,gz_body_rad_s,qw,qx,qy,qz,baro_timestamp_us,baro_relative_m,height_m,vertical_velocity_m_s,a_up_m_s2,a_used_m_s2,vertical_bias_m_s2,nis,accepted_axes,flags,dropped_records,recoveries
```

`flags`: bit0 actitud valida, bit1 vertical valida, bit2 reposo, bit3 ultima
lectura barometrica aceptada, bit4 MPU leido, bit5 inicializando, bit6 error.
`baro_timestamp_us` permite fusionar cada muestra del barometro una sola vez.

## Prioridad de telemetria en SD

- Cola de telemetria independiente de 32 registros; el filtro utiliza registros
  compactos de 104 bytes en lugar de los 224 bytes del registro completo.
- El escritor atiende primero la telemetria y vuelve a comprobar su cola despues
  de cada escritura del filtro. El filtro puede escribir como maximo dos registros
  por pasada y no inicia otra escritura si ya transcurrieron 2 ms. Ese presupuesto
  no interrumpe una operacion FAT/SD que ya este bloqueada dentro del driver.
- Si hay cuatro telemetrias pendientes, se descartan nuevas muestras del filtro
  antes de permitir que aumente su carga. Ambos productores usan envios sin espera.
- Si falla abrir/escribir/sincronizar el CSV del filtro, se cierra solo ese archivo.
  La telemetria continua siempre que su archivo y la tarjeta sigan funcionando.
- Al comenzar y cada cinco segundos, si no hay telemetria pendiente, se comprueba
  el espacio libre. Con 64 MiB o menos se desactiva el filtro y se deja ese margen
  para telemetria. Si no se puede consultar la capacidad, tambien se desactiva
  el registro opcional. No se borra ningun archivo existente.
- Los logs `SD` y `REC_OFF` muestran registros y descartes de ambos flujos. Los
  contadores son acumulativos desde el arranque. `dropped_records` del filtro
  incluye descarte por prioridad, cola llena y registros descartados al cerrar
  su cola por un fallo; no cuantifica bytes ya almacenados que pierda una SD averiada.

## Secuencia sugerida en banco

1. Arrancar quieto y nivelado segun el montaje configurado; esperar calibracion.
2. REC_ON, dejar quieto 20 s: observar altura/velocidad y confirmacion de reposo.
3. Inclinar, girar y desplazar en vertical; despues dejar inmovil 20 s. Comparar
   altura barometrica y filtrada, aceptacion barometrica, NIS y recuperaciones.
4. Repetir a una altura fisica distinta: recuperar no debe cambiar el cero original.
5. REC_OFF; revisar los CSV y las muestras descartadas. Confirmar tambien el
   comportamiento si QMC rechaza correcciones antes de evaluar yaw absoluto.

## Pruebas reproducibles sin hardware

El generador extrae las funciones reales de main3 y las compila con los nucleos
reales `altitude_kalman.c` y `ekq_q.c`. Solo simula reloj y cola barometrica.
En una terminal de desarrollador de Visual Studio, desde la raiz del proyecto:

```powershell
python tests/main3_navigation/generate_host_test.py
cl /nologo /std:c11 /O2 /W3 /Itests\bmp180_ema\stubs /Icomponents\altitude_kalman\include /Icomponents\ekq_q\include tmp\main3_navigation\test.c components\altitude_kalman\altitude_kalman.c components\ekq_q\ekq_q.c /Fe:tmp\main3_navigation\test.exe /Fo:tmp\main3_navigation\
.\tmp\main3_navigation\test.exe
```

Casos: reposo, entrada completa de 12 m/s2, movimiento seguido de reposo,
divergencia forzada, outlier aislado de 100 m, timestamps repetidos/atrasados/
futuros, perdida de barometro/IMU, recuperacion a 500 m, ascenso de 10 m/s,
aprendizaje de bias, giro de 4 s seguido de reposo de 5 s sin confirmacion nueva
de gravedad (yaw conserva 100 grados), equivalencia x/P con 1000 actualizaciones
IMU y 200 correcciones magneticas de main2, errores/dt invalidos sin corromper
estado y continuidad de los angulos de telemetria ante muestras perdidas/vencidas.

Con un `build/compile_commands.json` ESP32-S3 ya configurado:

```powershell
python tests/main3_navigation/check_firmware.py
```

Compila y enlaza main3 por separado usando las librerias del build, dejando
CMake intacto. Los artefactos quedan en `tmp/main3_navigation`. No graba la placa.

Para los ensayos especificos del escritor SD, desde una terminal MSVC:

```powershell
python tests/main3_navigation/generate_storage_test.py
cl /nologo /std:c11 /O2 /W3 /Itests\bmp180_ema\stubs tmp\main3_storage\test.c /Fe:tmp\main3_storage\test.exe /Fo:tmp\main3_storage\test.obj
.\tmp\main3_storage\test.exe
```

Simula colas, reloj y fallos de archivo; comprueba las funciones reales de main3
para prioridad, llegadas durante una escritura, presupuesto, presion de colas,
contadores, fallos opcionales y reserva de espacio. No caracteriza la latencia
ni la fiabilidad de una tarjeta fisica.
Tambien ejecuta el flujo real de REC_ON con una apertura que simula la restriccion
FAT 8.3: los nombres nuevos permiten activar 10 Hz; un fallo del archivo de
telemetria conserva 1 Hz y un fallo del archivo opcional no impide los 10 Hz.
