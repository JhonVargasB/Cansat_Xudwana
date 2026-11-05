# Pruebas BMP180 EMA

El test compila el driver C de produccion, con solo I2C, reloj y esperas RTOS
simulados. No necesita ESP32 conectado. Comprueba primera muestra, entrada
constante, dt irregular, respuesta a escalon, atenuacion de fluctuaciones,
duplicados, entradas invalidas, fallos I2C, conservacion de la presion original,
altitud a partir de la presion EMA, bypass y reinicializacion. Tambien confirma
que el driver conserva las dos esperas de conversion por lectura correcta.

Desde la raiz del proyecto, en Developer Command Prompt de Visual Studio:

```bat
mkdir tmp\bmp180_ema
cl /nologo /std:c11 /O2 /W3 /I tests\bmp180_ema\stubs /I components\bmp180\include tests\bmp180_ema\test_bmp180_ema.c /Fe:tmp\bmp180_ema\test.exe /Fo:tmp\bmp180_ema\test.obj
tmp\bmp180_ema\test.exe
```

Con GCC/Clang y una shell POSIX:

```sh
mkdir -p tmp/bmp180_ema
cc -std=c11 -O2 -Itests/bmp180_ema/stubs -Icomponents/bmp180/include tests/bmp180_ema/test_bmp180_ema.c -lm -o tmp/bmp180_ema/test
./tmp/bmp180_ema/test
```

Los artefactos en `tmp` estan excluidos de Git. El test no mide tiempos del
ESP32 ni caracteriza ruido o respuesta en vuelo. En placa, `Test_bmp180.c`
permite observar P_raw, P_EMA y dt manteniendo el periodo de 100 ms. En main2,
`REC_ON` registra ambas presiones en `/sdcard/telem_ema.csv` para compararlas.
