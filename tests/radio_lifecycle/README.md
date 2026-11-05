# Prueba del ciclo de radio

El generador extrae del main2 actual la tarea `vTaskTransmitData` y los helpers
de periodo, preparacion de trama, estado y cambio de grabacion. Los compila
con reloj, colas, eventos, notificaciones y radio simulados. El algoritmo de
planificacion probado es el codigo de produccion, no una copia.

Escenarios: trama al iniciar, calibracion inicial, 1 Hz en espera, REC_ON a
mitad del periodo lento, REC_OFF, REC_ON fallido, recalibracion, muestras
caducadas, recuperacion de adquisicion y fallo SD. Se comprueban secuencias RF,
plazos, invalidacion de datos y lectura de ACK antes de power_down. Tambien se
verifica el orden de inicializacion y que adquisicion sigue usando su periodo
propio. No simula fisicamente el nRF24 ni acredita autonomia de bateria.

Desde la raiz del proyecto, generar el test:

```sh
python tests/radio_lifecycle/generate_host_test.py
```

En Developer Command Prompt de Visual Studio:

```bat
cl /nologo /std:c11 /O2 /W3 tmp\radio_lifecycle\test.c /Fe:tmp\radio_lifecycle\test.exe /Fo:tmp\radio_lifecycle\test.obj
tmp\radio_lifecycle\test.exe
```

Con GCC/Clang:

```sh
cc -std=c11 -O2 tmp/radio_lifecycle/test.c -o tmp/radio_lifecycle/test
./tmp/radio_lifecycle/test
```

Los artefactos permanecen en tmp, excluido de Git. La validacion en placa debe
comprobar que aparecen NRF_TX antes del mensaje de calibracion terminada,
periodo nominal de 1 s en espera, 100 ms tras REC_ON y 1 s tras REC_OFF. El
buzzer y el estado SD se verifican con los mismos comandos de la estacion.
