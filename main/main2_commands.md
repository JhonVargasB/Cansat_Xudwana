# Comandos de main2.c

Seleccionar solo `main2.c` en `main/CMakeLists.txt`. Mantiene las tareas del
main integrado y la telemetria de 32 bytes. Usa los pines actuales de `shared.h`.

| Comando | Accion |
| --- | --- |
| REC_ON | Abre `telem.csv` y `ekf.csv` en append y comienza a guardar. No borra los registros anteriores. |
| REC_OFF | Impide nuevas muestras SD, guarda las pendientes, sincroniza y cierra ambos CSV. Luego emite 1000 Hz durante 500 ms con el buzzer de `shared.h`. |
| CALIBRAR | Requiere reposo y nivel, con Z del MPU hacia arriba. Calibra offsets de accel y gyro, reinicializa actitud respecto a gravedad, bias/covarianza del EKF y filtro vertical. Yaw vuelve a cero relativo. |

La tarjeta se monta al iniciar, pero no se graba hasta REC_ON. REC_OFF cierra
los archivos y deja la tarjeta montada para otro REC_ON. La calibracion inicial
del main original se conserva. El comando CALIBRAR permite repetirla despues.
El cero de altura relativa del arranque se conserva tambien al recalibrar.

La tarea de radio es la unica que accede al nRF24. Despacha operaciones por
colas a las tareas de SD y actitud, que devuelven el resultado al terminar.
No hay lecturas concurrentes del MPU durante la calibracion. GPS, barometro y
radio continuan; la actitud se marca invalida mientras se calibra. El archivo
EKF presenta un intervalo sin muestras durante ese procedimiento.

## Protocolo de radio

Se usa `nrf24_command_protocol.h`: comando packed de 7 bytes y respuesta PTX
de 9 bytes, con los magic y CRC-16/CCITT-FALSE del documento de la estacion.
La respuesta ACCEPTED se emite despues de finalizar la operacion real.

Se admite una operacion pendiente. Los reintentos con la misma secuencia y
comando no repiten la accion. Mientras termina no se envia DUPLICATE ni una
confirmacion prematura. Al terminar se conserva el resultado para responder a
reintentos; una operacion exitosa repetida responde DUPLICATE y un fallo
repetido conserva el codigo de error. Otro comando mientras esta ocupado
responde REJECTED_STATE; la misma secuencia con otro codigo, INVALID_COMMAND.

`state` en las respuestas usa bit0=registro activo y bit1=calibrando:
0=sin registro, 1=grabando, 2=calibrando sin registro, 3=ambos.
Un fallo del buzzer produce ACTUATOR_ERROR aunque los CSV ya se hayan cerrado;
el bit0 permite distinguir que el registro esta detenido.

## Tiempo de calibracion y prueba

Las rutinas actuales del MPU toman 1000 muestras cada una, mas dos esperas
de 2 segundos y la inicializacion de actitud. No caben en el timeout de 5 s
de la estacion descrita en el documento. Para CALIBRAR, configurar alli una
espera final de 20 s y un timeout absoluto de 25 s. Ampliar solo el timeout
absoluto no basta si la espera final sigue siendo 1500 ms.

Prueba en hardware: esperar a que termine la inicializacion, enviar REC_ON,
verificar respuesta ACCEPTED con state=1, enviar REC_OFF, verificar tono y
respuesta con state=0, y comprobar ambos CSV. Repetir REC_ON debe continuar
los mismos archivos sin otra cabecera. Probar CALIBRAR quieto y nivelado,
verificando que la telemetria sigue llegando y no se recalibra por reintentos.
Las calibraciones son en RAM, no se persisten en NVS.
