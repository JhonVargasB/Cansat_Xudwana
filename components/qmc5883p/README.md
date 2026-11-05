# QMC5883P

Driver ESP-IDF 5.4 basado en [QST, QMC5883P Datasheet Rev. E](https://www.qstcorp.com/upload/pdf/202512/13-52-19%20QMC5883P%20Datasheet%20Rev.E.pdf).

Mantiene el objeto, configuracion por defecto y bus externo de los demas
componentes. El bus y el bypass del MPU6050 pertenecen a la aplicacion.

```c
qmc5883p_t magnetometer = {
    .bus_handle = bus_handle,
    .config = QMC5883P_DEFAULT_CONFIG(),
};
ESP_ERROR_CHECK(qmc5883p_init(&magnetometer));
if (qmc5883p_read_data(&magnetometer, 100) == ESP_OK) {
    // magnetometer.data.raw_x/raw_y/raw_z: cuentas con signo.
    // magnetometer.data.x_ut/y_ut/z_ut/norm_ut: microteslas.
}
ESP_ERROR_CHECK(qmc5883p_deinit(&magnetometer));
```

Por defecto: direccion 0x2C, I2C a 100 kHz, modo normal a 100 Hz, rango
nominal +/-8 G, OSR1=8 y OSR2=8. Se verifica CHIP_ID=0x80 antes y despues
del reinicio, y los controles se leen despues de configurarlos.

El reset se activa escribiendo 0x80 en CTRL2, se espera 10 ms y se desactiva
escribiendo 0x00; se esperan otros 10 ms antes de verificar el estado. No se
espera un auto-clear de SOFT_RST: las revisiones A/C/E solo lo documentan
explicitamente para SELF_TEST. La liberacion explicita tambien se utiliza
en la [propuesta de driver Linux IIO probada en hardware](https://www.spinics.net/lists/kernel/msg6160746.html).
Se comprueban ID, modo suspend y CTRL2 antes de configurar el rango y ODR.
Si el rango leido no coincide, init falla; no se supone una sensibilidad
distinta ni se publican microteslas con una escala sin confirmar.

Los ejemplos de QST requieren escribir 0x06 en el registro 0x29 para los
signos de los ejes. Esta configuracion se aplica durante init. No representa
la rotacion de montaje hacia el marco body del CanSat.

La lectura consulta STATUS y, si DRDY esta activo, obtiene los seis bytes
X/Y/Z en una sola transferencia. El estado se lee una sola vez por sondeo:
DRDY y OVFL se borran al leer STATUS. Las muestras con OVFL se descartan.
La conversion usa la sensibilidad nominal correspondiente al rango seleccionado.

`read_data(..., 0)` hace un sondeo y devuelve ESP_ERR_NOT_FINISHED si no hay
dato nuevo. Con un timeout positivo espera cediendo CPU y devuelve
ESP_ERR_TIMEOUT si no aparece un dato. Las transacciones I2C tienen ademas
el limite independiente io_timeout_ms. Solo ESP_OK publica una muestra nueva
con valid=true; ante un fallo, los valores previos permanecen con valid=false.

Para probar la GY-87, seleccionar `test_qmc.c` en `main/CMakeLists.txt`.
El test activa el bypass, lee a 10 Hz e imprime cuentas y microteslas.
Si la configuracion falla, compara las lecturas con repeated START y con
STOP intermedio, y prueba los cuatro valores del campo RNG; luego restaura
los controles y desactiva el bypass. Son diagnosticos, no deteccion de revision:
la direccion 0x2C y el ID 0x80 no distinguen la revision de silicio.
No calibra hard/soft iron, no calcula rumbo y no modifica el EKF ni la
telemetria. Una sola tarea debe controlar cada objeto, o usar exclusion
mutua externa. `deinit` suspende el sensor y libera su handle, sin cerrar
el bus compartido.
