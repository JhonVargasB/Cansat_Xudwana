#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "neo6m.h"
#include "esp_log.h"

static const char *TAG = "neo6m ";

esp_err_t neo6m_init(neo6m_t *neo6m)
{
    if (neo6m == NULL)
        return ESP_ERR_INVALID_ARG;

    uart_config_t uart_cfg =
        {
            .baud_rate = neo6m->config.baud_rate,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT};

    ESP_ERROR_CHECK(uart_driver_install(neo6m->config.uart_num, neo6m->config.rx_buffer_size, neo6m->config.tx_buffer_size, 0, NULL, 0));

    ESP_ERROR_CHECK(uart_param_config(neo6m->config.uart_num, &uart_cfg));

    ESP_ERROR_CHECK(uart_set_pin(neo6m->config.uart_num, neo6m->config.tx_pin, neo6m->config.rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART inicializado: UART%d, %lu bps", neo6m->config.uart_num, neo6m->config.baud_rate);

    return ESP_OK;
}

/*
Convierte la latitud/longitud que viene en formato NMEA a grados decimales
e.g. Formato NMEA:
$GPGGA,193709.00,2333.03120,S,04637.99848,W,1,07,0.99,760.0,M,0.0,M,,*XX

Latitud  2333.03120 con S -> -23.55052000
Longitud 04637.99848 con W -> -46.63330800

Formato decimal
*/
static double nmea_to_decimal(const char *value, const char *dir)
{
    if (value == NULL || dir == NULL || value[0] == '\0')
        return 0.0;

    // turn the text value into a float
    double raw = atof(value);

    int degrees = (int)(raw / 100);
    double minutes = raw - (degrees * 100);
    double decimal = degrees + (minutes / 60.0);

    if (dir[0] == 'S' || dir[0] == 'W')
        decimal = -decimal;

    return decimal;
}

/*
separa una sentencia NMEA en campos, usando la coma , como separador.
e.g.
$GPGGA,193709.00,2333.03120,S,04637.99848,W,1,07,0.99,760.0,M,0.0,M,,*XX
fields[0]  = "$GPGGA"
fields[1]  = "193709.00"
fields[2]  = "2333.03120"
fields[3]  = "S"
fields[4]  = "04637.99848"
fields[5]  = "W"
fields[6]  = "1"
fields[7]  = "07"
fields[8]  = "0.99"
fields[9]  = "3308.7"
fields[10] = "M"
fields[11] = "24.2"
fields[12] = "M"
fields[13] = ""
fields[14] = ""
*/

static int split_nmea(char *line, char *fields[], int max_fields)
{
    int count = 0;
    char *field = line; // puntero al inicio de la linea

    while (count < max_fields)
    {
        char *separator = strchr(field, ','); // busca el caracter , retorna el puntero del caracter
        char *checksum = strchr(field, '*');  // busca el caracter * retorna el puntero del caracter

        if (checksum != NULL && (separator == NULL || checksum < separator))
        {
            *checksum = '\0';
        }

        fields[count++] = field;

        if (separator == NULL)
        {
            break;
        }

        *separator = '\0';
        field = separator + 1;
    }

    return count;
}

static esp_err_t parse_gga(const char *line, neo6m_data_t *neo6m)
{
    char copy[128];
    char *fields[20];

    strncpy(copy, line, sizeof(copy));
    copy[sizeof(copy) - 1] = '\0';

    int n = split_nmea(copy, fields, 20);

    if (n < 10)
        return ESP_FAIL;

    strncpy(neo6m->utc_time, fields[1], sizeof(neo6m->utc_time));
    neo6m->utc_time[sizeof(neo6m->utc_time) - 1] = '\0';

    neo6m->latitude = nmea_to_decimal(fields[2], fields[3]);
    neo6m->longitude = nmea_to_decimal(fields[4], fields[5]);

    neo6m->fix_quality = atoi(fields[6]);
    neo6m->satellites_used = atoi(fields[7]);
    neo6m->hdop = atof(fields[8]);
    neo6m->altitude_m = atof(fields[9]);

    neo6m->valid = (neo6m->fix_quality > 0);

    return ESP_OK;
}

static esp_err_t parse_gsa(const char *line, neo6m_data_t *neo6m)
{
    char copy[128];
    char *fields[20];

    strncpy(copy, line, sizeof(copy));
    copy[sizeof(copy) - 1] = '\0';

    int n = split_nmea(copy, fields, 20);

    if (n < 18)
        return ESP_FAIL;

    neo6m->fix_type = atoi(fields[2]);

    neo6m->pdop = atof(fields[15]);
    neo6m->hdop = atof(fields[16]);

    neo6m->vdop = atof(fields[17]);

    return ESP_OK;
}

static esp_err_t parse_gsv(const char *line, neo6m_data_t *neo6m)
{
    char copy[128];
    char *fields[24];

    strncpy(copy, line, sizeof(copy));
    copy[sizeof(copy) - 1] = '\0';

    int n = split_nmea(copy, fields, 24);

    if (n < 4)
        return ESP_FAIL;

    neo6m->satellites_visible = atoi(fields[3]);

    return ESP_OK;
}

static esp_err_t parse_rmc(const char *line, neo6m_data_t *neo6m)
{
    char copy[128];
    char *fields[20];

    strncpy(copy, line, sizeof(copy));
    copy[sizeof(copy) - 1] = '\0';

    int n = split_nmea(copy, fields, 20);

    if (n < 10)
        return ESP_FAIL;

    strncpy(neo6m->utc_time, fields[1], sizeof(neo6m->utc_time));
    neo6m->utc_time[sizeof(neo6m->utc_time) - 1] = '\0';

    char status = fields[2][0];

    if (status == 'A')
    {
        neo6m->latitude = nmea_to_decimal(fields[3], fields[4]);
        neo6m->longitude = nmea_to_decimal(fields[5], fields[6]);

        float speed_knots = atof(fields[7]);
        neo6m->speed_kmh = speed_knots * 1.852f;

        strncpy(neo6m->date, fields[9], sizeof(neo6m->date));
        neo6m->date[sizeof(neo6m->date) - 1] = '\0';

        neo6m->valid = true;
    }
    else
    {
        neo6m->valid = false;
    }

    return ESP_OK;
}

static esp_err_t neo6m_parse_line(const char *line, neo6m_data_t *neo6m)
{
    if (line == NULL || neo6m == NULL)
        return ESP_ERR_INVALID_ARG;

    if (strncmp(line, "$GPGGA", 6) == 0)
        return parse_gga(line, neo6m);

    if (strncmp(line, "$GPGSA", 6) == 0)
        return parse_gsa(line, neo6m);

    if (strncmp(line, "$GPGSV", 6) == 0)
        return parse_gsv(line, neo6m);

    if (strncmp(line, "$GPRMC", 6) == 0)
        return parse_rmc(line, neo6m);

    return ESP_ERR_NOT_SUPPORTED;
}

bool neo6m_has_fix(const neo6m_data_t *neo6m)
{
    if (neo6m == NULL)
        return false;

    return neo6m->valid && neo6m->fix_quality > 0 && neo6m->fix_type >= 2;
}

void neo6m_print_data(neo6m_t *neo6m)
{
    if (neo6m == NULL)
        return;

    ESP_LOGI(TAG, "Latitud: %.8f", neo6m->data.latitude);
    ESP_LOGI(TAG, "Longitud: %.8f", neo6m->data.longitude);
    ESP_LOGI(TAG, "Altitud: %.2f m", neo6m->data.altitude_m);
    ESP_LOGI(TAG, "Velocidad: %.2f km/h", neo6m->data.speed_kmh);
    ESP_LOGI(TAG, "Fix quality: %u", neo6m->data.fix_quality);
    ESP_LOGI(TAG, "Fix type: %u", neo6m->data.fix_type);
    ESP_LOGI(TAG, "Sat usados: %u", neo6m->data.satellites_used);
    ESP_LOGI(TAG, "Sat visibles: %u", neo6m->data.satellites_visible);
    ESP_LOGI(TAG, "PDOP: %.2f | HDOP: %.2f | VDOP: %.2f",
             neo6m->data.pdop, neo6m->data.hdop, neo6m->data.vdop);
    ESP_LOGI(TAG, "UTC: %s | Fecha: %s", neo6m->data.utc_time, neo6m->data.date);
}
esp_err_t neo6m_update(neo6m_t *neo6m, uint32_t timeout_ms)
{
    if (neo6m == NULL)
        return ESP_ERR_INVALID_ARG;

    uint8_t byte;
    char line[128];
    int index = 0;

    while (1)
    {
        int len = uart_read_bytes(
            neo6m->config.uart_num,
            &byte,
            1,
            pdMS_TO_TICKS(timeout_ms));

        if (len <= 0)
            return ESP_ERR_TIMEOUT;

        if (byte == '\n')
        {
            line[index] = '\0';

            if (index > 0)
            {
                // Descomentar para ver todas las sentencias NMEA recibidas
                //ESP_LOGI(TAG, "RX: %s", line);

                esp_err_t err = neo6m_parse_line(line, &neo6m->data);

                /*
                 * Cuando llega una trama RMC ya tenemos
                 * una actualización completa de los datos.
                 */
                if (strncmp(line, "$GPRMC", 6) == 0)
                {
                    return err;
                }
            }

            index = 0;
        }
        else if (byte != '\r')
        {
            if (index < sizeof(line) - 1)
            {
                line[index++] = (char)byte;
            }
            else
            {
                /* Overflow del buffer */
                index = 0;
            }
        }
    }
}