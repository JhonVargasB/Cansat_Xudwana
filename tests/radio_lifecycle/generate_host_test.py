"""Generate a host test from the actual main2 radio functions (no firmware edits)."""
from pathlib import Path
import re

root = Path(__file__).resolve().parents[2]
source = (root / 'main/main2.c').read_text(encoding='utf-8')
shared = (root / 'components/shared/include/shared.h').read_text(encoding='utf-8')
out = root / 'tmp/radio_lifecycle'
out.mkdir(parents=True, exist_ok=True)

def function(name):
    m = re.search(r'(?:static )?(?:void|uint32_t|uint8_t|telemetry_data_t) '
                  + name + r'\([^;]*?\)\s*\{', source)
    pos, depth = m.end(), 1
    while depth:
        depth += (source[pos] == '{') - (source[pos] == '}')
        pos += 1
    return source[m.start():pos]

def structure(text, name):
    end = text.index('} ' + name + ';') + len('} ' + name + ';')
    return text[text.rfind('typedef struct', 0, end):end]

app = function('app_main')
assert app.index('init_rtos_resources();') < app.index('ESP_ERROR_CHECK(init_nrf24());')
assert app.index('xTaskCreate(vTaskTransmitData') < app.index('calibrate_accel_for_mounting()')
assert app.index('calibrate_accel_for_mounting()') < app.index('xTaskCreate(vTaskAttitude')
assert 'pdMS_TO_TICKS(TELEMETRY_PERIOD_DEFAULT_MS)' in source
print('PASS startup order: resources -> radio task -> sensors/calibration -> acquisition tasks')

macros = [line for line in shared.splitlines()
          if re.match(r'#define (NRF24_(?:IDLE_TX_PERIOD_MS|RECORDING_TX_PERIOD_MS|RADIO_SAMPLE_MAX_AGE_US|TX_TIMEOUT_MS)|TELEMETRY_STATUS_[A-Z0-9_]+|SD_LOGGING_READY)\b', line)]
types = structure(shared, 'telemetry_data_t') + '\n' + structure(source, 'radio_sample_t')
prefix = r'''
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <setjmp.h>
#define CHECK(c) do { if (!(c)) { fprintf(stderr,"FAIL %d: %s\n",__LINE__,#c); exit(1); } } while(0)
#define BIT1 2U
#define BIT2 4U
#define COMMAND_CALIBRATING BIT1
#define APP_INITIALIZING BIT2
#define ESP_OK 0
#define pdTRUE 1
#define portTICK_PERIOD_MS 10U
#define NRF24_MODE_POWER_DOWN 0
#define NRF24_MODE_STANDBY 1
#define ESP_LOGI(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
typedef uint32_t EventBits_t;
typedef uint32_t TickType_t;
typedef int esp_err_t;
typedef struct {uint16_t sequence; uint8_t command;} nrf24_command_packet_t;
typedef struct {nrf24_command_packet_t command; uint8_t result; bool completed;} command_result_t;
typedef struct {nrf24_command_packet_t command; bool valid,pending; uint8_t result,state;} command_history_t;
static int EventGroup_status, Mutex_Logging, Queue_CommandResults, Queue_radio;
static void *s_radio_task=(void*)1;
static struct {int mode;} nrf24l01;
static EventBits_t flags=APP_INITIALIZING;
static int64_t clock_us;
static unsigned notifications, power_down_count, wake_count, ack_consumed;
static bool ack_pending, result_pending, produce=true;
static command_result_t pending_result;
static jmp_buf done;
static void set_logging_enabled(bool enabled);
static int64_t esp_timer_get_time(void) {return clock_us;}
static EventBits_t xEventGroupGetBits(int group) {(void)group;return flags;}
static void xEventGroupSetBits(int group,EventBits_t b) {(void)group;flags|=b;}
static void xEventGroupClearBits(int group,EventBits_t b) {(void)group;flags&=~b;}
static void xSemaphoreTake(int mutex,unsigned wait) {(void)mutex;(void)wait;}
static void xSemaphoreGive(int mutex) {(void)mutex;}
#define portMAX_DELAY UINT32_MAX
static void xTaskNotifyGive(void *task) {(void)task;++notifications;}
'''
fixture = r'''
typedef struct {int64_t time_us; telemetry_data_t data;} trace_t;
static trace_t trace[100];
static unsigned count;
static radio_sample_t latest;
static int xQueuePeek(int q,void *data,unsigned wait) {
    (void)q;(void)wait;
    if(flags & APP_INITIALIZING) return 0;
    if(produce) {
        latest.timestamp_us=clock_us/100000*100000;
        latest.data.sequence=(uint32_t)(latest.timestamp_us/100000);
        latest.data.status=TELEMETRY_STATUS_MPU6050_OK|TELEMETRY_STATUS_BMP180_OK|TELEMETRY_STATUS_GPS_FIX;
        latest.data.yaw_centi_deg=12000;latest.data.roll_centi_deg=120;
        latest.data.pitch_centi_deg=-250;latest.data.vertical_speed_cms=50;
    }
    memcpy(data,&latest,sizeof(latest));return 1;
}
static int xQueueReceive(int q,void *data,unsigned wait) {
    (void)q;(void)wait;
    if(!result_pending) return 0;
    memcpy(data,&pending_result,sizeof(pending_result));result_pending=false;return 1;
}
static esp_err_t nrf24l01_transmit(void *radio,const void *data,size_t size,unsigned timeout) {
    (void)radio;(void)timeout;
    CHECK(!ack_pending);CHECK(size==sizeof(telemetry_data_t));CHECK(count<100);
    if(nrf24l01.mode==NRF24_MODE_POWER_DOWN)++wake_count;
    trace[count].time_us=clock_us;memcpy(&trace[count++].data,data,size);
    nrf24l01.mode=NRF24_MODE_STANDBY;ack_pending=true;clock_us+=1000;return ESP_OK;
}
static void process_ack_command(command_history_t *history) {
    (void)history;if(ack_pending){ack_pending=false;++ack_consumed;}
}
static void send_command_response(const command_result_t *result,uint8_t state) {
    (void)result;(void)state;CHECK(!ack_pending);
    if(nrf24l01.mode==NRF24_MODE_POWER_DOWN)++wake_count;
    nrf24l01.mode=NRF24_MODE_STANDBY;ack_pending=true;clock_us+=1000;
}
static void log_nrf24_telemetry(const telemetry_data_t *data,esp_err_t result) {(void)data;CHECK(result==ESP_OK);}
static esp_err_t nrf24l01_power_down(void *radio) {
    (void)radio;CHECK(!ack_pending);nrf24l01.mode=NRF24_MODE_POWER_DOWN;++power_down_count;return ESP_OK;
}
static const int64_t events[]={2000000,3350000,4050000,4700000,5200000,6700000,7250000,7800000,8900000,9350000,10900000};
static unsigned event_index;
static void event(unsigned i) {
    if(i==0) flags&=~APP_INITIALIZING;
    if(i==1||i==6) {
        set_logging_enabled(true);result_pending=true;
        pending_result=(command_result_t){.completed=true,.result=0};
    }
    if(i==2||i==9) {
        set_logging_enabled(false);result_pending=true;
        pending_result=(command_result_t){.completed=true,.result=i==9?4:0};
    }
    if(i==3) { /* REC_ON fallido: no cambia estado ni frecuencia. */
        pending_result=(command_result_t){.completed=true,.result=4};result_pending=true;xTaskNotifyGive(s_radio_task);
    }
    if(i==4) flags|=COMMAND_CALIBRATING;
    if(i==5) flags&=~COMMAND_CALIBRATING;
    if(i==7) produce=false;
    if(i==8) produce=true;
    if(i==10) longjmp(done,1);
}
static uint32_t ulTaskNotifyTake(int clear,TickType_t ticks) {
    (void)clear;
    if(notifications) {notifications=0;return 1;}
    CHECK(nrf24l01.mode==NRF24_MODE_POWER_DOWN);
    int64_t deadline=clock_us+(int64_t)ticks*10000;
    while(event_index<sizeof(events)/sizeof(events[0]) && events[event_index]<=deadline) {
        clock_us=events[event_index];event(event_index++);
        if(notifications){notifications=0;return 1;}
    }
    clock_us=deadline;return 0;
}
'''
tests = r'''
int main(void) {
    CHECK(sizeof(telemetry_data_t)==32);
    CHECK(radio_period_ms(APP_INITIALIZING|SD_LOGGING_READY)==1000);
    CHECK(radio_period_ms(0)==1000);
    CHECK(radio_period_ms(SD_LOGGING_READY)==100);
    CHECK(radio_period_ms(SD_LOGGING_READY|COMMAND_CALIBRATING)==100);
    if(!setjmp(done))vTaskTransmitData(NULL);
    CHECK(count>20);CHECK(trace[0].time_us==0);
    bool boot=false,live=false,calibration=false,stale=false,recovered=false;
    bool start_fast=false,stop_slow=false;
    unsigned fast=0,slow_after_failure=0;
    for(unsigned i=0;i<count;i++) {
        int64_t t=trace[i].time_us;telemetry_data_t *d=&trace[i].data;
        CHECK(d->sequence==i);
        if(t<2000000){CHECK(d->status==0);CHECK(d->yaw_centi_deg==0);boot=true;}
        if(t>2000000&&t<3350000){CHECK(d->status&TELEMETRY_STATUS_MPU6050_OK);live=true;}
        if(t>=3350000&&t<4050000){CHECK(d->status&TELEMETRY_STATUS_SD_OK);++fast;}
        if(t>=3350000&&t<3360000)start_fast=true;
        if(t>=4050000&&t<4060000)stop_slow=true;
        if(t>=4700000&&t<5200000){CHECK(!(d->status&TELEMETRY_STATUS_SD_OK));++slow_after_failure;}
        if(t>=5200000&&t<6700000){CHECK(!(d->status&TELEMETRY_STATUS_MPU6050_OK));CHECK(d->yaw_centi_deg==0);calibration=true;}
        if(t>8400000&&t<8900000){CHECK(d->status==TELEMETRY_STATUS_SD_OK);stale=true;}
        if(t>9000000&&t<9350000){CHECK(d->status&TELEMETRY_STATUS_MPU6050_OK);recovered=true;}
        if(i>0&&t>=3450000&&t<4050000) {
            int64_t delta=t-trace[i-1].time_us;CHECK(delta>=90000&&delta<=110000);
        }
        if(i>0&&t>10000000)CHECK(t-trace[i-1].time_us>=990000);
    }
    CHECK(boot&&live&&calibration&&stale&&recovered&&start_fast&&stop_slow);
    CHECK(fast>=6);CHECK(slow_after_failure<=1);
    CHECK(power_down_count>=count);CHECK(wake_count>=count-1);CHECK(ack_consumed>=count);
    printf("PASS %u telemetry frames: early boot, 1/10 Hz, immediate REC_ON/OFF, failed REC_ON, calibration and stale data\n",count);
    printf("PASS %u power_down calls, %u wakeups, %u ACK payload consumptions before sleep\n",power_down_count,wake_count,ack_consumed);
    return 0;
}
'''
selected = '\n'.join(function(n) for n in ('radio_period_ms','prepare_radio_telemetry',
                                          'set_logging_enabled','command_state','vTaskTransmitData'))
(out/'test.c').write_text(prefix+'\n'+'\n'.join(macros)+'\n'+types+fixture+selected+tests,encoding='utf-8')
print(out/'test.c')
