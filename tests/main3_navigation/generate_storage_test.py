"""Test main3's actual SD scheduling, compact CSV and optional-filter failures."""
from pathlib import Path
import re

root = Path(__file__).resolve().parents[2]
source = (root / "main/main3.c").read_text(encoding="utf-8")
shared = (root / "components/shared/include/shared.h").read_text(encoding="utf-8")
out = root / "tmp/main3_storage"
out.mkdir(parents=True, exist_ok=True)


def function(name):
    match = re.search(r"static (?:void|bool|int|uint32_t|esp_err_t|filter_storage_record_t) " + name + r"\([^;]*?\)\s*\{", source)
    assert match, name
    pos, depth = match.end(), 1
    while depth:
        depth += (source[pos] == "{") - (source[pos] == "}")
        pos += 1
    return source[match.start():pos]


def structure(text, name):
    end = text.index("} " + name + ";") + len("} " + name + ";")
    return text[text.rfind("typedef struct", 0, end):end]


end = source.index("};", source.index("} s_sd_policy =")) + 2
config = source[source.rfind("static const struct {", 0, end):end]
macros = "\n".join(line for line in shared.splitlines() if re.match(
    r"#define (?:TELEMETRY_COORDINATE_SCALE|TELEMETRY_CENTI_SCALE|NRF24_IDLE_TX_PERIOD_MS|NRF24_RECORDING_TX_PERIOD_MS|EKF_LOG_STATE_DIM|EKF_LOG_MPU_OK|EKF_LOG_INITIALIZING|SD_LOGGING_READY)", line))
paths = "\n".join(re.search(r"static const char " + name + r"\[\]\s*=.*?;", source, re.DOTALL).group(0)
                  for name in ["s_test_telemetry_path", "s_test_ekf_path", "s_filter_sd_header"])
types = "\n".join(structure(shared, name) for name in [
    "telemetry_data_t", "telemetry_log_record_t", "attitude_data_t", "ekf_log_record_t"])
types += "\n" + "\n".join(structure(source, name) for name in [
    "navigation_diagnostics_t", "filter_test_record_t", "filter_storage_record_t", "sd_logger_t"])
prefix = r'''
#define _CRT_SECURE_NO_WARNINGS
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <io.h>
#include "esp_err.h"
#include "esp_log.h"
#define fileno _fileno
#define fsync _commit
#define CHECK(c) do {if (!(c)) {fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#c);exit(1);}}while(0)
#define pdTRUE 1
#define BIT3 8U
#define APP_INITIALIZING 4U
#define SD_FILTER_LOGGING_READY BIT3
#define portMAX_DELAY UINT32_MAX
#define SD_FILE_BUFFER_SIZE 4096U
#define TELEMETRY_EMA_DATALOG_HEADER "telemetry_header"
typedef uint32_t EventBits_t;
'''
mocks = r'''
typedef struct {unsigned char records[64][512]; unsigned size,capacity,head,count;} mock_queue_t;
typedef mock_queue_t *QueueHandle_t;
static mock_queue_t tq,fq;
static QueueHandle_t Queue_data=&tq,Queue_EkfLog=&fq;
static int Mutex_Logging,EventGroup_status;
static EventBits_t flags;
static sd_logger_t s_logger;
static int64_t clock_us;
static unsigned write_cost_us;
static bool locked,inject_telemetry,fail_filter_write,fail_filter_flush,fail_telemetry_flush;
static bool fail_space_query;static uint64_t free_space;
static bool fail_telemetry_open,fail_filter_open;
static void *s_radio_task;
static struct {struct {const char *mount_point;} config;bool mounted;} sd_card={.config={"/sdcard"},.mounted=true};
static char trace[256];static unsigned trace_size;
static int64_t esp_timer_get_time(void){return clock_us;}
static esp_err_t esp_vfs_fat_info(const char *path,uint64_t *total,uint64_t *available){
    (void)path;*total=UINT64_C(8)*1024*1024*1024;*available=free_space;
    return fail_space_query?ESP_FAIL:ESP_OK;
}
static void xSemaphoreTake(int mutex,unsigned wait){(void)mutex;(void)wait;CHECK(!locked);locked=true;}
static void xSemaphoreGive(int mutex){(void)mutex;CHECK(locked);locked=false;}
static void xTaskNotifyGive(void *task){(void)task;}
static EventBits_t xEventGroupGetBits(int group){(void)group;return flags;}
static void xEventGroupSetBits(int group,EventBits_t bits){(void)group;flags|=bits;}
static void xEventGroupClearBits(int group,EventBits_t bits){(void)group;flags&=~bits;}
static unsigned uxQueueMessagesWaiting(QueueHandle_t q){return q->count;}
static void xQueueReset(QueueHandle_t q){q->head=q->count=0;}
static int xQueueSend(QueueHandle_t q,const void *record,unsigned wait){
    CHECK(wait==0);if(q->count==q->capacity)return 0;
    memcpy(q->records[(q->head+q->count)%q->capacity],record,q->size);q->count++;return pdTRUE;
}
static int xQueueReceive(QueueHandle_t q,void *record,unsigned wait){
    CHECK(wait==0);if(q->count==0)return 0;
    memcpy(record,q->records[q->head],q->size);q->head=(q->head+1)%q->capacity;q->count--;return pdTRUE;
}
static int host_fprintf(FILE *file,const char *format,...){
    if(file==s_logger.telemetry || file==s_logger.ekf){
        CHECK(trace_size+1<sizeof(trace));
        trace[trace_size++]=file==s_logger.telemetry?'T':'F';trace[trace_size]=0;
        clock_us+=write_cost_us;
        if(file==s_logger.ekf && inject_telemetry){
            telemetry_log_record_t record={0};CHECK(xQueueSend(Queue_data,&record,0)==pdTRUE);inject_telemetry=false;
        }
        if(file==s_logger.ekf && fail_filter_write)return -1;
    }
    va_list args;va_start(args,format);int result=vfprintf(file,format,args);va_end(args);return result;
}
static int host_fflush(FILE *file){
    if((file==s_logger.ekf && fail_filter_flush)||(file==s_logger.telemetry && fail_telemetry_flush))return -1;
    return fflush(file);
}
#define fprintf host_fprintf
#define fflush host_fflush
static esp_err_t close_log_file(FILE **file);
static esp_err_t flush_sd_logs(void);
static esp_err_t sd_init(void *card){(void)card;sd_card.mounted=true;return ESP_OK;}
static bool short_filename(const char *path){
    const char *base=strrchr(path,'/');base=base?base+1:path;
    const char *dot=strchr(base,'.');
    return dot && dot>base && dot-base<=8 && strlen(dot+1)>0 && strlen(dot+1)<=3;
}
/* Simula la restriccion real FF_USE_LFN=0. La escritura se hace en tmp. */
static FILE *open_csv_append(const char *path,const char *header,char *buffer,size_t size){
    if(!short_filename(path) || (fail_telemetry_open && strcmp(path,s_test_telemetry_path)==0)
        || (fail_filter_open && strcmp(path,s_test_ekf_path)==0))return NULL;
    char destination[256];const char *base=strrchr(path,'/')+1;
    snprintf(destination,sizeof(destination),"tmp/main3_storage/%s",base);
    FILE *file=fopen(destination,"w+");if(!file)return NULL;
    CHECK(setvbuf(file,buffer,_IOFBF,size)==0);CHECK(fputs(header,file)>=0);CHECK(fputc('\n',file)!=EOF);
    return file;
}
'''
names = ["compact_filter_record", "write_ekf_csv", "write_telemetry_csv", "close_log_file",
         "disable_filter_logging", "reserve_telemetry_space", "write_sd_batch", "flush_sd_logs", "enqueue_sd_record",
         "set_logging_enabled", "radio_period_ms", "start_sd_logging"]
fixture = r'''
static void reset_fixture(void){
    if(s_logger.telemetry)fclose(s_logger.telemetry);
    if(s_logger.ekf)fclose(s_logger.ekf);
    memset(&s_logger,0,sizeof(s_logger));memset(&tq,0,sizeof(tq));memset(&fq,0,sizeof(fq));
    tq.size=sizeof(telemetry_log_record_t);tq.capacity=s_sd_policy.telemetry_queue_length;
    fq.size=sizeof(filter_storage_record_t);fq.capacity=64;
    CHECK(tq.size<=512 && fq.size<=512);
    s_logger.telemetry=fopen("tmp/main3_storage/telemetry.csv","w+");
    s_logger.ekf=fopen("tmp/main3_storage/filter.csv","w+");
    CHECK(s_logger.telemetry && s_logger.ekf);
    flags=SD_LOGGING_READY|SD_FILTER_LOGGING_READY;locked=false;
    clock_us=1000000;write_cost_us=100;trace_size=0;trace[0]=0;
    inject_telemetry=fail_filter_write=fail_filter_flush=fail_telemetry_flush=false;
    fail_space_query=false;free_space=UINT64_C(1024)*1024*1024;
    fail_telemetry_open=fail_filter_open=false;
}
static void fill(unsigned telemetries,unsigned filters){
    telemetry_log_record_t telemetry={0};filter_storage_record_t filter={0};
    for(unsigned k=0;k<telemetries;k++)CHECK(xQueueSend(Queue_data,&telemetry,0)==pdTRUE);
    for(unsigned k=0;k<filters;k++)CHECK(xQueueSend(Queue_EkfLog,&filter,0)==pdTRUE);
}
static void test_priority(void){
    reset_fixture();fill(3,6);CHECK(write_sd_batch()==ESP_OK);
    CHECK(strcmp(trace,"TTTFF")==0 && tq.count==0 && fq.count==4);
    CHECK(s_logger.telemetry_records==3 && s_logger.ekf_records==2);
    reset_fixture();fill(0,6);inject_telemetry=true;CHECK(write_sd_batch()==ESP_OK);
    CHECK(strcmp(trace,"FTF")==0 && s_logger.telemetry_records==1);
    puts("PASS telemetry precedes filter and is rechecked after each optional write");
}
static void test_budget(void){
    reset_fixture();fill(0,6);write_cost_us=3000;inject_telemetry=true;
    CHECK(write_sd_batch()==ESP_OK);
    CHECK(strcmp(trace,"FT")==0 && fq.count==5 && tq.count==0);
    puts("PASS optional 2 ms budget stops the next filter write while telemetry still gets serviced");
}
static void test_pressure_and_reserved_capacity(void){
    reset_fixture();fill(4,0);filter_storage_record_t filter={0};telemetry_log_record_t telemetry={0};
    CHECK(!enqueue_sd_record(Queue_EkfLog,&filter));CHECK(s_logger.ekf_drops==1);
    CHECK(enqueue_sd_record(Queue_data,&telemetry));CHECK(s_logger.telemetry_drops==0);
    reset_fixture();fill(0,64);
    CHECK(!enqueue_sd_record(Queue_EkfLog,&filter));CHECK(enqueue_sd_record(Queue_data,&telemetry));
    CHECK(s_logger.ekf_drops==1 && tq.count==1 && fq.count==64);
    reset_fixture();fill(32,0);CHECK(!enqueue_sd_record(Queue_data,&telemetry));
    CHECK(s_logger.telemetry_drops==1);
    flags=0;CHECK(enqueue_sd_record(Queue_EkfLog,&filter));CHECK(fq.count==0);
    puts("PASS filter discarded first under pressure, independent telemetry capacity, explicit drop counters");
}
static void test_filter_failure(void){
    reset_fixture();fill(2,3);fail_filter_write=true;CHECK(write_sd_batch()==ESP_OK);
    CHECK(s_logger.telemetry && !s_logger.ekf && fq.count==0);
    CHECK(flags==SD_LOGGING_READY && s_logger.ekf_drops==3);
    telemetry_log_record_t telemetry={0};CHECK(enqueue_sd_record(Queue_data,&telemetry));
    CHECK(write_sd_batch()==ESP_OK && s_logger.telemetry_records==3);
    reset_fixture();fill(0,2);fail_filter_flush=true;CHECK(flush_sd_logs()==ESP_OK);
    CHECK(s_logger.telemetry && !s_logger.ekf && flags==SD_LOGGING_READY && fq.count==0);
    reset_fixture();fail_telemetry_flush=true;CHECK(flush_sd_logs()==ESP_FAIL);
    puts("PASS optional write/flush failures preserve telemetry; telemetry failure is propagated");
}
static void test_compact_record(void){
    reset_fixture();filter_test_record_t full={0};
    full.ekf.attitude.timestamp_us=1234567;full.navigation.baro_age_us=4567;
    full.ekf.attitude.valid=true;full.ekf.attitude.vertical_valid=true;
    full.navigation.stationary=true;full.navigation.baro_accepted=true;
    full.ekf.status=EKF_LOG_MPU_OK;full.ekf.accepted_axes=7;
    full.ekf.accel_body_m_s2[2]=-9.80665f;full.ekf.attitude.quaternion[0]=1;
    filter_storage_record_t compact=compact_filter_record(&full);
    CHECK(compact.baro_timestamp_us==1230000 && compact.flags==31 && compact.accepted_axes==7);
    CHECK(sizeof(compact)<sizeof(full));s_logger.ekf_drops=9;
    CHECK(enqueue_sd_record(Queue_EkfLog,&compact));
    CHECK(xQueueReceive(Queue_EkfLog,&compact,0)==pdTRUE && compact.dropped_records==9);
    FILE *csv=fopen("tmp/main3_storage/compact.csv","w");CHECK(csv);
    CHECK(write_ekf_csv(csv,"",&compact)>0);CHECK(fclose(csv)==0);
    printf("PASS compact CSV; queued record reduced from %zu to %zu bytes\n",sizeof(full),sizeof(compact));
}
static void test_space_reserve(void){
    reset_fixture();reserve_telemetry_space();CHECK(s_logger.ekf);
    fill(2,3);free_space=s_sd_policy.telemetry_reserve_bytes;
    reserve_telemetry_space();
    CHECK(!s_logger.ekf && s_logger.telemetry && tq.count==2 && fq.count==0);
    CHECK(flags==SD_LOGGING_READY && s_logger.ekf_drops==3);
    CHECK(write_sd_batch()==ESP_OK && s_logger.telemetry_records==2);
    reset_fixture();fail_space_query=true;reserve_telemetry_space();
    CHECK(!s_logger.ekf && s_logger.telemetry && flags==SD_LOGGING_READY);
    puts("PASS disk-space reserve or failed capacity query disables only the optional filter log");
}
static void test_recording_changes_radio_rate(void){
    reset_fixture();fclose(s_logger.telemetry);fclose(s_logger.ekf);
    s_logger.telemetry=s_logger.ekf=NULL;flags=0;
    CHECK(!short_filename("/sdcard/telem_ema.csv"));
    CHECK(short_filename(s_test_telemetry_path) && short_filename(s_test_ekf_path));
    CHECK(radio_period_ms(flags)==1000);
    CHECK(start_sd_logging()==ESP_OK);
    CHECK(s_logger.telemetry && s_logger.ekf && (flags&SD_LOGGING_READY));
    CHECK(radio_period_ms(flags)==100);
    set_logging_enabled(false);CHECK(radio_period_ms(flags)==1000);
    reset_fixture();fclose(s_logger.telemetry);fclose(s_logger.ekf);
    s_logger.telemetry=s_logger.ekf=NULL;flags=0;fail_telemetry_open=true;
    CHECK(start_sd_logging()!=ESP_OK && radio_period_ms(flags)==1000);
    fail_telemetry_open=false;fail_filter_open=true;
    CHECK(start_sd_logging()==ESP_OK && s_logger.telemetry && !s_logger.ekf);
    CHECK(radio_period_ms(flags)==100);
    puts("PASS real REC_ON path uses 8.3 filenames and switches 1 -> 10 Hz; telemetry-open failure stays at 1 Hz");
}
int main(void){
    test_priority();test_budget();test_pressure_and_reserved_capacity();test_filter_failure();test_compact_record();test_space_reserve();
    test_recording_changes_radio_rate();
    if(s_logger.telemetry)fclose(s_logger.telemetry);if(s_logger.ekf)fclose(s_logger.ekf);
    return 0;
}
'''
(out / "test.c").write_text("\n".join([prefix, macros, config, paths, types, mocks]
    + [function(name) for name in names] + [fixture]), encoding="utf-8")
print("Generated storage test from actual main3.c functions")
