"""Exercise main3's real navigation helpers with the real EKF/Kalman cores."""
from pathlib import Path
import re

root = Path(__file__).resolve().parents[2]
source = (root / "main/main3.c").read_text(encoding="utf-8")
shared = (root / "components/shared/include/shared.h").read_text(encoding="utf-8")
out = root / "tmp/main3_navigation"
out.mkdir(parents=True, exist_ok=True)


def function(name):
    match = re.search(r"static (?:void|bool|float|int|int16_t|int32_t|esp_err_t|filter_storage_record_t) " + name + r"\([^;]*?\)\s*\{", source)
    assert match, name
    pos, depth = match.end(), 1
    while depth:
        depth += (source[pos] == "{") - (source[pos] == "}")
        pos += 1
    return source[match.start():pos]


def structure(text, name):
    end = text.index("} " + name + ";") + len("} " + name + ";")
    return text[text.rfind("typedef struct", 0, end):end]


config_end = source.index("};", source.index("} s_test_config =")) + 2
config_start = source.rfind("static const struct {", 0, config_end)
config = source[config_start:config_end]
macros = "\n".join(line for line in shared.splitlines()
                   if re.match(r"#define (?:ALTITUDE_BARO_MAX_|ATTITUDE_MAX_AGE_US|TELEMETRY_CENTI_SCALE|TELEMETRY_STATUS_MPU6050_OK|DEG_TO_RAD|RAD_TO_DEG|YAW_MEASUREMENT_NOISE_STD_RAD|EKF_LOG_STATE_DIM|EKF_LOG_MPU_OK|EKF_LOG_INITIALIZING)", line))
types = "\n".join(structure(shared, name) for name in ["telemetry_data_t", "attitude_data_t", "ekf_log_record_t", "barometer_sample_t"])
types += "\n" + "\n".join(structure(source, name) for name in [
    "vertical_navigation_state_t", "attitude_supervision_t", "navigation_diagnostics_t", "filter_test_record_t",
    "filter_storage_record_t"])
names = ["vector_norm", "lowpass_update", "gravity_error", "attitude_sample_is_still",
         "barometer_is_stationary", "supervise_attitude", "update_barometer_tracking",
         "anchor_vertical_filter", "update_vertical_navigation", "apply_magnetic_yaw_correction", "update_imu_attitude",
         "scale_to_i16", "scale_to_i32", "update_telemetry_attitude",
         "write_navigation_csv", "compact_filter_record", "write_ekf_csv"]
prefix = r'''
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "altitude_kalman.h"
#include "ekq_q.h"
#define CHECK(c) do {if (!(c)) {fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#c);exit(1);}}while(0)
#define NEAR(a,b,tol) CHECK(isfinite(a) && fabsf((a)-(b))<(tol))
#define pdTRUE 1
static int64_t clock_us;
int64_t esp_timer_get_time(void) {return clock_us;}
'''
globals_code = r'''
static vertical_navigation_state_t s_vertical;
static attitude_supervision_t s_attitude_health;
static navigation_diagnostics_t s_navigation_diagnostics;
static altitude_kalman_t altitude_kalman;
static ekf_q_t ekf_q;
static ekf_q_config_t ekf_qcfg;
static struct {int unused;} s_magnetic_yaw;
static int Queue_Baro;
static bool queued;
static barometer_sample_t queued_baro;
static ekf_log_record_t sample;
static uint32_t next_sequence;
static int xQueueReceive(int queue, void *output, unsigned wait) {
    (void)queue;(void)wait;
    if(!queued)return 0;
    memcpy(output,&queued_baro,sizeof(queued_baro));queued=false;return pdTRUE;
}
'''
fixture = r'''
static void reset_fixture(void) {
    memset(&s_vertical,0,sizeof(s_vertical));
    memset(&s_attitude_health,0,sizeof(s_attitude_health));
    memset(&s_navigation_diagnostics,0,sizeof(s_navigation_diagnostics));
    memset(&sample,0,sizeof(sample));clock_us=1000000;queued=false;next_sequence=0;
    altitude_kalman_config_t config=ALTITUDE_KALMAN_DEFAULT_CONFIG();
    CHECK(altitude_kalman_init(&altitude_kalman,&config)==ESP_OK);
    ekf_qcfg=(ekf_q_config_t)EKF_Q_DEFAULT_CONFIG();
    CHECK(ekf_q_init(&ekf_q,&ekf_qcfg)==ESP_OK);
    const float gravity[3]={0,0,-9.80665f};
    CHECK(ekf_q_set_initial_attitude(&ekf_q,gravity)==ESP_OK);
}
static void step(bool imu_ok,float a_up,float gyro,bool with_baro,float height) {
    clock_us+=10000;
    sample=(ekf_log_record_t){0};
    sample.sequence=next_sequence++;
    sample.attitude.timestamp_us=clock_us;sample.attitude.valid=imu_ok;
    sample.accepted_axes=imu_ok?7:0;
    sample.accel_body_m_s2[2]=-9.80665f-a_up;
    sample.gyro_body_rad_s[0]=gyro;
    sample.attitude.linear_accel_nav_m_s2[2]=-a_up;
    if(with_baro){queued_baro=(barometer_sample_t){clock_us,height};queued=true;}
    update_vertical_navigation(&sample);
    CHECK(altitude_kalman.config.nis_gate_enabled);
}
static void stationary(unsigned count) {
    for(unsigned k=0;k<count;k++)step(true,0,0,k%10==0,0);
}
static void test_rest_and_full_acceleration(void) {
    reset_fixture();stationary(1000);
    CHECK(sample.attitude.vertical_valid && s_vertical.stationary);
    NEAR(sample.attitude.height_m,0,0.01f);
    NEAR(sample.attitude.vertical_velocity_m_s,0,0.01f);
    step(true,12.0f,0.5f,false,0);
    CHECK(!s_vertical.stationary);
    NEAR(s_navigation_diagnostics.accel_used_m_s2,12,0.01f);
    puts("PASS rest confirmation and full 12 m/s2 input (no clipping or 20% gain)");
}
static void test_movement_then_rest(void) {
    reset_fixture();stationary(1000);
    for(unsigned k=0;k<100;k++)step(true,3,0.5f,k%10==0,0);
    stationary(1500);
    CHECK(sample.attitude.vertical_valid && s_vertical.stationary);
    NEAR(sample.attitude.height_m,0,0.5f);
    NEAR(sample.attitude.vertical_velocity_m_s,0,0.15f);
    printf("PASS movement then rest: h=%.3f v=%.3f\n",sample.attitude.height_m,sample.attitude.vertical_velocity_m_s);
}
static void test_divergence_recovery(void) {
    reset_fixture();stationary(1000);
    altitude_kalman.state.height_m=50;
    altitude_kalman.state.vertical_velocity_m_s=5;
    stationary(300);
    CHECK(s_vertical.recoveries>0 && sample.attitude.vertical_valid);
    NEAR(sample.attitude.height_m,0,0.5f);
    NEAR(sample.attitude.vertical_velocity_m_s,0,0.2f);
    puts("PASS divergent h/v recovered using coherent barometer; NIS remains enabled");
}
static void test_isolated_outlier_and_duplicates(void) {
    reset_fixture();stationary(1000);
    step(true,0,0,true,100);
    CHECK(!s_navigation_diagnostics.baro_accepted && s_vertical.recoveries==0);
    stationary(100);
    NEAR(sample.attitude.height_m,0,0.2f);
    int64_t last=s_vertical.last_accepted_us;
    queued_baro=(barometer_sample_t){s_vertical.last_consumed_us,50};queued=true;
    step(true,0,0,false,0);
    CHECK(s_vertical.last_accepted_us==last);
    puts("PASS isolated 100 m outlier and repeated timestamp do not re-anchor the filter");
}
static void test_stale_future_and_reference(void) {
    reset_fixture();stationary(1000);
    for(unsigned k=0;k<120;k++)step(true,0,0,false,0);
    CHECK(!sample.attitude.vertical_valid);
    CHECK(!s_vertical.anchored && isnan(sample.attitude.height_m));
    queued_baro=(barometer_sample_t){clock_us-200000,500};queued=true;
    step(true,0,0,false,0);
    CHECK(!s_vertical.anchored);
    queued_baro=(barometer_sample_t){clock_us+30000,500};queued=true;
    step(true,0,0,false,0);CHECK(s_vertical.pending);
    step(true,0,0,false,0);CHECK(s_vertical.pending);
    step(true,0,0,false,0);CHECK(!s_vertical.pending);
    for(unsigned k=0;k<200;k++)step(true,0,0.5f,k%10==0,500);
    CHECK(sample.attitude.vertical_valid);
    NEAR(sample.attitude.height_m,500,0.1f);
    puts("PASS stale/future measurements, fallback and recovery preserve 500 m relative height");
}
static void test_barometer_without_imu(void) {
    reset_fixture();
    for(unsigned k=0;k<500;k++)step(false,0,0,k%10==0,20);
    CHECK(!sample.attitude.valid && sample.attitude.vertical_valid);
    CHECK(!s_vertical.stationary);
    NEAR(sample.attitude.height_m,20,0.1f);
    CHECK(isnan(s_navigation_diagnostics.accel_up_m_s2));
    puts("PASS IMU loss: independent barometric model remains valid and reports missing acceleration");
}
static void test_ascent_and_bias(void) {
    reset_fixture();
    for(unsigned k=0;k<5000;k++)step(true,0,0.5f,k%10==0,(float)(k+1)*0.1f);
    CHECK(sample.attitude.vertical_valid && !s_vertical.stationary);
    NEAR(sample.attitude.height_m,500,1.5f);
    NEAR(sample.attitude.vertical_velocity_m_s,10,1.0f);
    reset_fixture();
    for(unsigned k=0;k<4000;k++)step(true,0.15f,0,k%10==0,0);
    CHECK(s_vertical.stationary);
    NEAR(s_vertical.accel_bias_m_s2,0.15f,0.015f);
    puts("PASS 10 m/s ascent to 500 m and vertical-bias learning only at confirmed rest");
}
static void test_attitude_continuity(void) {
    reset_fixture();stationary(1000);
    for(unsigned k=0;k<900;k++){
        const float gyro[3]={0,0,k<400?25.0f*DEG_TO_RAD:0};
        step(true,0,0,k%10==0,0);
        CHECK(ekf_q_predict(&ekf_q,gyro,0.01f)==ESP_OK);
        sample.accepted_axes=0; /* Sin confirmacion nueva de gravedad. */
        memcpy(sample.gyro_body_rad_s,gyro,sizeof(gyro));
        ekf_q_get_quaternion(&ekf_q,sample.attitude.quaternion);
        ekf_q_get_euler(&ekf_q,&sample.attitude.roll_rad,&sample.attitude.pitch_rad,&sample.attitude.yaw_rad);
        float saved_x[7],saved_P[7][7];
        memcpy(saved_x,ekf_q.x,sizeof(saved_x));memcpy(saved_P,ekf_q.P,sizeof(saved_P));
        CHECK(supervise_attitude(&sample));
        CHECK(memcmp(saved_x,ekf_q.x,sizeof(saved_x))==0 && memcmp(saved_P,ekf_q.P,sizeof(saved_P))==0);
    }
    CHECK(s_attitude_health.resets==0);
    NEAR(sample.attitude.yaw_rad,100.0f*DEG_TO_RAD,0.01f);
    puts("PASS 4 s rotation plus 5 s rest without gravity acceptance: yaw stays at 100 deg, no expiry/reset");
}
static void test_csv(void) {
    filter_test_record_t record={.ekf=sample,.navigation=s_navigation_diagnostics};
    FILE *file=fopen("tmp/main3_navigation/record.csv","w");CHECK(file);
    filter_storage_record_t compact=compact_filter_record(&record);
    CHECK(write_ekf_csv(file,"",&compact)>0);CHECK(fclose(file)==0);
    puts("PASS SD diagnostic serialization");
}
static void test_attitude_matches_main2(void) {
    reset_fixture();
    static ekf_q_t reference;reference=ekf_q;
    const float gyro[3]={0,0,0.15f},force[3]={0,0,-9.80665f};
    for(unsigned k=0;k<1000;k++){
        CHECK(ekf_q_update(&reference,gyro,force,0.01f)==ESP_OK);
        CHECK(update_imu_attitude(gyro,force,0.01f)==ESP_OK);
        if(k%5==0){
            CHECK(ekf_q_correct_yaw(&reference,0.6f,YAW_MEASUREMENT_NOISE_STD_RAD)==ESP_OK);
            CHECK(apply_magnetic_yaw_correction(0.6f)==ESP_OK);
        }
        CHECK(memcmp(reference.x,ekf_q.x,sizeof(reference.x))==0);
        CHECK(memcmp(reference.P,ekf_q.P,sizeof(reference.P))==0);
    }
    float saved_x[7],saved_P[7][7];
    memcpy(saved_x,ekf_q.x,sizeof(saved_x));memcpy(saved_P,ekf_q.P,sizeof(saved_P));
    const float bad_gyro[3]={NAN,0,0};
    CHECK(update_imu_attitude(bad_gyro,force,0.01f)!=ESP_OK);
    CHECK(update_imu_attitude(gyro,force,0.2f)!=ESP_OK);
    CHECK(memcmp(saved_x,ekf_q.x,sizeof(saved_x))==0);
    CHECK(memcmp(saved_P,ekf_q.P,sizeof(saved_P))==0);
    CHECK(ekf_q.diagnostics.initialized);
    puts("PASS same x/P as main2 for 1000 IMU + 200 yaw corrections; invalid sample/dt keep the previous state");
}
static void test_telemetry_angle_continuity(void){
    reset_fixture();
    telemetry_data_t data={.altitude_cm=50000};
    attitude_data_t a={.timestamp_us=clock_us,.valid=true,.vertical_valid=true,
        .roll_rad=0.3f,.pitch_rad=-0.2f,.yaw_rad=1.1f,.height_m=123,.vertical_velocity_m_s=2};
    update_telemetry_attitude(&data,&a,clock_us);
    CHECK(data.altitude_cm==50000);
    CHECK(data.status&TELEMETRY_STATUS_MPU6050_OK);
    CHECK(data.roll_centi_deg!=0 && data.pitch_centi_deg!=0 && data.yaw_centi_deg!=0);
    const telemetry_data_t saved=data;
    a.valid=false;a.vertical_valid=false;
    update_telemetry_attitude(&data,&a,clock_us);
    CHECK(!(data.status&TELEMETRY_STATUS_MPU6050_OK));
    CHECK(data.roll_centi_deg==saved.roll_centi_deg && data.pitch_centi_deg==saved.pitch_centi_deg && data.yaw_centi_deg==saved.yaw_centi_deg);
    a.valid=true;a.roll_rad=0;a.pitch_rad=0;a.yaw_rad=0;
    update_telemetry_attitude(&data,&a,clock_us+ATTITUDE_MAX_AGE_US+1);
    CHECK(!(data.status&TELEMETRY_STATUS_MPU6050_OK) && data.yaw_centi_deg==saved.yaw_centi_deg);
    a.timestamp_us=clock_us+300000;a.roll_rad=0.4f;a.pitch_rad=-0.1f;a.yaw_rad=1.2f;
    update_telemetry_attitude(&data,&a,a.timestamp_us);
    CHECK((data.status&TELEMETRY_STATUS_MPU6050_OK) && data.yaw_centi_deg!=saved.yaw_centi_deg);
    puts("PASS missing/stale attitude keeps last angles with MPU_OK cleared, then resumes with new data");
}
int main(void) {
    test_rest_and_full_acceleration();test_movement_then_rest();test_divergence_recovery();
    test_isolated_outlier_and_duplicates();test_stale_future_and_reference();
    test_barometer_without_imu();test_ascent_and_bias();test_attitude_continuity();test_csv();
    test_attitude_matches_main2();test_telemetry_angle_continuity();
    return 0;
}
'''
(out / "test.c").write_text("\n".join([prefix, macros, config, types, globals_code]
    + [function(name) for name in names] + [fixture]), encoding="utf-8")
header = source[source.index("static const char s_filter_sd_header[]"):]
columns = "".join(re.findall(r'"([^"]*)"', header[:header.index(";")])).split(",")
(out / "columns.txt").write_text(",".join(columns), encoding="utf-8")
print("Generated host test from main3.c; expected CSV columns:", len(columns))
task=function("vTaskAttitude")
assert "ekf_q.diagnostics.initialized = false;" not in task
assert "ekf_q_set_initial_attitude" not in function("supervise_attitude")
print("PASS attitude task does not deinitialize EKF on read errors or dt gaps; supervisor never resets orientation")
