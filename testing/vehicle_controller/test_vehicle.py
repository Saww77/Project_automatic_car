"""Compile actual controller/telemetry C with mocked RTOS, motors and MQTT transport."""
from pathlib import Path
import subprocess
import tempfile
root=Path(__file__).resolve().parents[2]
def read(name):return (root/name).read_text()
def strip(text):return '\n'.join(x for x in text.splitlines() if not x.startswith('#include '))+'\n'
prelude=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <setjmp.h>
#define TM_MQTT 1
#define VC_AUTO_START 0
#define EXPORT
#define IMPORT extern
typedef unsigned char UB;
static int tm_printf(UB *format,...){(void)format;return 0;}
typedef int ER;typedef int INT;typedef int BOOL;typedef int ID;typedef unsigned UW;
typedef void (*FP)(void);
typedef struct { unsigned mtxatr; } T_CMTX;
typedef struct { uint32_t lo; } SYSTIM;
#define TRUE 1
#define FALSE 0
#define E_OK 0
#define E_OBJ -1
#define E_NOSPT -2
#define E_PAR -3
#define E_TMOUT -4
#define TA_INHERIT 1
#define TP_PRC1 1
#define TMO_FEVR -1
static uint32_t clock_ms=10000;
static int tk_cre_mtx(T_CMTX *p){(void)p;return 1;}
static void on_lock(void);
static int tk_loc_mtx(ID id,int timeout){(void)id;(void)timeout;on_lock();return 0;}
static int tk_unl_mtx(ID id){(void)id;return 0;}
static jmp_buf command_return;
static bool exit_command_loop;
static int tk_dly_tsk(int t){(void)t;if(exit_command_loop)longjmp(command_return,1);return 0;}
static void tk_get_tim(SYSTIM *t){t->lo=clock_ms;}
static ID app_start_task(FP entry,int pri,UW core,const char *name)
{(void)entry;(void)pri;(void)core;(void)name;return 1;}
'''
headers=''.join(strip(read(x)) for x in (
'app_program/subsystem2/subsystem2.h',
'app_program/subsystem2/motion_core.h',
'app_program/subsystem2/motion_config.h',
'app_program/vehicle_controller/vehicle_core.h',
'app_program/vehicle_controller/vehicle_controller.h',
'app_program/subsystem1/s1_wire.h',
'app_program/subsystem1/s1_comm.h',
'lib/liblwip/sysdepend/pico_rp2040/mqtt_bridge.h'))
mocks=r'''
static bool motor_ready=true;
static S2_STATUS motor_status;
static unsigned links=7,commands,stops,last_left,last_right;
static bool queue_ok=true,fail_tx;
static unsigned clear_requests;
static uint32_t timing_gap=12000;
uint32_t s2_core_timing_fault_gap_us(void){return timing_gap;}
bool motion_clear_faults(void){clear_requests++;s2_put_bits(&motor_status,5,4,0);timing_gap=0;return true;}
static unsigned char rx_ops[8];static unsigned rx_count,rx_index;
int mqtt_bridge_receive(unsigned char frame[MQTT_BRIDGE_FRAME_MAX]){
 if(rx_index==rx_count)return E_TMOUT;
 frame[0]=MQTT_ROUTE_COMMAND;frame[1]=rx_ops[rx_index++];return 2;
}
static unsigned sent[MQTT_ROUTE_COUNT],payload_size[MQTT_ROUTE_COUNT];
static unsigned char payload[MQTT_ROUTE_COUNT][32];
bool motion_get_status(S2_STATUS *s){*s=motor_status;return motor_ready;}
bool motion_ready(void){return motor_ready;}
bool motion_set_wheel_rpm(unsigned l,unsigned r){
 if(!queue_ok)return false;
 commands++;last_left=l;last_right=r;return true;
}
void motion_stop(void){stops++;}
unsigned mqtt_bridge_status(void){return links;}
int mqtt_bridge_send(unsigned route,const unsigned char *bytes,unsigned length){
 assert(route<MQTT_ROUTE_COUNT && length<=32);
 if(fail_tx)return E_TMOUT;
 sent[route]++;payload_size[route]=length;memcpy(payload[route],bytes,length);return 0;
}
'''
comm=read('app_program/subsystem1/s1_comm.c')

sources=''.join(strip(x) for x in (
read('app_program/vehicle_controller/vehicle_core.c'),
read('app_program/vehicle_controller/vehicle_controller.c'),
read('app_program/subsystem1/s1_wire.c'),comm,
read('app_program/subsystem1/s1_telemetry.c')))
sources += r'''
static bool update_during_lock;
static void on_lock(void){
 if(update_during_lock){update_during_lock=false;data.line_timestamp_ms=++clock_ms;data.line_valid=TRUE;}
}
'''
tests=r'''
static void line(void){vehicle_submit_line(100,0,900,24,0,FALSE,0);}
static void arm(void){line();assert(vehicle_request_start());vehicle_service_once();assert(model.running);}
int main(void){
 assert(vehicle_init()==0);
 VEHICLE_SNAPSHOT diag;
 assert(vehicle_get_snapshot(&diag) && diag.start_block==VC_START_NO_LINE);
 motor_ready=false;assert(vehicle_get_snapshot(&diag) && diag.start_block==VC_START_STARTUP);
 motor_ready=true;s2_put_bits(&motor_status,5,4,4);
 assert(vehicle_get_snapshot(&diag) && diag.start_block==VC_START_MOTOR);
 s2_put_bits(&motor_status,5,4,0);
 line();clock_ms+=151;assert(vehicle_get_snapshot(&diag) && diag.start_block==VC_START_OLD_LINE);
 line();vehicle_service_once();assert(commands==0);
 arm();assert(last_left==24 && last_right==0);
 vehicle_submit_line(800,0,100,0,24,FALSE,0);
 vehicle_submit_line(100,0,800,24,12,FALSE,0);
 vehicle_service_once();assert(last_left==24 && last_right==12);
 unsigned before=commands;
 vehicle_request_stop();assert(!vehicle_request_start());line();vehicle_service_once();
 assert(!model.running && stops==1 && commands==before);
 vehicle_service_once();assert(commands==before);
 update_during_lock=true;assert(vehicle_request_start());
 update_during_lock=true;vehicle_service_once();assert(model.running);
 arm();links=1;vehicle_service_once();assert(model.reason==VC_REASON_LINK && !model.running);
 links=7;line();vehicle_service_once();assert(!model.running);
 arm();clock_ms+=151;vehicle_service_once();assert(model.reason==VC_REASON_STALE_LINE && !model.running);
 arm();vehicle_invalidate_line();vehicle_service_once();assert(!model.running);
 arm();vehicle_submit_line(0,0,0,0,0,TRUE,0);line();
 assert(!vehicle_request_start());vehicle_service_once();assert(!model.running);
 arm();assert(vehicle_submit_obstacle(100,80,200,300,TRUE));vehicle_service_once();
 assert(!vehicle_request_start() && !model.running);
 assert(vehicle_submit_obstacle(500,80,200,300,FALSE));line();vehicle_service_once();assert(!model.running);
 arm();s2_put_bits(&motor_status,5,4,4);vehicle_service_once();assert(model.reason==VC_REASON_MOTOR);
 assert(!vehicle_request_start());s2_put_bits(&motor_status,5,4,0);
 arm();vehicle_request_stop();vehicle_service_once();
 clock_ms=UINT32_MAX-10;line();clock_ms=9;assert(vehicle_request_start());vehicle_service_once();assert(model.running);
 before=commands;queue_ok=false;vehicle_submit_line(500,0,900,20,10,FALSE,0);
 vehicle_service_once();assert(commands==before);queue_ok=true;vehicle_service_once();assert(commands==before+1);
 vehicle_request_stop();vehicle_service_once();
 puts("PASS: sole-owner curve commands, latest value, STOP priority, latched line stop, link/fault/stale stops, explicit restart, obstacle interlock, clock wrap, queue retry");
 /* Actual publish functions and binary encoders. */
 memset(sent,0,sizeof(sent));line();
 data.terrain_valid=data.obstacle_valid=FALSE;
 s2_put_bits(&motor_status,9,9,30);s2_put_bits(&motor_status,18,9,24);
 s1_publish_telemetry();
 assert(sent[MQTT_ROUTE_VEHICLE]==1 && sent[MQTT_ROUTE_MOTION]==1 && sent[MQTT_ROUTE_LINE]==1);
 assert(payload_size[MQTT_ROUTE_CONTROLLER]==31 && payload[MQTT_ROUTE_CONTROLLER][0]==2);
 assert(payload_size[MQTT_ROUTE_MOTION]==9 && payload_size[MQTT_ROUTE_LINE]==1);
 assert(payload[MQTT_ROUTE_LINE][0]==4);
 S2_STATUS received;memcpy(received.bytes,payload[MQTT_ROUTE_MOTION],9);
 assert(s2_get_bits(&received,9,9)==30 && s2_get_bits(&received,18,9)==24);
 assert(sent[MQTT_ROUTE_TERRAIN]==0 && sent[MQTT_ROUTE_OBSTACLE]==0);
 assert(vehicle_submit_terrain(125,2));assert(vehicle_submit_obstacle(500,80,200,300,FALSE));
 s1_publish_telemetry();
 assert(payload_size[MQTT_ROUTE_TERRAIN]==2 && s1_wire_get(payload[MQTT_ROUTE_TERRAIN],0,11)==125);
 assert(payload_size[MQTT_ROUTE_OBSTACLE]==6 && s1_wire_get(payload[MQTT_ROUTE_OBSTACLE],0,12)==500);
 clock_ms+=1001;unsigned terrain_count=sent[MQTT_ROUTE_TERRAIN],line_count=sent[MQTT_ROUTE_LINE];
 s1_publish_telemetry();assert(sent[MQTT_ROUTE_TERRAIN]==terrain_count && sent[MQTT_ROUTE_LINE]==line_count);
 unsigned dropped=data.telemetry_drops;fail_tx=true;line();s1_publish_telemetry();assert(data.telemetry_drops==dropped+4);
 fail_tx=false;links=0;before=sent[MQTT_ROUTE_VEHICLE];s1_publish_telemetry();assert(sent[MQTT_ROUTE_VEHICLE]==before);
 unsigned op=99;unsigned char start=1,stop=0,bad=3;
 assert(s1_command_decode(&start,1,&op)&&op==1);assert(s1_command_decode(&stop,1,&op)&&op==0);
 assert(!s1_command_decode(&bad,1,&op));assert(!s1_command_decode(&start,2,&op));
 puts("PASS: vehicle/motion/line binary telemetry, real-only terrain/obstacle, freshness, backpressure count, disconnected suppression, strict binary commands");
 links=7;line();rx_index=0;rx_count=3;rx_ops[0]=1;rx_ops[1]=0;rx_ops[2]=1;
 before=commands;exit_command_loop=true;
 if(setjmp(command_return)==0)s1_comm_task(0,0);
 exit_command_loop=false;
 assert(stop_pending && !start_pending && commands==before);
 assert(payload[MQTT_ROUTE_ACK][0]==3); /* final START rejected after STOP */
 vehicle_service_once();assert(!model.running);
 rx_index=0;rx_count=1;rx_ops[0]=1;exit_command_loop=true;
 if(setjmp(command_return)==0)s1_comm_task(0,0);
 exit_command_loop=false;
 assert(start_pending && payload[MQTT_ROUTE_ACK][0]==1 && commands==before);
 vehicle_service_once();assert(model.running);
 puts("PASS: actual MQTT command task queues intent only, START/STOP/START prioritises STOP, ACK marks acceptance/rejection");
 assert(!vehicle_request_reset()); /* moving */
 vehicle_request_stop();vehicle_service_once();
 s2_put_bits(&motor_status,5,4,2);vehicle_service_once();
 assert(vehicle_get_snapshot(&diag) && diag.timing_fault_gap_us==12000 && diag.control_gap_limit_us==15000);
 before=commands;rx_index=0;rx_count=1;rx_ops[0]=2;exit_command_loop=true;
 if(setjmp(command_return)==0)s1_comm_task(0,0);
 exit_command_loop=false;
 assert(reset_pending && payload[MQTT_ROUTE_ACK][0]==4);
 vehicle_service_once();vehicle_service_once();
 assert(clear_requests==1 && !model.running && model.state==VC_IDLE && commands==before && !reset_pending);
 assert(!vehicle_request_reset()); /* no fault */
 line();assert(vehicle_request_start());vehicle_service_once();assert(model.running);
 puts("PASS: stopped-only RESET routes through controller, clears fault without motion, requires a separate START; timing gap/limit diagnostics");
}
'''
with tempfile.TemporaryDirectory() as temp:
 source=Path(temp)/'test.c';binary=Path(temp)/'test'
 source.write_text(prelude+headers+mocks+sources+tests)
 subprocess.run(['gcc','-std=gnu11','-Wall','-Wextra','-Werror',str(source),'-o',str(binary)],check=True)
 subprocess.run([str(binary)],check=True)

# Compile and exercise the actual non-MQTT autostart policy too.
normal_tests = r'''
int main(void){
 links=0;motor_ready=false;assert(vehicle_init()==0);
 vehicle_submit_line(100,0,900,24,0,FALSE,0);vehicle_service_once();assert(!model.running);
 motor_ready=true;vehicle_service_once();assert(model.running && last_left==24 && last_right==0);
 vehicle_request_stop();vehicle_service_once();vehicle_service_once();assert(!model.running);
 assert(vehicle_request_start());vehicle_service_once();assert(model.running);
 puts("PASS: non-MQTT waits for motor readiness, autostarts once, STOP cancels autostart, explicit restart");
}
'''
with tempfile.TemporaryDirectory() as temp:
 source=Path(temp)/'test.c';binary=Path(temp)/'test'
 source.write_text(prelude.replace('#define TM_MQTT 1','#define TM_MQTT 0').replace('#define VC_AUTO_START 0','#define VC_AUTO_START 1')+headers+mocks+sources+normal_tests)
 subprocess.run(['gcc','-std=gnu11','-Wall','-Wextra','-Werror','-Wno-unused-function','-Wno-unused-variable',str(source),'-o',str(binary)],check=True)
 subprocess.run([str(binary)],check=True)
