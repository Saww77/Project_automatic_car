"""Exercise the actual motion core with mocked PWM, time and encoder inputs."""
from pathlib import Path
import subprocess
import tempfile
root = Path(__file__).resolve().parents[2]
folder = root / 'app_program/subsystem2'
public = (folder/'subsystem2.h').read_text().replace('#include "../app.h"', '')
core_header = (folder/'motion_core.h').read_text().replace('#include "subsystem2.h"', '')
core = '\n'.join(line for line in (folder/'motion_core.c').read_text().splitlines()
                 if not line.startswith('#include '))
prelude = '''#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
typedef int ER; typedef int INT; typedef unsigned int UW;
#define EXPORT
#define E_OK 0
#define DI(x) ((x)=0)
#define EI(x) ((void)(x))
#define S2_GPIO_EDGE_RISE 8u
static uint64_t mock_time;
static unsigned pwm[32];
void s2_hw_pwm_write(unsigned pin,unsigned level) { pwm[pin]=level; }
void s2_hw_pwm_init(void) { for(unsigned i=0;i<32;i++)pwm[i]=0; }
uint64_t s2_hw_time_us(void) { return mock_time; }
ER s2_hw_encoder_init(void) { return E_OK; }
'''
transport_header = (folder/'motion_transport.h').read_text().replace('#include "subsystem2.h"', '')
transport = '\n'.join(line for line in (folder/'motion_transport.c').read_text().splitlines()
                      if not line.startswith('#include '))
owner = (folder/'task_motion.c').read_text()
execute = owner[owner.index('static bool execute'):owner.index('EXPORT void motion_control_task')]
rtos_mock = """
typedef int ID;
typedef struct { unsigned mtxatr; } T_CMTX;
typedef struct { uint32_t lo; } SYSTIM;
#define E_OBJ -1
#define TA_INHERIT 0
#define TMO_FEVR -1
#define UB unsigned char
static int tm_printf(UB *fmt,...) { (void)fmt;return 0; }
static int tk_cre_mtx(T_CMTX *c) { (void)c;return 1; }
static int tk_loc_mtx(ID id,int t) { (void)id;(void)t;return 0; }
static int tk_unl_mtx(ID id) { (void)id;return 0; }
static int tk_slp_tsk(int t) { (void)t;return 0; }
static void tk_get_tim(SYSTIM *t) { t->lo=mock_time/1000; }
"""
tests = '''
static void tick(unsigned left_edges, unsigned right_edges) {
 for(unsigned i=0;i<left_edges;i++) subsystem2_encoder_gpio_irq(LEFT_ENC_A,8);
 for(unsigned i=0;i<right_edges;i++) subsystem2_encoder_gpio_irq(RIGHT_ENC_A,8);
 mock_time+=10000; s2_core_service();
}
int main(void) {
 s2_core_init(); assert(s2_core_set_wheel_levels(1,1)); tick(0,0);
 assert(pwm[LEFT_IN_A]==320 && pwm[RIGHT_IN_A]==320);
 assert(pwm[LEFT_IN_B]==0 && pwm[RIGHT_IN_B]==0);
 assert(s2_core_set_wheel_levels(0,1)); tick(0,0);
 assert(pwm[LEFT_IN_A]==0 && pwm[LEFT_IN_B]==0 && pwm[RIGHT_IN_A]>0);
 assert(s2_core_set_wheel_levels(1,1)); tick(0,0);
 assert(pwm[LEFT_IN_A]>0 && pwm[RIGHT_IN_A]>0);
 puts("PASS: both forward PWM outputs, left stop, left restart at level 1");
 s2_core_init(); assert(s2_core_set_wheel_levels(1,1));
 unsigned peak=0;
 for(unsigned i=0;i<40;i++) {
  tick(0,6); /* left stationary; right near its 60 RPM target */
  if(pwm[LEFT_IN_A]>peak) peak=pwm[LEFT_IN_A];
  assert(s2_core_faults()==0);
 }
 assert(peak==340);
 printf("Confirmed: legacy 60 RPM correction remains %u/1000 PWM before stall timeout\\n",peak);
 for(unsigned i=0;i<20;i++) tick(0,6);
 assert(s2_core_faults() & 4u);
 assert(pwm[LEFT_IN_A]==0 && pwm[RIGHT_IN_A]==0);
 puts("PASS: stationary left wheel trips stall protection and stops BOTH motors");
 s2_core_init(); assert(s2_core_set_wheel_rpm(30,30));
 peak=0;
 for(unsigned i=0;i<45;i++) {
  tick(0,0);
  if(pwm[LEFT_IN_A]>peak) peak=pwm[LEFT_IN_A];
  assert(s2_core_faults()==0);
 }
 assert(peak>350);
 assert(s2_core_set_wheel_rpm(12,24)); tick(1,2);
 assert(pwm[LEFT_IN_A]>0 && pwm[RIGHT_IN_A]>0);
 assert(s2_core_set_wheel_rpm(0,24)); tick(0,2);
 assert(pwm[LEFT_IN_A]==0 && pwm[RIGHT_IN_A]>0);
 assert(s2_core_set_wheel_rpm(30,30)); tick(0,2);
 assert(pwm[LEFT_IN_A]>0 && pwm[RIGHT_IN_A]>0);
 assert(!s2_core_set_wheel_rpm(241,30));
 puts("PASS: 30 RPM target overcomes old 34% correction ceiling; 12/24 RPM curve, stop/restart, bounds");
 assert(s2_transport_init()==0);
 S2_STATUS snapshot; s2_core_snapshot(&snapshot);s2_publish(&snapshot,true);
 for(unsigned l=0;l<=126;l+=2) for(unsigned r=0;r<=126;r+=2) {
  uint16_t word;uint32_t issued;
  if(!l&&!r)continue;
  assert(motion_set_wheel_rpm(l,r));
  assert(s2_take_request(&word,&issued));
  assert((word&15u)==S2_WHEEL_RPM);
  assert(execute(word));
  assert(state.left_rpm==l && state.right_rpm==r);
 }
 assert(!motion_set_wheel_rpm(31,30));
 assert(!motion_set_wheel_rpm(128,30));
 assert(motion_set_wheel_rpm(0,0));
 bool zero=true;assert(s2_take_stop(&zero) && !zero);
 puts("PASS: all 4095 fine-RPM command pairs round-trip through real queue and owner decoder; invalid inputs and zero stop");
}
'''
with tempfile.TemporaryDirectory() as temp:
    source=Path(temp)/'motion_test.c'; binary=Path(temp)/'motion_test'
    source.write_text(prelude+public+core_header+'\n'+(folder/'motion_config.h').read_text()+'\n'+core+rtos_mock+transport_header+transport+execute+tests)
    subprocess.run(['gcc','-std=gnu11','-Wall','-Wextra','-Werror',str(source),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
