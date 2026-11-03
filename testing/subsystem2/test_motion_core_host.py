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
 printf("Confirmed: left with zero encoder pulses reaches only %u/1000 PWM before stall timeout\\n",peak);
 for(unsigned i=0;i<20;i++) tick(0,6);
 assert(s2_core_faults() & 4u);
 assert(pwm[LEFT_IN_A]==0 && pwm[RIGHT_IN_A]==0);
 puts("PASS: stationary left wheel trips stall protection and stops BOTH motors");
}
'''
with tempfile.TemporaryDirectory() as temp:
    source=Path(temp)/'motion_test.c'; binary=Path(temp)/'motion_test'
    source.write_text(prelude+public+core_header+'\n'+(folder/'motion_config.h').read_text()+'\n'+core+tests)
    subprocess.run(['gcc','-std=gnu11','-Wall','-Wextra','-Werror',str(source),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
