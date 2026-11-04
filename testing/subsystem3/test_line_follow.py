"""Compile the actual PD controller and exercise stateful sensor sequences."""
from pathlib import Path
import subprocess
import tempfile
root=Path(__file__).resolve().parents[2]
folder=root/'app_program/subsystem3'
tests=r'''
#include <assert.h>
#include <stdio.h>
#include "line_control.h"
static void sample(int l,int c,int r,unsigned dt,LF_STATE *s,LF_CMD *o){
 line_decide_timed(l,c,r,dt,s,o);
 assert(o->left>=0&&o->left<=S3_MAX_RPM&&o->right>=0&&o->right<=S3_MAX_RPM);
 assert(o->left%2==0&&o->right%2==0);
 if(o->stop)assert(o->left==0&&o->right==0);
}
int main(void){
 assert(line_normalize(130,130,3200)==0);
 assert(line_normalize(3200,130,3200)==1000);
 assert(line_normalize(3200,3200,130)==0);
 assert(line_normalize(130,3200,130)==1000);
 assert(line_normalize(4095,130,3200)==1000);
 assert(line_normalize(0,130,3200)==0);
 assert(line_normalize(1000,1000,1050)==-1);
 assert(line_normalize(-1,130,3200)==-1);
 int junction=0;for(int i=0;i<500;i++)assert(!line_junction_update(1000,1000,false,&junction));
 assert(junction==0);
 for(int i=0;i<JUNCTION_MIN_MS/LINE_PERIOD_MS-1;i++)assert(!line_junction_update(1000,1000,true,&junction));
 assert(line_junction_update(1000,1000,true,&junction));
 LF_STATE s={0};LF_CMD o;
 for(int i=0;i<500;i++){sample(25,(S3_IR_MASK&2)?1000:0,16,10,&s,&o);assert(!o.stop&&o.left==24&&o.right==24);}
 s=(LF_STATE){0};sample(0,0,600,10,&s,&o);assert(o.left>o.right);
 s=(LF_STATE){0};sample(600,0,0,10,&s,&o);assert(o.left<o.right);
 /* Derivative damps the returning error rather than forcing the opposite turn. */
 s=(LF_STATE){.initialized=true,.error=600};sample(0,0,0,10,&s,&o);
 assert(s.derivative<0&&o.left>=o.right);
 for(int i=0;i<15;i++)sample(0,0,0,10,&s,&o);
 assert(o.left==24&&o.right==24&&s.error==0);
 /* Sustained evidence enters a slow tight turn; an isolated hit does not. */
 for(int direction=-1;direction<=1;direction+=2){
  s=(LF_STATE){0};
  for(int i=0;i<S3_TIGHT_CONFIRM_MS/10;i++)sample(direction<0?1000:0,0,direction>0?1000:0,10,&s,&o);
  assert(s.mode==LF_TIGHT);
  assert(direction<0?(o.left==0&&o.right==20):(o.left==20&&o.right==0));
  sample(0,0,0,10,&s,&o);assert(s.mode==LF_RECOVER&&!o.stop);
  sample(direction>0?1000:0,0,direction<0?1000:0,10,&s,&o);
  assert(!o.stop&&s.mode==LF_TRACK); /* visible opposite edge reacquired */
 }
 s=(LF_STATE){0};sample(1000,0,0,10,&s,&o);sample(0,0,0,10,&s,&o);assert(s.mode==LF_TRACK);
 s=(LF_STATE){0};for(int i=0;i<S3_TIGHT_CONFIRM_MS/10;i++)sample(1000,0,0,10,&s,&o);
 for(int i=0;i<S3_RECOVERY_MS/10;i++)sample(0,0,0,10,&s,&o);
 assert(o.stop&&s.mode==LF_LOST);
 sample(1000,0,0,10,&s,&o);assert(!o.stop&&s.mode==LF_TRACK);
 /* Actual elapsed time, not number of calls, bounds recovery. */
 s=(LF_STATE){0};for(int i=0;i<4;i++)sample(1000,0,0,20,&s,&o);
 assert(s.mode==LF_TIGHT);
 sample(0,0,0,S3_RECOVERY_MS+20,&s,&o);assert(o.stop);
 s=(LF_STATE){0};for(int i=0;i<S3_THREE_SENSOR_LOST_MS/10;i++)sample(0,0,0,10,&s,&o);
 assert(o.stop==((S3_IR_MASK&2)!=0));
 /* S-curve reversals and recovery do not latch the old turn direction. */
 s=(LF_STATE){0};for(int i=0;i<10;i++)sample(900,0,100,10,&s,&o);
 for(int i=0;i<10;i++)sample(100,0,900,10,&s,&o);
 assert(o.left>o.right&&s.last_direction==1);
 for(int l=0;l<=1000;l+=100)for(int c=0;c<=1000;c+=100)for(int r=0;r<=1000;r+=100){
  s=(LF_STATE){0};for(int t=0;t<10;t++)sample(l,c,r,10,&s,&o);
 }
 puts("PASS: calibration/polarity, straight noise, PD direction/damping, tight turns, bounded recovery, elapsed time, S-curves, startup junction guard and1331 stateful output combinations");
}
'''
with tempfile.TemporaryDirectory() as temp:
 source=Path(temp)/'test.c';binary=Path(temp)/'test';source.write_text(tests)
 for mask in ('5','7'):
  subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror','-DS3_IR_MASK='+mask,'-I',str(folder),str(source),str(folder/'line_control.c'),'-o',str(binary)],check=True)
  subprocess.run([str(binary)],check=True)

# Confirm the actual gain knobs alter steering and D damps a returning error.
probe=r'''
#include <stdio.h>
#include "line_control.h"
int main(void){LF_STATE s={.initialized=true,.error=600};LF_CMD c;
 line_decide(0,0,0,&s,&c);printf("%d %d\n",c.left,c.right);}
'''
with tempfile.TemporaryDirectory() as temp:
 source=Path(temp)/'probe.c';binary=Path(temp)/'probe';source.write_text(probe)
 values=[]
 for flags in ([],['-DS3_KD_MILLI=0'],['-DS3_KP_MILLI=0','-DS3_KD_MILLI=0']):
  subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror',*flags,'-I',str(folder),str(source),str(folder/'line_control.c'),'-o',str(binary)],check=True)
  values.append(tuple(map(int,subprocess.check_output([str(binary)],text=True).split())))
 assert values[0][0]-values[0][1] < values[1][0]-values[1][1]
 assert values[2][0]==values[2][1]
 print('PASS: actual Kp/Kd overrides change steering; derivative reduces the returning-error differential')
