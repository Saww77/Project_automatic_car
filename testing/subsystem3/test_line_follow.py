"""Host regression tests compile the actual pure controller from the firmware."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
src = (root / 'app_program/subsystem3/task_line_follow.c').read_text()
header = (root / 'app_program/subsystem3/subsystem3.h').read_text()
constants = '\n'.join(line for line in header.splitlines() if line.startswith('#define ')
                      and len(line.split()) >= 3)
logic = src[src.index('EXPORT void line_decide'):src.index('/* Send a command')]
prelude = '''#include <assert.h>
#include <stdio.h>
typedef int INT;
typedef int BOOL;
#define EXPORT
#define TRUE 1
#define FALSE 0
typedef struct { INT last_pos; INT lost_ms; INT turn_ms; INT steer; BOOL seen; } LF_STATE;
typedef struct { INT left; INT right; BOOL stop; } LF_CMD;
'''
tests = '''
static void sample(int l,int c,int r,LF_STATE *s,LF_CMD *cmd) {
 int old=s->steer;
 line_decide(l,c,r,s,cmd);
 assert(cmd->left>=0 && cmd->left<=20 && cmd->right>=0 && cmd->right<=20);
 assert(cmd->left%2==0 && cmd->right%2==0);
 if(!cmd->stop) assert(s->steer-old<=S3_RELEASE_STEP && old-s->steer<=S3_RELEASE_STEP);
}
int main(void) {
 LF_STATE s={0}; LF_CMD c;
 sample(0,0,0,&s,&c);assert(c.left==20 && c.right==20);
 /* Alternating noise cannot repeatedly stop a wheel. */
 for(int i=0;i<50;i++) {
  sample(1000,0,0,&s,&c);assert(c.left>=18 && c.right==20);
  sample(0,0,1000,&s,&c);assert(c.left==20 && c.right==20);
 }
 /* A sustained tight left curve reaches a bounded inner-wheel stop. */
 s=(LF_STATE){0};
 for(int i=0;i<10;i++)sample(900,0,0,&s,&c);
 assert(c.left==0 && c.right==20);
 /* White/white progressively releases it, completely within 50 ms. */
 sample(0,0,0,&s,&c);assert(c.left==4 && c.right==20);
 for(int i=1;i<5;i++)sample(0,0,0,&s,&c);
 assert(c.left==20 && c.right==20 && s.steer==0);
 /* Sustained right curve, then S-curve reversal, through straight. */
 for(int i=0;i<10;i++)sample(0,0,900,&s,&c);
 assert(c.left==20 && c.right==0);
 for(int i=0;i<10;i++)sample(900,0,0,&s,&c);
 assert(s.steer==0 && c.left==20 && c.right==20);
 for(int i=0;i<10;i++)sample(900,0,0,&s,&c);
 assert(c.left==0 && c.right==20);
 /* Hysteresis retains an existing signal at 500, but not a new one. */
 s=(LF_STATE){0};sample(500,0,0,&s,&c);assert(!s.seen);
 sample(650,0,0,&s,&c);assert(s.seen);
 sample(500,0,0,&s,&c);assert(s.seen);
 sample(350,0,0,&s,&c);assert(!s.seen);
 /* A gentle bend never stops the inner wheel. */
 s=(LF_STATE){0};for(int i=0;i<30;i++)sample(650,0,300,&s,&c);
 assert(c.left>0 && c.left<c.right);
 for(int i=0;i<5;i++)sample(0,1000,0,&s,&c);
 assert(c.left==20 && c.right==20);
 s=(LF_STATE){0};
 for(int i=0;i<LOST_TIMEOUT_MS/LINE_PERIOD_MS;i++)sample(0,0,0,&s,&c);
 assert(c.stop==((S3_IR_MASK&2)!=0));
 for(int l=0;l<=1000;l+=100)for(int m=0;m<=1000;m+=100)
 for(int r=0;r<=1000;r+=100){s=(LF_STATE){0};sample(l,m,r,&s,&c);}
 puts("PASS: left/right sustained curves, S-curve reversal, bounded wheel changes, hysteresis, alternating noise, gradual release, timeout, 1331 bounds");
}
'''
with tempfile.TemporaryDirectory() as folder:
    for mask in ('0x5', '0x7'):
        config = '\n'.join('#define S3_IR_MASK '+mask if line.startswith('#define S3_IR_MASK ') else line for line in constants.splitlines())
        source = Path(folder)/'test.c'
        binary = Path(folder)/'test'
        source.write_text(prelude+config+'\n'+logic+tests)
        subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror',str(source),'-o',str(binary)],check=True)
        subprocess.run([str(binary)],check=True)
