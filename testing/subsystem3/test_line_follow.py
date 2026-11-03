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
typedef struct { INT last_pos; INT lost_ms; INT turn_ms; } LF_STATE;
typedef struct { INT left; INT right; BOOL stop; } LF_CMD;
'''
tests = '''
static void sample(int l,int c,int r,LF_STATE *s,LF_CMD *cmd) {
 line_decide(l,c,r,s,cmd);
 assert(cmd->left>=0 && cmd->left<=20 && cmd->right>=0 && cmd->right<=20);
 assert(cmd->left%2==0 && cmd->right%2==0);
}
int main(void) {
 LF_STATE s={0}; LF_CMD c;
 sample(0,1000,0,&s,&c);assert(c.left==20 && c.right==20);
 /* Isolated/alternating edge hits do not trigger stop-wheel pivots. */
 for(int i=0;i<40;i++) {
  sample(1000,0,0,&s,&c);assert(c.left>=8 && c.right==20);
  sample(0,0,1000,&s,&c);assert(c.right>=8 && c.left==20);
 }
 /* Corrections increase progressively, not all-or-nothing. */
 s=(LF_STATE){0};sample(650,900,400,&s,&c);int gentle=c.left;
 sample(900,900,200,&s,&c);assert(c.left<gentle && c.left>0);
 /* Moderate turning must not pre-arm an immediate pivot. */
 s=(LF_STATE){0};for(int i=0;i<20;i++)sample(650,0,200,&s,&c);
 sample(1000,0,0,&s,&c);assert(c.left>0);
 /* Sustained tight-loop readings still permit a sharp turn. */
 s=(LF_STATE){0};
 for(int i=0;i<S3_TURN_CONFIRM_MS/LINE_PERIOD_MS-1;i++) {
  sample(1000,0,0,&s,&c);assert(c.left>=8 && c.right==20);
 }
 sample(1000,0,0,&s,&c);assert(c.left==0 && c.right==20);
 /* White must release a stopped wheel immediately, gently and bounded. */
 for(int i=0;i<S3_CURVE_HOLD_MS/LINE_PERIOD_MS;i++) {
  sample(0,0,0,&s,&c);assert(!c.stop && c.left>=12 && c.right==20);
 }
 sample(0,0,0,&s,&c);
 if(!(S3_IR_MASK&2)) assert(c.left==20 && c.right==20 && s.last_pos==0);
 /* A visible centre cancels the correction in either sensor configuration. */
 sample(0,1000,0,&s,&c);assert(c.left==20 && c.right==20);
 s=(LF_STATE){0};
 for(int i=0;i<LOST_TIMEOUT_MS/LINE_PERIOD_MS;i++) sample(0,0,0,&s,&c);
 assert(c.stop==((S3_IR_MASK&2)!=0));
 /* Single edge then white cannot arm a held turn in two-sensor mode. */
 s=(LF_STATE){0};sample(1000,0,0,&s,&c);sample(0,0,0,&s,&c);
 if(!(S3_IR_MASK&2)) assert(c.left==20 && c.right==20);
 s=(LF_STATE){0};sample(350,0,100,&s,&c);assert(c.left==20 && c.right==20);
 for(int l=0;l<=1000;l+=100) for(int m=0;m<=1000;m+=100)
 for(int r=0;r<=1000;r+=100) { s=(LF_STATE){0};sample(l,m,r,&s,&c); }
 puts("PASS: constant slow speed, proportional correction, alternating-edge damping, confirmed sharp pivot, immediate wheel release, bounded soft hold, 1331 output bounds");
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
