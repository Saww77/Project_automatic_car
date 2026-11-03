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
 assert(cmd->left>=0 && cmd->left<=S3_CRUISE_RPM);
 assert(cmd->right>=0 && cmd->right<=S3_CRUISE_RPM);
 assert(cmd->left%2==0 && cmd->right%2==0);
}
int main(void) {
 LF_STATE s={0}; LF_CMD c;
 sample(0,1000,0,&s,&c);
 assert(!c.stop && c.left==30 && c.right==30);
 sample(650,900,300,&s,&c);
 assert(!c.stop && c.left==10 && c.right==20);
 sample(300,900,650,&s,&c);
 assert(!c.stop && c.left==20 && c.right==10);
 /* Tight-turn response before the previous 750-point threshold. */
 s=(LF_STATE){0}; sample(650,0,100,&s,&c);
 assert(c.left==0 && c.right==20);
 sample(650,0,100,&s,&c); assert(s.turn_ms>=S3_TURN_CONFIRM_MS);
 sample(0,0,0,&s,&c);
 assert(!c.stop && c.left==0 && c.right==20);
 for(int t=LINE_PERIOD_MS;t<S3_PIVOT_HOLD_MS;t+=LINE_PERIOD_MS)
  sample(0,0,0,&s,&c);
 sample(0,0,0,&s,&c);
 if(!(S3_IR_MASK&2)) assert(c.left==10 && c.right==20);
 for(int t=S3_PIVOT_HOLD_MS+LINE_PERIOD_MS;t<S3_CURVE_HOLD_MS;t+=LINE_PERIOD_MS)
  sample(0,0,0,&s,&c);
 sample(0,0,0,&s,&c);
 if(!(S3_IR_MASK&2)) assert(c.left==30 && c.right==30 && s.last_pos==0);
 for(int i=0;i<LOST_TIMEOUT_MS/LINE_PERIOD_MS;i++) sample(0,0,0,&s,&c);
 assert(c.stop==((S3_IR_MASK&2)!=0));
 /* Right-hand loop and repeated edge/gap readings preserve turn direction. */
 s=(LF_STATE){0};
 for(int i=0;i<30;i++) {
  sample(100,0,650,&s,&c); sample(100,0,650,&s,&c);
  sample(0,0,0,&s,&c);
  assert(!c.stop && c.left==20 && c.right==0);
 }
 /* A visible centred line exits the turn immediately. */
 sample(0,1000,0,&s,&c);
 assert(!c.stop && c.left==30 && c.right==30 && s.last_pos==0);
 /* An isolated dark sample must not latch a two-sensor turn. */
 s=(LF_STATE){0}; sample(1000,0,0,&s,&c); sample(0,0,0,&s,&c);
 if(!(S3_IR_MASK&2)) assert(c.left==30 && c.right==30);
 /* Both-white startup and modest white-surface imbalance stay straight. */
 s=(LF_STATE){0}; sample(350,0,100,&s,&c);
 assert(!c.stop && c.left==30 && c.right==30);
 sample(100,0,350,&s,&c);
 assert(!c.stop && c.left==30 && c.right==30);
 /* Reversal of curve direction resets confirmation. */
 s=(LF_STATE){0};sample(1000,0,0,&s,&c);sample(1000,0,0,&s,&c);
 sample(0,0,1000,&s,&c);assert(s.turn_ms==LINE_PERIOD_MS);
 for(int l=0;l<=1000;l+=100) for(int m=0;m<=1000;m+=100)
 for(int r=0;r<=1000;r+=100) {
  s=(LF_STATE){0}; sample(l,m,r,&s,&c);
 }
 puts("PASS: slow straight, earlier tight turns, confirmed bounded hold, soft release, repeated loop gaps, no latched wheel, 1331 output bounds");
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
