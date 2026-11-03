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
typedef struct { INT last_pos; INT lost_ms; } LF_STATE;
typedef struct { INT left; INT right; BOOL stop; } LF_CMD;
'''
tests = '''
int main(void) {
 LF_STATE s={0,0}; LF_CMD c;
 line_decide(0,1000,0,&s,&c);
 assert(!c.stop && c.left==1 && c.right==1);
 line_decide(1000,0,0,&s,&c);
 assert(!c.stop && c.left==0 && c.right==1);
 line_decide(0,0,0,&s,&c);
 assert(!c.stop && c.left==((S3_IR_MASK & 2)?0:1) && c.right==1);
 for(int i=1;i<LOST_TIMEOUT_MS/LINE_PERIOD_MS;i++) line_decide(0,0,0,&s,&c);
 assert(c.stop == ((S3_IR_MASK & 2)!=0));
 if(c.stop) assert(c.left==0 && c.right==0);
 line_decide(0,0,1000,&s,&c);
 assert(!c.stop && c.left==1 && c.right==0 && s.lost_ms==0);
 line_decide(0,1000,0,&s,&c);
 assert(!c.stop && c.left==1 && c.right==1 && s.last_pos==0);
 /* White-surface imbalance must never stop one wheel. */
 s=(LF_STATE){0,0}; line_decide(350,0,100,&s,&c);
 assert(!c.stop && c.left==1 && c.right==1);
 line_decide(100,0,350,&s,&c);
 assert(!c.stop && c.left==1 && c.right==1);
 s=(LF_STATE){0,0};
 for(int i=0;i<LOST_TIMEOUT_MS/LINE_PERIOD_MS;i++) line_decide(0,0,0,&s,&c);
 assert(c.stop == ((S3_IR_MASK & 2)!=0));
 for(int l=0;l<=1000;l+=100) for(int m=0;m<=1000;m+=100)
 for(int r=0;r<=1000;r+=100) {
  s=(LF_STATE){0,0}; line_decide(l,m,r,&s,&c);
  assert(c.left>=0 && c.left<=1 && c.right>=0 && c.right<=1);
 }
 puts("PASS: straight, both curves, recovery, timeout, reacquisition, sensor modes, 1331 bounded outputs");
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
