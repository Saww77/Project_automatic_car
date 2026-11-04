#ifndef LINE_CONTROL_H
#define LINE_CONTROL_H
#include <stdbool.h>
#include "line_config.h"
typedef enum { LF_TRACK, LF_TIGHT, LF_RECOVER, LF_LOST } LF_MODE;
typedef struct {
    int error,derivative,last_direction,turn_ms,release_ms,lost_ms;
    bool initialized;
    LF_MODE mode;
} LF_STATE;
typedef struct { int left,right;bool stop; } LF_CMD;
int line_normalize(int raw,int white,int black);
void line_decide_timed(int l,int c,int r,unsigned elapsed_ms,LF_STATE *st,LF_CMD *cmd);
void line_decide(int l,int c,int r,LF_STATE *st,LF_CMD *cmd);
bool line_junction_update(int l,int r,bool following,int *elapsed_ms);
#endif
