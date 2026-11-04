/*
 * task_line_follow.c: line following (Buddy 3).
 * line_sense()  reads the 3 sensors -> darkness 0..1000
 * line_decide() PURE logic: darkness in, wheel levels out (easy to unit-test)
 * line_follow_task() runs both every LINE_PERIOD_MS and drives Motion Control
 */
#include "subsystem3.h"

LOCAL const INT white_raw[3]={S3_WHITE_LEFT,S3_WHITE_CENTRE,S3_WHITE_RIGHT};
LOCAL const INT black_raw[3]={S3_BLACK_LEFT,S3_BLACK_CENTRE,S3_BLACK_RIGHT};
LOCAL INT last_raw[3];
LOCAL ER read_raw(void)
{
    for(INT i=0;i<3;i++){
        last_raw[i]=(S3_IR_MASK&(1<<i))?ir_read(i):0;
        if(last_raw[i]<0)return E_IO;
    }
    return E_OK;
}
EXPORT ER line_sense(INT *l,INT *c,INT *r)
{
    INT dark[3];
    if(read_raw()!=E_OK)return E_IO;
    for(INT i=0;i<3;i++){
        dark[i]=(S3_IR_MASK&(1<<i))?line_normalize(last_raw[i],white_raw[i],black_raw[i]):0;
        if(dark[i]<0)return E_IO;
    }
    *l=dark[0];*c=dark[1];*r=dark[2];return E_OK;
}

/* Send a command recommendation to the vehicle controller; never drive S2 here. */
#include "../vehicle_controller/vehicle_controller.h"
LOCAL ID owner_id;
LOCAL void tick_release(void *exinf) { (void)exinf;(void)tk_wup_tsk(owner_id); }
EXPORT void line_follow_task(INT stacd,void *exinf)
{
    LF_STATE st={0};LF_CMD cmd;T_CCYC cyc={0};
    INT l,c,r,junc_ms=0,diag_ms=0;
    UW previous_ms=0;
    BOOL had_time=FALSE;
    BOOL halted=FALSE,last_stop=FALSE;
    LF_MODE last_mode=LF_TRACK;
    (void)stacd;(void)exinf;
#if APP_LINE_CALIBRATE
    vehicle_request_stop();
    for(;;){
        vehicle_invalidate_line();
        if(read_raw()==E_OK)tm_printf((UB*)"[CALIBRATION] raw L=%d C=%d R=%d mask=0x%x MOTORS DISABLED\n",last_raw[0],last_raw[1],last_raw[2],S3_IR_MASK);
        else tm_printf((UB*)"[CALIBRATION] ADC read failed; check AO pins. MOTORS DISABLED\n");
        tk_dly_tsk(250);
    }
#endif
    owner_id=tk_get_tid();
    cyc.cycatr=TA_HLNG|TA_STA;cyc.cychdr=(FP)tick_release;
    /* The kernel tick is10 ms: request the line period directly. */
    cyc.cyctim=LINE_PERIOD_MS;cyc.cycphs=LINE_PERIOD_MS;
    if(tk_cre_cyc(&cyc)<E_OK){vehicle_invalidate_line();tk_ext_tsk();return;}
    for(;;){
        tk_slp_tsk(TMO_FEVR);(void)tk_can_wup(owner_id);
        if(line_sense(&l,&c,&r)!=E_OK){vehicle_invalidate_line();st=(LF_STATE){0};continue;}
        BOOL following=vehicle_is_following();
        if(!following)st=(LF_STATE){0};
        SYSTIM time;tk_get_tim(&time);
        UW elapsed=had_time?time.lo-previous_ms:LINE_PERIOD_MS;
        previous_ms=time.lo;had_time=TRUE;
        if(!halted && line_junction_update(l,r,following,&junc_ms)){
            halted=TRUE;tm_printf((UB*)"[S3] junction: stop recommendation\n");
        }
        line_decide_timed(l,c,r,elapsed,&st,&cmd);
        if(halted)cmd=(LF_CMD){0,0,TRUE};
        /* Barcode decoding remains disabled in the existing hardware workflow. */
        vehicle_submit_line(l,c,r,cmd.left,cmd.right,cmd.stop,0);
        diag_ms+=LINE_PERIOD_MS;
        if(diag_ms>=250 || st.mode!=last_mode || cmd.stop!=last_stop){
            last_mode=st.mode;last_stop=cmd.stop;
            diag_ms=0;
            tm_printf((UB*)"[LINE_PD] mode=%d raw=%d/%d/%d dark=%d/%d/%d error=%d D=%d dt=%u target=%d/%d stop=%d\n",
                      st.mode,last_raw[0],last_raw[1],last_raw[2],l,c,r,st.error,st.derivative,
                      (unsigned)elapsed,cmd.left,cmd.right,cmd.stop);
            s3_motion_diagnostics();
        }
    }
}
