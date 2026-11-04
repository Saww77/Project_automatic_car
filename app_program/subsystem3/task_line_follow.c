/*
 * task_line_follow.c: line following (Buddy 3).
 * line_sense()  reads the 3 sensors -> darkness 0..1000
 * line_decide() PURE logic: darkness in, wheel levels out (easy to unit-test)
 * line_follow_task() runs both every LINE_PERIOD_MS and drives Motion Control
 */
#include "subsystem3.h"

/* Calibration: raw ADC on white paper and on the black line (L, C, R).
   Measure on your track; the live check below prints the raw values. */
LOCAL const INT white_raw[3] = { 130, 130, 130 };
LOCAL const INT black_raw[3] = { 3200, 3200, 3200 };

LOCAL INT darkness(INT raw, INT white, INT black)
{
    INT d;
    if (black <= white) { return 0; }
    d = ((raw - white) * 1000) / (black - white);
    if (d < 0)    { d = 0; }
    if (d > 1000) { d = 1000; }
    return d;
}

/* Read all three sensors; masked-off sensors count as white (0). */
EXPORT ER line_sense(INT *l, INT *c, INT *r)
{
    INT raw[3];
    INT i;
    INT dk[3];

    for (i = 0; i < 3; i++) {
        if (S3_IR_MASK & (1 << i)) {
            raw[i] = ir_read(i);
            if (raw[i] < 0) { return E_IO; }     /* ADC error: skip this cycle */
            dk[i] = darkness(raw[i], white_raw[i], black_raw[i]);
        } else {
            dk[i] = 0;
        }
    }
    *l = dk[0]; *c = dk[1]; *r = dk[2];
    return E_OK;
}

/* PURE decision logic. l,c,r = darkness 0..1000. No hardware, no printing. */
/* Fine RPM commands permit slow travel below the old 60 RPM minimum.
 * Two outer sensors cannot distinguish centred white/white from loss.
 * Remember steering across line loss only when the centre sensor is enabled.
 */
EXPORT void line_decide(INT l, INT c, INT r, LF_STATE *st, LF_CMD *cmd)
{
    INT err = r - l;
    INT a = (err < 0) ? -err : err;
    BOOL visible = (l >= LINE_DARK || c >= LINE_DARK || r >= LINE_DARK);

    /* Reject white-surface offsets: a difference alone is not a line.
     * Two outer sensors cannot distinguish centred white/white from loss.
     * Clear remembered steering in that case instead of latching a pivot.
     */
    if (!(S3_IR_MASK & 0x2) && !visible) {
        st->last_pos = 0;
        st->lost_ms = 0;
        cmd->stop = FALSE;
        cmd->left = S3_CRUISE_RPM;
        cmd->right = S3_CRUISE_RPM;
        return;
    }

    cmd->stop = FALSE;
    cmd->left = S3_CRUISE_RPM;
    cmd->right = S3_CRUISE_RPM;

    if (visible) {
        st->lost_ms = 0;
        if (a >= POS_STRAIGHT && (l >= LINE_DARK || r >= LINE_DARK)) {
            st->last_pos = err;
        } else {
            /* Centre seen, or a balanced wide marking: go straight. */
            st->last_pos = 0;
        }
    } else {
        if (st->lost_ms < LOST_TIMEOUT_MS) {
            st->lost_ms += LINE_PERIOD_MS;
        }
        /* A working centre sensor removes the white/white ambiguity.
         * Two-sensor mode permits centred travel between the outer sensors.
         */
        if ((st->last_pos != 0 || (S3_IR_MASK & 0x2)) &&
            st->lost_ms >= LOST_TIMEOUT_MS) {
            cmd->left = LEVEL_STOPPED;
            cmd->right = LEVEL_STOPPED;
            cmd->stop = TRUE;
            return;
        }
        err = st->last_pos;
        a = (err < 0) ? -err : err;
    }

    /* Gentle curves retain forward drive on BOTH wheels. Only a strong
     * outer-only reading (or three-sensor loss recovery) coasts the inner.
     * Commands remain even RPM values for the compact transport.
     */
    if (a >= POS_STRAIGHT) {
        INT inner = S3_INNER_RPM;
        cmd->left = S3_CURVE_RPM;
        cmd->right = S3_CURVE_RPM;
        if (!visible || (a >= 750 && c < LINE_DARK)) {
            inner = LEVEL_STOPPED;
        }
        if (err > 0) { cmd->right = inner; }
        else         { cmd->left = inner; }
    }
}

/* Do not latch the junction test during startup/IDLE: that could reject
 * every subsequent START before the car ever moved. */
EXPORT BOOL line_junction_update(INT l,INT r,BOOL following,INT *elapsed_ms)
{
    if(!following || l<JUNC_DARK || r<JUNC_DARK){*elapsed_ms=0;return FALSE;}
    if(*elapsed_ms<JUNCTION_MIN_MS)*elapsed_ms+=LINE_PERIOD_MS;
    return *elapsed_ms>=JUNCTION_MIN_MS;
}

/* Send a command recommendation to the vehicle controller; never drive S2 here. */
#include "../vehicle_controller/vehicle_controller.h"
LOCAL ID owner_id;
LOCAL void tick_release(void *exinf) { (void)exinf;(void)tk_wup_tsk(owner_id); }
EXPORT void line_follow_task(INT stacd,void *exinf)
{
    LF_STATE st={0};LF_CMD cmd;T_CCYC cyc={0};
    INT l,c,r,junc_ms=0,diag_ms=0;
    BOOL halted=FALSE;
    (void)stacd;(void)exinf;
    owner_id=tk_get_tid();
    cyc.cycatr=TA_HLNG|TA_STA;cyc.cychdr=(FP)tick_release;
    /* The kernel tick is10 ms: request the line period directly. */
    cyc.cyctim=LINE_PERIOD_MS;cyc.cycphs=LINE_PERIOD_MS;
    if(tk_cre_cyc(&cyc)<E_OK){vehicle_invalidate_line();tk_ext_tsk();return;}
    for(;;){
        tk_slp_tsk(TMO_FEVR);(void)tk_can_wup(owner_id);
        if(line_sense(&l,&c,&r)!=E_OK){vehicle_invalidate_line();st=(LF_STATE){0};continue;}
        if(!halted && line_junction_update(l,r,vehicle_is_following(),&junc_ms)){
            halted=TRUE;tm_printf((UB*)"[S3] junction: stop recommendation\n");
        }
        line_decide(l,c,r,&st,&cmd);
        if(halted)cmd=(LF_CMD){0,0,TRUE};
        /* Barcode decoding remains disabled in the existing hardware workflow. */
        vehicle_submit_line(l,c,r,cmd.left,cmd.right,cmd.stop,0);
        diag_ms+=LINE_PERIOD_MS;
        if(diag_ms>=500){
            diag_ms=0;
            tm_printf((UB*)"[S3] dark L=%d C=%d R=%d target RPM L=%d R=%d stop=%d\n",
                      l,c,r,cmd.left,cmd.right,cmd.stop);
            s3_motion_diagnostics();
        }
    }
}
