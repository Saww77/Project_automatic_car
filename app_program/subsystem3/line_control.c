#include "line_control.h"
static int clamp(int x,int lo,int hi){return x<lo?lo:x>hi?hi:x;}
static int magnitude(int x){return x<0?-x:x;}
static int rpm(int value){value=clamp(value,S3_MIN_ROLLING_RPM,S3_MAX_RPM);return ((value+1)/2)*2;}
int line_normalize(int raw,int white,int black)
{
    int span=black-white;
    if(raw<0||raw>4095||white<0||white>4095||black<0||black>4095||magnitude(span)<S3_MIN_CAL_SPAN)return -1;
    return clamp((raw-white)*1000/span,0,1000);
}
void line_decide_timed(int l,int c,int r,unsigned elapsed_ms,LF_STATE *s,LF_CMD *out)
{
    int dt=(int)(elapsed_ms>1000u?1000u:elapsed_ms);
    if(dt<1)dt=1;
    int pd_dt=clamp(dt,1,50);
    if(dt>50){s->turn_ms=0;s->initialized=false;}
    l=clamp(l,0,1000);c=clamp(c,0,1000);r=clamp(r,0,1000);
    int raw=r-l,a=magnitude(raw),direction=raw>0?1:raw<0?-1:0;
    bool present=l>=S3_PRESENT_DARK || r>=S3_PRESENT_DARK ||
                 ((S3_IR_MASK&2) && c>=S3_PRESENT_DARK);
    *out=(LF_CMD){S3_CRUISE_RPM,S3_CRUISE_RPM,false};
    if(!present){
        if(s->mode==LF_TIGHT || s->mode==LF_RECOVER){
            s->mode=LF_RECOVER;s->lost_ms=clamp(s->lost_ms+dt,0,S3_RECOVERY_MS);
            if(s->lost_ms>=S3_RECOVERY_MS){s->mode=LF_LOST;*out=(LF_CMD){0,0,true};return;}
            out->left=s->last_direction<0?0:S3_TIGHT_OUTER_RPM;
            out->right=s->last_direction>0?0:S3_TIGHT_OUTER_RPM;
            return;
        }
        if(s->mode==LF_LOST){*out=(LF_CMD){0,0,true};return;}
        if(S3_IR_MASK&2){
            s->lost_ms=clamp(s->lost_ms+dt,0,S3_THREE_SENSOR_LOST_MS);
            if(s->lost_ms>=S3_THREE_SENSOR_LOST_MS){s->mode=LF_LOST;*out=(LF_CMD){0,0,true};return;}
        }
        /* Two-white TRACK readings may be centred: decay PD, don't invent a search. */
        raw=0;a=0;s->turn_ms=0;
    }else{
        if(s->mode==LF_RECOVER || s->mode==LF_LOST){
            s->mode=LF_TRACK;s->initialized=false;s->turn_ms=0;
        }
        s->lost_ms=0;
        bool strong=a>=S3_TIGHT_ENTER_ERROR && c<S3_PRESENT_DARK;
        if(s->mode==LF_TIGHT){
            if(direction && direction!=s->last_direction && a>S3_TIGHT_EXIT_ERROR){
                s->mode=LF_TRACK;s->turn_ms=0;s->release_ms=0;s->initialized=false;
            }else if(a<=S3_TIGHT_EXIT_ERROR){
                s->release_ms+=dt;
                if(s->release_ms>=S3_TIGHT_RELEASE_MS){s->mode=LF_TRACK;s->turn_ms=0;s->release_ms=0;}
            }else s->release_ms=0;
        }
        if(s->mode==LF_TRACK){
            if(strong){
                if(direction==s->last_direction)s->turn_ms=clamp(s->turn_ms+dt,0,S3_TIGHT_CONFIRM_MS);
                else s->turn_ms=LINE_PERIOD_MS;
                s->last_direction=direction;
                if(s->turn_ms>=S3_TIGHT_CONFIRM_MS){s->mode=LF_TIGHT;s->release_ms=0;}
            }else s->turn_ms=0;
        }
    }
    if(s->mode==LF_TIGHT){
        out->left=s->last_direction<0?0:S3_TIGHT_OUTER_RPM;
        out->right=s->last_direction>0?0:S3_TIGHT_OUTER_RPM;
        return;
    }
    /* Filter error and time-normalised derivative; suppress derivative kick at start/reacquisition. */
    if(!s->initialized){s->error=raw;s->derivative=0;s->initialized=true;}
    else{
        int error=(s->error+raw)/2;
        int change=clamp((error-s->error)*LINE_PERIOD_MS/pd_dt,-2000,2000);
        s->derivative=(2*s->derivative+change)/3;
        s->error=error;
    }
    int error=s->error;
    if(magnitude(error)<=S3_ERROR_DEADBAND){s->error=0;s->derivative=0;error=0;}
    int correction=(S3_KP_MILLI*error+S3_KD_MILLI*s->derivative)/1000;
    /* D damps a correction; it must not create opposite steering from a fading same-side signal. */
    if(error>0)correction=clamp(correction,0,S3_MAX_RPM);
    else if(error<0)correction=clamp(correction,-S3_MAX_RPM,0);
    else correction=0;
    int base=S3_CRUISE_RPM-magnitude(error)*S3_CURVE_BASE_CUT/1000;
    out->left=rpm(base+correction);out->right=rpm(base-correction);
}
void line_decide(int l,int c,int r,LF_STATE *s,LF_CMD *out)
{line_decide_timed(l,c,r,LINE_PERIOD_MS,s,out);}
bool line_junction_update(int l,int r,bool following,int *elapsed)
{
    if(!following||l<JUNC_DARK||r<JUNC_DARK){*elapsed=0;return false;}
    if(*elapsed<JUNCTION_MIN_MS)*elapsed+=LINE_PERIOD_MS;
    return *elapsed>=JUNCTION_MIN_MS;
}
