#include "subsystem3.h"
#include "../subsystem2/subsystem2.h"

/* Called by the line task, never from the time-critical motor owner. */
EXPORT void s3_motion_diagnostics(void)
{
    S2_STATUS status;
    if (!motion_get_status(&status)) { return; }
    tm_printf((UB*)"[MOTION] actual rpm L=%u R=%u PWM L=%u%% R=%u%% faults=0x%x\n",
              s2_get_bits(&status, 9, 9), s2_get_bits(&status, 18, 9),
              s2_get_bits(&status, 27, 7), s2_get_bits(&status, 34, 7),
              s2_get_bits(&status, 5, 4));
}
