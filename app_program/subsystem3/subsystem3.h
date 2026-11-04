/*
 *	subsystem3.h
 *	Subsystem 3 -- Consumer (statistics).  Student ID: <student ID>.
 *
 *	Public interface of subsystem 3.  Other subsystems include it as
 *	"../subsystem3/subsystem3.h".
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"

#define S3_PRI_CONSUMER		16	/* priority band 16-17 (TEAM.md) */
#define S3_BATCH_SIZE		5	/* report to subsystem 4 every N samples */

/* ---- Line following: tunables ---- */
#define S3_PRI_LINE       17
#include "line_control.h"

// Barcode
#define S3_TICK_MS       1      /* barcode sampling tick */
#define LINE_EVERY_N     10     /* line-follow runs every 10th tick = 10 ms */
#define BC_ELEMENTS      29
#define S3_ADVANCE_MM    60     /* centre of wheels onto the junction: MEASURE sensor-to-axle distance */
#define S3_TURN_90       2      /* Buddy 2: steps of 45 degrees, so 2 = 90, 4 = 180 */

IMPORT void barcode_reset(void);
IMPORT void barcode_feed(INT dark);
IMPORT char barcode_take(void);
IMPORT BOOL barcode_busy(void);

IMPORT INT  ir_read(INT ch);
IMPORT ER   line_sense(INT *l, INT *c, INT *r);
IMPORT void line_follow_task(INT stacd, void *exinf);

/* Motion Control calls go through motion_adapter.c (prints now, real later) */
IMPORT void s3_motion_diagnostics(void);

/* IF-01: send SAMPLE_MSG (subsystem2.h) here with tk_snd_mbx */
IMPORT ID	mbx_samples;

/* IF-04: statistics.  Read them ONLY while holding mtx_stats:
 *	tk_loc_mtx(mtx_stats, TMO_FEVR);  copy = app_stats;  tk_unl_mtx(mtx_stats);
 */
typedef struct {
	UW	received;	/* samples received so far */
	INT	min;
	INT	max;
	W	sum;
} APP_STATS;

IMPORT ID		mtx_stats;
IMPORT APP_STATS	app_stats;

/* Private to subsystem 3 */
IMPORT void	consumer_task(INT stacd, void *exinf);

#endif /* SUBSYSTEM3_H */
