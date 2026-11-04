#ifndef LINE_CONFIG_H
#define LINE_CONFIG_H
/* Measure at the mounted height on YOUR white mat and black tape. */
#ifndef S3_WHITE_LEFT
#define S3_WHITE_LEFT 130
#endif
#ifndef S3_BLACK_LEFT
#define S3_BLACK_LEFT 3200
#endif
#ifndef S3_WHITE_CENTRE
#define S3_WHITE_CENTRE 130
#endif
#ifndef S3_BLACK_CENTRE
#define S3_BLACK_CENTRE 3200
#endif
#ifndef S3_WHITE_RIGHT
#define S3_WHITE_RIGHT 130
#endif
#ifndef S3_BLACK_RIGHT
#define S3_BLACK_RIGHT 3200
#endif
#ifndef S3_IR_MASK
#define S3_IR_MASK 0x5 /* GP26 left, GP28 right. Separate barcode sensor is not centre. */
#endif
#ifndef S3_KP_MILLI
#define S3_KP_MILLI 18 /* differential RPM per1000 darkness error */
#endif
#ifndef S3_KD_MILLI
#define S3_KD_MILLI 12 /* differential RPM per1000 error-change per nominal10 ms */
#endif
#if S3_KP_MILLI < 0 || S3_KP_MILLI > 200 || S3_KD_MILLI < 0 || S3_KD_MILLI > 200
#error Steering gains must be integer milli gains in0..200
#endif
/* No steering integral. Wheel-speed PID remains in Buddy2. */
#define LINE_PERIOD_MS 10
#define JUNC_DARK 600
#define JUNCTION_MIN_MS 30
#define S3_MIN_CAL_SPAN 200
#define S3_PRESENT_DARK 150
#define S3_ERROR_DEADBAND 80
#define S3_CRUISE_RPM 24
#define S3_MAX_RPM 30
#define S3_MIN_ROLLING_RPM 12 /* avoid tiny unattainable TRACK wheel targets */
#define S3_CURVE_BASE_CUT 8
#define S3_TIGHT_OUTER_RPM 20
#define S3_TIGHT_ENTER_ERROR 650
#define S3_TIGHT_EXIT_ERROR 250
#define S3_TIGHT_CONFIRM_MS 60
#define S3_TIGHT_RELEASE_MS 20
#define S3_RECOVERY_MS 180
#define S3_THREE_SENSOR_LOST_MS 600
#ifndef APP_LINE_CALIBRATE
#define APP_LINE_CALIBRATE 0
#endif
#endif
