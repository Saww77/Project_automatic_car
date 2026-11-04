#include "s1_comm.h"
#include "../subsystem2/subsystem2.h"
#include "../vehicle_controller/vehicle_controller.h"
#include "../../lib/liblwip/sysdepend/pico_rp2040/mqtt_bridge.h"
ER s1_publish_motion(void)
{
#if TM_MQTT
    S2_STATUS s;
    if(!motion_get_status(&s))return E_OBJ;
    return (ER)mqtt_bridge_send(MQTT_ROUTE_MOTION,s.bytes,sizeof(s.bytes));
#else
    return E_NOSPT;
#endif
}
#if TM_MQTT
static void account(ER result)
{ if(result!=E_OK)vehicle_record_telemetry_drop(); }
#endif
void s1_publish_telemetry(void)
{
#if TM_MQTT
    VEHICLE_SNAPSHOT s;
    if((s1_connection_status()&3u)!=3u || !vehicle_get_snapshot(&s))return;
    unsigned faults=s.motor_faults?S1_FAULT_MOTOR:0;
    if(s.motor_faults&1u)faults|=S1_FAULT_ENCODER;
    if(s.reason==VC_REASON_LINK)faults|=S1_FAULT_WIFI;
    account(s1_publish_vehicle(s.state,faults));
    /* No fabricated zero measurements during startup or absent subsystems. */
    if(motion_ready())account(s1_publish_motion());
    if(s.line_valid && (uint32_t)(s.timestamp_ms-s.line_timestamp_ms)<=VC_LINE_MAX_AGE_MS){
        unsigned ir=(s.l>=600?1u:0u)|(s.c>=600?2u:0u)|(s.r>=600?4u:0u);
        account(s1_publish_line(ir,s.barcode));
    }
    if(s.terrain_valid && (uint32_t)(s.timestamp_ms-s.terrain_timestamp_ms)<=VC_SENSOR_MAX_AGE_MS)
        account(s1_publish_terrain(s.height_mm,s.terrain_phase));
    if(s.obstacle_valid && (uint32_t)(s.timestamp_ms-s.obstacle_timestamp_ms)<=VC_SENSOR_MAX_AGE_MS)
        account(s1_publish_obstacle(s.near_mm,s.width_mm,s.clear_left_mm,s.clear_right_mm));
#endif
}
