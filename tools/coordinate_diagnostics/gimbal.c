/* PC-only: real gimbal parser, coordinate conversion, 5A and CAN serializers.
 * RC, sensor samples, peer health and time are fixtures, not hardware evidence. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "autoaim.h"
#include "link.h"
#include "INS_task.h"
#include "planned_trajectory.h"
#include "board_comm.h"
#include "usbd_cdc_if.h"
#include "actual_imu_mapping.h"
USBD_HandleTypeDef hUsbDeviceFS;
static uint32_t now=1000;
static VT13_RC_ctrl_t rc;
static uint8_t feedback[16];
uint32_t HAL_GetTick(void){return now;}
uint8_t VT13_IsOnline(uint32_t n,uint32_t t){(void)n;(void)t;return 1;}
const VT13_RC_ctrl_t *get_VT13_remote_control_point(void){return &rc;}
uint8_t BoardFeedback_Copy(uint8_t d[8]){memset(d,0,8);d[0]=1;d[3]=RC_MODE_C;d[7]=16;return 1;}
void MX_USB_DEVICE_Init(void){hUsbDeviceFS.dev_state=USBD_STATE_CONFIGURED;hUsbDeviceFS.pClassData=feedback;}
uint8_t CDC_Transmit_FS(uint8_t *p,uint16_t n){assert(n==16);memcpy(feedback,p,16);return USBD_OK;}
static void hex(const uint8_t *p,unsigned n){for(unsigned i=0;i<n;i++)printf("%02x",p[i]);}
int main(void)
{
    float raw,world,mechanical;char bytes[77];uint8_t frame[38],head[8]={0},tail[8]={0};
    assert(scanf("%f %f %f %76s",&raw,&world,&mechanical,bytes)==4);
    for(unsigned i=0;i<38;i++){unsigned b;assert(sscanf(bytes+2*i,"%2x",&b)==1);frame[i]=(uint8_t)b;}
    rc.rc.mode_sw=RC_MODE_C;AutoAim_Init();
    float angles[3]={0};angles[INS_YAW_ADDRESS_OFFSET]=raw;angles[INS_PITCH_ADDRESS_OFFSET]=world;
    GimbalImu_Publish(angles);AutoAim_Update(now,&rc,1);
    /* Respect the production 10ms feedback period after AutoAim_Init. */
    now+=10;GimbalImu_Publish(angles);AutoAim_Update(now,&rc,1);
    PlannedTrajectory_UpdatePitchAlignment(now,mechanical,world,1);
    Aimbot_FeedUsb(frame,7);Aimbot_FeedUsb(frame+7,31);AutoAim_Update(now,&rc,1);
    planned_trajectory_t plan={0};planned_trajectory_debug_t d;
    uint8_t executable=PlannedTrajectory_CopyExecutable(&plan,now);
    PlannedTrajectory_CopyDebug(&d);
    uint8_t has_head=AutoAim_FillYawCan(head),has_tail=AutoAim_FillYawTrajectoryCan(tail);
    assert(feedback[0]==0x5a&&Aimbot_Crc16Verify(feedback,16));
    printf("%u,%u,%u,%u,%d,%u,",d.reason,executable,has_head,has_tail,
        actual_imu_ticks(raw),AutoAim_RemoteStatus(now)&BOARD_AIM_EPOCH_MASK);
    hex(feedback,16);putchar(',');hex(head,8);putchar(',');hex(tail,8);
    printf(",%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",(double)d.mapped_pitch,
        (double)plan.yaw,(double)plan.pitch,(double)plan.yaw_speed,
        (double)plan.pitch_speed,(double)plan.yaw_acc,(double)plan.pitch_acc);
}
