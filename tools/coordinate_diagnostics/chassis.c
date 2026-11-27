/* PC-only: real chassis CAN receiver and Yaw controller; CAN writes captured. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "board_comm.h"
#include "CAN_receive.h"
#include "yaw_task.h"
#include "robot_modules.h"
CAN_HandleTypeDef hcan1,hcan2;
const RobotModule robot_yaw_module={gimbal_control_Init,gimbal_pid_control};
extern float set_6020pos,set_6020v;
static uint32_t now=1000;
static uint8_t epoch,mode=RC_MODE_C,rc_sequence,payload[8];
static CAN_RxHeaderTypeDef msg;
static int16_t output;
static int16_t ticks;
uint32_t HAL_GetTick(void){return now;}
void osDelay(uint32_t n){now+=n;}
float chassis_get_vw(void){return 0;}
uint8_t chassis_take_spin_recenter(float *p){(void)p;return 0;}
void abs_limit(float *v,float limit){if(*v>limit)*v=limit;if(*v< -limit)*v= -limit;}
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *c){(void)c;return 3;}
int HAL_CAN_GetRxMessage(CAN_HandleTypeDef *c,uint32_t f,CAN_RxHeaderTypeDef *h,uint8_t *d){(void)c;(void)f;*h=msg;memcpy(d,payload,8);return HAL_OK;}
int HAL_CAN_AddTxMessage(CAN_HandleTypeDef *c,CAN_TxHeaderTypeDef *h,uint8_t *d,uint32_t *b){if(c==&hcan1&&h->StdId==0x1ff)output=board_i16(d);*b=1;return HAL_OK;}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *);
static void rx(uint16_t id,const uint8_t *data){memcpy(payload,data,8);msg.StdId=id;msg.IDE=0;msg.RTR=0;msg.DLC=8;HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);}
static void health(void){
    uint8_t d[8]={0};rx(BOARD_RC_CH_ID,d);
    d[2]=mode<<5;d[4]=rc_sequence++;d[5]=1;d[6]=BOARD_PROTOCOL;
    d[7]=(uint8_t)(BOARD_RC_AUTO_CONTRACT|BOARD_RC_IMU_FRESH|epoch);rx(BOARD_RC_EXTRA_ID,d);
    memset(d,0,8);board_put_i16(d,ticks);board_put_i16(d+2,BOARD_AIM_CAN_BUDGET_MS);
    d[6]=BOARD_PROTOCOL;d[7]=epoch;rx(BOARD_IMU_ID,d);
    memset(d,0,8);rx(0x205,d);
}
static void unhex(const char *s,uint8_t d[8]){for(unsigned i=0;i<8;i++){unsigned b;assert(sscanf(s+2*i,"%2x",&b)==1);d[i]=(uint8_t)b;}}
int main(void){
    int tick_input,epoch_input,variant;char a[17],b[17];uint8_t head[8],tail[8];
    assert(scanf("%d %d %d %16s %16s",&tick_input,&epoch_input,&variant,a,b)==5);
    ticks=(int16_t)tick_input;epoch=(uint8_t)epoch_input;unhex(a,head);unhex(b,tail);
    health();gimbal_control_Init();
    if(variant==1){rx(BOARD_YAW_TRAJ_ID,tail);now++;rx(BOARD_YAW_AIM_ID,head);}
    else {rx(BOARD_YAW_AIM_ID,head);now++;
        if(variant!=2){if(variant==3)tail[4]++;rx(BOARD_YAW_TRAJ_ID,tail);}}
    float angle=0,speed=0,accel=0;
    uint8_t valid=BoardAim_GetTrajectory(&angle,&speed,&accel);
    gimbal_pid_control();
    float current=ticks*(360.0f/8192.0f);while(current>=180)current-=360;
    printf("%u,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d",valid,(double)angle,
        (double)speed,(double)accel,(double)current,(double)set_6020pos,(double)set_6020v,output);
    mode=RC_MODE_S;epoch=(uint8_t)((epoch+1)&BOARD_AIM_EPOCH_MASK);health();gimbal_pid_control();
    assert(output==0);printf(",%d\n",output);
}
