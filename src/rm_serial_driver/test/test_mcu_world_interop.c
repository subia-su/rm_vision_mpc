/* Host-only harness: links the actual infantry2.1 USB parser and planner C files. */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include "main.h"
#include "vision.h"
#include "planned_trajectory.h"
static uint32_t now=1000;
static unsigned checks;
#define CHECK(x) do{checks++;assert(x);}while(0)
uint32_t HAL_GetTick(void){return now;}
int main(int argc,char **argv)
{
    CHECK(argc==2);FILE *f=fopen(argv[1],"rb");CHECK(f!=NULL);
    PlannedTrajectory_Reset();PlannedTrajectory_SetExecutionAllowed(1u);
    const float world[]={0.268172f,0.266214f,0.257770f,0.246818f,0.226271f,
        0.209574f,0.192580f,0.173229f,0.132287f,0.106861f};
    uint8_t frame[38];planned_trajectory_t plan;planned_trajectory_debug_t d;
    for(unsigned i=0;i<15;i++){
        CHECK(fread(frame,1,38,f)==38);now++;
        PlannedTrajectory_UpdatePitchAlignment(now,i>=12?-0.28f:-0.15f,0.14f,1u);
        /* Real stream must handle split USB deliveries of the actual C++ bytes. */
        Aimbot_FeedUsb(frame,7);Aimbot_FeedUsb(frame+7,31);
        PlannedTrajectory_CopyDebug(&d);
        if(i<10){
            CHECK(PlannedTrajectory_CopyExecutable(&plan,now));
            CHECK(fabsf(plan.pitch-(-0.01f-world[i]))<0.000002f);
            CHECK(plan.pitch>=-0.39f&&plan.pitch<=0.07f&&plan.sequence==i+1);
            CHECK(d.reason==PLAN_RX_ACCEPTED);
        }else if(i==10){
            CHECK(!PlannedTrajectory_CopyExecutable(&plan,now));CHECK(d.reason==PLAN_RX_HOST_INVALID);
        }else if(i==11){
            CHECK(!PlannedTrajectory_CopyExecutable(&plan,now));CHECK(d.sequence==11); /* CRC failure never reaches planner. */
        }else if(i==12){
            CHECK(!PlannedTrajectory_CopyExecutable(&plan,now));CHECK(d.reason==PLAN_RX_MECHANICAL_PITCH);
        }else if(i==13){
            CHECK(PlannedTrajectory_CopyExecutable(&plan,now));CHECK(fabsf(plan.pitch+0.246861f)<0.000002f);
        }else{
            CHECK(!PlannedTrajectory_CopyExecutable(&plan,now));CHECK(d.reason==PLAN_RX_HOST_INVALID);
        }
    }
    CHECK(fgetc(f)==EOF);fclose(f);
    printf("Actual C++ A6 -> actual MCU USB/world-Pitch parser: %u checks PASS\n",checks);
}
