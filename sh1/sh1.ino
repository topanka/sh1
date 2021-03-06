#include <Servo.h> 

#include <uccbst.h>
#include <sh1tmr.h>
#include <uccbcrc.h>

#include <PID_v1.h>
#include <RunningAverage.h>
#include "DualVNH5019MotorShield.h"

#define SH1_VERSION        "1.0.2"

//battery
#define UCCB_BATTV_PORT              A1
#define UCCB_BATTA_PORT              A0
int g_battV=-1;
int g_battA=-1;

//temperature
#define UCCB_TEMPERATURE_PORT       A4
int g_temperature=-1;
MYTMR g_tmr_temperature={0};

//general
unsigned long g_millis=0;
unsigned long g_loop_cnt=0;         //loop counter
unsigned long g_loop_cps=0;         //loop counter per sec
unsigned long g_loop_ct=0;
int g_recv_ready=0;                 //uccb command received

//UCCB

//battery
int g_cb_battV=-1;

//joy
int g_cb_tsX=-1;
int g_cb_tsY=-1;

int g_cb_fsX=-1;
int g_cb_fsY=-1;
int g_cb_fsZ=-1;
int g_cb_fsBS=-1;
int g_cb_fsBE=-1;

//10p switch
int g_cb_sw10p=-1;

//6p button
int g_cb_b6p=-1;
int g_cb_b6pBS=-1;
int g_cb_b6pBE=-1;

//comm
unsigned long g_cb_w_commpkt_counter=0;

//servo
int g_cb_tsxp=0;
int g_cb_tsyp=0;

int g_cb_rdd=0;

//lights

#define SH1_POSLIGHT_PORT        26
int g_cb_lightpos=UCCB_PL_OFF;
MYTMR g_tmr_lightpos={0};

//motor
int g_cb_m1s=0;
int g_cb_m2s=0;

uint16_t g_rpm_m1=0;
uint16_t g_rpm_m2=0;
int8_t g_dir_m1=0;
int8_t g_dir_m2=0;

MYTMR g_tmr_checkmc={0};

double g_SP,g_iv,g_ov;
/*
double g_Kp=0.3662053;
double g_Ki=2.883506;
double g_Kd=0.005392;
*/

/*
double g_Kp=0.691721;
double g_Ki=5.446623;
//double g_Kd=0.000185;
double g_Kd=0.000085;
*/

/*
double g_Kp=0.833333;
//double g_Ki=6.561654;
double g_Ki=8.561654;
//double g_Kd=0.01227;
double g_Kd=0.03227;
*/


double g_Kpa=0.8;
double g_Kia=10.4;
double g_Kda=0.025;


/*
double g_Kpa=0.4;
double g_Kia=5.0;
double g_Kda=0.008;
*/

double g_Kpc=0.691721;
double g_Kic=5.446623;
double g_Kdc=0.010185;

PID mdPID(&g_iv,&g_ov,&g_SP,0.0,0.0,0.0,DIRECT);

