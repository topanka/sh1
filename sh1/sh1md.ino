DualVNH5019MotorShield md;

#define SH1_MOTORCURRENT_MIN      300

RunningAverage m1cra(7);
RunningAverage m2cra=(7);

void md_setup(void)
{
  md.init();
  
  g_SP=97;
  mdPID.SetMode(AUTOMATIC);
  mdPID.SetSampleTime(10);
  mdPID.SetOutputLimits(0,400);
  tmr_init(&g_tmr_checkmc,10);
  
}

void md_go(void)
{
   md.setM1Speed(80);
   md.setM2Speed(80);
}

void md_stop(void)
{
   md.setM1Brake(200);
   md.setM2Brake(200);
}

int md_getmc(unsigned int *m1c, unsigned int *m2c)
{
  unsigned int mc;
  
  if(m1c != NULL) {
    mc=md.getM1CurrentMilliamps();
    m1cra.addValue(mc);
    *m1c=m1cra.getAverage();
    if(*m1c < SH1_MOTORCURRENT_MIN) {
      *m1c=0;
    }
  }
  if(m2c != NULL) {
    mc=md.getM2CurrentMilliamps();
    m2cra.addValue(mc);
    *m2c=m2cra.getAverage();
    if(*m2c < SH1_MOTORCURRENT_MIN) {
      *m2c=0;
    }
  }
  return(0);
}

int md_checkmc(void)
{
  if(tmr_do(&g_tmr_checkmc) != 1) return(0);
  m1cra.addValue(md.getM1CurrentMilliamps());
  m2cra.addValue(md.getM2CurrentMilliamps());
  return(1);
}

int md_setspeed(void)
{

   if(g_recv_ready != 1) return(0);
  
//g_cb_m1s=48;  
//g_cb_m1s=0;  
//g_cb_m2s=45;  
//g_cb_m2s=0;  
  
/*  
Serial.print(g_cb_m1s);
Serial.print(" ");
Serial.println(g_cb_m2s);
*/

   md.setM1Speed(g_cb_m1s);
   md.setM2Speed(g_cb_m2s);
}

#define SP1   60
#define SP2   120

void md_pidtest(void)
{
  static unsigned long t0=0;
  static uint8_t f=0;
  unsigned long mm;
  uint16_t rpm1;
  int sp1,sp2;
  
  if(t0 == 0) {
    md.setM1Speed(SP1);
    mm=micros();
    rpm1=qe_rpm1_tbi();
    Serial.print(SP1,DEC);
    Serial.print(" ");
    Serial.print(mm,DEC);
    Serial.print(" ");
    Serial.println(rpm1);
    t0=g_millis;
  }
  if(g_millis < t0+2000) {
    mm=micros();
    rpm1=qe_rpm1_tbi();
    Serial.print(SP1,DEC);
    Serial.print(" ");
    Serial.print(mm,DEC);
    Serial.print(" ");
    Serial.println(rpm1);
  } else if(g_millis < t0+4000) {
    if(f == 0) {
      md.setM1Speed(SP2);
      f=1;
    }
    mm=micros();
    rpm1=qe_rpm1_tbi();
    Serial.print(SP2,DEC);
    Serial.print(" ");
    Serial.print(mm,DEC);
    Serial.print(" ");
    Serial.println(rpm1);
 
  } else {
     md.setM1Speed(0);
     while(1) {};
  }

}

void md_pidtest2(void)
{
  static unsigned long t0=0;
  unsigned long mm;
  double gap;
  static double prev_iv=0.0;
  char a;
  
/*  
  g_iv=0;
  
  while(g_iv <= 120) {
  mdPID.Compute();
  Serial.print(g_iv,DEC);
  Serial.print(" ");
  Serial.println(g_ov);
  g_iv+=10;
  delay(50);
  }
  while(1) {};
*/  
  
  if(t0 == 0) {
    t0=millis();
    md.setM1Speed(60);
    delay(500);
    g_iv=qe_rpm1_tbi();
    delay(500);
    g_iv=qe_rpm1_tbi();
  }
  
//  mdPID.SetTunings(g_Kpa,g_Kia,g_Kda);
   
  g_iv=qe_rpm1_tbi();
  if(g_iv < 1.0) return;
//  if(g_iv < prev_iv) return;
  prev_iv=g_iv;
  gap=abs(g_SP-g_iv);
//  if(gap < g_SP*0.2) {
  if(gap < -1.0) {
    a='c';
    mdPID.SetTunings(g_Kpc,g_Kic,g_Kdc);
  } else {
    a='a';
    mdPID.SetTunings(g_Kpa,g_Kia,g_Kda);
  }
  if(!mdPID.Compute()) return;
  mm=micros();
  md.setM1Speed(g_ov);

  Serial.print(g_SP,DEC);
  Serial.print(" ");
  Serial.print(mm,DEC);
  Serial.print(" ");
  Serial.print(g_ov,DEC);
  Serial.print(" ");
  Serial.print(g_iv,DEC);
  Serial.print(" ");
//  Serial.println(mdPID.GetITerm(),DEC);
  Serial.println(a);
  
  if(millis()-t0 > 3000) {
    md.setM1Speed(0);
    while(1) {};
  }
  
}

