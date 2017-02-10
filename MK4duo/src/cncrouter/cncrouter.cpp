
#include "../../base.h"

void cnc_init() {

   #if ENABLED(CNCROUTER)
      SET_OUTPUT(CNCROUTER_PIN);
      #if ENABLED(FAST_PWM_CNCROUTER)
         setPwmFrequency(CNCROUTER_PIN, 2); // No prescaling. Pwm frequency = F_CPU/256/64
      #endif
   #endif

}


#if ENABLED(CNCROUTER)

#if ENABLED(FAST_PWM_CNCROUTER)

unsigned char fast_pwm_cncrouter;
void setPwmCNCRouter(unsigned char pwm); // XXX pwm level or cnc router speed?

void setPwmCNCRouter(unsigned char pwm) {
  fast_pwm_cncrouter = pwm;
  analogWrite(CNCROUTER_PIN, pwm);
}

int getCNCSpeed() {
   return fast_pwm_cncrouter;
}


#endif

void disable_cncrouter() {
   #if ENABLED(FAST_PWM_CNCROUTER)
      setPwmCNCRouter(0);
   #else
      WRITE_CNCROUTER(LOW);
   #endif
}

void setCNCRouterSpeed(unsigned long rpm, bool clockwise) {
   #if ENABLED(FAST_PWM_CNCROUTER)
     if(rpm > MAX_CNCROUTER_SPEED) rpm = MAX_CNCROUTER_SPEED;
     setPwmCNCRouter(rpm*255/MAX_CNCROUTER_SPEED);
   #else
     WRITE_CNCROUTER((rpm) ? 1 : 0);  
   #endif

}


#endif // CNCROUTER
