

#ifndef CNCROUTER_H
#define CNCROUTER_H

void cnc_init(); // initialize cnc router


#if ENABLED(FAST_PWM_CNCROUTER)
int getCNCSpeed();
#endif

void setCNCRouterSpeed(unsigned long rpm, bool clockwise);
void disable_cncrouter();

#endif
