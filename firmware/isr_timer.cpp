#include "isr_timer.h"
#include "firmware.h"
#include "SAMDTimerInterrupt.h"
#include "SAMD_ISR_Timer.h"

#if (TIMER_INTERRUPT_USING_SAMD21)
// Init SAMD timer TIMER_TCC
//SAMDTimer ITimer(TIMER_TC4);
//SAMDTimer ITimer(TIMER_TC5);
//SAMDTimer ITimer(TIMER_TCC);
//SAMDTimer ITimer(TIMER_TCC1);
//SAMDTimer ITimer(TIMER_TCC2);
#endif
// Depending on the board, you can select SAMD21 Hardware Timer from TC3, TC4, TC5, TCC, TCC1 or TCC2
// SAMD51 Hardware Timer only TC3
// Init SAMD timer TIMER_TC3 
SAMDTimer ITimer(TIMER_TC3); // using TIMER_TC3

// Init SAMD_ISR_Timer
// Each SAMD_ISR_Timer can service 16 different ISR-based timers
SAMD_ISR_Timer ISR_Timer;

void timerHandler(){
  ISR_Timer.run();  
}

void timerSetup(){
  if (ITimer.attachInterruptInterval_MS(HW_TIMER_INTERVAL_MS, timerHandler)){
    Serial.print(F("Starting ITimer OK, millis() = ")); 
    Serial.println(millis());
  }
  else {
    Serial.println(F("Can't set ITimer. Select another freq. or timer"));
  }

  // You can use up to 16 timer for each ISR_Timer
  ISR_Timer.setInterval(1000L,  PIDControl);
}
