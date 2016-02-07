/* 
 * This section contains the hardware specific portions of IRrecvBase
 */
/* If your hardware is set up to do both output and input but your particular sketch
 * doesn't do any output, this method will ensure that your output pin is low
 * and doesn't turn on your IR LED or any output circuit.
 */
void IRrecvBase::No_Output (void) {
#if defined(IR_SEND_PWM_PIN)
 pinMode(IR_SEND_PWM_PIN, OUTPUT);  
 digitalWrite(IR_SEND_PWM_PIN, LOW); // When not sending PWM, we want it low    
#endif
}

// enable/disable blinking of pin 13 on IR processing
void IRrecvBase::blink13(bool blinkflag)
{
  irparams.blinkflag = blinkflag;
  if (blinkflag)
     pinMode(BLINKLED, OUTPUT);
}

//Do the actual blinking off and on
//This is not part of IRrecvBase because it may need to be inside an ISR
//and we cannot pass parameters to them.
void do_Blink(void) {
  if (irparams.blinkflag) {
    if(irparams.rawlen % 2) {
      BLINKLED_ON();  // turn pin 13 LED on
    } 
    else {
      BLINKLED_OFF();  // turn pin 13 LED off
    }
  }
}
#ifdef USE_IRRECV
/*
 * The original IRrecv which uses 50µs timer driven interrupts to sample input pin.
 */
void IRrecv::resume() {
  // initialize state machine variables
  irparams.rcvstate = STATE_IDLE;
  IRrecvBase::resume();
}

void IRrecv::enableIRIn(void) {
  IRrecvBase::enableIRIn();
  // setup pulse clock timer interrupt
  cli();
  IR_RECV_CONFIG_TICKS();
  IR_RECV_ENABLE_INTR;
  sei();
}

bool IRrecv::GetResults(IRdecodeBase *decoder) {
  if (irparams.rcvstate != STATE_STOP) return false;
  IRrecvBase::GetResults(decoder,USECPERTICK);
  return true;
}

#define _GAP 5000 // Minimum map between transmissions
#define GAP_TICKS (_GAP/USECPERTICK)
/*
 * This interrupt service routine is only used by IRrecv and may or may not be used by other
 * extensions of the IRrecBase. It is timer driven interrupt code to collect raw data.
 * Widths of alternating SPACE, MARK are recorded in rawbuf. Recorded in ticks of 50 microseconds.
 * rawlen counts the number of entries recorded so far. First entry is the SPACE between transmissions.
 * As soon as a SPACE gets long, ready is set, state switches to IDLE, timing of SPACE continues.
 * As soon as first MARK arrives, gap width is recorded, ready is cleared, and new logging starts.
 */
ISR(IR_RECV_INTR_NAME)
{
  enum irdata_t {IR_MARK=0, IR_SPACE=1};
  irdata_t irdata = (irdata_t)digitalRead(irparams.recvpin);
  irparams.timer++; // One more 50us tick
  if (irparams.rawlen >= RAWBUF) {
    // Buffer overflow
    irparams.rcvstate = STATE_STOP;
  }
  switch(irparams.rcvstate) {
  case STATE_IDLE: // In the middle of a gap
    if (irdata == IR_MARK) {
      if (irparams.timer < GAP_TICKS) {
        // Not big enough to be a gap.
        irparams.timer = 0;
      } 
      else {
        // gap just ended, record duration and start recording transmission
        irparams.rawlen = 0;
        irparams.rawbuf[irparams.rawlen++] = irparams.timer;
        irparams.timer = 0;
        irparams.rcvstate = STATE_MARK;
      }
    }
    break;
  case STATE_MARK: // timing MARK
    if (irdata == IR_SPACE) {   // MARK ended, record time
      irparams.rawbuf[irparams.rawlen++] = irparams.timer;
      irparams.timer = 0;
      irparams.rcvstate = STATE_SPACE;
    }
    break;
  case STATE_SPACE: // timing SPACE
    if (irdata == IR_MARK) { // SPACE just ended, record it
      irparams.rawbuf[irparams.rawlen++] = irparams.timer;
      irparams.timer = 0;
      irparams.rcvstate = STATE_MARK;
    } 
    else { // SPACE
      if (irparams.timer > GAP_TICKS) {
        // big SPACE, indicates gap between codes
        // Mark current code as ready for processing
        // Switch to STOP
        // Don't reset timer; keep counting space width
        irparams.rcvstate = STATE_STOP;
      } 
    }
    break;
  case STATE_STOP: // waiting, measuring gap
    if (irdata == IR_MARK) { // reset gap timer
      irparams.timer = 0;
    }
    break;
  }
  do_Blink();
}
#endif //end of ifdef USE_IRRECV
/*
 * The hardware specific portions of IRsendBase
 */
void IRsendBase::enableIROut(unsigned char khz) {
//NOTE: the comments on this routine accompanied the original early version of IRremote library
//which only used TIMER2. The parameters defined in IRLibTimer.h may or may not work this way.
  // Enables IR output.  The khz value controls the modulation frequency in kilohertz.
  // The IR output will be on pin 3 (OC2B).
  // This routine is designed for 36-40KHz; if you use it for other values, it's up to you
  // to make sure it gives reasonable results.  (Watch out for overflow / underflow / rounding.)
  // TIMER2 is used in phase-correct PWM mode, with OCR2A controlling the frequency and OCR2B
  // controlling the duty cycle.
  // There is no prescaling, so the output frequency is 16MHz / (2 * OCR2A)
  // To turn the output on and off, we leave the PWM running, but connect and disconnect the output pin.
  // A few hours staring at the ATmega documentation and this will all make sense.
  // See my Secrets of Arduino PWM at http://www.righto.com/2009/07/secrets-of-arduino-pwm.html for details.
  
  // Disable the Timer2 Interrupt (which is used for receiving IR)
 IR_RECV_DISABLE_INTR; //Timer2 Overflow Interrupt    
 pinMode(IR_SEND_PWM_PIN, OUTPUT);  
 digitalWrite(IR_SEND_PWM_PIN, LOW); // When not sending PWM, we want it low    
 IR_SEND_CONFIG_KHZ(khz);
 }

IRsendBase::IRsendBase () {
 pinMode(IR_SEND_PWM_PIN, OUTPUT);  
 digitalWrite(IR_SEND_PWM_PIN, LOW); // When not sending PWM, we want it low    
}

//The Arduino built in function delayMicroseconds has limits we wish to exceed
//Therefore we have created this alternative
void  My_delay_uSecs(unsigned int T) {
  if(T){if(T>16000) {delayMicroseconds(T % 1000); delay(T/1000); } else delayMicroseconds(T);};
}

void IRsendBase::mark(unsigned int time) {
 IR_SEND_PWM_START;
 IR_SEND_MARK_TIME(time);
 Extent+=time;
}

void IRsendBase::space(unsigned int time) {
 IR_SEND_PWM_STOP;
 My_delay_uSecs(time);
 Extent+=time;
}

/*
 * Various debugging routines
 */


#ifdef IRLIB_TRACE
void IRLIB_ATTEMPT_MESSAGE(const __FlashStringHelper * s) {Serial.print(F("Attempting ")); Serial.print(s); Serial.println(F(" decode:"));};
void IRLIB_TRACE_MESSAGE(const __FlashStringHelper * s) {Serial.print(F("Executing ")); Serial.println(s);};
byte IRLIB_REJECTION_MESSAGE(const __FlashStringHelper * s) { Serial.print(F(" Protocol failed because ")); Serial.print(s); Serial.println(F(" wrong.")); return false;};
byte IRLIB_DATA_ERROR_MESSAGE(const __FlashStringHelper * s, unsigned char index, unsigned int value, unsigned int expected) {  
 IRLIB_REJECTION_MESSAGE(s); Serial.print(F("Error occurred with rawbuf[")); Serial.print(index,DEC); Serial.print(F("]=")); Serial.print(value,DEC);
 Serial.print(F(" expected:")); Serial.println(expected,DEC); return false;
};
#endif

