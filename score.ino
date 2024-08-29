#include "LowPower.h"

#include "math.h"
#define SCORE_NODEC_VER (4)
// Chip: ATMEGA328P
//   Oscillator=8Mhz internal
//   Bootloader=none
//   Brownout=none 
// History
//   12/11/2022: v1.0 declared
//   12/27/2022: re-arranged PIO map to simplify wiring, v2.0 declared, production intent
//   12/28/2022: re-arranged PIO map to simplify wiring, v3.0 declared, production intent
//   12/30/2022: remember last brightness upon shutdown, set low brightness setting to 1, v4.0 declared, production intent, show version on powerup
//



#define USES_UART

#ifdef USES_UART
  #define DBG_MSG(x) Serial.println(x)
  char buf[256];
#else
  #define DBG_MSG(x)
#endif

// define IO pin functions
#define BOARD_LED (5)
#define V12_EN (4)
#define BUT1 (2)
#define PWM_MOD (3)

#define N_DIG (2)
#define N_SEG (7)
#define MSB_DIG (0)
#define LSB_DIG (1)
// GPIO for each segment
const int seg_gpio[N_DIG][N_SEG] = 
{ // GPIOs for segment a, b, c, d, e, f, g (see map below)
  {12, 8,  7,  6,  9,  10, 11}, 
  {19, 13, 14, 15, 16, 17, 18} 
}; 

// segment bitmap for each digit
int seg_bitmap[2][10] = 
{ 
  {0, 6,   91, 121, 116, 109, 111, 56, 127, 125}, 
  {63, 48, 91, 121, 116, 109, 111, 56, 127, 125} 
};

// define conshbtants
#define STATE_DN (0)
#define STATE_UP (1)
#define EVENT_NONE (0)
#define EVENT_SINGLE_PRESS (1)
#define EVENT_DOUBLE_PRESS (2)
#define EVENT_TRIPLE_PRESS (3)
#define EVENT_LONG_PRESS (4)
#define EVENT_VERY_LONG_PRESS (5)
#define EVENT_SUPER_LONG_PRESS (6)
#define EVENT_TIMEOUT1 (7)
#define EVENT_TIMEOUT2 (8)
// inputsm states
#define INPUT_STATE_IDLE (0)
#define INPUT_STATE_DOWN (1)
#define INPUT_STATE_UP (2)
#define INPUT_STATE_HOLD (3)
#define INPUT_STATE_LONG_HOLD (4)
#define INPUT_STATE_DOUBLEPRESS (5)
// app states
#define STATE_IDLE (0)
#define STATE_SELFTEST (1)
#define STATE_SLEEP (7)
#define STATE_INACTIVITY_REDUCEBRIGHT (8)
#define STATE_INACTIVITY_SLEEP (9)

// pwm
#define PWM_OFF (0)
#define PWM_L1 (1)
#define PWM_L2 (16)
#define PWM_L3 (64)
#define PWM_L4 (128)
#define PWM_DEF (PWM_L3)
#define PWM_MAX (255)
#define PWM_MAX2 (PWM_MAX/2)
// button timings
#define TICK_RATE (10)  // 10ms is desired tick *CLOCK SENSITIVE
#define BUTTON_WAIT_CT (20)
#define BUTTON_DEBOUNCE_CT (2)
#define BUTTON_LONG_CT (50)
#define BUTTON_VERY_LONG_CT (200)
#define BUTTON_SUPER_LONG_CT (500)
#define CT_RATE (20)
#define HB_CMP (199)
#define HB_MAX (200)
// go dimmed state after 10 minutes (60000)
//#define INACTIVITY_THRESH1_CT (1000UL)
#define INACTIVITY_THRESH1_CT (60000UL)
// go sleep after 1 hour (360000)
//#define INACTIVITY_THRESH2_CT (2000UL)
#define INACTIVITY_THRESH2_CT (360000UL)

//#define STEST_RATE (3)

// all persistent variables here
int ev, pwm_val, last_pwm_val, app_st, input_st, br_step, score;
int b1_up_ct, b1_dn_ct, tick_ct, hb_timer;
unsigned long inactive_ctr;

void disp_seg(int pos, int x)
{
  for (int i=0; i<(N_SEG); i++)  
    digitalWrite(seg_gpio[pos][i], ( ( x & (1<<i) ) ) ? 1:0 ); 
}




void show_score(int the_score)
{
  int seg_val;

  seg_val = seg_bitmap[MSB_DIG][int(the_score / 10)];
  disp_seg(MSB_DIG, seg_val); // set LEFT (MSB)
  seg_val = seg_bitmap[LSB_DIG][the_score % 10];
  disp_seg(LSB_DIG, seg_val); // set RIGHT (LSB)
}      

void reset_input_state(int st)
{
  b1_dn_ct = 0;
  b1_up_ct = 0;
  input_st = st;
}

void setup() 
{
  reset_state();

  // 2^n divisor, ie:
  // 0 = normal
  // 1 = factor of 2
  // 4 = factor of 16, 1mhz (500ms to 8)
  // 5 = factor of 32, 500khz (500ms to 16s)
//  CLKPR = 0x80;
//  CLKPR = 0x01;  

//  disp_seg(0, 127);
//  disp_seg(1, 127);

#ifdef USES_UART
  Serial.begin(115200);// 115200
  // SGTBD shorten or remove this delay (was 1sec)
  delay(100); //Take some time to open up the Serial Monitor
#endif  

  DBG_MSG("starting...");


}


// the loop function runs over and over again forever
void loop() {
  delay(TICK_RATE);

  input_sm();
  app_sm();
  analogWrite(PWM_MOD, pwm_val);

  // LED test pattern
  digitalWrite(BOARD_LED, (hb_timer > HB_CMP) );  // turn the LED on (HIGH is the voltage level)
  if (++hb_timer > HB_MAX) hb_timer = 0;

#if 1
  if (app_st == STATE_SLEEP || app_st == STATE_INACTIVITY_SLEEP)
  {

    // prepare IO for sleep
    disable_io();     

    //Go to sleep now
    DBG_MSG("Waiting for button to clear");
    while (digitalRead(BUT1) == STATE_DN) delay(10); // wait for finger off button

    // Allow wake up pin to trigger interrupt on low.
    attachInterrupt(0, wakeUp, LOW);
      
    // Enter power down state with ADC and BOD module disabled.
    // Wake up when wake up pin is low.
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 
      
    // Disable external pin interrupt on wake up pin.
    detachInterrupt(0); 

    // reset everythibg
    reset_state();

  }
#endif

}

// handle input events, convert input events to app states
void app_sm()
{
  int seg_val;
  if (app_st == STATE_SELFTEST)
  {
#if 0
    int L = sizeof(testmap1) / sizeof(int);
    if (st_step < L)           // Cycle through segments
    {
      seg_val = testmap1[st_step % L];
      disp_seg(MSB_DIG, seg_val);
      seg_val = testmap2[st_step % L];
      disp_seg(LSB_DIG, seg_val);
      if (tick_ct % STEST_RATE == 0) st_step = st_step + 1;
    }
    else
#endif     
    if (br_step < 100)   // cycle through brightness
    {
      pwm_val = int(PWM_MAX2 + PWM_MAX2*sin( 2 * 2 * 3.1415 * br_step / 100 ));
      disp_seg(MSB_DIG, 127);
      disp_seg(LSB_DIG, 127);
      br_step = br_step + 1;
    }
    else if (br_step < 200)
    {
      seg_val = seg_bitmap[MSB_DIG][0];
      disp_seg(MSB_DIG, seg_val);
      seg_val = seg_bitmap[LSB_DIG][SCORE_NODEC_VER];
      disp_seg(LSB_DIG, seg_val);
      br_step = br_step + 1;
    }
    else
    {
      app_st = STATE_IDLE;
//      st_step = 0;
      pwm_val = last_pwm_val;
    }
  }
  else if (app_st == STATE_IDLE)
  {
    if (ev == EVENT_SINGLE_PRESS) 
    {
      DBG_MSG("inc score");
      score = score + 1;
    }
    else if (ev == EVENT_LONG_PRESS) 
    {
      if (tick_ct % CT_RATE == 0) 
      {
        score = score - 1;
        if (score < 0) score = 0;
      }
    }
    else if (ev == EVENT_VERY_LONG_PRESS) 
    {
      score = 0;
      DBG_MSG("resetting score");
    }
    else if (ev == EVENT_DOUBLE_PRESS)
    {
      if (pwm_val == PWM_MAX)        pwm_val = PWM_L1;
      else if (pwm_val == PWM_L1)    pwm_val = PWM_L2;
      else if (pwm_val == PWM_L2)    pwm_val = PWM_L3;
      else if (pwm_val == PWM_L3)    pwm_val = PWM_L4;
      else if (pwm_val == PWM_L4)    pwm_val = PWM_MAX;
      DBG_MSG("change brightness");

    }
    else if (ev == EVENT_SUPER_LONG_PRESS)
    {
      app_st = STATE_SLEEP;
      last_pwm_val = pwm_val;
    }
    else if (ev == EVENT_TIMEOUT1)
    {
      app_st = STATE_INACTIVITY_REDUCEBRIGHT;
      last_pwm_val = pwm_val;
      pwm_val = PWM_L1;
    }
    show_score(score);
  }
  else if (app_st == STATE_INACTIVITY_REDUCEBRIGHT)
  {
    if  (inactive_ctr < INACTIVITY_THRESH1_CT)
    {
      // transition back to full power operation
      app_st = STATE_IDLE;
      pwm_val = last_pwm_val;
      while (digitalRead(BUT1) == STATE_DN) 
        delay(10);
      reset_input_state(INPUT_STATE_IDLE);
      delay(10);
    }
    else if (ev == EVENT_TIMEOUT2)
    {
      app_st = STATE_INACTIVITY_SLEEP;
    }
  }
  tick_ct = tick_ct + 1;
}

void input_sm()
{
  ev = EVENT_NONE;

  // detect buttons
  if ( digitalRead(BUT1) == STATE_DN ) b1_dn_ct = b1_dn_ct + 1;
  if ( digitalRead(BUT1) == STATE_UP ) b1_up_ct = b1_up_ct + 1;
  
  if (b1_dn_ct > 0) 
    inactive_ctr = 0;
  else
    ++inactive_ctr;

  //sprintf(line,  "but_ct=%d, %d", b1_up_ct, b1_dn_ct);
  //DBG_MSG( line  );
  
  if ( input_st == INPUT_STATE_IDLE)
  {
    // generate evens from buttons
    if (b1_dn_ct > 0)
      reset_input_state(INPUT_STATE_DOWN);

    if (inactive_ctr >= INACTIVITY_THRESH2_CT)
    {
      ev = EVENT_TIMEOUT2;
      DBG_MSG("inactivity timeout 2 exceeded");
    }
    else if (inactive_ctr >= INACTIVITY_THRESH1_CT)
    {
      ev = EVENT_TIMEOUT1;
      DBG_MSG("inactivityp timeout 1 exceeded");
    }
  }
  else if (input_st == INPUT_STATE_DOWN)
  {
    // button is in unstable "down", look for long presses or wait for stable
    // and transition to "up" state
    // check for long press
    if ( b1_dn_ct >= BUTTON_LONG_CT)
    {
      ev = EVENT_LONG_PRESS;
      input_st = INPUT_STATE_HOLD;
      DBG_MSG("long press detected");
    }
    // if stable press detected, transition to up state
    else if ( b1_dn_ct >= BUTTON_DEBOUNCE_CT && b1_up_ct >= BUTTON_DEBOUNCE_CT)
    {
       reset_input_state(INPUT_STATE_UP);
       DBG_MSG("stable down, wait for up");
    }
    // if down was spurious(ie up is stable), transition back to idle
    else if (b1_up_ct >= BUTTON_DEBOUNCE_CT)
    {
      DBG_MSG("spurios click");
      reset_input_state(INPUT_STATE_IDLE);
    }
  }   
  else if (input_st == INPUT_STATE_UP)
  {    
    // waiting for additional period in stable up position
    if (b1_up_ct >= BUTTON_WAIT_CT)
    {
      ev = EVENT_SINGLE_PRESS;    
      reset_input_state(INPUT_STATE_IDLE);
      DBG_MSG("single press detected");
    }
    // button went to stable down before stabilizing in up: doubleclick
    else if (b1_dn_ct >= BUTTON_DEBOUNCE_CT)
    {
      reset_input_state(INPUT_STATE_DOUBLEPRESS);
      DBG_MSG("wait for doublepress");
    }
  }  
  else if (input_st == INPUT_STATE_DOUBLEPRESS)
  {
    if (b1_up_ct >= BUTTON_WAIT_CT)
    {
      ev = EVENT_DOUBLE_PRESS;
      reset_input_state(INPUT_STATE_IDLE);
      DBG_MSG("double press detected");   
    }
  }
  else if ( input_st == INPUT_STATE_HOLD)
  {    
    // wait for stable up
    if (b1_up_ct >= BUTTON_DEBOUNCE_CT) 
      reset_input_state(INPUT_STATE_IDLE);
    // check for super long hold
    else if ( b1_dn_ct >= BUTTON_SUPER_LONG_CT)
    {
      reset_input_state(INPUT_STATE_IDLE);
      ev = EVENT_SUPER_LONG_PRESS;
    }
    // check for very long hold
    else if (b1_dn_ct >= BUTTON_VERY_LONG_CT)
      ev = EVENT_VERY_LONG_PRESS;
    else if ( b1_dn_ct >= BUTTON_LONG_CT ) // check for super long hold
      ev = EVENT_LONG_PRESS;
  }
}


void wakeUp()
{
    // Just a handler for the pin interrupt.
}
void reset_state()
{
  // initial values for persistent variables
  if (app_st == STATE_INACTIVITY_SLEEP || app_st == STATE_SLEEP)
  {
    // waking up from inactivity timeout: resume
    app_st = STATE_IDLE;
    pwm_val = last_pwm_val;
  }
  else
  {
    last_pwm_val = PWM_DEF;
    // powerup or waking up from manual shutdown: restart
    app_st = STATE_SELFTEST;
    score = 0;
    //pwm_val = PWM_DEF; // will be set after selftest
  }
  ev = EVENT_NONE;
  input_st = INPUT_STATE_IDLE;
  br_step = 0;
  b1_up_ct = 0;
  b1_dn_ct = 0;
  tick_ct = 0;
  hb_timer = 0; // maintains persistence across sleep
  inactive_ctr = 0;
  // setup HW

  enable_io();
  
}

void disable_io()
{
  // make everything high impedience
  pinMode(BOARD_LED, INPUT);
  pinMode(BUT1, INPUT_PULLUP);
  pinMode(PWM_MOD, INPUT);
  for (int i=0; i<N_SEG; i++)
  {
    pinMode(seg_gpio[MSB_DIG][i], INPUT);
    pinMode(seg_gpio[LSB_DIG][i], INPUT);
  }

  // shut down switched +5 and +12v supplies
  pinMode(V12_EN, INPUT);
}

void enable_io()
{
  // initialize digital pin BOARD_LED as an output.
  pinMode(V12_EN, OUTPUT);
  // enable switch 5v and 12v supply
  digitalWrite(V12_EN, 0); // enable 12 volt supply
  delay(200);

  pinMode(BOARD_LED, OUTPUT);
  pinMode(BUT1, INPUT_PULLUP);
  pinMode(PWM_MOD, OUTPUT);
  // set PWM frequency (clock sensitive)
  // TCCR2B = TCCR2B & B11111000 | B00000100;    // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz (The DEFAULT)
  TCCR2B = TCCR2B & B11111000 |    B00000010;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
  for (int i=0; i<N_SEG; i++)
  {
      pinMode(seg_gpio[MSB_DIG][i], OUTPUT);
      pinMode(seg_gpio[LSB_DIG][i], OUTPUT);
  }
  // set initial states!
  disp_seg(MSB_DIG, 127);
  disp_seg(LSB_DIG, 127);

  // wire unused inputs to high, thie sames current
  ADCSRA = 0; 

// {TBD} set unconnected pins to INPUT_PULLUP

}

