#include "LowPower.h"
#include "math.h"

//#define USES_UART

#ifdef USES_UART
  #define DBG_MSG(x) Serial.println(x)
  char buf[256];
#else
  #define DBG_MSG(x)
#endif

// define IO pin functions"

#define BOARD_LED (LED_BUILTIN)
#define CABINET_LIGHT (5)
#define V12_EN (7)
#define BUT1 (2)
#define PWM_MOD (11)

#define N_BCD (4)
#define N_DIG (2)
#define MSB_DIG (0)
#define LSB_DIG (1)
const int seg_gpio[N_DIG][N_BCD] = { {9, 16, 12, 10}, {14, 19, 18, 15}  }; // BCD binary digitsn, MSB/LSB order

int testmap[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0};


// define constants
#define STATE_DN (0)
#define STATE_UP (1)
#define EVENT_NONE (0)
#define EVENT_SINGLE_PRESS (1)
#define EVENT_DOUBLE_PRESS (2)
#define EVENT_TRIPLE_PRESS (3)
#define EVENT_LONG_PRESS (4)
#define EVENT_VERY_LONG_PRESS (5)
#define EVENT_SUPER_LONG_PRESS (6)
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
#define STATE_INC (2)
#define STATE_CLEAR (5)
#define STATE_SLEEP (7)
// pwm
#define PWM_OFF (0)
#define PWM_L1 (4)
#define PWM_L2 (16)
#define PWM_L3 (64)
#define PWM_MAX (255)
#define PWM_MAX2 (PWM_MAX/2)
// button timings
#define TICK_RATE (10) // 10ms is desired tick *CLOCK SENSITIVE
#define BLINK_MOD (10) // 10hz if 10ms
#define BUTTON_WAIT_CT (20)
#define BUTTON_DEBOUNCE_CT (2)
#define BUTTON_LONG_CT (50)
#define BUTTON_VERY_LONG_CT (200)
#define BUTTON_SUPER_LONG_CT (500)
#define CT_RATE (20)

// all persistent variables here
int ev, pwm_val, app_st, input_st, br_step, score;
int b1_up_ct, b1_dn_ct, tick_ct, loop_count;

void disp_seg(int pos, int x)
{
  int dig;

  dig = ( (1<<0) & x) ? 1 : 0;  digitalWrite(seg_gpio[pos][0], dig);
  dig = ( (1<<1) & x) ? 1 : 0;  digitalWrite(seg_gpio[pos][1], dig);
  dig = ( (1<<2) & x) ? 1 : 0;  digitalWrite(seg_gpio[pos][2], dig);
  dig = ( (1<<3) & x) ? 1 : 0;  digitalWrite(seg_gpio[pos][3], dig);
}

void reset_state()
{
  // initial values for persistent variables
  ev = EVENT_NONE;
  pwm_val = 255;
  app_st = STATE_SELFTEST;
  input_st = INPUT_STATE_IDLE;
  br_step = 0;
  score = 0;
  b1_up_ct = 0;
  b1_dn_ct = 0;
  tick_ct = 0;
  loop_count = 0; // maintains persistence across sleep
  // setup HW

  enable_io();
  
}

void show_score(int the_score)
{
    int seg_val;
    seg_val = int(the_score / 10);
    disp_seg(MSB_DIG, seg_val); // set LEFT (MSB)
    seg_val = the_score % 10;
    disp_seg(LSB_DIG, seg_val); // set RIGHT (LSB)
}      
void reset_input_state(int st)
{
  b1_dn_ct = 0;
  b1_up_ct = 0;
  input_st = st;
}


// the setup function runs once when you press reset or power the board
void setup() {

  pinMode(V12_EN, OUTPUT);
  // enable switch 5v and 12v supply
  digitalWrite(V12_EN, 0); // enable 12 volt supply
  pinMode(LED_BUILTIN, OUTPUT);

  reset_state();

  // 2^n divisor, ie:
  // 0 = normal
  // 1 = factor of 2
  // 4 = factor of 16, 1mhz (500ms to 8)
  // 5 = factor of 32, 500khz (500ms to 16s)
//  CLKPR = 0x80;
//  CLKPR = 0x01;  

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
  digitalWrite(LED_BUILTIN, loop_count%BLINK_MOD);  // turn the LED on (HIGH is the voltage level)
  ++loop_count;

#if 1
  if (app_st == STATE_SLEEP)
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


void app_sm()
{
  int seg_val;
  if (app_st == STATE_SELFTEST)
  {
    if (br_step < 100)   // cycle through brightness
    {
        pwm_val = int(PWM_MAX2 + PWM_MAX2*sin( 2 * 2 * 3.1415 * br_step / 100 ));
        disp_seg(MSB_DIG, 8);
        disp_seg(LSB_DIG, 8);
        br_step = br_step + 1;
    }
    else
    {
      app_st = STATE_IDLE;
      pwm_val = PWM_MAX;
    }
  }
  else
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
      else if (pwm_val == PWM_L3)    pwm_val = PWM_MAX;
      DBG_MSG("change brightness");

    }
    else if (ev == EVENT_SUPER_LONG_PRESS)  app_st = STATE_SLEEP;
    show_score(score);
  }
  tick_ct = tick_ct + 1;
}

void input_sm()
{
  ev = EVENT_NONE;

  // detect buttons
  if ( digitalRead(BUT1) == STATE_DN ) b1_dn_ct = b1_dn_ct + 1;
  if ( digitalRead(BUT1) == STATE_UP ) b1_up_ct = b1_up_ct + 1;
  
  //sprintf(line,  "but_ct=%d, %d", b1_up_ct, b1_dn_ct);
  //DBG_MSG( line  );
  
  if ( input_st == INPUT_STATE_IDLE)
  {
    // generate evens from buttons
    if (b1_dn_ct > 0)
      reset_input_state(INPUT_STATE_DOWN);
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

void disable_io()
{
      // make everything high impedience
    pinMode(LED_BUILTIN, INPUT);
    pinMode(CABINET_LIGHT, INPUT);
    pinMode(BUT1, INPUT_PULLUP);
    pinMode(PWM_MOD, INPUT);
    pinMode(seg_gpio[0][0], INPUT);
    pinMode(seg_gpio[0][1], INPUT);
    pinMode(seg_gpio[0][2], INPUT);
    pinMode(seg_gpio[0][3], INPUT);
    pinMode(seg_gpio[1][0], INPUT);
    pinMode(seg_gpio[1][1], INPUT);
    pinMode(seg_gpio[1][2], INPUT);
    pinMode(seg_gpio[1][3], INPUT);

    // shut down switched +5 and +12v supplies
    pinMode(V12_EN, INPUT);


}

void enable_io()
{
    // initialize digital pin LED_BUILTIN as an output.
  pinMode(V12_EN, OUTPUT);
  // enable switch 5v and 12v supply
  digitalWrite(V12_EN, 0); // enable 12 volt supply

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(CABINET_LIGHT, OUTPUT);
  pinMode(BUT1, INPUT_PULLUP);
  pinMode(PWM_MOD, OUTPUT);
  pinMode(seg_gpio[0][0], OUTPUT);
  pinMode(seg_gpio[0][1], OUTPUT);
  pinMode(seg_gpio[0][2], OUTPUT);
  pinMode(seg_gpio[0][3], OUTPUT);
  pinMode(seg_gpio[1][0], OUTPUT);
  pinMode(seg_gpio[1][1], OUTPUT);
  pinMode(seg_gpio[1][2], OUTPUT);
  pinMode(seg_gpio[1][3], OUTPUT);

  // set cabinet ligth
  analogWrite(CABINET_LIGHT, PWM_MAX);

  // set initial states!
  disp_seg(MSB_DIG, 8);
  disp_seg(LSB_DIG, 8);

}
