//
//
//  History
//      V2: 8/2/2024
//        -Disable 1 & 255 brightness
//        -Implement 2/5/10 scheme for double-hold
//        -use keypres when in inactivity timeout (dim)
//        -when wakeup from inactivity sleep, retain original brightness
//
//
//
//

#include <FastLED.h>

#define USES_UART

#ifdef USES_UART
#define DBG_MSG(x) Serial.println(x)
char buf[256];
#else
#define DBG_MSG(x)
#endif

// Which pin on the Arduino is connected to the NeoPixels?
#define BUT_PIN (2)
#define LED_PIN (11)  // On Trinket or Gemma, suggest changing this to 1
#define ENABLE_PIN (3)
// params
#define BRIGHT_START (64)
#define SW_VERSION (2)
#define ST_DEL (100)
#define ST_TERM (200)

// config
#define N_DIG (2)
#define N_SEG (7)
#define MSB_DIG (0)
#define LSB_DIG (1)
#define PIX_PER_SEG (3)
#define PIX_PER_DIG (23)
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
#define EVENT_DOUBLE_LONG_PRESS (9)
#define EVENT_DOUBLE_VERY_LONG_PRESS (10)
#define EVENT_DOUBLE_SUPER_LONG_PRESS (11)
// inputsm states
#define INPUT_STATE_IDLE (0)
#define INPUT_STATE_DOWN (1)
#define INPUT_STATE_UP (2)
#define INPUT_STATE_HOLD (3)
#define INPUT_STATE_LONG_HOLD (4)
#define INPUT_STATE_DOUBLEPRESS (5)
#define INPUT_STATE_DOUBLE_LONGPRESS (6)
// app states
#define STATE_IDLE (0)
#define STATE_SELFTEST (1)
#define STATE_SLEEP (7)
#define STATE_INACTIVITY_REDUCEBRIGHT (8)
#define STATE_INACTIVITY_SLEEP (9)
// button timings
#define TICK_RATE (10)  // 10ms is desired tick *CLOCK SENSITIVE
#define BUTTON_WAIT_CT (20)
#define BUTTON_DEBOUNCE_CT (2)
#define BUTTON_HALFSEC_HOLD_CT (50)
#define BUTTON_TWOSEC_HOLD_CT (200)
#define BUTTON_FIVESEC_HOLD_CT (500)
#define BUTTON_TENSEC_HOLD_CT (1000)
#define CT_RATE (20)
#define HB_CMP (199)
#define HB_MAX (200)
#define COLOR_SCHEME_WHITE (0)
#define COLOR_SCHEME_TIGERS (1)
#define COLOR_SCHEME_MSU (2)
#define COLOR_SCHEME_MILLER (3)
#define COLOR_SCHEME_REDWINGS (4)
#define COLOR_SCHEME_COUNT (5)
#define COLOR_MODE_SCHEME (0)
#define COLOR_MODE_PALLETE_ROT (1)

// go dimmed state after 10 minutes (60000)
//#define INACTIVITY_THRESH1_CT (1000UL)
#define INACTIVITY_THRESH1_CT (60000UL)
// go sleep after 1 hour (360000)
//#define INACTIVITY_THRESH2_CT (2000UL)
#define INACTIVITY_THRESH2_CT (360000UL)

//#define STEST_RATE (3)

// all persistent variables here
int ev, app_st, input_st, score, scheme, color_mode;
int b1_up_ct, b1_dn_ct, tick_ct, hb_timer;
int bright, bright_prev;
unsigned long inactive_ctr;
CRGBPalette16 currentPalette = RainbowColors_p;

//Adafruit_NeoPixel pixels(2*PIX_PER_DIG, PIX_IO, NEO_GRB + NEO_KHZ800);
CRGB pixels[2 * PIX_PER_DIG];

//color scheme table                   white              tigers               MSU                    Miller Lite          RedWings
CRGB msb_color[COLOR_SCHEME_COUNT] = { CRGB(255,255,255), CHSV(20, 255, 255),  CRGB(255,255,255),     CHSV(36, 187, 186),  CRGB(255,255,255)  };
CRGB lsb_color[COLOR_SCHEME_COUNT] = { CRGB(255,255,255), CHSV(171, 255, 255), CHSV(95, 209.92, 77),  CHSV(160, 255, 76),  CHSV(250, 255, 236) }; 


const int seg_pix[N_DIG][N_SEG] = {
  { 0 + PIX_PER_DIG, 4 + PIX_PER_DIG, 7 + PIX_PER_DIG, 10 + PIX_PER_DIG, 13 + PIX_PER_DIG, 17 + PIX_PER_DIG, 20 + PIX_PER_DIG },
  { 0, 4, 7, 10, 13, 17, 20 },
};
const int seg_pix_len[N_DIG][N_SEG] = {
  { 4, 3, 3, 3, 4, 3, 3 },
  { 4, 3, 3, 3, 4, 3, 3 },
};

// segment bitmap for each digit
const int seg_bitmap[2][10] = {  // MSB,LSB
  { 0, 1 + 2, 32 + 16 + 64 + 2 + 4, 32 + 16 + 64 + 8 + 4, 1 + 64 + 16 + 8, 32 + 1 + 64 + 8 + 4, 32 + 1 + 2 + 4 + 8 + 64, 32 + 16 + 8, 1 + 2 + 4 + 8 + 16 + 32 + 64, 1 + 32 + 16 + 64 + 8 },
  { 1 + 2 + 4 + 8 + 16 + 32, 8 + 16, 32 + 16 + 64 + 2 + 4, 32 + 16 + 64 + 8 + 4, 1 + 64 + 16 + 8, 32 + 1 + 64 + 8 + 4, 32 + 1 + 2 + 4 + 8 + 64, 32 + 16 + 8, 1 + 2 + 4 + 8 + 16 + 32 + 64, 1 + 32 + 16 + 64 + 8 }
};

void disp_seg(int pos, int x) {
  static int ctr=0;

  for (int i = 0; i < (N_SEG); i++) {
    int pixel_num;
    int pixel_len;
    CRGB pixel_val;
    CRGB cur_color_msb, cur_color_lsb;

    if (color_mode == 0) {
      cur_color_msb = msb_color[scheme];
      cur_color_lsb = lsb_color[scheme];
    }
    else {
      cur_color_msb = ColorFromPalette( currentPalette, i+ctr, 255, LINEARBLEND);
      cur_color_lsb = ColorFromPalette( currentPalette, i+ctr+23, 255, LINEARBLEND);
    }


    if (pos == 0)
      pixel_val = ((x & (1 << i))) ? cur_color_msb : 0;
    else
      pixel_val = ((x & (1 << i))) ? cur_color_lsb : 0;
    pixel_num = seg_pix[pos][i];
    pixel_len = seg_pix_len[pos][i];

    for (int j = 0; j < pixel_len; ++j)
      pixels[pixel_num + j] = pixel_val;
  }
  ++ctr;

}

void show_score(int the_score) {
  int seg_val;

  //sprintf(buf,  "SHOW_SCORE MSB: score=%d", the_score);
  //DBG_MSG( buf  );
  seg_val = seg_bitmap[MSB_DIG][int(the_score / 10)];
  disp_seg(MSB_DIG, seg_val);  // set LEFT (MSB)

  seg_val = seg_bitmap[LSB_DIG][the_score % 10];
  disp_seg(LSB_DIG, seg_val);  // set RIGHT (LSB)

  FastLED.show();
}

void reset_input_state(int st) {
  b1_dn_ct = 0;
  b1_up_ct = 0;
  input_st = st;
}


void setup() {

  FastLED.addLeds<WS2812, LED_PIN, GRB>(pixels, (PIX_PER_DIG * 2)).setCorrection(TypicalLEDStrip);


#ifdef USES_UART
  Serial.begin(115200);  // 115200
  // SGTBD shorten or remove this delay (was 1sec)
  delay(100);  //Take some time to open up the Serial Monitor
#endif

  // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
  // Any other board, you can remove this part (but no harm leaving it):
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  // END of Trinket-specific code.
  app_st = STATE_IDLE;
  reset_state();

  DBG_MSG("starting");
}
void wake_int() {
}

void loop() {

  delay(TICK_RATE);

  input_sm();
  app_sm();

  if (score == 21 || score == 88)
    color_mode = COLOR_MODE_PALLETE_ROT;
  else
    color_mode = COLOR_MODE_SCHEME;    


  if (app_st == STATE_SLEEP || app_st == STATE_INACTIVITY_SLEEP) {

    corn_sleep();
  }
}

// handle input events, convert input events to app states
void app_sm() {
  int seg_val;

  if (app_st == STATE_SELFTEST) {

    if (tick_ct < ST_DEL)
    {
      score=88;
    }

    if (tick_ct >= ST_DEL) {
      // show version
      score=SW_VERSION;
    }

    if (tick_ct > ST_TERM) {
      app_st = STATE_IDLE;
      score = 0;
    }
    show_score (score);


  } else if (app_st == STATE_IDLE) {
    if (ev == EVENT_SINGLE_PRESS) {
      DBG_MSG("inc score");
      score = score + 1;
    } else if (ev == EVENT_LONG_PRESS) {
      if (tick_ct % CT_RATE == 0) {
        score = score - 1;
        if (score < 0) score = 0;
      }
    } else if (ev == EVENT_VERY_LONG_PRESS) {
      score = 0;  // 88
      DBG_MSG("resetting score");
    } else if (ev == EVENT_DOUBLE_PRESS) {
      // TBD cycle through brightness
      DBG_MSG("change brightness");
      if (bright == 4)
        bright = 16;
      else if (bright == 16)
        bright = 64;
      else if (bright == 64)
        bright = 128;
      else if (bright == 128)
        bright = 4;
      //else if (bright == 255)
      //  bright = 1;
      FastLED.setBrightness(bright);
    } else if (ev == EVENT_DOUBLE_LONG_PRESS) {
      // change color scheme
      scheme = scheme + 1;
      if (scheme >= COLOR_SCHEME_COUNT) scheme = 0;
    } else if (ev == EVENT_DOUBLE_VERY_LONG_PRESS) {
      // go to sleep (debug)
      app_st = STATE_SLEEP;
      DBG_MSG("Going to sleep(dbl+very_long_press)");
      delay(100);
    } else if (ev == EVENT_DOUBLE_SUPER_LONG_PRESS) {
      // change score to 88 (debug)
      score = 88;
    } else if (ev == EVENT_SUPER_LONG_PRESS) {
      app_st = STATE_SLEEP;
      DBG_MSG("Going to sleep");
      delay(100);
    } else if (ev == EVENT_TIMEOUT1) {
      app_st = STATE_INACTIVITY_REDUCEBRIGHT;
      bright_prev = bright;
      bright = 16;
      FastLED.setBrightness(bright);
      // TBD reduce brightness to inactivity value
    }
    show_score(score);
  } else if (app_st == STATE_INACTIVITY_REDUCEBRIGHT) {
    if (inactive_ctr < INACTIVITY_THRESH1_CT) {
      // transition back to full power operation
      bright = bright_prev;
      FastLED.setBrightness(bright);
      app_st = STATE_IDLE;
//      while (digitalRead(BUT_PIN) == STATE_DN)
//        delay(10);
      reset_input_state(INPUT_STATE_IDLE);
      delay(10);
    } else if (ev == EVENT_TIMEOUT2) {
      bright = bright_prev;
      app_st = STATE_INACTIVITY_SLEEP;
    }
  }
  tick_ct = tick_ct + 1;
}

void input_sm() {
  ev = EVENT_NONE;

  // detect buttons
  if (digitalRead(BUT_PIN) == STATE_DN) b1_dn_ct = b1_dn_ct + 1;
  if (digitalRead(BUT_PIN) == STATE_UP) b1_up_ct = b1_up_ct + 1;

  if (b1_dn_ct > 0)
    inactive_ctr = 0;
  else
    ++inactive_ctr;

  //sprintf(line,  "but_ct=%d, %d", b1_up_ct, b1_dn_ct);
  //DBG_MSG( line  );

  if (input_st == INPUT_STATE_IDLE) {
    // generate evens from buttons
    if (b1_dn_ct > 0)
      reset_input_state(INPUT_STATE_DOWN);

    if (inactive_ctr >= INACTIVITY_THRESH2_CT) {
      ev = EVENT_TIMEOUT2;
      DBG_MSG("inactivity timeout 2 exceeded");
    } else if (inactive_ctr >= INACTIVITY_THRESH1_CT) {
      ev = EVENT_TIMEOUT1;
      DBG_MSG("inactivityp timeout 1 exceeded");
    }
  } else if (input_st == INPUT_STATE_DOWN) {
    // button is in unstable "down", look for long presses or wait for stable
    // and transition to "up" state
    // check for long press
    if (b1_dn_ct >= BUTTON_HALFSEC_HOLD_CT) {
      ev = EVENT_LONG_PRESS;
      input_st = INPUT_STATE_HOLD;
      DBG_MSG("long press detected");
    }
    // if stable press detected, transition to up state
    else if (b1_dn_ct >= BUTTON_DEBOUNCE_CT && b1_up_ct >= BUTTON_DEBOUNCE_CT) {
      reset_input_state(INPUT_STATE_UP);
      DBG_MSG("stable down, wait for up");
    }
    // if down was spurious(ie up is stable), transition back to idle
    else if (b1_up_ct >= BUTTON_DEBOUNCE_CT) {
      DBG_MSG("spurios click");
      reset_input_state(INPUT_STATE_IDLE);
    }
  } else if (input_st == INPUT_STATE_UP) {
    // waiting for additional period in stable up position
    if (b1_up_ct >= BUTTON_WAIT_CT) {
      ev = EVENT_SINGLE_PRESS;
      reset_input_state(INPUT_STATE_IDLE);
      DBG_MSG("single press detected");
    }
    // button went to stable down before stabilizing in up: doubleclick
    else if (b1_dn_ct >= BUTTON_DEBOUNCE_CT) {
      reset_input_state(INPUT_STATE_DOUBLEPRESS);
      DBG_MSG("wait for doublepress");
    }
  } else if (input_st == INPUT_STATE_DOUBLEPRESS) {
    if (b1_up_ct >= BUTTON_WAIT_CT) {
      ev = EVENT_DOUBLE_PRESS;
      reset_input_state(INPUT_STATE_IDLE);
      DBG_MSG("double press detected");
    } else if (b1_dn_ct >= BUTTON_TWOSEC_HOLD_CT) {
      // down button continues to be held instead of released, move to "double_xxx_press"
      reset_input_state(INPUT_STATE_DOUBLE_LONGPRESS);
    }
  } else if (input_st == INPUT_STATE_DOUBLE_LONGPRESS) {
    if (b1_up_ct >= BUTTON_WAIT_CT && b1_dn_ct <= BUTTON_TWOSEC_HOLD_CT) {
      ev = EVENT_DOUBLE_LONG_PRESS;
      reset_input_state(INPUT_STATE_IDLE);
      DBG_MSG("double+long press detected");
    }
    if (b1_up_ct >= BUTTON_WAIT_CT && b1_dn_ct <= BUTTON_FIVESEC_HOLD_CT) {
      ev = EVENT_DOUBLE_VERY_LONG_PRESS;
      reset_input_state(INPUT_STATE_IDLE);
      DBG_MSG("double+very long press detected");
    }
    if (b1_up_ct >= BUTTON_WAIT_CT && b1_dn_ct <= BUTTON_TENSEC_HOLD_CT) {
      ev = EVENT_DOUBLE_SUPER_LONG_PRESS;
      reset_input_state(INPUT_STATE_IDLE);
      DBG_MSG("double+super long press detected");
    }
  } else if (input_st == INPUT_STATE_HOLD) {
    // wait for stable up
    if (b1_up_ct >= BUTTON_DEBOUNCE_CT)
      reset_input_state(INPUT_STATE_IDLE);
    // check for super long hold
    else if (b1_dn_ct >= BUTTON_FIVESEC_HOLD_CT) {
      DBG_MSG("super long press detectedd");
      reset_input_state(INPUT_STATE_IDLE);
      ev = EVENT_SUPER_LONG_PRESS;
    }
    // check for very long hold
    else if (b1_dn_ct >= BUTTON_TWOSEC_HOLD_CT) {
      DBG_MSG("very long press detectedd");
      ev = EVENT_VERY_LONG_PRESS;
    } else if (b1_dn_ct >= BUTTON_HALFSEC_HOLD_CT)  // check for super long hold
    {
      DBG_MSG("long press detected");
      ev = EVENT_LONG_PRESS;
    }
  }
}


void reset_state() {
  // initial values for persistent variables
  if (app_st == STATE_INACTIVITY_SLEEP || app_st == STATE_SLEEP) {
    // waking up from inactivity timeout or sleep: resume
    app_st = STATE_IDLE;
  } else {
    // "cold start" powerup or waking up from manual shutdown: restart
    app_st = STATE_SELFTEST;
    score = 0;
    scheme = 0;
    color_mode = 0;
    bright = BRIGHT_START;
    bright_prev = bright;
  }
  ev = EVENT_NONE;
  input_st = INPUT_STATE_IDLE;
  b1_up_ct = 0;
  b1_dn_ct = 0;
  tick_ct = 0;
  hb_timer = 0;  // maintains persistence across sleep
  inactive_ctr = 0;
  // setup HW

  enable_io();
  FastLED.setBrightness(bright);

}

void disable_io() {

  // make everything high impedience
  DDRB = 0x00;
  PORTB = 0x00;
  DDRC = 0x00;
  PORTC = 0x00;
  DDRD = 0x00;
  PORTD = 0x00;
  pinMode(BUT_PIN, INPUT_PULLUP);
}

void enable_io() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(BUT_PIN, INPUT_PULLUP);
  delay(250);
  // wire unused inputs to high, thie sames current
  //ADCSRA = 0; SSSSSSSSSS
  digitalWrite(ENABLE_PIN, 1);
  // {TBD} set unconnected pins to INPUT_PULLUP
}


void corn_sleep() {
  int spurious_wakeup = 1;

  //FastLED.clearData();
  for (int i = 0; i < 2 * PIX_PER_DIG; ++i)
    pixels[i] = 0x00;
  FastLED.show();

  // prepare IO for sleep
  disable_io();

  //Go to sleep now
  DBG_MSG("Sleep: wait for button to clear");
  while (digitalRead(BUT_PIN) == STATE_DN) delay(10);  // wait for finger off button

  DBG_MSG("Sleep: go to sleep now");
  // Allow wake up pin to trigger interrupt on low.

  //attachInterrupt(), wake_int, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUT_PIN), wake_int, FALLING);

  while (spurious_wakeup == 1) {
    // Enter power down state with ADC and BOD module disabled.
    // Wake up when wake up pin is low.
    // LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    // disable ADC
    ADCSRA &= ~(1 << 7);
    // enable sleep
    SMCR |= (1 << 2);  // power down mode
    SMCR |= 1;         // enable sleep
    // BOD disable
    MCUCR != (3 << 5);                       // set both the BODS and BODSE at same time
    MCUCR = (MCUCR & ~(1 << 5)) | (1 << 6);  // then set the BODS bit and clear the BODSE bit at the same time
    __asm__ __volatile__("sleep");

    // if the wakeup request is legit (it not "spurious"), the button should be in a "down" state.
    // if not the case, go back to sleep
    if (digitalRead(BUT_PIN) == STATE_DN)
      spurious_wakeup = 0;  // button should be down for true wake-up request
  }
  // Disable external pin interrupt on wake up pin.
  detachInterrupt(0);

  delay(200);
  DBG_MSG("Sleep: Wake");
  //while (1);


  //    // reset everythibg
  reset_state();
}

