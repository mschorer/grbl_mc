/*
  esc spindle controller for grbl
  by ms@ms-ite.de
  
  - POC code!!
  - controlled via i2c
  - connects to a rotary encoder + switch for speed/manual stop
  - outputs a servo signal (1-2ms, 50Hz) for esc control
  - no led feedback (cut trace) as i2c uses digisparks led pin
*/

#include <inttypes.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
//#include <util/twi.h>
//#include <util/delay.h>

#include <Wire.h>

// include the library header
#include <glcd.h>
#include <fonts/allFonts.h>

#include <PID_AutoTune_v0.h>
#include <PID_v1.h>
 
#define F_CPU 16000000

#define VERSION  "1.0k"

#ifndef TWI_RX_BUFFER_SIZE
#define TWI_RX_BUFFER_SIZE ( 16 )
#endif

#define PIN_SDA 0
#define PIN_SCK 2

#define DIR_SERVO  DDRC
#define PORT_SERVO PORTC
#define PIN_SERVO  PC6

#define DIR_LED    DDRC
#define PORT_LED   PORTC
#define PIN_LED    PC7

#define PINB_UPDN   PIND
#define DIR_UPDN    DDRD
#define PORT_UPDN   PORTD
#define PIN_UP      PD2
#define PIN_DOWN    PD3

#define PINB_MUTE   PINC
#define DIR_MUTE    DDRC
#define PORT_MUTE   PORTC
#define PIN_MUTE    PE6

#define DIR_RPM_SENSE   DDRD
#define PORT_RPM_SENSE  PORTD
#define PIN_RPM_SENSE   PD7

#define DEBOUNCE_TICKS 2

#define SERVO_PULSE_1MS 2000      // 1ms
#define SERVO_PULSE_19MS 38000   // 1ms

#define PWM_OFF 0
#define PWM_SLOW 50
#define PWM_MAX 2000

#define RPM_OFF 0
#define RPM_MAX 20000  //8520

#define RPM_PWM_SCALE ( RPM_MAX / PWM_MAX)

#define BLINK_TICKS_MANUAL 5
#define BLINK_TICKS_MASTER 1

#define NOP 0
#define UP 1
#define DOWN 2

#define MSG_LEN 45
#define MSG_MAX 42
//-----------------------------------------------------------

enum SequencerState_t {
  PULSE,
  PAUSE
};

volatile byte state_machine = PULSE;
volatile byte servo_pause = 0;

volatile byte pin_updn = 0xff;
volatile byte pin_mute = 0xff;

volatile int _rpm_value = 0;
volatile int _rpm_current = 0;

volatile uint16_t sys_ticks;

volatile uint16_t _rpm_avg[6];
volatile uint8_t _rpm_avg_idx = 0;
volatile uint16_t _rpm_avg_sum = 0;

volatile int _rpm_pwm = PWM_OFF;
volatile int d_rpm_pwm = -1;

gText rpmDisplay;
bool setup_rpm = true;
int d_rpm_value = -1;
int d_rpm_current = -1;

volatile uint8_t cooling = 0;
uint8_t d_cooling = 0;

gText toolDisplay;
bool setup_tooling = true;
volatile uint8_t tool_index = 0;
uint8_t d_tool_index = 0;
volatile uint8_t tool_current = 0;
uint8_t d_tool_current = 0;

volatile char message[MSG_LEN] = "";
char mbuffer[MSG_LEN] = "";
volatile bool msg_changed = true;
gText msgDisplay, tlabelDisplay, coolDisplay;

volatile boolean _forceOff = false;

double consKp=1.0, consKi=15.0, consKd=5.0;
//double consKp=5, consKi=0.2, consKd=0.5;
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

//int _blinkCount = 0;
//int _blinkPhase = false;

void setup( void) {
  GLCD.Init();
  GLCD.SelectFont(System5x7);

  GLCD.CursorTo(0, 7);
  GLCD.print("gCTRL ");
  GLCD.print( VERSION);

  for( uint8_t i=0; i < 6; i++) {
    _rpm_avg[ i] = 0;
  }

  //----------------------------------------------------------------------------
  // configure pins

  // inputs  
  DIR_UPDN &= ~( (1<< PIN_UP) | (1<< PIN_DOWN));
  PORT_UPDN |= ( (1<< PIN_DOWN) | (1<< PIN_UP));  // enable pullup for inputs

  DIR_MUTE  &= ~( (1<< PIN_MUTE));
  PORT_MUTE |= (1<< PIN_MUTE);  // enable pullup for input
  
  DIR_RPM_SENSE &= ~(1<< PIN_RPM_SENSE);
  PORT_RPM_SENSE |= (1<< PIN_RPM_SENSE);  // enable pullup for input

  // outputs
  DIR_SERVO |= (1<< PIN_SERVO);      // set PE6 as output
  PORT_SERVO &= ~(1<< PIN_SERVO);    // preset to 0

  DIR_LED |= (1<< PIN_LED);      // set Pc7 as output
//  PORT_LED &= ~(1<< PIN_LED);    // preset to 0

  DDRD |= (1<< PD5);

  // enable pin-change interrupts for 3 sources
//  EICRA = ((0<< ISC31) | (1<< ISC30) | (0<< ISC21) | (1<< ISC20)) | ( EICRA & 0x0f);
//  EICRB = (0<< ISC61) | (1<< ISC60);
//  EIMSK |= (1<< INT6) | (1<< INT3) | (1<< INT2);
  
  //----------------------------------------------------------------------------
  // setup time/counters
  
  // setup prescalers
  GTCCR = (0<< TSM) | (0<< PSRSYNC);

  // setup counter for rpm mesuring
  // disconnect oc1a/oc1b, external clock, normal mode, no interrupts
  TCCR0A = 0x00;  //(0<< COM0A1) | (0<< COM0A0) | (0<< COM0B1) | (0<< COM0B0) | (0<< WGM01) | (0<< WGM00);
  TCCR0B = 0x07;  //(0<< FOC0A) | (0<< FOC0B) | (0<< WGM02) | (1<< CS02) | ( 1<< CS01) | (1<< CS00);
  TCNT0 = 0;
  OCR0A = 0;     // not used
  OCR0B = 0;    // not used
  TIMSK0 = 0x00;  //(0<< OCIE0B) | (0<< OCIE0A) | (0<< TOIE0);

  // setup timer for rpm gate
  // disconnect pins, 1024x prescaler, ctc mode, interrupt on ocra match
  TCCR3A = 0x00;  //(0<< COM3A1) | (0<< COM3A0) | (0<<COM3B1) | (0<< COM3B0) | (0<<COM3C1) | (0<< COM3C0) | ( 0<< WGM31) | (0<< WGM30);
  TCCR3B = 0x0d;  //(0<< ICNC3) | (0<< ICES3) | (0<< WGM33) | (1<< WGM32) | (1<< CS32) | ( 0<< CS31) | (1<< CS30);
  TCCR3C = 0x00;  //(0<< FOC3A);
  TCNT3 = 0;
  OCR3A = 3125;  //2604;   // set for 6Hz
  OCR3B = 0;    // not used
  OCR3C = 0;
  TIMSK3 = 0x02;  //(0<< ICIE3) | (0<< OCIE3C) | (0<< OCIE3B) | (1<< OCIE3A) | (0<< TOIE3);

  // setup timer for pulse generation
  // disconnect pins, 8x prescaler, ctc mode, interrupt on ocra match
  TCCR1A = 0x00;  //(0<< COM1A1) | (0<< COM1A0) | (0<<COM1B1) | (0<< COM1B0) | (0<<COM1C1) | (0<< COM1C0) | ( 0<< WGM11) | (0<< WGM10);
  TCCR1B = 0x0a;  //(0<< ICNC1) | (0<< ICES1) | (0<< WGM13) | (1<< WGM12) | (0<< CS12) | ( 1<< CS11) | (0<< CS10);
  TCCR1C = 0x00;  //(0<< FOC1A) | (0<< FOC1B) | (0<< FOC1C);
  TCNT1 = 0;
  OCR1A = 2000;   // set for 2000ticks/ms
  OCR1B = 0;    // not used
  OCR1C = 0;  // 16Mhz / 8 / 20Hz 
  TIMSK1 = 0x02;  //(0<< ICIE1) | (0<< OCIE1C) | (0<< OCIE1B) | (1<< OCIE1A) | (0<< TOIE1);
/*
  // Init the internal PLL
  PLLFRQ = (0<< PINMUX) | (0<< PLLUSB) | (1<< PLLTM1) | (1<< PLLTM0) | (0<< PDIV3) | (1<< PDIV2) | (0<< PDIV1) | (0<< PDIV0);
  PLLCSR |= _BV(PLLE);
  while(!(PLLCSR & _BV(PLOCK)));
   
  // setup pwm mode for led
  // disconnect oc1a/oc1b, external clock, normal mode, no interrupts
  TC4H  = 0x00;
  TCCR4A = 0x82;  //(1<< COM4A1) | (0<< COM4A0) | (0<< COM4B1) | (0<< COM4B0) | (0<< FOC4A) | (0<< FOC4B) |(1<< PWM4A) | (0<< PWM4B);
  TCCR4B = 0x08;  //(0<< PWM4X) | (0<< PSR4) | (0<< DTPS40) | (0<< DTPS41) | (1<< CS43) | (0<< CS42) | (0<< CS41) | (0<< CS40);
  TCCR4C = 0x00;  //(0<< COM4A1S) | (0<< COM4A0S) | (0<< COM4B1S) | (0<< COM4B0S) | (0<< COM4D1) | (0<< COM4D0) | (0<< FOC4D) | (0<< PWM4D);
  TCCR4D = 0x01;  //(0<< FPIE4) | (0<< FPEN4) | (0<< FPNC4) | (0<< FPES4) | (0<< FPAC4) | (0<< FPF4) | (0<< WGM41) | (1<< WGM40);
  TCCR4E = 0x00;  //(0<< TLOCK4) | (0<< ENHC4) | (0<< OC4OE5) | (0<< OC4OE4) | (0<< OC4OE3) | (0<< OE4OC2) | (0<< OE4OC1) | (0<< OE4OC0);
//  TCNT4 = 0;
  OCR4A = 0x80;     // pwm freq
  OCR4B = 0x0;    // not used
  TC4H  = 0x03;
  OCR4C = 0xf0;     // top
  TC4H  = 0x00;
  OCR4D = 0;    // not used
  TIMSK4 = 0x00;  //(0<< OCIE4d) | (0<< OCIE4A) | (0<< OCIE4B) | (0<< TOIE4);
*/
  //----------------------------------------------------------------------------
  // bring up services
  
  Wire.begin( 0x5c);                // join i2c bus with address $5c for SpeedControl
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event

  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  GLCD.print(" [min]");
  
  // do not go into ESC set up mode
  // output off for 4s
  _rpm_pwm = RPM_OFF;
  setLED( _rpm_pwm, true, true);
  sys_ticks = 0;
  while( sys_ticks < 10) {
    ;
  }
  GLCD.print( "[max]");

  // make ESC detect digispark, do not enter ESC setup, allow ESC to scale to pulse/pause ratio
  // output on for 1s
  _rpm_pwm = PWM_MAX;
  setLED( _rpm_pwm, true, false);

  sys_ticks = 0;
  while( sys_ticks < 10) {
    ;
  }

  GLCD.ClearScreen();
  
  Setpoint = 0;
  myPID.SetOutputLimits(0, 2000);
//  myPID.SetSampleTime( 160);
  myPID.SetMode(AUTOMATIC);
}

void loop( void) { 
  setupUI();
  
  // now loop
  _rpm_pwm = PWM_OFF;
  uint16_t tock = sys_ticks;
  bool forcePaint = true;

  while( true) {
    setLED( _rpm_pwm, _forceOff /* || ((PINB & (1<< PIN_MASTER)) == HIGH) */, _forceOff);
    
    updateRpm( forcePaint);
    updateCooling( forcePaint);
    updateTooling( forcePaint);
    if ( msg_changed) updateMessage();

    // do busy waiting, using arduino delay/millis etc will block timer1
    // tock frquency is 6Hz
    while( tock == sys_ticks) {
      ;
    }
    tock = sys_ticks;
    forcePaint = false;
        
    PIND |= (1<< PD5);
  }
}

//----------------------------------------------------------------------------
// handle ui
  
void setupUI() {
  GLCD.CursorTo(0,2);
  GLCD.Printf("rpm");

  coolDisplay = gText( 0, 28, 63, 44, SCROLL_UP);
  coolDisplay.SelectFont( System5x7);
  coolDisplay.SetFontColor(BLACK); // set font color 
  coolDisplay.ClearArea();
  coolDisplay.CursorTo(0,0);
  coolDisplay.print("Coolant");

  rpmDisplay = gText(57, 0, 127, 32, SCROLL_DOWN);
  rpmDisplay.SelectFont( lcdnums14x24 /*Verdana24 fixednums15x31*/);
  rpmDisplay.SetFontColor(BLACK); // set font color 
  rpmDisplay.ClearArea();

  tlabelDisplay = gText( 70, 28, 114, 44, SCROLL_DOWN);
  tlabelDisplay.SelectFont( System5x7);
  tlabelDisplay.SetFontColor(BLACK); // set font color 
  tlabelDisplay.ClearArea();

  tlabelDisplay.CursorTo( 0,0);
  tlabelDisplay.print("Tool#");
  tlabelDisplay.CursorTo( 2,1);
  tlabelDisplay.print("->#");

  toolDisplay = gText( 115, 28, 127, 44, SCROLL_DOWN);
  toolDisplay.SelectFont( lcdnums12x16);
  toolDisplay.SetFontColor(BLACK); // set font color 
  toolDisplay.ClearArea();

  msgDisplay = gText( 0, 48, 127, 63, SCROLL_UP);
  msgDisplay.SelectFont( System5x7);
  msgDisplay.SetFontColor(BLACK); // set font color 
  msgDisplay.ClearArea();
  
  GLCD.DrawRect( 0,0,52,6,BLACK);
}

void updateRpm( bool force) {
  if ( d_rpm_pwm != _rpm_pwm || force) {  
/*    GLCD.CursorTo(0,0);
    if ( _rpm_pwm) GLCD.Printf("%05d", _rpm_pwm);
    else GLCD.Puts( "-off-");
*/    d_rpm_pwm = _rpm_pwm;
    int bar = max( 0, min( 50, round( _rpm_pwm / 40)));
    GLCD.FillRect( 1,1,bar,4,BLACK);
    if ( bar < 50) GLCD.FillRect( bar+1,1,50-bar,4,WHITE);
  }
  if ( d_rpm_value != _rpm_value || force) {  
    GLCD.CursorTo(0,1);
    if ( _rpm_value) GLCD.Printf("%05d", _rpm_value);
    else GLCD.Puts( "-off-");
    d_rpm_value = _rpm_value;
  }
  if ( d_rpm_current != _rpm_current || force) {  
    rpmDisplay.CursorTo(0,0);
    if ( _rpm_current < 20000) rpmDisplay.Printf("%05d", _rpm_current);
    d_rpm_current = _rpm_current;
  }
}

void updateCooling( bool force) {
  if ( cooling != d_cooling || force) {
    coolDisplay.CursorTo(0,1);
    switch( cooling) {
      case 3: coolDisplay.print( "[MST/FLD]"); break;
      case 2: coolDisplay.print( "[---/FLD]"); break;
      case 1: coolDisplay.print( "[MST/---]"); break;
      case 0:
      default: coolDisplay.print( "[---/---]"); break;
    }
    d_cooling = cooling;
  }
}

void updateTooling( bool force) {
  if ( tool_index != d_tool_index || force) {
    tlabelDisplay.CursorTo( 5, 1);
    tlabelDisplay.print( tool_index, DEC);
    d_tool_index = tool_index;
  }
  if ( tool_current != d_tool_current || force) {
    toolDisplay.CursorTo( 0,0);
    toolDisplay.Printf( "%01d", tool_current);
    d_tool_current = tool_current;
  }
}

void updateMessage() {
  uint8_t i=0;
  uint8_t j=0;
  
  while(( j < MSG_MAX) && ( i < MSG_LEN)) {
    if ( message[i] == 0) break;
    
    mbuffer[i] = message[i];
    i++;

    switch( message[i]) {
      case 10:
      case 13:
      break;
    
      default:
        j++;
    }
  }
  mbuffer[i] = 0;  

  msgDisplay.ClearArea();
  msgDisplay.CursorTo(0,0);
  msgDisplay.Puts( mbuffer);
  msg_changed = false;  
}

//----------------------------------------------------------------------------
// handle i2c protocol
  
void receiveEvent( int howMany) {
  int rpm = 0; // receive byte as a character
  uint8_t c = 0;

  while ( howMany > 0) {    
    
    c = Wire.read();
    howMany--;
    
    if ( c & 0x80) {
      rpm = ( c & 0x7f) << 8;
      if ( howMany > 0) {
        c = Wire.read();
        rpm |= c;
        howMany--;
      }
  
      if ( rpm > RPM_MAX) _rpm_value = RPM_MAX;
      else if ( rpm < RPM_OFF) _rpm_value = RPM_OFF;
      else _rpm_value = rpm;
      
      Setpoint = (double) _rpm_value;
    } else {
      switch( (c & 0x70) >> 4) {
        case 0x00:
          cooling = (c & 0x03) ? ( cooling | c & 0x03) : 0;
          break;
          
        case 0x01:
          tool_current = c & 0x0f;
          
        case 0x02:
          tool_index = c & 0x0f;
          break;
          
        case 0x07:
        default:
          byte msgidx = 0;
          while( howMany-- > 0 && msgidx < 31) {
            c = Wire.read();
            message[ msgidx] = c;
            msgidx++;
          }
          message[ msgidx] = 0;
          msg_changed = true;
      }
    }
  }
}

void requestEvent() {
  Wire.write( _rpm_value);
}

//----------------------------------------------------------------------------
// helpers
  
byte gamma_correction(byte input) {
  unsigned int multiplied = input * input;
  return multiplied / 256;
}

byte encoder( byte input) {
  byte res = 0;
  
  if ( input & ( 1<< PIN_UP)) res |= 0x02;
  if ( input & ( 1<< PIN_DOWN)) res |= 0x01;
  
  return res;
}

//----------------------------------------------------------------------------
// interrupt handlers for buttons

// level change interrupt on 
ISR( INT2_vect) {
  turnDecode();
}

ISR( INT3_vect) {
  turnDecode();
}
  
void turnDecode() {
  
  //      00 01 10 11
  //  00  .  -  +  .
  //  01  +  .  .  -
  //  11  .  +  -  .
  //  10  -  .  .  +
  
  const byte enc_states[] = { NOP, DOWN, UP, NOP, 
                              UP, NOP, NOP, DOWN, 
                              DOWN, NOP, NOP, UP, 
                              NOP, UP, DOWN, NOP};

  static byte encoder_pos = 0;
  
  byte pin_dir = PINB_UPDN & ((1<< PIN_UP) | (1<< PIN_DOWN));
  
  // should always be true as we're triggered only on edges
  if ( pin_dir ^ pin_updn) {
    pin_updn = pin_dir;

    byte enc = encoder( pin_dir);
    
    if ( enc != encoder_pos) {
      enc |= encoder_pos << 2;
      
      switch ( enc_states[ enc]) {
        case UP:
          if ( _rpm_value < RPM_MAX) _rpm_value += RPM_PWM_SCALE;
          else _rpm_value = RPM_MAX;
        break;
        
        case DOWN:
          if ( _rpm_value > RPM_OFF) _rpm_value -= RPM_PWM_SCALE;
          else _rpm_value = RPM_OFF;
        break;
        
        default:
          ;
      }
      encoder_pos = enc & 0x03;
    }
  }
}

// level change interrupt on 
ISR( INT6_vect) {
  
  byte mute = PINB_MUTE & (1<< PIN_MUTE);
  
  // should always be true as we're triggered only on edges
  if ( mute ^ pin_mute) {
    pin_mute = mute;

    if (( mute & (1<< PIN_MUTE)) == LOW) _forceOff = !_forceOff;
  }
}

//----------------------------------------------------------------------------
// rpm counter gate interrupt
  
// rpm measurement gate clock
ISR(TIMER3_COMPA_vect) {
  sys_ticks++;
  uint16_t rotations = (uint16_t) TCNT0;

  _rpm_avg_sum -= _rpm_avg[ _rpm_avg_idx];
  _rpm_avg_sum += rotations;

  _rpm_avg[ _rpm_avg_idx] = rotations;
  _rpm_avg_idx++;
  if ( _rpm_avg_idx > 5) _rpm_avg_idx = 0;
  
  _rpm_current = _rpm_avg_sum * 50;  // * 5 * 60 / 6
  TCNT0 = 0;

  Input = (double) _rpm_current;
  myPID.Compute();
  _rpm_pwm = (int) Output;

  PINC |= (1<< PC7);    
}

//----------------------------------------------------------------------------
// pwm timer
  
// timer interrupt for pwm signal generation
ISR(TIMER1_COMPA_vect) {
    switch( state_machine) {
      case PULSE:
        OCR1A = SERVO_PULSE_1MS + _rpm_pwm;
        PORT_SERVO |= (1 << PIN_SERVO);
        state_machine = PAUSE;
      break;

      case PAUSE:
      default:
        OCR1A = SERVO_PULSE_19MS - _rpm_pwm;
        PORT_SERVO &= ~( 1 << PIN_SERVO);

//        _rpm_pwm = ( _forceOff ? 0 : (_rpm_value / RPM_PWM_SCALE));
        state_machine = PULSE;  
    }
}//end ISR TIM0_COMPA_vect

void setLED( byte val, boolean doBlink, boolean freq) {
/*
  TC4H  = (val >> 9) & 0x03;
  OCR4A = 0xff & ( val >> 1);
*/
  /*
  if ( doBlink) {
    if ( _blinkCount <= 0) {
      _blinkPhase = ! _blinkPhase;
      _blinkCount = freq ? BLINK_TICKS_MANUAL : BLINK_TICKS_MASTER;
    } else _blinkCount--;
    
    OCR0A = _blinkPhase ? gamma_correction( val) : 0;
  } else {
    OCR0A = gamma_correction( val);
  }
  */
}

