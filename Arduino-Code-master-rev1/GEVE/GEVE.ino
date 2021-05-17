

/* ----------------------
  GEVE - Ventilator monitoring system V1.0

  Date : 07.04.20

  Authors :
  Stoeckli H. - HEPIA
  Rossel R.   - ANGARA TECHNOLOGY


  Inputs :
  ON/OFF - Switch
  Error acknowledgment - Switch
  Breath per minute - bpm (breath/minute)  - Potentiometer - 8 to 30 - typical 12
  Tidal Volumne - VT (ml)  - Potentiometer - 200 to 800 ml - need to be calibrated (better start low)
  Inspiration/expiration ratio - IE (-) - Potentiometer - 1:3 to 1:1 - typical 1:2
  Pressure - Pressure sensor SSCDRNN160MDAA5 +/- 163 cmH2O (ideally SSCDRNN100MDAA5 but unavailable, nother +/- 60 to 100 cmH20 sensor can fit)

  Outputs :
  LCD screen 4X20 2004A - Displays BPM, VT, IE, Pressure, Plateau Pressure, PEEP and errors
  Motor driver LECPA OM04505
  Buzzer - CMT-8540S-SMT-TR (to be tested)
  LED


  Timers :

  Timer 0 : Used with Time function
  Timer 1 : Set Repiratory rythm trough motor control
  Timer 2 : Used with Tone() function (alarm)

  Modes :

  Mode 1 : Volume control - selected breaths delivered at constant rate, pressure monitored only for
  safety. - Only suitable for sedated or paralyzed patients.

  Mode 2  : ****    NOT IMPLEMENTED YET    ****     Assist control - When the patient tries to breathe in,
  the pressure sensor will see a pressure drop, and the machine will begin squeezing the bag in order
  to assist in the breath.
  Because the compression is triggered by the patient’s breath,
  the machine will be operating in sync with the patient’s natural breathing.
  A maximum time-between-breath timeout will also be used to deliver a breath after a certain time,
  even if the patient doesn’t trigger it. This is so that the machine will still deliver a minimum amount of breathing,
  even if the patient becomes unable to breathe for themselves. An alert will sound.


  Code related to the PC interface is marked with 'PCIF' to easily find it.

*/

// ================= CHANGE LCD MODEL HERE ====================0
//#define LCD_NEWHAVEN 0       // NOT FULLY OPERATIONNAL !
#define LCD_2004A 1

#include <Wire.h>
#ifdef LCD_2004A
#include "LiquidCrystal_I2C.h"
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>


//---- MIN/MAX entry values and pressure limits ----//

#define MIN_BPM 8                       // in breath/minute
#define MAX_BPM 30                      // in breath/minute
#define DELTA_BPM (MAX_BPM-MIN_BPM)     // used to optimize calculus

#define MIN_VT 200                      // in ml
#define MAX_VT 800                      // in ml
#define DELTA_VT (MAX_VT-MIN_VT)    // In ml, used to optimize calculus

#define MIN_IE 1                        // in ratio E/I
#define MAX_IE 3                        // in ratio E/I
#define DELTA_IE (MAX_IE-MIN_IE)        // In ratio, used to optimize calculus

#define MIN_PEEP 1                      // in cmH2O
#define MAX_PEEP 20                     // in cmH2O
#define DELTA_PEEP (MAX_PEEP-MIN_PEEP)  // In cmH2O, used to optimize calculus
#define PEEP_MARGIN 3                 // in cmH2O, added pressure value to PEEP to start at initialization


//---- Fixed parameters ----//

// Mechanical

#define DISTANCE_PER_PULSE 0.0125                                       // In mm, linear motor displacement for each pulse 
#define SQUEEZED_VOLUME    1000                                         // In ml, total volume when bag squeezed (not total bag volume)
//#define TOTAL_RANGE        110
#define TOTAL_RANGE        50                                           // In mm
#define VOLUME_PER_DISTANCE    SQUEEZED_VOLUME/TOTAL_RANGE              // In ml/mm (approximated total volume 1L/20cm approximated total range)
#define VOLUME_PER_PULSE       VOLUME_PER_DISTANCE*DISTANCE_PER_PULSE   // In ml/pulse
#define MOTOR_POSITION_LIMIT_INIT_MM       170                           // Max init range before error in mm 
#define MOTOR_POSITION_LIMIT_INIT_PULSE   MOTOR_POSITION_LIMIT_INIT_MM/DISTANCE_PER_PULSE  // Max init range before error in pulses
#define MOTOR_PULSE_WIDTH   4                                      // In us, note that due to the execution time, the real value will be a little bit higher (few us more ) 


// Pressure sensor
#define PRESSURE_SENSOR_POS_DYNAMIC 163 // Pressure sensor positive dynamic in cmH2O
#define MAX_PRESSURE 38                 // Max pressure at any moment in cmH2O


#define ALARM_FREQUENCY 4000            // Alarm sound frequency  
#define ALARM_SPEED 10000               // Repetitive speed of alarm

#define STATE_OK 0                      // System state ok
#define STATE_ERR_HIGH_PRESSURE 1       // Overpressure occured, need acknowledgement
#define STATE_ERR_LOW_PRESSURE 2        // Low pressure occured, need acknowledgement
#define STATE_ERR_FAIL_INIT 3           // Initialisation failure occured
#define STATE_ERR_MOTOR 4               // Motor failure detected 

#define ON 1
#define OFF 0
#define BUTTON_PUSHED 0
#define BUTTON_RELEASED 1



//---- Machine state definition ----//

#define STOP 0
#define SET_INIT 1
#define INIT 2
#define SET_INHALE 3
#define INHALE 4
#define SET_PAUSE1 5
#define PAUSE1 6
#define SET_EXHALE 7
#define EXHALE 8
#define SET_PAUSE2 9
#define PAUSE2 10

//---- Pins definition ----//

//Analog inputs
#define PIN_BPM_IN A0
#define PIN_IE_IN A1
#define PIN_VT_IN A2
#define PIN_PRESSURE_IN A3
// A4 AND A5 RESERVED FOR I2C


//Digital inputs
#define PIN_SWITCH_ONOFF 2
#define PIN_SWITCH_ERR_ACK 3

#define PIN_WAREA 4        // Motor position between W-AREA1 and W-AREA2 (defined in driver)
#define PIN_BUSY 5         // If ON, Motor is positioning 
#define PIN_SETON 6         //  ON, if motor went back once to origin and knows his position
#define PIN_INP 7         //  ON, if motor at desired position



//Digital outputs
#define PIN_BUZZER 9
#define PIN_MOTOR_STEP_BACK 10
#define PIN_MOTOR_STEP_FORTH 11
#define PIN_MOTOR_ORIGIN 12
#define PIN_LED 13




//------- Functions definitions -------//

#ifdef LCD_NEWHAVEN

void LCDsetCursor(int Column, byte Line);
void LCDWrite(const char* text);
void LCDclear();

#endif

void SendPulse(int pin, int timeus);                         // Motor pulse function
void ActiveCount1(float timetointerruptus, long Ninterrupt); // Enable Counter 2 with interruption after timetointerrupt second
void StopCount1();

int interrtime1;                        // global variable to reload the counter
long Ninter1 = 0;                       // Number of interrupt
int cnt1 = 0;                           // interruption counter


//----------- User Inputs variables declaration -----------//

float bpm = 0;    // In breath/minute
float IE = 0;   // Inspiration/expiration ratio 1:x
int VT = 0;     // In pulses   //MAY CHANGE
int peep = 5;     // In cmH20
bool start = 1; // ON/OFF switch


//----------- Measured variables declaration -----------//
int pressure = 15; // in cmH2O
int plateau_pressure = 0; // in cmH2O
int plateau_peep = 0; // in cmH2O

//----------- Internal variables declaration -----------//
float T = 0;    // INHALE/EXHALE period in seconds
float Tin = 0;  // INHALE period in seconds
float Tex = 0;  // EXHALE period in seconds
float Vin = 0; // flowrate in pulses/second   //MAY CHANGE
float Vex = 0; // flowrate in pulses/second   //MAY CHANGE
float Psin = 0; // time/pulse  //MAY CHANGE
float Psex = 0; // time/pulse    //MAY CHANGE
long Npulse = 0; // Total cycle pulses    //MAY CHANGE

float Tplateau = 300; // Pause time to measure plateau pressure in millisec
float Tpeep = 50; // Pause time to measure plateau peep in millisec

int state = STOP; // Initial system state machine

byte error_state = STATE_OK; // System error state

bool restarted = 0;

bool first_stop = 1;

bool firststop = 1;

float Tinitmax = 5; // Max time to init before sending an error

bool initialized = 0;

int cnt_alarm = 0;

int cnt_positioning;

bool stop_origin_busy = 0;

//----------- LCD management variable -----------//
int lastbpm = 0;
int lastpressure = 0;
int lastplatpressure = 0;
int lastpeep = 0;
bool laststate = 1;
int blink_error = 0;
bool printerror = 0;

int cntscreenchange = 0;
bool screenshow = 0;
bool oldscreenshow = 1;



#ifdef LCD_2004A
LiquidCrystal_I2C lcd(0x27, 20, 4); //Set the LCD address to 0x27 for a 20 chars and 4 lines display
#endif

#ifdef LCD_NEWHAVEN
const byte LCDa = 0x28; //LCD address on I2C bus
#endif


//----------- Motor driver variable declaration -----------//
long PEEP_relative_origine = 0;
int motorposition = 0;

int savedmotorposition = 0;

//----------- Debug variable declaration -----------//
long timestart = 0;
long interuptcnt = 0;




// PCIF------ Type declaration of runtime data for GEVE Ventilator Monitoring ---------- //
struct GEVE_VeMon_Data
{
  bool on_off;
  bool error_ack;
  double breath_pm;
  double tidal_volume;
  double in_ex_ratio;
  double peep;
  double pressure;
  double pressure_plateau;
  double motor_position;
  bool alarm_p_high;
  bool alarm_p_low;
  bool alarm_init;
};


// PCIF------ Variable definition of type GEVE runtime data for communication ---------- //
GEVE_VeMon_Data geve_status =
{
  false,
  false,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  false,
  false,
  false
};

void setup()
{
  //----------- I/O init. -----------//

  // Input
  pinMode(PIN_SWITCH_ONOFF, INPUT_PULLUP);
  pinMode(PIN_SWITCH_ERR_ACK, INPUT_PULLUP);
  pinMode(PIN_WAREA, INPUT);
  pinMode(PIN_BUSY, INPUT);
  pinMode(PIN_SETON, INPUT);
  pinMode(PIN_INP, INPUT);

  //TIMSK0 = 0;

  //Output
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_MOTOR_STEP_BACK, OUTPUT);
  pinMode(PIN_MOTOR_STEP_FORTH, OUTPUT);
  pinMode(PIN_MOTOR_ORIGIN, OUTPUT);
  pinMode(PIN_LED, OUTPUT);

  digitalWrite(PIN_MOTOR_STEP_BACK, LOW);
  digitalWrite(PIN_MOTOR_STEP_FORTH, LOW);
  digitalWrite(PIN_MOTOR_ORIGIN, HIGH);

  //------------Serial comm. init.------------//
  Serial.begin(9600);

  // ----------- LCD init.------------//
#ifdef LCD_2004A
  lcd.begin();
  lcd.backlight();
  lcd.clear();

#endif

#ifdef LCD_NEWHAVEN

  TWBR = 100000; //sets I2C speed to 100kHz very important
  LCDclear();
  //delay(500);     // for observation only can be removed

#endif

}
int starttimemesure = 0;
void loop()
{
  // ================START OF Read potentiometers & display values ======================//

  bpm = (int)(MAX_BPM - ((float)analogRead(PIN_BPM_IN) * DELTA_BPM / 1024)) ;

  VT = (int)(MAX_VT - ((float)analogRead(PIN_VT_IN) * DELTA_VT / 1024)) ;

  IE = MAX_IE - ((float)(analogRead(PIN_IE_IN)) * DELTA_IE / 1024);

  // read pressure
  pressure = (int)(((float)(analogRead(PIN_PRESSURE_IN) - 512) * PRESSURE_SENSOR_POS_DYNAMIC / 1024) * 5 / 2) ;

  // PCIF -----vvvvv----- comment out to test PC interface ----------
  //Serial.print("Pressure : ");
  //Serial.println(pressure);
  // PCIF -----^^^^^----- end commenting out ----------

  geve_status.breath_pm    = bpm; // <--- PCIF ---
  geve_status.tidal_volume = VT; // <--- PCIF ---
  geve_status.in_ex_ratio  = IE; // <--- PCIF ---




  // ====================== START OF Update display =======================//


  if (screenshow == 0)
  {
    if (oldscreenshow == 1)
    {
#ifdef LCD_2004A
      lcd.setCursor(0, 1);
      lcd.print("BPM:    ");
      lcd.setCursor(8, 1);
      lcd.print("Tid.Vol.:   ");
      lcd.setCursor(0, 2);
      lcd.print("IE:         ");
      lcd.setCursor(12, 2);
      lcd.print("PEEP.:  ");
      lcd.setCursor(0, 3);
      lcd.print("Press:    ");
      lcd.setCursor(10, 3);
      lcd.print("PressPl:  ");
#endif

#ifdef LCD_NEWHAVEN
      LCDsetCursor(0, 1);
      LCDWrite("BPM:");
      LCDsetCursor(8, 1);
      LCDWrite("Tid.Vol.:");
      LCDsetCursor(0, 2);
      LCDWrite("IE:");
      LCDsetCursor(12, 2);
      LCDWrite("PEEP.:");
      LCDsetCursor(0, 3);
      LCDWrite("Press:");
      LCDsetCursor(10, 3);
      LCDWrite("PressPl:");
#endif
    }

    if ((bpm < 10) && (bpm != lastbpm) && (oldscreenshow == 1))
    {
#ifdef LCD_2004A
      lcd.setCursor(4, 1);
      lcd.print("  ");
#endif

#ifdef LCD_NEWHAVEN
      LCDsetCursor(4, 1);
      LCDWrite("  ");
#endif
    }
    lastbpm = bpm;

#ifdef LCD_2004A
    lcd.setCursor(4, 1);
    lcd.print((int)bpm);

    lcd.setCursor(17, 1);
    lcd.print(VT);

    lcd.setCursor(4, 2);
    lcd.print(IE);
#endif

#ifdef LCD_NEWHAVEN
    LCDsetCursor(4, 1);
    LCDWrite((int)bpm);

    LCDsetCursor(17, 1);
    LCDWrite(VT);

    LCDsetCursor(4, 2);
    LCDWrite((char) IE);
#endif

    if ((peep < 10) && (peep != lastpeep))
    {
#ifdef LCD_2004A
      lcd.setCursor(18, 2);
      lcd.print("  ");

#endif
#ifdef LCD_NEWHAVEN


      LCDsetCursor(18, 2);
      LCDWrite("  ");

#endif

    }
    lastpeep = peep;

#ifdef LCD_2004A
    lcd.setCursor(18, 2);
    lcd.print(plateau_peep);
#endif

#ifdef LCD_NEWHAVEN
    LCDsetCursor(18, 2);
    LCDWrite(plateau_peep);
#endif

    if ((pressure < 10) && (pressure != lastpressure))
    {
#ifdef LCD_2004A
      lcd.setCursor(6, 3);
      lcd.print("  ");
#endif

#ifdef LCD_NEWHAVEN
      LCDsetCursor(6, 3);
      LCDWrite("  ");
#endif
    }
    lastpressure = pressure;

#ifdef LCD_2004A
    lcd.setCursor(6, 3);
    lcd.print(pressure);
#endif
#ifdef LCD_NEWHAVEN
    LCDsetCursor(6, 3);
    LCDWrite(pressure);
#endif

#ifdef LCD_2004A
    lcd.setCursor(18, 3);
    lcd.print(plateau_pressure);
#endif
#ifdef LCD_NEWHAVEN
    LCDsetCursor(18, 3);
    LCDWrite(plateau_pressure);
#endif


  }
  else if (oldscreenshow == 0 && screenshow == 1)
  {

#ifdef LCD_2004A
    lcd.setCursor(0, 1);
    lcd.print("Before starting the ");
    lcd.setCursor(0, 2);
    lcd.print("system, make sure to");
    lcd.setCursor(0, 3);
    lcd.print("deflate the testlung");

#endif

#ifdef LCD_NEWHAVEN
    LCDsetCursor(0, 1);
    LCDWrite("                    ");
    LCDsetCursor(0, 2);
    LCDWrite("                    ");
    LCDsetCursor(0, 3);
    LCDWrite("                    ");
#endif

  }
  oldscreenshow = screenshow ;

  geve_status.motor_position = motorposition; // <--- PCIF ---
  geve_status.pressure = pressure; // <--- PCIF ---


  // ====================== END OF Update display =======================//



  // ---------- Check ON/OFF ---------- //
  if (!digitalRead(PIN_SWITCH_ONOFF) == OFF) // Inversé car état bouton physique inversé
  {
    TIMSK2 = 0;
    cnt1 = 0;
    error_state = STATE_OK;
    if (firststop) laststate = 1;
    firststop = 0;
    state = STOP;
    geve_status.on_off = false; // <--- PCIF ---
  }
  else
  {
    firststop = 1;
    geve_status.on_off = true; // <--- PCIF ---
  }


  // ---------- Check ALARMS (pressure) ---------- //
  if ((pressure > MAX_PRESSURE) || (initialized && (pressure < peep)))
  {

    if (pressure > MAX_PRESSURE)
    {
      error_state = STATE_ERR_HIGH_PRESSURE;            // IN CASE OF HIGH PRESSURE, THE SYSTEM STOPS AND GENERATES AN ALARM
      TIMSK2 = 0;
      cnt1 = 0;
      state = STOP;
    }
    if (pressure < peep)
    {
      error_state = STATE_ERR_LOW_PRESSURE;             // IN CASE OF LOW PRESSURE, THE SYSTEM CONTINUE ITS CYCLE, BUT GENERATES AN ALARM
    }
  }


  // ---------- Check ERROR ACK ---------- //
  if (digitalRead(PIN_SWITCH_ERR_ACK) == BUTTON_PUSHED)  // Allow user to acknowledge an occured error and display again system state on LCD
  {
    error_state = STATE_OK;
    geve_status.error_ack = true; // <--- PCIF ---
  }
  else
  {
    geve_status.error_ack = false; // <--- PCIF ---
  }


  // ---------- Check & handle error states ---------- //
  if (error_state)                                      // if an error occured alarm is activated and error message is displayed on LCD
  {
    if (blink_error < 7)
    {
      printerror = 1;
    }
    else if (blink_error < 14)
    {
      printerror = 0;
    }
    else
    {
      blink_error = 0;
    }
    blink_error++;


    // START OF ERROR HANDLING State Machine

    switch (error_state)
    {

      case STATE_ERR_HIGH_PRESSURE :

        if (printerror)
        {

          tone(PIN_BUZZER, ALARM_FREQUENCY, 2000);
          geve_status.alarm_p_high = true; // <--- PCIF ---
#ifdef LCD_2004A
          lcd.setCursor(0, 0);
          lcd.print("HIGH PRESSURE ERROR ");
#endif

#ifdef LCD_NEWHAVEN
          LCDsetCursor(0, 0);
          LCDWrite("HIGH PRESSURE ERROR ");
#endif
        }
        else
        {
          noTone(PIN_BUZZER);
          geve_status.alarm_p_high = false; // <--- PCIF ---
        }

        break;
      // ----- end ERROR case STATE_ERR_HIGH_PRESSURE


      case STATE_ERR_LOW_PRESSURE :

        if (printerror)
        {
          tone(PIN_BUZZER, ALARM_FREQUENCY, 2000);
          geve_status.alarm_p_low  = true; // <--- PCIF ---
#ifdef LCD_2004A
          lcd.setCursor(0, 0);
          lcd.print("LOW PRESSURE ERROR  ");
#endif;
#ifdef LCD_NEWHAVEN
          LCDsetCursor(0, 0);
          LCDWrite("LOW PRESSURE ERROR  ");
#endif;
        }
        else
        {
          noTone(PIN_BUZZER);
          geve_status.alarm_p_low  = false; // <--- PCIF ---
        }

        break;
      // ----- end ERROR case STATE_ERR_LOW_PRESSURE


      case STATE_ERR_FAIL_INIT :

        if (printerror)
        {
          tone(PIN_BUZZER, ALARM_FREQUENCY, 2000);
          geve_status.alarm_init = true; // <--- PCIF ---
#ifdef LCD_2004A
          lcd.setCursor(0, 0);
          lcd.print("INIT. ERROR         ");
#endif;
#ifdef LCD_NEWHAVEN
          LCDsetCursor(0, 0);
          LCDWrite("INIT. ERROR         ");
#endif;
        }
        else
        {
          noTone(PIN_BUZZER);
          geve_status.alarm_init = false; // <--- PCIF ---
        }

        break;
      // ----- end ERROR case STATE_ERR_FAIL_INIT

      case STATE_ERR_MOTOR:

        if (printerror)
        {
          tone(PIN_BUZZER, ALARM_FREQUENCY, 2000);
          geve_status.alarm_init = true; // <--- PCIF ---
#ifdef LCD_2004A
          lcd.setCursor(0, 0);
          lcd.print("MOTOR ERROR         ");
#endif;
#ifdef LCD_NEWHAVEN
          LCDsetCursor(0, 0);
          LCDWrite("MOTOR ERROR         ");
#endif;
        }
        else
        {
          noTone(PIN_BUZZER);
          geve_status.alarm_init = false; // <--- PCIF ---
        }

        break;
        // ----- end ERROR case STATE_ERR_FAIL_INIT




    }
    // END OF ERROR HANDLING State Machine

  }
  else  // No error (error_state = STATE_OK), reset all alarms to false.
  {
    noTone(PIN_BUZZER);
    printerror = 0;
    geve_status.alarm_p_high = false; // <--- PCIF ---
    geve_status.alarm_p_low  = false; // <--- PCIF ---
    geve_status.alarm_init   = false; // <--- PCIF ---
  }



  // END OF Read potentiometers & display values =====================================================


  // START OF System state Machine

  switch (state)
  {
    case STOP :

      initialized = 0;
      if (digitalRead(PIN_INP) || digitalRead(PIN_BUSY))
      {
        digitalWrite(PIN_MOTOR_ORIGIN, 0); // open system
      }
      else
      {
        digitalWrite(PIN_MOTOR_ORIGIN, 1); // open system

      }

      if (laststate && !printerror)
      {
        first_stop = 1;
      }
      if (!printerror)
      {
#ifdef LCD_2004A
        //lcd.setCursor(0, 0);
        //lcd.print("                    ");
        lcd.setCursor(0, 0);
        lcd.print("STOP               ");
#endif;
#ifdef LCD_NEWHAVEN
        LCDsetCursor(0, 0);
        LCDWrite("                    ");
        LCDsetCursor(0, 0);
        LCDWrite("STOP                ");
#endif;
      }
      // PCIF -----vvvvv----- comment out to test PC interface ----------
      //Serial.println("Stop system");
      // PCIF -----^^^^^----- end commenting out ----------

      laststate = 0;


      if (!digitalRead(PIN_SWITCH_ONOFF)) // Inversé car état bouton physique inversé
      {
        screenshow = 0;
        cntscreenchange = 0;

        if (first_stop) ActiveCount1(6000, 3000);
        first_stop = 0;
        if (digitalRead(PIN_INP) == true) //waiting on system to be open
        {
          state = SET_INIT;
          laststate = 1;
        }
      }
      else
      {
        delay(5);
        if (cntscreenchange >= 100) // chaque 5 secondes - change d'affichage
        {
          cntscreenchange = 0;
          screenshow = !screenshow;
        }
        else
        {
          cntscreenchange++;
        }
      }



      // PCIF ----- Should the ON/OFF variable be changed here instead of the 'Check ON/OFF' part?

      break;
    // ----- end case STOP -----


    case SET_INIT : // ------ System squeeze bag until PEEP reached, if not at 30% course => alarm -------

      if (!printerror)
      {
#ifdef LCD_2004A
        lcd.setCursor(0, 0);
        lcd.print("SET INITIALIZATION ");
#endif;
#ifdef LCD_NEWHAVEN
        LCDsetCursor(0, 0);
        LCDWrite("SET INITIALIZATION ");
#endif;
      }
      if (digitalRead(PIN_INP) == true)
      {
        digitalWrite(PIN_MOTOR_ORIGIN, 0); // open system
      }

      if ((digitalRead(PIN_INP) == true) && (digitalRead(PIN_SETON) == true) ) //waiting on system to be open
      {
        digitalWrite(PIN_MOTOR_ORIGIN, 1);
        motorposition = 0;
        state = INIT;
      }

      break;
    // ----- end case SET_INIT -----


    case INIT :

      if (!printerror)
      {
#ifdef LCD_2004A
        lcd.setCursor(0, 0);
        lcd.print("INITIALIZATION     ");
#endif;
#ifdef LCD_NEWHAVEN
        LCDsetCursor(0, 0);
        LCDWrite("INITIALIZATION    ");
#endif;
      }

      // PCIF -----vvvvv----- comment out to test PC interface ----------
      //Serial.println(MOTOR_POSITION_LIMIT_INIT_PULSE);
      // PCIF -----^^^^^----- end commenting out ----------

      if ((motorposition < MOTOR_POSITION_LIMIT_INIT_PULSE) && (pressure < (peep + PEEP_MARGIN)))
      {
        if (digitalRead(PIN_INP) == true) // if motor took desired position
        {
          for (int j = 0; j < 100; j++)
          {
            SendPulse(PIN_MOTOR_STEP_FORTH, MOTOR_PULSE_WIDTH);
            motorposition++;
            delayMicroseconds(500);
          }
        }
      }

      else if (pressure >= (peep + PEEP_MARGIN))
      {
  
        PEEP_relative_origine = motorposition;

        initialized = 1;
        motorposition = 0;
        StopCount1();
        state = SET_INHALE;
      }
      else
      {
        initialized = 0;
        StopCount1();
        state = STOP;
        error_state = STATE_ERR_FAIL_INIT;
      }

      break;
    // ----- end case INIT -----


    case SET_INHALE :

      // PCIF -----vvvvv----- comment out to test PC interface ----------

      if (digitalRead(PIN_INP) == true) {
        cnt_positioning = 0;
        //Serial.print("Expiration time : ");
        //Serial.println(millis() - timestart);
        //Serial.print("Motor position after expiration: ");
        //Serial.println(motorposition);
        // PCIF -----^^^^^----- end commenting out ----------

        interuptcnt = 0;

        // TO BE DISCUSS : does the rythm variables must change only at the end of a cycle (better for motor control?) ?

        // Rythm variable update
        T = 60 / bpm;
        Tin = T / (1 + IE);
        Tex = T - Tin;

        Vin = VT / (Tin * VOLUME_PER_PULSE) ;
        Vex = VT / (Tex * VOLUME_PER_PULSE);

        Npulse = Vin * Tin;

        Psin = 1000000 / Vin;
        Psex = 1000000 / Vex;


        // set motor speed Vin -----------------

        // PCIF -----vvvvv----- comment out to test PC interface ----------
        //Serial.println("INHALATION");
        // PCIF -----^^^^^----- end commenting out ----------

        if (!printerror)
        {
#ifdef LCD_2004A
          lcd.setCursor(0, 0);
          lcd.print("INHALATION         ");
#endif
#ifdef LCD_NEWHAVEN
          LCDsetCursor(0, 0);
          LCDWrite("INHALATION         ");
#endif
        }


        // Set Tin timer

        ActiveCount1(Psin, Npulse);
        timestart = millis();
        digitalWrite(PIN_LED, HIGH);
        if (state == SET_INHALE) state = INHALE;
      }
      else
      {

        if (++cnt_positioning > 100) {
          state = STOP;
          error_state = STATE_ERR_MOTOR;
          cnt_positioning = 0;
        }
      }

      break;
    // ----- end case SET_INHALE -----



    case INHALE : //INHALE phase

      if (!printerror)
      {
#ifdef LCD_2004A
        lcd.setCursor(0, 0);
        lcd.print("INHALATION         ");
#endif
#ifdef LCD_NEWHAVEN
        LCDsetCursor(0, 0);
        LCDWrite("INHALATION        ");
#endif
      }

      break;
    // ----- end case INAHLE -----


    case SET_PAUSE1 : //Set Plateau pressure measurement

      // PCIF -----vvvvv----- comment out to test PC interface ----------
      //Serial.print("Inhalation time : ");
      //Serial.println(millis() - timestart);
      //Serial.print("Motor position after inspiration: ");
      //Serial.println(motorposition);
      // PCIF -----^^^^^----- end commenting out ----------

      interuptcnt = 0;


      ActiveCount1((Tplateau) * 10, 100);

      timestart = millis();

      if (state == SET_PAUSE1) state = PAUSE1;

      break;
    // ----- end case SET_PAUSE1 -----




    case PAUSE1 :

      plateau_pressure = pressure;
      geve_status.pressure_plateau = pressure; // <--- PCIF ---
#ifdef LCD_2004A
      lcd.setCursor(18, 3);
      lcd.print("  ");
      lcd.setCursor(18, 3);
      lcd.print(plateau_pressure);
#endif
#ifdef LCD_NEWHAVEN
      LCDsetCursor(18, 3);
      LCDWrite("  ");
      LCDsetCursor(18, 3);
      LCDWrite(plateau_pressure);
#endif

      break;
    // ----- end case PAUSE1 -----


    case SET_EXHALE :

      if (digitalRead(PIN_INP) == true) {
        cnt_positioning = 0;
        // PCIF -----vvvvv----- comment out to test PC interface ----------
        //Serial.print("Pause time : ");
        //Serial.println(millis() - timestart);
        // set motor speed Vin-----------------
        //Serial.println("EXPIRATION");
        // PCIF -----^^^^^----- end commenting out ----------

        // Set Tin timer
        ActiveCount1(Psex, Npulse);
        timestart = millis();
        if (!printerror)
        {
#ifdef LCD_2004A
          lcd.setCursor(0, 0);
          lcd.print("EXPIRATION         ");
#endif
#ifdef LCD_NEWHAVEN
          LCDsetCursor(0, 0);
          LCDWrite("EXPIRATION         ");
#endif
        }
        digitalWrite(PIN_LED, LOW);
        if (state == SET_EXHALE) state = EXHALE;
      }
      else
      {

        if (++cnt_positioning > 100) {
          state = STOP;
          error_state = STATE_ERR_MOTOR;
          cnt_positioning = 0;
        }
      }

      break;
    // ----- end case SET_EXHALE -----


    case EXHALE :

      if (!printerror)
      {
#ifdef LCD_2004A
        lcd.setCursor(0, 0);
        lcd.print("EXPIRATION         ");
#endif
#ifdef LCD_NEWHAVEN
        LCDsetCursor(0, 0);
        LCDWrite("EXPIRATION         ");
#endif
      }

      break;
    // ----- end case EXHALE -----

    case SET_PAUSE2 : //Set Plateau pressure measurement

      // PCIF -----vvvvv----- comment out to test PC interface ----------
      //Serial.print("Inhalation time : ");
      //Serial.println(millis() - timestart);
      //Serial.print("Motor position after inspiration: ");
      //Serial.println(motorposition);
      // PCIF -----^^^^^----- end commenting out ----------

      interuptcnt = 0;

      ActiveCount1((Tpeep) * 10, 100);

      timestart = millis();

      if (state == SET_PAUSE2) state = PAUSE2;

      break;
    // ----- end case SET_PAUSE2 -----


    case PAUSE2 :

      plateau_peep = pressure;
      geve_status.peep = pressure; // <--- PCIF ---
#ifdef LCD_2004A
      lcd.setCursor(18, 2);
      lcd.print("  ");
      lcd.setCursor(18, 2);
      lcd.print(plateau_peep);
#endif
#ifdef LCD_NEWHAVEN
      LCDsetCursor(18, 2);
      LCDWrite("  ");
      LCDsetCursor(18, 2);
      LCDWrite(plateau_peep);
#endif

      break;
      // ----- end case PAUSE2 -----
  }
  // END OF State Machine




  //
  // PCIF------ Communicate status to PC interface ---------- //

  Serial.print(geve_status.on_off);
  Serial.print("_");
  Serial.print(geve_status.error_ack);
  Serial.print("_");
  Serial.print(geve_status.breath_pm);
  Serial.print("_");
  Serial.print(geve_status.tidal_volume);
  Serial.print("_");
  Serial.print(geve_status.in_ex_ratio);
  Serial.print("_");
  Serial.print(geve_status.peep);
  Serial.print("_");
  Serial.print(geve_status.pressure);
  Serial.print("_");
  Serial.print(geve_status.pressure_plateau);
  Serial.print("_");
  Serial.print(geve_status.motor_position);
  Serial.print("_");
  Serial.print(geve_status.alarm_p_high);
  Serial.print("_");
  Serial.print(geve_status.alarm_p_low);
  Serial.print("_");
  Serial.print(geve_status.alarm_init);

  Serial.println();
  //

}


void ActiveCount1(float timetointerruptus, long Ninterrupt)
{

  cli();
  cnt1 = 0;
  Ninter1 = Ninterrupt;
  bitClear (TCCR1A, WGM10); // WGM20 = 0
  bitClear (TCCR1A, WGM11); // WGM21 = 0
  TCCR1B = 0b00000010; // Clock / 8 soit 0.5 micro-s et WGM22 = 0
  TIMSK1 = 0b00000001; // Interruption locale autorisée par TOIE2
  interrtime1 = 65536 - timetointerruptus * 2;
  TCNT1 = interrtime1;
  //cnt_time =timetointerrupt*31250;
  sei(); // Active l'interruption globale
}

void StopCount1()
{
  TIMSK1 = 0;
}


ISR(TIMER1_OVF_vect)
{
  TCNT1  = interrtime1;
  if ((state == INHALE) || (state == SET_INHALE)) {
    SendPulse(PIN_MOTOR_STEP_FORTH, MOTOR_PULSE_WIDTH);
    motorposition++;
  };
  if (state == EXHALE || (state == SET_EXHALE)) {
    SendPulse(PIN_MOTOR_STEP_BACK, MOTOR_PULSE_WIDTH);
    motorposition--;
  };
  if (cnt1++ == (Ninter1 - 1))
  {

    cnt1 = 0;
    TIMSK1 = 0;

    switch (state)
    {

      case STOP:
        error_state = STATE_ERR_FAIL_INIT;

        break;

      case SET_INIT: state = STOP;
        error_state = STATE_ERR_FAIL_INIT;

        break;

      case INIT: state = STOP;
        error_state = STATE_ERR_FAIL_INIT;
        break;

      case INHALE: state = SET_PAUSE1;

        break;

      case EXHALE: state = SET_PAUSE2;

        break;

      case PAUSE1: state = SET_EXHALE;

        break;

      case PAUSE2: state = SET_INHALE;

        break;

    }

  }

}

void SendPulse(int pin, int timeus) {

  digitalWrite(pin, LOW);
  digitalWrite(pin, HIGH);
  delayMicroseconds(timeus);
  digitalWrite(pin, LOW);

}


#ifdef LCD_NEWHAVEN

// Clear the NHD Extended LCD Screen
void LCDclear()
{
  Wire.beginTransmission(LCDa);
  Wire.write (0xFE);
  Wire.write(0x51);
  Wire.endTransmission();
  delay(2);
}



void LCDWrite(const char* text)
{
  Wire.beginTransmission(LCDa);
  Wire.write(text);
  Wire.endTransmission();
}

void LCDsetCursor(int Column, byte Line)
{
  int pos = 0;
  switch (Line)
  {
    case 1: pos |= 0x40; break;
    case 2: pos |= 0x14; break;
    case 3: pos |= 0x54; break;
  }

  Wire.beginTransmission(LCDa);
  Wire.write (0xFE);
  Wire.write(0x45);
  Wire.write(pos + Column);
  Wire.endTransmission();
  delay(2);
}


#endif
