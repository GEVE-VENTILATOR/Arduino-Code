// ---------- Type declaration of runtime data for GEVE Ventilator Monitoring ---------- //
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


// ---------- Variable definition of type GEVE runtime data for communication ---------- // 
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


// ---------- Simulate some input and measurement values ---------- //
// So far I have two potentiometers and a button connected.
int normPoti   = A0;
int heliPoti   = A3;
int ACK_BUTTON = 10;

void setup()
{
  pinMode(ACK_BUTTON, INPUT_PULLUP);
  // Initialize pseudo random number generator from unconnected analog pin noise.
  randomSeed(analogRead(1));
  // Set up Serial library at 9600 Baud to communicate status messages to and from PC.
  Serial.begin(9600);
}


void loop()
{  
  // Fill the GEVE data structure with test values for now to send.
  geve_status.on_off           = true;
  geve_status.error_ack        = !digitalRead(ACK_BUTTON);
  geve_status.breath_pm        = analogRead(normPoti);
  geve_status.tidal_volume     = analogRead(heliPoti);
  geve_status.in_ex_ratio      = random(10)       + ( random(0,  100) /  100.0 );
  geve_status.peep             = random(150, 200) + ( random(0,  100) /  100.0 );
  geve_status.pressure         = random(1000)     + ( random(0, 1000) / 1000.0 );
  geve_status.pressure_plateau = random(1000)     + ( random(0, 1000) / 1000.0 );  
  geve_status.motor_position   = random(1000)     + ( random(0, 1000) / 1000.0 );  
  geve_status.alarm_p_high = false;
  geve_status.alarm_p_low  = false;
  if ( geve_status.breath_pm > 512 )
  {
    geve_status.alarm_init = true;
  }
  else
  {
    geve_status.alarm_init = false;
  }

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
  
  // Send a new data point only every 100 ms.
  delay(100);
}
