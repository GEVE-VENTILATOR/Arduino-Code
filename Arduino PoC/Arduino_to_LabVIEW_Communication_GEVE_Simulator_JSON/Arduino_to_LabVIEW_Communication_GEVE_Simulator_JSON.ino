#include <ArduinoJson.h>
#include <SoftwareSerial.h>

const byte rxPin = 2;
const byte txPin = 3;

// set up a new serial object
SoftwareSerial mySerial (rxPin, txPin);


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
  bool alarm;
};
// The struct should look like this in JSON format:
/*
{
  "On/Off": false,
  "Error Ack": false,
  "bpm": 0.0,
  "VT": 0.0,
  "IE": 0.0,
  "PEEP": 0.0,
  "Pressure": 0.0,
  "Plateau Pressure": 0.0,
  "Alarm":false
}
*/

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
  false
};


// ---------- Simulate some input and measurement values ---------- //
// So far I have two potentiometers and a button connected.
int normPoti   = A0;
int heliPoti   = A3;
int ACK_BUTTON = 10;

StaticJsonDocument<200> doc;

void setup()
{ 
  pinMode(ACK_BUTTON, INPUT_PULLUP);
  // Initialize pseudo random number generator from unconnected analog pin noise.
  randomSeed(analogRead(1));
  // Set up Serial library at 9600 Baud to communicate status messages to and from PC.
  Serial.begin(9600);
  mySerial.begin(9600);
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
  if ( geve_status.breath_pm > 512 )
  {
    geve_status.alarm = true;
  }
  else
  {
    geve_status.alarm = false;
  }



  
  //
  // Mirror the communication data structure into the JSON document.
  doc["On/Off"]           = geve_status.on_off;
  doc["Error Ack"]        = geve_status.error_ack;
  doc["bpm"]              = geve_status.breath_pm;
  doc["VT"]               = geve_status.tidal_volume;
  doc["IE"]               = geve_status.in_ex_ratio;
  doc["PEEP"]             = geve_status.peep;
  doc["Pressure"]         = geve_status.pressure;
  doc["Plateau Pressure"] = geve_status.pressure_plateau;  
  doc["Alarm"]            = geve_status.alarm;
  //
  
  
  // DEBUG: Start the stopwatch to measure the time it takes to send the data out over the serial port.
  unsigned long StartTime = millis();


  //
  // Generate the minified JSON and send it to the Serial port.
  serializeJson(doc, Serial);
  // This bit of code takes 73 ms for execution.
  //
  
  
  /*
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
  Serial.print(geve_status.alarm);
  Serial.println();
  */

  /*
  mySerial.print(geve_status.on_off);
  mySerial.print("_");
  mySerial.print(geve_status.error_ack);
  mySerial.print("_");
  mySerial.print(geve_status.breath_pm);  
  mySerial.print("_");
  mySerial.print(geve_status.tidal_volume);
  mySerial.print("_");
  mySerial.print(geve_status.in_ex_ratio);
  mySerial.print("_");
  mySerial.print(geve_status.peep);
  mySerial.print("_");
  mySerial.print(geve_status.pressure);  
  mySerial.print("_");
  mySerial.print(geve_status.pressure_plateau);  
  mySerial.print("_");
  mySerial.print(geve_status.alarm);
  mySerial.println();
  */

  
  // DEBUG: Stop the stopwatch to measure the time it takes to send the data out over the serial port.
  unsigned long CurrentTime = millis();
  unsigned long ElapsedTime = CurrentTime - StartTime;
  
  Serial.println(ElapsedTime);
  
  // Normal Serial:            4 ms
  // Software Serial:         50 ms
  // JSON over normal Serial: 75 ms

  
  // Send a new data point only every 100 ms.
  delay(100);

  // Measured in LabVIEW, This takes about 180 ms!!!
}
