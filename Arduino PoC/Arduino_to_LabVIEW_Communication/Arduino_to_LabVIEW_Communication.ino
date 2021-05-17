// Define how much serial data we expect before a newline.
const unsigned int MAX_INPUT = 128;


// ---------- Processes INCOMING serial data after a terminator was received. ---------- //
void process_data(const char * data)
{
  
  String inData = String(data);
  
  // DEBUG: Echo the command that was received back as a confirmation message.
  // DEBUG: Serial.print("Received: ");
  // DEBUG: Serial.println(inData);
  
  // Process INCOMING data here.
  
}


// ---------- Build a line from incoming data before processing it. ---------- //
void process_incoming_byte(const byte inByte)
{
  static char         input_line[MAX_INPUT];
  static unsigned int input_pos = 0;

  switch (inByte)
  {
    // Mark end of line when termination character was received.      
    case '\n':
      // Set the terminating null byte for string.
      input_line[input_pos] = 0;
      
      // Process the completed input line.
      process_data(input_line);
      
      // Reset buffer for next communication.
      input_pos = 0;  
      break;
      
    // Discard carriage return character.
    case '\r':
      break;

    // Concatenate any other character received to the data string.
    default:
      // Keep adding to the string if not full. Allow for terminating null byte.
      if ( input_pos < ( MAX_INPUT - 1 ) )
      {
        input_line[input_pos++] = inByte;
      }
      break;  
  }
}





// ---------- TEST: Serial write anything ----------
template <typename T> unsigned int Serial_writeAnything (const T& value)
{
  const byte * p = (const byte*) &value;
  unsigned int i;
  for (i = 0; i < sizeof value; i++)
  {
    Serial.write(*p++);
  }
  Serial.write('\r');
  Serial.write('\n');
  Serial.flush();
  return i;
}
// End of Serial_writeAnything





// ---------- Type declaration of runtime data for GEVE status ---------- //
struct GEVE_Runtime_Data {
  unsigned long iterations;    // 32 bit, 4 bytes, Max range: 0 ... 4294967295, Expected range: 0 ... 4294967295
  int poti_setting;            // 16 bit, 2 bytes, Max range: -32768 ... 32767, Expected range: 0 ... 1023
  double pressure_measurement; // 32 bit, 4 bytes, Max range: ..., Expected range: ...
  bool alarm;                  //  8 bit, 1 byte,  Can be 0 or any other value (typically 1)
};


// ---------- Variable definition of type GEVE runtime data for communication ---------- // 
GEVE_Runtime_Data geve_status =
{
  0,
  0,
  0.0,
  false
};


// Specify which analog pin the potentiometer is connected to.
int sensorPin = A0;


void setup() {
  
  // Initialize pseudo random number generator from unconnected analog pin noise.
  randomSeed(analogRead(1));
  
  // Set up Serial library at 9600 Baud to communicate status messages to and from PC.
  Serial.begin(9600);
  // DEBUG: Serial.println("Serial communication established!");

}


// Define some runtime variables for debugging.
unsigned long iteration_counter = 0;
unsigned long serial_buffer_remaining = 0;


void loop() {
  
  // If serial data is available, process it.
  while ( Serial.available() > 0 ) {
    process_incoming_byte(Serial.read());
  }

  // Fill the GEVE data structure with test values for now to send.
  geve_status.iterations = iteration_counter;
  geve_status.poti_setting = analogRead(sensorPin);
  geve_status.pressure_measurement = random(1000) + ( random(0, 1000) / 1000.0 );
  if ( geve_status.poti_setting > 512 )
  {
      geve_status.alarm = true;
  }
  else
  {
    geve_status.alarm = false;
  }

  serial_buffer_remaining = Serial.availableForWrite();
  //if ( serial_buffer_remaining >= sizeof(geve_status) )
  //{
    // Sending a human-readable string for now.
    // TODO: This should be replaced soon with a JSON string (use ArduinoJSON library).
    
    // DEBUG:
    //Serial.print(" -=");
    //Serial.print(Serial.availableForWrite());
    //Serial.print("=- ");
    
    Serial.print(geve_status.iterations);
    Serial.print("_");
    Serial.print(geve_status.poti_setting);
    Serial.print("_");
    Serial.print(geve_status.pressure_measurement);  
    Serial.print("_");
    Serial.print(geve_status.alarm);

    // DEBUG:
    //Serial.print(" -=");
    //Serial.print(Serial.availableForWrite());
    //Serial.print("=- ");
    
    Serial.println();
    //Serial.flush();
    
    //Serial_writeAnything(geve_status);
  //}

  // Send a new data point only every 100 ms.
  delay(100);
  
  // Perform other actions here.

  iteration_counter = iteration_counter + 1;
  
}
