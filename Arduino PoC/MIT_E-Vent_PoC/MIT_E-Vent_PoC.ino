/* Changelog:
 *  2020-03-27: Quick Proof-of-Concept using copy-pasted examples
 * 
 * Material:
 * - Arduino Nano (Controller) 
 * - LCD 1602 Display 
 * - LM2596S DC-DC Converter (12 V to 5 V max 3 A)
 * - ULN2803A Darlington Transistor Arrays (12 V Switching)
 * - BMP085 Pressure Sensor (deprecated, need replacement)
 * - Keyes L298 Motor Driver (2 DC or 1 Stepper)
 * - Potentiometer, small parts, etc. 
 */

#include <Wire.h>
#include <Adafruit_BMP085.h>

/*************************************************** 
  This is an example for the BMP085 Barometric Pressure & Temp Sensor

  Designed specifically to work with the Adafruit BMP085 Breakout 
  ----> https://www.adafruit.com/products/391

  These displays use I2C to communicate, 2 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

// Connect VCC of the BMP085 sensor to 3.3V (NOT 5.0V!)
// Connect GND to Ground
// Connect SCL to i2c clock - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 5
// Connect SDA to i2c data - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 4
// EOC is not used, it signifies an end of conversion
// XCLR is a reset pin, also not used here

Adafruit_BMP085 bmp;

int sensorPin = A0;    // select the input pin for the potentiometer
int ledPin = 13;       // select the pin for the LED
int sensorValue = 0;   // variable to store the value coming from the sensor

double temperatureValue    = 0;
double pressureValue_Pa    = 0;
double pressureValue_mmH2O = 0;

// constants won't change. Used here to set a pin number:
const int pin_hiPwr_LED =  6;// the number of the LED pin

// Variables will change:
int ledState = LOW;             // ledState used to set the LED

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change:
const long interval = 1000;           // interval at which to blink (milliseconds)

// include the library code:
#include <LiquidCrystal.h>

// connect motor controller pins to Arduino digital pins
// motor one
int enA = 7;
int in1 = 8;
int in2 = 9;

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {

  // set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  pinMode(ledPin, OUTPUT);   // declare the ledPin as an OUTPUT
  pinMode(pin_hiPwr_LED, OUTPUT);   // declare the ledPin as an OUTPUT
  
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  
  Serial.begin(9600);
  Serial.println("Serial communication works.");
  if (!bmp.begin())
  {
	  Serial.println("Could not find a valid BMP085 sensor, check wiring!");
	  while (1) {}
  }
  else
  {
    Serial.println("Reading from sensor...");
  }
}
  
void loop() {
  
    sensorValue = analogRead(sensorPin);
    Serial.print("Poti Value = ");
    Serial.print(sensorValue);
    Serial.println();
    
    temperatureValue = bmp.readTemperature();
    Serial.print("Temperature = ");
    Serial.print(temperatureValue);
    Serial.println(" *C");

    pressureValue_Pa = bmp.readPressure();
    // 1 Pa = 0.10197162129779 mmH2O
    pressureValue_mmH2O = pressureValue_Pa * 0.10197162129779;
    Serial.print("Pressure = ");
    Serial.print(pressureValue_Pa);
    Serial.print(" Pa");
    Serial.print(" equals ");
    Serial.print(pressureValue_mmH2O);
    Serial.println(" mmH2O");
    
    // Calculate altitude assuming 'standard' barometric
    // pressure of 1013.25 millibar = 101325 Pascal
    Serial.print("Altitude = ");
    Serial.print(bmp.readAltitude());
    Serial.println(" meters");

    Serial.print("Pressure at sealevel (calculated) = ");
    Serial.print(bmp.readSealevelPressure());
    Serial.println(" Pa");

  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
    Serial.print("Real altitude = ");
    Serial.print(bmp.readAltitude(101500));
    Serial.println(" meters");
    
    Serial.println();

    if ( sensorValue < 333 )
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("P: ");
      lcd.print(pressureValue_mmH2O);
      lcd.print(" mmH2O");
    }
    else if ( ( sensorValue >= 333 ) && ( sensorValue < 666 ) )
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("T: ");
      lcd.print(temperatureValue);
      lcd.print(" *C");
    }
    else if ( sensorValue >= 666 )
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("R: ");
      lcd.print(sensorValue);
      lcd.print("");
    }

    delay(50);
    
  // check to see if it's time to blink the LED; that is, if the difference
  // between the current time and last time you blinked the LED is bigger than
  // the interval at which you want to blink the LED.
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= sensorValue) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(pin_hiPwr_LED, ledState);

      // this function will run the motors in both directions at a fixed speed
    // turn on motor A
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    // set speed to 200 out of possible range 0~255
    analogWrite(enA, sensorValue/4);
  }
}
