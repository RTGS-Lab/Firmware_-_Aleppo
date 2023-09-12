#include <stdlib.h>

#define REC_EN 3 //Active LOW
#define FOUT 4
#define SEND_EN 2 //Active HIGH

const uint8_t TX_SDI12 = 1;
const uint8_t RX_SDI12 = 0;

// SDI-12 constants 
const unsigned long BREAK_MIN = 7;
const int MARKING_PERIOD = 9; //>8.33ms for standard marking period

// Flags
volatile bool breakEvent = false; //Used by the ISR to communicate a marking event

uint8_t sdi12ADR = 0; //Default to address 0
const String COMPATABILITY_NUMBER = "13"; //Compattible with SDI-12 version v1.3
const String COMPANY_NAME = "GEMS    "; //Name of sensor mgf
const String MODEL_NUMBER = "ALEPPO"; //Model number of the device 
const String SENSOR_VERSION = "0.0"; //Sensor version number
const String SERIAL_NUMBER = "123"; //FIX! Read in from EEPROM!

void setup() {
	
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  pinMode(REC_EN, OUTPUT); //DIR
  pinMode(FOUT, OUTPUT); //FOUT
  pinMode(SEND_EN, OUTPUT);
  digitalWrite(SEND_EN, LOW);
  digitalWrite(FOUT, HIGH);
  digitalWrite(REC_EN, LOW);
  Serial.println("BEGIN!");
  attachInterrupt(digitalPinToInterrupt(RX_SDI12), detectBreak, CHANGE);
}

void loop() {
	static int readLength = 0;
	static String dValues[10];  // 10 String objects to hold the responses to aD0!-aD9! commands
	String readString;
	char readArray[25] = {0};
  	
  	char Input = NULL;
  	if(breakEvent) {
  		breakEvent = false; //Clear flag
  		detachInterrupt(digitalPinToInterrupt(RX_SDI12)); //Turn off interrupt while transmission is happening 
  		Serial.println("BREAK DETECTED"); //DEBUG!
  		
  		
  		Serial1.begin(1200, SERIAL_7E1);
		while(Input != '!') { //FIX! ADD TIMEOUT!
	        if(Serial1.available() > 0) {
	          Input = Serial1.read();
	          Serial.println(Input, HEX); //DEBUG!
	          if(Input != '!') { //Wait for return
	            readArray[readLength] = Input;
	          readLength++;
	        }
	        if(Input == '!') {
	          readString = String(readArray);
	          readString.trim();
	          memset(readArray, 0, sizeof(readArray));
	          readLength = 0;

	          Serial.print(">");
	          Serial.println(readString); //Echo back to serial monitor
	          delay(3); //Make sure bus has been released
	          mark(MARKING_PERIOD);
	          Serial1.begin(1200, SERIAL_7E1);
	          if(readString.charAt(0) == '?') Serial1.print(sdi12ADR); //Respond to address query
	          if(readString.toInt() == sdi12ADR) { //If address matches 
	          	switch(readString.charAt(1)) {
	          		case 'I':
	          			Serial1.print(identification()); //Return identification string
	          			break;
      				case 'A':
      					sdi12ADR = readString.charAt(2) - '0'; //Convert new address to integer //FIX! Test first, write to EEPROM
      					break;

	          	}
	          }
	          Serial1.flush();
	          delay(10); //DEBUG! Return to 1ms??
		        unsigned long localTime = millis();
		        while(Serial1.available() > 0 && (millis() - localTime) < 10) Serial1.read(); //Clear buffer, read for at most 10ms //DEBUG!
	          releaseBus();
	        }
	        }

		}
		attachInterrupt(digitalPinToInterrupt(RX_SDI12), detectBreak, CHANGE); //Restore interrupt while waiting for the next event 
	}
}

void releaseBus() 
{
	pinMode(REC_EN, OUTPUT); //DEBUG!
	digitalWrite(SEND_EN, LOW); //Turn off transmit 
	digitalWrite(REC_EN, LOW); //Set direction to inpout
}

void mark(unsigned long period)
{
	pinMode(REC_EN, OUTPUT); //DEBUG!
	pinMode(SEND_EN, OUTPUT); 
	digitalWrite(TX_SDI12, 1); //Preempt value before turning output on
	delay(1);
	pinMode(TX_SDI12, OUTPUT); //Make sure transmit pin is set as output
	digitalWrite(SEND_EN, HIGH); //Set direction to output
	digitalWrite(REC_EN, HIGH); //Set direction to output
	digitalWrite(TX_SDI12, 1); //Begin marking condition
	delay(period); //Wait for a given marking period
}

void detectBreak()
{
	static unsigned long lowPeriod = 0; //Keep track of how long the low pulse has been
	static unsigned long fallingEdge = 0; //Keep track of when the falling edge occored 
	int val = digitalRead(RX_SDI12);
	if(val == LOW) fallingEdge = millis(); //Start on falling edge
	if(val == HIGH) {
		lowPeriod = millis() - fallingEdge;
		if(lowPeriod > BREAK_MIN) breakEvent = true; //If pulse is within range, set flag
	}
}

String identification()
{
	return String(sdi12ADR) + COMPATABILITY_NUMBER + COMPANY_NAME + MODEL_NUMBER + SENSOR_VERSION + SERIAL_NUMBER;
}

