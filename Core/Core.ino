#include <stdlib.h>
#include <PID_v1.h>
#include <Wire.h>
#include <SparkFun_FS3000_Arduino_Library.h> 
#include <PCAL9535A.h>
#include <NHB_AD7124.h>
#include <STS31.h>
#include <PAC1720.h>
#include "AT24MAC402.h"
#include <VEML6030.h>

AT24MAC402 eeprom; // all address pins to GND
uint8_t mac[6];
uint8_t uuid[16];
const uint16_t ctrlOffset = 0;
const uint16_t calibOffset = 32;
const uint16_t factoryOffset = 128; 

// SYSTEM
PCAL9535A IO;
const uint8_t csPin = 10;
const int filterSelBits = 384; //Turn filter on max - sacrifice rate for resolution 
Ad7124 adc(csPin, 1000000);
PAC1720 csa;
VEML6030 als;

const uint8_t I2C_OB_EN = 5;

enum pinsIO {
	PWR_A_FAULT = 0,
	PWR_B_FAULT = 3,
	I2C_A_EN = 2,
	I2C_B_EN = 5,
	PWR_A_EN = 1,
	PWR_B_EN = 4,
	FIVEV_EN = 7,
	CALIB = 8,
	FACTORY_RST = 9,
	FIVEV_PG =10,
	CSA_INT = 11,
	TOF_EN = 12,
	TOF_INT = 13,
	WP = 14,
	ACCEL_INT = 15
};

enum pins {
	ADC_EN = 6,
	FAN_A_EN = 8,
	FAN_B_EN = 9,
	FAN_A_TACH = 17,
	FAN_B_TACH = 18,
	FAN_A_FAULT = 15,
	FAN_B_FAULT = 16,
	GEN_INT = 19,
	FAULT_LED = 7,
	GEN_STAT_LED = 13,
	CALIBRATE_LED =14
};

#define REC_EN 3 //Active LOW
#define FOUT 4
#define SEND_EN 2 //Active HIGH

const uint8_t TX_SDI12 = 1;
const uint8_t RX_SDI12 = 0;

// SDI-12 constants 
const unsigned long BREAK_MIN = 7;
const int MARKING_PERIOD = 9; //>8.33ms for standard marking period
const unsigned long breakTimeout = 1000; //Wait as most 1 second for break to be resolved 

// Flags
volatile bool breakEvent = false; //Used by the ISR to communicate a marking event. Set by ISR, cleared in main loop
volatile bool calibrateEvent = false; //Used by the ISR to comunicate a calibration event. Set by IST, cleared in main loop

uint8_t sdi12ADR = 0; //Default to address 0
const String COMPATABILITY_NUMBER = "13"; //Compattible with SDI-12 version v1.3
const String COMPANY_NAME = "GEMS    "; //Name of sensor mgf
const String MODEL_NUMBER = "ALEPPO"; //Model number of the device 
const String SENSOR_VERSION = "0.0"; //Sensor version number //FIX! read from EEPROM
const String SERIAL_NUMBER = "123"; //FIX! Read in from EEPROM!

// Analog performance 
const float avgPeriod = 5; //Number of seconds to average value over 

// Radiation shield params and PID control
struct fanInfo{
	float speed; //RPM speed
	float current; //Current, mA
	float airflow; //Airflow, m/s
	float temp; //Aux temp, °C
	float pwm; //Commanded PWM value for fan, 0-100
};

fanInfo fanA; //Initialize fans 
fanInfo fanB;

FS3000 flowSenseA;
FS3000 flowSenseB;

STS31 tempSensorA = STS31();
STS31 tempSensorB = STS31();

double tempA = 0;
double tempB = 0;
double diffTemp = 0;
double chipTemp = 0;
double tempOffset = 0; //Offset between tempA and tempB to zero sensors 

const unsigned long pidServiceInterval = 25; //Number of milliseconds between PID service events
const unsigned long fanStart = 5000; //Don't start fans for first 5 seconds to prevent power weirdness
double setpointA, speedA, pwmA;
double setpointB, speedB, pwmB;
double Kp=5, Ki=10, Kd=10;
PID fanA_PID(&speedA, &pwmA, &setpointA, Kp, Ki, Kd, DIRECT);
PID fanB_PID(&speedB, &pwmB, &setpointB, Kp, Ki, Kd, DIRECT);

float luxFanCutoff = 1000; //If lux value is less than this, turn fans off 

void setup() {
	
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Init - Start");
  pinMode(pins::FAULT_LED, OUTPUT);
  pinMode(pins::CALIBRATE_LED, OUTPUT);
  pinMode(pins::GEN_STAT_LED, OUTPUT);
  digitalWrite(pins::FAULT_LED, LOW);
  digitalWrite(pins::CALIBRATE_LED, LOW);
  digitalWrite(pins::GEN_STAT_LED, LOW);

  Wire.begin();
  pinMode(I2C_OB_EN, OUTPUT);
	digitalWrite(I2C_OB_EN, HIGH);
	eeprom.begin();
	loadFromEEPROM(); //Load in all system values from EEPROM
	IO.begin();
	Serial.print("ALS Init\t");
	Serial.println(als.begin());  //Begin the ambient light sensor 
	als.SetIntTime(IT25); //Set to minimum int time - we only care about high lux values
	als.SetGain(GAIN_1_8); //Set to min gain, again, we only care about high lux values 
	Serial.println("Flow Sense Init:"); //DEBUG!
	Serial.println(initRemote(0));
	Serial.println(initRemote(1));
	// pinMode(pins::FAN_A_EN, OUTPUT);
	// pinMode(pins::FAN_B_EN, OUTPUT);
	initFans();
	initADC();

  IO.pinMode(pinsIO::CALIB, INPUT_PULLUP);
	IO.setInterrupt(pinsIO::CALIB, true); //Turn on interrupt on calibration pin
  pinMode(REC_EN, OUTPUT); //DIR
  pinMode(FOUT, OUTPUT); //FOUT
  pinMode(SEND_EN, OUTPUT);
  digitalWrite(SEND_EN, LOW);
  digitalWrite(FOUT, HIGH);
  digitalWrite(REC_EN, LOW);
  Serial.println("BEGIN!");
  pinMode(RX_SDI12, INPUT);
  pinMode(pins::GEN_INT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RX_SDI12), detectBreak, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pins::GEN_INT), detectExpInt, FALLING);
}

void loop() {
	static int readLength = 0;
	static String dValues[10];  // 10 String objects to hold the responses to aD0!-aD9! commands
	String readString;
	char readArray[25] = {0};
  	
  	char Input = NULL;
  	if(breakEvent) {
  		detachInterrupt(digitalPinToInterrupt(RX_SDI12)); //Turn off interrupt while transmission is happening 
  		breakEvent = false; //Clear flag
  		
  		Serial.println("BREAK DETECTED"); //DEBUG!
			unsigned long breakStart = millis();
  		
  		Serial1.begin(1200, SERIAL_7E1);
  		// && (millis() - breakStart) < breakTimeout
		while(Input != '!' && ((millis() - breakStart) < breakTimeout)) { //FIX! ADD TIMEOUT!
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
	          	static String dataStrs[10]; 
	          	switch(readString.charAt(1)) {
	          		case 'I':
	          			Serial1.print(identification()); //Return identification string
	          			break;
	      				case 'A':
	      					sdi12ADR = char2Int(readString.charAt(2)); //Convert new address to integer //FIX! Test first, write to EEPROM
	      					break;
	      				case 'M':
	      					Serial.println("START MEASUREMENT"); //DEBUG!
	      					Serial.println(readString);
	      					Serial.println(readString.charAt(2));
	      					Serial.println(readString.charAt(2), HEX);
	      					Serial.println(char2Int(readString.charAt(2)));
	      					Serial1.print(measurment(char2Int(readString.charAt(2)))); //Print measurment response and start conversion 
      						data(char2Int(readString.charAt(2)), dataStrs); //Load measurments into dataStrs to be accesed later
	      					break;
	      				case 'D':
	      					Serial.println("SEND DATA"); //DEBUG!
	      					Serial1.print(dataStrs[char2Int(readString.charAt(2))]); //Print measurment response and start conversion //FIX! Deal with -1 response??
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
		// Serial1.end(); //DEBUG!
		// releaseBus(); //Make sure to release bus even if caused by timeout along the way
		attachInterrupt(digitalPinToInterrupt(RX_SDI12), detectBreak, CHANGE); //Restore interrupt while waiting for the next event 
	}
	// if(millis() > 10000) initFans(); //DEBUG!

	static unsigned long pidLastServiced = millis();
	static bool genLEDState = false;
	if((millis() - pidLastServiced) > pidServiceInterval && millis() > fanStart) {
		digitalWrite(pins::GEN_STAT_LED, genLEDState);
		genLEDState = ~genLEDState; //Toggle LED state each time serviced 
		// Serial.print("Service Start Time"); //DEBUG!
		// Serial.println(millis());
		fanService();
		// Serial.print("Service Stop Time");//DEBUG!
		// Serial.println(millis());
		pidLastServiced = millis();
	}

	if(calibrateEvent) {
		calibrateEvent = false; //Clear flag
		Serial.println("CALIBRATE!"); //DEBUG!
		// noInterrupts(); //Turn interrupts off so timing is not interrupted
		detachInterrupt(digitalPinToInterrupt(RX_SDI12)); //Turn off SDI-12 while calibration is going on
		setCalibration();
		// interrupts(); //Turn interrupts back on once complete
		attachInterrupt(digitalPinToInterrupt(RX_SDI12), detectBreak, CHANGE); //Restore interrupt while waiting for the next event 
		Serial.println("CALIBRATION COMPLETE!"); //DEBUG!
		Serial.println(tempOffset);
		digitalWrite(pins::CALIBRATE_LED, LOW); //Turn back off after calibration completed 
	}

	// getRemoteData(); //DEBUG!
	// delay(1000);
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
		if(lowPeriod > BREAK_MIN && lowPeriod < 20) breakEvent = true; //If pulse is within range, set flag //DEBUG!
	}
}

String identification()
{
	return String(sdi12ADR) + COMPATABILITY_NUMBER + COMPANY_NAME + MODEL_NUMBER + SENSOR_VERSION + SERIAL_NUMBER;
}

String measurment(int num)
{
	String reponse = ""; //Keep track of the response from the sensor
	switch(num) { //Adjust response based on measurment type
		case 0:
			reponse = "0053"; //5 seconds to gather data, 3 data points //FIX??
			break;
		case 1:
			reponse = "0014"; //1 second to gather data, 4 data points
			break;
		case 2:
			reponse = "0014"; //1 second to gather data, 4 data points 
			break;
	}
	return String(sdi12ADR) + reponse; //Append address
}

int data(int num, String outputs[])
{
	String reponse = ""; //Keep track of the response from the sensor
	switch(num) { //Adjust response based on measurment type
		case 0:
			getADCData();
			outputs[0] = String(sdi12ADR) + float2SDI(diffTemp, 5) + float2SDI(tempA, 5) + float2SDI(tempB, 5); //FIX! 
			Serial.println(outputs[0]);
			break;
		case 1:
			getRemoteData();
			getFanData();
			outputs[0] = String(sdi12ADR) + "+" + String(fanA.airflow) + "+" + String(fanA.speed) + "+" + String(fanA.current) + "+" + String(fanA.temp);
			outputs[1] = String(sdi12ADR) + "+" + String(fanB.airflow) + "+" + String(fanB.speed) + "+" + String(fanB.current) + "+" + String(fanB.temp);
			outputs[2] = String(sdi12ADR) + "+" + String(als.GetLux());
			Serial.println(outputs[0]);
			Serial.println(outputs[1]);
			Serial.println(outputs[2]);
			break;
	}
	return 0;
}

String float2SDI(double data, int sigFigs)
{
	String leader = (data >= 0 ? "+" : ""); //Select the leading +/- value
	return leader + String(data, 5); //FIX!!!!!! CALCULATE SIG FIGS!!!
}

int char2Int(char val)
{
	if(val < 0x3A && val > 0x2F) return val - 0x30; //Offset value to get correct number 
	else return 0;
}

int initRemote(bool sensor)
{
	int offset = (sensor ? 0 : 3); //Select offset based on which setsor to init
	int antiOffset = (sensor ? 3 : 0); //Select the offset for whichever sensor is NOT being initialized 
	IO.pinMode(pinsIO::PWR_A_EN + offset, OUTPUT);
	IO.pinMode(pinsIO::I2C_A_EN + offset, OUTPUT);
	IO.pinMode(pinsIO::I2C_A_EN + antiOffset, OUTPUT); //Make sure whichever remote is not being used it disconected from I2C bus
	IO.digitalWrite(pinsIO::I2C_A_EN + antiOffset, LOW);
	IO.digitalWrite(pinsIO::PWR_A_EN + offset, HIGH);
	IO.digitalWrite(pinsIO::I2C_A_EN + offset, HIGH);

	bool init = true;
	switch(sensor){
		case 0:
			init = flowSenseA.begin();
			flowSenseA.setRange(AIRFLOW_RANGE_15_MPS);
			init = init & tempSensorA.begin(0x4A);
			break;
		case 1:
			init = flowSenseB.begin();
			flowSenseB.setRange(AIRFLOW_RANGE_15_MPS);
			init = init & tempSensorB.begin(0x4A);
			break;
	}
	if(init == false) return -1; //Return error code if failed to initalize 
	else return 0; 
}

int initADC()
{
	pinMode(pins::ADC_EN, OUTPUT);
  digitalWrite(pins::ADC_EN, HIGH); //Enable ADC 
	adc.begin();
	// adc.setAdcControl (AD7124_OpMode_SingleConv, AD7124_FullPower, true);
	adc.setAdcControl (AD7124_OpMode_Continuous, AD7124_FullPower, true);
	// adc.setup[0].setConfig(AD7124_Ref_ExtRef1, AD7124_Gain_4, false, AD7124_Burnout_Off, 0.511);
	adc.setup[0].setConfig(AD7124_Ref_ExtRef1, AD7124_Gain_4, false, AD7124_Burnout_Off, 0.511); 
	adc.setup[1].setConfig(AD7124_Ref_ExtRef1, AD7124_Gain_4, false, AD7124_Burnout_Off, 0.511);
  adc.setup[2].setConfig(AD7124_Ref_Internal, AD7124_Gain_1, true);

	adc.setup[0].setFilter(AD7124_Filter_SINC4, filterSelBits, AD7124_PostFilter_dB92);
  adc.setup[1].setFilter(AD7124_Filter_SINC4, filterSelBits, AD7124_PostFilter_dB92);
  adc.setup[2].setFilter(AD7124_Filter_SINC4, filterSelBits);

  adc.setChannel(0, 0, AD7124_Input_AIN2, AD7124_Input_AIN3, true); //RTD A
	adc.setChannel(1, 1, AD7124_Input_AIN4, AD7124_Input_AIN5, true); //RTD B
	// adc.setChannel(0, 0, AD7124_Input_AIN4, AD7124_Input_AIN5, true); //RTD B
  adc.setChannel(2, 2, AD7124_Input_TEMP, AD7124_Input_AVSS, true); //Internal temp
  return 0;
}

int initFans()
{
	csa.begin(); //Initialze CSA
	pinMode(pins::FAN_A_EN, OUTPUT);
	pinMode(pins::FAN_B_EN, OUTPUT);
	pinMode(pins::FAN_A_TACH, INPUT);
	pinMode(pins::FAN_B_TACH, INPUT);
	digitalWrite(pins::FAN_A_EN, LOW); //Make sure are off to start
	digitalWrite(pins::FAN_B_EN, LOW); //Make sure are off to start
	fanA_PID.SetMode(AUTOMATIC);
	fanB_PID.SetMode(AUTOMATIC);
	// analogWrite(pins::FAN_A_EN, 128); //DEBUG!
	// analogWrite(pins::FAN_B_EN, 128); //DEBUG!
	// tone(pins::FAN_A_EN, 100);
	// tone(pins::FAN_B_EN, 100);

	return 0; //FIX! Add check for over current, I2C fault on power monitor 
}

int getRemoteData()
{
	//ADD CURRENT
	//ADD TEMP
	//ADD PWM
	IO.digitalWrite(pinsIO::I2C_B_EN, LOW);
	IO.digitalWrite(pinsIO::I2C_A_EN, HIGH);
	IO.digitalWrite(pinsIO::PWR_A_EN, HIGH);
	fanA.temp = tempSensorA.readTemperature();
	fanA.airflow = flowSenseA.readMetersPerSecond();
	Serial.print("Flow Speed A:\t"); //DEBUG!
	Serial.print(fanA.airflow);

	IO.digitalWrite(pinsIO::I2C_A_EN, LOW);
	IO.digitalWrite(pinsIO::I2C_B_EN, HIGH);
	IO.digitalWrite(pinsIO::PWR_B_EN, HIGH);
	fanB.temp = tempSensorB.readTemperature();
	fanB.airflow = flowSenseB.readMetersPerSecond();
	Serial.print("\tFlow Speed B: "); //DEBUG!
	Serial.println(fanB.airflow);

	// fanA.temp = 4; //DEBUG!
	// fanB.temp = 5; 
	return 0; //DEBUG!
}

int getFanData()
{
	// float fanSpeed[2] = {0.0}; //Measure the RPM value of the fans  
	fanA.current = csa.GetCurrent(0, 100); //Grab current (calculated based on CSR values) from Channel 1 [mA] //FIX! Make sure to add averageing!
	fanB.current = csa.GetCurrent(1, 100); //Grab current (calculated based on CSR values) from Channel 2 [mA]
	if(pwmA > 0 || pwmB > 0) {
		analogWrite(pins::FAN_A_EN, 255); //DEBUG!
		analogWrite(pins::FAN_B_EN, 255); //DEBUG!
		// for(int fan = 0; fan < 2; fan++) {
		delay(10); //DEBUG!
			float periodA = pulseIn(pins::FAN_A_TACH, LOW, 250000); //Wait up to 250ms for a pulse 
			float periodB = pulseIn(pins::FAN_B_TACH, LOW, 250000);
			Serial.println("Pulse Length Fan Tach");
			Serial.println(periodA);
			Serial.println(periodB);
			// for(int i = 0; i < 5; i++) { //Find the average over 5 periods 
			// 	while(digitalRead(pins::FAN_A_TACH) == HIGH); //FIX! Add timeout~
			// 	unsigned long edgeTime = millis();
			// 	while(digitalRead(pins::FAN_A_TACH) == LOW);
			// 	while(digitalRead(pins::FAN_A_TACH) == HIGH);
			// 	unsigned long period = millis() - edgeTime; //Find period in ms
			// 	freq = freq + (1000.0/period);
			// }
			// freq = freq/5; //Average
			// fanSpeed[0] = (1000000.0/freqA)*60.0; //Convert to RPM //FIX! CHECK MATH!
			// fanSpeed[0] = (1000000.0/freqA)*60.0; //Convert to RPM //FIX! CHECK MATH!
		// }
		analogWrite(pins::FAN_A_EN, pwmA); //Return to previous vals
		analogWrite(pins::FAN_B_EN, pwmB); 
		fanA.speed = (1000000.0/(periodA*4.0))*60.0; //Convert to RPM //FIX! CHECK MATH!; Multiply by 4 to account for 2 periods per rotation, so 4x low periods
		fanB.speed = (1000000.0/(periodB*4.0))*60.0;
	}
	else { //If fans not commanded to spin, report speed as 0
		fanA.speed = 0;
		fanB.speed = 0;
	}
	return 0; //DEBUG!
}

int getADCData()
{
	chipTemp = adc.readIcTemp(2); 
  
  adc.setExCurrent(0, AD7124_ExCurrent_100uA);
  delay(1000);
  tempA = volt2temp(adc.readVolts(0));
  adc.setExCurrent(1, AD7124_ExCurrent_100uA); 
  delay(1000);
  tempB = volt2temp(adc.readVolts(1)); 
  adc.setExCurrent(0, AD7124_ExCurrent_Off); 
  diffTemp = (tempA - tempB) - tempOffset;
  Serial.println("ADC Data"); //DEBUG!
  Serial.println(tempA, 5);
  Serial.println(tempB, 5);
  Serial.println(chipTemp);
  return 0;
}

void loadFromEEPROM()
{
	pinMode(I2C_OB_EN, OUTPUT);
	digitalWrite(I2C_OB_EN, HIGH);
	uint8_t storedAddress = eeprom.readByte(0);
	if(storedAddress >= 0 && storedAddress <= 9) sdi12ADR = storedAddress; //If value is within range, copy over
	double storedTempOffset = readDouble(calibOffset);
	if(storedTempOffset < 10 && storedTempOffset > -10) tempOffset = storedTempOffset; //Check if within range, then set //FIX! make range global

}

void setCalibration()
{
	digitalWrite(pins::CALIBRATE_LED, HIGH); //Turn on calibration LED to indicate start of calibration 
	pinMode(I2C_OB_EN, OUTPUT);
	digitalWrite(I2C_OB_EN, HIGH);
	double err = 0;
	for(int i = 0; i < 10; i++) { //FIX! make calibration number avg global variable
		getADCData(); //Update values
		err = err + (tempA - tempB); //Sum all errors
	}
	err = err/10.0; //Find average

	if(err < 10.0 && err > -10.0) {
		writeDouble(err, calibOffset); //If within range, write back to eeprom
		tempOffset = err; //Update local as well
	}
}

void fanService()
{
	setpointA = 2.5; //Command 1 m/s air speed
	setpointB = 2.5;
	// getFanData(); //Get new data from sensors
	// getRemoteData();
	float luxVal = als.GetLux();
	if(luxVal > luxFanCutoff) {
		IO.safeMode(PCAL9535A::SAFEOFF); //Turn off safe mode to speed up writes
		//DEBUG! Getting only air speed to save on time 
		IO.digitalWrite(pinsIO::I2C_B_EN, LOW);
		IO.digitalWrite(pinsIO::I2C_A_EN, HIGH);
		// IO.digitalWrite(pinsIO::PWR_A_EN, HIGH);
		// fanA.temp = tempSensorA.readTemperature();
		fanA.airflow = flowSenseA.readMetersPerSecond();
		IO.digitalWrite(pinsIO::I2C_A_EN, LOW);
		IO.digitalWrite(pinsIO::I2C_B_EN, HIGH);
		// IO.digitalWrite(pinsIO::PWR_B_EN, HIGH);
		// fanB.temp = tempSensorB.readTemperature();
		fanB.airflow = flowSenseB.readMetersPerSecond();

		speedA = fanA.airflow; //Pass to PID controller 
		speedB = fanB.airflow;
		fanA_PID.Compute(); //Compute with new inputs
		fanB_PID.Compute();
		analogWrite(pins::FAN_A_EN, pwmA); //Set output values
		analogWrite(pins::FAN_B_EN, pwmB);
		Serial.print("Fan Vals:\t"); //DEBUG!
		Serial.print(pwmA);
		Serial.print("\t");
		Serial.print(pwmB);
		Serial.print("\t");
		Serial.println(luxVal);
	}
	else { //If light is less than cutoff value, turn off fans 
		pwmA = 0; //Establish global values as off
		pwmB = 0;
		analogWrite(pins::FAN_A_EN, 0); //Set output values
		analogWrite(pins::FAN_B_EN, 0);
		// Serial.print("Lux Val:\t");
		// Serial.println(luxVal);
	}
	
	
}

void detectExpInt()
{
	if(IO.getInterrupt(pinsIO::CALIB)) calibrateEvent = true; //If calibration button pin was cause of interrupt, set flag
}

double volt2temp(double volt)
{
	const double a = -5.775e-4;
	const double b = 3.9083;
	const double c = 1000.0*(1.0 - 10.0*(volt));

	double temp = (-b + sqrt(pow(b, 2) - 4*a*c))/(2*a);
	return temp;
}

int writeDouble(double val, uint16_t position) //Break double into bytes, then write into EEPROM
{ 
    char buffer[sizeof(double)];
    memcpy(&buffer,&val,sizeof(double));
    for (int i=0; i<sizeof(double); i++) {
        eeprom.writeByte(position+i,buffer[i]);
    }
    return 1;
}

double readDouble(uint16_t position) //Undo actions of writeDouble to recover bytes from EEPROM and cast them into standard double format 
{
    char buffer[sizeof(double)];
    double loaded;
    for (int i = 0; i<sizeof(double);i++) {
        Serial.println(i);
        buffer[i] = eeprom.readByte(i+position);
    }
    memcpy(&loaded,&buffer,sizeof(double));
    return loaded;
}
