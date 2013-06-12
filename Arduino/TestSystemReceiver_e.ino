#include <MAX6675.h>

#include <XBee.h>




/***************************
 *
 *  Setup environment
 *
 ***************************/

char test = 'a' ;

int val ; // placholder vaiable
byte i ;   // looping variable
byte j ;


// liquid level
#define LIQUID_LEVEL_P 1 // Analog sensor input
int _liquidLevel = 0 ; // storage variable to read into
int liquidLevel_offset = 0 ; // offset amount for level
void checkLookup(int val) ;

int lookupReadings[] = {
  0, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000} 
;
int lookup[] = {
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11} 
;
int lookupLength = 11 ;

int minLevel = lookup[0] ; // minimum level before relay is tripped
int maxLevel = lookup[lookupLength] ;
int levelFlag = 0 ; 
void watchLevel(int relay) ; // function to always check the water level and trip the relay and send the data



// water flow
#define WATER_FLOW_P 2 // Digital
int _waterFlow = 1 ; // storage variable to read into
void readWaterFlow() ;  // prototypes
volatile int NbTopsFan; //measuring the rising edges of the signal
int Calc; 


// temperature and humidity
#define TEMP_HUMID_P 0 // Analog sensor input
int _temperature = 2 ;
int _humidity = 3 ;
void readTempHumid() ; // prototype
byte read_tempHumid() ;
byte dht11_dat[5]; // for reading temp and humid
byte dht11_in;
//byte result ;


// thermocouple
#define THERMO_DAT_P 8 // data input
#define THERMO_CS_P 7
#define THERMO_CLK_P 6
int _thermocouple = 4 ;
int units = 1;
MAX6675 thermoco(THERMO_CLK_P, THERMO_CS_P, THERMO_DAT_P, units);


// light intensity
#define LIGHT_FR_P 3  // i
#define LIGHT_L0_P 4 
#define LIGHT_L1_P 5 
int _lightFrequency = 5 ;
long getUwattCm2() ;
void add_pulse() ;
int calcSensitivity;    
unsigned long sensitivityHighThresh = 2000;
unsigned long sensitivityLowThresh = 100000;

unsigned long pulseCount = 0;
unsigned long currentTime = millis();  
unsigned long startTime = currentTime; 
unsigned int tm_diff = 0;

unsigned long frequency;
float uWattCm2;
volatile unsigned long curPulseCount;
unsigned int count = 0;
unsigned int scale;   // holds the TLS scale value, see below
#define READ_TM 1000 // milleseconds between frequency calculations


// Relay outputs
#define RELAY0_P 10 
#define RELAY1_P 11 
#define RELAY2_P 12 
#define RELAY3_P 13
int _relay0 = 0 ; //6
int _relay1 = 0 ; //7
int _relay2 = 0 ; //8
int _relay3 = 0 ; //9
void relayTest() ;
void relayOn(char relay) ;
void relayOff(char relay) ;

// Water Trip
#define WATER_TRIP_P 9 // digtal Pin, is normally high
int _waterTrip = 10;
void readWaterTrip() ;


//Current
#define CURRENT0_P 5 // analaog input
#define CURRENT1_P 6 
#define CURRENT2_P 7
int _current0 = 11 ;
int _current1 = 12 ;
int _current2 = 13 ;
void readCurrent() ;


//pH
int _pH = 14 ;


//UV
int _UV = 15 ;


// dissolved Oxygen
int _dOxygen = 16 ;


// XBee 
// used for sending data by radio
//
const uint8_t num_data = 17 ;
//
int sensor_data[num_data] ;
uint8_t dataPayload[num_data*2] ; // one for the high byte and one for the low, hence *2
//  XBee object
XBee xbee = XBee();
// SH + SL Address of receiving coordinator XBee
XBeeAddress64 addr64 = XBeeAddress64(0x0013a200, 0x406f2213);
ZBTxRequest zbTx = ZBTxRequest(addr64, dataPayload, sizeof(dataPayload));
ZBTxStatusResponse txStatus = ZBTxStatusResponse();
void packSendData() ; // prototype
void testSend(char in) ;
//
XBeeResponse response = XBeeResponse();
// create reusable response objects for responses we expect to handle 
ZBRxResponse rx = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();
void packReadData() ;

void interpret_action(char action) ;
void readSensorsSend() ;



/***************************
 ***************************
 *
 * Setup
 *
 ***************************
 ***************************/

void setup() {

  // start serial
  xbee.begin(9600);



  // water flow interupt
  pinMode(WATER_FLOW_P, INPUT) ;
  attachInterrupt(0, rpm, RISING); //and the interrupt is attached

  // temp and humid setup
  DDRC |= _BV(TEMP_HUMID_P);
  PORTC |= _BV(TEMP_HUMID_P);

  // thermocouple
  pinMode(THERMO_DAT_P, INPUT) ;
  pinMode(THERMO_CS_P, OUTPUT) ;
  pinMode(THERMO_CLK_P, OUTPUT) ;

  //light intensity
  pinMode(LIGHT_FR_P, INPUT) ;
  pinMode(LIGHT_L0_P, OUTPUT) ;
  pinMode(LIGHT_L1_P, OUTPUT) ;

  attachInterrupt(1, add_pulse, RISING);
  scale = 100; // set this to match TSL_S2 and TSL_S3 


  // Relays
  pinMode(RELAY0_P, OUTPUT) ;
  pinMode(RELAY1_P, OUTPUT) ;
  pinMode(RELAY2_P, OUTPUT) ;
  pinMode(RELAY3_P, OUTPUT) ;
  digitalWrite(RELAY0_P, LOW) ;
  digitalWrite(RELAY1_P, LOW) ;
  digitalWrite(RELAY2_P, LOW) ;
  digitalWrite(RELAY3_P, LOW) ;

  // water trip
  pinMode(WATER_TRIP_P, INPUT) ;
  /*
*/

}


/***************************
 ***************************
 *
 * Arduino loop function
 *
 ***************************
 ***************************/

void flash() {
 for(i=0;i<3;i++) {
  digitalWrite(13, HIGH) ;
  delay(10) ;
  digitalWrite(13, LOW) ;
  delay(10) ;
 } 
}

// continuously reads packets, looking for ZB Receive or Modem Status
void loop() {

  packReadData() ;
  watchLevel(RELAY0_P) ; //checks water level
  delay(10) ;

}

/***************************
 ***************************
 *
 * XBee comms 
 *
 ***************************
 ***************************/

void packReadData() {

  xbee.readPacket(100) ;

  if (xbee.getResponse().isAvailable()) {
    flash() ;

    if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
      // got a zb rx packet

      // now fill our zb rx class
      xbee.getResponse().getZBRxResponse(rx);

      char in = (char) rx.getData(0) ;
      if(in == 'a') {
        packSendData() ;
      } 
      else if(in == 'b') {
        testSend(in) ;
      }
      else if(in == 'r') {
        char num = (char) rx.getData(1) ;
        testSend(num) ;
        relayOn(num) ;
      } 
      else if(in == 's') {
        char num = (char) rx.getData(1) ;
        testSend(num) ;
        relayOff(num) ;
      } // end if elses

    } 
  } 
}

void testSend(char in) {
  dataPayload[0] = in ;
  for(i=1; i<num_data*2; i++)  dataPayload[i] = (uint8_t) 120;
  // send off the data to the radio
  xbee.send(zbTx) ;
}

void packSendData() {

  i=0 ;
  sensor_data[i++] = _liquidLevel ;
  sensor_data[i++] = _waterFlow ;
  sensor_data[i++] = _temperature ;
  sensor_data[i++] = _humidity ;
  sensor_data[i++] = _thermocouple ;
  sensor_data[i++] = _lightFrequency ;
  sensor_data[i++] = _relay0 ;
  sensor_data[i++] = _relay1 ;
  sensor_data[i++] = _relay2 ;
  sensor_data[i++] = _relay3 ;
  sensor_data[i++] = _waterTrip ;
  sensor_data[i++] = _current0 ;
  sensor_data[i++] = _current1 ;
  sensor_data[i++] = _current2 ;
  sensor_data[i++] = _pH ;
  sensor_data[i++] = _UV ;
  sensor_data[i++] = _dOxygen ;


  j=0 ;
  for(i=0; i<num_data; i++) {
    dataPayload[j] =  (char)  (sensor_data[i] >> 8) & 0xff ;
    dataPayload[j+1] = (char) sensor_data[i] & 0xff;
    j+=2;
  } 

  xbee.send(zbTx) ;
}




/***************************
 ***************************
 *
 * Sensors
 *
 ***************************
 ***************************/


void readSensors() {

  readLiquidLevel() ;

  readWaterFlow() ;

  readTempHumid() ;

  getUwattCm2() ;
  _lightFrequency = frequency ;

  _thermocouple = thermoco.read_temp() ;

  readWaterTrip() ;

  readCurrent() ;

}

/***************************
 *
 * Liquid level
 *
 ***************************/

void readLiquidLevel() {
  val = analogRead(LIQUID_LEVEL_P) ;
  //_liquidLevel = map(val, 240, 30, 0, 12) ;
  checkLookup(val) ;
  _liquidLevel += liquidLevel_offset ; 
  // _liquidLevel = val ;
}


void checkLookup(int val) {
 for(i=lookupLength-1; i>=0; i--) {
   if(val >= lookupReadings[i])  _liquidLevel = lookup[i] ;
 }
}


// check to see how water level is doing
void watchLevel(int relay) {
  readLiquidLevel() ;
  if(_liquidLevel <= minLevel && levelFlag == 0) {
    digitalWrite(relay, HIGH) ;
    levelFlag = 1 ;
    packSendData() ;
  }
  if(_liquidLevel >= maxLevel && levelFlag == 1) {
    digitalWrite(relay, HIGH) ; 
    levelFlag = 0 ;
    packSendData() ;
  }
}
/***************************
 *
 * Water flow interupt method  
 *
 ***************************/


void readWaterFlow() {
  NbTopsFan = 0;	//Set NbTops to 0 ready for calculations
  //sei();		//Enables interrupts
  delay (1000);	//Wait 1 second
  //cli();		//Disable interrupts
  Calc = (NbTopsFan * 60 / 7.5); //(Pulse frequency x 60) / 7.5Q, = flow rate in L/hour 
  _waterFlow = Calc ;

}

void rpm ()     //This is the function that the interupt calls 
{ 
  NbTopsFan++;  //This function measures the rising and falling edge of the hall effect sensors signal
} 


/***************************
 *
 * Temp and humidity reading  
 *
 ***************************/


void readTempHumid() {

  // start condition
  // 1. pull-down i/o pin from 18ms
  PORTC &= ~_BV(TEMP_HUMID_P);
  delay(18);
  PORTC |= _BV(TEMP_HUMID_P);
  delayMicroseconds(40);

  DDRC &= ~_BV(TEMP_HUMID_P);
  delayMicroseconds(40);

  dht11_in = PINC & _BV(TEMP_HUMID_P);

  if(dht11_in){

    return;
  }
  delayMicroseconds(80);

  dht11_in = PINC & _BV(TEMP_HUMID_P);

  if(!dht11_in){

    return;
  }
  delayMicroseconds(80);
  // now ready for data reception
  for (i=0; i<5; i++)
    dht11_dat[i] = read_tempHumid();

  DDRC |= _BV(TEMP_HUMID_P);
  PORTC |= _BV(TEMP_HUMID_P);

  byte dht11_check_sum = dht11_dat[0]+dht11_dat[1]+dht11_dat[2]+dht11_dat[3];
  // check check_sum
  if(dht11_dat[4]!= dht11_check_sum)
  {

  }


  _humidity = int(dht11_dat[0]) ;
  _temperature = int(dht11_dat[2]) ;

}

byte read_tempHumid()
{
  byte i = 0;
  byte result=0;
  for(i=0; i< 8; i++){

    while(!(PINC & _BV(TEMP_HUMID_P)));  // wait for 50us
    delayMicroseconds(30);

    if(PINC & _BV(TEMP_HUMID_P)) 
      result |=(1<<(7-i));
    while((PINC & _BV(TEMP_HUMID_P)));  // wait '1' finish

  }
  return result;
}




/***************************
 *
 * Light intensity 
 *
 ***************************/

void add_pulse() {
  // increase pulse count
  pulseCount++;

  // DON'T calculate the frequency every READ_TM ms
  // just store the pulse count to be used outside of the interrupt
  currentTime = millis();
  if( currentTime - startTime >= READ_TM ) 
  { 
    curPulseCount = pulseCount;  // use curPulseCount for calculating freq/uW
    pulseCount = 0;     
    startTime = millis(); 
  }
} 

long getUwattCm2() {
  // copy pulse counter and multiply.
  // the multiplication is necessary for the current
  // frequency scaling level.  

  frequency = curPulseCount * scale;

  // get uW observed - assume 640nm wavelength
  // calc_sensitivity is our divide-by to map to a given signal strength
  // for a given sensitivity (each level of greater sensitivity reduces the signal
  // (uW) by a factor of 10)

  float uw_cm2 = (float) frequency / (float) calcSensitivity;

  // extrapolate into entire cm2 area
  uWattCm2  = uw_cm2  * ( (float) 1 / (float) 0.0136 );

  return(uWattCm2);
}
void setSensitivity()
{
  getUwattCm2();
  if (uWattCm2 <  sensitivityHighThresh)
  {
    sensitivity(3);
    return;
  }  
  if (uWattCm2 > sensitivityLowThresh )
  {
    sensitivity(1);
    return;
  } 
  sensitivity(2); 
}
void sensitivity(uint8_t level)
{
  switch (level) 
  {
  case 1:
    if (calcSensitivity != 10)
    {

    }
    digitalWrite(LIGHT_L0_P, HIGH);  // S0 HIGH and S1 LOW = 1x sensitivity
    digitalWrite(LIGHT_L1_P, LOW);
    calcSensitivity = 10;
    break;
  case 2:
    if (calcSensitivity != 100)
    {

    }
    digitalWrite(LIGHT_L0_P, LOW);  // S0 LOW and S1 HIGH = 10x sensitivity
    digitalWrite(LIGHT_L1_P, HIGH);
    calcSensitivity = 100;
    break;
  case 3:
    if (calcSensitivity != 1000)
    {

    }
    digitalWrite(LIGHT_L0_P, HIGH);  // S0 HIGH and S1 HIGH = 100x sensitivity
    digitalWrite(LIGHT_L1_P, HIGH);
    calcSensitivity = 1000;
    break;
  }
  return;
}

/***************************
 *
 * Water Trip
 *
 ***************************/

void readWaterTrip() {
  _waterTrip = digitalRead(WATER_TRIP_P) ; 

}

/***************************
 *
 * Current Sensors
 *
 ***************************/

void readCurrent() {

  _current0 = analogRead(CURRENT0_P) ;
  _current1 = analogRead(CURRENT1_P) ; 
  _current2 = analogRead(CURRENT2_P) ; 

}

/***************************
 *
 * pH
 *
 ***************************/
/***************************
 *
 * UV
 *
 ***************************/
/***************************
 *
 * dissolved Oxygen
 *
 ***************************/

/***************************
 *
 * Relays
 *
 ***************************/

void relayTest() {
  relayOn(1) ;
  delay(50) ;
  relayOn(2) ;
  delay(50) ;
  relayOn(3) ;
  delay(50) ;
  relayOn(4) ;

  delay(100) ;

  relayOff(4) ;
  delay(50) ;
  relayOff(3) ;
  delay(50) ;
  relayOff(2) ;
  delay(50) ;
  relayOff(1) ;
}

void relayOn(char relay) {
  switch (relay) {
  case 'a':
    digitalWrite(RELAY0_P, HIGH) ;
    _relay0 = 1 ;
    break ; 
  case 'b':
    digitalWrite(RELAY1_P, HIGH) ;
    _relay1 = 1 ;
    break ; 
  case 'c':
    digitalWrite(RELAY2_P, HIGH) ;
    _relay2 = 1 ;
    break ; 
  case 'd':
    digitalWrite(RELAY3_P, HIGH) ;
    _relay3 = 1 ;
    break ; 
  }
}

void relayOff(char relay) {
  switch (relay) {
  case 'a':
    digitalWrite(RELAY0_P, LOW) ;
    _relay0 = 0 ;
    break ; 
  case 'b':
    digitalWrite(RELAY1_P, LOW) ;
    _relay1 = 0 ;
    break ; 
  case 'c':
    digitalWrite(RELAY2_P, LOW) ;
    _relay2 = 0 ;
    break ; 
  case 'd':
    digitalWrite(RELAY3_P, LOW) ;
    _relay3 = 0 ;
    break ; 
  }
}





