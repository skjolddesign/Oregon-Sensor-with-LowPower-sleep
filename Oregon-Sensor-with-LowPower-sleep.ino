   /* 5uA low power Oregon encoder with DHT AM2302/AM2330 sensor.
    *  
    * LOW POWER:
    * Arduino pro mini 3.3v 8M. removed regulator + power led/resistor. Thats it. (enhanced version from Ebay)
    * 
    * DHT:
    * DHT sensonr connected to Arduino gnd, D2(vcc), D3(data).
    * AM2320 is drop in replacement.http://www.kandrsmith.org/rjs/misc/hygrometers/calib_many.html
    * AM2320 tested ok wit single bus. see datasheet for siglebus vs i2c: https://akizukidenshi.com/download/ds/aosong/AM2320.pdf
    * AM2320 3 wire connection: vcc, data, nc, gnd. add 10k smd resistor on vcc to data.
    * 
    * RF TRANSMITTER:
    * STX882 transmitter connectedto Arduino D7(gnd), D8(vcc), D9(data). 
    * Antena: Coil 17cm solid core wire around ø6mm screwdriver.
    *
    * FIXED READVCC LOWPOWER 4.5uA!
    */
         
    //+------------------------------------------------------------------+
    //| DHT AM2302, pin 1=vcc, 2=data, 3=not in use, 4=gnd
    //+------------------------------------------------------------------+
    #include "DHT.h"
    int DHT_Vcc_Pin = 3; 
    #define DHT_Data_Pin 2      // data
 // Uncomment whatever DHT type you're using!
    #define DHTTYPE DHT11   // DHT 11 (whole numbers only) 
 //#define DHTTYPE DHT22   // DHT 22 (AM2302), AM2321
 //#define DHTTYPE DHT21   // DHT 21 (AM2301)
    #define led 13 
    DHT dht(DHT_Data_Pin, DHTTYPE);
 
   //+------------------------------------------------------------------+
   //| oregon transmitter
   //+------------------------------------------------------------------+
    byte addresse = 11; // 0-255. sensor adress, will be shown in telldus.com. 
    const byte txGnd = 9;
    const byte tx_Vcc_Pin = 11; 
    const byte TX_DATA_PIN = 10; //tx data pin
    //#define THN132N  //enbart termometer, ingen humidity. Modell:1A2D/EA4C
    const unsigned long TIME = 512;
    const unsigned long TWOTIME = TIME*2;
    #define SEND_HIGH() digitalWrite(TX_DATA_PIN, HIGH)
    #define SEND_LOW() digitalWrite(TX_DATA_PIN, LOW)
    // Buffer for Oregon message
    #ifdef THN132N
        byte OregonMessageBuffer[8];
    #else
         byte OregonMessageBuffer[9];
    #endif

    //+------------------------------------------------------------------+
    //| BATTERY
    //+------------------------------------------------------------------+
    bool batOK = true;
    int batCnt = 100;//batt counter. used to set when to check the battery. set to 100 to trigg at startup.
    //+------------------------------------------------------------------+
    //| LOW POWER
    //+------------------------------------------------------------------+ 
    #include <JeeLib.h>  // Include library containing low power functions
    ISR(WDT_vect) { Sleepy::watchdogEvent(); } // Setup for low power waiting
    const long sleeptime = 60000; // set to 60000, to get a minute cycle
    int cyclesToSleep = 5; // set minute here(5 cycles)
    int cycleCounter = 100; //low power counter, used to sleep more than one minute. set to 100 to trigg at startup.
   
    
//+------------------------------------------------------------------+
//| SETUP
//+------------------------------------------------------------------+
    void setup() {
      
   //---  dht Setup ---------
   pinMode(DHT_Vcc_Pin, OUTPUT);     // sets the digital pin as output
   digitalWrite(DHT_Vcc_Pin, LOW);   // sets power off
   pinMode(led, OUTPUT);              // sets the digital pin as output
   digitalWrite(led, LOW);            // sets the LED off
   
   Serial.begin(38400); 
   dht.begin();

  //---  tx Setup ---------
  pinMode(tx_Vcc_Pin, OUTPUT);          // sets the digital pin as output
  digitalWrite(tx_Vcc_Pin, LOW);        // sets power off
  pinMode(txGnd, OUTPUT);             // sets the digital pin as output
  digitalWrite(txGnd, LOW);  
  pinMode(TX_DATA_PIN, OUTPUT);            // sets the digital pin as output
  
  Serial.print(F("\n[Oregon V2.1 encoder] Adress = "));
  Serial.println(addresse);
  Serial.println(F("Low power 5uA. DHT AM2302. 5min intervall.)"));
  delay(200);
  
 //---- oregon setup---------------
  SEND_LOW();  
  #ifdef THN132N  
    // Create the Oregon message for a temperature only sensor (TNHN132N)
    byte ID[] = {0xEA,0x4C};
  #else
     // Create the Oregon message for a temperature/humidity sensor (THGR2228N)
    byte ID[] = {0x1A,0x2D};
  #endif  
 
  setType(OregonMessageBuffer, ID);
  setChannel(OregonMessageBuffer, 0x20);
//setId(OregonMessageBuffer, 0xBB); //BB=187
//getTempAndSend();
} //end setup


 //+------------------------------------------------------------------+
 //| MAIN LOOP
 //+------------------------------------------------------------------+
 void loop() {
if (cycleCounter>=cyclesToSleep){ //1minute x 5. = 5minutters intervall
    getTempAndSend();
    cycleCounter=0;
}
cycleCounter+=1;     
sleep(sleeptime);//sleep for one minute, then wake up again.
} //end main loop



void getTempAndSend(){
      //--- power on
      digitalWrite(DHT_Vcc_Pin, HIGH);   // sensor power
      sleep(600); // (dht boot time=500ms)
      digitalWrite(led, HIGH);   // LED  
      // Reading temperature or humidity takes about 250 milliseconds!
      // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
      float h = dht.readHumidity();
      // Read temperature as Celsius
      float t = dht.readTemperature();
      //--- power off
      digitalWrite(DHT_Vcc_Pin, LOW);   // sensor power
      digitalWrite(led, LOW);   // sensor power
      digitalWrite(DHT_Data_Pin, LOW);   // sensor pin low for 5uA low power. Unngår strømlekasje til vcc via motstand. 
       
      // Check if any reads failed and exit early (to try again).
      if (isnan(h) || isnan(t)) {
        Serial.println("Failed to read from DHT sensor!");
        return;
      }
     
      Serial.print("Humidity: "); 
      Serial.print(h);
      Serial.print(" %\t");
      Serial.print("Temperature: "); 
      Serial.print(t);
      Serial.println(" *C ");
      send433(t,h,addresse);// temp,humidity,address
      
}

//----- sleep low power----------------------------------
void sleep(long milli){
   Serial.flush(); //Waits for the transmission of outgoing serial data to complete
   Sleepy::loseSomeTime(milli); //msecs Number of milliseconds to sleep, in range 0..65535. 
  }

//--- read battery every 10. syklus.
void checkBattery(){
batCnt++;
if (batCnt>=10){
  batCnt=0;
  int vcc = readVccFixedLowPower();
  Serial.print("Vcc = " );
  Serial.println(vcc);
batOK = (vcc >= 3300); //ok if over millivolt. use bool in tx send func.
  }  
}

 //---------------- Send data to Telldus ----------------------------
 void send433(float temperature, byte humidity, byte Identitet) {

checkBattery();
digitalWrite(tx_Vcc_Pin, HIGH);   // sets the LED on

  // Get Temperature, humidity and battery level from sensors
  // (ie: 1wire DS18B20 for température, ...)
  setId(OregonMessageBuffer, Identitet); //BB=187
 // setBatteryLevel(OregonMessageBuffer, 1); // 0 : low, 1 : high
  setBatteryLevel(OregonMessageBuffer, batOK); // 0 : low, 1 : high
  setTemperature(OregonMessageBuffer, temperature); //org  setTemperature(OregonMessageBuffer, 55.5);
 
#ifndef THN132N
  // Set Humidity
  setHumidity(OregonMessageBuffer, humidity);
#endif  
 
  // Calculate the checksum
  calculateAndSetChecksum(OregonMessageBuffer);
 
  // Show the Oregon Message
  for (byte i = 0; i < sizeof(OregonMessageBuffer); ++i)   {     Serial.print(OregonMessageBuffer[i] >> 4, HEX);
    Serial.print(OregonMessageBuffer[i] & 0x0F, HEX);
  }
    Serial.println();
  // Send the Message over RF
  sendOregon(OregonMessageBuffer, sizeof(OregonMessageBuffer));
  // Send a "pause"
  SEND_LOW();
  delayMicroseconds(TWOTIME*8);
  // Send a copie of the first message. The v2.1 protocol send the
  // message two time 
  sendOregon(OregonMessageBuffer, sizeof(OregonMessageBuffer));
 
  SEND_LOW();
  digitalWrite(tx_Vcc_Pin, LOW);   // sets the LED on
 }
 // ---------- End send data till Telldus ---------

 /**
 * \brief    Send logical "0" over RF
 * \details  azero bit be represented by an off-to-on transition
 * \         of the RF signal at the middle of a clock period.
 * \         Remenber, the Oregon v2.1 protocol add an inverted bit first 
 */
inline void sendZero(void) {

  SEND_HIGH();
  delayMicroseconds(TIME);
  SEND_LOW();
  delayMicroseconds(TWOTIME);
  SEND_HIGH();
  delayMicroseconds(TIME);
}
 
/**
 * \brief    Send logical "1" over RF
 * \details  a one bit be represented by an on-to-off transition
 * \         of the RF signal at the middle of a clock period.
 * \         Remenber, the Oregon v2.1 protocol add an inverted bit first 
 */
inline void sendOne(void) {

   SEND_LOW();
   delayMicroseconds(TIME);
   SEND_HIGH();
   delayMicroseconds(TWOTIME);
   SEND_LOW();
   delayMicroseconds(TIME);
}
 
/**
* Send a bits quarter (4 bits = MSB from 8 bits value) over RF
*
* @param data Source data to process and sent
*/
 
/**
 * \brief    Send a bits quarter (4 bits = MSB from 8 bits value) over RF
 * \param    data   Data to send
 */
inline void sendQuarterMSB(const byte data){ 

  (bitRead(data, 4)) ? sendOne() : sendZero();
  (bitRead(data, 5)) ? sendOne() : sendZero();
  (bitRead(data, 6)) ? sendOne() : sendZero();
  (bitRead(data, 7)) ? sendOne() : sendZero();
}
 
/**
 * \brief    Send a bits quarter (4 bits = LSB from 8 bits value) over RF
 * \param    data   Data to send
 */
inline void sendQuarterLSB(const byte data) {

  (bitRead(data, 0)) ? sendOne() : sendZero();
  (bitRead(data, 1)) ? sendOne() : sendZero();
  (bitRead(data, 2)) ? sendOne() : sendZero();
  (bitRead(data, 3)) ? sendOne() : sendZero();
}
 
/******************************************************************/
/******************************************************************/
/******************************************************************/
 
/**
 * \brief    Send a buffer over RF
 * \param    data   Data to send
 * \param    size   size of data to send
 */
void sendData(byte *data, byte size){

  for(byte i = 0; i < size; ++i)
  {
    sendQuarterLSB(data[i]);
    sendQuarterMSB(data[i]);
  }
}
 
/**
 * \brief    Send an Oregon message
 * \param    data   The Oregon message
 */
void sendOregon(byte *data, byte size){

    sendPreamble();
    //sendSync();
    sendData(data, size);
    sendPostamble();
}
 
/**
 * \brief    Send preamble
 * \details  The preamble consists of 16 "1" bits
 */
inline void sendPreamble(void)
{
  byte PREAMBLE[]={0xFF,0xFF};
  sendData(PREAMBLE, 2);
}
 
/**
 * \brief    Send postamble
 * \details  The postamble consists of 8 "0" bits
 */
inline void sendPostamble(void)
{
#ifdef THN132N
  sendQuarterLSB(0x00);
#else
  byte POSTAMBLE[]={0x00};
  sendData(POSTAMBLE, 1);  
#endif
}
 
/**
 * \brief    Send sync nibble
 * \details  The sync is 0xA. It is not use in this version since the sync nibble
 * \         is include in the Oregon message to send.
 */
inline void sendSync(void)
{
  sendQuarterLSB(0xA);
}
 
/******************************************************************/
/******************************************************************/
/******************************************************************/
 
/**
 * \brief    Set the sensor type
 * \param    data       Oregon message
 * \param    type       Sensor type
 */
inline void setType(byte *data, byte* type) 
{
  data[0] = type[0];
  data[1] = type[1];
}
 
/**
 * \brief    Set the sensor channel
 * \param    data       Oregon message
 * \param    channel    Sensor channel (0x10, 0x20, 0x30)
 */
inline void setChannel(byte *data, byte channel) 
{
    data[2] = channel;
}
 
/**
 * \brief    Set the sensor ID
 * \param    data       Oregon message
 * \param    ID         Sensor unique ID
 */
inline void setId(byte *data, byte ID) 
{
  data[3] = ID;
}
 
/**
 * \brief    Set the sensor battery level
 * \param    data       Oregon message
 * \param    level      Battery level (0 = low, 1 = high)
 */
void setBatteryLevel(byte *data, byte level)
{
  if(!level) data[4] = 0x0C;
  else data[4] = 0x00;
}
 
/**
 * \brief    Set the sensor temperature
 * \param    data       Oregon message
 * \param    temp       the temperature
 */
void setTemperature(byte *data, float temp) 
{
  // Set temperature sign
  if(temp < 0)
  {
    data[6] = 0x08;
    temp *= -1;  
  }
  else
  {
    data[6] = 0x00;
  }
 
  // Determine decimal and float part
  int tempInt = (int)temp;
  int td = (int)(tempInt / 10);
  int tf = (int)round((float)((float)tempInt/10 - (float)td) * 10);
 
  int tempFloat =  (int)round((float)(temp - (float)tempInt) * 10);
 
  // Set temperature decimal part
  data[5] = (td << 4);
  data[5] |= tf;
 
  // Set temperature float part
  data[4] |= (tempFloat << 4);
}
 
/**
 * \brief    Set the sensor humidity
 * \param    data       Oregon message
 * \param    hum        the humidity
 */
void setHumidity(byte* data, byte hum)
{
    data[7] = (hum/10);
    data[6] |= (hum - data[7]*10) << 4;
}
 
/**
 * \brief    Sum data for checksum
 * \param    count      number of bit to sum
 * \param    data       Oregon message
 */
int Sum(byte count, const byte* data)
{
  int s = 0;
 
  for(byte i = 0; i<count;i++)
  {
    s += (data[i]&0xF0) >> 4;
    s += (data[i]&0xF);
  }
 
  if(int(count) != count)
    s += (data[count]&0xF0) >> 4;
 
  return s;
}
 
/**
 * \brief    Calculate checksum
 * \param    data       Oregon message
 */
void calculateAndSetChecksum(byte* data)
{
#ifdef THN132N
    int s = ((Sum(6, data) + (data[6]&0xF) - 0xa) & 0xff);
 
    data[6] |=  (s&0x0F) << 4;     data[7] =  (s&0xF0) >> 4;
#else
    data[8] = ((Sum(8, data) - 0xa) & 0xFF);
#endif
}
 /***************** End send 433 *************************************************/
/******************************************************************/
/******************************************************************/
long readVccFixedLowPower() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(1); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = ( 1023L * 1100L) / result; // Back-calculate AVcc in mV. 1100 is volt on vRef.
  //to fix lowpower drain ADMUX is changed again.
  ADMUX = _BV(REFS1); //fixes readVcc current drain in lowPower
  Serial.print("Vcc = " );
  Serial.println(result); 
  return result;
}
