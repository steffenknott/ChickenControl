// Base
#include "Arduino.h"
// I2C
#include "Wire.h" // I2C for RTC, BME280 and BH1750
#include "uRTCLib.h" // RTC
#include <BH1750.h> // Light Sensor
#include <Adafruit_ADS1015.h> // 12 bit ADC for current measurement
// Ethernet via EtherCard (SPI)
#include <EtherCard.h>
#include <IPAddress.h>
// RFID Reader via Wiegand library
#include <Wiegand.h>



// SETTINGS
// Location
#define LATITUDE 48.25
#define LONGITUDE 10.98
#define UTCOFFSET 1
// Astroautomatic mode
#define SUNRISEOFFSET 0.5 // Open x hours after sunrise
#define SUNDOWNOFFSET 0.5 // Close x hours after sundown
// Lightsensor mode
#define LIGHTOPENTHRESHOLD 50 // Lux needed before door opens
#define LIGHTCLOSETHRESHOLD 5 // Lux level that triggers door close
#define LIGHTGRACESECS 180 // seconds to wait after threshold has been crossed before action is triggered

// Pins coop door related
#define PINCOOPDOORMANUALUP 22
#define PINCOOPDOORMANUALDOWN 23
#define PINCOOPDOORMANUALSTOP 24
#define PINCOOPDOORMODEASTRO 25
#define PINCOOPDOORMODELIGHT 26
#define PINCOOPDOORMODEMANUAL 27
#define PINCOOPDOORUPPEREND 28
#define PINCOOPDOORLOWEREND 29
#define PINCOOPDOORMOTORUP 8 //PWM PIN for motor
#define PINCOOPDOORMOTORDOWN 9 //PWM PIN for motor

// Pins water related
#define PINWATERLEVEL 32

// Pins RFID related
#define PINRFID1D0 2
#define PINRFID1D1 3
#define PINRFID2D0 4
#define PINRFID2D1 5

// Pins power related
#define PINAMPSPANEL2CONTROLLER A1
#define PINAMPSCONTROLLER2BATTERY A2
#define PINAMPSCONTROLLER2LOAD A3
#define PINVOLTAGEPANEL A4
#define PINVOLTAGEBATTERY A5

// STATUS VARIABLES
bool CoopdoorUp = false;
bool CoopdoorDown = false;
bool LightsensorLastSensedState = false; // false = no light, true = light
bool LightsensorCurrentHardState = false;
unsigned long LightsensorLastChange = 0;

// TIMERS
int timerFreqSendMeasurements = 15000;
unsigned long timerLastSendMeasurements = 0;

// RTC
  uRTCLib rtc(0x68, 0x57);
  //  rtc.set(0, 10, 11, 2, 29, 5, 18);
  //  RTCLib::set(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year)

// Light Sensor
  BH1750 lightMeter(0x23);

// RFID reader
  WIEGAND wg1;
  WIEGAND wg2;

// Ethernet
  #define USEDHCP 1 // set to 0 to use a static address
  #if STATIC
    // ethernet interface ip address
    static byte myip[] = { 192,168,0,200 };
    // gateway ip address
    static byte gwip[] = { 192,168,0,1 };
  #endif
  static byte mymac[] = { 0x70,0x69,0x69,0x2D,0x30,0x31 };
  byte Ethernet::buffer[500]; // tcp/ip send and receive buffer
  byte serveraddr[] = {10,0,10,172};
  uint16_t serverport = 1337;
  uint16_t localport = 1338;

// ADC
  Adafruit_ADS1015 adc;
  
//TEST

int motPower = 0;
bool motReverse = false;
bool warnung=false;
//END TEST



///////////////////////////////////////////////////////////////
//
// SETUP ROUTINE
//
///////////////////////////////////////////////////////////////
  
void setup() {
  // init serial comms
    Serial.begin(9600);

  // Ethernet
    if (ether.begin(sizeof Ethernet::buffer, mymac) == 0)
    Serial.println(F("Failed to access Ethernet controller"));
    #if STATIC
      ether.staticSetup(myip, gwip);
    #else
      if (!ether.dhcpSetup())
        Serial.println(F("DHCP failed"));
    #endif
    while (ether.clientWaitingGw())
      ether.packetLoop(ether.packetReceive());
    ether.printIp("IP:  ", ether.myip);
    ether.printIp("GW:  ", ether.gwip);
    ether.printIp("DNS: ", ether.dnsip);
    //register command processor to port 42.
    ether.udpServerListenOnPort(&udpProcessCommand, 42);
  
  // init I2C (RTC, BH1750)
    Wire.begin();

  // Set digital pin modes
    pinMode(PINCOOPDOORMANUALUP, INPUT_PULLUP);
    pinMode(PINCOOPDOORMANUALDOWN, INPUT_PULLUP);
    pinMode(PINCOOPDOORMANUALSTOP, INPUT_PULLUP);
    pinMode(PINCOOPDOORMODEASTRO, INPUT_PULLUP);
    pinMode(PINCOOPDOORMODELIGHT, INPUT_PULLUP);
    pinMode(PINCOOPDOORMODEMANUAL, INPUT_PULLUP);
    pinMode(PINCOOPDOORUPPEREND, INPUT_PULLUP);
    pinMode(PINCOOPDOORLOWEREND, INPUT_PULLUP);
    pinMode(PINCOOPDOORMOTORUP, OUTPUT);
    pinMode(PINCOOPDOORMOTORDOWN, OUTPUT);
    // Pins water related
    pinMode(PINWATERLEVEL, INPUT_PULLUP);

  // RFID
    wg1.begin(PINRFID1D0, PINRFID1D1);
    wg2.begin(PINRFID2D0, PINRFID2D1);

  // Light Meter BH1750
    if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE))
    {
      Serial.println(F("BH1750 Advanced begin"));
    }
    else
    {
      Serial.println(F("Error initialising BH1750"));
    }

  // I2C ADC
    adc.setGain(GAIN_ONE);
    adc.begin();


}

void loop() {
  rtc.refresh();
  ether.packetLoop(ether.packetReceive());
  
  float aufgang = getSunrise(getDayOfYear());
  float untergang = getSundown(getDayOfYear());

  testkram();

  if (millis() > timerLastSendMeasurements + timerFreqSendMeasurements)
  {
    sendMeasurements();
    timerLastSendMeasurements = millis();
  }
  
  Serial.print("Tag >");
  Serial.print(getDayOfYear());
  Serial.print("<: Aufgang >");
  Serial.print(aufgang);
  Serial.print("< Untergang >");
  Serial.print(untergang);
  Serial.println("<");
  delay(1000);
}

void testkram()
{
  if (digitalRead(PINWATERLEVEL) == warnung)
  {
    // no change
  }
  else
  {
    warnung = digitalRead(PINWATERLEVEL);
    Serial.print("Wasserstand ");
    Serial.println(warnung ? "niedrig" : "ok");
    char text[] = "Wasserstand!\n";
    send(text);
  }
  

  if (!digitalRead(PINCOOPDOORLOWEREND) == CoopdoorDown)
  {
    // no change
  }
  else
  {
    CoopdoorDown= !digitalRead(PINCOOPDOORLOWEREND);
    Serial.print("Tor ");
    Serial.println(CoopdoorDown ? "unten" : "nicht unten");
    if (CoopdoorDown)
      send("Tor unten!\n");
    else
      send("Tor nicht unten!\n");
  }

    // put your main code here, to run repeatedly:
  if (!digitalRead(PINCOOPDOORUPPEREND) == CoopdoorUp)
  {
    // no change
  }
  else
  {
    CoopdoorUp=!digitalRead(PINCOOPDOORUPPEREND);
    Serial.print("Tor ");
    Serial.println(CoopdoorUp? "oben" : "nicht oben");
    if (CoopdoorUp)
      send("Tor oben!\n");
    else
      send("Tor nicht oben!\n");

  }

  if(wg1.available())
  {
    Serial.print("Wiegand HEX = ");
    Serial.print(wg1.getCode(),HEX);
    Serial.print(", DECIMAL = ");
    Serial.print(wg1.getCode());
    Serial.print(", Type W");
    Serial.println(wg1.getWiegandType());
    char buf [8];
    sprintf (buf, "%07i", wg1.getCode());    
    send("Chicken >");
    send(buf);
    send("< laid an egg in nest 1!\n");
  }


}


///////////////////////////////////////////////////////////////
//
// hard status functions (read from sensors)
//
///////////////////////////////////////////////////////////////

bool getDoorIsOpen()
{
  return !digitalRead(PINCOOPDOORUPPEREND);
}

bool getDoorIsClosed()
{
  return !digitalRead(PINCOOPDOORLOWEREND);
}


///////////////////////////////////////////////////////////////
//
// Amps sensors
//
///////////////////////////////////////////////////////////////

int getAmpsP2C()
{
    int amp = 1000* (analogRead(PINAMPSPANEL2CONTROLLER) - 511) / 20.0;
    Serial.println(analogRead(PINAMPSPANEL2CONTROLLER));
    Serial.println(amp);
    return amp;
}


///////////////////////////////////////////////////////////////
//
// Light sensor functions (BM1750)
//
///////////////////////////////////////////////////////////////

uint16_t getLux()
{
    uint16_t lux = lightMeter.readLightLevel();
    return lux;
}

bool getLightsensorDoorState()  // true = open, false = closed
{
    bool state = false;
    uint16_t lux = lightMeter.readLightLevel();
    Serial.print("Light: ");
    Serial.print(lux);
    Serial.println(" lx");

    if (lux >= LIGHTOPENTHRESHOLD)
      state = true;
    else if (lux > LIGHTCLOSETHRESHOLD && getDoorIsOpen)
      state = true;
    else if (lux > LIGHTCLOSETHRESHOLD && getDoorIsClosed)
      state = false;
    else
      state = false;

    if (state != LightsensorLastSensedState)
    {
      LightsensorLastChange = millis();
    }

    if (LightsensorLastChange + LIGHTGRACESECS < millis())
    {
      LightsensorCurrentHardState = state;
      return state;
    }
    else
    {
      return LightsensorCurrentHardState;
    }
}


void flipPin(int pin)
{
  if (digitalRead(pin))
    digitalWrite(pin, LOW);
  else
    digitalWrite(pin, HIGH);
}

///////////////////////////////////////////////////////////////
//
// Ethernet functions
//
///////////////////////////////////////////////////////////////

void udpProcessCommand(uint16_t dest_port, uint8_t src_ip[IP_LEN], uint16_t src_port, const char *data, uint16_t len){
  IPAddress src(src_ip[0],src_ip[1],src_ip[2],src_ip[3]);
  
  Serial.print("dest_port: ");
  Serial.println(dest_port);
  Serial.print("src_port: ");
  Serial.println(src_port);
  
  emptySpaces(data);
  
  Serial.print("src_port: ");
  ether.printIp(src_ip);
  Serial.print("data: >");
  Serial.print(data);
  Serial.println("<");

  if (String(data) == "DoorUp")
  {
    digitalWrite(LED_BUILTIN, LOW);
  }
  else if (String(data) == "DoorStop")
  {
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else if (String(data) == "R1")
  {
    flipPin(40);
  }
  else if (String(data) == "R2")
  {
    flipPin(41);
  }
  else if (String(data) == "R3")
  {
    flipPin(42);
  }
  else if (String(data) == "R4")
  {
    flipPin(43);
  }
  else if (String(data) == "+")
  {
    motPower = motPower+50;
    if (motPower >1023)
      motPower=1023; 
  }
  else if (String(data) == "-")
  {
    motPower = motPower-50;
    if (motPower < 0)
    motPower =0;
  }
  else if (String(data) == "x")
  {
    motReverse = !motReverse;
  }
  else
  {
    Serial.println("Command not implemented.");
  }
  if (motReverse)
  {
    analogWrite(PINCOOPDOORMOTORUP, motPower);
    analogWrite(PINCOOPDOORMOTORDOWN,0);
  }
  else
  {
    analogWrite(PINCOOPDOORMOTORDOWN, motPower);
    analogWrite(PINCOOPDOORMOTORUP,0);
  }
}

void sendMeasurements()
{
    char myConcatenation[80];
    sprintf(myConcatenation,"lux = %i\namps = %i mA\n", getLux(), getAmpsP2C());
    send(myConcatenation);
}

void send(char textToSend[])
{
    ether.sendUdp(textToSend, strlen(textToSend), localport, serveraddr, serverport ); 
}

void emptySpaces(char* my_string)
{
    char* c = my_string;
    char* k = my_string;
    while(*k != 0)
    {
        *c = *k++;
        if(*c != ' ' && *c != '\n' && *c != '\r' && *c != '\f' && *c != '\t' && *c != '\v')
            c++;
    }
    *c = 0;
}

///////////////////////////////////////////////////////////////
//
// Door control functions
//
///////////////////////////////////////////////////////////////

void controlCoopDoor()
{
  // check which mode is active
  if (digitalRead(PINCOOPDOORMODEASTRO) == 0)
  {
    doorAstro();
  }
  
  if (digitalRead(PINCOOPDOORMODELIGHT) == 0)
  {
    doorLightsensor();
  }
  
  if (digitalRead(PINCOOPDOORMODEMANUAL) == 0)
  {
    doorManual();
  }


  if (CoopdoorDown == true && digitalRead(PINCOOPDOORLOWEREND) != 0)
  {
    //TODO run motor downwards
  }
  else if (CoopdoorUp == true && digitalRead(PINCOOPDOORUPPEREND) != 0)
  {
    //TODO run motor upwards
  }
  else
  {
    //TODO stop motor
  }
  
}

void doorAstro()
{
  float currentHourOfDay = rtc.hour() + (rtc.minute()/60);
  if (currentHourOfDay > getSunrise(getDayOfYear()) + SUNRISEOFFSET && currentHourOfDay < getSundown(getDayOfYear()) + SUNDOWNOFFSET)
  {
    CoopdoorUp = true;
    CoopdoorDown = false;
  }
  else
  {
    CoopdoorUp = false;
    CoopdoorDown = true;
  }
}

void doorLightsensor()
{
  CoopdoorUp = getLightsensorDoorState() ? true : false;
  CoopdoorDown = !CoopdoorUp;
}

void doorManual()
{
  if (digitalRead(PINCOOPDOORMANUALDOWN) == 0)
  {
    CoopdoorUp = false;
    CoopdoorDown = true;
  }
  if (digitalRead(PINCOOPDOORMANUALUP) == 0)
  { // UP has priority over DOWN
    CoopdoorUp = true;
    CoopdoorDown = false;
  }
  if (digitalRead(PINCOOPDOORMANUALSTOP) == 0)
  { // STOP has priority over UP and DOWN
    CoopdoorUp = false;
    CoopdoorDown = false;
  }
}

///////////////////////////////////////////////////////////////
//
// RTC functions
//
///////////////////////////////////////////////////////////////

int getDayOfYear() {
  int daysPerMonth[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  int dayOfYear=0;
  int month = 0;
  if (rtc.month()>1)
  {
    while (month < rtc.month() - 1)
    {
      dayOfYear += daysPerMonth[month];
      month++;
    }
  }
  dayOfYear += rtc.day();
  Serial.println("month=" + rtc.month());
  return dayOfYear;
}


///////////////////////////////////////////////////////////////
//
// Berechnung Sonnenaufgang/-untergang
//
///////////////////////////////////////////////////////////////

// die folgenden Formeln wurden der Seite http://lexikon.astronomie.info/zeitgleichung/ entnommen
// Codebasis: https://www.arduinoforum.de/arduino-Thread-Sonnenaufgang-untergang-für-Steuerungen-berechnen

// subfunction to compute Sonnendeklination 
float sonnendeklination(int T) {
    // Deklination der Sonne in Radians
    // Formula 2008 by Arnold(at)Barmettler.com, fit to 20 years of average declinations (2008-2017)
    return 0.409526325277017*sin(0.0169060504029192*(T-80.0856919827619)); 
}

// subfunction to compute Zeitdifferenz    
float zeitdifferenz(float Deklination, float B) {
    // Dauer des halben Tagbogens in Stunden: Zeit von Sonnenaufgang (Hoehe h) bis zum hoechsten Stand im Sueden
    return 12.0*acos((sin(-(50.0/60.0)*PI/180.0) - sin(B)*sin(Deklination)) / (cos(B)*cos(Deklination)))/PI;
}

// subfunction to compute Zeitgleichung
float zeitgleichung(int T) {
    return -0.170869921174742*sin(0.0336997028793971 * T + 0.465419984181394) - 0.129890681040717*sin(0.0178674832556871*T - 0.167936777524864);
}

// subfunction to compute sunrise 
float getSunrise(int dayOfYear) {
    float DK = sonnendeklination(dayOfYear);
    return 12 - zeitdifferenz(DK, LATITUDE*PI/180.0) - zeitgleichung(dayOfYear) - LONGITUDE /15.0 + UTCOFFSET;
}

// subfunction to compute sunset
float getSundown(int dayOfYear) {
    float DK = sonnendeklination(dayOfYear);
    return 12 + zeitdifferenz(DK, LATITUDE*PI/180.0) - zeitgleichung(dayOfYear) - LONGITUDE /15.0 + UTCOFFSET;
}

