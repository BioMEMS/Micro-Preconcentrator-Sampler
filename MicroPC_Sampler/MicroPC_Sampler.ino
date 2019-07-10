/*
  © The Regents of the University of California, Davis campus, 2019. 
  This material is available as open source for research and personal use 
  under a Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 
  International Public License 
  (https://creativecommons.org/licenses/by-nc-nd/4.0/)
  
  MicroPC_Sampler
  
  Uses an Adafruit Feather 32u4 Adalogger with the Adafruit Ultimate GPS FeatherWing to control
  sampling time and sampling frequency of the micro-preconcentrator sampler.  Data is recorded on the
  SD card.
  
  
  Journal Citation: 
  A.G. Fung, M.Y. Rajapakse, M.M. McCartney, A.K. Falcon, F.M. Fabia, N.J. Kenyon, and C.E. Davis, 
  "Wearable Environmental Monitor to Quantify Personal Ambient Voltatile Organic Compound Exposures,"
  ACS Sensors, vol. 4, no. 5 p 1358-1364, DOI: 10.1021/acssensors.9b00304 
 */

#include <Adafruit_GPS.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>  

#define GPSSerial Serial1
//#define GPSECHO true

#define CS_SD_PIN 4     //SD Card Chip Select Pin
#define BAT_PIN 9       //Battery Voltage Monitoring Pin
#define MOTORPWM_PIN 5  //PWM Pin that is used for the motor controller board
#define N_SAMPLES 1    //Number of samples to be taken, limited by memory
#define PWM_VAL 125     //Sample pump strength Values Range from 0-255
#define YEARPREFIX 20   //Year Prefix for Code.  Will need to be updated in 2100

#define H2M 60      //Number of minutes in an hour
#define MPD 60*24   //Number of minutes per day

///////////////////////////////////////////////////////////////
const int sampling_time=10; //Sampling time in minutes
//////////////////////////////////////////////////////////////

Adafruit_GPS GPS(&GPSSerial);  //Global GPS variable object
File dataFile;                 //Global dataFile object for SD card
bool samplingFlag=0;           //Indicates whether sampling  (1) or not (0)
unsigned int sampleNumber=0;   //Current sample number
const char filename[]="labtest.txt"; //Character Limit to filename
const int nextSample=H2M; //Time between sampling runs

//In GPS libary all times are uint8_t variables
/* onOffTimes contains the start/end time for each sample in terms of minutes
 This is due to memory constraints as including hours and minutes takes up
 too much memory.  Hours can easily be determine via _combined/60 and minutes
 via _combined%60
 timeWrap indicates if the end time happens on a different day
 With the GPS, all times are Greenwich Mean Time
*/
struct onOffTimes{
   int b_combined;  //Beginning time for sample in minutes
   int e_combined;  //Ending time for sample in minutes
  bool timeWrap;
} times[N_SAMPLES];

uint8_t event_trigger_seconds;  //The time is seconds that runs start and end

//Function Declarations

//Initializes times for sampling
void initializeTimes();
//Reads the temperature and humidity sensor
void RHTreading(float& humidity,float& temperature);
//Checks the time to start/stop sampling
int checkTime();

void setup()
{

  //Establish serial communication for debugging/verification issues

  Serial.begin(115200);

//Base PWM Freqeuncy is 245 HZ, need +15kHz for Motor control
//Change prescale factor on Timer 3 (8-bit phase correct PWM Mode) from 64 to 1
//Results in PWM Frequency of 31372Hz on pin 6.
//Last 3 bits of TCCR3B control prescale factor bitwise & keeps the rest of the
//registers the same
//Bitwise or just changes the prescale factor
  TCCR3B = (TCCR3B & 0b11111000) | 0x01;

 //Set Motor pin to Output for PWM signal
 pinMode(MOTORPWM_PIN, OUTPUT);
 //Write to 0 to stop motor as it typically starts running
 analogWrite(MOTORPWM_PIN,0);

 //Setup I2C communication line
 Wire.begin();

 //Delay 2 seconds to let user see serial communication
 delay(2000);
 //Verify SD Card works Could make a Function for this...
   Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(CS_SD_PIN)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

  dataFile = SD.open(filename, FILE_WRITE); //File_WRITE creates and appends

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println("Starting new session");
    dataFile.print("Sampling time: ");
    dataFile.print(sampling_time);
    dataFile.println(" minutes");
    dataFile.println("Date,Time,Fix,Latitude (Degrees),Longitude (Degrees),RH,Temp,VBat");
    dataFile.close();
    // print to the serial port too,
    Serial.println("Starting new session");
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.print("error opening ");
    Serial.println(filename);
  }
  ///////////////////////////////////////
  //Initialize GPS
  //Default communications rate is 9600
  GPS.begin(9600);
  //GPS.sendCommand("$PMTK225,0*2B");
delay(500);
  //UTC Time, Postion, course, speed, date
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  delay(500);
  //update position every 10 seconds
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_200_MILLIHERTZ);
  delay(500);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ);
//send data every 10 seconds
  
  delay(500);



  //Get first timepoint
  //3 acknowledgements, 1 actual signal, sometimes signal comes first,
  // need to wait for last one then
  for (int i=1;i<9;i++)
  {
    while (!GPS.newNMEAreceived())
    {
      char c=GPS.read();
    }
    delay(10);
    char *gps_ptr =GPS.lastNMEA();
    Serial.println(gps_ptr);
    GPS.parse(gps_ptr);

  }
  initializeTimes(); //Initialize the starting times
  dataFile = SD.open(filename, FILE_WRITE); //File_WRITE creates and appends

  // if the file is available, write to it:
  if (dataFile) {
    //List all starting times
    for(int i=0;i<N_SAMPLES;i++)
    {
      dataFile.println("Sampling starting time: ");
      dataFile.print(times[i].b_combined/60);
      dataFile.print(':');
      dataFile.print(times[i].b_combined%60);
      dataFile.print(':');
      dataFile.println(event_trigger_seconds);
      dataFile.println("Sampling end time: ");
      dataFile.print(times[i].e_combined/60);
      dataFile.print(':');
      dataFile.print(times[i].e_combined%60);
      dataFile.print(':');
      dataFile.println(event_trigger_seconds);
    }
      dataFile.close();

  }
  //Print all times to Serial port for verification
  for(int i=0;i<N_SAMPLES;i++)
    {
      Serial.println("Sampling starting time: ");
      Serial.print(times[i].b_combined/60);
      Serial.print(':');
      Serial.print(times[i].b_combined%60);
      Serial.print(':');
      Serial.println(event_trigger_seconds);
      Serial.println("Sampling end time: ");
      Serial.print(times[i].e_combined/60);
      Serial.print(':');
      Serial.print(times[i].e_combined%60);
      Serial.print(':');
      Serial.println(event_trigger_seconds);
    }
  Serial.println("List Complete");
}
/////////////////////////////////////////////////////

void loop()
{
  static char *gps_ptr; //Contains GPS messages
  static float temp=0;  //Temperature Variable
  static float rh=0;    //Relative humidity variable
  //Keep looping until new GPS message received
  float  vBat; //Voltage for battery
  char c;

    while (!GPS.newNMEAreceived())
    {
      c=GPS.read();
    }
 //When new GPS message recevied, check times and sensor
  if (GPS.newNMEAreceived())
  {
    //Print to serial port for verification
    Serial.println("Sampling starting time: ");
    Serial.print(times[sampleNumber].b_combined/60);
    Serial.print(':');
    Serial.print(times[sampleNumber].b_combined%60);
    Serial.print(':');
    Serial.println(event_trigger_seconds);
    Serial.println("Sampling end time: ");
    Serial.print(times[sampleNumber].e_combined/60);
    Serial.print(':');
    Serial.print(times[sampleNumber].e_combined%60);
    Serial.print(':');
    Serial.println(event_trigger_seconds);

    //Update GPS pointer to parse
    gps_ptr = GPS.lastNMEA();
    //Print to serial for verification
    Serial.print(gps_ptr);

    if (!GPS.parse(gps_ptr))//Also sets the newNMEAreceived() flag to false
      return;  //If parsing fails, start over
    RHTreading(rh,temp);//Get temperature and humidity

    //vBat converstion
    //2 for voltage diver, 3.3 for reference voltage, 1024 for 10 bit
    vBat= analogRead(BAT_PIN)*6.6/1024.0;
    //Print parsed data for verification purposes
      Serial.print(YEARPREFIX);
      Serial.print(GPS.year);
      Serial.print('-');
      Serial.print(GPS.month);
      Serial.print('-');
      Serial.print(GPS.day);
      Serial.print('T');
      Serial.print(GPS.hour);
      Serial.print(':');
      Serial.print(GPS.minute);
      Serial.print(':');
      Serial.print(GPS.seconds);
      Serial.print('.');
      Serial.print(GPS.milliseconds);
      Serial.print('Z');
      Serial.print(',');
      Serial.print(GPS.fix);
      Serial.print(',');
      Serial.print(GPS.latitudeDegrees,6);
      Serial.print(',');
      Serial.print(GPS.longitudeDegrees,6);
      Serial.print(',');
      Serial.print(rh);
      Serial.print(',');
      Serial.print(temp);
      Serial.print(',');
      Serial.println(vBat);

    if(checkTime()) //if sampling, record the data
    {
      Serial.println("Sampling");
      dataFile = SD.open(filename, FILE_WRITE);
    // if the file is available, write to it:
    if (dataFile) {
      dataFile.print(YEARPREFIX);
      dataFile.print(GPS.year);
      dataFile.print('-');
      dataFile.print(GPS.month);
      dataFile.print('-');
      dataFile.print(GPS.day);
      dataFile.print('T');
      dataFile.print(GPS.hour);
      dataFile.print(':');
      dataFile.print(GPS.minute);
      dataFile.print(':');
      dataFile.print(GPS.seconds);
      dataFile.print('.');
      dataFile.print(GPS.milliseconds);
      dataFile.print('Z');
      dataFile.print(',');
      dataFile.print(GPS.fix);
      dataFile.print(',');
      dataFile.print(GPS.latitudeDegrees,6);
      dataFile.print(',');
      dataFile.print(GPS.longitudeDegrees,6);
      dataFile.print(',');
      dataFile.print(rh);
      dataFile.print(',');
      dataFile.print(temp);
      dataFile.print(',');
      dataFile.println(vBat);
      dataFile.close();
     }
    }
  }
}

//Checks whether sampling should occur based on times
//Returns 1 if sampling, returns 0 if not
int checkTime()
{
  int testTime=GPS.hour*H2M+GPS.minute;//Convert to combined time
  //If not sampling, check to see if should be
  //Serial.println(testTime);
  if (!samplingFlag)
  {
    //Case if Sampling time wraps to next day
    if(times[sampleNumber].timeWrap && testTime>=
       times[sampleNumber].b_combined && testTime>
       times[sampleNumber].e_combined)
        {
          if (GPS.seconds>=event_trigger_seconds||testTime> times[sampleNumber].b_combined) //Check seconds
          { samplingFlag=1;
            //Set pump
            analogWrite(MOTORPWM_PIN, PWM_VAL);
            Serial.println("Sampling");
            return 1;
          }
        }
        //Case with sampling happening same day
    else if (testTime>= times[sampleNumber].b_combined &&
       testTime<times[sampleNumber].e_combined)
          {
            //Serial.println("Check seconds");
            if (GPS.seconds>=event_trigger_seconds||testTime> times[sampleNumber].b_combined) // check seconds
            { samplingFlag=1;
              //Serial.println(GPS.seconds);
              //Set pump
              analogWrite(MOTORPWM_PIN, PWM_VAL);
              Serial.println("Sampling");
              return 1;
            }
          }

    else
    {
      return 0;
    }
  }
  //If it is sampling, check to see if it should stop
  else if((times[sampleNumber].timeWrap && testTime>=
    times[sampleNumber].e_combined &&
    testTime<times[sampleNumber].b_combined)||
    (times[sampleNumber].timeWrap==0 &&
    testTime>=times[sampleNumber].e_combined))
    {
      if (GPS.seconds>=event_trigger_seconds||testTime>times[sampleNumber].e_combined)
          { //stop sampling, stop pump, stop logging data
            samplingFlag=0;
            Serial.println("Sampling stopped");
            //Set pump
            analogWrite(MOTORPWM_PIN, 0);
            sampleNumber++;
            if (sampleNumber>N_SAMPLES-1)
              sampleNumber=0;
            return 1;
          }
    }
    else
    { return 1;}
}

//Humidity and temperature are passed by reference and thus will change in loop
void RHTreading(float& humidity,float& temperature)
{
  unsigned int reading=0;
//Default address is 0x27, 39
//Enter command mode and set to normal mode
//Unable to send just the write byte that is needed to sample with Wire library
//So send three bytes, 0x80, 0x00, 0x00
  Wire.beginTransmission(39);
  //Serial.println("Transmission Started");
  (Wire.write(byte(0x80)));
  (Wire.write(byte(0x00)));
  (Wire.write(byte(0x00)));
  (Wire.endTransmission());
 //Needs 42.5ms to exit command mode
  delay(50);

 // Serial.println("Request");
  Wire.requestFrom(39,4);
  if (4 <= Wire.available()) { // if two bytes were received
    reading = Wire.read();  // receive high byte (overwrites previous reading)
    reading = reading << 8;    // shift high byte to be high 8 bits
    reading |= Wire.read(); // receive low byte as lower 8 bits
    if (reading>16384)
    {
      reading=-1; //Returns a negative number
    }
    humidity=reading/163.82; //Equation from datasheet

    reading = Wire.read();  // receive high byte (overwrites previous reading)
    reading = reading << 8;    // shift high byte to be high 8 bits
    reading |= Wire.read(); // receive low byte as lower 8 bits
    reading = reading>>2;
    if (reading>16384)
    {
      reading=-1;//Will return below -40C
    }
    temperature=reading/(16382.0)*165.0-40.0;//Equation from datasheet
  }

}

void initializeTimes()
{
//GPS time given in UTC

  //Tigger time in seconds after next GPS reading
  event_trigger_seconds=GPS.seconds+10;
  if (event_trigger_seconds>=60)
  {
    event_trigger_seconds-=60;
    times[0].b_combined=GPS.hour*H2M+GPS.minute+1;
  }
  else
  {
    times[0].b_combined=GPS.hour*H2M+GPS.minute;
  }
  times[0].e_combined=times[0].b_combined+sampling_time;
  //Serial.println(times[0].e_combined);
  if (times[0].e_combined>=MPD)
  {
    times[0].e_combined-=MPD;
    times[0].timeWrap=1;
  }
  else
  {
    times[0].timeWrap=0;
  }

  //
  for (int i=1;i<N_SAMPLES;i++)
  {
    //Calculate next sample time
    times[i].b_combined=times[i-1].b_combined+nextSample;
    //Check in sampling happens on the next day
    if (times[i].b_combined>=MPD)
    {
      times[i].b_combined-=MPD; //subtract a day's worth of time if so
    }
    //Calculate ending time
    times[i].e_combined=times[i].b_combined+sampling_time;
    //Check if ending time is on the next day
    if (times[i].e_combined>=MPD)
    {
      times[i].e_combined-=MPD; //subtract a day's worth of time if so
      times[i].timeWrap=1;  //Indicate a time wrap happend
    } else
    {
      times[i].timeWrap=0; //Same day sampling
    }
   }
}
