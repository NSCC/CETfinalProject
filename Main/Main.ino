//Libraries for the pressure, altitude and temperature sensor
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

//Library for the servo
#include <Servo.h>

//Library for the SD card reader
#include <SD.h>

//Libraries for the GPS module on the LORA/GPS shield
#include <NMEAGPS.h>
#include <GPSport.h>

//Library for managing the SPI pins of the leonardo
#include <SPI.h>

//Library for the LORA module on the LORA/GPS shield
#include <LoRa.h>

//Declaring variables from the GPS Library
NMEAGPS  gps; // This parses the GPS characters
gps_fix  fix; // This holds on to the latest values

//Declaring variables for the program
int           SDChipS = 7;            //Chip select for the SD cacrd reader
int           LoRaChipS = 10;         //Chip select for the LORA/GPS shield
int           BMPChipS = 6;           //Chip select for the pressure sensor
int           pos = 0;                //Position of the servo
float         lat, lon;               //Cases for latitude and longitude
float         temp, Pa, alt, altLast; //Temperature, pressure and altitude
float         Time = 0;               //Times for calculating speed
float         lastTime;          
float         Speed;                  //How fast it's going (m/s)



//Declaring variables from the SD card reader Library
File GPSData, BMPData;

//Creating servo object for the release
Servo releaser;  

//Setting the Chip select of the pressure sensor to the module
Adafruit_BMP280 bmp(BMPChipS); // hardware SPI

void digitalSwitch(int S, int B, int L){
  digitalWrite(SDChipS, S);
  digitalWrite(BMPChipS, B);
  digitalWrite(LoRaChipS, L);
  
  }

//setup function (only once)
void setup() {
  // initialize both serial ports:
  gpsPort.begin(9600);
  Serial.begin(9600);
  bmp.begin();

  releaser.attach(9); //setting servo to pin 9

  while(!Serial){;
  }
  
  //Setting the pin modes for all modules
  pinMode(LoRaChipS, OUTPUT);
  pinMode(SDChipS, OUTPUT);
  pinMode(BMPChipS, OUTPUT);

  //Setting the LORA output to LOW and all others to HIGH
  digitalSwitch(1,1,0);

  Serial.print("Initializing LORA...");

  //Initializing the LORA and checking for success
  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    //Don't do anything more:
    while(1);
  }
  
  Serial.println("LoRa started");

  //Setting the SD card reader output to LOW and all others to HIGH
  digitalSwitch(0,1,1);

  Serial.print("Initializing SD card...");

  //Initializing the SD card reader and checking for success
  if (!SD.begin(SDChipS)) {
    Serial.print("Card failed, or not present");
    //Don't do anything more:
    while(1);
  }
  
  Serial.println("SD card initialized");
    
  //Setting the pressure sensor output to LOW and all others to HIGH
  digitalSwitch(1,0,1);

  Serial.print("Initializing BMP280 sensor...");

  //Initializing the pressure sensor and checking for success
  if (!bmp.begin()) {  
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while(1);
  }

  Serial.print("BMP sensor initialized.");
  Serial.println("");
  Serial.println("All modules are green");
  Serial.println("");
  Serial.println("___________________________________________");
  Serial.println("");

  delay(2000);
 
}

void Data(){
    //Switching modules to the LORA/GPS shield for usage
    digitalSwitch(1,1,0);
  
    //If a reading can be recieved on the gps
    while (gps.available( gpsPort )) {
    //take the reading
    fix = gps.read();

    //Open a transmission to the reciever
    LoRa.beginPacket();

    //If there's a fix, save and transmit latitude and longitude
    if (fix.valid.location) {
      lat = fix.latitude();
      LoRa.print( fix.latitude(), 6) ;   
      lon = fix.longitude();
      LoRa.print( fix.longitude(), 6 );
      LoRa.print(alt);
    }
   }

   LoRa.println();
   LoRa.endPacket();

    //Switching modules to the pressure sensor for usage
    digitalSwitch(1,0,1);

    //Read temperature
    temp = bmp.readTemperature();
    Serial.println(bmp.readTemperature());

    //Read Altitude
    altLast = alt;
    alt = bmp.readAltitude(1013.25); //Altitude property should be set to estimate current altitude
    Serial.println(bmp.readAltitude(1013.25));
    
    //Read Atmospheric pressure
    Pa = bmp.readPressure();
    Serial.println(bmp.readPressure());

    //Calculating estimate rise or fall speed
    Speed = (alt-altLast)/(Time-lastTime); //Meters per second
    
    
    //End the transmission to the reciever   
    
  
  }

void DataSave(){
    //Switching modules to the SD card reader being used
    digitalSwitch(0,1,1);


    //Writing a txt file for use (if the file doesn't exist, it will create it.)
    GPSData = SD.open("GPSData.txt", FILE_WRITE);
    BMPData = SD.open("BMPData.txt", FILE_WRITE);

    //If the file is a valid one, save all information collected above in a CSV format
    if(GPSData){
      Serial.println("Writing to SD Card");
      GPSData.print(lat, 6);
      GPSData.print(",");
      GPSData.print(lon, 6);
      GPSData.print(",");
      GPSData.print(alt, 6);
      GPSData.print(" m");
      GPSData.println();
      GPSData.close();
      BMPData.print(temp);
      BMPData.print(" *C,");
      BMPData.print(Pa);
      BMPData.print("Pa,");
      BMPData.print(abs(Speed), 3);
      BMPData.print(" m/s,");
      BMPData.print(Time);
      BMPData.print(" seconds");
      BMPData.println();
      BMPData.close();
    }
    else{
      Serial.println("Error opening file");
    }
  }

void drop(){
    //Release helium balloon
    for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    releaser.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
    }
        

     //Release chute

     //return to the loop
  
  }

void loop() {

    //
    delay(1000);

    lastTime = Time;
    Time = millis()/1000;
    
    //LORA and BMP function
    Data();
    //SD reader function
    DataSave();

    //If the altitude is greater than 1000 meters
    if(alt >= 1000){
      drop();
    }

    //If the longitude or latitude is past a certain area
    if((lat <= -1)|| lon <= -1){
      drop();
    }

    //If a set amount of time has passed
    if(Time >= 12){ //ten minutes      
      drop();
    }

    //If the temperature drops below -37.5 degrees Celsius
    if(temp <= -37.5){
      drop();
    }
  }






