//Libraries for the pressure, altitude and temperature sensor
#include <Adafruit_BMP280.h>
//#include <Adafruit_Sensor.h>
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

//Declaring variables for the program
#define  SDChipS  7            //Chip select for the SD cacrd reader
#define  LoRaChipS 10          //Chip select for the LORA/GPS shield
#define  BMPChipS 6            //Chip select for the pressure sensor

int         pos = 0;                //Position of the servo
int         lat, lon;               //Cases for latitude and longitude
int         temp, Pa, alt, altLast; //Temperature, pressure and altitude
int         Time = 0;               //Times for calculating speed
int         lastTime;          
int         Speed;                  //How fast it's going (m/s)



//Declaring variables from the SD card reader Library
File data;

//Creating servo object for the release
Servo releaser;  

//Setting the Chip select of the pressure sensor to the module
Adafruit_BMP280 bmp(BMPChipS); // hardware SPI

static void digitalSwitch(int S, int B, int L){
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

#if 0
  Serial.print("Initializing LORA...");
#endif

  //Initializing the LORA and checking for success
  if (!LoRa.begin(915E6)) {
#if 0
    Serial.println("Starting LoRa failed!");
#endif
    //Don't do anything more:
    while(1);
  }

#if 0
  Serial.println("LoRa started");
#endif

  //Setting the SD card reader output to LOW and all others to HIGH
  digitalSwitch(0,1,1);

#if 0
  Serial.print("Initializing SD card...");
#endif

  //Initializing the SD card reader and checking for success
  if (!SD.begin(SDChipS)) {
#if 0
    Serial.print("Card failed, or not present");
#endif
    //Don't do anything more:
    while(1);
  }

#if 0
  Serial.println("SD card initialized");
#endif

  //Setting the pressure sensor output to LOW and all others to HIGH
  digitalSwitch(1,0,1);

#if 0
  Serial.print("Initializing BMP280 sensor...");
#endif

  //Initializing the pressure sensor and checking for success
  if (!bmp.begin()) {  
#if 0  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
#endif
    while(1);
  }

#if 0
  Serial.print("BMP sensor initialized.\n");
  Serial.println("");
  Serial.println("All modules are green");
  Serial.println("");
  Serial.println("___________________________________________");
  Serial.println("");
#endif

  delay(2000);
 
}

static void Data(){
    //Switching modules to the LORA/GPS shield for usage
    digitalSwitch(1,1,0);
  
    //If a reading can be recieved on the gps
    while (gps.available( gpsPort )) {
    //take the reading
    gps_fix  fix = gps.read();

    //Open a transmission to the reciever
    LoRa.beginPacket();

    //If there's a fix, save and transmit latitude and longitude
    if (fix.valid.location) {
      lat = fix.latitudeL();
      LoRa.print( fix.latitudeL() ) ;   
      lon = fix.longitudeL();
      LoRa.print( fix.longitudeL() );
      LoRa.print(alt);
    }
   }

   LoRa.println();
   LoRa.endPacket();

    //Switching modules to the pressure sensor for usage
    digitalSwitch(1,0,1);

    //Read temperature
    temp = bmp.readTemperature();
#if 0
    Serial.println(bmp.readTemperature());
#endif

    //Read Altitude
    altLast = alt;
    alt = bmp.readAltitude(1013); //Altitude property should be set to estimate current altitude
#if 0
    Serial.println(bmp.readAltitude(1013.25));
#endif
    
    //Read Atmospheric pressure
    Pa = bmp.readPressure();
#if 0
    Serial.println(bmp.readPressure());
#endif

    //Calculating estimate rise or fall speed
    Speed = (alt-altLast)/(Time-lastTime); //Meters per second
    
    
    //End the transmission to the reciever   
    
  
  }

static void DataSave(){
    //Switching modules to the SD card reader being used
    digitalSwitch(0,1,1);


    //Writing a txt file for use (if the file doesn't exist, it will create it.)
    data = SD.open("data.txt", FILE_WRITE);

    //If the file is a valid one, save all information collected above in a CSV format
    if(data){
#if 0
      Serial.println("Writing to SD Card");
#endif
      data.print(lat);
      data.print(",");
      data.print(lon);
      data.print(",");
      data.print(alt);
      data.print(" m");
      data.println();
      data.close();
      data.print(temp);
      data.print(" *C,");
      data.print(Pa);
      data.print("Pa,");
      data.print(abs(Speed));
      data.print(" m/s,");
      data.print(Time);
      data.print(" seconds");
      data.println();
      data.close();
    }
#if 0
    else{
      Serial.println("Error opening file");
    }
#endif
  }

static void drop(){
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
    if(temp <= -38){
      drop();
    }
  }






