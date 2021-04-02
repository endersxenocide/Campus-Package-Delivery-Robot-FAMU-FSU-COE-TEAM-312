//  Arduino Pro Mini 3.3V example
//  for   https://github.com/p-h-a-i-l/hoverboard-firmware-hack
//  visit https://pionierland.de/hoverhack/ to compile your firmware online :-)


void Setup_Odometry();

//#define DEBUG_RX

float Last_Error_Angle = 0;
float Derrivative = 0;

//Lidar and Ultrasonic Inputs
const int Lidarpin = 12;  // the number of the pushbutton pin
const int Ultrasonicpin =  13;    // the number of the LED pin

int estop = 1;
int endstop = 1;

//Motor Controller Variables
#define TIME_SEND 200
int iTest = 1000;
unsigned long iTimeSend = 0;

int iSpeed = 0;
int iSpeedMax = 1000;   //100 for SPEED_IS_KMH in config.h use 60 for max speed of 6.0 km/h
int iSpeedDelta = 2;   //20 for SPEED_IS_KMH in config.h use 2 for speed steps of 0.2 km/h 
int turnRate = 0;

float iSpeedL = 0;
float iSpeedR = 0;

//Waypoint Variables

         float WaypointX[] = {20,-20, 20,-20,20,-20};
         float WaypointY[] = {7,7, 7, 7, 7, 7};
         
         int WP_num = 0; //What waypoint we are on
         float isteer = 0;
         
          float Error_Angle = 0;

//Odometry Variables
float V = 0;
float W = 0;
float L = .545; //distance between wheels in meters
float theta = 0;
float last_theta = 0;

float WheelV_Xpos = 0;
float WheelV_Ypos = 0;
float WheelV_ThetaPOS = 0;

float k00 = 0;
float k01 = 0;
float k02 = 0;
float k10 = 0;
float k11 = 0;
float k12 = 0;
float k20 = 0;
float k21 = 0;
float k22 = 0;
float k30 = 0;
float k31 = 0;
float k32 = 0;

float t = 0.005; //time step (S) recommended 1/200s

//Set up SPIFFS (file system)
#include "FS.h"
#include "SPIFFS.h"
#define FORMAT_SPIFFS_IF_FAILED false //Set as true only for the first time you ever run


//Setup ISR Variables

volatile int interruptCounter;
int totalInterruptCounter;
 
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;



#include "MPU9250.h"
#include <FIR.h>

// Make an instance of the FIR filter. In this example we'll use
// floating point values and an 8 element filter. For a moving average
// that means an 8 point moving average.
FIR<float, 8> fir;

#include <math.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <esp_now.h>

//Set up motor controller

#include <SoftwareSerial.h>
SoftwareSerial oSerial(16,17); // RX, TX

typedef struct{
   int16_t steer;
   int16_t speed;
   uint32_t crc;
} Serialcommand;
Serialcommand oCmd;

typedef struct{
   int16_t iSpeedL; // 100* km/h
   int16_t iSpeedR; // 100* km/h
   uint16_t iHallSkippedL;
   uint16_t iHallSkippedR;
   uint16_t iTemp;  // °C
   uint16_t iVolt;  // 100* V
   int16_t iAmpL;  // 100* A
   int16_t iAmpR;  // 100* A
   uint32_t crc;
} SerialFeedback;
SerialFeedback oFeedback;



// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;

//Set up Magnometer variables
float heading = 0;
float mag_x = 0;
float mag_y = 0;
float GPSX = 0;
float GPSY = 0;
float DeltaX = 0;
float DeltaY = 0;
float Currentx = 0;
float Currenty = 0;


//File system Functions:
;void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\r\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("- failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println(" - not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.name(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("\tSIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\r\n", path);

    File file = fs.open(path);
    if(!file || file.isDirectory()){
        Serial.println("- failed to open file for reading");
        return;
    }

    Serial.println("- read from file:");
    while(file.available()){
        Serial.write(file.read());
    }
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\r\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("- failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("- file written");
    } else {
        Serial.println("- frite failed");
    }
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\r\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("- failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("- message appended");
    } else {
        Serial.println("- append failed");
    }
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
    Serial.printf("Renaming file %s to %s\r\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("- file renamed");
    } else {
        Serial.println("- rename failed");
    }
}

void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\r\n", path);
    if(fs.remove(path)){
        Serial.println("- file deleted");
    } else {
        Serial.println("- delete failed");
    }
}

void testFileIO(fs::FS &fs, const char * path){
    Serial.printf("Testing file I/O with %s\r\n", path);

    static uint8_t buf[512];
    size_t len = 0;
    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("- failed to open file for writing");
        return;
    }

    size_t i;
    Serial.print("- writing" );
    uint32_t start = millis();
    for(i=0; i<2048; i++){
        if ((i & 0x001F) == 0x001F){
          Serial.print(".");
        }
        file.write(buf, 512);
    }
    Serial.println("");
    uint32_t end = millis() - start;
    Serial.printf(" - %u bytes written in %u ms\r\n", 2048 * 512, end);
    file.close();

    file = fs.open(path);
    start = millis();
    end = start;
    i = 0;
    if(file && !file.isDirectory()){
        len = file.size();
        size_t flen = len;
        start = millis();
        Serial.print("- reading" );
        while(len){
            size_t toRead = len;
            if(toRead > 512){
                toRead = 512;
            }
            file.read(buf, toRead);
            if ((i++ & 0x001F) == 0x001F){
              Serial.print(".");
            }
            len -= toRead;
        }
        Serial.println("");
        end = millis() - start;
        Serial.printf("- %u bytes read in %u ms\r\n", flen, end);
        file.close();
    } else {
        Serial.println("- failed to open file for reading");
    }
}

//ISR function
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
}


//Lidar and Ultrasonic ESPNow Communication
// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
    int MotorPower;
    bool a;
    bool b;
    bool c;
    bool d;
    bool e;
    bool f;
    bool g;
    bool h;
    bool device;
} struct_message;

// Create a struct_message called myData
struct_message myData;


// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.println("Lidar or Ultrasonic?");
  Serial.println(myData.device);

if(myData.device == 0){ //Ultrasonic Sensor


Serial.println("8 Ultrasonic data: ");
  Serial.println(myData.a);
  Serial.println(myData.b);
  Serial.println(myData.c);
  Serial.println(myData.d);
  Serial.println(myData.e);
  Serial.println(myData.f);
  Serial.println(myData.g);
  Serial.println(myData.h);
  Serial.println(" ");
 
      if(myData.a && myData.b && myData.c && myData.d && myData.e && myData.f && myData.g && myData.h){//if ultrasonic sensor detects
      estop = 0;


      }
    else {
      estop = 1;
    }
      

}

if(myData.device == 1){ //Lidar


             Serial.println("Motor Power Value: ");
          Serial.println(myData.MotorPower);
  
      if(myData.MotorPower <= 100){//if ultrasonic sensor detects 
      estop = 0;

      }
          else {
      estop = 1;
    }

}




} 





//This is the corrected RTK and Wheel velocity position
float Corrected_X = 0;
float Corrected_Y = 0;
float Corrected_Theta = 0;


//Set up RTK GPS input
#define RXD2 4
#define TXD2 2

String gpsType; 
String intStr; 
float latitude, longitude, m2; 
int sign, d, m1; 
char dir; 

float Xpos = 0;
float Ypos = 0;
float ThetaPOS = 0;
float RTKLastX = 0;
float RTKLastY = 0;
float GPSMinDistance = 0;    //Tune this value; minimum distance robot needs to travel for movement to register: 0.05
float GPSMaxDistance = 10;      //max distance robot can be expected to travel between data
float DistanceRTK = 0;          //distance traveled calculated using X and Y positions

//Set orgin as the location of the coordinate Orgin. Update if location Changed!!
//float LatOrgin = 30.421948;
//float LongOrgin = -84.319579;
float LatOrgin = 30.42487;
float LongOrgin = -84.30500;

void setup() 
{
  //Set up ultrasonic and Lidar inputs
  pinMode(Lidarpin, INPUT);
  pinMode(Ultrasonicpin, INPUT);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);



  //2000000 is 1 second
  //Setup Timer ISR
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 5000, true); //every 1/200 set to 5000 seconds -- update this with t!!
  timerAlarmEnable(timer);

  //Initialize File System
  if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){
    Serial.println("SPIFFS Mount Failed");
    return;
  }

  //For a moving average we want all of the coefficients to be unity.
  float coef[8] = { 1., 1., 1., 1., 1., 1., 1., 1.};

  // Set the coefficients
  fir.setFilterCoeffs(coef);
    
  Serial.begin(115200);
  Serial.println("Hoverhack Test v1.0");

  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
     
  oSerial.begin(9600);
  //pinMode(LED_BUILTIN, OUTPUT);
}

uint32_t crc32_for_byte(uint32_t r) 
{
  for(int j = 0; j < 8; ++j)
    r = (r & 1? 0: (uint32_t)0xEDB88320L) ^ r >> 1;
  return r ^ (uint32_t)0xFF000000L;
}

void crc32(const void *data, size_t n_bytes, uint32_t* crc) {
  static uint32_t table[0x100];
  if(!*table)
    for(size_t i = 0; i < 0x100; ++i)
      table[i] = crc32_for_byte(i);
  for(size_t i = 0; i < n_bytes; ++i)
    *crc = table[(uint8_t)*crc ^ ((uint8_t*)data)[i]] ^ *crc >> 8;
}

void Send(int16_t iSpeed,int16_t iSteer)
{
  oCmd.steer = iSteer;
  oCmd.speed = iSpeed;

  uint32_t crc = 0;
  crc32((const void *)&oCmd, sizeof(Serialcommand)-4,   &crc);
  oCmd.crc = crc;
  
  oSerial.write((uint8_t *) &oCmd, sizeof(oCmd)); 
}

int iFailedRec = 0;
boolean Receive()
{
  //while (oSerial.available()) {Serial.print(" ");Serial.print(oSerial.read(),HEX);}return false;

  if (oSerial.available()<  sizeof(SerialFeedback))
    return false;

  SerialFeedback oNew;
  byte* p = (byte*)&oNew;
  for (unsigned int i=0; i < sizeof(SerialFeedback); i++)
    *p++ = oSerial.read();;

  uint32_t crc = 0;
  crc32((const void *)&oNew, sizeof(SerialFeedback)-4,   &crc);

#ifdef DEBUG_RX
  char sBuff[10];
  p = (byte*)&oNew;
  for (unsigned int i=0; i < sizeof(SerialFeedback); i++)
  {
    sprintf(sBuff," %02x",p[i]);
    Serial.print(sBuff);
  }
  Serial.print(" ?= ");Serial.println(crc,HEX);
#endif

  if (oNew.crc == crc)
  {
    memcpy(&oFeedback,&oNew,sizeof(SerialFeedback));
    return true;    
  }

#ifdef DEBUG_RX
  while (oSerial.available()) {Serial.print(" ");Serial.print(oSerial.read(),HEX);}Serial.println("\t:-(");
#else
  while (oSerial.available()) oSerial.read();   // empty garbage
  Serial.print("X");
  iFailedRec++;
#endif
  return false;
}


int count = 0;

void loop(void)
{  
 //Timer ISR
 if (interruptCounter > 0) {
 
    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);
    totalInterruptCounter++;

    unsigned long iNow = millis();
    if (Receive())
    {
      if (iFailedRec)
        Serial.println();
      
      iFailedRec = 0;
      //Serial.print("speedL: ");Serial.print(-0.01*(float)oFeedback.iSpeedL);
      //Serial.print("\tspeedR: ");Serial.print(-0.01*(float)oFeedback.iSpeedR);
      iSpeedL = -0.01*(float)oFeedback.iSpeedL;
      iSpeedR = -0.01*(float)oFeedback.iSpeedR;

      //twice wheel diameter *2, km/h to m/s *0.278 = 0.556
      iSpeedL *= 0.566;
      iSpeedR *= 0.566;


      //Serial.print("\tskippedL: ");Serial.print(oFeedback.iHallSkippedL);
      //Serial.print("\tskippedR: ");Serial.print(oFeedback.iHallSkippedR);
      //Serial.print("\t°C: ");Serial.print(oFeedback.iTemp);
      //Serial.print("\tU: ");Serial.print(0.01 * (float)oFeedback.iVolt);
      //Serial.print("\tlA: ");Serial.print(0.01 * (float)oFeedback.iAmpL);
      //Serial.print("\trA: ");Serial.println(0.01 * (float)oFeedback.iAmpR);
   }

V = 0.5*(iSpeedL + iSpeedR);//In m/s 
W = (iSpeedR - iSpeedL)/L; //L = Distance between the point of contact of the two wheels to the ground (in meters)

k00 = V*cos(last_theta);
k01 = V*sin(last_theta);
k02 = W;

k10 = V*cos(last_theta + t/2*k02);
k11 = V*sin(last_theta + t/2*k02);
k12 = W;


k20 = V*cos(last_theta + t/2*k12);
k21 = V*sin(last_theta + t/2*k12);
k22 = W;

k30 = V*cos(last_theta + t*k22);
k31 = V*sin(last_theta + t*k22);
k32 = W;

WheelV_Xpos = WheelV_Xpos + (t/6*(k00 + 2*(k10+k20) + k30));
WheelV_Ypos = WheelV_Ypos + (t/6*(k01 + 2*(k11+k21) + k31));
WheelV_ThetaPOS = WheelV_ThetaPOS + (t/6*(k02 + 2*(k12+k22) + k32));

//if(WheelV_ThetaPOS>2*3.141593){
//  WheelV_ThetaPOS = 0;
//}
//
//if(WheelV_ThetaPOS<0){
//  ThetaPOS = 2*3.141593;
//}

last_theta = WheelV_ThetaPOS;


Corrected_X = WheelV_Xpos + Xpos; //Add RTK position and Wheel Velocity based position
Corrected_Y = WheelV_Ypos + Ypos; //Add RTK position and Wheel Velocity based position
Corrected_Theta = WheelV_ThetaPOS + ThetaPOS;


        Error_Angle = Corrected_Theta - atan((WaypointY[WP_num]-Corrected_Y)/(WaypointX[WP_num] - Corrected_X));
        Derrivative = Error_Angle - Last_Error_Angle;
        Last_Error_Angle = Error_Angle;








//Write code to make speed control loop (So it can go up hills).
//Operating_Speed = 0;
//Desired_Velocity = 5;
//Velocity_Error = (Desired Velocity - V)
//Operating_Speed = 100*Desired_Velocity + (100 * Velocity_Error); //Ramp velocity as higher value until desired velocity reached







isteer = -1000*Error_Angle + -300000*Derrivative; //Note Need to check derrivative value. Works alright without load, but spins in circles with load (-1000 and -300000) also check signs!

        //These parameters seem reasonable
      
        if(isteer>150)
        {
          isteer = 150;
        }
        if(isteer <-150)
        {
          isteer = -150;
        }
  
            //Increment waypoint
           if(abs(Xpos - WaypointX[WP_num]) < 0.25 && abs(Ypos - WaypointY[WP_num]) < 0.25 && WP_num<(sizeof(WaypointX))) {
            WP_num++;
           }
          //Stop robot after waypoints reached
 if(WP_num>=(sizeof(WaypointX))) {
            endstop = 0;
           }





         Send(300*estop*endstop,isteer*estop*endstop); 


  if (iTimeSend > iNow) return;
  iTimeSend = iNow + TIME_SEND;
  
/*---------calculating lat and long from gps data---------*/
 while (Serial2.available()) {
    if(char(Serial2.read()) == '$'){
      gpsType=char(Serial2.read()); 
      for(int i=0; i<4; i++)
          gpsType += char(Serial2.read()); 
      
      if(gpsType == "GNGGA"){
        for(int i=0; i<11; i++)
            Serial2.read(); 
        if(Serial2.available() >0){ 
        intStr=char(Serial2.read());  //dd
        }
        if(Serial2.available() >0){
        intStr+=char(Serial2.read());   
        } 
        d=intStr.toInt(); 

        if(Serial2.available() >0){
        intStr=char(Serial2.read()); //mm
        }
        if(Serial2.available() >0){
        intStr+=char(Serial2.read());
        }        
        m1=intStr.toInt(); 

        if(Serial2.available() >0){
        Serial2.read(); //skipping the decimal point
        }
        if(Serial2.available() >0){
        intStr=char(Serial2.read()); 
        }
        for(int i=0; i<4; i++){
          if(Serial2.available() >0){
            intStr+=char(Serial2.read()); 
          }
        }
          
        m2=intStr.toInt(); 
        m2=m2/100000.0; 

        if(Serial2.available() >0){
        Serial2.read(); //skipping comma
        }
        if(Serial2.available() >0){
          dir = char(Serial2.read()); 
        }
        if(dir == 'N')
           sign = 1; 
        else if(dir == 'S')
          sign = -1; 

        //Serial.print("d_lat: ");  
        //Serial.println(d,10);
        
        latitude = sign*(d+(m1/60.0)+(m2/60.0)); 
        if(Serial2.available() >0){
          Serial2.read();             //skipping comma
        }
        if(Serial2.available() >0){
          intStr=char(Serial2.read());      //ddd
        }
        if(Serial2.available() >0){
          intStr+=char(Serial2.read());
        }
        if(Serial2.available() >0){ 
          intStr+=char(Serial2.read()); 
        }
        d=intStr.toInt(); 
        if(Serial2.available() >0){
          intStr=char(Serial2.read());     //mm
        }
        if(Serial2.available() >0){
          intStr+=char(Serial2.read());   
        }     
        m1=intStr.toInt(); 

        if(Serial2.available() >0){
          Serial2.read();           //skipping the decimal point
        }
        if(Serial2.available() >0){
          intStr=char(Serial2.read());    //mmmmm
        }
        for(int i=0; i<4; i++){
          if(Serial2.available() >0){
            intStr+=char(Serial2.read()); 
          }
        }
          
        m2=intStr.toInt(); 
        m2=m2/100000.0; 

        if(Serial2.available() >0){
          Serial2.read();           //skipping comma
        }
        if(Serial2.available() >0){
          dir = char(Serial2.read());
        } 
        if(dir == 'E')
           sign = 1; 
        else if(dir == 'W')
          sign = -1; 

        //Serial.print("d_lon: ");  
        //Serial.println(d, 10);
        
        longitude = sign*(d+(m1/60.0)+(m2/60.0)); 

        //Serial.println(m1);
        //Serial.println(m2, 8);


        //calculate distance travelled based on x and y pos




      //Serial.print("Ypos: ");
      //Serial.println(Ypos, 8);
      //Serial.print("RTKLastY: ");
      //Serial.println(RTKLastY, 8);
      //Serial.print("Xpos: ");
      //Serial.println(Xpos, 8);
      //Serial.print("RTKLastX: ");
      //Serial.println(RTKLastX, 8);


      
//                          Serial.print("Corrected_X: ");
//                          Serial.println(Corrected_X, 8);
//                          Serial.print("Corrected_Y: ");
//                          Serial.println(Corrected_Y, 8);
//                          Serial.print("Corrected_Theta: ");
//                          Serial.println(Corrected_Theta, 8);
        
        DistanceRTK = sqrt(sq(Ypos - RTKLastY) + sq(Xpos - RTKLastX));
        Serial.print("RTK Distance: ");
        Serial.println(DistanceRTK);

        float temp_GPSDistance; 
        float Xpos_temp; 
        float Ypos_temp; 
        Xpos_temp = (longitude - LongOrgin)*111320;
        Ypos_temp =(latitude - LatOrgin)*111320;
        temp_GPSDistance = sqrt(sq(Ypos_temp - RTKLastY) + sq(Xpos_temp - RTKLastX));


        //if distance < minDist, robot has not moved
        //if gpsdistance > max distance, invalid gps coord calculated
        if((temp_GPSDistance >= GPSMinDistance) && (temp_GPSDistance < GPSMaxDistance))
            {
                  //Update Robot Position if valid distance
                  Xpos = (longitude - LongOrgin)*111320;
                  Ypos = (latitude - LatOrgin)*111320;



                   if(temp_GPSDistance >=0.5){
                   //Update heading based upon RTK GPS if the robot has moved sufficiently
                  ThetaPOS = atan((Ypos - RTKLastY)/(Xpos - RTKLastX));
                  if((Xpos - RTKLastX)<0){                 
                    ThetaPOS = ThetaPOS + 3.141593;
                  }
                  
                          Serial.print("RTKNewangle: ");
                          Serial.println(ThetaPOS, 8);
                    
                   }
                
                  
                  //Store Position for next iteration
                  RTKLastX = Xpos;
                  RTKLastY = Ypos;


                  //Reset Wheel velocity variables
                   Reset_Wheel_Odometry();


                          
                          //Serial.print("Lat: ");
                          //Serial.println(latitude, 8);
                          //Serial.print("Long: ");
                          //Serial.println(longitude, 8);



                          Serial.print("RTKXpos: ");
                          Serial.println(Xpos, 8);
                          Serial.print("RTKYpos: ");
                          Serial.println(Ypos, 8);
                          
                  
                }     
      } 
    }
  }

  
  //Serial.print("Error Angle: ");    
  //Serial.println(Error_Angle);
  
  //Serial.print("Ispeed: ");    
  //Serial.println(isteer);
  
  
    //Print out current position
    //Serial.print("Heading: ");
    //Serial.println(ThetaPOS);
  
  
     //Serial.print("Angle to waypoint: ");
     //Serial.println(atan((WaypointY[WP_num]-Ypos)/(WaypointX[WP_num] - Xpos)));
     
     //Serial.print("    Currentx: ");
     //Serial.print(Xpos,8);
     //Serial.print("    Currenty: ");
     //Serial.println(Ypos,8);
  
     //Serial.print(" Left: ");
     //Serial.print(iSpeedL,8);
     //Serial.print("    Right: ");
     //Serial.println(iSpeedR,8);
     //Serial.println(iSpeedL);

  }

}



void Reset_Wheel_Odometry(){
                   V = 0;
                  W = 0;
                  theta = 0;
                  last_theta = 0;
                  WheelV_Xpos = 0;
                  WheelV_Ypos = 0;
                  WheelV_ThetaPOS = 0;
                  k00 = 0;
                  k01 = 0;
                  k02 = 0;
                  k10 = 0;
                  k11 = 0;
                  k12 = 0;
                  k20 = 0;
                  k21 = 0;
                  k22 = 0;
                  k30 = 0;
                  k31 = 0;
                  k32 = 0;
}
