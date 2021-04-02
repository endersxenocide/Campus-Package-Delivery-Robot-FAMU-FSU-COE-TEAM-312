#include <RPLidar.h>
#include <analogWrite.h>
#include <esp_now.h>
#include <WiFi.h>

// Change the pin mapping based on your needs.
#define UPDATES_PER_SECOND 100
#define DTPWM         4  // output PWM signal to be sent to the motor controller
#define RX2           16 // recieve pin
#define TX2           17 // transmit pin
#define RPLIDAR_MOTOR 5  // The PWM pin for control the speed of RPLIDAR's motor.
                         // This pin should connected with the RPLIDAR's MOTOCTRL signal 

// You need to create an driver instance 
RPLidar lidar;

////////////////////////////WiFi info/////////////////////////////////////////////
// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress1[] = {0x24, 0x0A, 0xC4, 0x58, 0x01, 0x6C}; //test ESP 1
uint8_t broadcastAddress2[] = {0x24, 0x0A, 0xC4, 0x5A, 0xC4, 0x28}; //test ESP 2
uint8_t broadcastAddress3[] = {0x24, 0x0A, 0xC4, 0x5A, 0xB0, 0x24}; //test ESP 3

uint8_t broadcastAddress4[] = {0x3C, 0x61, 0x05, 0x2A, 0x9D, 0x98}; //this is the MAC address of the LIDAR ESP on the robot
uint8_t broadcastAddress5[] = {0x3C, 0x61, 0x05, 0x2F, 0xA5, 0xE4}; //this is the MAC address of the MAIN ESP on the robot

// Structure example to send data
// Must match the receiver structure
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

////////////////////////////Functions///////////////////////////////////////////
//function to update the PWM sent to the motor controller. triggers once every 360 degree rotation.
void AccelDecel (bool accel, bool decel, int outputtracker) {
  if ((decel == false)) {
    analogWrite(DTPWM, outputtracker);
    myData.MotorPower = outputtracker;
  }
  else if ((decel == true)) {
    analogWrite(DTPWM, outputtracker);
    myData.MotorPower = outputtracker;
  }

  myData.device = 1; //1 is for lidar, 0 for ultrasonic
  myData.a = 0;
  myData.b = 0;
  myData.c = 0;
  myData.d = 0;
  myData.e = 0;
  myData.f = 0;
  myData.g = 0;
  myData.h = 0;

   esp_err_t DataSend = esp_now_send(broadcastAddress5, (uint8_t *) &myData, sizeof(myData));
   
  if (DataSend == ESP_OK) {
    Serial.println("Sent with success to MAIN ESP");
  }
  else {
    Serial.println("Error sending the data to MAIN ESP");
  }
}

//Function to check if data was delivered successfully
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
/////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  delay(3000);
  // bind the RPLIDAR driver to the arduino hardware serial
  Serial2.begin(115200);
  Serial.begin(115200);
  lidar.begin(Serial2);
  
  // set pin modes
  pinMode(RPLIDAR_MOTOR, OUTPUT);
  pinMode(DTPWM, OUTPUT);

// Setting up WiFi to make this ESP a transmitter
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peers
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress5, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
/*
  memcpy(peerInfo.peer_addr, broadcastAddress2, 6);
  peerInfo.channel = 1;  
  peerInfo.encrypt = false;
*/  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

float minDistance = 100000;
float cycleminDistance = 100000;
float cycleMinAngle = 0;
float angleAtMinDist = 0;
float prevangle = 0;
bool decel = false;
bool accel = false;
int CornerAngle = 30;
int outputtracker=0;
float SlowingFactor=0;
int FirstLED = 0;
int angleadder = 0;



//////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  if (IS_OK(lidar.waitPoint())) {
    //perform data processing here... 
    float distance = lidar.getCurrentPoint().distance;
    float angle = lidar.getCurrentPoint().angle;
    bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
    byte  quality  = lidar.getCurrentPoint().quality; //quality of the current measurement
    
    //this if triggers once per cycle (a cycle is one 360 degree rotation)
    if ((angle<25) && (prevangle>300) && ((angle - prevangle) < -300)) {
          //calling the functions that trigger once per cycle
          AccelDecel(accel, decel, outputtracker);
          
          //updating the value of outputtracker to be sent to the DTPWM on the next cycle
          if ((decel == false) && (outputtracker <=255)) {
            outputtracker = outputtracker+ 2;
            //analogWrite(DTPWM, outputtracker);
          }
          else if ((decel == true) && (outputtracker >= 0)) {
            outputtracker = outputtracker-(3000*SlowingFactor);
            if(outputtracker<0) {
              outputtracker=0;
            }
          }
          
          //resetting values for a new cycle
          minDistance = 100000;
          decel = false;
          accel = false;
          
    //this else triggers every sample except on the start of a new cycle
    } else {
        //telling the motor to slow down if an object is detected in direct path of robot.
        if (((angle<350) || ((angle > 190) && (angle <360))) && (distance<400) && (distance>0) && (decel!=1)) {
          decel=true;
          SlowingFactor=1/distance;
        }
        //checking each sample to see if it is the minimum distance point for that cycle
        if ( distance > 0 &&  (distance < minDistance)) {
          minDistance = distance;
          angleAtMinDist = angle;
       }
      }
        
      //storing the last angle
      prevangle = angle; //stores the last angle
          
//motor initialization on startup
} else {
    analogWrite(RPLIDAR_MOTOR, 100); //stop the rplidar motor
    Serial.println("motorstart");
    // try to detect RPLIDAR... 
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
       //detected...
       lidar.startScan();
       analogWrite(RPLIDAR_MOTOR, 180);
       delay(1000);
       Serial.println("ScanStart");
    }
  }
}
