#include <FIR.h>
#include <esp_now.h>
#include <WiFi.h>


//uint8_t broadcastAddress[] = {0x7C, 0x9E, 0xBD, 0xE2, 0xFC, 0x58};


uint8_t broadcastAddress[] = {0x3C, 0x61, 0x05, 0x2F, 0xA5, 0xE4};

FIR<int, 8> fir1, fir2, fir3, fir4, fir5, fir6, fir7, fir8;

// defines pins numbers
#define trigPin4 32    //left side front 4
#define echoPin4 33

#define trigPin6 25   //front right  6
#define echoPin6 26

#define trigPin8 14   //right side back  8
#define echoPin8 12

#define trigPin7 2   //right side front 7
#define echoPin7 15

#define trigPin5 22    //front left  5
#define echoPin5 23

#define trigPin3 19   //left side back  3
#define echoPin3 21

#define trigPin1 5    //back right 1
#define echoPin1 18

#define trigPin2 13   //back left 2
#define echoPin2 4

#define outputPin 27  //output pin

// defines variables

int duration1, duration2, duration3, duration4, duration5, duration6, duration7, duration8;
long distance1, distance2, distance3, distance4, distance5, distance6, distance7, distance8;
long output1, output2, output3, output4, output5, output6, output7, output8;

int x = 100;
int arr[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int flag = 0;

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

struct_message myData;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
    
  Serial.begin(9600); // Starts the serial communication
  
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;


    // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  int coef[8] = { 1., 1., 1., 1., 1., 1., 1., 1.};
  // Set the coefficients
  fir1.setFilterCoeffs(coef);
  fir2.setFilterCoeffs(coef);
  fir3.setFilterCoeffs(coef);
  fir4.setFilterCoeffs(coef);
  fir5.setFilterCoeffs(coef);
  fir6.setFilterCoeffs(coef);
  fir7.setFilterCoeffs(coef);
  fir8.setFilterCoeffs(coef);

  Serial.print("Gain set: ");
  Serial.println(fir1.getGain());
  
  pinMode(trigPin1, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin1, INPUT); // Sets the echoPin as an Input
  pinMode(trigPin2, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin2, INPUT); // Sets the echoPin as an Input
  pinMode(trigPin3, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin3, INPUT); // Sets the echoPin as an Input
  pinMode(trigPin4, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin4, INPUT); // Sets the echoPin as an Input
  pinMode(trigPin5, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin5, INPUT); // Sets the echoPin as an Input
  pinMode(trigPin6, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin6, INPUT); // Sets the echoPin as an Input
  pinMode(trigPin7, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin7, INPUT); // Sets the echoPin as an Input
  pinMode(trigPin8, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin8, INPUT); // Sets the echoPin as an Input
  pinMode(outputPin, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(echoPin1), bob1_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(echoPin2), bob2_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(echoPin3), bob3_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(echoPin4), bob4_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(echoPin5), bob5_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(echoPin6), bob6_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(echoPin7), bob7_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(echoPin8), bob8_ISR, RISING);
  
  Serial.setRxBufferSize(64);
}

void bob1_ISR()
{
  distance1 = duration1*0.034/2;
  if(distance1<400)
  {
    // Calculate the moving average
    output1 = fir1.processReading(distance1);
  }
  else
  {
    output1 = fir1.processReading(400);
  }

  if(distance1 < x)
  {
    arr[0] = 1;
  }
  else
  {
    arr[0] = 0;
  }
  
}
void bob2_ISR()
{
  distance2 = duration2*0.034/2;
  if(distance2<400)
  {
    // Calculate the moving average
    output2 = fir2.processReading(distance2);
  }
  else
  {
    output2 = fir2.processReading(400);
  }

  if(distance2 < x)
  {
    arr[1] = 1;
  }
  else
  {
    arr[1] = 0;
  }
  
}
void bob3_ISR()
{
  distance3 = duration3*0.034/2;
  if(distance3<400)
  {
    // Calculate the moving average
    output3 = fir3.processReading(distance3);
  }
  else
  {
    output3 = fir3.processReading(400);
  }

  if(distance3 < x)
  {
    arr[2] = 1;
  }
  else
  {
    arr[2] = 0;
  }
  
}
void bob4_ISR()
{
  distance4 = duration4*0.034/2;
  if(distance4<400)
  {
    // Calculate the moving average
    output4 = fir4.processReading(distance4);
  }
  else
  {
    output4 = fir4.processReading(400);
  }

  if(distance4 < x)
  {
    arr[3] = 1;
  }
  else
  {
    arr[3] = 0;
  }
  
}
void bob5_ISR()
{
  distance5 = duration5*0.034/2;
  if(distance5<400)
  {
    // Calculate the moving average
    output5 = fir5.processReading(distance5);
  }
  else
  {
    output5 = fir5.processReading(400);
  }

  if(distance5 < x)
  {
    arr[4] = 1;
  }
  else
  {
    arr[4] = 0;
  }
  
}
void bob6_ISR()
{
  distance6 = duration6*0.034/2;
  if(distance6<400)
  {
    // Calculate the moving average
    output6 = fir6.processReading(distance6);
  }
  else
  {
    output6 = fir6.processReading(400);
  }

  if(distance6 < x)
  {
    arr[5] = 1;
  }
  else
  {
    arr[5] = 0;
  }
}
void bob7_ISR()
{
  distance7 = duration7*0.034/2;
  if(distance7<400)
  {
    // Calculate the moving average
    output7 = fir7.processReading(distance7);
  }
  else
  {
    output7 = fir7.processReading(400);
  }

if(distance7 < x)
  {
    arr[6] = 1;
  }
  else
  {
    arr[6] = 0;
  }
  
}
void bob8_ISR()
{
  distance8 = duration8*0.034/2;
  if(distance8<400)
  {
    // Calculate the moving average
    output8 = fir8.processReading(distance8);
  }
  else
  {
    output8 = fir8.processReading(400);
  }

  if(distance8 < x)
  {
    arr[7] = 1;
  }
  else
  {
    arr[7] = 0;
  }
  
}
void loop() {

  myData.device = 0;
  myData.MotorPower = 0;
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  duration1 = pulseIn(echoPin1, HIGH);
  Serial.print("Top Left Sensor : ");
  Serial.println(output1);

  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  duration2 = pulseIn(echoPin2, HIGH);
  Serial.print("top Mid Left Sensor : ");
  Serial.println(output2);

  digitalWrite(trigPin3, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);
  duration3 = pulseIn(echoPin3, HIGH);
  Serial.print("bot mid Sensor : ");
  Serial.println(output3);
  
  digitalWrite(trigPin4, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin4, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin4, LOW);
  duration4 = pulseIn(echoPin4, HIGH);
  Serial.print("bot left Sensor : ");
  Serial.println(output4);

  digitalWrite(trigPin5, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin5, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin5, LOW);
  duration5 = pulseIn(echoPin5, HIGH);
  Serial.print("top right Sensor : ");
  Serial.println(output5);
  
  digitalWrite(trigPin6, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin6, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin6, LOW);
  duration6 = pulseIn(echoPin6, HIGH);
  Serial.print("top mid right Sensor : ");
  Serial.println(output6);

  digitalWrite(trigPin7, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin7, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin7, LOW);
  duration7 = pulseIn(echoPin7, HIGH);
  Serial.print("bot mid right Sensor : ");
  Serial.println(output7);

  digitalWrite(trigPin8, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin8, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin8, LOW);
  duration8 = pulseIn(echoPin8, HIGH);
  Serial.print("bot right Sensor : ");
  Serial.println(output8);


  myData.a = arr[0];
  myData.b = arr[1];
  myData.c = arr[2];
  myData.d = arr[3];
  myData.e = arr[4];
  myData.f = arr[5];
  myData.g = arr[6];
  myData.h = arr[7];


  for(int i = 0; i < 8; i++)
  {
    if(arr[i])
      flag = 1;
  }
  
  if(flag)
  {
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData , sizeof(myData));
   
      if (result == ESP_OK) {
        Serial.println("Sent with success");
      }
      else {
        Serial.println("Error sending the data");
      }
     // delay(2000);
  }
  else
  {
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
      if (result == ESP_OK) {
        Serial.println("Sent with success");
      }
      else {
        Serial.println("Error sending the data");
      }
     // delay(2000);
  }
  flag = 0;

  Serial.print("Array: ");
  for(int j = 0; j < 8; j++)
  {
    Serial.print(arr[j]);
    Serial.print(" ");
  }
  Serial.println();
  
  
}
