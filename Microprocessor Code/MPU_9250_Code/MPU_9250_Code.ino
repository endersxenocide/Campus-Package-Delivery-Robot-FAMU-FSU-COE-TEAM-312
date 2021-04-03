#include "MPU9250.h"
#include <FIR.h>
#include <math.h>

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


// Make an instance of the FIR filter. In this example we'll use
// floating point values and an 8 element filter. For a moving average
// that means an 8 point moving average.
FIR<float, 8> fir;



void setup() 
{

    //For a moving average we want all of the coefficients to be unity.
  float coef[8] = { 1., 1., 1., 1., 1., 1., 1., 1.};

  // Set the coefficients
  fir.setFilterCoeffs(coef);


  Serial.begin(115200);
  Serial.println("IMU TEST");

 // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }


}

void loop() {
  // read the sensor
  IMU.readSensor();
  // display the data
  //Serial.print(IMU.getAccelX_mss(),6);
  //Serial.print("\t");
  //Serial.print(IMU.getAccelY_mss(),6);
  //Serial.print("\t");
  //Serial.print(IMU.getAccelZ_mss(),6);
  //Serial.print("\t");
  //Serial.print(IMU.getGyroX_rads(),6);
  //Serial.print("\t");
  //Serial.print(IMU.getGyroY_rads(),6);
  //Serial.print("\t");
  //Serial.print(IMU.getGyroZ_rads(),6);
  //Serial.print("\t");
  Serial.print(IMU.getMagX_uT(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagY_uT(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagZ_uT(),6);


//https://arduino.stackexchange.com/questions/18625/converting-three-axis-magnetometer-to-degrees#:~:text=heading%20%3D%20atan2(y%2C%20x,(rather%20than%20magnetic)%20heading.


  
  //Serial.print("\t");
  //Serial.println(IMU.getTemperature_C(),6);
  delay(100);
}
