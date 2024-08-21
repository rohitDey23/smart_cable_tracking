#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>

#define I2C_MUX_ADDRESS 0x70
#define IMU_ADDRESS_1 0x28
#define IMU_ADDRESS_2 0x29

QWIICMUX mux;

Adafruit_BNO055 imu1 = Adafruit_BNO055();
Adafruit_BNO055 imu2 = Adafruit_BNO055();
Adafruit_BNO055 imu3 = Adafruit_BNO055();
Adafruit_BNO055 imu4 = Adafruit_BNO055();

void setup() {
  Serial.begin(115200);

  Wire.begin();
  mux.begin();
  
  mux.setPort(0);
  imu1.begin();
  imu1.setExtCrystalUse(true);
  
  mux.setPort(1);
  imu2.begin();
  imu2.setExtCrystalUse(true);
  
  mux.setPort(2);
  imu3.begin();
  imu3.setExtCrystalUse(true);
  
  mux.setPort(3);
  imu4.begin();
  imu4.setExtCrystalUse(true);

  delay(500);
}

String create_data(String val, imu::Quaternion quat_add)
{
  float x = quat_add.x();
  float y = quat_add.y();
  float z = quat_add.z();
  float w = quat_add.w();

  String data = String(x,4) + "," + String(y, 4) + "," + String(z, 4) + "," + String(w, 4);
  data = val + data;
  return data;
}


void loop() {
  
  String result = "";
  uint8_t sys, gyro, accel, mag = 0;
  
  mux.setPort(0);
  imu::Quaternion quat1 = imu1.getQuat();
  imu1.getCalibration(&sys, &gyro, &accel, &mag);
  result = create_data(result, quat1);
  result = result + "," + int(sys) + ",";
  sys, gyro, accel, mag = 0;

  mux.setPort(1);
  imu::Quaternion quat2 = imu2.getQuat();
  imu2.getCalibration(&sys, &gyro, &accel, &mag);
  result = create_data(result, quat2);
  result = result + "," + int(sys) + ",";
  sys, gyro, accel, mag = 0;


  mux.setPort(2);
  imu::Quaternion quat3 = imu3.getQuat();
  imu3.getCalibration(&sys, &gyro, &accel, &mag);
  result = create_data(result, quat3);
  result = result + "," + int(sys) + ",";
  sys, gyro, accel, mag = 0;


  mux.setPort(3);
  imu::Quaternion quat4 = imu4.getQuat();
  imu4.getCalibration(&sys, &gyro, &accel, &mag);
  result = create_data(result, quat4);
  result = result + "," + int(sys);


  Serial.println(result);
  delay(100);
}
