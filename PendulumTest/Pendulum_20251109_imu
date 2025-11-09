// 起動直後からの相対角度3軸のセンサフュージョン値をシリアルに出力する

#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;

// MPU control/status vars
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [roll, pitch, yaw]   roll/pitch/yaw container and gravity vector

void setupMPU() {
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  // IMU_zeroスクリプトの結果をここに代入する
  mpu.setXAccelOffset(-3851); 
  mpu.setYAccelOffset(-1775);
  mpu.setZAccelOffset(1103);
  mpu.setXGyroOffset(-7);
  mpu.setYGyroOffset(-28);
  mpu.setZGyroOffset(70);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print("DMP Initialization failed.");
  }
}

void getYawPitchRoll() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print(ypr[2] * 180 / M_PI);
    Serial.print(",");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print(",");
    Serial.println(ypr[0] * 180 / M_PI);
  }
}

void setup() {
  Serial.begin(115200);
  setupMPU();
}

void loop() {
  getYawPitchRoll();
}
