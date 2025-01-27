#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "BleComboKeyboard.h"
#include "BleComboMouse.h"

Adafruit_MPU6050 mpu;
BleComboKeyboard bleKeyboard("Hash's Mouse", "Hash_Oak", 100);
BleComboMouse bleMouse(&bleKeyboard);

const uint8_t thPin[2] = { 36, 39 };      // Touch pins on thumb finger
const uint8_t inPin[3] = { 34, 35, 32 };  // Touch pins on index finger

int mt = 500;
int ttv = 3500;   // Thumb touch trigger value
int itv = 500;    // Index touch trigger value
int mtv = 1800;   // Multiple touch threshold value
int jat = 100;    // Wait time to avoid touch jerk
int tbm = 500;    // Time before movement
int rt = 10;      // Touch relaxation time
int cf = 15;      // Cursor movement multiplier factor
float st = 0.5;   // Touch movement threshold value

void setup() {
  Serial.begin(115200);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  pinMode(thPin[0], INPUT);
  pinMode(thPin[1], INPUT);
  pinMode(inPin[0], INPUT);
  pinMode(inPin[1], INPUT);
  pinMode(inPin[2], INPUT);

  Serial.println("Starting BLE...");
  bleKeyboard.begin();
  bleMouse.begin();
  while (!bleKeyboard.isConnected()) delay(100);
  Serial.println("BLE Connected...");
}

int t1, t2, i1, i2, i3;
sensors_event_t a, g, temp;
unsigned long i,j;

void loop() {
  // mpu.getEvent(&a, &g, &temp);
  // Serial.print("rl:-1,ru:1,");
  // Serial.printf("acc_x:%f,acc_y:%f,acc_z:%f,gyr_x:%f,gyr_y:%f,gyr_z:%f\n",
  //               a.acceleration.x, a.acceleration.y, a.acceleration.z,
  //               g.gyro.x, g.gyro.y, g.gyro.z);
  // delay(100);
  if (!bleKeyboard.isConnected()) ESP.restart();
  t1 = analogRead(thPin[0]);
  t2 = analogRead(thPin[1]);
  i1 = analogRead(inPin[0]);
  i2 = analogRead(inPin[1]);
  i3 = analogRead(inPin[2]);
  // if (i1 > itv && t1 < ttv) {
  // Serial.print("rl:0,ru:4096,");
  // Serial.printf("t1:%d,t2:%d,i1:%d,i2:%d,i3:%d\n", t1, t2, i1, i2, i3);
  // }
  // return;
  if (t1 < mtv || t2 < mtv) return;
  if (i1 > itv && t1 < ttv) {
    Serial.println("Left click");
    Serial.printf("%d\t%d\n", i1, t1);
    pressMoveRelease(inPin[0], MOUSE_LEFT);
  } else if (i2 > itv && t1 < ttv) {
    Serial.println("Cursor");
    Serial.printf("%d\t%d\n", i2, t1);
    clickOrVary(inPin[1], MOUSE_LEFT);
  } else if (i3 > itv && t1 < ttv) {
    Serial.println("Right click");
    Serial.printf("%d\t%d\n", i3, t1);
    pressMoveRelease(inPin[2], MOUSE_RIGHT);
  } else if (i1 > itv && t2 < ttv) {
    Serial.print("Extra 1  ");
    Serial.printf("%d\t%d\n", i1, t2);
    clickOrVary(inPin[0], KEY_MEDIA_MUTE, KEY_MEDIA_VOLUME_DOWN, KEY_MEDIA_VOLUME_UP);
  } else if (i2 > itv && t2 < ttv) {
    Serial.print("Extra 2  ");
    Serial.printf("%d\t%d\n", i2, t2);
    pressMoveRelease(inPin[0], MOUSE_MIDDLE);
  } else if (i3 > itv && t2 < ttv) {
    Serial.print("Extra 3  ");
    Serial.printf("%d\t%d\n", i3, t2);
    clickOrVary(inPin[2], KEY_MEDIA_PLAY_PAUSE, KEY_MEDIA_PREVIOUS_TRACK, KEY_MEDIA_NEXT_TRACK);
  }
}

inline void pressMoveRelease(const uint8_t pin, const uint8_t action) {
  bleMouse.press(action);
  delay(rt);
  i = millis()+jat;
  // Serial.printf("1. %d %d\n",i,millis());
  while (i > millis() && analogRead(pin) > itv);   // To avoid touch jerk
  // Serial.printf("2. %d %d\n",i,millis());
  if (i <= millis()) {                  // If not released soon, start cursor movement
    delay(rt);
    // Serial.printf("3. %d %d\n",i,millis());
    while (analogRead(pin) > itv) {
      mpu.getEvent(&a, &g, &temp);
      // Serial.print("rl:-1,ru:1,");
      // Serial.printf("gyr_x:%f,gyr_y:%f,gyr_z:%f\n",
      //               g.gyro.x, g.gyro.y, g.gyro.z);
      bleMouse.move(g.gyro.y * cf, g.gyro.z * cf);
    }
  }
  bleMouse.release(action);
  while(i>millis());
}

inline void clickOrVary(const uint8_t pin, const uint8_t action) {
  i = millis()+tbm;
  j=millis()+jat;
  // Serial.printf("1. %d %d\n",i,millis());
  while (analogRead(pin) > itv) {
    if (j > millis()) continue;
    mpu.getEvent(&a, &g, &temp);
    if(g.gyro.y * 5 || g.gyro.z * 5) {            
      while (analogRead(pin) > itv) {
        mpu.getEvent(&a, &g, &temp);
        Serial.print("rl:-0.5,ru:0.5,");
        Serial.printf("gyr_x:%f,gyr_y:%f,gyr_z:%f\n",
                      g.gyro.x, g.gyro.y, g.gyro.z);
        bleMouse.move(g.gyro.y * cf, g.gyro.z * cf);
      }
    }
  }
  if(i > millis()) bleMouse.click(action);
  while(j>millis());
}

inline void clickOrVary(const uint8_t pin, const MediaKeyReport action, const MediaKeyReport actionL, const MediaKeyReport actionR) {
  while(i + tbm > millis() && analogRead(pin) > itv);
  
  
  delay(jat);
  if (analogRead(pin) > itv)
    while (analogRead(pin) > itv) {
      mpu.getEvent(&a, &g, &temp);
      // Serial.print("rl:-1,ru:1,");
      // Serial.printf("acc_x:%f,acc_y:%f,acc_z:%f,gyr_x:%f,gyr_y:%f,gyr_z:%f\n",
      //               a.acceleration.x, a.acceleration.y, a.acceleration.z,
      //               g.gyro.x, g.gyro.y, g.gyro.z);
      bleMouse.move(g.gyro.y * cf, g.gyro.z * cf);
    }
  else bleKeyboard.write(action);
}
