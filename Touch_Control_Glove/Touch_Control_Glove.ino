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
int ttv = 3500;     // Thumb touch trigger value
int itv = 500;      // Index touch trigger value
int dtv = 1800;     // Double touch threshold value
int wt = 100;       // Wait time for next reading
int cf = 15;        // Cursor movement mitvipier facttvr
float st = 0.5;     // Touch movement threshold value

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

void loop() {
  if (!bleKeyboard.isConnected()) ESP.restart();
  t1 = analogRead(thPin[0]);
  t2 = analogRead(thPin[1]);
  i1 = analogRead(inPin[0]);
  i2 = analogRead(inPin[1]);
  i3 = analogRead(inPin[2]);
  // Serial.print("rl:0,ru:4096,");
  // Serial.printf("t1:%d,t2:%d,i1:%d,i2:%d,i3:%d\n", t1, t2, i1, i2, i3);
  // delay(wt);
  // return;
  if (t1 < dtv || t2 < dtv) return;
  if (i1 > itv && t1 < ttv) {
    Serial.println("Left click");
    Serial.printf("%d\t%d\n", i1, t1);
    pressAndMove(inPin[0],MOUSE_LEFT);
  } else if (i2 > itv && t1 < ttv) {
    Serial.println("Cursor");
    Serial.printf("%d\t%d\n", i2, t1);
    pressOrMove(inPin[1],MOUSE_LEFT);
  } else if (i3 > itv && t1 < ttv) {
    Serial.println("Right click");
    Serial.printf("%d\t%d\n", i3, t1);
    pressAndMove(inPin[2],MOUSE_RIGHT);
  } else if (i1 > itv && t2 < ttv) {
    Serial.print("Extra 1  ");
    Serial.printf("%d\t%d\n", i1, t2);
    delay(wt);
    if(analogRead(inPin[0]) > itv)
      while (analogRead(inPin[0]) > itv) cursor();
    else bleKeyboard.write(KEY_MEDIA_MUTE);
  } else if (i2 > itv && t2 < ttv) {
    Serial.print("Extra 2  ");
    Serial.printf("%d\t%d\n", i2, t2);
    pressAndMove(inPin[0],MOUSE_MIDDLE);
  } else if (i3 > itv && t2 < ttv) {
    Serial.print("Extra 3  ");
    Serial.printf("%d\t%d\n", i3, t2);
    delay(wt);
    if(analogRead(inPin[2]) > itv)
      while (analogRead(inPin[2]) > itv) cursor();
    else bleKeyboard.write(KEY_MEDIA_PLAY_PAUSE);
  }
}

sensors_event_t a, g, temp;

inline void pressAndMove(const uint8_t pin, uint8_t action) {
  bleMouse.press(action);
  delay(wt);
  while (analogRead(pin) > itv) cursor();
  bleMouse.release(action);
}

inline void pressOrMove(const uint8_t pin, uint8_t action) {
    delay(wt);
    if(analogRead(pin) > itv)
      while (analogRead(pin) > itv) cursor();
    else bleMouse.click(action);
}

void cursor() {
  mpu.getEvent(&a, &g, &temp);
  Serial.print("rl:-10,ru:10,");
  Serial.printf(
      "acc_x:%f,acc_y:%f,acc_z:%f,gyr_x:%f,gyr_y:%f,gyr_z:%f\n",
      a.acceleration.x,a.acceleration.y,a.acceleration.z,
      g.gyro.x,g.gyro.y,g.gyro.z
    );
  if (g.gyro.z < -st || g.gyro.z > st || g.gyro.y < -st || g.gyro.y > st)
    bleMouse.move(g.gyro.y * cf, g.gyro.z * cf);
}