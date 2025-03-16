#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <BleGamepad.h>

#define numOfButtons 14
#define numOfHatSwitches 1

Adafruit_MPU6050 mpu;
BleGamepad bleGamepad("Hash's Glove Controller","Hashoak",100);
BleGamepadConfiguration bleGamepadConfig;

const uint8_t thPin[2] = { 32, 33 };      // Touch pins on thumb finger
const uint8_t inPin[3] = { 27, 14, 12 };  // Touch pins on index finger

// Controller btn map
//    7                8
//    5                6
//        9   14   10
//  1 8 7              4
//  2 H 6     13     1   3
//  3 4 5              2
//         11    12
//                                 

// Glove
// 1 4 3
// 6 2 8

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

  Serial.println("Connecting MPU6050 chip...");
  if (!mpu.begin()) { Serial.print('.'); delay(500); }
  Serial.println();
  Serial.println("MPU6050 chip connected...");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  pinMode(thPin[0], INPUT);
  pinMode(thPin[1], INPUT);
  pinMode(inPin[0], INPUT);
  pinMode(inPin[1], INPUT);
  pinMode(inPin[2], INPUT);

  Serial.println("Starting BLE...");
  bleGamepadConfig.setAutoReport(false);
  bleGamepadConfig.setControllerType(CONTROLLER_TYPE_GAMEPAD); // CONTROLLER_TYPE_JOYSTICK, CONTROLLER_TYPE_GAMEPAD (DEFAULT), CONTROLLER_TYPE_MULTI_AXIS
  bleGamepadConfig.setButtonCount(numOfButtons);
  bleGamepadConfig.setHatSwitchCount(numOfHatSwitches);
  bleGamepadConfig.setWhichAxes(1, 1, 1, 1, 1, 1, 0, 0);	// Disable sliders
  bleGamepadConfig.setVid(0x054c);
  bleGamepadConfig.setPid(0x09cc);
  // Some non-Windows operating systems and web based gamepad testers don't like min axis set below 0, so 0 is set by default
  bleGamepadConfig.setAxesMin(0x8001); // -32767 --> int16_t - 16 bit signed integer - Can be in decimal or hexadecimal
  // bleGamepadConfig.setAxesMin(0x0000); // 0 --> int16_t - 16 bit signed integer - Can be in decimal or hexadecimal
  bleGamepadConfig.setAxesMax(0x7FFF); // 32767 --> int16_t - 16 bit signed integer - Can be in decimal or hexadecimal 
  bleGamepad.begin(&bleGamepadConfig); // Simulation controls, special buttons and hats 2/3/4 are disabled by default

  Serial.print("Connecting BLE...");
  while (!bleGamepad.isConnected()) { Serial.print('.'); delay(500); }
  Serial.println();
  Serial.println("BLE connected...");
}

int t1, t2, i1, i2, i3;
sensors_event_t a, g, temp;
double ax,az;
unsigned long i,j;

long map(double x, double in_min, double in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop() {
  // mpu.getEvent(&a, &g, &temp);
  // Serial.print("rl:-10,ru:10,rml:-2,rmu:2,");
  // Serial.printf("acc_x:%f,acc_y:%f,acc_z:%f\n",
  //               a.acceleration.x, a.acceleration.y, a.acceleration.z);
  // delay(100);
  if (!bleGamepad.isConnected()) ESP.restart();
  mpu.getEvent(&a, &g, &temp);
  ax=a.acceleration.x,az=a.acceleration.z;
  if(abs(ax)>0.75 || abs(az)>2) {
    if(ax>0) ax=map(min(ax,5.0),0,5,0,bleGamepadConfig.getAxesMax());
    else ax=map(max(ax,-3.0),0,-3,0,bleGamepadConfig.getAxesMin());
    if(az>0) az=map(min(az,6.0),0,6,0,bleGamepadConfig.getAxesMax());
    else az=map(max(az,-6.0),0,-6,0,bleGamepadConfig.getAxesMin());
    bleGamepad.setLeftThumb(az,ax);
  }
  else bleGamepad.setLeftThumb();
  // bleGamepad.sendReport();
  t1 = analogRead(thPin[0]);
  t2 = analogRead(thPin[1]);
  i1 = analogRead(inPin[0]);
  i2 = analogRead(inPin[1]);
  i3 = analogRead(inPin[2]);
  if (t1 > mtv && t2 > mtv)
    Serial.print("rl:0,ru:4096,"),
    Serial.printf("t1:%d,t2:%d,i1:%d,i2:%d,i3:%d\n", t1, t2, i1, i2, i3);
  if (t1 < mtv || t2 < mtv) { bleGamepad.resetButtons(); }
  else if (i1 > itv && t1 < ttv) {
    Serial.println("▢");
    // Serial.printf("%d\t%d\n", i1, t1);
    bleGamepad.press(1);
    // pressMoveRelease(inPin[0], MOUSE_LEFT);
  } else if (i2 > itv && t1 < ttv) {
    Serial.println("△");
    // Serial.printf("%d\t%d\n", i2, t1);
    bleGamepad.press(4);
    // clickOrVary(inPin[1], MOUSE_LEFT);
  } else if (i3 > itv && t1 < ttv) {
    Serial.println("○");
    // Serial.printf("%d\t%d\n", i3, t1);
    bleGamepad.press(3);
    // pressMoveRelease(inPin[2], MOUSE_RIGHT);
  } else if (i1 > itv && t2 < ttv) {
    Serial.print("R1");
    // Serial.printf("%d\t%d\n", i1, t2);
    bleGamepad.press(6);
    // clickOrVary(inPin[0], KEY_MEDIA_MUTE, KEY_MEDIA_VOLUME_DOWN, KEY_MEDIA_VOLUME_UP);
  } else if (i2 > itv && t2 < ttv) {
    Serial.print("X");
    // Serial.printf("%d\t%d\n", i2, t2);
    bleGamepad.press(2);
    // pressMoveRelease(inPin[0], MOUSE_MIDDLE);
  } else if (i3 > itv && t2 < ttv) {
    Serial.print("R2");
    // Serial.printf("%d\t%d\n", i3, t2);
    bleGamepad.setLeftTrigger(bleGamepadConfig.getAxesMax());
    // clickOrVary(inPin[2], KEY_MEDIA_PLAY_PAUSE, KEY_MEDIA_PREVIOUS_TRACK, KEY_MEDIA_NEXT_TRACK);
  }
  else 
    bleGamepad.resetButtons(),
    bleGamepad.setLeftTrigger(bleGamepadConfig.getAxesMin()); // Reset all axes to zero
  bleGamepad.sendReport();
  delay(100);
}
