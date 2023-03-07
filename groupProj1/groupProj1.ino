#include <stdint.h>
#include "ICM_20948.h"
#include <LiquidCrystal.h>

LiquidCrystal lcd(17, D2, D6, D5, D4, D3);

ICM_20948_I2C myICM;
#define WIRE_PORT Wire
#define AD0_VAL 1

const uint ledClkPin = D7;
const uint ledLatchPin = D8;
const uint ledDataPin = D9;
const uint btnPin = A0;
const uint vmotorPin = 12;

void printScore(uint score) {
  // print "Score: ##" to first row of LCD
  lcd.setCursor(0, 1);
  lcd.print("Score: " + String(score));
  
  Serial.println("Game Over!");
  Serial.print("Score: ");
  Serial.println(score);
}

void printLives(uint lives) {
  // print "Lives: #" to second row of LCD
  lcd.setCursor(0, 0);
  lcd.print("Lives: " + String(lives));
  
  // If lives == 0, also print Game/Over to right side of LCD (4x2)
  if (lives == 0) {
    lcd.setCursor(12,0);
    lcd.print("GAME");
    lcd.setCursor(12,1);
    lcd.print("OVER");
  }
}

void writeToLEDMatrix(uint8_t rows, uint8_t cols_r, uint8_t cols_g) {
  digitalWrite(ledLatchPin, 0);
  shiftOut(ledDataPin, ledClkPin, MSBFIRST, ~cols_g);
  shiftOut(ledDataPin, ledClkPin, MSBFIRST, ~cols_r);
  shiftOut(ledDataPin, ledClkPin, MSBFIRST, rows);
  digitalWrite(ledLatchPin, 1);
}

void clearLEDMatrix() {
  writeToLEDMatrix(0,0,0);
}

void inline writeRedPoint(const uint8_t &x, const uint8_t &y) {
  writeToLEDMatrix(1<<y, 1<<x, 0);
}

void inline writeGreenPoint(const uint8_t &x, const uint8_t &y) {
  writeToLEDMatrix(1<<y, 0, 1<<x);
}

void inline writeOrangePoint(const uint8_t &x, const uint8_t &y) {
  uint8_t x1 = 1<<x;
  writeToLEDMatrix(1<<y, x1, x1);
}


void gyroToXY(const icm_20948_DMP_data_t &data, uint8_t *x_out, uint8_t *y_out) {

  // We have asked for GRV data so we should receive Quat6
  if (!((data.header & DMP_header_bitmap_Quat6) > 0))
    return;

  //Serial.printf("Quat6 data is: Q1:%ld Q2:%ld Q3:%ld\r\n", data.Quat6.Data.Q1, data.Quat6.Data.Q2, data.Quat6.Data.Q3);

  // Scale to +/- 1
  double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
  double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
  double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

  // Convert the quaternions to Euler angles (roll, pitch, yaw)
  // https://en.wikipedia.org/w/index.php?title=Conversion_between_quaternions_and_Euler_angles&section=8#Source_code_2

  // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
  // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
  // The quaternion data is scaled by 2^30.

  double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
  double q2sqr = q2 * q2;

  // roll (x-axis rotation)
  double t0 = +2.0 * (q0 * q1 + q2 * q3);
  double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
  double roll = atan2(t0, t1) * 180.0 / PI;

  // pitch (y-axis rotation)
  double t2 = +2.0 * (q0 * q2 - q3 * q1);
  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  double pitch = asin(t2) * 180.0 / PI;

  // // yaw (z-axis rotation)
  // double t3 = +2.0 * (q0 * q3 + q1 * q2);
  // double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
  // double yaw = atan2(t3, t4) * 180.0 / PI;

  *x_out = (uint8_t)(-pitch / 8.0 + 4) % 8;
  *y_out = (uint8_t)( -roll / 8.0 + 4) % 8;

  // Serial.print(F("Roll:"));
  // Serial.print(roll, 1);
  // Serial.print(F(" Pitch:"));
  // Serial.print(pitch, 1);

  // Serial.print(F(" X:"));
  // Serial.print(-pitch / 8.0 + 4, 1);
  // Serial.print(F(" Y:"));
  // Serial.print(-roll / 8.0 + 4, 1);

  // Serial.print(F(" X:"));
  // Serial.print(*x_out);
  // Serial.print(F(" Y:"));
  // Serial.println(*y_out);
}

void updateGamePointer(icm_20948_DMP_data_t *data, uint8_t *x_out, uint8_t *y_out) {
  while (true) {
    myICM.readDMPdataFromFIFO(data);
    while (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail) {
      myICM.readDMPdataFromFIFO(data);
    }
    if (myICM.status == ICM_20948_Stat_Ok) {
      gyroToXY(*data, x_out, y_out);
      writeRedPoint(*x_out, *y_out);
      return;
    }
  }
}


void setup() {
  Serial.begin(115200);

  pinMode(btnPin, INPUT);
  pinMode(vmotorPin, OUTPUT);

  pinMode(ledLatchPin, OUTPUT);
  pinMode(ledClkPin, OUTPUT);
  pinMode(ledDataPin, OUTPUT);
  clearLEDMatrix();
  writeToLEDMatrix(0xff, 0xff, 0);
  delay(1000);
  writeToLEDMatrix(0xff, 0, 0xff);
  delay(1000);


  // Initialize LCD
  lcd.begin(16,2);
  lcd.print("test1");

  // Setup Gyro
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  while (1)
  {
    // Initialize the ICM-20948
    // If the DMP is enabled, .begin performs a minimal startup. We need to configure the sample mode etc. manually.
    myICM.begin(WIRE_PORT, AD0_VAL);

    if (myICM.status == ICM_20948_Stat_Ok)
      break;

    Serial.println("Waiting on gyro...");
    delay(500);
  }

  bool success = true; // Use success to show if the DMP configuration was successful

  // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

  // DMP sensor options are defined in ICM_20948_DMP.h
  //    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
  //    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
  //    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
  //    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
  //    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
  //    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
  //    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
  //    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
  //    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
  //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
  //    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
  //    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

  // Enable the DMP Game Rotation Vector sensor
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);

  // Enable any additional sensors / features
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

  // Configuring DMP to output data at multiple ODRs:
  // DMP is capable of outputting multiple sensor data at different rates to FIFO.
  // Setting value can be calculated as follows:
  // Value = (DMP running rate / ODR ) - 1
  // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum

  // Enable the FIFO
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  // Check success
  if (success)
  {
    Serial.println(F("DMP enabled!"));
  }
  else
  {
    Serial.println(F("Enable DMP failed!"));
    Serial.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
    while (1) {
      writeToLEDMatrix(0xFF, 0xFF, 0);
      delay(1000);
      writeToLEDMatrix(0, 0, 0);
      delay(1000);
    }
  }
}




int delta = 10;
icm_20948_DMP_data_t data;

uint8_t pointer_x = 0;
uint8_t pointer_y = 0;


uint8_t num_targets = 0;
uint score = 0;
bool target_miss = false;
bool new_game = true;


void loop() {

  if (new_game) {
    new_game = false;
    score = 0;
    num_targets = 0;
  } else {
    num_targets++;
    score++;
  }

  printScore(score);
  clearLEDMatrix();
  delay(300);

  // determine and show new target positions
  uint8_t target_x[num_targets];
  uint8_t target_y[num_targets];
  for (int i = 0; i < num_targets; i++) {
    target_x[i] = random(8);
    target_y[i] = random(8);
    writeGreenPoint(target_x[i], target_y[i]);
    delay(800);
  }
  delay(300);
  clearLEDMatrix();
  delay(400);

  // guessing logic
  int mistakes_left = 3;
  for (int i = 0; i < num_targets; i++) {
    const uint8_t tx = target_x[i];
    const uint8_t ty = target_y[i];

    myICM.resetFIFO();
    while (true) {
      // navigate to a target
      updateGamePointer(&data, &pointer_x, &pointer_y);

      // lock it in
      // trigger on debounced rising edge then wait for falling edge
      if (!digitalRead(btnPin)) continue;
      delay(50);
      if (!digitalRead(btnPin)) continue;
      while (digitalRead(btnPin)) delay(10);
      // break;

      // check for incorrect match
      if ((pointer_x != tx) || (pointer_y != ty)) {
        writeToLEDMatrix(0xFF, 0xFF, 0);
        digitalWrite(vmotorPin, 1);

        mistakes_left--;
        printLives(mistakes_left);

        if (mistakes_left) {
          // try again
          delay(150);
          digitalWrite(vmotorPin, 0);
          continue;
        }

        // game over
        for (uint i = 0; i < 3; i++) {
          digitalWrite(vmotorPin, 1);  delay(200);
          digitalWrite(vmotorPin, 0);  delay(75);
        }
        // printScore(score);

        // wait for button press to start new game
        while (true) {
          // trigger on debounced rising edge then wait for falling edge
          if (!digitalRead(btnPin)) continue;
          delay(50);
          if (!digitalRead(btnPin)) continue;
          while (digitalRead(btnPin)) delay(10);

          

          new_game = true;
          return;
        }
      } else {
        break;
      }
    }
  }

  // show green animation before next round
  for (uint8_t c = 1; c <= 8; c++) {
    writeToLEDMatrix(0xff, 0, (1<<c)-1);
    delay(30);
  }
  for (uint8_t c = 1; c <= 8; c++) {
    writeToLEDMatrix(0xff, 0, ~((1<<c)-1));
    delay(30);
  }
}
