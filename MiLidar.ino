#include <MPU9250_asukiaaa.h>
#include <Adafruit_BMP280.h>
#include "SerialCommand.h"
#include "PID.h"

#include "config.h"
#include "lidar.h"

#define CALIB_SEC 20

Adafruit_BMP280 bme; // I2C
MPU9250_asukiaaa imu;

SerialCommand sCmd;

#define LED_PIN 12
boolean ledState = LOW;

uint8_t sensorId;
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;

PID rpmPID(&motor_rpm, &pwm_val, &lidar_config.rpm_setpoint, lidar_config.Kp, lidar_config.Ki, lidar_config.Kd, DIRECT);

const byte eState_Find_COMMAND = 0;                        // 1st state: find 0xFA (COMMAND) in input stream
const byte eState_Build_Packet = eState_Find_COMMAND + 1;  // 2nd state: build the packet
int eState = eState_Find_COMMAND;

unsigned long curMillis;
unsigned long lastMillis = millis();


void setMagMinMaxAndSetOffset(MPU9250_asukiaaa* sensor, int seconds) {
  unsigned long calibStartAt = millis();
  float magX, magXMin, magXMax, magY, magYMin, magYMax, magZ, magZMin, magZMax;

  sensor->magUpdate();
  magXMin = magXMax = sensor->magX();
  magYMin = magYMax = sensor->magY();
  magZMin = magZMax = sensor->magZ();

  while(millis() - calibStartAt < (unsigned long) seconds * 1000) {
    delay(100);
    sensor->magUpdate();
    magX = sensor->magX();
    magY = sensor->magY();
    magZ = sensor->magZ();
    if (magX > magXMax) magXMax = magX;
    if (magY > magYMax) magYMax = magY;
    if (magZ > magZMax) magZMax = magZ;
    if (magX < magXMin) magXMin = magX;
    if (magY < magYMin) magYMin = magY;
    if (magZ < magZMin) magZMin = magZ;
  }

  sensor->magXOffset = - (magXMax + magXMin) / 2;
  sensor->magYOffset = - (magYMax + magYMin) / 2;
  sensor->magZOffset = - (magZMax + magZMin) / 2;
}

inline void ledOn(){
    digitalWrite(LED_PIN, LOW);
}

inline void ledOff(){
    digitalWrite(LED_PIN, HIGH);
}

inline void motorStart(){
    lidar_config.motor_enable = true;
    pwm(lidar_config.motorPin, PWM_FREQ, 255);  
}

inline void motorStop(){
    lidar_config.motor_enable = false;
    pwm(lidar_config.motorPin, PWM_FREQ, 0);  
}

inline void motorSet(int value){
    pwm(lidar_config.motorPin, PWM_FREQ, value);    
}

void motorCheck() {  // Make sure the motor RPMs are good else shut it down
  now = millis();
  if (now - motor_check_timer > motor_check_interval) {
    if ((motor_rpm < lidar_config.rpm_min or motor_rpm > lidar_config.rpm_max) and pwm_val > 1000) {
      rpm_err++;
    }
    else {
      rpm_err = 0;
    }
    if (rpm_err > rpm_err_thresh) {
      motorStop();
      ledState = LOW;
      digitalWrite(LED_PIN, ledState);
    }
    motor_check_timer = millis();
  }
}

/*
   doSetAngle - Multi-angle range(s) implementation - DSH
   Command: SetAngles ddd, ddd-ddd, etc.
   Enter with: N/A
   Uses:       lidar_config.aryAngles (an array of 360 booleans) is set to appropriate values
   Calls:      showDist
   Exits with: N/A
   TEST THIS STRING:  SetAngles 16-20, 300-305, 123-124, 10
*/
void setAngle() {
  char c, *arg;
  boolean syntax_error = false;
  int doing_from_to, from, to, ix, lToken, n_groups = 0;

  for (ix = 0; ix < N_ANGLES; ix++)                      // initialize
    lidar_config.arrayAngles[ix] = false;
  doing_from_to = 0;                                     // state = doing 'from'
  // Make sure that there is at least 1 angle or group of angles present
  do {
    arg = sCmd.next();                                   // get the next token
    if (arg == NULL) {                                   // it's empty -- just exit
      sCmd.readSerial();
      arg = sCmd.next();
      break;
    }
    // see if the token has an embedded "-", meaning from - to
    lToken = strlen(arg);                                // get the length of the current token
    for (ix = 0; ix < lToken; ix++) {
      c = arg[ix];
      if (c == ',') {                                    // optional trailing comma
        doing_from_to = 0;
        break;
      }
      else if (c == '-') {                               // optional '-' means "from - to"
        to = 0;
        doing_from_to = 1;                               // from now on, we're doing 'to'
      }
      else if (c == ' ') {                               // ignore blanks
        Serial.println(F("{ }"));
      }
      else if ((c >= '0') && (c <= '9')) {
        if (doing_from_to == 0) {
          from *= 10;
          from += c - '0';
          to = from;                                      // default to = from
          n_groups++;                                     // count the number of active groups (s/b >= 1)
        }
        else {
          to *= 10;
          to += c - '0';
        }
      }
      else {
        syntax_error = true;
        n_groups = 0;
        break;
      }
    }  // for (ix = 0; ix < lToken; ix++)
    // validate 'from' and 'to' and set 'lidar_config.aryAngles' with correct values
    if ((from >= 0) && (from < N_ANGLES) && (to >= 0) && (to < N_ANGLES)) {
      if (to >= from) {
        for (ix = from; ix <= to; ix++) {
          lidar_config.arrayAngles[ix] = true;
        }
      }
      else {
        syntax_error = true;
        break;
      }
    }
    else {
      syntax_error = true;
      break;
    }
    from = 0;
    to = 0;
    doing_from_to = 0;
  }  // do
  while (arg != NULL);
  if (n_groups == 0)
    syntax_error = true;

  // Handle syntax errors
  if (syntax_error) {
    Serial.println(F(" "));
    Serial.println(F("Incorrect syntax"));
    Serial.println(F("  Example: SetAngle 0, 15-30, 45-50, 10"));
    Serial.println(F("  Example: SetAngle 0-359 to show all angles."));
    Serial.println(F("Notes: Use a space after each comma"));
    Serial.println(F("       No particular order is required"));
    Serial.println(F("       In a from-to pair, the 1st value must be lowest. From-to pairs can overlap ranges."));
  }
  else {                                                  // no errors detected, display the angles and start
    // We're ready to process multiple angles
    Serial.println(F(""));
    Serial.print(F("Angles:"));
    for (int ix = 0; ix < N_ANGLES; ix++) {               // display the angle array
      if (lidar_config.arrayAngles[ix]) {
        Serial.print(ix, DEC);
        Serial.print(F(","));
      }
    }
    Serial.println(F(""));
    showDist();
  }  // if not (syntax_error)
}

void setRPM() {
  double sVal = 0.0;
  char *arg;
  boolean syntax_error = false;

  arg = sCmd.next();
  if (arg != NULL) {
    sVal = atof(arg);    // Converts a char string to a float
    if (sVal < lidar_config.rpm_min) {
      sVal = lidar_config.rpm_min;
      Serial.println(F(" "));
      Serial.print(F("RPM too low. Setting to minimum "));
      Serial.println(lidar_config.rpm_min);
    }
    if (sVal > lidar_config.rpm_max) {
      sVal = lidar_config.rpm_max;
      Serial.println(F(" "));
      Serial.print(F("RPM too high. Setting to maximum "));
      Serial.println(lidar_config.rpm_max);
    }
  }
  else {
    syntax_error = true;
  }

  arg = sCmd.next();
  if (arg != NULL) {
    syntax_error = true;
  }

  if (syntax_error) {
    Serial.println(F(" "));
    Serial.println(F("Incorrect syntax.  Example: SetRPM 200"));
  }
  else {
    Serial.print(F("Old RPM setpoint:"));
    Serial.println(lidar_config.rpm_setpoint);
    lidar_config.rpm_setpoint = sVal;
    //Serial.println(F(" "));
    Serial.print(F("New RPM setpoint: "));
    Serial.println(sVal);
  }
}

void setKp() {
  double sVal = 0.0;
  char *arg;
  boolean syntax_error = false;

  arg = sCmd.next();
  if (arg != NULL) {
    sVal = atof(arg);    // Converts a char string to a float
  }
  else {
    syntax_error = true;
  }

  arg = sCmd.next();
  if (arg != NULL) {
    syntax_error = true;
  }

  if (syntax_error) {
    Serial.println(F(" "));
    Serial.println(F("Incorrect syntax.  Example: SetKp 1.0"));
  }
  else {
    Serial.println(F(" "));
    Serial.print(F("Setting Kp to: "));
    Serial.println(sVal);
    lidar_config.Kp = sVal;
    rpmPID.SetTunings(lidar_config.Kp, lidar_config.Ki, lidar_config.Kd);
  }
}

void setKi() {
  double sVal = 0.0;
  char *arg;
  boolean syntax_error = false;

  arg = sCmd.next();
  if (arg != NULL) {
    sVal = atof(arg);    // Converts a char string to a float
  }
  else {
    syntax_error = true;
  }

  arg = sCmd.next();
  if (arg != NULL) {
    syntax_error = true;
  }

  if (syntax_error) {
    Serial.println(F(" "));
    Serial.println(F("Incorrect syntax.  Example: SetKi 0.5"));
  }
  else {
    Serial.println(F(" "));
    Serial.print(F("Setting Ki to: "));
    Serial.println(sVal);
    lidar_config.Ki = sVal;
    rpmPID.SetTunings(lidar_config.Kp, lidar_config.Ki, lidar_config.Kd);
  }
}

void setKd() {
  double sVal = 0.0;
  char *arg;
  boolean syntax_error = false;

  arg = sCmd.next();
  if (arg != NULL) {
    sVal = atof(arg);    // Converts a char string to a float
  }
  else {
    syntax_error = true;
  }

  arg = sCmd.next();
  if (arg != NULL) {
    syntax_error = true;
  }

  if (syntax_error) {
    Serial.println(F(" "));
    Serial.println(F("Incorrect syntax.  Example: SetKd 0.001"));
  }
  else {
    Serial.println(F(" "));
    Serial.print(F("Setting Kd to: "));
    Serial.println(sVal);
    lidar_config.Kd = sVal;
    rpmPID.SetTunings(lidar_config.Kp, lidar_config.Ki, lidar_config.Kd);
  }
}

void setSampleTime() {
  double sVal = 0.0;
  char *arg;
  boolean syntax_error = false;

  arg = sCmd.next();
  if (arg != NULL) {
    sVal = atoi(arg);    // Converts a char string to an integer
  }
  else {
    syntax_error = true;
  }

  arg = sCmd.next();
  if (arg != NULL) {
    syntax_error = true;
  }

  if (syntax_error) {
    Serial.println(F(" "));
    Serial.println(F("Incorrect syntax.  Example: SetSampleTime 20"));
  }
  else {
    Serial.println(F(" "));
    Serial.print(F("Setting Sample time to: "));
    Serial.println(sVal);
    lidar_config.sample_time = sVal;
    rpmPID.SetSampleTime(lidar_config.sample_time);
  }
}

void initSerialCommands() {
  sCmd.addCommand("help",        help);
  sCmd.addCommand("Help",        help);
  sCmd.addCommand("?",        help);
  sCmd.addCommand("ShowConfig",  showConfig);
  sCmd.addCommand("SaveConfig",  saveConfig);
  sCmd.addCommand("ResetConfig", initConfig);

  sCmd.addCommand("SetAngle",      setAngle);
  sCmd.addCommand("SetRPM",        setRPM);
  sCmd.addCommand("SetKp",         setKp);
  sCmd.addCommand("SetKi",         setKi);
  sCmd.addCommand("SetKd",         setKd);
  sCmd.addCommand("SetSampleTime", setSampleTime);

  sCmd.addCommand("MotorOff", motorStop);
  sCmd.addCommand("MotorOn",  motorStart);

  sCmd.addCommand("ShowRaw",  showRaw);
  sCmd.addCommand("HideRaw", hideRaw);
  sCmd.addCommand("ShowDist",  showDist);
  sCmd.addCommand("HideDist",  hideDist);
  sCmd.addCommand("ShowRPM",  showRPM);
  sCmd.addCommand("HideRPM",  hideRPM);
  sCmd.addCommand("ShowErrors", showErrors);
  sCmd.addCommand("HideErrors", hideErrors);
  sCmd.addCommand("ShowInterval", showInterval);
  sCmd.addCommand("HideInterval", hideInterval);
  sCmd.addCommand("ShowAll", showAll);
  sCmd.addCommand("HideAll", hideAll);
}

/*
   eValidatePacket - Validate 'Packet'
   Enter with: 'Packet' is ready to check
   Uses:       CalcCRC
   Exits with: 0 = Packet is okay
   Error:      non-zero = Packet is no good
*/
byte eValidatePacket() {
  unsigned long chk32;
  unsigned long checksum;
  const int bytesToCheck = PACKET_LENGTH - 2;
  const int CalcCRC_Len = bytesToCheck / 2;
  unsigned int CalcCRC[CalcCRC_Len];

  byte b1a, b1b, b2a, b2b;
  int ix;

  for (int ix = 0; ix < CalcCRC_Len; ix++)       // initialize 'CalcCRC' array
    CalcCRC[ix] = 0;

  // Perform checksum validity test
  for (ix = 0; ix < bytesToCheck; ix += 2)      // build 'CalcCRC' array
    CalcCRC[ix / 2] = Packet[ix] + ((Packet[ix + 1]) << 8);

  chk32 = 0;
  for (ix = 0; ix < CalcCRC_Len; ix++)
    chk32 = (chk32 << 1) + CalcCRC[ix];
  checksum = (chk32 & 0x7FFF) + (chk32 >> 15);
  checksum &= 0x7FFF;
  b1a = checksum & 0xFF;
  b1b = Packet[OFFSET_TO_CRC_L];
  b2a = checksum >> 8;
  b2b = Packet[OFFSET_TO_CRC_M];
  if ((b1a == b1b) && (b2a == b2b))
    return VALID_PACKET;                       // okay
  else
    return INVALID_PACKET;                     // non-zero = bad CRC
}

/*
   processIndex - Process the packet element 'index'
   index is the index byte in the 90 packets, going from A0 (packet 0, readings 0 to 3) to F9
      (packet 89, readings 356 to 359).
   Enter with: N/A
   Uses:       Packet
               ledState gets toggled if angle = 0
               ledPin = which pin the LED is connected to
               ledState = LED on or off
               lidar_config.show_dist = true if we're supposed to show distance
               curMillis = milliseconds, now
               lastMillis = milliseconds, last time through this subroutine
               lidar_config.show_interval = true ==> display time interval once per revolution, at angle 0
   Calls:      digitalWrite() - used to toggle LED pin
               Serial.print
   Returns:    The first angle (of 4) in the current 'index' group
*/
uint16_t processIndex() {
  uint16_t angle = 0;
  uint16_t data_4deg_index = Packet[OFFSET_TO_INDEX] - INDEX_LO;
  angle = data_4deg_index * N_DATA_QUADS;     // 1st angle in the set of 4
  if (angle == 0) {
    if (ledState) {
      ledState = LOW;
    }
    else {
      ledState = HIGH;
    }
    digitalWrite(LED_PIN, ledState);

    if (lidar_config.show_rpm) {
      Serial.print(F("R,"));
      Serial.print((int)motor_rpm);
      Serial.print(F(","));
      Serial.println((int)pwm_val);
    }

    curMillis = millis();
    if (lidar_config.show_interval) {
      Serial.print(F("T,"));                                // Time Interval in ms since last complete revolution
      Serial.println(curMillis - lastMillis);
    }
    lastMillis = curMillis;

  } // if (angle == 0)
  return angle;
}

/*
   processSpeed- Process the packet element 'speed'
   speed is two-bytes of information, little-endian. It represents the speed, in 64th of RPM (aka value
      in RPM represented in fixed point, with 6 bits used for the decimal part).
   Enter with: N/A
   Uses:       Packet
               angle = if 0 then enable display of RPM and PWM
               lidar_config.show_rpm = true if we're supposed to display RPM and PWM
   Calls:      Serial.print
*/
void processSpeed() {
  motor_rph_low_byte = Packet[OFFSET_TO_SPEED_LSB];
  motor_rph_high_byte = Packet[OFFSET_TO_SPEED_MSB];
  motor_rph = (motor_rph_high_byte << 8) | motor_rph_low_byte;
  motor_rpm = float( (motor_rph_high_byte << 8) | motor_rph_low_byte ) / 64.0;
}

/*
   processDistance- Process the packet element 'distance'
   Enter with: iQuad = which one of the (4) readings to process, value = 0..3
   Uses:       Packet
               dist[] = sets distance to object in binary: ISbb bbbb bbbb bbbb
                                       so maximum distance is 0x3FFF (16383 decimal) millimeters (mm)
   Calls:      N/A
   Exits with: 0 = okay
   Error:      1 << 7 = INVALID_DATA_FLAG is set
               1 << 6 = STRENGTH_WARNING_FLAG is set
*/
byte processDistance(int iQuad) {
  uint8_t dataL, dataM;
  aryDist[iQuad] = 0;                     // initialize
  int iOffset = OFFSET_TO_4_DATA_READINGS + (iQuad * N_DATA_QUADS) + OFFSET_DATA_DISTANCE_LSB;
  // byte 0 : <distance 7:0> (LSB)
  // byte 1 : <"invalid data" flag> <"strength warning" flag> <distance 13:8> (MSB)
  dataM = Packet[iOffset + 1];           // get MSB of distance data + flags
  if (dataM & BAD_DATA_MASK)             // if either INVALID_DATA_FLAG or STRENGTH_WARNING_FLAG is set...
    return dataM & BAD_DATA_MASK;        // ...then return non-zero
  dataL = Packet[iOffset];               // LSB of distance data
  aryDist[iQuad] = dataL | ((dataM & 0x3F) << 8);
  return 0;                              // okay
}

/*
   processSignalStrength- Process the packet element 'signal strength'
   Enter with: iQuad = which one of the (4) readings to process, value = 0..3
   Uses:       Packet
               quality[] = signal quality
   Calls:      N/A
*/
void processSignalStrength(int iQuad) {
  uint8_t dataL, dataM;
  aryQuality[iQuad] = 0;                        // initialize
  int iOffset = OFFSET_TO_4_DATA_READINGS + (iQuad * N_DATA_QUADS) + OFFSET_DATA_SIGNAL_LSB;
  dataL = Packet[iOffset];                  // signal strength LSB
  dataM = Packet[iOffset + 1];
  aryQuality[iQuad] = dataL | (dataM << 8);
}

void LidarWorker(){
  byte aryInvalidDataFlag[N_DATA_QUADS] = {0, 0, 0, 0}; // non-zero = INVALID_DATA_FLAG or STRENGTH_WARNING_FLAG is set

  if (Serial1.available() > 0) {                  // read byte from LIDAR and relay to USB
    inByte = Serial1.read();                      // get incoming byte:
    if (lidar_config.raw_data)
      Serial.write(inByte);                 // relay

    // Switch, based on 'eState':
    // State 1: We're scanning for 0xFA (COMMAND) in the input stream
    // State 2: Build a complete data packet
    if (eState == eState_Find_COMMAND) {          // flush input until we get COMMAND byte
      if (inByte == COMMAND) {
        eState++;                                 // switch to 'build a packet' state
        Packet[ixPacket++] = inByte;              // store 1st byte of data into 'Packet'
      }
    }
    else {                                            // eState == eState_Build_Packet
      Packet[ixPacket++] = inByte;                    // keep storing input into 'Packet'
      if (ixPacket == PACKET_LENGTH) {                // we've got all the input bytes, so we're done building this packet
        if (eValidatePacket() == VALID_PACKET) {      // Check packet CRC
          startingAngle = processIndex();             // get the starting angle of this group (of 4), e.g., 0, 4, 8, 12, ...
          processSpeed();                             // process the speed
          // process each of the (4) sets of data in the packet
          for (int ix = 0; ix < N_DATA_QUADS; ix++)   // process the distance
            aryInvalidDataFlag[ix] = processDistance(ix);
          for (int ix = 0; ix < N_DATA_QUADS; ix++) { // process the signal strength (quality)
            aryQuality[ix] = 0;
            if (aryInvalidDataFlag[ix] == 0)
              processSignalStrength(ix);
          }
          if (lidar_config.show_dist) {                           // the 'ShowDistance' command is active
            for (int ix = 0; ix < N_DATA_QUADS; ix++) {
              if (lidar_config.arrayAngles[startingAngle + ix]) {             // if we're supposed to display that angle
                if (aryInvalidDataFlag[ix] & BAD_DATA_MASK) {  // if LIDAR reported a data error...
                  if (lidar_config.show_errors) {                           // if we're supposed to show data errors...
                    Serial.print(F("A,"));
                    Serial.print(startingAngle + ix);
                    Serial.print(F(","));
                    if (aryInvalidDataFlag[ix] & INVALID_DATA_FLAG)
                      Serial.println(F("I"));
                    if (aryInvalidDataFlag[ix] & STRENGTH_WARNING_FLAG)
                      Serial.println(F("S"));
                  }
                }
                else {                                         // show clean data
                  Serial.print(F("A,"));
                  Serial.print(startingAngle + ix);
                  Serial.print(F(","));
                  Serial.print(int(aryDist[ix]));
                  Serial.print(F(","));
                  Serial.println(aryQuality[ix]);
                }
              }  // if (lidar_config.aryAngles[startingAngle + ix])
            }  // for (int ix = 0; ix < N_DATA_QUADS; ix++)
          }  // if (lidar_config.show_dist)
        }  // if (eValidatePacket() == 0
        else if (lidar_config.show_errors) {                                // we have encountered a CRC error
          Serial.println(F("C,CRC"));
        }
        // initialize a bunch of stuff before we switch back to State 1
        for (int ix = 0; ix < N_DATA_QUADS; ix++) {
          aryDist[ix] = 0;
          aryQuality[ix] = 0;
          aryInvalidDataFlag[ix] = 0;
        }
        for (ixPacket = 0; ixPacket < PACKET_LENGTH; ixPacket++)  // clear out this packet
          Packet[ixPacket] = 0;
        ixPacket = 0;
        eState = eState_Find_COMMAND;                // This packet is done -- look for next COMMAND byte
      }  // if (ixPacket == PACKET_LENGTH)
    }  // if (eState == eState_Find_COMMAND)
  }  // if (Serial1.available() > 0)
  if (lidar_config.motor_enable) {
    rpmPID.Compute();
    if (pwm_val != pwm_last) {
      pwm(lidar_config.motorPin, PWM_FREQ, pwm_val);  // replacement for analogWrite()
      pwm_last = pwm_val;
    }
    motorCheck();
  }  // if (lidar_config.motor_enable)
}

void setup() {  
    pinMode(LED_PIN, OUTPUT);
    ledOn();
    
    // put your setup code here, to run once:
    Serial.begin(115200);   // USB serial 
    while (!Serial);
    Serial1.begin(115200);  // LDS data

#ifdef _ESP32_HAL_I2C_H_ // For ESP32
    Wire.begin(SDA_PIN, SCL_PIN); // SDA, SCL
#else
    Wire.begin();
#endif

    imu.setWire(&Wire);
    while (imu.readId(&sensorId) != 0) {
      Serial.println("Cannot find IMU to read sensorId");
      delay(1000);
    }
    
    imu.beginAccel();
    imu.beginGyro();
    imu.beginMag();
    
//    You can set your own offset for mag values
//    imu.magXOffset = -50;
//    imu.magYOffset = -55;
//    imu.magZOffset = -10;

    lidar_config = lidar_config_store.read();
    if ((lidar_config.id != sensorId) && (lidar_config.id != 0x77)) { // verify values have been initialized
      initConfig();
    }

//    Serial.println("Start scanning values of magnetometer to get offset values.");
//    Serial.println("Rotate your device for " + String(CALIB_SEC) + " seconds.");
//    setMagMinMaxAndSetOffset(&imu, CALIB_SEC);
//    Serial.println("Finished setting offset values.");

    pinMode(lidar_config.motorPin, OUTPUT);

    rpmPID.SetOutputLimits(lidar_config.pwm_min, lidar_config.pwm_max);
    rpmPID.SetSampleTime(lidar_config.sample_time);
    rpmPID.SetTunings(lidar_config.Kp, lidar_config.Ki, lidar_config.Kd);
    rpmPID.SetMode(AUTOMATIC);
    
    initSerialCommands();

    eState = eState_Find_COMMAND;
    for (ixPacket = 0; ixPacket < PACKET_LENGTH; ixPacket++)  // Initialize
      Packet[ixPacket] = 0;
    ixPacket = 0;

    showConfig();
    help();

    delay(3000);
    showRaw();
    
    ledOff();
}

void loop() {
  int result;

  sCmd.readSerial();  // check for incoming serial commands
  LidarWorker();
//  result = imu.readId(&sensorId);
//  if (result == 0) {
//    Serial.println("sensorId: " + String(sensorId));
//  } else {
//    Serial.println("Cannot read sensorId " + String(result));
//  }
//
//  result = imu.accelUpdate();
//  if (result == 0) {
//    aX = imu.accelX();
//    aY = imu.accelY();
//    aZ = imu.accelZ();
//    aSqrt = imu.accelSqrt();
//    Serial.println("accelX: " + String(aX));
//    Serial.println("accelY: " + String(aY));
//    Serial.println("accelZ: " + String(aZ));
//    Serial.println("accelSqrt: " + String(aSqrt));
//  } else {
//    Serial.println("Cannot read accel values " + String(result));
//  }
//
//  result = imu.gyroUpdate();
//  if (result == 0) {
//    gX = imu.gyroX();
//    gY = imu.gyroY();
//    gZ = imu.gyroZ();
//    Serial.println("gyroX: " + String(gX));
//    Serial.println("gyroY: " + String(gY));
//    Serial.println("gyroZ: " + String(gZ));
//  } else {
//    Serial.println("Cannot read gyro values " + String(result));
//  }
//
//  result = imu.magUpdate();
//  if (result == 0) {
//    mX = imu.magX();
//    mY = imu.magY();
//    mZ = imu.magZ();
//    mDirection = imu.magHorizDirection();
////    Serial.println("imu.magXOffset = " + String(imu.magXOffset) + ";");
////    Serial.println("imu.maxYOffset = " + String(imu.magYOffset) + ";");
////    Serial.println("imu.magZOffset = " + String(imu.magZOffset) + ";");
//    Serial.println("magX: " + String(mX));
//    Serial.println("maxY: " + String(mY));
//    Serial.println("magZ: " + String(mZ));
//    Serial.println("horizontal direction: " + String(mDirection));
//  } else {
//    Serial.println("Cannot read mag values " + String(result));
//  }
//
//  Serial.println("at " + String(millis()) + "ms");
//  Serial.println(""); // Add an empty line
//  delay(500);
  
//  Serial.print(Serial1.read());
}
