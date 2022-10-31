#include <FlashStorage.h>

const int N_ANGLES = 360;                // # of angles (0..359)

typedef struct {
  byte id;
  char version[6];
  int motorPin;            // pin connected to mosfet for motor speed control
  double rpm_setpoint;          // desired RPM (uses double to be compatible with PID library)
  double rpm_min;
  double rpm_max;
  double pwm_max;              // max analog value.  probably never needs to change from 1023
  double pwm_min;              // min analog pulse value to spin the motor
  int sample_time;             // how often to calculate the PID values

  // PID tuning values
  double Kp;
  double Ki;
  double Kd;

  boolean motor_enable;        // to spin the laser or not.  No data when not spinning
  boolean raw_data;            // to retransmit the seiral data to the USB port
  boolean show_dist;           // controlled by ShowDist and HideDist commands
  boolean show_rpm;            // controlled by ShowRPM and HideRPM commands
  boolean show_interval;       // true = show time interval, once per revolution, at angle=0
  boolean show_errors;         // Show CRC, signal strength and invalid data errors
  boolean arrayAngles[N_ANGLES]; // array of angles to display
} LidarConfig;

FlashStorage(lidar_config_store, LidarConfig);

LidarConfig lidar_config;

inline void saveConfig() {
  lidar_config_store.write(lidar_config);
  Serial.println(F("Config Saved."));
}

void hideRaw() {
  lidar_config.raw_data = false;
}

void showDist() {
  hideRaw();
  if (lidar_config.show_dist == false) {                  // suppress activity message if we're executing 'show distance'
    Serial.println(F(" "));
    Serial.println(F("Code,Angle,Distance(mm),Signal strength"));
  }
  lidar_config.show_dist = true;
}

void hideDist() {
  lidar_config.show_dist = false;
  if (lidar_config.show_dist == false) {                  // suppress activity message if we're executing 'show distance'
    Serial.println(F(" "));
    Serial.println(F("Hiding Distance data"));
  }
}

void showInterval() {
  lidar_config.show_interval = true;
  if (lidar_config.show_dist == false) {                  // suppress activity message if we're executing 'show distance'
    Serial.println(F(" "));
    Serial.println(F("Showing time interval (ms per revolution)"));
  }
}

void hideInterval() {
  lidar_config.show_interval = false;
  if (lidar_config.show_dist == false) {                  // suppress activity message if we're executing 'show distance'
    Serial.println(F(" "));
    Serial.println(F("Hiding time interval"));
  }
}


void showErrors() {
  lidar_config.show_errors = true;                                  // enable error display
  if (lidar_config.show_dist == false) {                  // suppress activity message if we're executing 'show distance'
    Serial.println(F(" "));
    Serial.println(F("Showing errors"));
  }
}

void hideErrors() {                                    // disable error display
  lidar_config.show_errors = false;
  if (lidar_config.show_dist == false) {                  // suppress activity message if we're executing 'show distance'
    Serial.println(F(" "));
    Serial.println(F("Hiding errors"));
  }
}

/*
   showRPM
*/
void showRPM() {
  lidar_config.show_rpm = true;
  if (lidar_config.raw_data == true) {
    hideRaw();
  }
  if (lidar_config.show_dist == false) {                  // suppress activity message if we're executing 'show distance'
    Serial.println(F(" "));
    Serial.println(F("Showing RPM data"));
  }
}

/*
   hideRPM
*/
void hideRPM() {
  lidar_config.show_rpm = false;
  if (lidar_config.show_dist == false) {                  // suppress activity message if we're executing 'show distance'
    Serial.println(F(" "));
    Serial.println(F("Hiding RPM data"));
  }
}

void showRaw() {
  lidar_config.raw_data = true;
  hideDist();
  hideRPM();
}

void showAll() {
  showDist();
  showErrors();
  showRPM();
  showInterval();
}

void hideAll() {
  hideDist();
  hideErrors();
  hideRPM();
  hideInterval();
}

void initConfig() {
  lidar_config.id = 0x77;
  strcpy(lidar_config.version, "1.4.2");

  lidar_config.motorPin = 10;  // pin connected N-Channel Mosfet

  lidar_config.rpm_setpoint = 200;  // desired RPM
  lidar_config.rpm_min = 180;
  lidar_config.rpm_max = 349;
  lidar_config.pwm_min = 100;
  lidar_config.pwm_max = 1023;
  lidar_config.sample_time = 20;
  lidar_config.Kp = 2.0;
  lidar_config.Ki = 1.0;
  lidar_config.Kd = 0.0;

  lidar_config.motor_enable = true;
  lidar_config.raw_data = false;
  lidar_config.show_dist = true;
  lidar_config.show_rpm = true;
  lidar_config.show_interval = true;
  lidar_config.show_errors = true;
  for (int ix = 0; ix < N_ANGLES; ix++)
    lidar_config.arrayAngles[ix] = true;

  lidar_config_store.write(lidar_config);
}

void showConfig() {
  if (lidar_config.raw_data == true) {
    hideRaw();
  }
  Serial.println(F(" "));
  Serial.println(F(" "));

  Serial.print(F("XV Lidar Controller Firmware Version "));
  Serial.println(lidar_config.version);

  Serial.println(F(" "));

  Serial.print(F("PWM pin: "));
  Serial.println(lidar_config.motorPin);

  Serial.print(F("Target RPM: "));
  Serial.println(lidar_config.rpm_setpoint);

  Serial.print(F("Max PWM: "));
  Serial.println(lidar_config.pwm_max);
  Serial.print(F("Min PWM: "));
  Serial.println(lidar_config.pwm_min);

  Serial.print(F("PID Kp: "));
  Serial.println(lidar_config.Kp);
  Serial.print(F("PID Ki: "));
  Serial.println(lidar_config.Ki);
  Serial.print(F("PID Kd: "));
  Serial.println(lidar_config.Kd);
  Serial.print(F("SampleTime: "));
  Serial.println(lidar_config.sample_time);

  Serial.print(F("Motor Enable: "));
  Serial.println(lidar_config.motor_enable);
  Serial.print(F("Show Raw Data: "));
  Serial.println(lidar_config.raw_data);
  Serial.print(F("Show Dist Data: "));
  Serial.println(lidar_config.show_dist);
  Serial.print(F("Show RPM Data: "));
  Serial.println(lidar_config.show_rpm);
  Serial.print(F("Show Time Interval: "));
  Serial.println(lidar_config.show_interval);
  Serial.print(F("Show Angle(s): "));
  for (int ix = 0; ix < N_ANGLES; ix++) {               // display the angle array
    if (lidar_config.arrayAngles[ix]) {
      Serial.print(ix, DEC);
      Serial.print(F(","));
    }
  }
  Serial.println(F(" "));
  Serial.println(F(" "));
}

void help() {
  if (lidar_config.raw_data == true) {
    hideRaw();
  }
  Serial.println(F(" "));
  Serial.println(F(" "));

  Serial.print(F("XV Lidar Controller Firmware Version "));
  Serial.println(lidar_config.version);

  Serial.println(F(" "));
  Serial.println(F(" "));

  Serial.println(F("List of available commands"));

  Serial.println(F(" "));
  Serial.println(F("Control commands"));
  Serial.println(F("  ShowConfig    - Show the running configuration"));
  Serial.println(F("  SaveConfig    - Save the running configuration to EEPROM"));
  Serial.println(F("  ResetConfig   - Restore the original configuration"));
  Serial.println(F("  SetAngle      - Show distance data for a multiple angles (Ex: SetAngle 0, 15-30, 45-50, 10)"));
  Serial.println(F("  SetRPM        - Set the desired rotation speed (min: 180, max: 349)"));
  Serial.println(F("  MotorOff      - Stop spinning the lidar"));
  Serial.println(F("  MotorOn       - Enable spinning of the lidar"));

  Serial.println(F(" "));
  Serial.println(F("Data commands"));
  Serial.println(F("  ShowRaw       - Enable the output of the raw lidar data (default)"));
  Serial.println(F("  HideRaw       - Stop outputting the raw data from the lidar"));
  Serial.println(F("  ShowDist      - Show angles with distance data"));
  Serial.println(F("  HideDist      - Hide the distance data"));
  Serial.println(F("  ShowErrors    - Show all error types (CRC, Signal Strength, and Invalid"));
  Serial.println(F("  HideErrors    - Hide angles with errors"));
  Serial.println(F("  ShowRPM       - Show the rotation speed"));
  Serial.println(F("  HideRPM       - Hide the rotation speed"));
  Serial.println(F("  ShowInterval  - Show time interval per revolution in ms, at angle=0"));
  Serial.println(F("  HideInterval  - Hide time interval"));
  Serial.println(F("  ShowAll       - Show the distance, errors, RPMs and interval data"));
  Serial.println(F("  HideAll       - Hide the distance, errors, RPMs and interval data"));

  Serial.println(F(" "));
  Serial.println(F("PID commands"));
  Serial.println(F("  SetKp         - Set the proportional gain"));
  Serial.println(F("  SetKi         - Set the integral gain"));
  Serial.println(F("  SetKd         - Set the derivative gain"));
  Serial.println(F("  SetSampleTime - Set the frequency the PID is calculated (ms)"));

  Serial.println(F(" "));
  Serial.println(F("Output comma-separated format:"));
  Serial.println(F("  A,<Angle>,<Distance in mm>,<Strength>"));
  Serial.println(F("  C,CRC error was generated by LIDAR"));
  Serial.println(F("  R,<RPMs>,<PWM value>"));
  Serial.println(F("  T,<Time interval in milliseconds>"));

  Serial.println(F(" "));
  Serial.println(F("Errors:"));
  Serial.println(F("  CRC = CRC Error"));
  Serial.println(F("    I = LIDAR reports Invalid data for this angle"));
  Serial.println(F("    S = LIDAR reports Poor signal strength for this angle"));
  Serial.println(F(" "));
}
