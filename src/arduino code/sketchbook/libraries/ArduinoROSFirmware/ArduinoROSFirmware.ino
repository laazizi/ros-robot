#include "Arduino.h"
#include "commands.h"
#include "sensors.h"
#include <Servo.h>
#include "servos.h"
#include <Wire.h>
#include "Adafruit_BMP085.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_ADXL345.h"
#include "L3G.h"
#include "motor_driver.h"
#include "encoder_driver.h"
#include <PID_v1.h>
#include "diff_controller.h"

#define BAUDRATE     57600

/* PID loop parameters */
// MAX Ticks per second is about 30-50 with current encoders!!!!
#define PID_RATE           10.0     // Hz
const int PID_INTERVAL = 1000 / PID_RATE;
unsigned long nextPID = PID_INTERVAL;

/* Stop the robot if it hasn't received a movement command
 in this number of milliseconds */
#define AUTO_STOP_INTERVAL 2000
long lastMotorCommand = AUTO_STOP_INTERVAL;

/* Pressure sensor */
Adafruit_BMP085 bmp;

/* Accelerometer */
Adafruit_ADXL345 accel = Adafruit_ADXL345(12345);
sensors_event_t event;

/* Gyroscope */
L3G gyro;

// A pair of varibles to help parse Serial1 commands (thanks Fergs)
int arg = 0;
int index = 0;
char chr; // Variable to hold an input character
char cmd; // Variable to hold the current single-character command

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

  switch(cmd) {
  case GET_BAUDRATE:
    Serial1.println(BAUDRATE);
    break;
  case ANALOG_READ:
    Serial1.println(analogRead(arg1));
    break;
  case DIGITAL_READ:
    Serial1.println(digitalRead(arg1));
    break;
  case ANALOG_WRITE:
    analogWrite(arg1, arg2);
    Serial1.println("OK"); 
    break;
  case DIGITAL_WRITE:
    if (arg2 == 0) digitalWrite(arg1, LOW);
    else if (arg2 == 1) digitalWrite(arg1, HIGH);
    Serial1.println("OK"); 
    break;
  case ENVIR_DATA: 
    Serial1.print(bmp.readTemperature());
    Serial1.print(" ");
    Serial1.print(bmp.readPressure());
    Serial1.print(" ");
    Serial1.println(bmp.readAltitude()); 
    break;
  case ACCELERATION:
    Serial1.print("X: "); 
    Serial1.print(event.acceleration.x); 
    Serial1.print("  ");
    Serial1.print("Y: "); 
    Serial1.print(event.acceleration.y); 
    Serial1.print("  ");
    Serial1.print("Z: "); 
    Serial1.print(event.acceleration.z); 
    Serial1.print("  ");
    Serial1.println("m/s^2 ");
    break;
  case GYROSCOPE:
    gyro.read();
    Serial1.print("G ");
    Serial1.print("X: ");
    Serial1.print((int)gyro.g.x);
    Serial1.print(" Y: ");
    Serial1.print((int)gyro.g.y);
    Serial1.print(" Z: ");
    Serial1.println((int)gyro.g.z);
    break;
  case PIN_MODE:
    if (arg2 == 0) pinMode(arg1, INPUT);
    else if (arg2 == 1) pinMode(arg1, OUTPUT);
    Serial1.println("OK");
    break;
  case PING:
    Serial1.println(Ping(arg1));
    break;
  case SERVO_WRITE:
    servos[arg1].write(arg2);
    Serial1.println("OK");
    break;
  case SERVO_READ:
    Serial1.println(servos[arg1].read());
    break;
  case MOTOR_SPEEDS:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0) {
      setMotorSpeeds(0, 0);
      moving = 0;
    }
    else moving = 1;
    //Set target encoder ticks per frame
    if(arg1<0) {
      leftPID.f = -1;    
    }
    else { 
      leftPID.f = 1;
    }
    leftPID.SetpointTicks = arg1/PID_RATE * leftPID.f;

    if(arg2<0) {
      rightPID.f = -1;
    }
    else { 
      rightPID.f = 1; 
    }
    rightPID.SetpointTicks = arg2/PID_RATE * rightPID.f;

    Serial1.println("OK"); 
    break;
  case READ_ENCODERS:
    Serial1.print(readEncoder(LEFT));
    Serial1.print(" ");
    Serial1.println(readEncoder(RIGHT));
    break;
  case RESET_ENCODERS:
    resetEncoders();
    resetPID();
    Serial1.println("OK");
    break;
  case UPDATE_PID:
    while ((str = strtok_r(p, ":", &p)) != '\0') {
      pid_args[i] = atoi(str);
      i++;
    }
    Kp = pid_args[0];
    Ki = pid_args[1];
    Kd = pid_args[2];
    myPIDL.SetTunings(Kp,Ki,Kd);
    myPIDR.SetTunings(Kp,Ki,Kd);
    Serial1.println("OK");
    break;
  case IR:
    Serial1.println("OK");
    break;
  default:
    Serial1.println("Invalid Command");
    break;
  }
}


void setup() {
  Serial1.begin(BAUDRATE);
  Serial.begin(BAUDRATE);
  Wire.begin();

  initMotorController();
  initEncoders();

  /* Attach servos if used */
  int i;
  for (i = 0; i < N_SERVOS; i++) {
    servos[i].attach(servoPins[i]);
  }
  servos[0].write(0);
  servos[1].write(90);

  myPIDL.SetMode(AUTOMATIC);
  myPIDR.SetMode(AUTOMATIC);
  myPIDL.SetOutputLimits(0,255);
  myPIDR.SetOutputLimits(0,255);

  /*
  if (!bmp.begin()) {
   Serial1.println("Could not find a valid BMP085 sensor, check wiring!");
   while (1) {
   }
   }
   
   if(!accel.begin())
   {
   Serial1.println("Ooops, no ADXL345 detected ... Check your wiring!");
   while(1);
   }
   
   if (!gyro.init())
   {
   Serial1.println("Failed to autodetect gyro type!");
   while (1);
   }
   
   gyro.enableDefault();
   */
}



/* Enter the main loop.  Read and parse input from the Serial1 port
 and run any valid commands. Run a PID calculation at the target
 interval and check for auto-stop conditions.
 */
void loop() {

  /* Accelerometer update*/
  //accel.getEvent(&event);
  unsigned long time;
  time = millis();
  Serial.println(time);
  while (Serial1.available() > 0) {

    // Read the next character
    chr = Serial1.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }

  // Execute motor commands
  if (millis() > nextPID) {
    nextPID += PID_INTERVAL;
    updatePID();
  }

  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
    setMotorSpeeds(0, 0);
    moving = 0;
  }
}





