#include <Servo.h>
#include <Romi32U4.h>
#include <PololuRPiSlave.h>
#include <Servo.h>

/* This example program shows how to make the Romi 32U4 Control Board 
 * into a Raspberry Pi I2C slave.  The RPi and Romi 32U4 Control Board can
 * exchange data bidirectionally, allowing each device to do what it
 * does best: high-level programming can be handled in a language such
 * as Python on the RPi, while the Romi 32U4 Control Board takes charge 
 * of motor control, analog inputs, and other low-level I/O.
 *
 * The example and libraries are available for download at:
 *
 * https://github.com/pololu/pololu-rpi-slave-arduino-library
 *
 * You will need the corresponding Raspberry Pi code, which is
 * available in that repository under the pi/ subfolder.  The Pi code
 * sets up a simple Python-based web application as a control panel
 * for your Raspberry Pi robot.
 */

// Custom data structure that we will use for interpreting the buffer.
// We recommend keeping this under 64 bytes total.  If you change the
// data format, make sure to update the corresponding code in
// a_star.py on the Raspberry Pi.

struct Data
{
  bool yellow, green, red;
  bool buttonA, buttonB, buttonC;

  int16_t leftMotor, rightMotor;
  uint16_t batteryMillivolts;
  uint16_t analog[6];

  bool playNotes;
  char notes[14];

  int16_t leftEncoder, rightEncoder;
  uint16_t liftServo, tiltServo, gripperServo;
};

PololuRPiSlave<struct Data,5> slave;
Romi32U4Motors motors;
Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;
Romi32U4ButtonC buttonC;
Romi32U4Encoders encoders;
Servo lServo;
Servo tServo;
Servo gServo;

uint16_t last_lift, last_tilt, last_grip;

void setup()
{
  // Set up the slave at I2C address 20.
  slave.init(20);
  lServo.attach(20);
  tServo.attach(4);
  gServo.attach(11);

  lServo.writeMicroseconds(1500);//800-1900
  tServo.writeMicroseconds(1500);//1400-1770
  gServo.writeMicroseconds(1000);//550-2370  

  
  slave.buffer.liftServo = 1500;
  last_lift = slave.buffer.liftServo;
  slave.buffer.tiltServo = 1500;
  last_tilt = slave.buffer.tiltServo;
  slave.buffer.gripperServo = 1000;
  last_grip = slave.buffer.gripperServo;
  slave.finalizeWrites();
}

void loop()
{
  // Call updateBuffer() before using the buffer, to get the latest
  // data including recent master writes.
  slave.updateBuffer();

  // Write various values into the data structure.
  slave.buffer.buttonA = buttonA.isPressed();
  slave.buffer.buttonB = buttonB.isPressed();
  slave.buffer.buttonC = buttonC.isPressed();

  // Change this to readBatteryMillivoltsLV() for the LV model.
  slave.buffer.batteryMillivolts = readBatteryMillivolts();

  slave.buffer.analog[0] = analogRead(0);
  slave.buffer.analog[3] = analogRead(3);
  slave.buffer.analog[4] = analogRead(4);
  

  // READING the buffer is allowed before or after finalizeWrites().
  ledYellow(slave.buffer.yellow);
  ledGreen(slave.buffer.green);
  ledRed(slave.buffer.red);
  motors.setSpeeds(slave.buffer.leftMotor, slave.buffer.rightMotor);
  
  if (last_lift != slave.buffer.liftServo){
    lServo.writeMicroseconds(slave.buffer.liftServo);
    last_lift = slave.buffer.liftServo;
  }
  if (last_tilt != slave.buffer.tiltServo){
    tServo.writeMicroseconds(slave.buffer.tiltServo);
    last_tilt = slave.buffer.tiltServo;
  }
  if (last_grip != slave.buffer.gripperServo){
    gServo.writeMicroseconds(slave.buffer.gripperServo);
    last_grip = slave.buffer.gripperServo;
  }
  
  

  slave.buffer.leftEncoder = encoders.getCountsLeft();
  slave.buffer.rightEncoder = encoders.getCountsRight();

  // When you are done WRITING, call finalizeWrites() to make modified
  // data available to I2C master.
  slave.finalizeWrites();
}
