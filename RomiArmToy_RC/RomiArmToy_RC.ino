#include <Romi32U4.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <AlfredoCRSF.h>
#include <HardwareSerial.h>

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

#define I2C_DEV_ADDR 20

struct Data
{
  bool yellow, green, red;
  bool buttonA, buttonB, buttonC;

  int16_t leftMotor, rightMotor;
  uint16_t batteryMillivolts;
  float batteryVoltage;
  uint16_t analog[6];

  bool playNotes;
  char notes[14];

  int16_t leftEncoder, rightEncoder;
  uint16_t liftServo, tiltServo, gripperServo;
};

Data slave;
Romi32U4Motors motors;
Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;
Romi32U4ButtonC buttonC;
Romi32U4Encoders encoders;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
uint16_t gripValue = 600; //550 = open, 2350 = closed
uint16_t tiltValue = 1540; //1380 = most down, 1770 = most up
uint16_t liftValue = 1190; //800 = lowest, 1790 = highest
int lastGrip = 1500;
int forward, turn, vertical, gripRelease;
AlfredoCRSF crsf;

void processCSRF(void);

uint16_t last_lift, last_tilt, last_grip;

void setup()
{
  // Set up the slave at I2C address 20.
  Wire.begin();

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
  delay(10);
  //pwm.writeMicroseconds(0, liftValue);
  pwm.writeMicroseconds(1, tiltValue);
  //pwm.writeMicroseconds(2, gripValue);

  Serial1.begin(115200);

  crsf.begin(Serial1);
  
  slave.liftServo = 1190;
  last_lift = slave.liftServo;
  slave.tiltServo = 1540;
  last_tilt = slave.tiltServo;
  slave.gripperServo = gripValue;
}

void loop()
{
  // Call updateBuffer() before using the buffer, to get the latest
  // data including recent master writes.
  //slave.updateBuffer();
  crsf.update();
  // Change this to readBatteryMillivoltsLV() for the LV model.
  slave.batteryMillivolts = readBatteryMillivolts();
  slave.batteryVoltage = (float)slave.batteryMillivolts/1000;

  slave.analog[0] = analogRead(0);
  slave.analog[3] = analogRead(3);
  slave.analog[4] = analogRead(4);
  
  processCSRF();
  
  motors.setSpeeds(slave.leftMotor, slave.rightMotor); 
  
  slave.leftEncoder = encoders.getCountsLeft();
  slave.rightEncoder = encoders.getCountsRight();

  pwm.writeMicroseconds(0, slave.liftServo);
  pwm.writeMicroseconds(1, slave.tiltServo);
  pwm.writeMicroseconds(2, slave.gripperServo);
}

void processCSRF(void)
{
  /* Print RC channels every 100 ms. */
  unsigned long thisTime = millis();
  static unsigned long lastTime = millis();

  /* Compensate for millis() overflow. */
  if (thisTime < lastTime)
  {
      lastTime = thisTime;
  }

  if (thisTime - lastTime >= 50) //20Hz
  {
      lastTime = thisTime;
      
      if(crsf.isLinkUp()){
        forward = map(crsf.getChannel(2),988,2012, -100, 100);
        turn = map(crsf.getChannel(1),988,2012, -100, 100);
        vertical = map(crsf.getChannel(3),988,2012, -100, 100);
        gripRelease = crsf.getChannel(8);
        if(crsf.getChannel(9) > 1500){
        slave.gripperServo +=20;
        }
        if(gripRelease != lastGrip){
          lastGrip = gripRelease;
          slave.gripperServo = 550;
        }
        
        sendRxBattery(slave.batteryVoltage, 0, 1800, 100);
        Serial.println(crsf.getChannel(2));
      }else{
        forward = 0;
        turn = 0;
      }
      
      int lift = map(vertical, 100, -100, 900, 1620);
      int tilt = map(vertical, 100, -100, 1340, 1450);
      int rightwheel = forward-turn;
      int leftwheel = forward+turn;
      slave.leftMotor = leftwheel;
      slave.rightMotor = rightwheel;
      slave.tiltServo = tilt;
      slave.liftServo = lift;
      if(slave.gripperServo > 2350) slave.gripperServo = 2350;

  }
}

static void sendRxBattery(float voltage, float current, float capacity, float remaining)
{
  crsf_sensor_battery_t crsfBatt = { 0 };

  // Values are MSB first (BigEndian)
  crsfBatt.voltage = htobe16((uint16_t)(voltage * 10.0));   //Volts
  crsfBatt.current = htobe16((uint16_t)(current * 10.0));   //Amps
  crsfBatt.capacity = htobe16((uint16_t)(capacity));   //mAh (with this implemetation max capacity is 65535mAh)
  crsfBatt.remaining = (uint8_t)(remaining);                //percent
  crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_BATTERY_SENSOR, &crsfBatt, sizeof(crsfBatt));
}