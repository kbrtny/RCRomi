# RCRomi
Romi from Pololu with their arm kit.

RomiArmToy_RC has additional hardware beyond the Romi base and arm.  The 3 servos are controlled with a PCA9685 Driver board and a 5v Buck regulator from battery voltage.
I'm using an EP2 ELRS receiver with the baud rate set at 115200 (Done through receiver webpage config) and a Radiomaster Pocket Controller.  
It is configured for the right stick to drive the base (stick to motor math needs to be fixed), the left Y axis (throttle) controlls the arm to go up and down (tilt is automatic to keep the gripper levelish).  Gripper closes by holding the SE button (left shoulder momentary), and toggling SD (right sholder toggle) to fully open it.
I report the battery voltage over the battery telemetry channel.  I'm using 6 Tenergy NIMH cells to power the robot.
You will also need the AlfredoCSRF, Adafruit PWM Servo Driver, and Romi 32U4 libraries.