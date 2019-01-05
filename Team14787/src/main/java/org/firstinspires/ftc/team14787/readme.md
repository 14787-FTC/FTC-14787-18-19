## TeamCode Module

This module contains all OpModes and control/subsystem classes included in the final build of the robot.

# OpMode List
### Autonomous
##### Crater Auton
Single sample starting on the crater side of the field
##### Double Sample
Double sample starting on the crater side of the field
##### Motor Testing
Simple motor test container
##### Vision Testing
Simply turns on game object detection for debugging

### TeleOp
##### Mecanum Drive
Moves the robot according to user-controlled input via the joystick; The left stick controls forward/backward movement and strafing while the right stick control rotational movement.

# Non-OpMode Classes
##### RobotHardware
Represents the hardware of a robot, including motors and minimalized control methods.
##### PID Controller
Handles the majority of PID operations in all movement to improve accuracy.
##### Vision
Controls the phone camera and provides a simple game object recognition API to handle sampling using Vuforia and TensorFlow Object Detection.
##### AutonOpMode
Abstact representation of the beginning of an autonomous OpMode; all auton OpModes are children of this class.
