# [BasicTestRobot.java on GitHub](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/BasicTestRobot.java)

### Robot Control

The class enables control of the robot's movement using a gamepad. It supports the following actions:
- Forward and backward movement.
- Strafing left and right.
- Rotating or turning.

### Orientation Tracking

The robot's orientation can be tracked using onboard sensors. The class provides the ability to reset the robot's orientation to a specific angle.

### AprilTag Integration

This class integrates with an AprilTag detection system, allowing the robot to detect and track AprilTags in its environment. It provides telemetry data related to the detected AprilTag's:
- X, Y, and Z coordinates.
- Roll, pitch, and yaw angles.

## Usage

To effectively use the `BasicTestRobot` class, follow these steps:

1. **Hardware Setup**: Ensure that your robot is correctly configured with the required hardware, including motor controllers, servos, and a webcam named "cam."

2. **Initialization**: In the [`runOpMode()`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/BasicTestRobot.java#L20) method, the robot's hardware components and vision system are initialized using the hardware map. Ensure that the hardware map correctly references your robot's components.

3. **TeleOperation**: The primary teleOperation loop is where you control the robot using the gamepad. Key control features include:
   - Left stick for forward/backward and left/right motion.
   - Right stick for turning.
   - D-pad for fine adjustments in orientation.

4. [**Orientation Reset**](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/BasicTestRobot.java#L39): Pressing the "X" button on the gamepad resets the robot's orientation to 0 degrees, which can be useful for aligning the robot's reference heading.

5. **AprilTag Integration**: If the robot encounters an AprilTag, telemetry data related to the detected tag's position and orientation will be displayed. This information can aid in navigation and object tracking.
