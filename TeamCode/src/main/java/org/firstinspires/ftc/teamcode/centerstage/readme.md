- [**Robot Files**](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/readme.md#robot-files)
- **Autonomous**
- [**Util Files**](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/readme.md#util-files)

# Robot Files
# [BasicTestRobot.java](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/BasicTestRobot.java)

### [Robot Control](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/BasicTestRobot.java#L54)

The class enables control of the robot's movement using a gamepad. It supports the following actions:
- Forward and backward movement.
- Strafing left and right.
- Rotating or turning.

### Orientation Tracking

The robot's orientation can be tracked using onboard sensors. The class provides the ability to reset the robot's orientation to a specific angle.

### [AprilTag Integration](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/BasicTestRobot.java#L63)

This class integrates with an AprilTag detection system, allowing the robot to detect and track AprilTags in its environment. It provides telemetry data related to the detected AprilTag's:
- X, Y, and Z coordinates.
- Roll, pitch, and yaw angles.

## Usage

To use the `BasicTestRobot` class, follow these steps:

1. **Hardware Setup**: Ensure that your robot is correctly configured with the required hardware, including motor controllers, servos, and a webcam named "cam."

2. **Initialization**: In the [`runOpMode()`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/BasicTestRobot.java#L20) method, the robot's hardware components and vision system are initialized using the hardware map. Ensure that the hardware map correctly references your robot's components.

3. [**TeleOperation**](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/BasicTestRobot.java#L54): The primary teleOperation loop is where you control the robot using the gamepad. Key control features include:
   - Left stick for forward/backward and left/right motion.
   - Right stick for turning.
   - D-pad for fine adjustments in orientation.

4. [**Orientation Reset**](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/BasicTestRobot.java#L39): Pressing the "X" button on the gamepad resets the robot's orientation to 0 degrees, which can be useful for aligning the robot's reference heading.

5. [**AprilTag Integration**](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/BasicTestRobot.java#L63): If the robot encounters an AprilTag, telemetry data related to the detected tag's position and orientation will be displayed. This information can aid in navigation and object tracking.

# Util Files
- [`DriveClass.java`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/readme.md#driveclassjava)
- [`Toggle.java`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/readme.md#togglejava)
- [`Location.java`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/readme.md#locationjava)
# [DriveClass.java](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/DriveClass.java)
## Purpose
The primary purpose of this class is to control the movement and positioning of a robot on the field. It manages the robot's motors, sensors, and integration with an Inertial Measurement Unit (IMU) to perform precise movements and navigate the game field.

## Class Members

### Constants
- [`tile`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/DriveClass.java#L15): A constant representing the distance of one tile on the field (0.6 units).

### Flags
- [`USE_ENCODERS`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/DriveClass.java#L27), [`USE_BRAKE`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/DriveClass.java#L28), [`USE_DASHBOARD_FIELD`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/DriveClass.java#29): Constants used as flags for configuring the behavior of the robot, like using encoders, enabling braking, or displaying the robot's position on a dashboard.

### Member Variables
- Various private member variables to store information about motors, IMU, robot type, starting position, movement parameters, and more.

### Enums
- [`ROBOT`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/DriveClass.java#56): An enum representing different robot types (e.g., SCORPION, COBALT).
- [`DriveMode`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/DriveClass.java#71): An enum representing different drive modes (LEFT and RIGHT) with associated origin and direction vectors.

### [Consturctor](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/DriveClass.java#L90)
- The class has a constructor that initializes various parameters, including the robot type, starting position, and drive mode.

## Methods

### [`init(HardwareMap hw)`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/DriveClass.java#L149)
- Initializes motors, sets their directions, and configures their modes and behaviors based on the specified parameters.

### [`initIMU(HardwareMap hw)`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/DriveClass.java#L200)
- Initializes the IMU (Inertial Measurement Unit) sensor, calibrates it, and configures its integration with the robot's movement.

### [`setPower(double forward, double turn, double strafe)`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/DriveClass.java#L241)
- Sets the power levels for the robot's motors to control forward, turn, and strafe movements.

### [`setPowerOriented(double y, double x, double turn, boolean fieldOriented)`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/DriveClass.java#L248)
- Sets the power levels for the robot's motors while considering field orientation if enabled.

### [`stopPower()`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/DriveClass.java#L242)
- Stops all robot motors by setting their power levels to zero.

### [`resetOrientation(double angle)`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/DriveClass.java#L266)
- Resets the robot's orientation to a specified angle.

### [`getHeading()`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/DriveClass.java#L271)
- Retrieves the current heading (orientation) of the robot.

### [`getDeltaHeading(double target)`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/DriveClass.java#L276)
- Calculates the difference in heading between the robot's current orientation and a target orientation.

### [`getForwardDistance()`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/DriveClass.java#L289)
- Calculates and returns the distance the robot has traveled in the forward direction using encoder ticks.

### [`getPosX()`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/DriveClass.java#L330) and [`getPosY()`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/DriveClass.java#L301)
- Retrieve the current X and Y positions of the robot on the field.

### [`goToLocation(Location location, double power, double targetHeading, double tolerance, double timeout)`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/DriveClass.java#L395)
- Moves the robot to a specified location on the field using PID control.

### [`drive(double forward, double sideward, double targetPower, double targetAngle, boolean fieldOriented, double tolerance)`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/DriveClass.java#L563)
- Controls the robot's movement using PID control, taking into account forward, sideward, target power, target angle, field orientation, and tolerance.

### [`hoverBoardMode()`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/DriveClass.java#L670)
- Implements a "hoverboard" mode to correct the robot's orientation when it deviates from a target angle.
  
The `DriveClass` is a crucial component of a robotics program, providing control and navigation capabilities for a robot on the game field.
# [Toggle.java](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/Toggle.java)
## Constructor

- [`Toggle()`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/Toggle.java#L7): Default constructor with no initial state.
- [`Toggle(boolean initialState)`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/Toggle.java#L8): Constructor with an initial state parameter.

## Methods

### [`update(boolean input)`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/Toggle.java#L10)

This method updates the toggle based on the given input. It is designed for handling toggles where the state changes only when the input is pressed and not when released.

### [`set(boolean input)`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/Toggle.java#L21)

Sets the toggle state explicitly. It can be used to directly set the state without relying on input changes.

### [`toggle()`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/Toggle.java#27)

Toggles the current state of the toggle and returns the new state.

### [`isPressed()`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/Toggle.java#L33)

Checks if the toggle is currently pressed.

### [`isChanged()`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/Toggle.java#L37)

Checks if the toggle state has changed since the last update.

### [`getState()`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/Toggle.java#L41)

Returns the current state of the toggle.

### [`isClicked()`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/Toggle.java#L43)

Checks if the toggle has changed and is currently pressed (clicked).

### [`isReleased()`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/Toggle.java#L47)

Checks if the toggle has changed and is not currently pressed (released).

## Example Usage

```java
// Create a new Toggle instance with an initial state of false
Toggle toggle = new Toggle(false);

// Update the toggle based on input
toggle.update(someInput);

// Check the current state of the toggle
boolean currentState = toggle.getState();

// Toggle the state of the toggle
toggle.toggle();

// Check if the toggle is currently pressed
boolean isPressed = toggle.isPressed();
```
# [Location.java](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/Location.java)
## Constructors

- [`Location(double x, double y)`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/Location.java#L9): Initializes a location with the specified x and y coordinates.
- [`Location(double x, double y, double angle)`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/Location.java#L14): Initializes a location with the specified x and y coordinates and an angle.
- [`Location(Location location)`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/Location.java#L20): Creates a copy of an existing location.
- [`Location(Location location, double xOffset, double yOffset, double angleOffset)`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/Location.java#L26): Creates a new location based on an existing location with added offsets.

## Methods

### Coordinate Flipping

- [`flipX()`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/Location.java#L32): Inverts the x-coordinate.
- [`flipY()`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/Location.java#L36): Inverts the y-coordinate.
- [`flipAngle()`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/Location.java#L40): Inverts the angle.

### Offset Manipulation

- [`offsetY(double offset)`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/Location.java#L44): Returns a new location with an offset added to the y-coordinate.
- [`offsetX(double offset)`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/Location.java#L96): Returns a new location with an offset added to the x-coordinate.
- [`stayOnX(DriveClass drive)`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/Location.java#L100): Returns a new location with the x-coordinate set to the current x-position from a `DriveClass` instance.
- [`stayOnY(DriveClass drive)`](https://github.com/SciFighters/FtcRobotController/blob/dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage/util/Location.java#L104): Returns a new location with the y-coordinate set to the current y-position from a `DriveClass` instance.

### Arithmetic Operations

- `add(Location location)`: Adds the x and y coordinates of another location to the current location.
- `addX(double x)`: Adds a specified value to the x-coordinate.
- `addY(double y)`: Adds a specified value to the y-coordinate.
- `subtract(Location location)`: Subtracts the x and y coordinates of another location from the current location.
- `subtractX(double x)`: Subtracts a specified value from the x-coordinate.
- `subtractY(double y)`: Subtracts a specified value from the y-coordinate.
- `multiply(float num)`: Multiplies both the x and y coordinates by a specified value.
- `multiplyX(double x)`: Multiplies the x-coordinate by a specified value.
- `multiplyY(double y)`: Multiplies the y-coordinate by a specified value.

## Example Usage

```java
// Create a new Location instance
Location location = new Location(10.0, 20.0);

// Flip the x-coordinate
location.flipX();

// Add an offset to the y-coordinate
Location newLocation = location.offsetY(5.0);

// Perform arithmetic operations
Location otherLocation = new Location(5.0, 10.0);
location.add(otherLocation);
location.multiply(2.0);
```
