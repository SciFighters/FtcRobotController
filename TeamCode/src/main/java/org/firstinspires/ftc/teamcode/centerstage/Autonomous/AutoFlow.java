package org.firstinspires.ftc.teamcode.centerstage.Autonomous;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.Arm;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Camera.AprilTagDetector;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Camera.DuckLine;
import org.firstinspires.ftc.teamcode.centerstage.Systems.DriveClass;
import org.firstinspires.ftc.teamcode.centerstage.Systems.IntakeSystem;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Component;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Robot;
import org.firstinspires.ftc.teamcode.centerstage.util.Location;
import org.firstinspires.ftc.teamcode.centerstage.util.Util;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.MathUtil;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.lang.annotation.Documented;

/**
 * Component for autonomous flow control.
 */
public class AutoFlow extends Component {
    /**
     * Telemetry instance for the dashboard.
     */
    public static Telemetry dashboardTelemetry;
    /**
     * Telemetry instance for multiple telemetry sources.
     */
    public static MultipleTelemetry telemetry;
    final double robotLength = 0.4404;
    final double tile = 0.6;
    final int screenWidth = 640;
    final int screenHeight = 360;
    public PropPos propPos = PropPos.NONE;
    public DuckLine duckLine;
    public Auto auto;
    public DriveClass drive;
    Robot robot;
    Location startLocation = new Location(1, robotLength / 2, 180); // PIXEL_STACK
    AprilTagDetector aprilTagDetector;
    FtcDashboard dashboard;
    IntakeSystem intakeSystem;
    Arm arm;
    ElapsedTime timer;
    OpenCvWebcam webcam;
    DriveClass.GotoSettings normalDriveSettings = new DriveClass.GotoSettings.Builder().setPower(1).setTolerance(0.05).setSlowdownMode(false).setTimeout(0).build();
    DriveClass.GotoSettings lowToleranceSettings = new DriveClass.GotoSettings.Builder().setPower(1).setTolerance(0.16).setSlowdownMode(true).setTimeout(0).build();
    AutoPath path;
    StartPos startPos;
    Location backdropLocation;
    ParkLocation parkLocation;
    int goToTries = 0;

    /**
     * Constructs an AutoFlow object.
     *
     * @param robot        The Robot object.
     * @param alliance     The Alliance color.
     * @param startPos     The starting position.
     * @param auto         The autonomous mode.
     * @param parkLocation The parking location.
     */
    public AutoFlow(Robot robot, Alliance alliance, StartPos startPos, Auto auto, ParkLocation parkLocation) {
        this.robot = robot;
        this.startPos = startPos;
        this.parkLocation = parkLocation;
        if (alliance == Alliance.RED) {
            startLocation = new Location(startPos == StartPos.PIXEL_STACK ? 1 : -0.3, (robotLength / 2) + (tile * 2), 0);
        } else if (alliance == Alliance.BLUE) {
            startLocation = new Location(startPos == StartPos.PIXEL_STACK ? 1 : -0.3, (-robotLength / 2) - (tile * 2), 180);
        }
        this.auto = auto;
        robot.alliance = alliance;
    }

    /**
     * Gets the tag ID according to the team's prop location.
     *
     * @param pos      The prop position.
     * @param alliance The Alliance color.
     * @return The tag ID.
     */
    public static int getTagIDAccordingToTeamPropLocation(PropPos pos, Alliance alliance) {
        if (alliance == Alliance.RED) {
            if (pos == PropPos.LEFT) {
                pos = PropPos.RIGHT;
            } else if (pos == PropPos.RIGHT) {
                pos = PropPos.LEFT;
            }
        }
        int id = 2;
        switch (pos) {
            case LEFT:
                id = 1;
                break;
            case RIGHT:
                id = 3;
                break;
        }
        if (alliance == Alliance.RED) {
            id += 3;
        }
        return id;
    }

    /**
     * Initializes the webcam for prop detection.
     */
    void initWebcam() {
        int cameraMonitorViewID = robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());

        WebcamName webcamName = robot.hardwareMap.get(WebcamName.class, "cam");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewID);

        this.duckLine = new DuckLine(this.robot, telemetry);
        webcam.setPipeline(this.duckLine);
        webcam.openCameraDevice();
        webcam.startStreaming(screenWidth, screenHeight, OpenCvCameraRotation.UPRIGHT);
    }

    /**
     * Starts AprilTag detection.
     */
    public void startAprilTagDetection() {
        aprilTagDetector = new AprilTagDetector("cam", new Size(screenWidth, screenHeight), robot.hardwareMap, robot.telemetry, AprilTagDetector.PortalConfiguration.DEFAULT);
    }

    /**
     * Initializes components and telemetry.
     */
    @Override
    public void init() {
        timer = new ElapsedTime();
        arm = robot.addComponent(Arm.class);
        arm.telemetry = telemetry;
        this.drive = robot.addComponent(DriveClass.class, new DriveClass(DriveClass.ROBOT.GLADOS, startLocation, DriveClass.USE_ENCODERS | DriveClass.USE_BRAKE | DriveClass.USE_DASHBOARD_FIELD, DriveClass.DriveMode.LEFT));
        intakeSystem = robot.addComponent(IntakeSystem.class);
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        telemetry = new MultipleTelemetry(dashboardTelemetry, robot.telemetry);
        initWebcam();
        drive.resetOrientation(startLocation.angle);
        dashboardTelemetry.update();
        backdropLocation = new Location(-tile * 2 + robotLength / 2 + 0.15, -tile - 0.15, 90);
    }

    /**
     * Builds the autonomous path.
     *
     * @return The constructed AutoPath.
     */
    public AutoPath buildPath() {
        AutoPath path = null;
        if (robot.alliance == Alliance.RED) {
            if (path != null) path.flipY();
            backdropLocation.flipY();
        }
        return path;
    }

    /**
     * Locks onto a specific tag by its ID.
     *
     * @param tagID The ID of the tag to lock onto.
     */
    @Deprecated
    public void lockOnTag(int tagID) {
        drive.turnTo(90, 1);
        double yOffset = 0.08;
        if (tagID == 2 || tagID == 5) {
            yOffset = 0;
        } else if (tagID == 1 || tagID == 4) {
            yOffset *= -1;
        }
        drive.goToLocation(new Location(backdropLocation.x, backdropLocation.y + yOffset, 90), normalDriveSettings);
        arm.alignToBoard(Arm.Position.One); // fixes distance to the board on the X axis
        AprilTagDetection tag = aprilTagDetector.getSpecificTag(tagID);
        double targetHeading = robot.alliance == AutoFlow.Alliance.BLUE ? 90 : -90;
        double leftDist = drive.getDistanceLeftSensorDistance(), rightDist = drive.getDistanceRightSensorDistance();
        double distance = Math.min(leftDist, rightDist);
        int distanceSensorIndex = leftDist == distance ? 1 : 2;
        ElapsedTime timeout = new ElapsedTime();
        robot.telemetry.clearAll();
        while (timeout.seconds() < 3) {
            tag = aprilTagDetector.getSpecificTag(tagID);
            if (tag == null) {
                drive.stopPower();
                continue;
            } else if (MathUtil.approximately(tag.rawPose.x, 0, 1)) {
                break;
            }
            double gainY = -0.016;

            double deltaAngle = drive.getDeltaHeading(targetHeading);
            double deltaY = tag.rawPose.x;

            double powerY = deltaY * gainY;

            telemetry.addData("DISTANCE X", tag.rawPose.x);
            telemetry.addData("DISTANCE Y", tag.rawPose.y);
            telemetry.addData("left right DISTANCE", MathUtil.approximately(tag.rawPose.x, 0, 1));
            telemetry.addData("backdrop DISTANCE", MathUtil.approximately(distance, Arm.Position.One.distanceFromBackdrop, 0.5));
            telemetry.update();
            drive.setPower(0, 0, powerY);
        }
        drive.setPower(0, 0, 0);
    }

    /**
     * Locks onto a tag based on the position of the team's prop.
     *
     * @param pos The position of the prop.
     */
    @Deprecated
    public void lockOnTagByProp(PropPos pos) {
        lockOnTag(getTagIDAccordingToTeamPropLocation(pos, robot.alliance));
    }

    /**
     * Camera is flipped automatically in order to see the prop correctly in different alliance
     * Note: The flipping (Left => Right) is only for the Red Alliance,
     * flipped again when putting the pixel on the backboard
     *
     * @param pos The prop position
     */
    public void placePurplePixelByProp(PropPos pos) {
        double yOffset = tile * Math.signum(startLocation.y) + 0.43 * Math.signum(startLocation.y);
        Location mid = new Location(startLocation.x - 0.25, startLocation.y - yOffset, 90);
        Location right = new Location(startLocation.x + 0.12, -tile, 90);
        Location left = new Location(startLocation.x - tile + 0.15, -tile, 90);
        if (startPos == StartPos.PIXEL_STACK) {
            left = new Location(startLocation.x - 0.08, -tile, startLocation.angle);
            right = new Location(startLocation.x + 0.2, -tile + 0.2, startLocation.angle);
            yOffset = tile * Math.signum(startLocation.y) + 0.6 * Math.signum(startLocation.y);
            mid = new Location(startLocation.x, startLocation.y - yOffset, startLocation.angle);
        }
        if (robot.alliance == Alliance.RED) {
            left.flipY();
            right.flipY();
        }
        if (pos == PropPos.MID || pos == PropPos.NONE) {
            drive.goToLocation(mid, normalDriveSettings);
        } else if (pos == PropPos.LEFT) {
            drive.goToLocation(left, normalDriveSettings);
            double angle = 90;
            if (startPos == StartPos.PIXEL_STACK) {
                angle *= -1;
            }
            drive.turnTo(angle, 1);
        } else if (pos == PropPos.RIGHT) {
            drive.goToLocation(right, normalDriveSettings);
            if (startPos == StartPos.BACKSTAGE) drive.turnTo(90, 1);
        }
        intakeSystem.spit();
        timer.reset();
        while (timer.seconds() < 0.75) {
            intakeSystem.update();
        }
        intakeSystem.stopIntake();
        intakeSystem.update();
        if (startPos == StartPos.PIXEL_STACK) {
            if (robot.alliance == Alliance.BLUE) {
                if (propPos == PropPos.RIGHT) {
                    drive.goToLocation(new Location(drive.getPosX(), drive.getPosY() + 0.3, drive.getHeading()), normalDriveSettings);
                } else if (propPos == PropPos.LEFT) {
                    drive.goToLocation(new Location(drive.getPosX() + 0.15, drive.getPosY(), drive.getHeading()), normalDriveSettings);
                }
            } else {
                if (propPos == PropPos.RIGHT) {
                    drive.goToLocation(new Location(drive.getPosX() + 0.15, drive.getPosY(), drive.getHeading()), normalDriveSettings);
                }
            }
        }
        if (startPos == StartPos.PIXEL_STACK) {
            Location location = new Location(drive.getPosX(), -0.1, drive.getHeading());
            if (robot.alliance == Alliance.RED) {
                location.flipY();
            }
            drive.goToLocation(location, normalDriveSettings);
            drive.turnTo(90, 1);

            Location pixelStack = new Location(tile * 3 - 0.26, -0.11, 90);
            if (robot.alliance == Alliance.RED) {
                pixelStack = pixelStack.flipY();
            }
            arm.openClaw(false);
            drive.goToLocation(pixelStack, normalDriveSettings);
            intakeSystem.collect();
            Thread intakeThread = Util.loopAsync(() -> {
                intakeSystem.update();
            }, robot);
            Location finalPixelStack = pixelStack;
            drive.goToLocation(pixelStack.subtractX(0.15), normalDriveSettings, null, () -> {
                drive.currentTarget = finalPixelStack;
            });
            robot.sleep(4000);
            drive.goToLocation(new Location(backdropLocation.x, 0, 90), lowToleranceSettings, () -> {
                if (drive.getPosX() <= tile * 3) {
                    arm.openClaw(true);
                    intakeSystem.spit();
                }
                if (drive.getPosX() < -0.5) {
                    arm.goToPos(2300);
                    intakeSystem.stopIntake();
                }
                if (drive.getPosX() < backdropLocation.x + 0.48) {
                    drive.currentTarget = backdropLocation;
                }
            }, () -> {
                drive.currentTarget = backdropLocation;
            });
            intakeThread.interrupt();
        }
    }

    /**
     * Executes the cycling routine.
     */
    public void cycle() {
        intakeSystem.collect();
        Thread thread = Util.loopAsync(intakeSystem::update, robot);
        drive.goToLocation(new Location(-tile * 1.5, drive.getPosY(), 90), lowToleranceSettings);
        drive.goToLocation(new Location(-tile * 1.5, 0, 90), lowToleranceSettings);
        drive.goToLocation(new Location(tile * 3 - 0.23, -0.15, 90), normalDriveSettings);
        arm.openClaw(false);
        robot.sleep(500);
        drive.goToLocation(new Location(-tile * 1.5, 0, 90), lowToleranceSettings, () -> {
            if (drive.getPosX() <= tile * 1.5 && intakeSystem.state() != IntakeSystem.State.Spit) {
                arm.openClaw(true);
                intakeSystem.spit();
            }
            if (drive.getPosX() <= -tile) {
                arm.goToPos(2500);
            }
        });
        intakeSystem.stopIntake();
        int goToTries = 0;
        goToBoardByProp(propPos, Arm.Position.Three);
        arm.goToPos(Arm.Position.Three); // makes the arm go yee
        robot.sleep(1300);
        arm.openClaw(false);
        robot.sleep(500);
        arm.goToPos(Arm.Position.Home);
    }

    /**
     * Runs the predefined path.
     */
    public void runPath() {
        intakeSystem.setServoPos(IntakeSystem.State.Idle);
        path = buildPath();
        propPos = DuckLine.getPropPos();
        stopPropDetection();
        if (auto != Auto.PARK && startPos == StartPos.BACKSTAGE) {
            arm.goToPos(2300);
        }
        int sleepTime = 2000;
        startAprilTagDetection();
        placePurplePixelByProp(propPos);
        if (auto != Auto.PARK) {
            if (path != null) path.run(); // moves to the backboard
            if (startPos == StartPos.PIXEL_STACK) {
                sleepTime = 700;
                PropPos posForWhitePixel = PropPos.MID;
                if (propPos == PropPos.MID) {
                    posForWhitePixel = PropPos.RIGHT;
                }
                goToBoardByProp(posForWhitePixel, Arm.Position.One);
                arm.goToPos(Arm.Position.One); // makes the arm go yee
                robot.sleep(2000);
                arm.dropBottomPixel();
                robot.sleep(500);
                arm.goToPos(Arm.Position.One.liftPos - 400);
            }
            goToBoardByProp(propPos, Arm.Position.One);
            arm.goToPos(Arm.Position.One); // makes the arm go yee
            robot.sleep(sleepTime);
            arm.openClaw(false);
            aprilTagDetector.stop();
            robot.sleep(700);
            arm.goToPos(Arm.Position.Home);
//            robot.sleep(2000);
            if (auto == Auto.CYCLING) {
                cycle();
            }
        }
        park();
        if (robot.time < 29) {
            robot.sleep(2000);
        }
        robot.requestOpModeStop(); // stops the program
    }

    /**
     * Stops all operations and components.
     */
    @Override
    public void stop() {
        stopPropDetection();
        aprilTagDetector.stop();
    }

    /**
     * Parks the robot in the specified location.
     */
    public void park() {
        double y = startLocation.y + tile * 2 * Math.signum(-startLocation.y) + 0.12 * Math.signum(-startLocation.y);
        double x = -tile * 2;
        if (parkLocation == ParkLocation.NEAR) {
            y = startLocation.y;
        }
        drive.goToLocation(new Location(drive.getPosX(), y, 90), lowToleranceSettings);
        drive.goToLocation(new Location(x, y, 90), lowToleranceSettings);
        drive.turnTo(90, 1);
    }

    /**
     * Stops prop detection and closes the webcam.
     */
    public void stopPropDetection() {
        webcam.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
            @Override
            public void onClose() {
                webcam.closeCameraDevice();

            }
        }); // closes the prop detection webcam
    }

    /**
     * Navigates the robot to the backboard based on the position of the team's prop.
     *
     * @param propPos The position of the prop.
     * @param armPos  The position of the arm.
     */
    public void goToBoardByProp(PropPos propPos, Arm.Position armPos) {
        DriveClass.GotoSettings setting = new DriveClass.GotoSettings.Builder().setPower(0.7).setTolerance(0.003).setSlowdownMode(false).setTimeout(0).build();
        if (goToTries > 5) {
            return;
        }
        goToTries++;
        double distanceFromBoardX = Math.min(drive.getDistanceLeftSensorDistance(), drive.getDistanceRightSensorDistance());
        distanceFromBoardX /= 100; // Convert from CM to METERS
        double targetDistanceFromBoardX = armPos.distanceFromBackdrop / 100; // Convert from CM to METERS
        double yOffset = 0.08;
        if (propPos == PropPos.LEFT && robot.alliance == Alliance.BLUE) {
            yOffset *= -1;
        } else if (propPos == PropPos.RIGHT && robot.alliance == Alliance.RED) {
            yOffset *= -1;
        }
        if (robot.alliance == Alliance.RED) {
            yOffset *= -1;
        }
        if (distanceFromBoardX > 0.3 || propPos == PropPos.MID || propPos == PropPos.NONE || startPos == StartPos.PIXEL_STACK) {
            drive.goToLocation(new Location(backdropLocation.x, backdropLocation.y + yOffset, 90), lowToleranceSettings);
            distanceFromBoardX = Math.min(drive.getDistanceLeftSensorDistance(), drive.getDistanceRightSensorDistance());
            distanceFromBoardX /= 100; // Convert from CM to METERS
        }
        AprilTagDetection tagDetection = aprilTagDetector.getSpecificTag(getTagIDAccordingToTeamPropLocation(propPos, robot.alliance));
        double tagDistanceY = 0;

        if (tagDetection != null) {
            tagDistanceY = tagDetection.rawPose.x / 100; // Convert from CM to METERS
        }

        double targetX = drive.getPosX() + (targetDistanceFromBoardX - distanceFromBoardX);
        double targetY = drive.getPosY() + tagDistanceY;

//        drive.goToLocation(new Location(targetX, targetY, 90), lowToleranceSettings);
        drive.goToLocation(new Location(targetX, targetY, 90), setting);

        if (tagDetection == null) {
            goToBoardByProp(propPos, armPos);
        }
    }

    public enum StartPos {
        PIXEL_STACK(1), BACKSTAGE(-1);

        int mul;

        StartPos(int mul) {
            this.mul = mul;
        }
    }

    public enum Alliance {
        BLUE, RED
    }

    public enum Auto {
        PARK(1, true), SHORT(2, true), LONG(3, true), FULL(4, true), CYCLING(5, true);

        public int value;
        public boolean _isParking;

        Auto(int value, boolean isParking) {
            this.value = value;
            this._isParking = isParking;
        }
    }

    public enum PropPos {
        RIGHT, MID, LEFT, NONE
    }

    public enum ParkLocation {
        FAR, NEAR
    }
}
