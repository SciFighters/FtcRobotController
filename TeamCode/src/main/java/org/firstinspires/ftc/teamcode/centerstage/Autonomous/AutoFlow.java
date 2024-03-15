package org.firstinspires.ftc.teamcode.centerstage.Autonomous;

import android.icu.text.RelativeDateTimeFormatter;
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
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class AutoFlow extends Component {
    public static Telemetry dashboardTelemetry;
    public static MultipleTelemetry telemetry;
    final double robotLength = 0.4404;
    final double tile = 0.6;
    final int screenWidth = 640;
    final int screenHeight = 360;
    public PropPos propPos = PropPos.NONE;
    public DuckLine duckLine;
    public Auto auto;
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

    private DriveClass drive;

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

    void initWebcam() {
        int cameraMonitorViewID = robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());

        WebcamName webcamName = robot.hardwareMap.get(WebcamName.class, "cam");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewID);

        this.duckLine = new DuckLine(this.robot.alliance, telemetry);
        webcam.setPipeline(this.duckLine);
        webcam.openCameraDevice();
        webcam.startStreaming(screenWidth, screenHeight, OpenCvCameraRotation.UPRIGHT);
    }

    void startAprilTagDetection() {
        aprilTagDetector = new AprilTagDetector("cam", new Size(screenWidth, screenHeight), robot.hardwareMap, robot.telemetry, AprilTagDetector.PortalConfiguration.DEFAULT);
    }

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
        intakeSystem.setServoPos(IntakeSystem.State.Idle);
//        aprilTagDetector = new AprilTagDetector("cam", new Size(800, 448), robot.hardwareMap, telemetry, new AprilTagDetector.PortalConfiguration());
    }

    public AutoPath buildPath() {
        AutoPath path = null;
//        backdropLocation = new Location(-tile * 2 + 0.3, startLocation.y + ((tile + 0.05) * Math.signum(-startLocation.y)), startLocation.angle);
//        backdropLocation = new Location(-tile * 2 + 0.3, -tile - Math.abs(startLocation.y), startLocation.angle);
        backdropLocation = new Location(-tile * 2 + robotLength / 2 + 0.15, -tile - 0.2, 90);
        if (startPos == StartPos.PIXEL_STACK) {
            path = new AutoPath.Builder().addStep(new Location(0, tile * 2 + 0.2, drive.getHeading()), lowToleranceSettings).addStep(new Location(-tile * 3 - 0.2, 0, 90), lowToleranceSettings, () -> {
                if (drive.getPosX() < -0.5) {
                    arm.goToPos(2300);
                }
            }).addLocation(new Location(backdropLocation.x, backdropLocation.y, 90), lowToleranceSettings).addStep(new Location(0, 0, 90), lowToleranceSettings).build(robot, startLocation);
        } else {
            if (propPos == PropPos.LEFT || propPos == PropPos.RIGHT) {
//                path = new AutoPath.Builder().addStep(new Location(0, 0, 90), normalDriveSettings)
//                        .addLocation(new Location(backdropLocation.x, backdropLocation.y, 90), normalDriveSettings)
//                        .build(robot, startLocation);
            } else
                path = new AutoPath.Builder().addLocation(new Location(drive.getPosX(), drive.getPosY(), drive.getHeading()), lowToleranceSettings).addStep(new Location(-tile / 2, 0, 90), lowToleranceSettings).build(robot, startLocation);
        }
        if (robot.alliance == Alliance.RED) {
            if (path != null) path.flipY();
            backdropLocation.flipY();
        }
        return path;
    }

    public void lockOnTag(int tagID) {
        drive.turnTo(90, 1);
        double yOffset = 0.08;
        if (tagID == 2 || tagID == 5) {
            yOffset = 0;
        } else if (tagID == 1 || tagID == 4) {
            yOffset *= -1;
        }
//        if (robot.alliance == Alliance.RED) {
//            yOffset *= -1;
//        }
        drive.goToLocation(new Location(backdropLocation.x, backdropLocation.y + yOffset, 90), normalDriveSettings);
        arm.alignToBoard(Arm.Position.One); // fixes distance to the board on the X axis
        AprilTagDetection tag = aprilTagDetector.getSpecificTag(tagID);
        double targetHeading = robot.alliance == AutoFlow.Alliance.BLUE ? 90 : -90;
        double leftDist = drive.getDistanceLeftSensorDistance(), rightDist = drive.getDistanceRightSensorDistance();
        double distance = Math.min(leftDist, rightDist);
        int distanceSensorIndex = leftDist == distance ? 1 : 2;
        ElapsedTime timeout = new ElapsedTime();
        robot.telemetry.clearAll();
//        assert tag != null;
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
//        drive.turnTo(targetHeading, 1);
    }

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
        double yOffset = tile * Math.signum(startLocation.y) + 0.4 * Math.signum(startLocation.y);
        Location mid = new Location(startLocation.x - 0.3, startLocation.y - yOffset, 90);
        Location right = new Location(startLocation.x + 0.12, -tile, 90);
        Location left = new Location(startLocation.x - tile + 0.15, -tile, 90);
        if (startPos == StartPos.PIXEL_STACK) {
            left = new Location(startLocation.x - 0.08, -tile, startLocation.angle);
            right = new Location(startLocation.x + 0.2, -tile + 0.2, startLocation.angle);
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
//        drive.turnTo(angle, 1);
        intakeSystem.spit();
        timer.reset();
        while (timer.seconds() < 0.75) {
            intakeSystem.update();
        }
        intakeSystem.stopIntake();
        intakeSystem.update();
//        if (propPosX == PropPos.MID) {
//            drive.goToLocation(new Location(drive.getPosX(), drive.getPosY() + 0.15 * Math.signum(-startLocation.y), drive.getHeading()), normalDriveSettings);
//        }
        if (startPos == StartPos.PIXEL_STACK && propPos == PropPos.RIGHT && robot.alliance == Alliance.BLUE) {
            drive.goToLocation(new Location(drive.getPosX(), drive.getPosY() + 0.3, drive.getHeading()), normalDriveSettings);
        }
        //        if (pos == PropPos.LEFT && robot.alliance == Alliance.BLUE) {
//            drive.goToLocation(left.add(new Location(0, tile)), lowToleranceSettings);
//        }
//        drive.turnTo(90, 0.5);

        if (startPos == StartPos.PIXEL_STACK) {
            Location location = new Location(drive.getPosX(), -0.15, drive.getHeading());
            if (robot.alliance == Alliance.RED) {
                location.flipY();
            }
            drive.goToLocation(location, normalDriveSettings);
            drive.turnTo(90, 1);
            robot.sleep(3000);
        }
    }

    public void cycle() {
        drive.goToLocation(new Location(-tile * 1.5, drive.getPosY(), 90), lowToleranceSettings);
        drive.goToLocation(new Location(-tile * 1.5, 0, 90), lowToleranceSettings);
        drive.goToLocation(new Location(tile * 3 - 0.23, -0.15, 90), normalDriveSettings);
        intakeSystem.collect();
        arm.openClaw(false);
        robot.sleep(300);
        Thread thread = Util.loopAsync(intakeSystem::update, robot);
        robot.sleep(1000);
        //        robot.sleep(4000);
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
        goToBoardByProp(propPos, Arm.Position.Three);
        arm.goToPos(Arm.Position.One); // makes the arm go yee
        robot.sleep(1300);
        arm.openClaw(false);
        robot.sleep(500);
        arm.goToPos(Arm.Position.Home);
    }

    public void runPath() {
        propPos = DuckLine.getPropPos();
        closeWebcam();
        if (auto != Auto.PARK && startPos == StartPos.BACKSTAGE) {
            arm.goToPos(2300);
        }
        startAprilTagDetection();
        placePurplePixelByProp(propPos);
        if (auto != Auto.PARK) {
            path = buildPath();
            if (path != null) path.run(); // moves to the backboard
//            lockOnTagByProp(propPos); // aligns on the Y axis
            goToBoardByProp(propPos, Arm.Position.One);
            aprilTagDetector.stop();
            arm.goToPos(Arm.Position.One); // makes the arm go yee
            robot.sleep(2000);
            arm.openClaw(false);
            robot.sleep(500);
            arm.goToPos(Arm.Position.Home);
//            robot.sleep(2000);
            if (auto == Auto.CYCLING) {
                cycle();
            }
        }
        park();
        robot.requestOpModeStop(); // stops the program
    }

    @Override
    public void stop() {
        closeWebcam();
        aprilTagDetector.stop();
    }

    public void park() {
        double y = startLocation.y + tile * 2 * Math.signum(-startLocation.y) + 0.1 * Math.signum(-startLocation.y);
        double x = -tile * 2.3 - 0.1;
        if (parkLocation == ParkLocation.RIGHT) {
            y = startLocation.y;
        }
        drive.goToLocation(new Location(drive.getPosX(), y, 90), lowToleranceSettings);
        drive.goToLocation(new Location(x, y, 90), lowToleranceSettings);
        drive.turnTo(90, 1);
    }

    public void closeWebcam() {
        webcam.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
            @Override
            public void onClose() {
                webcam.closeCameraDevice();

            }
        }); // closes the prop detection webcam
    }

    public void goToBoardByProp(PropPos propPos, Arm.Position armPos) {
        double distanceFromBoardX = Math.min(drive.getDistanceLeftSensorDistance(),
                drive.getDistanceRightSensorDistance());
        distanceFromBoardX /= 100; // Convert from CM to METERS
        double targetDistanceFromBoardX = armPos.distanceFromBackdrop / 100; // Convert from CM to METERS

        if (distanceFromBoardX > 0.3 || propPos == PropPos.MID || propPos == PropPos.NONE) {
            drive.goToLocation(new Location(backdropLocation.x, backdropLocation.y, 90), lowToleranceSettings);
            distanceFromBoardX = Math.min(drive.getDistanceLeftSensorDistance(),
                    drive.getDistanceRightSensorDistance());
            distanceFromBoardX /= 100; // Convert from CM to METERS
        }
        AprilTagDetection tagDetection = aprilTagDetector
                .getSpecificTag(getTagIDAccordingToTeamPropLocation(propPos, robot.alliance));
        double tagDistanceY = 0;
        if (tagDetection != null) {
            tagDistanceY = tagDetection.rawPose.x / 100; // Convert from CM to METERS
        }

        double targetX = drive.getPosX() + (targetDistanceFromBoardX - distanceFromBoardX);
        double targetY = drive.getPosY() + tagDistanceY;

        drive.goToLocation(new Location(targetX, targetY, 90), lowToleranceSettings);
        drive.goToLocation(new Location(targetX, targetY, 90), normalDriveSettings);
        if (tagDetection == null) {
            goToBoardByProp(propPos, armPos);
        }
    }

    public void goToBoard() {
        drive.goToLocation(backdropLocation, lowToleranceSettings);
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
        RIGHT, // RIGHT
        MID, // MID
        LEFT  // LEFT
        , NONE
    }

    public enum ParkLocation {
        LEFT, RIGHT
    }
}
