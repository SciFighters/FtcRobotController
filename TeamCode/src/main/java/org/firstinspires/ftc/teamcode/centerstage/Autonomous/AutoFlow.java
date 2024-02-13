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
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Robot;
import org.firstinspires.ftc.teamcode.centerstage.util.Location;
import org.firstinspires.ftc.teamcode.freight_frenzy.util.MathUtil;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class AutoFlow {
    public static Telemetry dashboardTelemetry;
    public static PropPos propPos = PropPos.NONE;
    final double robotLength = 0.4404;
    final double tile = 0.6;
    final int screenWidth = 640;
    final int screenHeight = 360;
    public DuckLine duckLine;
    public Auto auto;
    Robot robot;
    Location startLocation = new Location(1, robotLength / 2, 180); // PIXEL_STACK
    AprilTagDetector aprilTagDetector;
    MultipleTelemetry telemetry;
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

    void initWebcam() {
        int cameraMonitorViewID = robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());

        WebcamName webcamName = robot.hardwareMap.get(WebcamName.class, "cam");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewID);

        this.duckLine = new DuckLine(this.robot.alliance, telemetry);
        webcam.setPipeline(this.duckLine);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(screenWidth, screenHeight, OpenCvCameraRotation.UPRIGHT);
            }


            @Override
            public void onError(int errorCode) {
                telemetry.addData("camera initialization failed", errorCode);
            }
        });
    }

    void startAprilTagDetection() {
        aprilTagDetector = new AprilTagDetector("cam", new Size(screenWidth, screenHeight), robot.hardwareMap, robot.telemetry, AprilTagDetector.PortalConfiguration.DEFAULT);
    }

    public void init() {
        timer = new ElapsedTime();
        arm = robot.addComponent(Arm.class);
        this.drive = robot.addComponent(DriveClass.class, new DriveClass(DriveClass.ROBOT.GLADOS, startLocation, DriveClass.USE_ENCODERS | DriveClass.USE_BRAKE | DriveClass.USE_DASHBOARD_FIELD, DriveClass.DriveMode.LEFT));
        robot.addComponent(IntakeSystem.class);
        intakeSystem = robot.getComponent(IntakeSystem.class);
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        telemetry = new MultipleTelemetry(dashboardTelemetry, robot.telemetry);
        initWebcam();
        drive.resetOrientation(startLocation.angle);
        dashboardTelemetry.update();
        buildPath();
        intakeSystem.setServoPos(IntakeSystem.ServoPos.Close);
        intakeSystem.start();
//        aprilTagDetector = new AprilTagDetector("cam", new Size(800, 448), robot.hardwareMap, telemetry, new AprilTagDetector.PortalConfiguration());
    }

    public void buildPath() {
        backdropLocation = new Location(-tile * 2, startLocation.y + (tile * Math.signum(-startLocation.y)), startLocation.angle);
        if (robot.alliance == Alliance.RED) {
            backdropLocation = new Location(-tile * 2, startLocation.y - tile, startLocation.angle);
        }
        if (startPos == StartPos.PIXEL_STACK) {
            path = new AutoPath.Builder().addStep(new Location(0, tile * 2 + 0.3, startLocation.angle), lowToleranceSettings).addStep(new Location(0, 0, 90), normalDriveSettings).addStep(new Location(-tile * 3, 0, 90), lowToleranceSettings).addLocation(backdropLocation, normalDriveSettings).addStep(new Location(0, 0, 90), normalDriveSettings).build(drive, startLocation);
        } else {
            path = new AutoPath.Builder().addLocation(new Location(backdropLocation.x, startLocation.y + (tile * 2 * Math.signum(-startLocation.y)), startLocation.angle), normalDriveSettings).addLocation(backdropLocation, normalDriveSettings).addStep(new Location(0, 0, 90), lowToleranceSettings).build(drive, startLocation);
        }
        if (robot.alliance == Alliance.RED && startPos == StartPos.PIXEL_STACK) {
            path.flipY();
        }
    }

    public void lockOnTag(int tagID) {
        AprilTagDetection tag = aprilTagDetector.getSpecificTag(tagID);
        if (tag == null) return;

        while (tag != null && !MathUtil.approximately(tag.rawPose.x, 0.5, 1)) {
            double yPower;
            double gain = 0.016;
            double delta = 0.5 - tag.rawPose.x;
            telemetry.addData("DISTANCE X", tag.rawPose.x);
            telemetry.addData("DISTANCE Y", tag.rawPose.y);
            yPower = delta * -gain;
            if (tag.rawPose.x > 0) {
                yPower *= -1;
            }
            drive.setPowerOriented(yPower, 0, 0, true);
            tag = aprilTagDetector.getSpecificTag(tagID);
//            drive.turnTo(90, MathUtil.map(1, drive.getHeading(), 90, 0, 1));
        }
        drive.setPower(0, 0, 0);
    }

    public void lockOnTagByProp(PropPos pos) {
        lockOnTag(getTagIDAccordingToTeamPropLocation(pos, robot.alliance));
    }

    public int getTagIDAccordingToTeamPropLocation(PropPos pos, Alliance alliance) {
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

    public void placePixelByProp(PropPos pos) {
        double yCord = startLocation.y + (tile * 2 * Math.signum(-startLocation.y));
        Location mid = new Location(startLocation.x, yCord, startLocation.angle), left = new Location(startLocation.x, startLocation.y + tile, startLocation.angle), right = new Location(startLocation.x, startLocation.y + tile * 1.5, startLocation.angle);
        if (propPos == PropPos.MID || propPos == PropPos.NONE)
            drive.goToLocation(mid, normalDriveSettings);
        else if (propPos == PropPos.LEFT) {
            drive.goToLocation(left, normalDriveSettings);
            drive.turnTo(90, 1);
        } else if (propPos == PropPos.RIGHT) {
            drive.goToLocation(right, normalDriveSettings);
            drive.turnTo(-45, 1);
        }
        intakeSystem.spit();
        timer.reset();
        while (timer.seconds() < 0.4) {
            intakeSystem.spinMotor();
        }
        intakeSystem.stopIntake();
        intakeSystem.spinMotor();
    }

    public void run() {
        placePixelByProp(propPos);
        webcam.closeCameraDevice();
        startAprilTagDetection();
        path.run();
        arm.alignToBoard(Arm.Position.One);
        lockOnTagByProp(propPos);
        arm.goToPos(Arm.Position.One);
        robot.sleep(3500);
        arm.dropBottomPixel();
        robot.sleep(100);
        arm.goToPos(Arm.Position.Home);
        robot.sleep(2000);
        park();
        robot.requestOpModeStop();
    }

    public void park() {
        double y = startLocation.y + tile * (parkLocation == ParkLocation.RIGHT ? 2 : 0) * Math.signum(-startLocation.y) + (0.1 * Math.signum(-startLocation.y));
        drive.goToLocation(new Location(-tile * 2, y, 90), normalDriveSettings);
        drive.goToLocation(new Location(-tile * 3 + 0.15, y, 90), normalDriveSettings);
    }

    public void alignToBoard(Arm.Position position) {
        double distance = Math.min(drive.getDistanceLeftSensorDistance(), drive.getDistanceRightSensorDistance());

        while (!MathUtil.approximately(distance, position.distanceFromBackboard, 1)) {
            double delta = position.distanceFromBackboard - distance;
            double gain = 0.016;
            double power = -gain * delta;

            robot.telemetry.addData("Place pixel delta distance", delta);
            robot.telemetry.addData("power", power);
            robot.telemetry.update();
            drive.setPowerOriented(0, power, 0, true);
            distance = Math.min(drive.getDistanceLeftSensorDistance(), drive.getDistanceRightSensorDistance());
//            if (Math.abs(robot.gamepad1.left_stick_x) > 0.1 || Math.abs(robot.gamepad1.left_stick_y) > 0.1 || Math.abs(robot.gamepad2.left_stick_x) > 0.1 || Math.abs(robot.gamepad2.left_stick_y) > 0.1) {
//                break;
//            }
        }
    }

    public double pow(double x) {
        return Math.pow(x, 2) * Math.signum(x);
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
