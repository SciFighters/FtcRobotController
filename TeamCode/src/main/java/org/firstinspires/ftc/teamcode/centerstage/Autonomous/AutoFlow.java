package org.firstinspires.ftc.teamcode.centerstage.Autonomous;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.Arm;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Camera.AprilTagDetector;
import org.firstinspires.ftc.teamcode.centerstage.Systems.DriveClass;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Camera.DuckLine;
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
    Robot robot = null;
    Location startLocation = new Location(1, robotLength / 2, 180); // PIXEL_STACK
    AprilTagDetector aprilTagDetector;
    Alliance alliance;
    MultipleTelemetry telemetry;
    FtcDashboard dashboard;
    IntakeSystem intakeSystem;
    Arm arm;
    ElapsedTime timer;
    OpenCvWebcam webcam;
    DriveClass.GotoSettings normalDriveSettings = new DriveClass.GotoSettings.Builder().setPower(1).setTolerance(0.05).setSlowdownMode(false).setTimeout(0).build();
    DriveClass.GotoSettings lowToleranceSettings = new DriveClass.GotoSettings.Builder().setPower(1).setTolerance(0.16).setSlowdownMode(true).setTimeout(0).build();
    AutoPath path;
    private DriveClass drive;

    public AutoFlow(Robot robot, Alliance alliance, StartPos startPos, Auto auto) {
        this.alliance = alliance;
        this.robot = robot;
        if (alliance == Alliance.RED) {
            startLocation = new Location(startPos == StartPos.PIXEL_STACK ? 1 : -0.3, (robotLength / 2) + (tile * 2), 0);
        } else if (alliance == Alliance.BLUE) {
            startLocation = new Location(startPos == StartPos.PIXEL_STACK ? 1 : -0.3, (-robotLength / 2) - (tile * 2), 180);
        }
        this.auto = auto;

    }

    void initWebcam() {
        int cameraMonitorViewID = robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());

        WebcamName webcamName = robot.hardwareMap.get(WebcamName.class, "cam");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewID);

        this.duckLine = new DuckLine(this.alliance, telemetry);
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
        intakeSystem.stopIntake();
        intakeSystem.spinMotor();
        initWebcam();
        dashboardTelemetry.update();

        path = new AutoPath.Builder()
                .addStep(
                        new Location(0, tile * 2 + 0.3, startLocation.angle), lowToleranceSettings)
                .addStep(new Location(0, 0, 90), normalDriveSettings)
                .addStep(new Location(-tile * 3, 0, 90), lowToleranceSettings)
                .addStep(new Location(0, -tile - 0.3, 90), normalDriveSettings)
                .build(drive, startLocation);
        if (alliance == Alliance.RED) {
            path.flipY();
        }
//        aprilTagDetector = new AprilTagDetector("cam", new Size(800, 448), robot.hardwareMap, telemetry, new AprilTagDetector.PortalConfiguration());
    }

    public void pixelStackSideBlue() {
        intakeSystem.start();
        arm.start();
        drive.goToLocation(new Location(startLocation.x, startLocation.y + tile * 2.5, 180), lowToleranceSettings);
        if (propPos == PropPos.MID || propPos == PropPos.NONE)
            drive.goToLocation(new Location(startLocation.x, tile * 2.4, 180), normalDriveSettings);
        else if (propPos == PropPos.LEFT) {
            drive.goToLocation(new Location(startLocation.x, tile * 1.5, 180), normalDriveSettings);
            drive.turnTo(-90, 1);
        } else if (propPos == PropPos.RIGHT) {
            drive.goToLocation(new Location(startLocation.x, tile * 2, 180), normalDriveSettings);
            drive.turnTo(135, 1);
        }
        webcam.closeCameraDevice();
        intakeSystem.spit();
        timer.reset();
        while (timer.seconds() < 1) {
            intakeSystem.spinMotor();
        }
        intakeSystem.stopIntake();
        intakeSystem.spinMotor();
        startAprilTagDetection();
        drive.goToLocation(new Location(startLocation.x, tile * 2.4 + 0.1, 180), lowToleranceSettings);
        drive.turnTo(90, 1);
        drive.goToLocation(new Location(startLocation.x - tile * 2.5, tile * 2.4 + 0.1, 90), lowToleranceSettings);
        intakeSystem.setServoPos(IntakeSystem.ServoPos.Mid);
        arm.goToPos(Arm.Position.Three);
        drive.goToLocation(new Location(startLocation.x - tile * 4 + 0.1, tile * 2 - 0.3), 1, 90, 0.05, 0, false);
        arm.dropAndReturn();
    }

    public void pixelStackSideRed() {
        //go straight to this locationDelta.
        if (propPos == PropPos.MID || propPos == PropPos.NONE)
            drive.goToLocation(new Location(startLocation.x, startLocation.y - tile * 2), 1, 0, 0.05, 0, false);
        else if (propPos == PropPos.LEFT) {
            drive.goToLocation(new Location(startLocation.x, startLocation.y - tile), 1, 0, 0.05, 0, false);
            drive.turnTo(90, 1);
        } else if (propPos == PropPos.RIGHT) {
            drive.goToLocation(new Location(startLocation.x, startLocation.y - tile * 1.5), 1, 0, 0.05, 0, false);
            drive.turnTo(-45, 1);
        }
        webcam.closeCameraDevice();
        intakeSystem.spit();
        timer.reset();
        while (timer.seconds() < 1) {
            intakeSystem.spinMotor();
        }
        intakeSystem.stopIntake();
        intakeSystem.spinMotor();
        startAprilTagDetection();
        drive.goToLocation(new Location(startLocation.x, startLocation.y - tile * 2 - 0.3), 1, 90, 0.2, 0, true);
        drive.turnTo(90, 1);
        drive.goToLocation(new Location(startLocation.x - tile * 2.5, startLocation.y - tile * 2 - 0.3), 1, 90, 0.15, 0, true);
        intakeSystem.setServoPos(IntakeSystem.ServoPos.Mid);
        arm.goToPos(Arm.Position.Three);
        drive.goToLocation(new Location(startLocation.x - tile * 4 - 0.07, startLocation.y - tile * 1.5), 1, 90, 0.05, 0, false);
        arm.dropAndReturn();
        drive.goToLocation(new Location(startLocation.x - tile * 4 - 0.07, startLocation.y), 1, 90, 0.05, 0, false);
    }

    public void backdropSideBlue() {
        if (propPos == PropPos.MID || propPos == PropPos.NONE)
            drive.goToLocation(new Location(startLocation.x, tile * 2.4), 1, 180, 0.05, 0, false);
        else if (propPos == PropPos.LEFT) {
            drive.goToLocation(new Location(startLocation.x, tile * 1.5), 1, 180, 0.05, 0, false);
            drive.turnTo(-90, 1);
        } else if (propPos == PropPos.RIGHT) {
            drive.goToLocation(new Location(startLocation.x, tile * 2), 1, 180, 0.05, 0, false);
            drive.turnTo(135, 1);
        }
        webcam.closeCameraDevice();
        intakeSystem.spit();
        timer.reset();
        while (timer.seconds() < 1) {
            intakeSystem.spinMotor();
        }
        intakeSystem.stopIntake();
        intakeSystem.spinMotor();
        startAprilTagDetection();
        drive.turnTo(90, 1);
        if (auto != Auto.PARK) {
            arm.goToPos(Arm.Position.Three);
            robot.sleep(300);
            drive.goToLocation(new Location(startLocation.x - tile * 2 + 0.4, tile * 2 - 0.3), 1, 90, 0.05, 0, false);
            arm.dropAndReturn();
        }
        drive.goToLocation(new Location(startLocation.x - tile * 2 + 0.4, startLocation.y), 1, 90, 0.05, 0, false);
    }

    public void backdropSideRed() {
        if (propPos == PropPos.MID || propPos == PropPos.NONE)
            drive.goToLocation(new Location(startLocation.x, startLocation.y - tile * 2), 1, 0, 0.05, 0, false);
        else if (propPos == PropPos.LEFT) {
            drive.goToLocation(new Location(startLocation.x, startLocation.y - tile), 1, 0, 0.05, 0, false);
            drive.turnTo(90, 1);
        } else if (propPos == PropPos.RIGHT) {
            drive.goToLocation(new Location(startLocation.x, startLocation.y - tile * 1.5), 1, 0, 0.05, 0, false);
            drive.turnTo(-45, 1);
        }
        webcam.closeCameraDevice();
        intakeSystem.spit();
        timer.reset();
        while (timer.seconds() < 1) {
            intakeSystem.spinMotor();
        }
        intakeSystem.stopIntake();
        intakeSystem.spinMotor();
        startAprilTagDetection();
        drive.goToLocation(new Location(startLocation.x, startLocation.y - tile * 2.5), 1, 0.05, 0, true);
        drive.turnTo(90, 1);
        if (auto != Auto.PARK) {
            intakeSystem.setServoPos(IntakeSystem.ServoPos.Mid);
            arm.goToPos(Arm.Position.Three);
            robot.sleep(300);
            drive.goToLocation(new Location(startLocation.x - tile * 2 + 0.05, startLocation.y - tile * 1), 1, 90, 0.05, 0, false);
            lockOnTagByProp(propPos);
            arm.dropAndReturn();
            robot.sleep(1000);
        }
        drive.goToLocation(new Location(startLocation.x - tile * 2 + 0.3, startLocation.y), 1, 90, 0.15, 0, true);
        drive.goToLocation(new Location(startLocation.x - tile * 2.3, startLocation.y), 1, 90, 0.05, 0, true);
    }

    public void lockOnTag(int tagID) {
        AprilTagDetection tag = aprilTagDetector.getSpecificTag(tagID);
        if (tag == null) return;
        while (tag != null && Math.abs(tag.rawPose.x) > 0.05) {
            double strafe = 0;
            strafe = MathUtil.map(1, 0, Math.abs(tag.rawPose.x), 0, 1) * 2;
            if (tag.rawPose.x > 0) {
                strafe *= -1;
            }
            drive.setPower(0, 0, strafe);
            tag = aprilTagDetector.getSpecificTag(tagID);
            drive.turnTo(90, MathUtil.map(1, drive.getHeading(), 90, 0, 1));
        }
        drive.setPower(0, 0, 0);
    }

    public void lockOnTagByProp(PropPos pos) {
        lockOnTag(getTagIDAccordingToTeamPropLocation(pos, alliance));
    }

    public int getTagIDAccordingToTeamPropLocation(PropPos pos, Alliance alliance) {
        int id;
        switch (pos) {
            case LEFT:
                id = 1;
            case MID:
                id = 2;
            case RIGHT:
                id = 3;
            default:
                id = 2;
        }
        if (alliance == Alliance.RED) {
            id += 3;
        }
        return id;
    }

    public void placePixelByProp(PropPos pos) {
        Location mid = new Location(startLocation.x, startLocation.y + tile * 2, startLocation.angle), left = new Location(startLocation.x, startLocation.y + tile, startLocation.angle), right = new Location(startLocation.x, startLocation.y + tile * 1.5, startLocation.angle);
        if (propPos == PropPos.MID || propPos == PropPos.NONE)
            drive.goToLocation(mid, normalDriveSettings);
        else if (propPos == PropPos.LEFT) {
            drive.goToLocation(left, normalDriveSettings);
            drive.turnTo(90, 1);
        } else if (propPos == PropPos.RIGHT) {
            drive.goToLocation(right, normalDriveSettings);
            drive.turnTo(-45, 1);
        }
    }

    public void test() {
        path.run();
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
}
