package org.firstinspires.ftc.teamcode.centerstage.Autonomous;

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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class AutoFlow {
    private DriveClass drive;
    Robot robot = null;
    final double robotLength = 0.4404;
    final double tile = 0.6;
    Location startLocation = new Location(1, robotLength / 2, 180); // PIXEL_STACK
    AprilTagDetector aprilTagDetector;
    Alliance alliance;
    public DuckLine duckLine;
    final int screenWidth = 640;
    final int screenHeight = 360;
    int beaconPos = 0;
    MultipleTelemetry telemetry;
    FtcDashboard dashboard;
    public static Telemetry dashboardTelemetry;
    IntakeSystem intakeSystem;
    Arm arm;
    ElapsedTime timer;

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

    public static PropPos propPos = PropPos.NONE;
    public Auto auto;

    public AutoFlow(Robot robot, Alliance alliance, StartPos startPos, Auto auto) {
        this.alliance = alliance;
        this.robot = robot;
        if (alliance == Alliance.RED) {
            startLocation = new Location(startPos == StartPos.PIXEL_STACK ? 1 : -0.3, robotLength / 2 * 6, 0);
        } else if (alliance == Alliance.BLUE) {
            startLocation = new Location(startPos == StartPos.PIXEL_STACK ? 1 : -0.3, robotLength / 2, 180);
        }
        this.auto = auto;
    }

    void initWebcam() {
        int cameraMonitorViewID = robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());

        WebcamName webcamName = robot.hardwareMap.get(WebcamName.class, "cam");
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewID);

        this.duckLine = new DuckLine(this.alliance, telemetry);
        webcam.setPipeline(this.duckLine);

        // Remove the camera monitor view ID from here
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Use a different camera monitor view ID here or don't use it if you don't need it
                webcam.startStreaming(screenWidth, screenHeight, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("camera initialization failed", errorCode);
            }
        });
        robot.sleep(5000);
        webcam.closeCameraDevice();
    }


    public void init() {
        timer = new ElapsedTime();
        this.drive = robot.addComponent(DriveClass.class, new DriveClass(DriveClass.ROBOT.GLADOS, startLocation, DriveClass.USE_ENCODERS | DriveClass.USE_BRAKE | DriveClass.USE_DASHBOARD_FIELD, DriveClass.DriveMode.LEFT));
        intakeSystem = robot.addComponent(IntakeSystem.class);
        arm = robot.addComponent(Arm.class);
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        telemetry = new MultipleTelemetry(dashboardTelemetry, robot.telemetry);
        intakeSystem.setStateIdle();
        intakeSystem.spinMotor();
        initWebcam();
        dashboardTelemetry.update();
//        aprilTagDetector = new AprilTagDetector("cam", new Size(800, 448), robot.hardwareMap, telemetry, new AprilTagDetector.PortalConfiguration());
    }

    public void pixelStackSideBlue() {
        //go straight to this location.
        if (propPos == PropPos.MID)
            drive.goToLocation(new Location(startLocation.x, tile * 2.4), 1, 180, 0.05, 0, false);
        else if (propPos == PropPos.LEFT) {
            drive.goToLocation(new Location(startLocation.x, tile * 1.5), 1, 180, 0.05, 0, false);
            drive.turnTo(-90, 1);
        } else if (propPos == PropPos.RIGHT) {
            drive.goToLocation(new Location(startLocation.x, tile * 2), 1, 180, 0.05, 0, false);
            drive.turnTo(135, 1);
        }
        intakeSystem.setStateSpit();
        timer.reset();
        while (timer.seconds() < 1) {
            intakeSystem.spinMotor();
        }
        intakeSystem.setStateIdle();
        intakeSystem.spinMotor();
        drive.goToLocation(new Location(startLocation.x, tile * 2.4 + 0.1), 1, 180, 0.2, 0, true);
        drive.turnTo(90, 1);
        drive.goToLocation(new Location(startLocation.x - tile * 2.5, tile * 2.4 + 0.1), 1, 90, 0.15, 0, true);
        intakeSystem.setServoPos(IntakeSystem.ServoPos.Mid);
        arm.goToPos(Arm.Position.Three);
        drive.goToLocation(new Location(startLocation.x - tile * 4 + 0.1, tile * 2 - 0.3), 1, 90, 0.05, 0, false);
        arm.setClawPosition(false);
        robot.sleep(300);
        intakeSystem.setServoPos(IntakeSystem.ServoPos.Mid); // reset arm
        arm.goToPos(200);
        robot.sleep(1000);
        arm.goToPos(0);
        robot.sleep(3000);
        intakeSystem.setServoPos(IntakeSystem.ServoPos.Close);
    }

    public void pixelStackSideRed() {
        //go straight to this location.
        if (propPos == PropPos.MID || propPos == PropPos.NONE)
            drive.goToLocation(new Location(startLocation.x, startLocation.y - tile * 2), 1, 0, 0.05, 0, false);
        else if (propPos == PropPos.LEFT) {
            drive.goToLocation(new Location(startLocation.x, startLocation.y - tile), 1, 0, 0.05, 0, false);
            drive.turnTo(-90, 1);
        } else if (propPos == PropPos.RIGHT) {
            drive.goToLocation(new Location(startLocation.x, startLocation.y - tile * 1.5), 1, 0, 0.05, 0, false);
            drive.turnTo(135, 1);
        }
        intakeSystem.setStateSpit();
        timer.reset();
        while (timer.seconds() < 1) {
            intakeSystem.spinMotor();
        }
        intakeSystem.setStateIdle();
        intakeSystem.spinMotor();
        drive.goToLocation(new Location(startLocation.x, startLocation.y - tile * 2 - 0.3), 1, 90, 0.2, 0, true);
        drive.turnTo(90, 1);
        drive.goToLocation(new Location(startLocation.x - tile * 2.5, startLocation.y - tile * 2 - 0.3), 1, 90, 0.15, 0, true);
        intakeSystem.setServoPos(IntakeSystem.ServoPos.Mid);
        arm.goToPos(Arm.Position.Three);
        drive.goToLocation(new Location(startLocation.x - tile * 4 + 0.1, startLocation.y - tile * 1.5), 1, 90, 0.05, 0, false);
        arm.setClawPosition(false);
        robot.sleep(300);
        intakeSystem.setServoPos(IntakeSystem.ServoPos.Mid); // reset arm
        arm.goToPos(200);
        robot.sleep(1000);
        arm.goToPos(0);
        robot.sleep(3000);
        intakeSystem.setServoPos(IntakeSystem.ServoPos.Close);
    }

    public void backdropSide() {
        if (propPos == PropPos.MID)
            drive.goToLocation(new Location(startLocation.x, tile * 2.4), 1, 180, 0.05, 0, false);
        else if (propPos == PropPos.LEFT) {
            drive.goToLocation(new Location(startLocation.x, tile * 1.5), 1, 180, 0.05, 0, false);
            drive.turnTo(-90, 1);
        } else if (propPos == PropPos.RIGHT) {
            drive.goToLocation(new Location(startLocation.x, tile * 2), 1, 180, 0.05, 0, false);
            drive.turnTo(135, 1);
        }
        intakeSystem.setStateSpit();
        timer.reset();
        while (timer.seconds() < 1) {
            intakeSystem.spinMotor();
        }
        intakeSystem.setStateIdle();
        intakeSystem.spinMotor();
        drive.turnTo(90, 1);
        if (auto != Auto.PARK) {
            intakeSystem.setServoPos(IntakeSystem.ServoPos.Mid);
            arm.goToPos(Arm.Position.Three);
            robot.sleep(300);
            drive.goToLocation(new Location(startLocation.x - tile * 2 + 0.4, tile * 2 - 0.3), 1, 90, 0.05, 0, false);
            arm.setClawPosition(false);
            robot.sleep(300);
            intakeSystem.setServoPos(IntakeSystem.ServoPos.Mid);
            arm.goToPos(100);
            robot.sleep(1000);
            arm.goToPos(0);
            robot.sleep(3000);
            intakeSystem.setServoPos(IntakeSystem.ServoPos.Close);
        } else {
            drive.goToLocation(new Location(startLocation.x - tile * 2 + 0.5, startLocation.y), 1, 90, 0.05, 0, false);
        }
        drive.goToLocation(new Location(startLocation.x - tile * 2 + 0.5, startLocation.y), 1, 90, 0.05, 0, false);
    }
}
