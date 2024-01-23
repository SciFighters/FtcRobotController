package org.firstinspires.ftc.teamcode.centerstage.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.Arm;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Camera.AprilTagDetector;
import org.firstinspires.ftc.teamcode.centerstage.Systems.DriveClass;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Camera.DuckLine;
import org.firstinspires.ftc.teamcode.centerstage.Systems.IntakeSystem;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Robot;
import org.firstinspires.ftc.teamcode.centerstage.util.Util;
import org.firstinspires.ftc.teamcode.power_play.util.Location;
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
    Telemetry dashboardTelemetry;
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
        A, // LEFT
        B, // MID
        C  // RIGHT
    }

    PropPos propPos = PropPos.A;

    public AutoFlow(Robot robot, Alliance alliance, StartPos startPos, Auto auto) {
        this.alliance = alliance;
        this.robot = robot;
        if (alliance == Alliance.RED) {
            startLocation = new Location(startPos == StartPos.PIXEL_STACK ? 1 : -0.3, robotLength / 2 * 6, 180);
        } else if (alliance == Alliance.BLUE) {
            startLocation = new Location(startPos == StartPos.PIXEL_STACK ? 1 : -0.3, robotLength / 2, 180);
        }
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
        /**
         * TODO: check where the team prop is positioned {@link propLocation}
         *
         */
//        aprilTagDetector = new AprilTagDetector("cam", new Size(800, 448), robot.hardwareMap, telemetry, new AprilTagDetector.PortalConfiguration());
    }

    public void test() {
        //go straight to this location.
        if (propPos == PropPos.C)
            drive.goToLocation(new Location(1, tile * 2.4), 1, 180, 0.05, 0, false);
        else if (propPos == PropPos.A) {
            drive.goToLocation(new Location(0.9, tile * 1.5), 1, 180, 0.05, 0, false);
            drive.turnTo(-90, 1);
        }
        intakeSystem.setStateSpit();
        timer.reset();
        while (timer.seconds() < 1) {
            intakeSystem.spinMotor();
        }
        intakeSystem.setStateIdle();
        intakeSystem.spinMotor();
        drive.goToLocation(new Location(1, tile * 2.4 + 0.3), 1, 180, 0.2, 0, true);
        drive.turnTo(90, 1);
        //place the purple pixel on prop location.
        //turn to the bridge and go to the board.
        //open arm and locate the yellow pixel on the board.
        drive.goToLocation(new Location(1 - tile * 2.5, tile * 2.4 + 0.3), 1, 90, 0.15, 0, true);
        intakeSystem.setServoPos(IntakeSystem.ServoPos.Mid);
        arm.goToPos(Arm.Position.Three);
        drive.goToLocation(new Location(1 - tile * 4 + 0.1, tile * 2 - 0.3), 1, 90, 0.05, 0, false);
        arm.setClawPosition(false);
        robot.sleep(300);
//        cameraPipeline.lockOnTag(CameraPipeline.AprilTags.BlueCenter, 0.3, drive);
    }

    public void teaching() {

    }

}
