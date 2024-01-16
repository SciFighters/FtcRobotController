package org.firstinspires.ftc.teamcode.centerstage.Autonomous;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.Arm;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Camera.AprilTagDetector;
import org.firstinspires.ftc.teamcode.centerstage.Systems.DriveClass;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Camera.DuckLine;
import org.firstinspires.ftc.teamcode.power_play.util.Location;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class AutoFlow {
    private DriveClass drive;
    LinearOpMode opMode = null;
    final double robotLength = 0.4404;
    final double tile = 0.6;
    Location startLocation = new Location(0.9, robotLength / 2, 0); // PIXEL_STACK
    AprilTagDetector aprilTagDetector;
    Alliance alliance;
    public DuckLine duckLine;
    final int screenWidth = 640;
    final int screenHeight = 360;
    int beaconPos = 0;
    MultipleTelemetry telemetry;
    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;
    int propLocation; // 1 => left 2 => middle 3 => right

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

    public AutoFlow(LinearOpMode opMode, Alliance alliance, StartPos startPos, Auto auto) {
        this.alliance = alliance;
        this.opMode = opMode;

    }

    void initWebcam() {
        int cameraMonitorViewID = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

        WebcamName webcamName = opMode.hardwareMap.get(WebcamName.class, "cam");
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
        this.drive = new DriveClass(DriveClass.ROBOT.GLADOS, startLocation, DriveClass.USE_ENCODERS | DriveClass.USE_BRAKE | DriveClass.USE_DASHBOARD_FIELD, DriveClass.DriveMode.LEFT);

        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        telemetry = new MultipleTelemetry(dashboardTelemetry, opMode.telemetry);

        initWebcam();
        /**
         * TODO: check where the team prop is positioned {@link propLocation}
         *
         */
//        aprilTagDetector = new AprilTagDetector("cam", new Size(800, 448), opMode.hardwareMap, telemetry, new AprilTagDetector.PortalConfiguration());
        drive.init();
    }

    public void test() {
        //go straight to this location.
        drive.goToLocation(new Location(0.9, tile * 2 + 0.4), 1, 0, 0.15, 0, true);
        //turn to 180 degrees.
        //place the purple pixel on prop location.
        //turn to the bridge and go to the board.
        //open arm and locate the yellow pixel on the board.
        drive.goToLocation(new Location(0.9 - tile * 2, tile * 2 + 0.4), 1, -90, 0.15, 0, true);
        drive.goToLocation(new Location(0.9 - tile * 4 - tile * 2, tile * 2), 1, -90, 0.05, 0);
        opMode.sleep(300);
//        cameraPipeline.lockOnTag(CameraPipeline.AprilTags.BlueCenter, 0.3, drive);
    }

    public void teaching() {

    }

}
