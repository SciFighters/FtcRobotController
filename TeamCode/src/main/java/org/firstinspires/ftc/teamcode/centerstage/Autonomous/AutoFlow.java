package org.firstinspires.ftc.teamcode.centerstage.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Camera.CameraPipeline;
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
    CameraPipeline cameraPipeline;
    Alliance alliance;
    public DuckLine duckLine;
    final int screenWidth = 640;
    final int screenHeight = 360;
    int beaconPos = 0;

    public enum StartPos {
        PIXEL_STACK(1),
        BACKSTAGE(-1);

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
        this.drive = new DriveClass
                (
                        opMode, DriveClass.ROBOT.CONSTANTIN,
                        startLocation,
                        DriveClass.USE_ENCODERS | DriveClass.USE_BRAKE | DriveClass.USE_DASHBOARD_FIELD,
                        DriveClass.DriveMode.LEFT
                );
    }

    void initWebcam() {
        int cameraMonitorViewID = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

        WebcamName webcamName = opMode.hardwareMap.get(WebcamName.class, "cam");
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewID);

        this.duckLine = new DuckLine(this.alliance);
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
                opMode.telemetry.addData("camera initialization failed", errorCode);
            }
        });
    }


    public void init() {
        initWebcam();

//        cameraPipeline = new CameraPipeline("cam", new Size(800, 448), opMode.hardwareMap, opMode.telemetry, new CameraPipeline.PortalConfiguration());
        drive.init(opMode.hardwareMap);
    }

    public void test() {
        drive.goToLocation(new Location(0.9, tile * 3 + 0.4), 1, 0, 0.15, 0, true);
        drive.goToLocation(new Location(0.9 - tile * 3, drive.getPosY()), 1, -90, 0.15, 0, true);
        drive.goToLocation(new Location(drive.getPosX() - tile * 2, tile * 2), 1, -90, 0.05, 0);
        opMode.sleep(300);
//        cameraPipeline.lockOnTag(CameraPipeline.AprilTags.BlueCenter, 0.3, drive);
    }

}
