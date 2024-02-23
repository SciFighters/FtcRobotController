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
import org.firstinspires.ftc.teamcode.freight_frenzy.util.MathUtil;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class AutoFlow extends Component {
    public static Telemetry dashboardTelemetry;
    public static PropPos propPos = PropPos.NONE;
    public static MultipleTelemetry telemetry;
    final double robotLength = 0.4404;
    final double tile = 0.6;
    final int screenWidth = 640;
    final int screenHeight = 360;
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
    DriveClass.GotoSettings normalDriveSettings = new DriveClass.GotoSettings.Builder()
            .setPower(1).setTolerance(0.05).setSlowdownMode(false).setTimeout(0).build();
    DriveClass.GotoSettings lowToleranceSettings = new DriveClass.GotoSettings.Builder()
            .setPower(1).setTolerance(0.16).setSlowdownMode(true).setTimeout(0).build();
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
        this.drive = robot.addComponent(DriveClass.class, new DriveClass(DriveClass.ROBOT.GLADOS, startLocation, DriveClass.USE_ENCODERS | DriveClass.USE_BRAKE | DriveClass.USE_DASHBOARD_FIELD, DriveClass.DriveMode.LEFT));
        intakeSystem = robot.addComponent(IntakeSystem.class);
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        telemetry = new MultipleTelemetry(dashboardTelemetry, robot.telemetry);
        initWebcam();
        drive.resetOrientation(startLocation.angle);
        dashboardTelemetry.update();
        intakeSystem.setServoPos(IntakeSystem.State.Idle);
        intakeSystem.start();
//        aprilTagDetector = new AprilTagDetector("cam", new Size(800, 448), robot.hardwareMap, telemetry, new AprilTagDetector.PortalConfiguration());
    }

    public void buildPath() {
//        backdropLocation = new Location(-tile * 2 + 0.3, startLocation.y + ((tile + 0.05) * Math.signum(-startLocation.y)), startLocation.angle);
//        backdropLocation = new Location(-tile * 2 + 0.3, -tile - Math.abs(startLocation.y), startLocation.angle);
        backdropLocation = new Location(-tile * 2 + robotLength / 2, -tile - 0.15, startLocation.angle);
        if (startPos == StartPos.PIXEL_STACK) {
            path = new AutoPath.Builder()
                    .addStep(new Location(0, tile * 2 + 0.15, drive.getHeading()),
                            normalDriveSettings)
                    .addStep(new Location(-tile * 2, 0, 90), lowToleranceSettings)
                    .addLocation(new Location(backdropLocation.x, backdropLocation.y, 90), lowToleranceSettings)
                    .addStep(new Location(0, 0, 90), normalDriveSettings).build(robot, startLocation);
        } else {
            if (propPos == PropPos.LEFT || propPos == PropPos.RIGHT) {
                path = new AutoPath.Builder().addStep(new Location(0, 0, 90), normalDriveSettings).addLocation(new Location(backdropLocation.x, backdropLocation.y, 90), normalDriveSettings).build(robot, startLocation);
            } else
                path = new AutoPath.Builder().addStep(new Location(0, tile * 2 + 0.2, 90), normalDriveSettings)
                        .addStep(new Location(-tile, 0, 90), normalDriveSettings)
                        .addLocation(new Location(backdropLocation.x, backdropLocation.y, 90), normalDriveSettings).addStep(new Location(0, 0, 90), lowToleranceSettings)
                        .build(robot, startLocation);
        }
        if (robot.alliance == Alliance.RED) {
            path.flipY();
        }
    }

    public void lockOnTag(int tagID) {
        drive.goToLocation(
                new Location(drive.getPosX(), robot.alliance == Alliance.BLUE ? backdropLocation.y : backdropLocation.flipY().y, 90), normalDriveSettings);
        double yOffset = 0;
        if (tagID == 2 || tagID == 5) {
            return;
        }
        if (tagID == 1 || tagID == 4) {
            yOffset = 0.15;
        } else {
            yOffset = -0.15;
        }
        if (robot.alliance == Alliance.BLUE) {
            yOffset *= -1;
        }
        drive.goToLocation(new Location(drive.getPosX(), drive.getPosY() + (yOffset), 90), normalDriveSettings);
        AprilTagDetection tag = aprilTagDetector.getSpecificTag(tagID);
        if (tag == null) return;

        while (tag != null && !MathUtil.approximately(tag.rawPose.x, 0.02, 1)) {
            double yPower;
            double gain = -0.01;
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
        if (alliance == Alliance.RED && pos == PropPos.LEFT) {
            pos = PropPos.RIGHT;
        } else if (alliance == Alliance.RED && pos == PropPos.RIGHT) {
            pos = PropPos.LEFT;
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

    public void placePurplePixelByProp(PropPos pos) {
        double angle = startLocation.angle;
        double yOffset = tile * 2 * Math.signum(-startLocation.y);
        Location mid = new Location(startLocation.x, startLocation.y + yOffset, startLocation.angle);
        Location left = new Location(startLocation.x - tile, startLocation.y + (tile + 0.13 * Math.signum(-startLocation.y)) * Math.signum(-startLocation.y), startLocation.angle);
        Location right = new Location(startLocation.x - tile + 0.05, startLocation.y + tile * 1.5 * Math.signum(-startLocation.y), startLocation.angle);
        if (robot.alliance == Alliance.RED) {
            right = right.addY(0.1);
            right = right.addX(0.15);
        }
        if (startPos == StartPos.PIXEL_STACK) {
            left = new Location(startLocation.x - 0.1, startLocation.y + (tile * Math.signum(-startLocation.y) + 0.15 * Math.signum(-startLocation.y)), startLocation.angle);
            right = new Location(startLocation.x + 0.2, startLocation.y + tile * 2 * Math.signum(-startLocation.y) - 0.15 * Math.signum(-startLocation.y), startLocation.angle);
            if (robot.alliance == Alliance.RED && startPos == StartPos.PIXEL_STACK) {
                Location temp = left;
                left = right;
                right = temp;
            }
        }
        if (pos == PropPos.MID || pos == PropPos.NONE) {
            drive.goToLocation(mid, normalDriveSettings);
        } else if (pos == PropPos.LEFT || pos == PropPos.RIGHT) {
            Location target = (pos == PropPos.LEFT) ? left : right;
            angle = -90;
            if (startPos == StartPos.PIXEL_STACK && pos == PropPos.LEFT) {
                target = left;
            }
            if (robot.alliance == Alliance.RED && startPos == StartPos.BACKSTAGE && propPos == PropPos.LEFT) {
                target = right;
                angle = -90;
            } else if (robot.alliance == Alliance.RED && startPos == StartPos.BACKSTAGE && propPos == PropPos.RIGHT) {
                angle = 90;
            } else if (robot.alliance == Alliance.BLUE && startPos == StartPos.BACKSTAGE && propPos == PropPos.LEFT) {
                angle = 90;
            } else if (robot.alliance == Alliance.BLUE && startPos == StartPos.PIXEL_STACK && propPos == PropPos.RIGHT) {
                angle = startLocation.angle;
            }
            if (startPos == StartPos.PIXEL_STACK && propPos == PropPos.LEFT && robot.alliance == Alliance.RED) {
                angle = startLocation.angle;
            }
            drive.goToLocation(target, normalDriveSettings);
            if (target == left && startPos == StartPos.BACKSTAGE) {
                drive.goToLocation(new Location(target.addX(0.15).x, target.addX(0.15).y, drive.getHeading()), normalDriveSettings);
            }
        }

        drive.turnTo(angle, 1);
        intakeSystem.spit();
        timer.reset();
        while (timer.seconds() < 0.4) {
            intakeSystem.spinMotor();
        }
        intakeSystem.stopIntake();
        intakeSystem.spinMotor();
        if (propPos == PropPos.MID) {
            drive.goToLocation(new Location(drive.getPosX(), drive.getPosY() + 0.15 * Math.signum(-startLocation.y), drive.getHeading()), normalDriveSettings);
        }
        if (startPos == StartPos.PIXEL_STACK && propPos == PropPos.RIGHT && robot.alliance == Alliance.BLUE) {
            drive.goToLocation(new Location(drive.getPosX(), drive.getPosY() + 0.3, drive.getHeading()), normalDriveSettings);
        }
        //        if (pos == PropPos.LEFT && robot.alliance == Alliance.BLUE) {
//            drive.goToLocation(left.add(new Location(0, tile)), lowToleranceSettings);
//        }
        if ((startPos == StartPos.PIXEL_STACK && propPos == PropPos.LEFT && robot.alliance == Alliance.BLUE) || (startPos == StartPos.PIXEL_STACK && propPos == PropPos.RIGHT && robot.alliance == Alliance.RED)) {
            drive.goToLocation(left.addX(0.13), normalDriveSettings);
        }
        drive.turnTo(90, 0.5);

        if (startPos == StartPos.PIXEL_STACK) {
            robot.sleep(4000);
        }
    }


    public void runPath() {
        closeWebcam();
        placePurplePixelByProp(propPos);
        buildPath();
        startAprilTagDetection();
        if (auto != Auto.PARK) {
            path.run(); // moves to the backboard
            arm.alignToBoard(Arm.Position.One); // aligns to the board
            lockOnTagByProp(propPos); // aligns on the Y axis
            arm.goToPos(Arm.Position.One); // makes the arm go yee
            robot.sleep(3000);
            arm.closeClaw(false);
            robot.sleep(300);
            arm.goToPos(Arm.Position.Home);
//            robot.sleep(2000);
            aprilTagDetector.stop();
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
        double y = startLocation.y + tile * 2 * Math.signum(-startLocation.y) + 0.15 * Math.signum(-startLocation.y);
        double x = -tile * 2 - 0.1;
        if (parkLocation == ParkLocation.RIGHT) {
            y = startLocation.y;
        }
        drive.goToLocation(new Location(drive.getPosX(), y, 90), normalDriveSettings);
        drive.goToLocation(new Location(x, y, 90), normalDriveSettings);
    }

    public void closeWebcam() {
        webcam.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
            @Override
            public void onClose() {
                webcam.closeCameraDevice();

            }
        }); // closes the prop detection webcam
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
