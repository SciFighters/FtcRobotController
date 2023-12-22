package org.firstinspires.ftc.teamcode.centerstage;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.Arm;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Camera.AprilTagDetector;
import org.firstinspires.ftc.teamcode.centerstage.Systems.DriveClass;
import org.firstinspires.ftc.teamcode.centerstage.Systems.GatherSystem;
import org.firstinspires.ftc.teamcode.centerstage.util.Input.Input;
import org.firstinspires.ftc.teamcode.centerstage.util.Input.Toggle;
import org.firstinspires.ftc.teamcode.centerstage.util.Input.UpdateAutomatically;
import org.firstinspires.ftc.teamcode.power_play.util.Location;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
@TeleOp(group = "CENTERSTAGE")
public class GlaDOS extends LinearOpMode {
    FtcDashboard dashboard;
    DriveClass drive = new DriveClass(
            this,
            DriveClass.ROBOT.GLADOS,
            new Location(-0.9, 0.4404 / 2, 180),
            DriveClass.USE_ENCODERS | DriveClass.USE_BRAKE,
            DriveClass.DriveMode.LEFT
    );
    boolean allowMovement = true; // Flag to control movement
    double angle = 0;
    //region Camera
    final double maxTagSize = 239;
    AprilTagDetector aprilTagDetector;
    public static AprilTagDetector.PortalConfiguration portalConfiguration = new AprilTagDetector.PortalConfiguration();
    //endregion
    MultipleTelemetry multipleTelemetry;
    Telemetry dashboardTelemetry;
    @UpdateAutomatically
    Toggle rotateToggle = new Toggle(Input.KeyCode.Gamepad1A); // toggle to check rotation fix
    Arm arm;
    GatherSystem gatherSystem;

    @Override
    public void runOpMode() {
        initSystems();
        waitForStart();
        double targetHeading = drive.getHeading();
        drive.resetOrientation(0);
        while (!isStopRequested() && opModeIsActive()) {
            Input.updateControls();

            handleForceQuit();

            if (gamepad1.start) {
                if (gamepad1.x) {
                    drive.resetOrientation(0);
                    targetHeading = drive.getHeading();
                }
                continue;
            }
            arm.setPower(gamepad2.left_stick_y);

            boolean fieldOriented = !gamepad1.y;
            final double boostK = 0.5;
            double boost = gamepad1.right_trigger * boostK + (1 - boostK) * 2;
            double y = pow(gamepad1.left_stick_y) * boost;
            double x = pow(-gamepad1.left_stick_x) * boost;
            double turn = pow(gamepad1.right_stick_x * boost);
            if (gamepad2.a) {
                arm.goToPos(Arm.Position.One);
            }
            if (gamepad2.b) {
                arm.goToPos(Arm.Position.Two);
            }
            if (gamepad2.y) {
                arm.goToPos(Arm.Position.Three);
            }
            if (gamepad1.left_bumper) {
                gatherSystem.setGatherSystemState(GatherSystem.IntakeWheelsState.Idle);
            } else if (gamepad1.right_bumper) {
                gatherSystem.setGatherSystemState(GatherSystem.IntakeWheelsState.Collect);
            } else if (gamepad1.right_stick_button) {
                gatherSystem.setGatherSystemState(GatherSystem.IntakeWheelsState.Spit);
            }
            gatherSystem.spinMotor();
            if (gamepad1.dpad_left) drive.setPower(0, 0, 0.1);
            else if (gamepad1.dpad_right) drive.setPower(0, 0, -0.1);
            else if (gamepad1.dpad_up) drive.setPower(-0.1, 0, 0);
            else if (gamepad1.dpad_down) drive.setPower(0.1, 0, 0);
            else {
                if (aprilTagDetector.getDetections().size() > 0) { // Some tag is detected
                    drive.setPowerOriented(y, x, turn, fieldOriented);
                    allowMovement = true;
                    AprilTagDetection tag = aprilTagDetector.getSpecificTag(2); // Get the right wall middle tag
                    if (tag == null) {
                        continue;
                    }
                    if (rotateToggle.isClicked()) {
                        if (!drive.busy)
                            aprilTagDetector.lockOnTag(AprilTagDetector.AprilTags.BlueCenter, 0.3, drive);
                    }
//                    cameraPipeline.printTelemetryData(tag);
                    angle = -tag.ftcPose.yaw;
                } else {
                    drive.setPowerOriented(y, x, turn, fieldOriented);
                    allowMovement = true;
                }
                multipleTelemetry.addData("tags detected ", (int) aprilTagDetector.getDetections().size());
                multipleTelemetry.addData("Allow Movement", allowMovement);
                multipleTelemetry.addData("Motor ticks: ", drive.fl.getCurrentPosition());
                multipleTelemetry.addData("Gather state", gatherSystem.getIntakeWheelsState().toString());
                multipleTelemetry.addData("Field Oriented state", fieldOriented);
//                telemetry.addData("Timer", arm.timer.seconds());
//                telemetry.addData("State", arm.stateMachine.getCurrentState().toString());
                multipleTelemetry.update();
            }
        }
    }

    void zeroOnTarget() {
        AprilTagDetection tag = aprilTagDetector.getSpecificTag(2); // Get the right wall middle tag
        drive.turn(-tag.ftcPose.yaw, 0.11);
    }

    void initSystems() {
        gatherSystem = new GatherSystem(this);
        gatherSystem.init(hardwareMap);
        RobotLog.d("Init start");
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        multipleTelemetry = new MultipleTelemetry(dashboardTelemetry, telemetry);
        RobotLog.d("dashboard init");

        arm = new Arm(this, multipleTelemetry);
        telemetry.addData(">", "Init in progress...");
        telemetry.update();
        RobotLog.d("drive init start");
        drive.init(hardwareMap);
        RobotLog.d("cam init start");

        aprilTagDetector = new AprilTagDetector("cam", new Size(800, 448), hardwareMap, multipleTelemetry, portalConfiguration);
        Input.init(this, gamepad1, gamepad2);
        telemetry.addData(">", "Init finished. Press START");
        telemetry.update();
    }


    public double pow(double x) {
        return Math.pow(x, 2) * Math.signum(x);
    }

    void handleForceQuit() {
        if (Input.GetKeyPressed(Input.KeyCode.Gamepad1X) && Input.GetKeyPressed(Input.KeyCode.Gamepad1Start) && Input.GetKeyPressed(Input.KeyCode.Gamepad1Options)) { // Force stops the OpMode
            requestOpModeStop();
        }
    }

}
