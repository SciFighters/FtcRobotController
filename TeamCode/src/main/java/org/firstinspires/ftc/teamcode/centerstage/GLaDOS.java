package org.firstinspires.ftc.teamcode.centerstage;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.Arm;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Camera.AprilTagDetector;
import org.firstinspires.ftc.teamcode.centerstage.Systems.DriveClass;
import org.firstinspires.ftc.teamcode.centerstage.Systems.IntakeSystem;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Robot;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.RobotTelemetry;
import org.firstinspires.ftc.teamcode.centerstage.util.Input.Input;
import org.firstinspires.ftc.teamcode.centerstage.util.Input.Toggle;
import org.firstinspires.ftc.teamcode.centerstage.util.Input.UpdateAutomatically;
import org.firstinspires.ftc.teamcode.power_play.util.Location;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
@TeleOp(group = "CENTERSTAGE")
public class GLaDOS extends Robot {
    boolean fieldOriented = false;
    FtcDashboard dashboard;
    DriveClass drive;
    boolean allowMovement = true; // Flag to control movement
    double angle = 0;
    //region Camera
    final double maxTagSize = 239;
    AprilTagDetector aprilTagDetector;
    public static AprilTagDetector.PortalConfiguration portalConfiguration = new AprilTagDetector.PortalConfiguration();
    //endregion
    @RobotTelemetry
    MultipleTelemetry multipleTelemetry;
    Telemetry dashboardTelemetry;
    @UpdateAutomatically
    Toggle rotateToggle = new Toggle(Input.KeyCode.Gamepad1A); // toggle to check rotation by april tag
    Toggle turningToggle = new Toggle(); // for rotation fix
    @UpdateAutomatically
    Toggle robotOrientedToggle = new Toggle(Input.KeyCode.Gamepad1PS);
    Arm arm;
    IntakeSystem intakeSystem;
    double targetHeading;
    Thread armThread;
    int turningCount = 0;

    @Override
    public void initRobot() {
        initSystems();
    }

    @Override
    public void startRobot() {
        armThread.start();
        Input.init(this, gamepad1, gamepad2);
        targetHeading = drive.getHeading();
        drive.resetOrientation(180);
    }

    @Override
    public void updateLoop() {
        handleForceQuit();
        if (gamepad1.start) {
            if (gamepad1.x) {
                drive.resetOrientation(180);
                targetHeading = drive.getHeading();
            }
            return;
        }
        //region Arm & Intake
        double armBoost = 0.8;
        if (Input.GetAxis(Input.Axis.Gamepad2RightTrigger) > 0) {
            armBoost = 1.5;
        }
        arm.setMotorsPower(gamepad2.left_stick_y * Math.abs(gamepad2.left_stick_y) * armBoost);

        if (gamepad2.right_bumper) {
            arm.setClawPosition(true);
        }
        if (gamepad2.left_bumper) {
            arm.setClawPosition(false);
        }
        if (gamepad2.b) {
            arm.goToPos(Arm.Position.Two);
        }
        if (gamepad2.y) {
            arm.goToPos(Arm.Position.Three);
        }
        if (Input.GetKeyPressed(Input.KeyCode.Gamepad2DpadUp)) {
            intakeSystem.setStateIdle();
        } else if (Input.GetKeyPressed(Input.KeyCode.Gamepad2DpadDown)) {
            intakeSystem.setStateCollect();
        } else if (Input.GetKeyPressed(Input.KeyCode.Gamepad2RightStickButton)) {
            intakeSystem.setStateSpit();
        }
        if (robotOrientedToggle.isClicked()) {
            this.fieldOriented = !this.fieldOriented;
        }
        //endregion
        //region Driving
        final double boostK = 0.5;
        double boost = gamepad1.right_trigger * boostK + (1 - boostK) * 2;
        double y = pow(-gamepad1.left_stick_y) * boost;
        double x = pow(gamepad1.left_stick_x) * boost;
        double turn = pow(gamepad1.right_stick_x * boost);
        turningToggle.update(Math.abs(turn) > 0.02);
        if (turningToggle.isReleased()) {
            turningCount = 8;
        }
        if (!turningToggle.isPressed()) {
            turningCount--;
        }

        if (turningCount == 0) {
            targetHeading = drive.getHeading();
        }
        if (!turningToggle.isPressed() && turningCount < 0) {
            double delta = drive.getDeltaHeading(targetHeading);
            double gain = 0.02;
            turn = delta * gain;
        }
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
                    return;
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
            //region Telemetry
            multipleTelemetry.addData("tags detected ", (int) aprilTagDetector.getDetections().size());
            multipleTelemetry.addData("Allow Movement", allowMovement);
            multipleTelemetry.addData("Motor ticks: ", drive.fl.getCurrentPosition());
            multipleTelemetry.addData("Gather state", intakeSystem.getState().toString());
            multipleTelemetry.addData("Field Oriented state", fieldOriented);
            multipleTelemetry.addData("Arm State", arm.stateMachine.getCurrentState().toString());
            multipleTelemetry.addData("Arm pos", arm.getPos());
            multipleTelemetry.addData("Arm manual power", arm.getManualPower());
            multipleTelemetry.addData("Arm lift1 power", arm.lift1.getPower());
            multipleTelemetry.addData("Arm target pos", arm.getTargetPos());
            multipleTelemetry.addData("Field oriented", fieldOriented);
            multipleTelemetry.update();
            //endregion
        }
        //endregion Driving
    }

    void zeroOnTarget() {
        AprilTagDetection tag = aprilTagDetector.getSpecificTag(2); // Get the right wall middle tag
        drive.turn(-tag.ftcPose.yaw, 0.11);
    }

    void initSystems() {
        telemetryInit();
        intakeSystem = addComponent(IntakeSystem.class);
        drive = addComponent(DriveClass.class,
                new DriveClass(DriveClass.ROBOT.GLADOS, new Location(-0.9, 0.4404 / 2, 180), DriveClass.USE_ENCODERS | DriveClass.USE_BRAKE, DriveClass.DriveMode.LEFT));
        arm = addComponent(Arm.class);
        armThread = new Thread(arm);

        telemetry.addData(">", "Init in progress...");
        telemetry.update();
        RobotLog.d("drive init start");
        RobotLog.d("cam init start");

        aprilTagDetector = new AprilTagDetector("cam", new Size(800, 448), hardwareMap, multipleTelemetry, portalConfiguration);
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

    void telemetryInit() {
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        multipleTelemetry = new MultipleTelemetry(dashboardTelemetry, telemetry);
    }

}
