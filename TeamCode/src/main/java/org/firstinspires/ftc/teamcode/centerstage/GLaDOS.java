package org.firstinspires.ftc.teamcode.centerstage;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

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

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(group = "CENTERSTAGE")
public class GLaDOS extends Robot {
    // Robot components
    DriveClass drive;
    Arm arm;
    IntakeSystem intakeSystem;
    AprilTagDetector aprilTagDetector;

    // Telemetry and dashboard
    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;
    @RobotTelemetry
    MultipleTelemetry multipleTelemetry;

    // Control variables
    boolean fieldOriented = true;
    boolean allowMovement = true;
    double angle = 0;
    double targetHeading;
    int turningCount = 0;
    Gamepad.RumbleEffect rumbleEffect;

    // Input toggles
    @UpdateAutomatically
    Toggle rotateToggle = new Toggle(Input.KeyCode.Gamepad1A);
    Toggle turningToggle = new Toggle();
    @UpdateAutomatically
    Toggle robotOrientedToggle = new Toggle(Input.KeyCode.Gamepad1PS);

    double armBoost;

    @Override
    public void initRobot() {
        // Initialize robot components
        initSystems();
        rumbleEffect = new Gamepad.RumbleEffect.Builder().addStep(1.0, 1.0, 300).build();
    }

    @Override
    public void startRobot() {
        // Initialize input and start subsystems
        Input.init(this, gamepad1, gamepad2);
        targetHeading = drive.getHeading();
        drive.resetOrientation(180);
        intakeSystem.start();
    }

    @Override
    public void updateLoop() {
        // Handle force quit command
        handleForceQuit();

        // Check for reorientation command
        if (gamepad1.start && gamepad1.x) {
            drive.resetOrientation(180);
            targetHeading = drive.getHeading();
            gamepad1.runRumbleEffect(rumbleEffect);
            return;
        }

        // Update Arm and Intake subsystems
        updateArmAndIntake();

        // Update driving controls
        updateDriving();
    }

    // Method to update Arm and Intake subsystems
    private void updateArmAndIntake() {
        // Arm control based on gamepad input
        armBoost = (gamepad1.left_trigger > 0) ? 1.5 : 0.8;
        arm.setMotorsPower(pow(-gamepad2.left_stick_y) * armBoost);

        // Claw control based on gamepad input
        if (gamepad2.right_bumper) {
            arm.setClawPosition(true);
            if (!gamepad2.isRumbling()) {
                gamepad2.runRumbleEffect(rumbleEffect);
            }
        } else if (gamepad2.left_bumper) {
            arm.setClawPosition(false);
        }

        // Intake system control based on gamepad input
        if (Input.GetKeyPressed(Input.KeyCode.Gamepad2DpadUp)) {
            intakeSystem.setStateIdle();
        } else if (Input.GetKeyPressed(Input.KeyCode.Gamepad2DpadDown)) {
            intakeSystem.setStateCollect();
        } else if (Input.GetKeyPressed(Input.KeyCode.Gamepad2RightStickButton)) {
            intakeSystem.setStateSpit();
        }

        // Toggle field-oriented or robot-oriented control
        if (robotOrientedToggle.isClicked()) {
            this.fieldOriented = !this.fieldOriented;
        }
    }

    // Method to update driving controls
    private void updateDriving() {
        // Boost factor for driving speed
        double boost = (gamepad1.left_bumper) ? 1.5 : 0.7;

        // Drive control based on gamepad input
        double y = pow(-gamepad1.left_stick_y) * boost;
        double x = pow(gamepad1.left_stick_x) * boost;
        double turn = pow(gamepad1.right_stick_x * boost);

        // Update turning toggle state
        turningToggle.update(Math.abs(turn) > 0.02);

        // Reset turning count if toggle is released
        if (turningToggle.isReleased()) {
            turningCount = 8;
        }

        // Decrease turning count if toggle is not pressed
        if (!turningToggle.isPressed()) {
            turningCount--;
        }

        // Set target heading if turning count is zero
        if (turningCount == 0) {
            targetHeading = drive.getHeading();
        }

        // Apply rotation fix if turning count is less than zero
        if (!turningToggle.isPressed() && turningCount < 0) {
            double delta = drive.getDeltaHeading(targetHeading);
            double gain = 0.02;
            turn = delta * gain;
        }

        // Perform dpad-specific actions
        if (gamepad1.dpad_left) drive.setPower(0, 0, 0.1);
        else if (gamepad1.dpad_right) drive.setPower(0, 0, -0.1);
        else if (gamepad1.dpad_up) drive.setPower(-0.1, 0, 0);
        else if (gamepad1.dpad_down) drive.setPower(0.1, 0, 0);
        else {
            // Drive based on AprilTag detection
            if (aprilTagDetector.getDetections().size() > 0) {
                drive.setPowerOriented(y, x, turn, fieldOriented);
                allowMovement = true;

                // Get AprilTag details
                AprilTagDetection tag = aprilTagDetector.getSpecificTag(2);

                // Return if tag is null
                if (tag == null) {
                    return;
                }

                // Rotate toward the detected tag if toggle is clicked
                if (rotateToggle.isClicked() && !drive.busy) {
                    aprilTagDetector.lockOnTag(AprilTagDetector.AprilTags.BlueCenter, 0.3, drive);
                }

                angle = -tag.ftcPose.yaw;
            } else {
                // Drive without AprilTag detection
                drive.setPowerOriented(y, x, turn, fieldOriented);
                allowMovement = true;
            }

            // Update telemetry
            updateTelemetry();
        }
    }

    // Method to zero on target based on AprilTag detection
    private void zeroOnTarget() {
        AprilTagDetection tag = aprilTagDetector.getSpecificTag(2);
        drive.turn(-tag.ftcPose.yaw, 0.11);
    }

    // Method to initialize robot subsystems
    private void initSystems() {
        // Initialize telemetry
        telemetryInit();

        // Add robot components
        intakeSystem = addComponent(IntakeSystem.class);
        drive = addComponent(DriveClass.class,
                new DriveClass(DriveClass.ROBOT.GLADOS, new Location(-0.9, 0.4404 / 2, 180),
                        DriveClass.USE_ENCODERS | DriveClass.USE_BRAKE, DriveClass.DriveMode.LEFT));
        arm = addComponent(Arm.class);

        // Display initialization progress
        telemetry.addData(">", "Init in progress...");
        telemetry.update();

        // Set intake system state and start motors
        intakeSystem.setStateIdle();
        intakeSystem.spinMotor();

        // Initialize AprilTag detector
        aprilTagDetector = new AprilTagDetector("cam", new Size(800, 448), hardwareMap, multipleTelemetry,
                AprilTagDetector.PortalConfiguration.DEFAULT);

        // Display initialization completion message
        telemetry.addData(">", "Init finished. Press START");
        telemetry.update();
    }

    // Method to apply power function
    public double pow(double x) {
        return Math.pow(x, 2) * Math.signum(x);
    }

    // Method to handle force quit command
    private void handleForceQuit() {
        if (Input.GetKeyPressed(Input.KeyCode.Gamepad1Y) &&
                Input.GetKeyPressed(Input.KeyCode.Gamepad1Start) &&
                Input.GetKeyPressed(Input.KeyCode.Gamepad1Options)) {
            requestOpModeStop();
        }
    }

    // Method to initialize telemetry
    private void telemetryInit() {
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        multipleTelemetry = new MultipleTelemetry(dashboardTelemetry, telemetry);
    }

    // Method to update telemetry data
    private void updateTelemetry() {
        // Display relevant telemetry data
        multipleTelemetry.addData("tags detected ", aprilTagDetector.getDetections().size());
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
        multipleTelemetry.addData("Arm controlled power", pow(-gamepad2.left_stick_y) * armBoost);
        multipleTelemetry.addData("Arm distance", arm.getDistanceSensorDistance());
        multipleTelemetry.update();
    }
}
